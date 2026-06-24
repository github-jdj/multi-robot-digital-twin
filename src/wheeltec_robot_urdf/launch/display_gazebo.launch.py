import os
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def strip_gazebo_ros2_control(robot_description):
    root = ET.fromstring(robot_description)

    for child in list(root):
        if child.tag == "ros2_control":
            root.remove(child)
            continue

        if child.tag != "gazebo":
            continue

        for plugin in list(child.findall("plugin")):
            if plugin.get("name") == "gazebo_ros2_control":
                child.remove(plugin)

        if not child.attrib and len(list(child)) == 0 and (child.text or "").strip() == "":
            root.remove(child)

    return ET.tostring(root, encoding="unicode")


def prefix_robot_description_frames(robot_description, prefix):
    root = ET.fromstring(robot_description)
    root.set("name", f"{prefix}{root.get('name', 'robot')}")

    def prefixed(name):
        if not name or name.startswith(prefix):
            return name
        return f"{prefix}{name}"

    for link in root.findall(".//link"):
        link.set("name", prefixed(link.get("name")))

    for joint in root.findall(".//joint"):
        joint.set("name", prefixed(joint.get("name")))

        parent = joint.find("parent")
        if parent is not None:
            parent.set("link", prefixed(parent.get("link")))

        child = joint.find("child")
        if child is not None:
            child.set("link", prefixed(child.get("link")))

    for gazebo in root.findall(".//gazebo"):
        reference = gazebo.get("reference")
        if reference:
            gazebo.set("reference", prefixed(reference))

    return ET.tostring(root, encoding="unicode")


def add_gazebo_ros_state_plugin(world_file):
    tree = ET.parse(world_file)
    root = tree.getroot()
    world = root.find("world")

    if world is None:
        raise RuntimeError("SDF world file does not contain <world> tag")

    if world.find("./plugin[@name='gazebo_ros_state']") is None:
        plugin = ET.Element(
            "plugin",
            {
                "name": "gazebo_ros_state",
                "filename": "libgazebo_ros_state.so"
            }
        )
        update_rate = ET.SubElement(plugin, "update_rate")
        update_rate.text = "30.0"
        world.insert(0, plugin)

    patched_world_file = "/tmp/wheeltec_robot_urdf_my_map_with_state.world"
    tree.write(patched_world_file, encoding="unicode", xml_declaration=True)
    return patched_world_file


def generate_launch_description():
    package_name = "wheeltec_robot_urdf"

    pkg_share = get_package_share_directory(package_name)
    gazebo_pkg_share = "/tmp/wheeltec_robot_urdf_share"

    if os.path.islink(gazebo_pkg_share) and os.readlink(gazebo_pkg_share) != pkg_share:
        os.unlink(gazebo_pkg_share)
    if not os.path.exists(gazebo_pkg_share):
        os.symlink(pkg_share, gazebo_pkg_share)

    source_world_file = os.path.join(
        pkg_share,
        "world",
        "my_map.world"
    )
    world_file = add_gazebo_ros_state_plugin(source_world_file)

    urdf_file = os.path.join(
        pkg_share,
        "urdf",
        "V550_akm_robot.urdf"
    )

    controller_yaml = os.path.join(
        gazebo_pkg_share,
        "config",
        "ros2_controllers.yaml"
    )

    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    )

    with open(urdf_file, "r", encoding="utf-8") as file:
        robot_description = file.read()

    robot_description = robot_description.replace(
        '<?xml version="1.0" encoding="utf-8"?>',
        ""
    ).strip()

    robot_start = robot_description.find("<robot")
    if robot_start == -1:
        raise RuntimeError("URDF file does not contain <robot> tag")

    robot_description = robot_description[robot_start:].strip()

    controlled_robot_description = robot_description.replace(
        "package://wheeltec_robot_urdf",
        Path(gazebo_pkg_share).as_uri()
    )

    controlled_robot_description = controlled_robot_description.replace(
        "REPLACE_WITH_CONTROLLER_YAML",
        controller_yaml
    )

    follower_robot_description = prefix_robot_description_frames(
        strip_gazebo_ros2_control(controlled_robot_description),
        "robot2_"
    )

    disable_gazebo_model_database = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI",
        value=""
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
            "pause": "false"
        }.items()
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sim_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": controlled_robot_description,
                "use_sim_time": True
            }
        ]
    )

    follower_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot2",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": follower_robot_description,
                "use_sim_time": True
            }
        ]
    )

    car2_real_robot_description_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="car2",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": controlled_robot_description,
                "use_sim_time": False
            }
        ],
        remappings=[
            ("/tf", "/unused_car2_description_tf"),
            ("/tf_static", "/unused_car2_description_tf_static")
        ]
    )

    follower_joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="robot2",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": follower_robot_description,
                "use_sim_time": True
            }
        ]
    )

    map_to_base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_base_footprint_tf",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "map",
            "base_footprint"
        ]
    )

    base_footprint_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_to_base_link_tf",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "base_footprint",
            "base_link"
        ]
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_v550_robot",
        output="screen",
        arguments=[
            "-entity", "V550_akm_robot",
            "-topic", "robot_description",
            "-x", "-1.12",
            "-y", "-1.19",
            "-z", "0.08",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "-3.1"
        ]
    )

    delayed_spawn_robot_node = TimerAction(
        period=3.0,
        actions=[
            spawn_robot_node
        ]
    )

    spawn_follower_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_v550_robot_2",
        output="screen",
        arguments=[
            "-entity", "V550_akm_robot_2",
            "-topic", "/robot2/robot_description",
            "-x", "-1.12",
            "-y", "-2.04",
            "-z", "0.08",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "-3.1"
        ]
    )

    delayed_spawn_follower_robot_node = TimerAction(
        period=4.0,
        actions=[
            spawn_follower_robot_node
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30"
        ]
    )

    rear_wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="rear_wheel_velocity_controller_spawner",
        output="screen",
        arguments=[
            "rear_wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30"
        ]
    )

    steering_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="steering_position_controller_spawner",
        output="screen",
        arguments=[
            "steering_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30"
        ]
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=7.0,
        actions=[
            joint_state_broadcaster_spawner
        ]
    )

    delayed_rear_wheel_velocity_controller = TimerAction(
        period=10.0,
        actions=[
            rear_wheel_velocity_controller_spawner
        ]
    )

    delayed_steering_position_controller = TimerAction(
        period=13.0,
        actions=[
            steering_position_controller_spawner
        ]
    )

    return LaunchDescription([
        disable_gazebo_model_database,
        gazebo_node,
        map_to_base_footprint_tf,
        base_footprint_to_base_link_tf,
        robot_state_publisher_node,
        follower_robot_state_publisher_node,
        car2_real_robot_description_node,
        follower_joint_state_publisher_node,
        delayed_spawn_robot_node,
        delayed_spawn_follower_robot_node,
        delayed_joint_state_broadcaster,
        delayed_rear_wheel_velocity_controller,
        delayed_steering_position_controller
    ])
