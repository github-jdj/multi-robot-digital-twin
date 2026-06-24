import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    domain_id = LaunchConfiguration("domain_id")

    wheeltec_share = get_package_share_directory("wheeltec_robot_urdf")
    mycar_base_share = get_package_share_directory("mycar_base_sim")

    gazebo_launch_file = os.path.join(
        wheeltec_share,
        "launch",
        "display_gazebo.launch.py"
    )

    base_sim_launch_file = os.path.join(
        mycar_base_share,
        "app",
        "base_sim_twin.launch.py"
    )

    robot2_base_sim_launch_file = os.path.join(
        mycar_base_share,
        "app",
        "base_sim_twin_robot2.launch.py"
    )

    rviz_car1_config_file = os.path.join(
        wheeltec_share,
        "rviz",
        "car1.rviz"
    )

    rviz_car2_config_file = os.path.join(
        wheeltec_share,
        "rviz",
        "car2.rviz"
    )

    declare_domain_id = DeclareLaunchArgument(
        "domain_id",
        default_value="1",
        description="ROS_DOMAIN_ID used by Gazebo, digital twin base, and RViz."
    )

    set_domain_id = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=domain_id
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    rviz_car1 = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_car1",
                output="screen",
                arguments=[
                    "-d",
                    rviz_car1_config_file
                ]
            )
        ]
    )

    rviz_car2 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_car2",
                output="screen",
                remappings=[
                    ("/tf", "/car2/tf"),
                    ("/tf_static", "/car2/tf_static")
                ],
                arguments=[
                    "-d",
                    rviz_car2_config_file
                ]
            )
        ]
    )

    base_sim = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_sim_launch_file)
            )
        ]
    )

    robot2_base_sim = TimerAction(
        period=8.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot2_base_sim_launch_file)
            )
        ]
    )

    return LaunchDescription([
        declare_domain_id,
        set_domain_id,
        gazebo,
        rviz_car1,
        rviz_car2,
        base_sim,
        robot2_base_sim
    ])
