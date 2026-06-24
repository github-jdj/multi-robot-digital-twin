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

    declare_domain_id = DeclareLaunchArgument(
        "domain_id",
        default_value="1",
        description="ROS_DOMAIN_ID used by Gazebo and digital twin base."
    )

    set_domain_id = SetEnvironmentVariable(
        name="ROS_DOMAIN_ID",
        value=domain_id
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    base_sim = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_sim_launch_file)
            )
        ]
    )

    return LaunchDescription([
        declare_domain_id,
        set_domain_id,
        gazebo,
        base_sim
    ])
