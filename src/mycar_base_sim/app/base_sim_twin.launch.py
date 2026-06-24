import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "mycar_base_sim"
    pkg_share = get_package_share_directory(package_name)

    twin_config = os.path.join(pkg_share, "config", "sim_car_tf_twin.yaml")

    sim_car_tf_twin_node = Node(
        package=package_name,
        executable="sim_car_tf_twin_node",
        name="sim_car_tf_twin",
        output="screen",
        parameters=[twin_config]
    )

    return LaunchDescription([
        sim_car_tf_twin_node
    ])
