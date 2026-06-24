from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    car2_domain_bridge = Node(
        package="mycar_base_sim",
        executable="domain_topic_bridge.py",
        name="car2_domain_bridge",
        output="screen"
    )

    return LaunchDescription([
        car2_domain_bridge
    ])
