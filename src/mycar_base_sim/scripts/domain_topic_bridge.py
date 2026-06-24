#!/usr/bin/env python3

import threading

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class TopicBridge:
    def __init__(self, source_node, target_node, msg_type, source_topic, target_topic, qos):
        self.publisher = target_node.create_publisher(msg_type, target_topic, qos)
        self.subscription = source_node.create_subscription(
            msg_type,
            source_topic,
            self._callback,
            qos,
        )
        target_node.get_logger().info(f"Bridging {source_topic} -> {target_topic}")

    def _callback(self, msg):
        self.publisher.publish(msg)


def spin_executor(context, node):
    executor = MultiThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()


def main():
    source_context = rclpy.context.Context()
    target_context = rclpy.context.Context()

    rclpy.init(context=source_context, domain_id=2)
    rclpy.init(context=target_context, domain_id=1)

    source_node = Node("car2_domain2_source", context=source_context)
    target_node = Node("car2_domain1_bridge", context=target_context)

    default_qos = QoSProfile(depth=20)
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    tf_qos = QoSProfile(depth=100)
    tf_static_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    bridges = [
        TopicBridge(source_node, target_node, LaserScan, "/scan", "/car2/scan", sensor_qos),
        TopicBridge(source_node, target_node, OccupancyGrid, "/map", "/car2/map", tf_static_qos),
        TopicBridge(
            source_node,
            target_node,
            OccupancyGrid,
            "/map_updates",
            "/car2/map_updates",
            default_qos,
        ),
        TopicBridge(
            source_node,
            target_node,
            PoseWithCovarianceStamped,
            "/amcl_pose",
            "/car2/amcl_pose",
            default_qos,
        ),
        TopicBridge(
            source_node,
            target_node,
            PoseArray,
            "/particle_cloud",
            "/car2/particle_cloud",
            default_qos,
        ),
        TopicBridge(source_node, target_node, Odometry, "/odom", "/car2/odom", default_qos),
        TopicBridge(
            source_node,
            target_node,
            Odometry,
            "/odometry/filtered",
            "/car2/odometry/filtered",
            default_qos,
        ),
        TopicBridge(
            source_node,
            target_node,
            JointState,
            "/joint_states",
            "/car2/joint_states",
            default_qos,
        ),
        TopicBridge(
            source_node,
            target_node,
            String,
            "/robot_description",
            "/car2/robot_description",
            default_qos,
        ),
        TopicBridge(source_node, target_node, TFMessage, "/tf", "/car2/tf", tf_qos),
        TopicBridge(
            source_node,
            target_node,
            TFMessage,
            "/tf_static",
            "/car2/tf_static",
            tf_static_qos,
        ),
        TopicBridge(target_node, source_node, Twist, "/car2/cmd_vel", "/cmd_vel", default_qos),
        TopicBridge(
            target_node,
            source_node,
            PoseWithCovarianceStamped,
            "/car2/initialpose",
            "/initialpose",
            default_qos,
        ),
        TopicBridge(
            target_node,
            source_node,
            PoseStamped,
            "/car2/goal_pose",
            "/goal_pose",
            default_qos,
        ),
    ]

    source_thread = threading.Thread(target=spin_executor, args=(source_context, source_node))
    target_thread = threading.Thread(target=spin_executor, args=(target_context, target_node))
    source_thread.start()
    target_thread.start()

    try:
        source_thread.join()
        target_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        # Keep references alive until shutdown.
        _ = bridges
        source_node.destroy_node()
        target_node.destroy_node()
        source_context.shutdown()
        target_context.shutdown()


if __name__ == "__main__":
    main()
