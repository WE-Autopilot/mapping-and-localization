"""Bridge node that republishes SLAM odometry for localization consumers."""

from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SlamBridgeNode(Node):
    """Republish SLAM odometry immediately on localization topics."""

    def __init__(self) -> None:
        super().__init__('slam_bridge_node')

        self._input_slam_odom_topic = str(
            self.declare_parameter('input_slam_odom_topic', '/slam_odom').value
        )
        self._output_localization_odom_topic = str(
            self.declare_parameter(
                'output_localization_odom_topic',
                '/localization/odom',
            ).value
        )
        self._output_localization_pose_topic = str(
            self.declare_parameter(
                'output_localization_pose_topic',
                '/localization/pose',
            ).value
        )
        self._publish_pose = bool(
            self.declare_parameter('publish_pose', True).value
        )
        self._qos_depth = int(self.declare_parameter('qos_depth', 10).value)

        self._slam_odom_sub = self.create_subscription(
            Odometry,
            self._input_slam_odom_topic,
            self._on_slam_odom,
            self._qos_depth,
        )
        self._localization_odom_pub = self.create_publisher(
            Odometry,
            self._output_localization_odom_topic,
            self._qos_depth,
        )
        self._localization_pose_pub = self.create_publisher(
            PoseStamped,
            self._output_localization_pose_topic,
            self._qos_depth,
        )

        self.get_logger().info(
            (
                'SLAM bridge ready. '
                'odom: %s -> %s, pose topic: %s, publish mode: callback-driven'
            )
            % (
                self._input_slam_odom_topic,
                self._output_localization_odom_topic,
                self._output_localization_pose_topic,
            )
        )

    def _on_slam_odom(self, msg: Odometry) -> None:
        odom_message = deepcopy(msg)
        self._localization_odom_pub.publish(odom_message)

        if self._publish_pose:
            pose_message = PoseStamped()
            pose_message.header = odom_message.header
            pose_message.pose = odom_message.pose.pose
            self._localization_pose_pub.publish(pose_message)


def main(args: Optional[list[str]] = None) -> None:
    """Run the SLAM bridge node."""
    rclpy.init(args=args)
    node = SlamBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
