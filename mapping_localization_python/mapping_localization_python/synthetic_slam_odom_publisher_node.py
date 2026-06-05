"""Publish deterministic SLAM odometry for pipeline smoke tests."""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class SyntheticSlamOdomPublisherNode(Node):
    """Publish simple forward odometry on the same topic as SLAM."""

    def __init__(self) -> None:
        """Initialize publisher parameters and timer."""
        super().__init__('synthetic_slam_odom_publisher')

        self._output_topic = str(
            self.declare_parameter('output_topic', '/slam_odom').value
        )
        self._frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self._child_frame_id = str(
            self.declare_parameter('child_frame_id', 'base_link').value
        )
        self._publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 20.0).value
        )
        self._speed_mps = float(self.declare_parameter('speed_mps', 1.0).value)
        self._yaw_rate_radps = float(
            self.declare_parameter('yaw_rate_radps', 0.0).value
        )

        if self._publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'publish_rate_hz must be > 0; using 20.0 Hz'
            )
            self._publish_rate_hz = 20.0

        self._publisher = self.create_publisher(
            Odometry,
            self._output_topic,
            10,
        )
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(
            1.0 / self._publish_rate_hz,
            self._publish_odom,
        )

        self.get_logger().info(
            (
                'Synthetic SLAM odometry publisher ready. output: %s, '
                'speed: %.2f m/s, yaw_rate: %.2f rad/s'
            )
            % (self._output_topic, self._speed_mps, self._yaw_rate_radps)
        )

    def _publish_odom(self) -> None:
        now = self.get_clock().now()
        elapsed_sec = (now - self._start_time).nanoseconds / 1_000_000_000.0
        yaw = self._yaw_rate_radps * elapsed_sec

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.child_frame_id = self._child_frame_id
        msg.pose.pose.position.x = self._speed_mps * elapsed_sec
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = self._speed_mps
        msg.twist.twist.angular.z = self._yaw_rate_radps
        self._publisher.publish(msg)


def main(args=None) -> None:
    """Run the synthetic SLAM odometry publisher node."""
    rclpy.init(args=args)
    node = SyntheticSlamOdomPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
