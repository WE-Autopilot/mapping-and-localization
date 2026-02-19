"""Bridge node that republishes SLAM odometry for localization consumers."""

from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time


class SlamBridgeNode(Node):
    """Republish SLAM odometry on stable localization topics."""

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
        self._output_rate_hz = float(
            self.declare_parameter('output_rate_hz', 20.0).value
        )
        self._odom_timeout_sec = float(
            self.declare_parameter('odom_timeout_sec', 0.5).value
        )
        self._publish_pose = bool(
            self.declare_parameter('publish_pose', True).value
        )
        self._qos_depth = int(self.declare_parameter('qos_depth', 10).value)

        if self._output_rate_hz <= 0.0:
            self.get_logger().warning(
                'output_rate_hz must be > 0. Falling back to 20.0 Hz.'
            )
            self._output_rate_hz = 20.0

        self._last_odom: Optional[Odometry] = None
        self._last_odom_rx_time: Optional[Time] = None

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

        self._timer = self.create_timer(
            1.0 / self._output_rate_hz,
            self._publish_outputs,
        )

        self.get_logger().info(
            (
                'SLAM bridge ready. '
                'odom: %s -> %s, pose topic: %s, rate: %.2f Hz'
            )
            % (
                self._input_slam_odom_topic,
                self._output_localization_odom_topic,
                self._output_localization_pose_topic,
                self._output_rate_hz,
            )
        )

    def _on_slam_odom(self, msg: Odometry) -> None:
        self._last_odom = deepcopy(msg)
        self._last_odom_rx_time = self.get_clock().now()

    def _publish_outputs(self) -> None:
        now = self.get_clock().now()
        if self._last_odom is None or not self._is_recent(
            self._last_odom_rx_time,
            now,
            self._odom_timeout_sec,
        ):
            return

        odom_message = deepcopy(self._last_odom)
        self._localization_odom_pub.publish(odom_message)

        if self._publish_pose:
            pose_message = PoseStamped()
            pose_message.header = odom_message.header
            pose_message.pose = odom_message.pose.pose
            self._localization_pose_pub.publish(pose_message)

    @staticmethod
    def _is_recent(
        received_time: Optional[Time],
        now: Time,
        timeout_sec: float,
    ) -> bool:
        if received_time is None:
            return False
        age_sec = (now - received_time).nanoseconds / 1e9
        return age_sec <= timeout_sec


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
