"""Stable-rate perception pipeline for mapping/localization consumers."""

from copy import deepcopy
from typing import Optional

import rclpy
from ap1_msgs.msg import EntityStateArray, LaneBoundaries
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time


class PerceptionPipelineNode(Node):
    """Republish perception data and fill missing lane boundaries."""

    def __init__(self) -> None:
        super().__init__('perception_pipeline_node')

        self._input_entities_topic = str(
            self.declare_parameter(
                'input_entities_topic',
                '/perception/entities',
            ).value
        )
        self._input_lane_boundaries_topic = str(
            self.declare_parameter(
                'input_lane_boundaries_topic',
                '/perception/lane_boundaries',
            ).value
        )
        self._output_entities_topic = str(
            self.declare_parameter(
                'output_entities_topic',
                '/mapping/stable/entities',
            ).value
        )
        self._output_lane_boundaries_topic = str(
            self.declare_parameter(
                'output_lane_boundaries_topic',
                '/mapping/stable/lane_boundaries',
            ).value
        )
        self._output_rate_hz = float(
            self.declare_parameter('output_rate_hz', 20.0).value
        )
        self._entity_timeout_sec = float(
            self.declare_parameter('entity_timeout_sec', 0.5).value
        )
        self._lane_timeout_sec = float(
            self.declare_parameter('lane_timeout_sec', 1.0).value
        )
        self._min_lane_points = int(
            self.declare_parameter('min_lane_points', 2).value
        )
        self._qos_depth = int(self.declare_parameter('qos_depth', 10).value)

        if self._output_rate_hz <= 0.0:
            self.get_logger().warning(
                'output_rate_hz must be > 0. Falling back to 20.0 Hz.'
            )
            self._output_rate_hz = 20.0

        self._last_entities: Optional[EntityStateArray] = None
        self._last_entities_rx_time: Optional[Time] = None
        self._cached_left_boundary: list[Point] = []
        self._cached_right_boundary: list[Point] = []
        self._last_left_rx_time: Optional[Time] = None
        self._last_right_rx_time: Optional[Time] = None

        self._entities_sub = self.create_subscription(
            EntityStateArray,
            self._input_entities_topic,
            self._on_entities,
            self._qos_depth,
        )
        self._lane_boundaries_sub = self.create_subscription(
            LaneBoundaries,
            self._input_lane_boundaries_topic,
            self._on_lane_boundaries,
            self._qos_depth,
        )

        self._entities_pub = self.create_publisher(
            EntityStateArray,
            self._output_entities_topic,
            self._qos_depth,
        )
        self._lane_boundaries_pub = self.create_publisher(
            LaneBoundaries,
            self._output_lane_boundaries_topic,
            self._qos_depth,
        )

        self._timer = self.create_timer(
            1.0 / self._output_rate_hz,
            self._publish_stable_outputs,
        )

        self.get_logger().info(
            (
                'Perception pipeline ready. '
                'entities: %s -> %s, lane_boundaries: %s -> %s, '
                'rate: %.2f Hz'
            )
            % (
                self._input_entities_topic,
                self._output_entities_topic,
                self._input_lane_boundaries_topic,
                self._output_lane_boundaries_topic,
                self._output_rate_hz,
            )
        )

    def _on_entities(self, msg: EntityStateArray) -> None:
        self._last_entities = deepcopy(msg)
        self._last_entities_rx_time = self.get_clock().now()

    def _on_lane_boundaries(self, msg: LaneBoundaries) -> None:
        now = self.get_clock().now()

        if len(msg.left) >= self._min_lane_points:
            self._cached_left_boundary = deepcopy(msg.left)
            self._last_left_rx_time = now

        if len(msg.right) >= self._min_lane_points:
            self._cached_right_boundary = deepcopy(msg.right)
            self._last_right_rx_time = now

    def _publish_stable_outputs(self) -> None:
        now = self.get_clock().now()
        self._publish_entities(now)
        self._publish_lane_boundaries(now)

    def _publish_entities(self, now: Time) -> None:
        message_to_publish = EntityStateArray()
        if self._last_entities is not None and self._is_recent(
            self._last_entities_rx_time,
            now,
            self._entity_timeout_sec,
        ):
            message_to_publish = deepcopy(self._last_entities)
        self._entities_pub.publish(message_to_publish)

    def _publish_lane_boundaries(self, now: Time) -> None:
        message_to_publish = LaneBoundaries()

        if self._cached_left_boundary and self._is_recent(
            self._last_left_rx_time,
            now,
            self._lane_timeout_sec,
        ):
            message_to_publish.left = deepcopy(self._cached_left_boundary)

        if self._cached_right_boundary and self._is_recent(
            self._last_right_rx_time,
            now,
            self._lane_timeout_sec,
        ):
            message_to_publish.right = deepcopy(self._cached_right_boundary)

        self._lane_boundaries_pub.publish(message_to_publish)

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
    """Run the perception pipeline node."""
    rclpy.init(args=args)
    node = PerceptionPipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
