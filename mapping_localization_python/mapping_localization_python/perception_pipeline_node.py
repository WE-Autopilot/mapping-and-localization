"""Event-driven perception pipeline for mapping/localization consumers."""

from copy import deepcopy
from typing import Optional

import rclpy
from ap1_msgs.msg import EntityStateArray, LaneBoundaries
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time


class PerceptionPipelineNode(Node):
    """Republish perception data immediately and fill missing lane boundaries."""

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
        self._lane_timeout_sec = float(
            self.declare_parameter('lane_timeout_sec', 1.0).value
        )
        self._min_lane_points = int(
            self.declare_parameter('min_lane_points', 2).value
        )

        # Queue length is one, we don't need a queue per say
        self._qos_depth = int(self.declare_parameter('qos_depth', 1).value)
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

        self.get_logger().info(
            (
                'Perception pipeline ready. '
                'entities: %s -> %s, lane_boundaries: %s -> %s, '
                'publish mode: callback-driven'
            )
            % (
                self._input_entities_topic,
                self._output_entities_topic,
                self._input_lane_boundaries_topic,
                self._output_lane_boundaries_topic,
            )
        )

    def _on_entities(self, msg: EntityStateArray) -> None:
        now = self.get_clock().now()
        self._entities_pub.publish(deepcopy(msg))
        self._publish_cached_lane_boundaries(now)

    def _on_lane_boundaries(self, msg: LaneBoundaries) -> None:
        now = self.get_clock().now()
        message_to_publish = deepcopy(msg)

        if len(message_to_publish.left) >= self._min_lane_points:
            self._cached_left_boundary = deepcopy(message_to_publish.left)
            self._last_left_rx_time = now
        elif self._cached_left_boundary and self._is_recent(
            self._last_left_rx_time,
            now,
            self._lane_timeout_sec,
        ):
            message_to_publish.left = deepcopy(self._cached_left_boundary)

        if len(message_to_publish.right) >= self._min_lane_points:
            self._cached_right_boundary = deepcopy(message_to_publish.right)
            self._last_right_rx_time = now
        elif self._cached_right_boundary and self._is_recent(
            self._last_right_rx_time,
            now,
            self._lane_timeout_sec,
        ):
            message_to_publish.right = deepcopy(self._cached_right_boundary)

        self._lane_boundaries_pub.publish(message_to_publish)

    def _publish_cached_lane_boundaries(self, now: Time) -> None:
        message_to_publish = LaneBoundaries()
        has_data = False

        if self._cached_left_boundary and self._is_recent(
            self._last_left_rx_time,
            now,
            self._lane_timeout_sec,
        ):
            message_to_publish.left = deepcopy(self._cached_left_boundary)
            has_data = True

        if self._cached_right_boundary and self._is_recent(
            self._last_right_rx_time,
            now,
            self._lane_timeout_sec,
        ):
            message_to_publish.right = deepcopy(self._cached_right_boundary)
            has_data = True

        if has_data:
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
