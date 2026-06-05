"""Synthetic AP1 perception publisher for pipeline testing."""

import math
from typing import Optional

import rclpy
from ap1_msgs.msg import EntityState, EntityStateArray, LaneBoundaries
from geometry_msgs.msg import Point
from rclpy.node import Node


class SyntheticPerceptionPublisherNode(Node):
    """Publish deterministic AP1 sample data when Perception has none."""

    def __init__(self) -> None:
        super().__init__('synthetic_perception_publisher_node')

        self._output_entities_topic = str(
            self.declare_parameter(
                'output_entities_topic',
                '/ap1/perception/entities',
            ).value
        )
        self._output_lane_boundaries_topic = str(
            self.declare_parameter(
                'output_lane_boundaries_topic',
                '/ap1/perception/lanes',
            ).value
        )
        self._publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 10.0).value
        )
        self._waypoint_count = int(
            self.declare_parameter('waypoint_count', 16).value
        )
        self._waypoint_spacing_m = float(
            self.declare_parameter('waypoint_spacing_m', 0.5).value
        )
        self._lane_width_m = float(
            self.declare_parameter('lane_width_m', 3.5).value
        )
        self._curve_amplitude_m = float(
            self.declare_parameter('curve_amplitude_m', 1.2).value
        )
        self._curve_wavelength_m = float(
            self.declare_parameter('curve_wavelength_m', 45.0).value
        )
        self._tail_instability_points = int(
            self.declare_parameter('tail_instability_points', 4).value
        )
        self._tail_noise_max_m = float(
            self.declare_parameter('tail_noise_max_m', 0.9).value
        )
        self._qos_depth = int(self.declare_parameter('qos_depth', 10).value)

        if self._publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'publish_rate_hz must be > 0; using 10.0 Hz'
            )
            self._publish_rate_hz = 10.0

        if self._waypoint_count < 2:
            self.get_logger().warn('waypoint_count must be >= 2; using 16')
            self._waypoint_count = 16

        self._entities_pub = self.create_publisher(
            EntityStateArray,
            self._output_entities_topic,
            self._qos_depth,
        )
        self._lanes_pub = self.create_publisher(
            LaneBoundaries,
            self._output_lane_boundaries_topic,
            self._qos_depth,
        )
        self._publish_timer = self.create_timer(
            1.0 / self._publish_rate_hz,
            self._publish_sample_data,
        )

        self.get_logger().info(
            (
                'Synthetic perception publisher ready. entities: %s, '
                'lanes: %s, rate: %.2f Hz'
            )
            % (
                self._output_entities_topic,
                self._output_lane_boundaries_topic,
                self._publish_rate_hz,
            )
        )

    def _publish_sample_data(self) -> None:
        now = self.get_clock().now()
        current_time_sec = now.nanoseconds / 1e9

        entities_msg = EntityStateArray()
        entities_msg.header.stamp = now.to_msg()
        entities_msg.header.frame_id = 'base_link'
        entities_msg.entities = self._build_entities(current_time_sec)

        lanes_msg = LaneBoundaries()
        lanes_msg.header.stamp = now.to_msg()
        lanes_msg.header.frame_id = 'base_link'
        lanes_msg.left = self._build_lane_boundary(
            current_time_sec,
            is_left_lane=True,
        )
        lanes_msg.right = self._build_lane_boundary(
            current_time_sec,
            is_left_lane=False,
        )

        self._entities_pub.publish(entities_msg)
        self._lanes_pub.publish(lanes_msg)

    def _build_entities(self, current_time_sec: float) -> list[EntityState]:
        entities: list[EntityState] = []
        # Assumption: a small set of forward obstacles is enough to exercise
        # the AP1 entity path because Perception did not supply any tracks.
        base_positions = [
            (14.0, 0.8, 0.05),
            (27.0, -1.1, -0.03),
            (40.0, 2.4, 0.02),
        ]
        for index, (base_x, base_y, heading_bias) in enumerate(base_positions):
            entity = EntityState()
            entity.x = float(base_x + 0.6 * math.sin(current_time_sec + index))
            entity.y = float(
                base_y + 0.4 * math.cos(current_time_sec * 0.7 + index)
            )
            entity.z = 0.0
            entity.gamma = float(
                heading_bias + 0.08 * math.sin(current_time_sec * 0.5 + index)
            )
            entities.append(entity)
        return entities

    def _build_lane_boundary(
        self,
        current_time_sec: float,
        is_left_lane: bool,
    ) -> list[Point]:
        points: list[Point] = []
        # Assumption: these lane points are ego-relative with +X forward and +Y
        # left. The 0.5 m default spacing is a soft target of two waypoints per
        # meter, and the 3.5 m default lane width stays inside the 1-4 m range.
        side_sign = 1.0 if is_left_lane else -1.0
        tail_start_index = max(
            0,
            self._waypoint_count - self._tail_instability_points,
        )

        for index in range(self._waypoint_count):
            point = Point()
            distance_along_lane = index * self._waypoint_spacing_m
            curve_phase = (
                (2.0 * math.pi * distance_along_lane)
                / self._curve_wavelength_m
            ) + (current_time_sec * 0.2)
            centerline_y = 0.45 * self._curve_amplitude_m * math.sin(
                curve_phase
            )

            point.x = float(distance_along_lane)
            point.y = float(
                centerline_y + side_sign * (self._lane_width_m / 2.0)
            )
            point.z = 0.0

            # Left and right are sampled from the same centerline positions so
            # left[i] remains directly opposite right[i] for every waypoint.

            # Assumption: Perception only specified 16 waypoints and said the
            # prediction gets less stable near the tail, so the final few
            # points intentionally receive larger deterministic oscillations.
            if index >= tail_start_index and self._tail_instability_points > 0:
                tail_progress = (
                    (index - tail_start_index + 1)
                    / self._tail_instability_points
                )
                point.x += (
                    0.2
                    * self._tail_noise_max_m
                    * tail_progress
                    * math.sin(current_time_sec * 1.4 + index)
                )
                point.y += (
                    side_sign
                    * self._tail_noise_max_m
                    * tail_progress
                    * 0.35
                    * math.sin(current_time_sec * 2.1 + 0.8 * index)
                )

            points.append(point)

        return points


def main(args: Optional[list[str]] = None) -> None:
    """Run the synthetic perception publisher node."""
    rclpy.init(args=args)
    node = SyntheticPerceptionPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
