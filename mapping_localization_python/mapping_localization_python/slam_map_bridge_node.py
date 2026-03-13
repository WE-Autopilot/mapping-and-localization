"""Bridge Kitware SLAM map topics into AP1 map outputs."""

import os
from copy import deepcopy
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty

from mapping_localization_python.point_cloud_utils import (
    build_occupancy_grid,
    extract_xyz_points,
    merge_point_clouds,
    write_ascii_pcd,
)

try:
    from lidar_slam.srv import SavePc
except ImportError:  # pragma: no cover - optional dependency at runtime.
    SavePc = None


class SlamMapBridgeNode(Node):
    """Publish a merged SLAM map and expose a save service."""

    def __init__(self) -> None:
        super().__init__('slam_map_bridge_node')

        self._input_map_topics = [
            str(topic)
            for topic in self.declare_parameter(
                'input_map_topics',
                [
                    '/maps/edges',
                    '/maps/intensity_edges',
                    '/maps/planes',
                    '/maps/blobs',
                ],
            ).value
        ]
        self._output_slam_map_topic = str(
            self.declare_parameter(
                'output_slam_map_topic',
                '/slam_map',
            ).value
        )
        self._output_slam_map_grid_topic = str(
            self.declare_parameter(
                'output_slam_map_grid_topic',
                '/slam_map_grid',
            ).value
        )
        self._publish_occupancy_grid = bool(
            self.declare_parameter('publish_occupancy_grid', False).value
        )
        self._map_frame = str(
            self.declare_parameter('map_frame', 'map').value
        )
        self._map_publish_rate_hz = float(
            self.declare_parameter('map_publish_rate_hz', 1.0).value
        )
        self._occupancy_grid_resolution_m = float(
            self.declare_parameter(
                'occupancy_grid_resolution_m',
                0.5,
            ).value
        )
        self._occupancy_grid_padding_m = float(
            self.declare_parameter('occupancy_grid_padding_m', 2.0).value
        )
        self._save_service_name = str(
            self.declare_parameter('save_service_name', '/save_map').value
        )
        self._save_map_prefix = os.path.expanduser(
            str(
                self.declare_parameter(
                    'save_map_prefix',
                    '~/slam_maps/slam_map',
                ).value
            )
        )
        self._save_merged_map = bool(
            self.declare_parameter('save_merged_map', True).value
        )
        self._upstream_save_service_name = str(
            self.declare_parameter(
                'upstream_save_service_name',
                '/lidar_slam/save_pc',
            ).value
        )
        self._upstream_save_format = int(
            self.declare_parameter('upstream_save_format', 2).value
        )
        self._upstream_save_filtered = bool(
            self.declare_parameter('upstream_save_filtered', False).value
        )
        self._upstream_save_fixed = bool(
            self.declare_parameter('upstream_save_fixed', False).value
        )
        self._qos_depth = int(self.declare_parameter('qos_depth', 10).value)

        if self._map_publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'map_publish_rate_hz must be > 0; using 1.0 Hz'
            )
            self._map_publish_rate_hz = 1.0
        elif not 0.5 <= self._map_publish_rate_hz <= 1.0:
            self.get_logger().warn(
                'map_publish_rate_hz %.2f is outside the typical 0.5-1.0 Hz '
                'range'
                % self._map_publish_rate_hz
            )

        if self._occupancy_grid_resolution_m <= 0.0:
            self.get_logger().warn(
                'occupancy_grid_resolution_m must be > 0; using 0.5 m'
            )
            self._occupancy_grid_resolution_m = 0.5

        self._cached_map_clouds: dict[str, PointCloud2] = {}
        self._warned_frame_topics: set[str] = set()
        self._pending_save_futures: list[object] = []
        self._map_subscriptions = []

        for topic in self._input_map_topics:
            subscription = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, topic_name=topic: self._on_map_cloud(
                    topic_name,
                    msg,
                ),
                self._qos_depth,
            )
            self._map_subscriptions.append(subscription)

        self._slam_map_pub = self.create_publisher(
            PointCloud2,
            self._output_slam_map_topic,
            self._qos_depth,
        )
        self._slam_map_grid_pub = self.create_publisher(
            OccupancyGrid,
            self._output_slam_map_grid_topic,
            self._qos_depth,
        )
        self._save_map_service = self.create_service(
            Empty,
            self._save_service_name,
            self._on_save_map,
        )
        self._map_publish_timer = self.create_timer(
            1.0 / self._map_publish_rate_hz,
            self._publish_map,
        )

        self._upstream_save_client = None
        if SavePc is not None and self._upstream_save_service_name:
            self._upstream_save_client = self.create_client(
                SavePc,
                self._upstream_save_service_name,
            )
        elif SavePc is None:
            self.get_logger().warn(
                'lidar_slam.srv.SavePc is unavailable; /save_map will only '
                'write the merged /slam_map PCD.'
            )

        self.get_logger().info(
            (
                'SLAM map bridge ready. inputs: %s, point cloud: %s, '
                'grid: %s, save service: %s, frame: %s, rate: %.2f Hz'
            )
            % (
                ', '.join(self._input_map_topics),
                self._output_slam_map_topic,
                self._output_slam_map_grid_topic,
                self._save_service_name,
                self._map_frame,
                self._map_publish_rate_hz,
            )
        )

    def _on_map_cloud(self, topic_name: str, msg: PointCloud2) -> None:
        # Assumption: Kitware SLAM is configured with odometry_frame=map, so
        # the published keypoint maps are already expressed in the map frame.
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != self._map_frame:
            if topic_name not in self._warned_frame_topics:
                self.get_logger().warn(
                    (
                        'Received %s in frame %s; publishing assumes the data '
                        'is already expressed in %s'
                    )
                    % (
                        topic_name,
                        frame_id,
                        self._map_frame,
                    )
                )
                self._warned_frame_topics.add(topic_name)

        self._cached_map_clouds[topic_name] = deepcopy(msg)

    def _publish_map(self) -> None:
        merged_cloud = self._build_merged_cloud()
        if merged_cloud is None:
            return

        self._slam_map_pub.publish(merged_cloud)

        if self._publish_occupancy_grid:
            grid = build_occupancy_grid(
                points=extract_xyz_points(merged_cloud),
                frame_id=self._map_frame,
                stamp=merged_cloud.header.stamp,
                resolution_m=self._occupancy_grid_resolution_m,
                padding_m=self._occupancy_grid_padding_m,
            )
            self._slam_map_grid_pub.publish(grid)

    def _build_merged_cloud(self) -> Optional[PointCloud2]:
        return merge_point_clouds(
            clouds=self._cached_map_clouds.values(),
            frame_id=self._map_frame,
            stamp=self.get_clock().now().to_msg(),
        )

    def _on_save_map(
        self,
        _request: Empty.Request,
        response: Empty.Response,
    ) -> Empty.Response:
        merged_cloud = self._build_merged_cloud()
        if merged_cloud is None:
            self.get_logger().warn(
                'No cached map data is available yet; skipping /save_map'
            )
            return response

        if self._save_merged_map:
            merged_output_path = f'{self._save_map_prefix}_merged.pcd'
            write_ascii_pcd(
                output_path=merged_output_path,
                points=extract_xyz_points(merged_cloud),
            )
            self.get_logger().info(
                'Saved merged SLAM point cloud to %s' % merged_output_path
            )

        self._trigger_upstream_save()
        return response

    def _trigger_upstream_save(self) -> None:
        if self._upstream_save_client is None or SavePc is None:
            return

        if not self._upstream_save_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                (
                    'Upstream save service %s is unavailable; saved only the '
                    'merged /slam_map PCD'
                )
                % self._upstream_save_service_name
            )
            return

        request = SavePc.Request()
        request.output_prefix_path = self._save_map_prefix
        request.format = self._upstream_save_format
        request.filtered = self._upstream_save_filtered
        request.fixed = self._upstream_save_fixed
        future = self._upstream_save_client.call_async(request)
        future.add_done_callback(self._on_upstream_save_done)
        self._pending_save_futures.append(future)

    def _on_upstream_save_done(self, future: object) -> None:
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - ROS future type.
            self.get_logger().error(
                'Upstream map save failed: %s' % exc
            )
        else:
            if getattr(result, 'success', False):
                self.get_logger().info(
                    (
                        'Saved Kitware keypoint maps with prefix %s. Reuse '
                        'that prefix via maps.initial_maps for localization.'
                    )
                    % self._save_map_prefix
                )
            else:
                self.get_logger().warn(
                    'Kitware save service returned an unsuccessful result'
                )
        finally:
            self._pending_save_futures = [
                pending_future
                for pending_future in self._pending_save_futures
                if pending_future is not future
            ]


def main(args: Optional[list[str]] = None) -> None:
    """Run the SLAM map bridge node."""
    rclpy.init(args=args)
    node = SlamMapBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
