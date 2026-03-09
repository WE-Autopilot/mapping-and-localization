"""Bridge node that republishes SLAM outputs for localization consumers."""

from copy import deepcopy
from math import sqrt
from typing import List, Optional, Tuple

import rclpy
from ap1_msgs.msg import FloatStamped
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SlamBridgeNode(Node):
    """Republish SLAM odometry and publish map-frame pose with covariance."""

    def __init__(self) -> None:
        super().__init__('slam_bridge_node')

        self._input_slam_odom_topic = str(
            self.declare_parameter('input_slam_odom_topic', '/slam_odom').value
        )
        self._output_localization_odom_topic = str(
            self.declare_parameter(
                'output_localization_odom_topic',
                '/ap1/localization/odom',
            ).value
        )
        self._output_localization_pose_topic = str(
            self.declare_parameter(
                'output_localization_pose_topic',
                '/ap1/localization/pose',
            ).value
        )
        self._output_slam_pose_topic = str(
            self.declare_parameter(
                'output_slam_pose_topic',
                '/ap1/localization/slam_pose',
            ).value
        )
        self._output_distance_topic = str(
            self.declare_parameter(
                'output_distance_topic',
                '/ap1/localization/distance',
            ).value
        )
        self._publish_pose = bool(
            self.declare_parameter('publish_pose', True).value
        )
        self._publish_slam_pose = bool(
            self.declare_parameter('publish_slam_pose', True).value
        )
        self._publish_distance = bool(
            self.declare_parameter('publish_distance', True).value
        )
        self._slam_pose_frame = str(
            self.declare_parameter('slam_pose_frame', 'map').value
        )
        self._slam_pose_publish_rate_hz = float(
            self.declare_parameter('slam_pose_publish_rate_hz', 20.0).value
        )
        self._slam_pose_frequency_log_interval_sec = float(
            self.declare_parameter(
                'slam_pose_frequency_log_interval_sec',
                5.0,
            ).value
        )
        self._transform_timeout_sec = float(
            self.declare_parameter('transform_timeout_sec', 0.05).value
        )
        self._fallback_position_variance = float(
            self.declare_parameter('fallback_position_variance', 0.05).value
        )
        self._fallback_orientation_variance = float(
            self.declare_parameter(
                'fallback_orientation_variance',
                0.1,
            ).value
        )
        self._distance_jump_threshold_m = float(
            self.declare_parameter(
                'distance_jump_threshold_m',
                10.0,
            ).value
        )
        self._qos_depth = int(self.declare_parameter('qos_depth', 10).value)

        if self._slam_pose_publish_rate_hz < 10.0:
            self.get_logger().warn(
                'slam_pose_publish_rate_hz must be >= 10.0; using 10.0 Hz'
            )
            self._slam_pose_publish_rate_hz = 10.0

        if self._slam_pose_frequency_log_interval_sec <= 0.0:
            self.get_logger().warn(
                'slam_pose_frequency_log_interval_sec must be > 0; using 5.0s'
            )
            self._slam_pose_frequency_log_interval_sec = 5.0

        if self._distance_jump_threshold_m <= 0.0:
            self.get_logger().warn(
                'distance_jump_threshold_m must be > 0; using 10.0m'
            )
            self._distance_jump_threshold_m = 10.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_slam_pose: Optional[PoseWithCovarianceStamped] = None
        self._last_distance_position: Optional[Tuple[float, float]] = None
        self._last_distance_frame: Optional[str] = None
        self._total_distance_m = 0.0
        self._slam_pose_publish_count = 0
        self._frequency_window_start = self.get_clock().now()
        self._last_transform_warning = (
            self.get_clock().now() - Duration(seconds=30.0)
        )

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
        self._slam_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self._output_slam_pose_topic,
            self._qos_depth,
        )
        self._distance_pub = self.create_publisher(
            FloatStamped,
            self._output_distance_topic,
            self._qos_depth,
        )

        self._slam_pose_timer = self.create_timer(
            1.0 / self._slam_pose_publish_rate_hz,
            self._publish_latest_slam_pose,
        )
        self._frequency_log_timer = self.create_timer(
            self._slam_pose_frequency_log_interval_sec,
            self._log_slam_pose_frequency,
        )

        self.get_logger().info(
            (
                'SLAM bridge ready. odom: %s -> %s, pose: %s, slam_pose: %s, '
                'distance: %s (frame: %s, rate: %.2f Hz)'
            )
            % (
                self._input_slam_odom_topic,
                self._output_localization_odom_topic,
                self._output_localization_pose_topic,
                self._output_slam_pose_topic,
                self._output_distance_topic,
                self._slam_pose_frame,
                self._slam_pose_publish_rate_hz,
            )
        )

    def _on_slam_odom(self, msg: Odometry) -> None:
        odom_message = deepcopy(msg)
        self._localization_odom_pub.publish(odom_message)
        if self._publish_distance:
            self._update_and_publish_distance(odom_message)

        if self._publish_pose:
            pose_message = PoseStamped()
            pose_message.header = odom_message.header
            pose_message.pose = odom_message.pose.pose
            self._localization_pose_pub.publish(pose_message)

        if not self._publish_slam_pose:
            return

        slam_pose_message = self._build_slam_pose_message(odom_message)
        if slam_pose_message is not None:
            self._latest_slam_pose = slam_pose_message

    def _update_and_publish_distance(self, odom_message: Odometry) -> None:
        source_frame = odom_message.header.frame_id.strip()
        current_position = (
            odom_message.pose.pose.position.x,
            odom_message.pose.pose.position.y,
        )

        if self._last_distance_position is None:
            self._last_distance_position = current_position
            self._last_distance_frame = source_frame
        elif self._last_distance_frame != source_frame:
            self.get_logger().warn(
                (
                    'Distance integration frame changed from %s to %s; '
                    'resetting reference point'
                )
                % (
                    self._last_distance_frame,
                    source_frame,
                )
            )
            self._last_distance_position = current_position
            self._last_distance_frame = source_frame
        else:
            delta_x = current_position[0] - self._last_distance_position[0]
            delta_y = current_position[1] - self._last_distance_position[1]
            step_distance = sqrt(delta_x * delta_x + delta_y * delta_y)

            # Integrate planar motion to avoid counting vertical pose noise.
            if step_distance <= self._distance_jump_threshold_m:
                self._total_distance_m += step_distance
            else:
                self.get_logger().warn(
                    'Ignoring %.2fm odometry jump while integrating distance'
                    % step_distance
                )

            self._last_distance_position = current_position

        distance_message = FloatStamped()
        distance_message.header = odom_message.header
        distance_message.header.frame_id = source_frame
        distance_message.value = float(self._total_distance_m)
        self._distance_pub.publish(distance_message)

    def _build_slam_pose_message(
        self,
        odom_message: Odometry,
    ) -> Optional[PoseWithCovarianceStamped]:
        source_frame = odom_message.header.frame_id.strip()
        if not source_frame:
            self.get_logger().warn(
                'Received /slam_odom without frame_id; skipping /slam_pose'
            )
            return None

        pose = deepcopy(odom_message.pose.pose)
        covariance = self._sanitize_covariance(
            list(odom_message.pose.covariance)
        )

        if source_frame != self._slam_pose_frame:
            transformed = self._transform_pose_and_covariance(
                pose=pose,
                covariance=covariance,
                source_frame=source_frame,
                target_frame=self._slam_pose_frame,
                stamp=odom_message.header.stamp,
            )
            if transformed is None:
                return None
            pose, covariance = transformed

        slam_pose_message = PoseWithCovarianceStamped()
        slam_pose_message.header = odom_message.header
        slam_pose_message.header.frame_id = self._slam_pose_frame
        slam_pose_message.pose.pose = pose
        slam_pose_message.pose.covariance = covariance
        return slam_pose_message

    def _publish_latest_slam_pose(self) -> None:
        if not self._publish_slam_pose or self._latest_slam_pose is None:
            return

        slam_pose_message = deepcopy(self._latest_slam_pose)
        slam_pose_message.header.stamp = self.get_clock().now().to_msg()
        self._slam_pose_pub.publish(slam_pose_message)
        self._slam_pose_publish_count += 1

    def _log_slam_pose_frequency(self) -> None:
        if not self._publish_slam_pose:
            return

        now = self.get_clock().now()
        elapsed = (now - self._frequency_window_start).nanoseconds / 1e9
        if elapsed <= 0.0:
            return

        rate = self._slam_pose_publish_count / elapsed
        self.get_logger().info(
            'Published %s at %.2f Hz over %.2fs'
            % (
                self._output_slam_pose_topic,
                rate,
                elapsed,
            )
        )
        self._slam_pose_publish_count = 0
        self._frequency_window_start = now

    def _transform_pose_and_covariance(
        self,
        pose: Pose,
        covariance: List[float],
        source_frame: str,
        target_frame: str,
        stamp: TimeMsg,
    ) -> Optional[Tuple[Pose, List[float]]]:
        transform = self._lookup_transform(
            target_frame=target_frame,
            source_frame=source_frame,
            stamp=stamp,
        )
        if transform is None:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform_quaternion = (
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        in_position = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        rotated_position = self._rotate_vector(in_position, transform_quaternion)

        in_orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        out_orientation = self._normalize_quaternion(
            self._quat_multiply(transform_quaternion, in_orientation)
        )

        out_pose = Pose()
        out_pose.position.x = rotated_position[0] + translation.x
        out_pose.position.y = rotated_position[1] + translation.y
        out_pose.position.z = rotated_position[2] + translation.z
        out_pose.orientation.x = out_orientation[0]
        out_pose.orientation.y = out_orientation[1]
        out_pose.orientation.z = out_orientation[2]
        out_pose.orientation.w = out_orientation[3]

        out_covariance = self._rotate_covariance(
            covariance,
            transform_quaternion,
        )
        return out_pose, out_covariance

    def _lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        stamp: TimeMsg,
    ):
        timeout = Duration(seconds=self._transform_timeout_sec)
        stamp_time = Time.from_msg(stamp)

        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp_time,
                timeout=timeout,
            )
        except TransformException:
            pass

        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=timeout,
            )
        except TransformException as ex:
            now = self.get_clock().now()
            if (now - self._last_transform_warning).nanoseconds >= int(5e9):
                self.get_logger().warn(
                    (
                        'Failed to transform %s -> %s for /slam_pose: %s'
                    )
                    % (
                        source_frame,
                        target_frame,
                        ex,
                    )
                )
                self._last_transform_warning = now
            return None

    def _sanitize_covariance(self, covariance: List[float]) -> List[float]:
        if len(covariance) != 36:
            return self._fallback_covariance_matrix()
        if any(abs(value) > 1e-12 for value in covariance):
            return covariance
        return self._fallback_covariance_matrix()

    def _fallback_covariance_matrix(self) -> List[float]:
        covariance = [0.0] * 36
        for idx in (0, 7, 14):
            covariance[idx] = self._fallback_position_variance
        for idx in (21, 28, 35):
            covariance[idx] = self._fallback_orientation_variance
        return covariance

    @staticmethod
    def _quat_multiply(
        first: Tuple[float, float, float, float],
        second: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        x1, y1, z1, w1 = first
        x2, y2, z2, w2 = second
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _normalize_quaternion(
        quaternion: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        x, y, z, w = quaternion
        norm = sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / norm
        return (x * inv, y * inv, z * inv, w * inv)

    @classmethod
    def _rotation_matrix(
        cls,
        quaternion: Tuple[float, float, float, float],
    ) -> List[List[float]]:
        x, y, z, w = cls._normalize_quaternion(quaternion)
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    @classmethod
    def _rotate_vector(
        cls,
        vector: Tuple[float, float, float],
        quaternion: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float]:
        rotation = cls._rotation_matrix(quaternion)
        return (
            rotation[0][0] * vector[0]
            + rotation[0][1] * vector[1]
            + rotation[0][2] * vector[2],
            rotation[1][0] * vector[0]
            + rotation[1][1] * vector[1]
            + rotation[1][2] * vector[2],
            rotation[2][0] * vector[0]
            + rotation[2][1] * vector[1]
            + rotation[2][2] * vector[2],
        )

    @classmethod
    def _rotate_covariance(
        cls,
        covariance: List[float],
        quaternion: Tuple[float, float, float, float],
    ) -> List[float]:
        if len(covariance) != 36:
            return covariance

        rotation = cls._rotation_matrix(quaternion)
        transform = [[0.0 for _ in range(6)] for _ in range(6)]
        for row in range(3):
            for col in range(3):
                transform[row][col] = rotation[row][col]
                transform[row + 3][col + 3] = rotation[row][col]

        cov_matrix = [
            [covariance[row * 6 + col] for col in range(6)]
            for row in range(6)
        ]
        transform_t = cls._transpose_6x6(transform)
        transformed = cls._matmul_6x6(transform, cov_matrix)
        transformed = cls._matmul_6x6(transformed, transform_t)
        return [
            transformed[row][col]
            for row in range(6)
            for col in range(6)
        ]

    @staticmethod
    def _transpose_6x6(matrix: List[List[float]]) -> List[List[float]]:
        return [
            [matrix[col][row] for col in range(6)]
            for row in range(6)
        ]

    @staticmethod
    def _matmul_6x6(
        left: List[List[float]],
        right: List[List[float]],
    ) -> List[List[float]]:
        product = [[0.0 for _ in range(6)] for _ in range(6)]
        for row in range(6):
            for col in range(6):
                product[row][col] = sum(
                    left[row][k] * right[k][col]
                    for k in range(6)
                )
        return product


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
