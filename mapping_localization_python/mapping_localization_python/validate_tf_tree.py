"""Validate the ROS2 TF tree for the mapping/localization pipeline."""

import time

import rclpy
import rclpy.node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# ── Expected transform values ──────────────────────────────────────────────
PARENT_FRAME = 'base_link'
CHILD_FRAME = 'lidar_frame'
EXPECTED_Z = 0.3       # metres above base_link
TOLERANCE = 0.01       # acceptable error in metres

# Frames needed for the full SLAM chain (requires SLAM to be running)
MAP_FRAME = 'map'
ODOM_FRAME = 'odom'

# How long to wait for transforms to arrive before checking
WAIT_SECONDS = 5.0


def _pass(msg: str) -> None:
    print(f'[PASS] {msg}')


def _fail(msg: str) -> None:
    print(f'[FAIL] {msg}')


def _warn(msg: str) -> None:
    print(f'[WARN] {msg}')


def check_static_transform(buffer: tf2_ros.Buffer) -> bool:
    """Check that base_link -> lidar_frame transform exists."""
    try:
        buffer.lookup_transform(PARENT_FRAME, CHILD_FRAME, rclpy.time.Time())
        _pass(f'{PARENT_FRAME} -> {CHILD_FRAME} transform exists')
        return True
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        _fail(f'{PARENT_FRAME} -> {CHILD_FRAME} transform not found: {e}')
        return False


def check_transform_values(buffer: tf2_ros.Buffer) -> bool:
    """Check that the translation values match expected mounting position."""
    try:
        t = buffer.lookup_transform(PARENT_FRAME, CHILD_FRAME, rclpy.time.Time())
        trans = t.transform.translation

        x_ok = abs(trans.x) <= TOLERANCE
        y_ok = abs(trans.y) <= TOLERANCE
        z_ok = abs(trans.z - EXPECTED_Z) <= TOLERANCE

        if x_ok and y_ok and z_ok:
            _pass(
                f'Transform values correct: '
                f'x={trans.x:.3f} y={trans.y:.3f} z={trans.z:.3f}'
            )
            return True
        else:
            _fail(
                f'Transform values incorrect: '
                f'x={trans.x:.3f} y={trans.y:.3f} z={trans.z:.3f} '
                f'(expected x=0.0 y=0.0 z={EXPECTED_Z})'
            )
            return False
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        _fail(f'Could not read transform values: {e}')
        return False


def check_full_chain(buffer: tf2_ros.Buffer) -> bool:
    """Check the full map -> odom -> base_link -> lidar_frame chain.

    This requires SLAM to be running with live or recorded sensor data.
    A failure here does NOT mean the static TF is broken.
    """
    try:
        buffer.lookup_transform(MAP_FRAME, CHILD_FRAME, rclpy.time.Time())
        _pass(f'Full chain {MAP_FRAME} -> {ODOM_FRAME} -> {PARENT_FRAME} -> {CHILD_FRAME} connected')
        return True
    except (LookupException, ConnectivityException) as e:
        _warn(
            f'Full chain not available (SLAM probably not running): {e}\n'
            f'       This is expected in a dev environment without live sensor data.'
        )
        return None   # None = skipped, not a hard failure


def check_no_duplicates(node: rclpy.node.Node) -> bool:
    """Check that only one node is publishing the base_link -> lidar_frame transform."""
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    import threading

    publishers_seen = []

    qos = QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    def callback(msg):
        for t in msg.transforms:
            if (
                t.header.frame_id == PARENT_FRAME
                and t.child_frame_id == CHILD_FRAME
            ):
                publishers_seen.append(t.header.frame_id)

    from tf2_msgs.msg import TFMessage
    sub = node.create_subscription(TFMessage, '/tf_static', callback, qos)  # noqa: F841

    # Spin briefly to collect messages
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    count = len(publishers_seen)
    if count == 1:
        _pass('No duplicate publishers for base_link -> lidar_frame (found 1)')
        return True
    elif count == 0:
        _fail('No publisher found for base_link -> lidar_frame on /tf_static')
        return False
    else:
        _fail(f'Duplicate publishers detected: {count} messages seen for base_link -> lidar_frame')
        return False


def main() -> None:
    """Run all TF validation checks and report results."""
    rclpy.init()
    node = rclpy.create_node('tf_validator')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)  # noqa: F841

    print(f'\nWaiting {WAIT_SECONDS}s for transforms to arrive...\n')

    # Spin while waiting so the listener can receive transforms
    deadline = time.time() + WAIT_SECONDS
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('─' * 50)
    print('TF TREE VALIDATION RESULTS')
    print('─' * 50)

    results = []
    results.append(check_static_transform(buffer))
    results.append(check_transform_values(buffer))
    results.append(check_no_duplicates(node))

    full_chain = check_full_chain(buffer)
    if full_chain is not None:
        results.append(full_chain)

    print('─' * 50)

    hard_failures = [r for r in results if r is False]
    if not hard_failures:
        print('OVERALL: ALL CHECKS PASSED ✓')
    else:
        print(f'OVERALL: {len(hard_failures)} CHECK(S) FAILED ✗')

    print('─' * 50)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

