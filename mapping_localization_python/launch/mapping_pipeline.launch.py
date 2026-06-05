"""Launch a bare-minimum mapping/localization pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create launch description for pipeline and optional Kitware SLAM."""
    use_kitware_slam = LaunchConfiguration('use_kitware_slam')
    use_synthetic_perception = LaunchConfiguration('use_synthetic_perception')
    use_synthetic_odometry = LaunchConfiguration('use_synthetic_odometry')
    kitware_slam_params = LaunchConfiguration('kitware_slam_params')
    kitware_initial_maps_path = LaunchConfiguration(
        'kitware_initial_maps_path'
    )
    output_slam_pose_topic = LaunchConfiguration('output_slam_pose_topic')
    output_distance_topic = LaunchConfiguration('output_distance_topic')
    output_slam_map_topic = LaunchConfiguration('output_slam_map_topic')
    output_slam_map_grid_topic = LaunchConfiguration(
        'output_slam_map_grid_topic'
    )
    slam_pose_frame = LaunchConfiguration('slam_pose_frame')
    slam_pose_publish_rate_hz = LaunchConfiguration(
        'slam_pose_publish_rate_hz'
    )
    slam_pose_frequency_log_interval_sec = LaunchConfiguration(
        'slam_pose_frequency_log_interval_sec'
    )
    slam_map_publish_rate_hz = LaunchConfiguration('slam_map_publish_rate_hz')
    publish_slam_map_grid = LaunchConfiguration('publish_slam_map_grid')
    save_map_prefix = LaunchConfiguration('save_map_prefix')
    synthetic_publish_rate_hz = LaunchConfiguration(
        'synthetic_publish_rate_hz'
    )
    synthetic_odometry_publish_rate_hz = LaunchConfiguration(
        'synthetic_odometry_publish_rate_hz'
    )
    synthetic_odometry_speed_mps = LaunchConfiguration(
        'synthetic_odometry_speed_mps'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_kitware_slam',
                default_value='false',
                description='Start lidar_slam/lidar_slam_node in this launch.',
            ),
            DeclareLaunchArgument(
                'use_synthetic_perception',
                default_value='false',
                description=(
                    'Publish deterministic AP1 sample perception data.'
                ),
            ),
            DeclareLaunchArgument(
                'use_synthetic_odometry',
                default_value='false',
                description=(
                    'Publish deterministic /slam_odom for smoke tests.'
                ),
            ),
            DeclareLaunchArgument(
                'kitware_slam_params',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('mapping_localization_python'),
                        'config',
                        'kitware_slam_params.yaml',
                    ]
                ),
                description=(
                    'Parameter YAML passed to lidar_slam_node when enabled.'
                ),
            ),
            DeclareLaunchArgument(
                'kitware_initial_maps_path',
                default_value='',
                description='Optional keypoint map prefix to preload in SLAM.',
            ),
            DeclareLaunchArgument(
                'output_slam_pose_topic',
                default_value='/ap1/localization/slam_pose',
                description='Topic for PoseWithCovarianceStamped SLAM pose.',
            ),
            DeclareLaunchArgument(
                'output_distance_topic',
                default_value='/ap1/localization/distance',
                description='Topic for cumulative distance travelled.',
            ),
            DeclareLaunchArgument(
                'output_slam_map_topic',
                default_value='/slam_map',
                description='Topic for the merged SLAM map point cloud.',
            ),
            DeclareLaunchArgument(
                'output_slam_map_grid_topic',
                default_value='/slam_map_grid',
                description='Topic for the optional SLAM occupancy grid.',
            ),
            DeclareLaunchArgument(
                'slam_pose_frame',
                default_value='map',
                description='Frame for SLAM pose output.',
            ),
            DeclareLaunchArgument(
                'slam_pose_publish_rate_hz',
                default_value='20.0',
                description='Publish rate for SLAM pose (minimum 10 Hz).',
            ),
            DeclareLaunchArgument(
                'slam_pose_frequency_log_interval_sec',
                default_value='5.0',
                description='Interval in seconds to log SLAM pose rate.',
            ),
            DeclareLaunchArgument(
                'slam_map_publish_rate_hz',
                default_value='1.0',
                description='Publish rate for the merged SLAM map.',
            ),
            DeclareLaunchArgument(
                'publish_slam_map_grid',
                default_value='false',
                description=(
                    'Also publish the merged map as an occupancy grid.'
                ),
            ),
            DeclareLaunchArgument(
                'save_map_prefix',
                default_value='~/slam_maps/slam_map',
                description='Prefix used by /save_map for PCD output.',
            ),
            DeclareLaunchArgument(
                'synthetic_publish_rate_hz',
                default_value='10.0',
                description='Publish rate for synthetic AP1 perception data.',
            ),
            DeclareLaunchArgument(
                'synthetic_odometry_publish_rate_hz',
                default_value='20.0',
                description='Publish rate for synthetic /slam_odom data.',
            ),
            DeclareLaunchArgument(
                'synthetic_odometry_speed_mps',
                default_value='1.0',
                description='Forward speed for synthetic /slam_odom data.',
            ),
            Node(
                package='mapping_localization_python',
                executable='perception_pipeline_node',
                name='perception_pipeline',
                output='screen',
            ),
            Node(
                package='mapping_localization_python',
                executable='stored_point_registry_node',
                name='stored_point_registry',
                output='screen',
            ),
            Node(
                package='mapping_localization_python',
                executable='synthetic_perception_publisher_node',
                name='synthetic_perception_publisher',
                output='screen',
                parameters=[
                    {
                        'publish_rate_hz': synthetic_publish_rate_hz,
                    }
                ],
                condition=IfCondition(use_synthetic_perception),
            ),
            Node(
                package='mapping_localization_python',
                executable='synthetic_slam_odom_publisher_node',
                name='synthetic_slam_odom_publisher',
                output='screen',
                parameters=[
                    {
                        'publish_rate_hz': synthetic_odometry_publish_rate_hz,
                        'speed_mps': synthetic_odometry_speed_mps,
                    }
                ],
                condition=IfCondition(use_synthetic_odometry),
            ),
            Node(
                package='mapping_localization_python',
                executable='slam_bridge_node',
                name='slam_bridge',
                output='screen',
                parameters=[
                    {
                        'output_slam_pose_topic': output_slam_pose_topic,
                        'output_distance_topic': output_distance_topic,
                        'slam_pose_frame': slam_pose_frame,
                        'slam_pose_publish_rate_hz': slam_pose_publish_rate_hz,
                        'slam_pose_frequency_log_interval_sec': (
                            slam_pose_frequency_log_interval_sec
                        ),
                    }
                ],
            ),
            Node(
                package='mapping_localization_python',
                executable='slam_map_bridge_node',
                name='slam_map_bridge',
                output='screen',
                parameters=[
                    {
                        'output_slam_map_topic': output_slam_map_topic,
                        'output_slam_map_grid_topic': (
                            output_slam_map_grid_topic
                        ),
                        'map_publish_rate_hz': slam_map_publish_rate_hz,
                        'publish_occupancy_grid': publish_slam_map_grid,
                        'save_map_prefix': save_map_prefix,
                    }
                ],
                condition=IfCondition(use_kitware_slam),
            ),
            Node(
                package='lidar_slam',
                executable='lidar_slam_node',
                name='kitware_lidar_slam',
                output='screen',
                parameters=[
                    kitware_slam_params,
                    {'maps.initial_maps': kitware_initial_maps_path},
                ],
                condition=IfCondition(use_kitware_slam),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_lidar_tf',
                arguments=[
                    '--x',
                    '0.0',  # meters left/right (0 = centered)
                    '--y',
                    '0.0',  # meters forward/back (0 = centered)
                    '--z',
                    '0.3',  # meters up/down (0.3 = 30cm above base)
                    '--roll',
                    '0.0',  # rotation around X axis (0 = no tilt)
                    '--pitch',
                    '0.0',  # rotation around Y axis (0 = no tilt)
                    '--yaw',
                    '0.0',  # rotation around Z axis (0 = facing forward)
                    '--frame-id',
                    'base_link',  # parent frame (the robot body)
                    '--child-frame-id',
                    'lidar_frame',  # child frame (the sensor)
                ],
                output='screen',
            ),
        ]
    )
