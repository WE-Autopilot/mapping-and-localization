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
    kitware_slam_params = LaunchConfiguration('kitware_slam_params')
    perception_output_rate_hz = LaunchConfiguration(
        'perception_output_rate_hz'
    )
    localization_output_rate_hz = LaunchConfiguration(
        'localization_output_rate_hz'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_kitware_slam',
                default_value='false',
                description='Start lidar_slam/lidar_slam_node in this launch.',
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
                'perception_output_rate_hz',
                default_value='20.0',
                description=(
                    'Stable output rate for entities and lane boundaries.'
                ),
            ),
            DeclareLaunchArgument(
                'localization_output_rate_hz',
                default_value='20.0',
                description='Stable output rate for localization topics.',
            ),
            Node(
                package='mapping_localization_python',
                executable='perception_pipeline_node',
                name='perception_pipeline',
                output='screen',
                parameters=[
                    {
                        'output_rate_hz': perception_output_rate_hz,
                    }
                ],
            ),
            Node(
                package='mapping_localization_python',
                executable='slam_bridge_node',
                name='slam_bridge',
                output='screen',
                parameters=[
                    {
                        'output_rate_hz': localization_output_rate_hz,
                    }
                ],
            ),
            Node(
                package='lidar_slam',
                executable='lidar_slam_node',
                name='kitware_lidar_slam',
                output='screen',
                parameters=[kitware_slam_params],
                condition=IfCondition(use_kitware_slam),
            ),
        ]
    )
