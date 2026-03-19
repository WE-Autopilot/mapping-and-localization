from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    map_file = LaunchConfiguration("map_file")
    slam_package = LaunchConfiguration("slam_package")
    slam_executable = LaunchConfiguration("slam_executable")

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="",
        description="Path to prebuilt map"
    )

    slam_package_arg = DeclareLaunchArgument(
        "slam_package",
        default_value="kitware_slam"
    )

    slam_executable_arg = DeclareLaunchArgument(
        "slam_executable",
        default_value="slam_node"
    )

    pkg_share = get_package_share_directory("mapping_localization_python")
    params_file = os.path.join(pkg_share, "config", "slam_params_localization.yaml")

    slam_node = Node(
        package=slam_package,
        executable=slam_executable,
        name="lidar_slam_localization",
        output="screen",
        parameters=[params_file, {"map_file": map_file}],
    )

    return LaunchDescription([
        map_file_arg,
        slam_package_arg,
        slam_executable_arg,
        slam_node,
    ])

