"""Utilities for working with ROS PointCloud2 messages."""

import math
import os
import struct
from copy import deepcopy
from typing import Iterable, Optional, Sequence

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField


def point_count(cloud: PointCloud2) -> int:
    """Return the number of points in a point cloud."""
    return int(cloud.width) * int(cloud.height)


def merge_point_clouds(
    clouds: Iterable[PointCloud2],
    frame_id: str,
    stamp: TimeMsg,
) -> Optional[PointCloud2]:
    """Merge compatible PointCloud2 messages into a single cloud."""
    usable_clouds = [
        _flatten_cloud(cloud)
        for cloud in clouds
        if point_count(cloud) > 0 and int(cloud.point_step) > 0 and cloud.data
    ]
    if not usable_clouds:
        return None

    template = deepcopy(usable_clouds[0])
    signature = _layout_signature(template)
    merged_data = [bytes(template.data)]
    total_points = int(template.width)
    is_dense = bool(template.is_dense)

    for cloud in usable_clouds[1:]:
        if _layout_signature(cloud) != signature:
            continue
        merged_data.append(bytes(cloud.data))
        total_points += int(cloud.width)
        is_dense = is_dense and bool(cloud.is_dense)

    template.header.frame_id = frame_id
    template.header.stamp = stamp
    template.height = 1
    template.width = total_points
    template.row_step = int(template.point_step) * total_points
    template.is_dense = is_dense
    template.data = b''.join(merged_data)
    return template


def extract_xyz_points(cloud: PointCloud2) -> list[tuple[float, float, float]]:
    """Extract finite XYZ tuples from a PointCloud2 message."""
    fields = {field.name: field for field in cloud.fields}
    if not _has_float32_xyz(fields):
        return []

    data = bytes(cloud.data)
    endian = '>' if cloud.is_bigendian else '<'
    point_step = int(cloud.point_step)
    x_offset = int(fields['x'].offset)
    y_offset = int(fields['y'].offset)
    z_offset = int(fields['z'].offset)
    points: list[tuple[float, float, float]] = []

    for index in range(point_count(cloud)):
        base_offset = index * point_step
        x_value = struct.unpack_from(
            endian + 'f',
            data,
            base_offset + x_offset,
        )[0]
        y_value = struct.unpack_from(
            endian + 'f',
            data,
            base_offset + y_offset,
        )[0]
        z_value = struct.unpack_from(
            endian + 'f',
            data,
            base_offset + z_offset,
        )[0]
        if math.isfinite(x_value) and math.isfinite(y_value) and math.isfinite(
            z_value
        ):
            points.append((x_value, y_value, z_value))

    return points


def build_occupancy_grid(
    points: Sequence[tuple[float, float, float]],
    frame_id: str,
    stamp: TimeMsg,
    resolution_m: float,
    padding_m: float,
) -> OccupancyGrid:
    """Convert a point list into a simple XY occupancy grid."""
    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.header.stamp = stamp
    grid.info.resolution = float(resolution_m)
    grid.info.origin.orientation.w = 1.0

    if not points:
        return grid

    min_x = min(point[0] for point in points) - padding_m
    max_x = max(point[0] for point in points) + padding_m
    min_y = min(point[1] for point in points) - padding_m
    max_y = max(point[1] for point in points) + padding_m

    width = max(1, int(math.ceil((max_x - min_x) / resolution_m)) + 1)
    height = max(1, int(math.ceil((max_y - min_y) / resolution_m)) + 1)
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = min_x
    grid.info.origin.position.y = min_y

    data = [0] * (width * height)
    for x_value, y_value, _ in points:
        x_index = int((x_value - min_x) / resolution_m)
        y_index = int((y_value - min_y) / resolution_m)
        if 0 <= x_index < width and 0 <= y_index < height:
            data[y_index * width + x_index] = 100

    grid.data = data
    return grid


def write_ascii_pcd(
    output_path: str,
    points: Sequence[tuple[float, float, float]],
) -> None:
    """Write XYZ points to a simple ASCII PCD file."""
    expanded_path = os.path.expanduser(output_path)
    output_dir = os.path.dirname(expanded_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    with open(expanded_path, 'w', encoding='utf-8') as output_file:
        output_file.write('# .PCD v0.7 - Point Cloud Data file format\n')
        output_file.write('VERSION 0.7\n')
        output_file.write('FIELDS x y z\n')
        output_file.write('SIZE 4 4 4\n')
        output_file.write('TYPE F F F\n')
        output_file.write('COUNT 1 1 1\n')
        output_file.write(f'WIDTH {len(points)}\n')
        output_file.write('HEIGHT 1\n')
        output_file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        output_file.write(f'POINTS {len(points)}\n')
        output_file.write('DATA ascii\n')
        for x_value, y_value, z_value in points:
            output_file.write(
                f'{x_value:.6f} {y_value:.6f} {z_value:.6f}\n'
            )


def _flatten_cloud(cloud: PointCloud2) -> PointCloud2:
    """Normalize a cloud so it can be concatenated point-wise."""
    normalized = deepcopy(cloud)
    normalized.height = 1
    normalized.width = point_count(cloud)
    normalized.row_step = int(normalized.point_step) * int(normalized.width)
    normalized.data = bytes(cloud.data)
    return normalized


def _layout_signature(cloud: PointCloud2) -> tuple[object, ...]:
    """Describe the structural layout of a PointCloud2 message."""
    return (
        tuple(
            (field.name, field.offset, field.datatype, field.count)
            for field in cloud.fields
        ),
        int(cloud.point_step),
        bool(cloud.is_bigendian),
    )


def _has_float32_xyz(fields: dict[str, PointField]) -> bool:
    """Check that x, y, z are present as float32 fields."""
    required_names = ('x', 'y', 'z')
    return all(
        field_name in fields
        and fields[field_name].datatype == PointField.FLOAT32
        for field_name in required_names
    )
