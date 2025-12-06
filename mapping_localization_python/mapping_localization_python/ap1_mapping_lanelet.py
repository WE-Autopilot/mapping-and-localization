#!/usr/bin/env python3
"""
ap1_mapping_Lanelet

ROS2 node that:
- Subscribes to the base OSM map (`/ap1/mapping/OSM/map`)
- Subscribes to perception-style lane waypoints
  (`/ap1/test/perception/lane_waypoints`)
- Builds real Lanelet2 lanelets (left bound, right bound, centerline)
- Inserts those lanelets into a LaneletMap along with the base OSM map
- Adds placeholder regulatory elements
- Publishes a valid Lanelet2 OSM XML HD map on `/ap1/mapping/full_had_map`.

This node is the "bridge" between perception-style lane geometry and
Lanelet2-style HD maps for Planning/Control.
"""

from typing import Optional, List

import os
import tempfile

import rclpy
from rclpy.node import Node

# ----- ROS messages (adjust names/fields if your .msg differ) -----
from ap1_msgs.msg import OsmMap       # must have: string osm_xml
from ap1_msgs.msg import LaneWaypoints  # must have: Point[] left_points, right_points
from ap1_msgs.msg import HDMap        # must have: string osm_xml
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Point

# ----- Lanelet2 imports -----
import lanelet2
from lanelet2.core import (
    AttributeMap,
    Lanelet,
    LaneletMap,
    LineString3d,
    Point3d,
    TrafficLight,
    getId,
)
from lanelet2.io import Origin, load, write
from lanelet2.projection import UtmProjector


class LaneletMappingNode(Node):
    """
    Node implementing the continuous OSM ➜ Lanelet2 HD map conversion.

    Subscriptions:
        /ap1/mapping/OSM/map           : ap1_msgs/OsmMap
        /ap1/test/perception/lane_waypoints : ap1_msgs/LaneWaypoints

    Publications:
        /ap1/mapping/full_had_map      : ap1_msgs/HDMap (Lanelet2 OSM XML)

    Behaviour:
        - Store latest base OSM XML string.
        - Store latest lane waypoints (left/right boundaries).
        - Each time both are available, rebuild a LaneletMap:
            - Load base OSM XML into LaneletMap
            - Convert waypoints -> left/right LineString3d
            - Compute centerline LineString3d
            - Construct Lanelet object and add to map
            - Add placeholder regulatory element (TrafficLight) to the lanelet
            - Serialize full map back to Lanelet2-compatible OSM XML
            - Publish as HDMap
    """

    def __init__(self) -> None:
        super().__init__("ap1_mapping_Lanelet")

        # --- Subscribers ---
        qos_latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.osm_sub = self.create_subscription(
            OsmMap,
            "/ap1/mapping/OSM/map",
            self.osm_callback,
            qos_latched,
        )

        self.waypoints_sub = self.create_subscription(
            LaneWaypoints,
            "/ap1/test/perception/lane_waypoints",
            self.lane_waypoints_callback,
            10,
        )

        # --- Publisher ---
        self.had_map_pub = self.create_publisher(
            HDMap,
            "/ap1/mapping/full_had_map",
            10,
        )

        # --- Internal state ---
        self._base_osm_xml: Optional[str] = None
        self._latest_waypoints: Optional[LaneWaypoints] = None

        # Projector: choose your real origin later (Lat, Lon)
        # For now, we assume local coordinates; origin just anchors the map.
        self._origin = Origin(0.0, 0.0)  # TODO: set to actual map origin
        self._projector = UtmProjector(self._origin)

        self.get_logger().info("ap1_mapping_Lanelet node started.")

    # ROS Callbacks
    def osm_callback(self, msg: OsmMap) -> None:
        """Store the most recent base OSM XML and try to rebuild the HD map."""
        self._base_osm_xml = msg.osm_xml
        self.get_logger().debug("Received new base OSM map.")
        self.maybe_update_hd_map()

    def lane_waypoints_callback(self, msg: LaneWaypoints) -> None:
        """Store the most recent lane waypoints and try to rebuild the HD map."""
        self._latest_waypoints = msg
        self.get_logger().debug("Received new lane waypoints.")
        self.maybe_update_hd_map()

    # Main update pipeline
    def maybe_update_hd_map(self) -> None:
        """
        Run a full OSM ➜ Lanelet2 update if we have both:
        - base OSM XML
        - latest lane waypoints
        """
        if self._base_osm_xml is None:
            self.get_logger().debug("Skipping update: base OSM map not yet received.")
            return

        if self._latest_waypoints is None:
            self.get_logger().debug("Skipping update: lane waypoints not yet received.")
            return

        try:
            lanelet_map = self.build_lanelet_map()
            osm_xml = self.serialize_lanelet_map(lanelet_map)
            self.publish_hd_map(osm_xml)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error while building HD Lanelet map: {exc}")

    # Lanelet2 map construction
    def build_lanelet_map(self) -> LaneletMap:
        """
        Build a LaneletMap by:
        1. Loading the base OSM XML into a LaneletMap.
        2. Converting lane waypoints into Lanelet2 primitives.
        3. Inserting the new Lanelet (with centerline + regulatory element).
        """
        # 1) Load base OSM XML into a LaneletMap
        base_map = self.load_lanelet_map_from_xml(self._base_osm_xml)

        # 2) Convert waypoints -> Lanelet primitives
        left_points, right_points = self.extract_waypoint_arrays(self._latest_waypoints)

        left_ls, right_ls = self.build_lane_boundaries(left_points, right_points)
        center_ls = self.build_centerline(left_points, right_points)

        # 3) Create Lanelet object from boundaries
        lanelet_attrs = AttributeMap()
        lanelet_attrs["type"] = "lanelet"
        lanelet_attrs["subtype"] = "road"

        lane = Lanelet(
            getId(),        # lanelet id
            left_ls,
            right_ls,
            lanelet_attrs,
        )

        # Set the centerline explicitly (helps some tools/planners)
        center_ls.attributes["type"] = "line_thin"
        center_ls.attributes["subtype"] = "centerline"

        base_map.add(center_ls)

        # 4) Add a placeholder regulatory element (TrafficLight) so spec sees <regulatory_element>
        self.attach_placeholder_regulatory_element(lane, center_ls)

        # 5) Add lanelet to the map (this also adds boundaries, points, regs, etc.)
        base_map.add(lane)

        return base_map

    def extract_waypoint_arrays(self, msg: LaneWaypoints) -> tuple[List[Point], List[Point]]:
        """
        Extract left/right waypoint arrays and ensure they have at least 2 points.

        We truncate both to the minimum length to keep them aligned.
        """
        left_points = list(msg.left_points)
        right_points = list(msg.right_points)

        if not left_points or not right_points:
            raise ValueError("LaneWaypoints must contain at least one left and right point.")

        # Use the min length to ensure 1-to-1 pairing
        n = min(len(left_points), len(right_points))
        if n < 2:
            raise ValueError("Need at least 2 waypoint pairs to build lane boundaries.")

        return left_points[:n], right_points[:n]

    def build_lane_boundaries(self, left_points: List[Point], right_points: List[Point]) -> tuple[LineString3d, LineString3d]:
        """
        Convert left/right geometry_msgs/Point arrays into LineString3d boundaries.
        """
        left_pts_3d = [
            Point3d(getId(), p.x, p.y, p.z) for p in left_points
        ]
        right_pts_3d = [
            Point3d(getId(), p.x, p.y, p.z) for p in right_points
        ]

        left_attrs = AttributeMap()
        left_attrs["type"] = "line_thin"
        left_attrs["subtype"] = "road_border"

        right_attrs = AttributeMap()
        right_attrs["type"] = "line_thin"
        right_attrs["subtype"] = "road_border"

        left_ls = LineString3d(getId(), left_pts_3d, left_attrs)
        right_ls = LineString3d(getId(), right_pts_3d, right_attrs)

        return left_ls, right_ls

    def build_centerline(self, left_points: List[Point], right_points: List[Point]) -> LineString3d:
        """
        Build a centerline LineString3d by averaging left/right boundaries pointwise.

        Math:
            center.x = (left.x + right.x) / 2
            center.y = (left.y + right.y) / 2
            center.z = (left.z + right.z) / 2
        """
        center_pts_3d: List[Point3d] = []

        for l, r in zip(left_points, right_points):
            cx = 0.5 * (l.x + r.x)
            cy = 0.5 * (l.y + r.y)
            cz = 0.5 * (l.z + r.z)
            center_pts_3d.append(Point3d(getId(), cx, cy, cz))

        center_attrs = AttributeMap()
        center_attrs["type"] = "line_thin"
        center_attrs["subtype"] = "centerline"

        center_ls = LineString3d(getId(), center_pts_3d, center_attrs)
        return center_ls

    def attach_placeholder_regulatory_element(self, lane: Lanelet, centerline: LineString3d) -> None:
        """
        Attach a minimal TrafficLight regulatory element as a placeholder.

        This will ensure that the exported OSM contains a <regulatory_element> section,
        which the spec explicitly requires (even as a stub).

        The geometry is fake but structurally valid:
            - trafficLightLinestring: single small segment near the end of the lane
            - stopLine: short segment just before it
        """
        try:
            # Use last centerline point as reference for the traffic light position
            last_pt = centerline[-1]
            before_last_pt = centerline[-2]

            # A tiny linestring representing the physical traffic light
            tl_pts = [
                Point3d(getId(), last_pt.x, last_pt.y, last_pt.z),
                Point3d(getId(), last_pt.x, last_pt.y + 0.5, last_pt.z),
            ]
            tl_ls_attrs = AttributeMap()
            tl_ls_attrs["type"] = "traffic_light"
            tl_ls = LineString3d(getId(), tl_pts, tl_ls_attrs)

            # A short stop line before the light
            stop_pts = [
                Point3d(getId(), before_last_pt.x, before_last_pt.y, before_last_pt.z),
                Point3d(getId(), before_last_pt.x, before_last_pt.y + 0.3, before_last_pt.z),
            ]
            stop_attrs = AttributeMap()
            stop_attrs["type"] = "stop_line"
            stop_line_ls = LineString3d(getId(), stop_pts, stop_attrs)

            # Construct TrafficLight regulatory element (see Lanelet2 Python bindings)
            reg_attrs = AttributeMap()
            reg_attrs["subtype"] = "traffic_light"

            regelem = TrafficLight(
                getId(),
                reg_attrs,
                [tl_ls],        # trafficLights
                stop_line_ls,   # optional stopLine
            )

            lane.addRegulatoryElement(regelem)
        except Exception as exc:  # noqa: BLE001
            # This is non-critical; the lanelet itself is still valid.
            self.get_logger().warn(f"Failed to attach placeholder regulatory element: {exc}")

    # Lanelet2 <-> XML helpers
    def load_lanelet_map_from_xml(self, xml: str) -> LaneletMap:
        """
        Load a LaneletMap from an OSM XML string using lanelet2.io.load().

        The lanelet2 Python API expects a filename, so we:
        1. Write the XML string into a temporary file.
        2. Call lanelet2.io.load(temp_file, projector).
        3. Delete the temporary file.
        """
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".osm", delete=False
        ) as tmp:
            tmp.write(xml)
            tmp.flush()
            tmp_path = tmp.name

        try:
            lanelet_map = load(tmp_path, self._projector)
        finally:
            try:
                os.remove(tmp_path)
            except OSError:
                pass

        return lanelet_map

    def serialize_lanelet_map(self, lanelet_map: LaneletMap) -> str:
        """
        Serialize a LaneletMap into an OSM XML string using lanelet2.io.write().

        The Python API writes to a file path, so we:
        1. Create a temporary file.
        2. Call write(temp_file, projector, lanelet_map).
        3. Read back the contents.
        4. Delete the temporary file.
        """
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".osm", delete=False
        ) as tmp:
            tmp_path = tmp.name

        try:
            write(tmp_path, lanelet_map, self._projector)
            with open(tmp_path, "r", encoding="utf-8") as f:
                xml = f.read()
        finally:
            try:
                os.remove(tmp_path)
            except OSError:
                pass

        return xml

    # Publishing
    def publish_hd_map(self, osm_xml: str) -> None:
        """
        Publish the updated Lanelet2 OSM XML as an HDMap message.
        """
        msg = HDMap()
        msg.osm_xml = osm_xml
        self.had_map_pub.publish(msg)
        self.get_logger().debug("Published updated /ap1/mapping/full_had_map.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneletMappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
