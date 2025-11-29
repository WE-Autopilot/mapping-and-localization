#!/usr/bin/env python3
"""
ap1_mapping_Lanelet Node
------------------------
This node fuses two map sources:

Inputs:
    1. OSM base map (topic: /ap1/mapping/OSM/map)
       - Message type: OsmMapMsg
       - Contains raw OSM XML data

    2. Perception lane waypoints (topic: /ap1/test/perception/lane_waypoints)
       - Message type: LaneWaypoints
       - Contains left_lane[] and right_lane[] arrays of geometry_msgs/Point

Output:
    - Publishes a minimal Lanelet2-style XML HD map on:
         /ap1/mapping/full_had_map
      - Message type: HDMap

Node Behavior:
    - Waits for both OSM XML and lane waypoints
    - Computes a simple centerline by averaging left/right lane boundaries
    - Creates a very simple XML lanelet structure
    - Publishes the HD map at 2 Hz (0.5s timer)

This is a simplified placeholder pipeline for the real Lanelet2 map-builder.
"""

import rclpy
from rclpy.node import Node
from ap1_msgs.msg import LaneWaypoints, OsmMapMsg, HDMap


class LaneletMappingNode(Node):
    """Node that converts OSM + lane waypoints into a Lanelet-like HD map."""

    def __init__(self):
        super().__init__("ap1_mapping_Lanelet")

        # Storage for latest incoming messages
        self.latest_osm = None       # raw OSM XML string
        self.latest_wp = None        # LaneWaypoints message

        # SUBSCRIPTIONS 

        # OSM XML input
        self.create_subscription(
            OsmMapMsg,
            "/ap1/mapping/OSM/map",
            self.on_osm,
            10
        )

        # Perception lane waypoints input
        self.create_subscription(
            LaneWaypoints,
            "/ap1/test/perception/lane_waypoints",
            self.on_wp,
            10
        )

        # PUBLISHER 

        # Publishes the generated HD map XML
        self.pub = self.create_publisher(
            HDMap,
            "/ap1/mapping/full_had_map",
            10
        )

        # Timer runs at 2 Hz and checks if both inputs are available
        self.timer = self.create_timer(0.5, self.process_map)

    # CALLBACKS
    def on_osm(self, msg):
        """Store latest OSM XML map."""
        self.latest_osm = msg.osm_xml
        self.get_logger().info("Received OSM map")

    def on_wp(self, msg):
        """Store latest lane waypoint message."""
        self.latest_wp = msg
        self.get_logger().info("Received lane waypoints")

    # MAIN MAP PROCESSING
    def process_map(self):
        """Builds a simple centerline and outputs a minimal Lanelet-like XML."""
        # Require both OSM and lane waypoints before processing
        if self.latest_osm is None or self.latest_wp is None:
            return

        # Compute centerline by averaging left and right lane boundaries
        centerline = []
        for L, R in zip(self.latest_wp.left_lane, self.latest_wp.right_lane):
            cx = (L.x + R.x) / 2.0
            cy = (L.y + R.y) / 2.0
            cz = (L.z + R.z) / 2.0
            centerline.append((cx, cy, cz))

        # Construct a simple Lanelet-like XML (placeholder)
        xml = "<lanelet_map>\n"
        xml += "  <centerline>\n"

        for i, (x, y, z) in enumerate(centerline):
            xml += f"    <pt id='{1000+i}' x='{x}' y='{y}' z='{z}' />\n"

        xml += "  </centerline>\n"
        xml += "</lanelet_map>"

        # Publish as HDMap message
        out = HDMap()
        out.lanelet_xml = xml
        self.pub.publish(out)

        self.get_logger().info("Published updated Lanelet HD map")


# NODE ENTRY POINT
def main(args=None):
    """Initialize ROS, start node, run event loop."""
    rclpy.init(args=args)
    node = LaneletMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
