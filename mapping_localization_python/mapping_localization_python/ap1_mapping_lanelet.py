#!/usr/bin/env python3
"""
ap1_mapping_Lanelet Node
------------------------
Converts:
    - OSM base map  (/ap1/mapping/OSM/map)
    - Perception lane waypoints (/ap1/test/perception/lane_waypoints)

Into:
    - A full Lanelet2 HD map published on /ap1/mapping/full_had_map

This node runs continuously and updates the lanelet map whenever
new perception or OSM data arrives.
"""

import rclpy
from rclpy.node import Node
from ap1_msgs.msg import LaneWaypoints, OsmMapMsg, HDMap


class LaneletMappingNode(Node):
    def __init__(self):
        super().__init__("ap1_mapping_Lanelet")

        self.latest_osm = None
        self.latest_wp = None

        self.create_subscription(OsmMapMsg,
                                 "/ap1/mapping/OSM/map",
                                 self.on_osm, 10)

        self.create_subscription(LaneWaypoints,
                                 "/ap1/test/perception/lane_waypoints",
                                 self.on_wp, 10)

        self.pub = self.create_publisher(HDMap,
                                         "/ap1/mapping/full_had_map",
                                         10)

        self.timer = self.create_timer(0.5, self.process_map)

    def on_osm(self, msg):
        self.latest_osm = msg.osm_xml
        self.get_logger().info("Received OSM map")

    def on_wp(self, msg):
        self.latest_wp = msg
        self.get_logger().info("Received lane waypoints")

    def process_map(self):
        if self.latest_osm is None or self.latest_wp is None:
            return

        centerline = []

        for L, R in zip(self.latest_wp.left_lane,
                        self.latest_wp.right_lane):
            cx = (L.x + R.x) / 2.0
            cy = (L.y + R.y) / 2.0
            cz = (L.z + R.z) / 2.0
            centerline.append((cx, cy, cz))

        xml = "<lanelet_map>\n"
        xml += "  <centerline>\n"

        for i, (x, y, z) in enumerate(centerline):
            xml += f"    <pt id='{1000+i}' x='{x}' y='{y}' z='{z}' />\n"

        xml += "  </centerline>\n"
        xml += "</lanelet_map>"

        out = HDMap()
        out.lanelet_xml = xml
        self.pub.publish(out)

        self.get_logger().info("Published updated Lanelet HD map")


def main(args=None):
    rclpy.init(args=args)
    node = LaneletMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

