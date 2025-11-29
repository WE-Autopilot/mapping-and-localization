import rclpy
from rclpy.node import Node
from ap1_msgs.msg import OsmMapMsg

class TestOSM(Node):
    def __init__(self):
        super().__init__("test_osm_pub")
        self.pub = self.create_publisher(OsmMapMsg,
                                         "/ap1/mapping/OSM/map", 10)
        self.timer = self.create_timer(1.0, self.publish_osm)

    def publish_osm(self):
        msg = OsmMapMsg()
        msg.osm_xml = "<osm version='0.6'></osm>"  # empty OSM
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TestOSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
