import rclpy
from rclpy.node import Node
from ap1_msgs.msg import LaneWaypoints
from geometry_msgs.msg import Point

class TestLane(Node):
    def __init__(self):
        super().__init__("test_lane_waypoints_pub")
        self.pub = self.create_publisher(LaneWaypoints,
                                         "/ap1/test/perception/lane_waypoints", 10)
        self.timer = self.create_timer(0.5, self.publish_fake_lane)

    def publish_fake_lane(self):
        msg = LaneWaypoints()

        # Left boundary (example)
        p1 = Point(x=0.0, y=1.0, z=0.0)
        p2 = Point(x=5.0, y=1.0, z=0.0)
        p3 = Point(x=10.0, y=1.0, z=0.0)

        # Right boundary (example)
        r1 = Point(x=0.0, y=-1.0, z=0.0)
        r2 = Point(x=5.0, y=-1.0, z=0.0)
        r3 = Point(x=10.0, y=-1.0, z=0.0)

        msg.left_lane = [p1, p2, p3]
        msg.right_lane = [r1, r2, r3]

        self.pub.publish(msg)
        self.get_logger().info("Published test lane waypoints")


def main():
    rclpy.init()
    node = TestLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
