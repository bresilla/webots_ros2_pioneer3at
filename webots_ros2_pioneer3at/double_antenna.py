import math
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger

class Remapper(Node):
    def __init__(self, args):
        super().__init__("gps_fix")
        self.get_logger().info('-----------------------------')
        self.get_logger().info('STARTING DOUBLE ANTENNA MODE')
        self.get_logger().info('-----------------------------')
        self.gps_back = self.create_subscription(NavSatFix, '/Pioneer3at/gps_back', self.back_callback, 10)
        self.gps_front = self.create_subscription(NavSatFix, '/Pioneer3at/gps_front', self.front_callback, 10)

        self.fix_front = self.create_publisher(NavSatFix, "/gps/front", 10)
        self.fix_back = self.create_publisher(NavSatFix, "/gps/back", 10)

    def front_callback(self, msg):
        new_msg = msg
        self.fix_front.publish(msg)
    
    def back_callback(self, msg):
        self.fix_back.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navfix = Remapper(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()