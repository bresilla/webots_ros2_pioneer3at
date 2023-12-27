import math
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger

class Remapper(Node):
    def __init__(self, args):
        super().__init__("pioneer3at_ros2")
        self.gps_back = self.create_subscription(NavSatFix, '/Pioneer3at/gps_back', self.back_callback, 10)
        self.gps_front = self.create_subscription(NavSatFix, '/Pioneer3at/gps_front', self.front_callback, 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 1)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 1)
        self.fix_front = self.create_publisher(NavSatFix, "/gps/front", 10)
        self.fix_back = self.create_publisher(NavSatFix, "/gps/back", 10)

    def front_callback(self, msg):
        new_msg = msg
        self.fix_front.publish(msg)
        self.fix_pub.publish(msg)
        self.gps_pub.publish(msg)
    
    def back_callback(self, msg):
        self.fix_back.publish(msg)

    def imu_callback(self, msg):
        self.imu_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)

def main(args=None):
    # pass
    rclpy.init(args=args)
    navfix = Remapper(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()