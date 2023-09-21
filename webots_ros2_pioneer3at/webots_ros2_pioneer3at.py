import math
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger

class Remapper(Node):
    def __init__(self, args):
        super().__init__("gps_fix")
        self.gps_sub = self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 1)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 1)

    def imu_callback(self, msg):
        self.imu_pub.publish(msg)

    def gps_callback(self, msg):
        self.gps_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navfix = Remapper(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()