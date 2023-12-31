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

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 1)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 1)
        self.gps_main = self.create_publisher(NavSatFix, "/gps_main", 10)
        self.gps_aux = self.create_publisher(NavSatFix, "/gps_aux", 10)

    def front_callback(self, msg):
        msg.header.frame_id = "gps"
        self.gps_main.publish(msg)
    
    def back_callback(self, msg):
        msg.header.frame_id = "gps_aux"
        self.gps_aux.publish(msg)

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