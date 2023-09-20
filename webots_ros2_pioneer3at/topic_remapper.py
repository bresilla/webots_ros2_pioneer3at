import math
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger
from handy_msgs.msg import Float32Stamped

def distance(coord1, coord2):
    radius_earth = 6_367_449
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = radius_earth * c
    return distance

class Remapper(Node):
    def __init__(self, args):
        super().__init__("gps_fix")
        self.gps_sub = self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 1)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 1)

        self.dist_pub = self.create_publisher(Float32Stamped, "/gps/distance", 10)
        self.srv = self.create_service(Trigger, '/gps/reset_distance', self.reset_distance)
        self.ifix = None

    def imu_callback(self, msg):
        self.imu_pub.publish(msg)

    def reset_distance(self, request, response):
        self.ifix = None
        return response

    def gps_callback(self, msg):
        new_msg = Float32Stamped()
        new_msg.header = msg.header
        if self.ifix is None:
            self.ifix = msg
            new_msg.data = 0.0
        else:
            delta = distance((self.ifix.latitude, self.ifix.longitude), (msg.latitude, msg.longitude))
            new_msg.data = delta
        self.dist_pub.publish(new_msg)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navfix = Remapper(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

