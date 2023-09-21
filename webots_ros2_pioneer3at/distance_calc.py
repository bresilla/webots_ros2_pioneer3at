import math
import rclpy
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.srv import Trigger, SetBool
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

def bearing(coord1, coord2):
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    diffLong = lon2 - lon1
    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    bearing = math.radians(bearing)
    bearing = math.pi - bearing
    return bearing

class Remapper(Node):
    def __init__(self, args):
        super().__init__("distance_calc")
        self.tmp_position = None
        self.fix_position = None
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.dist_pub = self.create_publisher(Float32Stamped, "/dot/distance", 10)
        self.angle_pub = self.create_publisher(Float32Stamped, "/dot/bearing", 10)
        self.odom_pub = self.create_publisher(Odometry, "/dot/odom", 10)
        self.dot_pub = self.create_publisher(NavSatFix, "/dot/fix", 10)
        self.srv = self.create_service(Trigger, '/dot/fix_set', self.fix_set)

    def fix_set(self, request, response):
        self.fix_position = self.tmp_position
        response.message = "Distance reset"
        response.success = True
        return response

    def gps_callback(self, msg):
        self.tmp_position = msg
        dist_msg = Float32Stamped()
        bear_msg = Float32Stamped()
        dist_msg.header = msg.header
        bear_msg.header = msg.header
        if self.fix_position is None:
            dist_msg.data = 0.0
            bear_msg.data = 0.0
            self.dist_pub.publish(dist_msg)
            self.angle_pub.publish(bear_msg)
            return
        
        dist = distance((self.fix_position.latitude, self.fix_position.longitude), (msg.latitude, msg.longitude))
        bear = bearing((self.fix_position.latitude, self.fix_position.longitude), (msg.latitude, msg.longitude))
        dist_msg.data = dist
        bear_msg.data = bear
        self.dot_pub.publish(self.fix_position)
        self.dist_pub.publish(dist_msg)
        self.angle_pub.publish(bear_msg)

        odom_msg = Odometry()
        odom_msg.header = msg.header
        # odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = dist * math.cos(bear)
        odom_msg.pose.pose.position.y = dist * math.sin(bear)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    navfix = Remapper(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

