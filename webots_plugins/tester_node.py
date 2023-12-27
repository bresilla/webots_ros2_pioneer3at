import rclpy
from std_msgs.msg import String

class Node:
    # The `init` method is called only once the driver is initialized. You will always get two arguments in the `init` method.
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node. However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.create_node('test_node')

        # This will print the parameter from the URDF file. `{ 'parameterExample': 'someValue' }`
        self.properties = properties

        # The robot property allows you to access the Standard Webots API.
        # See: https://cyberbotics.com/doc/reference/robot
        self.robot = webots_node.robot
        self.__node.get_logger().info('  - robot: ' + str(self.robot.getName()))
        self.__node.get_logger().info('  - timestep: ' + str(int(self.robot.getBasicTimeStep())))

        # The robot property allows you to access the Supervisor Webots API if the robot is a Supervisor.
        # See: https://cyberbotics.com/doc/reference/supervisor
        self.super = webots_node.robot.getSelf()
        self.__node.get_logger().info('  - supervisor? ' + str(self.robot.getSupervisor()))
        

        self.__node.create_timer(1.0, self.timer_callback)
        self.string_publisher = self.__node.create_publisher(String, '/sim/string', 10)

    def timer_callback(self):
        # self.node.get_logger().info('INFO')
        self.string_publisher.publish(String(data=f' --- {str(self.properties)} --- '))

    def step(self):
        # print('step')
        rclpy.spin_once(self.__node, timeout_sec=0)
