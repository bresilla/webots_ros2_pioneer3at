from rosgraph_msgs.msg import Clock
import rclpy
import numpy as np
from nav_msgs.msg import Odometry

# spawn an object/box and add to object list
def spawnObject(self, pose=[1, 1], color=[1, 0, 0], name='ball', size=[0.05, 0.05, 0.05], recognition=True):
    root_node = self.getRoot()
    rec_col = f"recognitionColors [ {color[0]} {color[1]} {color[2]} ]" if recognition else ""
    children_field = root_node.getField('children')
    def_name = f"DEF {name} Solid {{ \
                    translation {pose[0]} {pose[1]} {size[2]/2} \
                    children [ \
                        Shape {{ \
                            appearance Appearance {{ \
                                material Material {{ \
                                    diffuseColor {color[0]} {color[1]} {color[2]} \
                                }} \
                            }} \
                            geometry Box {{ \
                                size {size[0]} {size[1]} {size[2]} \
                            }} \
                        }} \
                    ] \
                    {rec_col} \
                    }}"
    children_field.importMFNodeFromString(-1, def_name)
    obj_node = self.getFromDef(name)
    self.objects[name] = obj_node
    return obj_node

class ObjectSpawner:
    # The `init` method is called only once the driver is initialized. You will always get two arguments in the `init` method.
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node. However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.create_node('sim_direct')

        # This will print the parameter from the URDF file. `{ 'parameterExample': 'someValue' }`
        self.__node.get_logger().info('  - properties: ' + str(properties))

        # The robot property allows you to access the standard Webots API.
        # See: https://cyberbotics.com/doc/reference/robot
        self.super = webots_node.robot.getSelf()
        self.robot = webots_node.robot
        self.__node.get_logger().info('  - robot: ' + str(self.robot.getName()))
        self.__node.get_logger().info('  - timestep: ' + str(int(self.robot.getBasicTimeStep())))

        # The robot property allows you to access the Supervisor Webots API only if the robot is a Supervisor.
        # The function "self.__robot.getSupervisor()" will return "true" in case the robot is a Supervisor.
        # See: https://cyberbotics.com/doc/reference/supervisor
        self.__node.get_logger().info('  - supervisor? ' + str(self.robot.getSupervisor()))

        # The robot property also allows you to access the Driver Webots API in case the robot is based on a Car.
        # See: https://cyberbotics.com/doc/automobile/driver-library

        # Create a simple publisher, subscriber and "Clock" variable.
        self.__node.create_subscription(Clock, 'clock', self.__clock_callback, 1)
        self.__publisher = self.__node.create_publisher(Clock, 'custom_clock', 1)
        self.__clock = Clock()

        # other publishers
        self.__state_publisher = self.__node.create_publisher(Odometry, '/rtk/sim/pose', 1)

    def __clock_callback(self, msg):
        self.__clock = msg

    # The `step` method is called at every step.
    def step(self):
        # The self.__node has to be spinned once in order to execute callback functions.
        rclpy.spin_once(self.__node, timeout_sec=0)

        msg = Odometry()
        msg.header.stamp = self.__clock.clock
        msg.header.frame_id = "abs"
        msg.pose.pose.position.x = self.super.getPosition()[0]
        msg.pose.pose.position.y = self.super.getPosition()[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        theta = np.arctan2(self.super.getOrientation()[3], self.super.getOrientation()[0])
        msg.pose.pose.orientation.z = np.sin(theta / 2)
        msg.pose.pose.orientation.w = np.cos(theta / 2)
        self.__state_publisher.publish(msg)

        self.__publisher.publish(self.__clock)