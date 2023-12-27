import os
import pathlib
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.utils import controller_url_prefix


def get_ros2_nodes(*args):
    package_dir = get_package_share_directory('webots_ros2_pioneer3at')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)    
    world = LaunchConfiguration('world')

    # Launch Webots
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )


    # Foorprint and robot state publishers
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # diff drive controller and joint state broadcaster spawners
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix='python.exe' if os.name == 'nt' else '',
        arguments=['diffdrive_controller'] +  ['--controller-manager-timeout', '50'],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    ) 
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix='python.exe' if os.name == 'nt' else '',
        arguments=['joint_state_broadcaster'] +  ['--controller-manager-timeout', '50'],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    # ROS 2 driver for Webots/Robot
    ros2_control = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    robot_driver = WebotsController(
        robot_name='Pioneer3at',
        parameters=[
            {
                'robot_description': pathlib.Path(os.path.join(package_dir, 'resource', 'pioneer_webots.urdf')).read_text(),
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True
            },
            ros2_control
        ],
        remappings=[('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')],
        respawn=True
    )


    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots._supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    # This action will kill all nodes once the Webots simulation has exited
    exit_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[
                launch.actions.UnregisterEventHandler(
                    event_handler=reset_handler.event_handler
                ),
                launch.actions.EmitEvent(event=launch.events.Shutdown())
            ],
        )
    )

    # Launch other processes and actions
    robot_process = Node(
        package='webots_ros2_pioneer3at',
        executable='webots_ros2_pioneer3at',
        name='webots_ros2_pioneer3at',
        output='screen',
    )

    return [
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        robot_driver,
        reset_handler,
        exit_handler,
        robot_process,
    ]


def generate_launch_description():
    world_argument = DeclareLaunchArgument(
        'world',
        default_value='farm.wbt',
        description='Choose one of the world files from `/world` directory'
    )

    return LaunchDescription(
        [world_argument] +
        get_ros2_nodes()
    )
