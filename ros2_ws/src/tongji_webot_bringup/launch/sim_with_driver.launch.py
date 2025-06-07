#!/usr/bin/env python

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('tongji_webot_bringup')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'office_simple.wbt'),
        mode=mode,
        ros2_supervisor=True
    )

    keyboard_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',
    )

    map_to_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    velodyne_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--use-timestamp', '0', '0', '0.25', '0', '0', '0', 'base_link', 'velodyne_link']
    )

    # 添加LDS-01激光雷达帧的TF变换
    lds_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'LDS-01']
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'turtlebot.urdf')
    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy'])
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]

    turtlebot_driver = WebotsController(
        robot_name='TurtleBot3Burger',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners
    )

    # ⏱ 延迟启动 Webots 控制器和控制器 spawner，确保 TF 提前发布
    delayed_turtlebot_driver = TimerAction(
        period=5.0,  # 增加延迟时间到5秒
        actions=[turtlebot_driver]
    )
    delayed_waiting_nodes = TimerAction(
        period=6.0,  # 增加延迟时间到6秒
        actions=[waiting_nodes]
    )
    delayed_keyboard_teleop = TimerAction(
        period=7.0,  # 增加延迟时间到7秒
        actions=[keyboard_teleop_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='office_simple.wbt'),
        DeclareLaunchArgument('mode', default_value='realtime'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        webots,
        webots._supervisor,

        # 🚀 优先发布静态 TF（避免 message filter 报错）
        # robot_state_publisher,
        footprint_publisher,
        velodyne_tf_publisher,
        lds_tf_publisher,  # 添加LDS-01激光雷达TF
        map_to_odom_publisher,

        # ⏱ 延迟发布雷达 scan（来自驱动器）
        delayed_turtlebot_driver,
        delayed_waiting_nodes,
        delayed_keyboard_teleop,

        # Webots 退出时自动关闭 ROS
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
