from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 声明ROS2桥接相关参数
    api_url_arg = DeclareLaunchArgument(
        'api_url',
        default_value='http://localhost:8000/api/robot/arrived',
        description='API服务器的URL地址'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error, fatal)'
    )
    
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8080',
        description='HTTP服务器监听端口'
    )
    
    # 获取launch配置
    api_url = LaunchConfiguration('api_url')
    log_level = LaunchConfiguration('log_level')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tongji_webot_bringup'),
                'launch',
                'sim_with_driver.launch.py'
            ])
        ])
    )

    nav2_launch = TimerAction(
        period=10.0,  # 延迟启动Nav2
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'bringup_launch.py'
                    ])
                ]),
                launch_arguments={
                    'map': '/home/jerry/Documents/webots/ros2_ws/src/auto_explorer/map/my_office_map_2.yaml',
                    'params_file': '/home/jerry/Documents/webots/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml',
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    # 自动发布初始位姿节点，延迟15秒再启动，确保AMCL已经开始订阅
    initial_pose_publisher = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='tongji_webot_bringup',
                executable='auto_initial_pose.py',
                name='auto_initial_pose',
                output='screen'
            )
        ]
    )

    # 多目标导航节点，延迟20秒启动确保导航系统已就绪
    multi_goal_planner_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='multi_goal_planner',
                executable='optimized_multi_nav.py',
                name='optimized_multi_nav',
                output='screen',
                parameters=[],
                arguments=['--ros-args', '--log-level', log_level],
            )
        ]
    )

    # ROS2桥接节点 - HTTP服务器，延迟12秒启动
    topic_bridge_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros2_bridge',
                executable='publish_msg.py',
                name='topic_bridge_node',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
            )
        ]
    )

    # 导航结果桥接节点，延迟12秒启动
    nav_result_bridge_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros2_bridge',
                executable='nav_result_bridge.py',
                name='nav_result_bridge',
                output='screen',
                parameters=[
                    {'api_url': api_url}
                ],
                arguments=['--ros-args', '--log-level', log_level],
            )
        ]
    )

    # 启动信息
    start_info = LogInfo(
        msg=[
            'Starting Unified Navigation System...\n',
            'Components: Simulation + Navigation + Multi-Goal Planning + ROS2 Bridge\n',
            'Nav API URL: ', api_url, '\n',
            'Log Level: ', log_level, '\n',
            'Use: ros2 topic pub --once /multi_nav_command std_msgs/String "data: \'[\"小会议室\", \"经理室\"]\'"'
        ]
    )

    return LaunchDescription([
        # 参数声明
        api_url_arg,
        log_level_arg,
        http_port_arg,
        
        # 启动信息
        start_info,
        
        # 启动顺序：仿真 -> 导航 -> 初始位姿 -> 多目标规划 -> 桥接服务
        sim_launch,
        nav2_launch,
        initial_pose_publisher,
        multi_goal_planner_node,
        topic_bridge_node,
        nav_result_bridge_node
    ])
