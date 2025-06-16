#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成launch描述，用于启动bridge节点
    """
    
    # 声明launch参数
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
    http_port = LaunchConfiguration('http_port')
    
    # 创建nav_result_bridge节点
    nav_result_bridge_node = Node(
        package='ros2_bridge',
        executable='nav_result_bridge.py',
        name='nav_result_bridge',
        output='screen',
        parameters=[
            {'api_url': api_url}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            # 如果需要重映射话题，可以在这里添加
            # ('/navigate_to_pose/result', '/custom_nav_result')
        ]
    )
    
    # 创建topic_bridge_node节点
    topic_bridge_node = Node(
        package='ros2_bridge',
        executable='publish_msg.py',
        name='topic_bridge_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            'Starting ROS 2 Bridge nodes...\n',
            'Nav API URL: ', api_url, '\n',
            'HTTP Server Port: ', http_port, '\n',
            'Log Level: ', log_level
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        api_url_arg,
        log_level_arg,
        http_port_arg,
        
        # 启动信息
        start_info,
        
        # 节点
        nav_result_bridge_node,
        topic_bridge_node,
    ])