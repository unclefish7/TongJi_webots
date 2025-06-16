#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成launch描述，用于启动nav_result_bridge节点（自定义配置版本）
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
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='节点命名空间'
    )
    
    nav_topic_arg = DeclareLaunchArgument(
        'nav_result_topic',
        default_value='/navigate_to_pose/result',
        description='导航结果话题名称'
    )
    
    # 获取launch配置
    api_url = LaunchConfiguration('api_url')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    nav_topic = LaunchConfiguration('nav_result_topic')
    
    # 创建nav_result_bridge节点
    nav_result_bridge_node = Node(
        package='ros2_bridge',
        executable='nav_result_bridge.py',
        name='nav_result_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            {'api_url': api_url}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/navigate_to_pose/result', nav_topic)
        ]
    )
    
    return LaunchDescription([
        # 参数声明
        api_url_arg,
        log_level_arg,
        namespace_arg,
        nav_topic_arg,
        
        # 节点
        nav_result_bridge_node,
    ])
