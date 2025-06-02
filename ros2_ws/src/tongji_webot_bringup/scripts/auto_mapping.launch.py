from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    tongji_pkg = FindPackageShare('tongji_webot_bringup')
    nav2_pkg = FindPackageShare('nav2_bringup')
    slam_pkg = FindPackageShare('slam_toolbox')

    return LaunchDescription([
        # 1. 启动 Webots 仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([tongji_pkg, 'launch', 'sim_with_driver.launch.py'])
            ])
        ),

        # 2. 启动 SLAM Toolbox（建图）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([slam_pkg, 'launch', 'online_async_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        ),

        # 3. 启动 Nav2 栈（路径规划）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_pkg, 'launch', 'bringup_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        ),

        # 4. 启动 Explore Lite 自动探索
        Node(
            package='nav2_explore',
            executable='explore',
            name='explore',
            output='screen',
            parameters=[{
                'planner_frequency': 1.0,
                'progress_timeout': 30.0,
                'visualize': True
            }]
        ),
    ])
