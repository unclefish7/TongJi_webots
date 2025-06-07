from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
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

    return LaunchDescription([
        sim_launch,
        nav2_launch,
        initial_pose_publisher
    ])
