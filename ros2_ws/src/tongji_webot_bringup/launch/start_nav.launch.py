from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
        period=3.0,  # 延迟 3 秒启动 Nav2，等 Webots clock 和 TF 建立
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

    return LaunchDescription([
        sim_launch,
        nav2_launch
    ])
