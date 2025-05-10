import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('auto_explorer'),
        'config',
        'slam_toolbox_online_async.yaml'
    )

    return LaunchDescription([
        # 启动 slam_toolbox，带参数文件
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[param_file]
        ),

        # 启动你的自动探索程序
        # Node(
        #     package='auto_explorer',
        #     executable='auto_explorer',
        #     name='auto_explorer',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )
    ])
