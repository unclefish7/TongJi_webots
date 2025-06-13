from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('auto_explorer')
    node_executable = os.path.join(pkg_share, 'auto_explorer', 'semantic_nav.py')
    return LaunchDescription([
        Node(
            package='auto_explorer',
            executable='semantic_nav.py',
            name='semantic_navigator',
            output='screen'
        )
    ])
