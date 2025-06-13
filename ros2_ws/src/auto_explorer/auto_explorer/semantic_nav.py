#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
import json
import os

class SemanticNavigator(Node):
    def __init__(self):
        super().__init__('semantic_navigator')
        # 使用 share directory 获取map.json路径
        map_file = os.path.join(
            get_package_share_directory('auto_explorer'),
            'map',
            'map.json'
        )
        # 读取语义地图
        with open(map_file, 'r', encoding='utf-8') as f:
            self.semantic_map = json.load(f)
        self.get_logger().info(f"Loaded semantic map: {list(self.semantic_map.keys())}")
        # 订阅/nav_command
        self.subscription = self.create_subscription(
            String,
            '/nav_command',
            self.nav_command_callback,
            10
        )
        # 创建ActionClient
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def nav_command_callback(self, msg):
        room = msg.data.strip()
        if room not in self.semantic_map:
            self.get_logger().warn(f"目标房间 '{room}' 不在语义地图中")
            return
        pose_dict = self.semantic_map[room]['pose']
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = pose_dict['position']['x']
        pose_msg.pose.position.y = pose_dict['position']['y']
        pose_msg.pose.position.z = pose_dict['position']['z']
        pose_msg.pose.orientation.x = pose_dict['orientation']['x']
        pose_msg.pose.orientation.y = pose_dict['orientation']['y']
        pose_msg.pose.orientation.z = pose_dict['orientation']['z']
        pose_msg.pose.orientation.w = pose_dict['orientation']['w']
        self.get_logger().info(f"开始导航到: {room}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
        self.get_logger().info('导航目标已接受，正在导航...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'导航完成，状态码: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
