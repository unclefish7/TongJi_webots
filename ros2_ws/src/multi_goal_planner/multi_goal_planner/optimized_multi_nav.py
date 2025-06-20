#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json
import os
import math
import itertools
import tf2_ros
from tf2_ros import TransformException
from ament_index_python.packages import get_package_share_directory
import requests  # 添加HTTP请求库


class OptimizedMultiNav(Node):
    def __init__(self):
        super().__init__('optimized_multi_nav')
        
        # API端点配置
        self.API_BASE_URL = "http://localhost:8000"
        self.ARRIVED_API_ENDPOINT = f"{self.API_BASE_URL}/api/tasks/robot/arrived"
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            '/multi_nav_command',
            self.command_callback,
            10
        )
        
        # 创建next订阅者
        self.next_subscription = self.create_subscription(
            String,
            '/next',
            self.next_callback,
            10
        )
        
        # 创建next发布者
        self.next_publisher = self.create_publisher(String, '/next', 10)
        
        # 创建 TF 缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建导航动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 加载地图数据
        self.load_map_data()
        
        # 当前导航状态
        self.current_goals = []
        self.current_goal_index = 0
        self.is_navigating = False
        self.waiting_for_next = False  # 标记是否在等待next信号
        self.current_goal_handle = None  # 保存当前的目标句柄用于取消
        self.task_cancelled = False  # 标记任务是否被取消
        
        self.get_logger().info('Optimized Multi Navigation node initialized')
        self.get_logger().info('Using TF to get current robot position')
        self.get_logger().info('Waiting for /next topic to start navigation...')

    def next_callback(self, msg):
        """处理next信号，继续到下一个目标点"""
        self.get_logger().info(f'Received next signal: {msg.data}')
        self.get_logger().info(f'Current state - waiting_for_next: {self.waiting_for_next}, goals available: {len(self.current_goals)}, current_index: {self.current_goal_index}')
        
        if msg.data == "start" or msg.data == "continue":
            if self.waiting_for_next and self.current_goals and self.current_goal_index < len(self.current_goals):
                self.get_logger().info(f'Received next signal, continuing to goal {self.current_goal_index + 1}/{len(self.current_goals)}...')
                self.waiting_for_next = False
                self.navigate_to_next_goal()
            else:
                reason = []
                if not self.waiting_for_next:
                    reason.append("not waiting for next")
                if not self.current_goals:
                    reason.append("no goals available")
                if self.current_goal_index >= len(self.current_goals):
                    reason.append("all goals completed")
                self.get_logger().warn(f'Received next signal but cannot continue: {", ".join(reason)}')
        else:
            self.get_logger().warn(f'Unrecognized next command: {msg.data}')

    def cancel_current_navigation(self):
        """取消当前的导航任务"""
        try:
            if self.current_goal_handle is not None:
                self.get_logger().info('Canceling current navigation goal')
                cancel_future = self.current_goal_handle.cancel_goal_async()
                # 不等待取消结果，直接继续
                self.current_goal_handle = None
            
            # 重置导航状态
            self.is_navigating = False
            self.waiting_for_next = False
            # 标记当前任务已被取消，避免处理取消结果
            self.task_cancelled = True
            
        except Exception as e:
            self.get_logger().warn(f'Error canceling navigation: {str(e)}')
            # 即使取消失败，也要重置状态
            self.is_navigating = False
            self.waiting_for_next = False
            self.current_goal_handle = None
            self.task_cancelled = True

    def get_current_position(self):
        """使用TF获取机器人当前位置"""
        try:
            # 查询从 map 到 base_link 的变换
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            current_position = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y
            }
            
            return current_position
            
        except TransformException as e:
            self.get_logger().warn(f'Failed to get current position: {e}')
            return None

    def load_map_data(self):
        """从 JSON 文件加载地图数据"""
        try:
            package_share_directory = get_package_share_directory('multi_goal_planner')
            map_file_path = os.path.join(package_share_directory, 'map', 'map.json')
            
            with open(map_file_path, 'r', encoding='utf-8') as f:
                self.map_data = json.load(f)
            
            self.get_logger().info(f'Loaded {len(self.map_data)} locations from map.json')
            for location in self.map_data.keys():
                self.get_logger().info(f'  - {location}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load map data: {str(e)}')
            self.map_data = {}

    def optimize_path(self, goal_names):
        """使用TSP算法优化路径，以当前位置为起点"""
        if len(goal_names) <= 1:
            return goal_names
        
        # 获取当前位置
        current_position = self.get_current_position()
        if current_position is None:
            self.get_logger().warn('Current position not available, using original order')
            return goal_names
        
        # 构建距离矩阵：包含当前位置(索引0)和所有目标点
        n = len(goal_names) + 1  # +1 for current position
        distance_matrix = [[0.0 for _ in range(n)] for _ in range(n)]
        
        # 计算当前位置到所有目标点的距离
        for i in range(1, n):  # 从1开始，0是当前位置
            goal_name = goal_names[i-1]
            distance_matrix[0][i] = self.calculate_distance_from_position(current_position, goal_name)
            distance_matrix[i][0] = distance_matrix[0][i]  # 对称距离
        
        # 计算目标点之间的距离
        for i in range(1, n):
            for j in range(1, n):
                if i != j:
                    distance_matrix[i][j] = self.calculate_distance(goal_names[i-1], goal_names[j-1])
        
        # 求解TSP，固定起点为当前位置(索引0)
        best_path = self.solve_tsp_with_fixed_start(goal_names, distance_matrix)
        
        return best_path

    def calculate_distance_from_position(self, position, location_name):
        """计算给定位置到地图中某个点的距离"""
        target_pos = self.map_data[location_name]['pose']['position']
        
        dx = position['x'] - target_pos['x']
        dy = position['y'] - target_pos['y']
        
        return math.sqrt(dx * dx + dy * dy)

    def calculate_distance(self, location1, location2):
        """计算两个位置之间的欧氏距离"""
        pos1 = self.map_data[location1]['pose']['position']
        pos2 = self.map_data[location2]['pose']['position']
        
        dx = pos1['x'] - pos2['x']
        dy = pos1['y'] - pos2['y']
        
        return math.sqrt(dx * dx + dy * dy)

    def solve_tsp_with_fixed_start(self, goal_names, distance_matrix):
        """固定起点的TSP求解"""
        n = len(goal_names)
        
        if n <= 7:  # 限制在7个目标点以内使用暴力法（加上起点共8个点）
            return self.solve_tsp_brute_force_fixed_start(goal_names, distance_matrix)
        else:
            return self.solve_tsp_nearest_neighbor_fixed_start(goal_names, distance_matrix)

    def solve_tsp_brute_force_fixed_start(self, goal_names, distance_matrix):
        """暴力求解TSP，固定起点为当前位置"""
        n = len(goal_names)
        min_distance = float('inf')
        best_path = goal_names[:]
        
        # 生成目标点的所有排列（不包括起点）
        goal_indices = list(range(1, n + 1))  # 1到n，对应goal_names的索引
        
        for perm in itertools.permutations(goal_indices):
            total_distance = 0.0
            
            # 从当前位置(索引0)到第一个目标点
            total_distance += distance_matrix[0][perm[0]]
            
            # 目标点之间的距离
            for i in range(len(perm) - 1):
                total_distance += distance_matrix[perm[i]][perm[i + 1]]
            
            if total_distance < min_distance:
                min_distance = total_distance
                best_path = [goal_names[i - 1] for i in perm]  # 转换回goal_names索引
        
        self.get_logger().info(f'TSP solution (from current position) distance: {min_distance:.2f}m')
        return best_path

    def solve_tsp_nearest_neighbor_fixed_start(self, goal_names, distance_matrix):
        """最近邻启发式求解TSP，固定起点为当前位置"""
        n = len(goal_names)
        unvisited = set(range(1, n + 1))  # 目标点索引：1到n
        current = 0  # 从当前位置开始
        path = []
        total_distance = 0.0
        
        while unvisited:
            nearest = min(unvisited, key=lambda x: distance_matrix[current][x])
            total_distance += distance_matrix[current][nearest]
            current = nearest
            path.append(current)
            unvisited.remove(current)
        
        result = [goal_names[i - 1] for i in path]  # 转换回goal_names索引
        self.get_logger().info(f'Nearest neighbor solution (from current position) distance: {total_distance:.2f}m')
        return result

    def command_callback(self, msg):
        """处理多目标导航命令"""
        try:
            goal_names = json.loads(msg.data)
            
            if not isinstance(goal_names, list):
                self.get_logger().error('Command must be a JSON list of location names')
                return
            
            if len(goal_names) == 0:
                self.get_logger().warn('Received empty goal list')
                return
            
            # 检查是否能获取当前位置
            current_position = self.get_current_position()
            if current_position is None:
                self.get_logger().warn('Current position not available yet, please wait...')
                return
            
            self.get_logger().info(f'Received navigation command: {goal_names}')
            self.get_logger().info(f'Current position: x={current_position["x"]:.2f}, y={current_position["y"]:.2f}')
            
            # 验证所有目标点是否存在
            valid_goals = []
            for goal_name in goal_names:
                if goal_name in self.map_data:
                    valid_goals.append(goal_name)
                else:
                    self.get_logger().warn(f'Unknown location: {goal_name}')
            
            if len(valid_goals) == 0:
                self.get_logger().error('No valid goals found')
                return
            
            # 如果正在导航或等待next信号，取消当前任务并切换到新任务
            if self.is_navigating or self.waiting_for_next:
                self.get_logger().info('Switching to new navigation task, canceling current task')
                self.cancel_current_navigation()
            
            # 重置状态
            self.waiting_for_next = False
            self.is_navigating = False
            self.task_cancelled = False  # 重置取消标记
            
            # 计算最优路径（以当前位置为起点）
            optimized_goals = self.optimize_path(valid_goals)
            self.get_logger().info(f'Optimized path from current position: {optimized_goals}')
            
            # 准备导航
            self.current_goals = optimized_goals
            self.current_goal_index = 0
            self.waiting_for_next = True
            
            # 等待手动发送next信号开始导航
            first_goal_name = self.current_goals[0] if self.current_goals else "unknown"
            self.get_logger().info(f'Task ready! Waiting for /next signal to start navigation to: {first_goal_name}')
            self.get_logger().info('Please publish: ros2 topic pub --once /next std_msgs/String "data: \'start\'"')
            
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON format in command')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def navigate_to_next_goal(self):
        """导航到下一个目标点"""
        if self.current_goal_index >= len(self.current_goals):
            self.get_logger().info('All goals completed!')
            self.is_navigating = False
            self.waiting_for_next = False
            return
        
        goal_name = self.current_goals[self.current_goal_index]
        
        if goal_name not in self.map_data:
            self.get_logger().error(f'Goal {goal_name} not found in map data')
            self.current_goal_index += 1
            self.navigate_to_next_goal()
            return
        
        # 等待导航服务可用
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return
        
        # 创建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(goal_name)
        
        self.get_logger().info(f'Navigating to goal {self.current_goal_index + 1}/{len(self.current_goals)}: {goal_name}')
        
        # 发送导航目标
        self.is_navigating = True
        self.waiting_for_next = False
        self.task_cancelled = False  # 重置取消标记
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_response_callback)

    def create_pose_stamped(self, location_name):
        """从地图数据创建 PoseStamped 消息"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        location_data = self.map_data[location_name]['pose']
        
        # 设置位置
        pose_stamped.pose.position.x = location_data['position']['x']
        pose_stamped.pose.position.y = location_data['position']['y']
        pose_stamped.pose.position.z = location_data['position']['z']
        
        # 设置方向
        pose_stamped.pose.orientation.x = location_data['orientation']['x']
        pose_stamped.pose.orientation.y = location_data['orientation']['y']
        pose_stamped.pose.orientation.z = location_data['orientation']['z']
        pose_stamped.pose.orientation.w = location_data['orientation']['w']
        
        return pose_stamped

    def navigation_response_callback(self, future):
        """处理导航响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.is_navigating = False
            self.current_goal_handle = None
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # 保存目标句柄用于后续取消操作
        self.current_goal_handle = goal_handle
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def notify_backend_arrived(self):
        """通知后端API机器人已到达目标点"""
        try:
            self.get_logger().info(f'Notifying backend API: robot arrived at {self.current_goals[self.current_goal_index-1]}')
            
            # 发送POST请求到后端API
            response = requests.post(
                self.ARRIVED_API_ENDPOINT,
                json={},  # 空JSON体，API只需要知道机器人到达
                headers={'Content-Type': 'application/json'},
                timeout=5.0
            )
            
            if response.status_code == 200:
                self.get_logger().info('Successfully notified backend API')
                data = response.json()
                self.get_logger().info(f'Backend response: {data.get("message", "No message")}')
                return True
            else:
                self.get_logger().error(f'Failed to notify backend API: HTTP {response.status_code}')
                self.get_logger().error(f'Response: {response.text[:100]}')
                return False
                
        except requests.exceptions.ConnectionError:
            self.get_logger().error('Connection error: Backend API not available')
            return False
        except requests.exceptions.Timeout:
            self.get_logger().error('Timeout error: Backend API took too long to respond')
            return False
        except Exception as e:
            self.get_logger().error(f'Error notifying backend API: {str(e)}')
            return False

    def navigation_result_callback(self, future):
        """处理导航结果"""
        # 如果任务已被取消，忽略这个结果回调
        if self.task_cancelled:
            self.get_logger().info('Ignoring navigation result - task was cancelled')
            return
        
        try:
            result_wrapper = future.result()
            status = result_wrapper.status
            result = result_wrapper.result
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                goal_name = self.current_goals[self.current_goal_index]
                self.get_logger().info(f'Successfully reached: {goal_name}')
                
                # 移动到下一个目标索引
                self.current_goal_index += 1
                self.is_navigating = False
                self.current_goal_handle = None  # 清理目标句柄
                
                self.get_logger().info(f'Updated state - current_index: {self.current_goal_index}, total_goals: {len(self.current_goals)}')
                
                # 通知后端API机器人已到达
                api_notified = self.notify_backend_arrived()
                if not api_notified:
                    self.get_logger().warn('Backend API notification failed, but continuing with navigation process')
                
                # 检查是否还有更多目标
                if self.current_goal_index < len(self.current_goals):
                    next_goal_name = self.current_goals[self.current_goal_index]
                    self.get_logger().info(f'Reached {goal_name}. Waiting for /next signal to continue to: {next_goal_name}')
                    self.waiting_for_next = True
                    self.get_logger().info(f'Set waiting_for_next = True, please send next signal')
                else:
                    self.get_logger().info('All goals completed!')
                    self.waiting_for_next = False
                    
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info('Navigation goal was cancelled')
                self.is_navigating = False
                self.current_goal_handle = None
                
            else:
                goal_name = self.current_goals[self.current_goal_index] if self.current_goal_index < len(self.current_goals) else "unknown"
                self.get_logger().warn(f'Navigation failed to reach: {goal_name} (Status: {status})')
                self.is_navigating = False
                self.current_goal_handle = None
                
        except Exception as e:
            self.get_logger().error(f'Error processing navigation result: {str(e)}')
            self.is_navigating = False
            self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OptimizedMultiNav()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
