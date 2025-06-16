#!/usr/bin/env python3

import json
import requests
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class NavResultBridge(Node):
    """
    ROS 2 节点，监听导航结果并发送HTTP通知
    """
    
    def __init__(self):
        super().__init__('nav_result_bridge')
        
        # 声明并获取参数
        self.declare_parameter('api_url', 'http://localhost:8000/api/robot/arrived')
        self.api_url = self.get_parameter('api_url').get_parameter_value().string_value
        
        # 创建订阅者，监听导航结果
        self.subscription = self.create_subscription(
            NavigateToPose.Result,
            '/navigate_to_pose/result',
            self.nav_result_callback,
            10
        )
        
        self.get_logger().info(f'Nav Result Bridge 节点已启动')
        self.get_logger().info(f'正在监听话题: /navigate_to_pose/result')
        self.get_logger().info(f'API URL: {self.api_url}')
    
    def nav_result_callback(self, msg):
        """
        处理导航结果消息的回调函数
        
        Args:
            msg: NavigateToPose.Result 类型的消息
        """
        self.get_logger().info(f'收到导航结果，error_code: {msg.result.error_code}')
        
        # 检查导航是否成功（error_code == 0 表示成功）
        if msg.result.error_code == 0:
            self.get_logger().info('导航成功，发送HTTP通知')
            self.send_arrival_notification()
        else:
            self.get_logger().warn(f'导航失败，error_code: {msg.result.error_code}')
    
    def send_arrival_notification(self):
        """
        发送到达通知到HTTP API
        """
        try:
            # 准备JSON数据
            data = {"status": "arrived"}
            
            # 发送POST请求
            response = requests.post(
                self.api_url,
                json=data,
                headers={'Content-Type': 'application/json'},
                timeout=5.0  # 5秒超时
            )
            
            # 检查响应状态
            if response.status_code == 200:
                self.get_logger().info(f'成功发送到达通知到API: {self.api_url}')
            else:
                self.get_logger().error(f'API响应错误，状态码: {response.status_code}')
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'HTTP请求失败: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'发送通知时发生未知错误: {str(e)}')


def main(args=None):
    """
    主函数
    """
    # 初始化ROS 2
    rclpy.init(args=args)
    
    try:
        # 创建节点
        nav_bridge = NavResultBridge()
        
        # 运行节点
        rclpy.spin(nav_bridge)
        
    except KeyboardInterrupt:
        print('\n收到中断信号，正在关闭节点...')
    except Exception as e:
        print(f'节点运行时发生错误: {str(e)}')
    finally:
        # 清理资源
        if 'nav_bridge' in locals():
            nav_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
