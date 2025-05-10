#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.3, self.control_loop)

        # === 参数设置 ===
        self.obstacle_threshold = 0.6     # 障碍物判断距离
        self.forward_speed = 0.3          # 前进速度
        self.backward_speed = 0.01        # 后退速度
        self.turn_speed = 1.0             # 固定右转角速度
        self.back_duration = 1            # 后退持续计时器次数
        self.turn_duration = 2            # 转向持续时间

        # === 状态变量 ===
        self.obstacle_ahead = False
        self.state = 'forward'
        self.state_timer = 0

    def scan_callback(self, msg):
        # 检查正前方 ±30°
        n = len(msg.ranges)
        center = n // 2
        front_ranges = msg.ranges[center - 30 : center + 30]
        valid = [r for r in front_ranges if 0.05 < r < msg.range_max]
        self.obstacle_ahead = any(r < self.obstacle_threshold for r in valid)

    def control_loop(self):
        twist = Twist()

        if self.state == 'forward':
            if self.obstacle_ahead:
                self.get_logger().info('遇到障碍，开始后退')
                self.state = 'backward'
                self.state_timer = self.back_duration
            else:
                twist.linear.x = self.forward_speed

        elif self.state == 'backward':
            twist.linear.x = self.backward_speed
            self.state_timer -= 1
            if self.state_timer <= 0:
                self.get_logger().info('后退完成，右转脱困')
                self.state = 'turning'
                self.state_timer = self.turn_duration

        elif self.state == 'turning':
            twist.angular.z = -self.turn_speed  # 固定右转
            self.state_timer -= 1
            if self.state_timer <= 0:
                self.get_logger().info('转向完成，恢复前进')
                self.state = 'forward'

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
