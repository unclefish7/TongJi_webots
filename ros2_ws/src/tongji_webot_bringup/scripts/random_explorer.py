#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.timer = self.create_timer(0.2, self.move_forward)
        self.obstacle_detected = False

    def scan_callback(self, msg):
        # 检查前方 +-20度范围是否有障碍
        front_ranges = msg.ranges[len(msg.ranges)//2 - 20 : len(msg.ranges)//2 + 20]
        # 忽略inf和0，避免干扰
        valid_ranges = [r for r in front_ranges if 0.05 < r < 1.0]
        self.obstacle_detected = len(valid_ranges) > 0

    def move_forward(self):
        twist = Twist()
        if self.obstacle_detected:
            # 随机转向角速度（-1.0 到 1.0 rad/s）
            twist.angular.z = random.uniform(-1.0, 1.0)
            twist.linear.x = 0.0
            self.get_logger().info("🚧 Obstacle detected! Turning.")
        else:
            twist.linear.x = 0.2  # 匀速前进
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
