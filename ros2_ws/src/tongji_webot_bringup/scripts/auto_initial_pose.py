#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from tf2_ros import TransformException

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.published = False

    def timer_callback(self):
        if self.published:
            return

        try:
            # ✅ 从 odom → base_link 查询机器人在 odom 坐标系下的位置
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time()
            )

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'   # ✅ 注意：这里必须设为 map，AMCL 需要这个

            # 把 odom 下的位置作为 map 下的估计位置发出去
            pose_msg.pose.pose.position.x = transform.transform.translation.x
            pose_msg.pose.pose.position.y = transform.transform.translation.y
            pose_msg.pose.pose.position.z = 0.0
            pose_msg.pose.pose.orientation = transform.transform.rotation

            # 设置协方差
            pose_msg.pose.covariance[0] = 0.25
            pose_msg.pose.covariance[7] = 0.25
            pose_msg.pose.covariance[35] = 0.0685

            self.publisher.publish(pose_msg)
            self.get_logger().info('✅ Published initial pose from odom → base_link')
            self.published = True

        except TransformException as e:
            self.get_logger().warn(f'TF not ready: {e}')

def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
