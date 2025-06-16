#!/bin/bash

# 安装Python依赖
echo "安装Python依赖..."
pip3 install -r requirements.txt

# 编译ROS 2包
echo "编译ROS 2包..."
cd /home/jerry/Documents/webots/ros2_ws
colcon build --packages-select ros2_bridge

# 启动节点（使用launch文件）
echo "启动nav_result_bridge节点（使用launch文件）..."
source install/setup.bash
ros2 launch ros2_bridge ros2_bridge.launch.py