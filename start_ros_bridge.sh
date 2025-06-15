#!/bin/bash

# ROS Bridge 启动脚本
# 用于启动ROS Bridge WebSocket服务器，使Web前端能够与ROS2通信

echo "启动ROS2 Bridge WebSocket服务器..."

# 检查ROS2环境是否已设置
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置。请先source ROS2 setup文件。"
    echo "例如: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查rosbridge_server是否已安装
if ! ros2 pkg list | grep -q rosbridge_server; then
    echo "错误: rosbridge_server 未安装。"
    echo "请运行: sudo apt install ros-$ROS_DISTRO-rosbridge-server"
    exit 1
fi

# 启动rosbridge_server
echo "正在启动rosbridge_server..."
echo "WebSocket地址将为: ws://localhost:9090"
echo "按Ctrl+C停止服务"

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
