cmake_minimum_required(VERSION 3.5)
project(auto_explorer)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# 安装 Python 可执行节点（修复核心问题）
install(PROGRAMS
  auto_explorer/auto_explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME auto_explorer
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
