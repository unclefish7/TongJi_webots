cmake_minimum_required(VERSION 3.5)
project(auto_explorer)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# 安装 Python 可执行节点
install(PROGRAMS
  auto_explorer/auto_explorer.py
  auto_explorer/semantic_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
