cmake_minimum_required(VERSION 3.10)
project(tongji_webot_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(clock_publisher src/clock_publisher.cpp)
ament_target_dependencies(clock_publisher rclcpp rosgraph_msgs)

install(
  DIRECTORY launch resource worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(TARGETS
  clock_publisher
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
