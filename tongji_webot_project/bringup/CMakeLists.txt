cmake_minimum_required(VERSION 3.10)
project(tongji_webot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch resource worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
