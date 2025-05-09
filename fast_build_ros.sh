#!/bin/bash
cd ~/Documents/webots/ros2_ws
colcon build --packages-select tongji_webot_bringup
source install/setup.bash

ros2 launch tongji_webot_bringup sim_with_driver.launch.py


# 静态建图相关
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=true
# 新开一个终端
source install/setup.bash
mkdir -p /home/jerry/Documents/webots/ros2_ws/install/tongji_webot_bringup/lib/tongji_webot_bringup
ln -s /home/jerry/Documents/webots/ros2_ws/install/tongji_webot_bringup/lib/tongji_webot_bringup/random_explorer.py /home/jerry/Documents/webots/ros2_ws/install/tongji_webot_bringup/lib/tongji_webot_bringup/random_explorer
ros2 run tongji_webot_bringup random_explorer