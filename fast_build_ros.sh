#!/bin/bash
cd ~/Documents/webots/ros2_ws
colcon build
source install/setup.bash

ros2 launch tongji_webot_bringup sim_with_driver.launch.py


# 静态建图相关
ros2 launch auto_explorer auto_explorer.launch.py
ros2 launch auto_explorer cartographer.launch.py

# 保存地图
ros2 run nav2_map_server map_saver_cli -f my_office_map

# clear map
ros2 service call /slam_toolbox/clear --std_srvs/srv/Empty

# save map
ros2 run nav2_map_server map_saver_cli -f /home/jerry/Documents/webots/ros2_ws/src/auto_explorer/map/my_office_map

# 运行nav2 bringup
ros2 launch nav2_bringup bringup_launch.py map:=/home/jerry/Documents/webots/ros2_ws/src/auto_explorer/map/my_office_map_2.yaml params_file:=/home/jerry/Documents/webots/ros2_ws/src/navigation2/nav2_bringup/params/nav2_params.yaml


# deprecated
# ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=true
# # 新开一个终端
# source install/setup.bash
# mkdir -p /home/jerry/Documents/webots/ros2_ws/install/tongji_webot_bringup/lib/tongji_webot_bringup
# ln -s /home/jerry/Documents/webots/ros2_ws/install/tongji_webot_bringup/lib/tongji_webot_bringup/random_explorer.py /home/jerry/Documents/webots/ros2_ws/install/tongji_webot_bringup/lib/tongji_webot_bringup/random_explorer
# ros2 run tongji_webot_bringup random_explorer