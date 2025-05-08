#!/bin/bash
cd ~/Documents/webots/ros2_ws
colcon build --packages-select tongji_webot_bringup
source install/setup.bash

ros2 launch tongji_webot_bringup sim_with_driver.launch.py