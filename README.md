# webots

## 一键启动webot(webot环境+nav2静态导航+发布initial pose)
> ros2 launch tongji_webot_bringup start_nav.launch.py

## 启动rviz2
> rviz2

## 启动语义导航
> ros2 launch auto_explorer semantic_nav.launch.py

### 发布语义导航指令示例
```bash
ros2 topic pub --once /nav_command std_msgs/String "data: '经理室'"
ros2 topic pub --once /nav_command std_msgs/String "data: '财务处'"
ros2 topic pub --once /nav_command std_msgs/String "data: '等候处'"
ros2 topic pub --once /nav_command std_msgs/String "data: '前台'"
ros2 topic pub --once /nav_command std_msgs/String "data: '休息室'"
ros2 topic pub --once /nav_command std_msgs/String "data: '小办公区'"
ros2 topic pub --once /nav_command std_msgs/String "data: '大办公区'"
ros2 topic pub --once /nav_command std_msgs/String "data: '大会议室'"
ros2 topic pub --once /nav_command std_msgs/String "data: '小会议室'"
```