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

## 启动多目标任务调度
> ros2 launch multi_goal_planner optimized_multi_nav.launch.py

### 发布多目标导航示例
```bash
# ✅ 1. 小会议室 → 经理室
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小会议室\", \"经理室\"]'"

# ✅ 2. 小办公区 → 前台 → 大办公区
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小办公区\", \"前台\", \"大办公区\"]'"

# ✅ 3. 财务处 → 等候处 → 小会议室（观察是否起点不是第一个）
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"财务处\", \"等候处\", \"小会议室\"]'"

# ✅ 4. 大测试：经理室 → 财务处 → 小办公区 → 小会议室 → 等候处
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"经理室\", \"财务处\", \"小办公区\", \"小会议室\", \"等候处\"]'"

# ⚠️ 5. 含错误点名（“未知点”）测试
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小会议室\", \"不存在的点\", \"经理室\"]'"

```