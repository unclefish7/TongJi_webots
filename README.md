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

## 🔄 新版本多目标任务调度（带等待机制）

### 功能说明
新版本的多目标导航系统添加了等待确认机制：
- 接收到新任务后，系统会计算最优路径但不会自动开始导航
- 需要手动发送第一个 `/next` 信号来启动整个导航任务
- 机器人到达每个目标点后会停下来，等待接收 `/next` 话题的确认信号
- 只有收到 `/next` 信号后，才会继续前往下一个目标点
- 如果有新的 `/multi_nav_command` 任务进来，会丢弃当前任务直接执行新任务

### 使用步骤
```bash
# 1. 启动多目标导航节点
ros2 launch multi_goal_planner optimized_multi_nav.launch.py

# 2. 发送多目标任务（系统会计算路径但不会自动开始导航）
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小会议室\", \"经理室\"]'"

# 3. 手动发送第一个next信号开始导航到第一个目标
ros2 topic pub --once /next std_msgs/String "data: 'start'"

# 4. 机器人到达第一个目标后，发送next信号继续到下一个目标
ros2 topic pub --once /next std_msgs/String "data: 'continue'"

# 5. 重复步骤4直到完成所有目标点
```

### 测试案例
```bash
# ✅ 简单测试：小会议室 → 经理室（需要发送2次/next信号）
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小会议室\", \"经理室\"]'"
# 手动启动: ros2 topic pub --once /next std_msgs/String "data: 'start'"
# 到达小会议室后: ros2 topic pub --once /next std_msgs/String "data: 'continue'"

# ✅ 三点测试：小办公区 → 前台 → 大办公区（需要发送3次/next信号）  
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小办公区\", \"前台\", \"大办公区\"]'"
# 手动启动: ros2 topic pub --once /next std_msgs/String "data: 'start'"
# 到达小办公区后: ros2 topic pub --once /next std_msgs/String "data: 'continue'"
# 到达前台后: ros2 topic pub --once /next std_msgs/String "data: 'continue'"

# ✅ 复杂测试：经理室 → 财务处 → 小办公区 → 小会议室 → 等候处（需要发送5次/next信号）
ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"经理室\", \"财务处\", \"小办公区\", \"小会议室\", \"等候处\"]'"
# 手动启动: ros2 topic pub --once /next std_msgs/String "data: 'start'"
# 然后每到达一个目标点后都需要发送continue信号
```