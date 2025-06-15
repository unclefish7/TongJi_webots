#!/usr/bin/env python3

# opt# # ✅ 使用示例：
# # 1. 启动多目标导航节点
# ros2 launch multi_goal_planner optimized_multi_nav.launch.py
# 
# # 2. 发送多目标任务（系统会计算路径但不会自动开始导航）
# ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小会议室\", \"经理室\"]'"
# 
# # 3. 手动发送第一个next信号开始导航到第一个目标
# ros2 topic pub --once /next std_msgs/String "data: 'start'"
# 
# # 4. 机器人到达第一个目标后，发送next信号继续到下一个目标
# ros2 topic pub --once /next std_msgs/String "data: 'continue'"
# 
# # 更多测试案例：
# # ✅ 小办公区 → 前台 → 大办公区（需要发送3次/next信号）
# ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小办公区\", \"前台\", \"大办公区\"]'"
# # 手动启动: ros2 topic pub --once /next std_msgs/String "data: 'start'"
# # 到达小办公区后: ros2 topic pub --once /next std_msgs/String "data: 'continue'"
# # 到达前台后: ros2 topic pub --once /next std_msgs/String "data: 'continue'"av.launch.py

# 本 launch 文件用于启动 multi_goal_planner 包中的主节点 `optimized_multi_nav`，实现以下核心功能：

# 📌 功能描述：
# 该节点支持接收一组语义目标点（例如"经理室"、"财务处"、"小会议室"），
# 并从地图文件（map.json）中获取对应坐标信息，计算多点之间的最优访问顺序（路径最短），
# 然后依次调用 ROS 2 的 `NavigateToPose` 动作接口，控制机器人依序前往这些目标点。
# 
# 🔄 新增等待机制：
# - 接收到新任务后，系统会计算最优路径但不会自动开始导航
# - 需要手动发送第一个 `/next` 信号来启动整个导航任务
# - 机器人到达每个目标点后会停下来，等待接收 `/next` 话题的确认信号
# - 只有收到 `/next` 信号后，才会继续前往下一个目标点
# - 如果有新的 `/multi_nav_command` 任务进来，会丢弃当前任务直接执行新任务

# ✅ 主要特性：
# - 一次性输入多个语义目标点
# - 基于 TSP 路径优化算法（初期为暴力全排列）
# - 自动查表获取语义点位姿（map/map.json）
# - 发起导航指令并在目标完成后等待确认再继续
# - 支持手动控制导航节奏（通过 /next 话题）
# - 可用于多点送货、巡检、路径优化调度任务等场景

# 📎 启动方式：
# 构建工作空间后，在终端运行：
#     ros2 launch multi_goal_planner optimized_multi_nav.launch.py

# # ✅ 1. 小会议室 → 经理室
# ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小会议室\", \"经理室\"]'"

# # ✅ 2. 小办公区 → 前台 → 大办公区
# ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小办公区\", \"前台\", \"大办公区\"]'"

# # ✅ 3. 财务处 → 等候处 → 小会议室（观察是否起点不是第一个）
# ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"财务处\", \"等候处\", \"小会议室\"]'"

# # ✅ 4. 大测试：经理室 → 财务处 → 小办公区 → 小会议室 → 等候处
# ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"经理室\", \"财务处\", \"小办公区\", \"小会议室\", \"等候处\"]'"

# # ⚠️ 5. 含错误点名（“未知点”）测试
# ros2 topic pub --once /multi_nav_command std_msgs/String "data: '[\"小会议室\", \"不存在的点\", \"经理室\"]'"


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_goal_planner',
            executable='optimized_multi_nav.py',
            name='optimized_multi_nav',
            output='screen',
            parameters=[],
            remappings=[
                # 可以在这里添加话题重映射
            ]
        )
    ])
