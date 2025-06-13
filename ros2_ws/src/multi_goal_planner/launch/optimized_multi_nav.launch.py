#!/usr/bin/env python3

# optimized_multi_nav.launch.py

# 本 launch 文件用于启动 multi_goal_planner 包中的主节点 `optimized_multi_nav`，实现以下核心功能：

# 📌 功能描述：
# 该节点支持接收一组语义目标点（例如"经理室"、"财务处"、"小会议室"），
# 并从地图文件（map.json）中获取对应坐标信息，计算多点之间的最优访问顺序（路径最短），
# 然后依次调用 ROS 2 的 `NavigateToPose` 动作接口，控制机器人依序前往这些目标点。

# ✅ 主要特性：
# - 一次性输入多个语义目标点
# - 基于 TSP 路径优化算法（初期为暴力全排列）
# - 自动查表获取语义点位姿（map/map.json）
# - 发起导航指令并在目标完成后继续下一个点
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
