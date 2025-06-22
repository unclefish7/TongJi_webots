# 增强任务调度系统指南

## 概述

本增强调度系统实现了智能的任务优先级计算，结合了地理位置、等待时间、超时历史等多个因素，实现更合理的任务调度。

## 系统架构

### 1. 后端 (FastAPI + task_service.py)
- **职责**: 数据收集和传递
- **功能**: 
  - 收集任务等待时间、超时历史等信息
  - 将完整的任务详细信息发送给ROS2节点
  - 接收ROS2返回的优化结果

### 2. ROS2节点 (optimized_multi_nav.py)
- **职责**: 权重计算和路径优化
- **功能**:
  - 接收任务详细信息
  - 计算综合优先级权重
  - 执行增强TSP算法
  - 返回优化后的任务顺序和优先级

## 数据流程

### 1. 任务创建阶段
```python
# 后端保存原始安全等级
task_data = {
    "security_level": "L2",
    "original_security_level": "L2",  # 用于跟踪降级历史
    # ...其他字段
}
```

### 2. 队列启动阶段
```python
# 后端构建任务详细信息
task_detail = {
    "task_id": "T20241222123456ABC123",
    "location_name": "办公室A",
    "task_type": "send",
    "waiting_time_seconds": 1800,  # 等待30分钟
    "has_timeout_history": True,   # 曾经降级过
    "original_security_level": "L2",
    "security_level": "L1"         # 当前级别（降级后）
}

# 发送到ROS2的 /enhanced_multi_nav_command 话题
{
    "tasks": [task_detail1, task_detail2, ...],
    "timestamp": "2024-12-22T12:34:56",
    "scheduler_version": "enhanced_v1.0"
}
```

### 3. ROS2权重计算阶段
```python
# 权重计算公式
final_cost = distance_cost + type_cost - waiting_advantage - timeout_advantage - level_advantage

# 各项权重
- distance_cost: 地理距离成本
- waiting_advantage: 等待时间优势（等待越久越优先）
- timeout_advantage: 超时历史优势（有超时历史的优先）
- type_cost: 任务类型成本（call任务优先级稍低）
- level_advantage: 原始安全等级优势（原始等级越高越优先）
```

### 4. 优化结果返回
```python
# ROS2返回给后端的格式
{
    "optimized_tasks": [
        {
            "task_id": "T20241222123456ABC123",
            "final_priority": 8.5,
            "distance_from_current": 15.2,
            "optimization_reason": "timeout_history_long_waiting"
        }
    ]
}
```

## 权重计算详解

### 权重因子配置
```python
WAITING_TIME_WEIGHT = 0.5      # 等待时间权重
TIMEOUT_HISTORY_WEIGHT = 10.0  # 超时历史权重
DISTANCE_WEIGHT = 1.0          # 距离权重
TASK_TYPE_WEIGHT = 2.0         # 任务类型权重
LEVEL_WEIGHT = 5.0             # 原始安全等级权重
```

### 计算公式
1. **距离成本**: `min(distance/10.0, 10.0) * DISTANCE_WEIGHT`
2. **等待优势**: `min(waiting_minutes * 0.1, 5.0) * WAITING_TIME_WEIGHT`
3. **超时优势**: `TIMEOUT_HISTORY_WEIGHT if has_timeout_history else 0.0`
4. **类型成本**: `TASK_TYPE_WEIGHT if task_type == 'call' else 0.0`
5. **等级优势**: `level_advantage[original_level] * LEVEL_WEIGHT`

### 最终优先级
```python
final_cost = distance_cost + type_cost - waiting_advantage - timeout_advantage - level_advantage
final_priority = 10.0 - min(max(final_cost, 0.0), 10.0)  # 范围0-10，越高越优先
```

## API端点

### 1. 增强优化顺序端点
```
POST /api/tasks/robot/enhanced_optimized_order
```

**请求格式**:
```json
{
    "optimized_tasks": [
        {
            "task_id": "T20241222123456ABC123",
            "final_priority": 8.5,
            "distance_from_current": 15.2,
            "optimization_reason": "timeout_history_long_waiting"
        }
    ]
}
```

## 话题通信

### 1. 增强多目标导航话题
```
话题: /enhanced_multi_nav_command
类型: std_msgs/String
```

**消息格式**:
```json
{
    "tasks": [
        {
            "task_id": "T20241222123456ABC123",
            "location_name": "办公室A",
            "task_type": "send",
            "security_level": "L1",
            "original_security_level": "L2",
            "waiting_time_seconds": 1800,
            "has_timeout_history": true,
            "receiver": "user123",
            "created_at": "2024-12-22T12:00:00",
            "description": "重要文件送达"
        }
    ],
    "timestamp": "2024-12-22T12:34:56",
    "scheduler_version": "enhanced_v1.0"
}
```

## 使用示例

### 1. 启动增强调度
```python
# 后端创建任务后自动加入队列
success, message = start_task_execution()
```

### 2. ROS2处理流程
```bash
# ROS2接收增强命令
[INFO] Received enhanced navigation command with 3 tasks
[INFO] Task T123 priority calculation: distance_cost=2.5, waiting_adv=1.5, timeout_adv=10.0, final_priority=8.2
[INFO] Enhanced TSP optimization completed
[INFO] Notifying backend API: enhanced optimized order
```

### 3. 后端接收优化结果
```python
# 自动调用 handle_robot_optimized_order()
# 重新排列执行队列中的任务顺序
```

## 优势特性

1. **解耦设计**: 权重计算逻辑完全在ROS2节点中，后端只负责数据传递
2. **智能调度**: 综合考虑距离、等待时间、超时历史等多个因素
3. **向后兼容**: 保留原有的简单调度模式
4. **可配置权重**: 可以轻松调整各项权重因子
5. **详细日志**: 完整记录优化过程和原因

## 配置调优

### 调整权重因子
在 `optimized_multi_nav.py` 中修改权重配置:
```python
# 增加超时历史的重要性
TIMEOUT_HISTORY_WEIGHT = 15.0

# 减少距离因子的影响
DISTANCE_WEIGHT = 0.5

# 增加等待时间的影响
WAITING_TIME_WEIGHT = 1.0
```

### 调整优先级阈值
```python
# 修改等待时间优势的计算
waiting_advantage = min(waiting_minutes * 0.2, 8.0)  # 增加系数和上限

# 修改安全等级权重
level_advantages = {'L3': 4.0, 'L2': 3.0, 'L1': 2.0, 'L0': 0.0}
```

这个增强调度系统现在完全实现了你要求的解耦设计，权重计算在ROS2节点中进行，后端只负责数据传递和接收优化结果。
