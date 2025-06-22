# Secure Robot API - 增强智能任务调度系统

这是一个基于优先级的机器人任务调度API系统，支持多优先级队列管理和**增强智能任务调度**。系统采用分离式架构，结合FastAPI后端和ROS2节点，实现了多因子权重计算、智能TSP优化和动态优先级调整。同时集成了语音识别功能，支持通过语音指令进行地点选择。

## 核心特性

🚀 **增强智能调度**: 不仅考虑地理距离，还综合分析等待时间、超时历史、任务类型、安全等级等多个因素

🎯 **分离式架构**: 后端负责数据收集，ROS2节点负责智能计算，职责明确，性能优化

📊 **多因子权重**: 五大权重因子（距离、等待时间、超时历史、任务类型、安全等级）智能计算任务优先级

🔄 **智能TSP算法**: 增强的旅行商问题求解，平衡路径距离和任务优先级

📈 **动态优先级**: 任务等待时间越长自动获得更高优先级，确保公平调度

🛡️ **容错恢复**: 超时任务在重新调度时自动获得优先级提升

## 快速开始

### 1. 安装依赖

```bash
# 安装基础依赖
pip install -r requirements.txt

# 安装语音识别依赖
pip install -r requirements_voice.txt
```

### 2. 配置环境变量

复制环境配置示例文件并修改：

```bash
cp .env.example .env
```

在`.env`文件中设置您的OpenAI API密钥：

```env
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_BASE_URL=https://api.openai.com/v1
```

### 3. 设置语音识别模型

```bash
# 运行Vosk模型下载脚本
./setup_vosk.sh
```

### 4. 启动服务

```bash
# 启动API服务
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## 语音识别功能

### 支持的功能
- **语音转文本**: 使用Vosk进行本地语音识别
- **智能语义分析**: 使用LLM进行语义理解和地点提取
- **多种音频格式**: 支持WAV、MP3、M4A等格式
- **实时处理**: 前端录音，后端实时处理

### API接口
- `POST /voice/recognize` - 语音识别接口
- `GET /voice/locations` - 获取可用地点列表
- `POST /voice/test-recognition` - 测试文本地点提取

### 支持的地点
- 经理室、财务处、等候处、前台、休息室
- 小办公区、大办公区、小会议室、大会议室

## 任务调度方法

### 优先级队列系统

系统支持四个优先级级别的任务队列：

- **L3 (最高优先级)**: 紧急任务，优先级值 = 3
- **L2 (中等优先级)**: 普通重要任务，优先级值 = 2  
- **L1 (最低优先级)**: 一般任务，优先级值 = 1
- **L0 (降级队列)**: 超时降级任务，优先级值 = 0（仅接收从L1降级的任务）

### 任务调度策略

#### 1. 基本调度原则
- 系统始终优先执行最高优先级队列中的任务
- 同一优先级队列内的任务按照添加顺序执行
- 只有当前优先级队列完全执行完毕后，才需要手动启动下一优先级队列

#### 2. 优先级抢占机制
当调用 `start` 接口时，系统会检查是否需要进行优先级抢占：

- **高优先级抢占**: 如果新任务的优先级高于当前执行队列，立即中断当前队列
- **同优先级抢占**: 如果新任务与当前执行队列同优先级，也会进行抢占以重新调度
- **低优先级拒绝**: 如果新任务的优先级低于当前执行队列，拒绝启动并返回错误
- **L0队列特殊规则**: L0队列只有在L1-L3队列完全为空时才会被执行

#### 3. 任务到达与取件机制
机器人到达目标位置后的处理流程：

1. **到达状态**: 机器人到达后，任务进入`arrived`状态，等待用户取件
2. **取件认证**: 用户需要通过认证接口验证身份后才能开柜门取件
3. **取件完成**: 用户成功取件后，任务标记为`completed`
4. **超时降级**: 如果用户在规定时间内（默认10秒）未取件，任务会降级：
   - L3 → L2
   - L2 → L1  
   - L1 → L0
   - L0级任务超时不再降级

#### 4. 增强智能调度系统
系统集成了智能的任务优先级计算和TSP（旅行商问题）优化：

**权重计算架构**：
- **后端职责**: 收集和传递任务详细信息（等待时间、超时历史、任务类型等）
- **ROS2节点职责**: 执行智能权重计算和路径优化算法
- **双向通信**: 后端发送任务信息，ROS2返回优化结果和优先级

**智能优先级计算**：
1. **多因子权重**: 综合考虑地理距离、等待时间、超时历史、任务类型、原始安全等级
2. **动态优化**: 实时计算每个任务的综合优先级分数
3. **智能TSP**: 基于权重的增强TSP算法，不仅优化距离，还考虑优先级
4. **优化原因**: 提供详细的优化原因说明，便于理解调度决策

**权重因子详解**：
- **距离成本** (1.0倍权重): 到当前位置的地理距离
- **等待优势** (0.5倍权重): 等待时间越长，优先级越高
- **超时优势** (10.0倍权重): 有超时历史的任务优先处理
- **任务类型** (2.0倍权重): call任务优先级稍低于send任务
- **安全等级** (5.0倍权重): 原始安全等级越高，优先级越高

**增强通信协议**：
```json
// 后端发送给ROS2的增强任务信息
{
  "tasks": [
    {
      "task_id": "T123",
      "location_name": "办公室A",
      "task_type": "send",
      "waiting_time_seconds": 1800,
      "has_timeout_history": true,
      "original_security_level": "L2",
      "security_level": "L1"
    }
  ]
}

// ROS2返回的优化结果
{
  "optimized_tasks": [
    {
      "task_id": "T123",
      "final_priority": 8.5,
      "distance_from_current": 15.2,
      "optimization_reason": "timeout_history_long_waiting"
    }
  ]
}
```

#### 5. 任务回退与重新调度
抢占发生时：
1. 当前执行队列中未完成的任务会回退到等待队列
2. 被中断的任务状态重置为 `pending`
3. 系统重新获取最高优先级队列（包含新任务和回退任务）
4. 所有任务重新参与调度，确保无任务丢失

### API 接口

#### 启动任务执行
```http
POST /api/tasks/start
```
- 启动最高优先级队列的执行
- 支持优先级抢占机制
- 自动使用增强调度系统进行智能优化
- 返回启动结果和调度详情

**响应示例:**
```json
{
  "success": true,
  "message": "L3 queue started successfully with 3 enhanced tasks: ['T20241220143021ABC123', 'T20241220143022DEF456', 'T20241220143023GHI789']"
}
```

**增强调度日志示例:**
```
Task 1: T20241220143021ABC123 at 办公室A (waiting: 1800s, timeout_history: True)
Task 2: T20241220143022DEF456 at 会议室B (waiting: 600s, timeout_history: False)  
Task 3: T20241220143023GHI789 at 前台 (waiting: 300s, timeout_history: False)

Enhanced TSP optimization completed:
  1. T20241220143021ABC123 at 办公室A (priority: 8.5, distance: 15.2m)
  2. T20241220143023GHI789 at 前台 (priority: 6.2, distance: 8.5m)
  3. T20241220143022DEF456 at 会议室B (priority: 5.8, distance: 12.1m)
```

#### 继续下一个任务
```http
POST /api/tasks/next
```
- 继续执行当前队列中的下一个任务
- 只能在当前队列内使用，不能跨优先级

**响应示例:**
```json
{
  "success": true,
  "message": "Next command sent for L3 queue, continuing to target 2"
}
```

#### 机器人到达通知
```http
POST /api/tasks/robot/arrived
```
- 机器人到达目标位置时调用
- 将当前任务标记为到达状态，等待用户取件

**响应示例:**
```json
{
  "success": true,
  "message": "Robot arrived at target 2/3, task T20241220143021ABC123 is waiting for pickup"
}
```

#### 用户取件认证
```http
POST /api/pickup/authenticate
```
**请求参数:**
```json
{
  "user_id": "emp002",
  "auth_code": "123456"
}
```

#### 用户取件执行
```http
POST /api/pickup/execute
```
**请求参数:**
```json
{
  "user_id": "emp002",
  "task_id": "T20241220143021ABC123"
}
```

#### 接收ROS2优化顺序（基础版本）
```http
POST /api/tasks/robot/optimized_order
```
**请求参数:**
```json
{
  "original_order": ["财务处", "会议室A", "人事部"],
  "optimized_order": ["会议室A", "财务处", "人事部"]
}
```

#### 接收ROS2增强优化顺序（推荐）
```http
POST /api/tasks/robot/enhanced_optimized_order
```
**请求参数:**
```json
{
  "optimized_tasks": [
    {
      "task_id": "T20241220143021ABC123",
      "final_priority": 8.5,
      "distance_from_current": 15.2,
      "optimization_reason": "timeout_history_long_waiting"
    },
    {
      "task_id": "T20241220143022DEF456", 
      "final_priority": 6.2,
      "distance_from_current": 8.5,
      "optimization_reason": "high_priority_distance_optimization"
    }
  ]
}
```

**优化原因说明:**
- `timeout_history`: 有超时历史，优先处理
- `long_waiting`: 等待时间较长，提升优先级
- `high_priority`: 综合优先级较高
- `call_task`: call类型任务
- `distance_optimization`: 基于距离的标准优化

## 队列状态监控 API

### 获取队列状态信息

**接口地址**: `GET /api/tasks/queue/status`

**请求示例:**
```bash
curl -X 'GET' \
  'http://localhost:8000/api/tasks/queue/status' \
  -H 'accept: application/json'
```

**完整响应结构:**
```json
{
  "queues": { /* 基本队列信息 */ },
  "queue_details": { /* 等待队列详细信息 */ },
  "current_executing_task": { /* 当前执行任务信息 */ },
  "arrived_task": { /* 到达任务信息 */ },
  "execution_queue": { /* 执行队列详细信息 */ },
  "completed_tasks": [ /* 已完成任务历史 */ ],
  "current_execution": { /* 执行状态信息 */ },
  "ros2_bridge": { /* 系统状态信息 */ },
  "summary": { /* 汇总统计信息 */ },
  "downgrade_strategy": { /* 当前降级策略信息 */ }
}
```

### 返回信息详解

#### 1. 基本队列信息
```json
{
  "queues": {
    "L3": 2,  // L3队列中等待的任务数量
    "L2": 1,  // L2队列中等待的任务数量  
    "L1": 3,  // L1队列中等待的任务数量
    "L0": 0   // L0降级队列中等待的任务数量
  }
}
```

#### 2. 等待队列详细信息
```json
{
  "queue_details": {
    "L3": [
      {
        "task_id": "T20241220143021ABC123",
        "user_id": "emp001",
        "receiver": "emp002", 
        "location_id": "finance_office",
        "location_label": "财务处",
        "security_level": "L3",
        "description": "重要文件投递",
        "status": "pending",
        "created_at": "2024-12-20T14:30:21Z",
        "locker_id": "L001"
      }
    ],
    "L2": [...],
    "L1": [...],
    "L0": [...]  // 降级队列中的任务
  }
}
```

#### 3. 当前执行任务信息
```json
{
  "current_executing_task": {
    "task_id": "T20241220143021ABC123",
    "user_id": "emp001",
    "receiver": "emp002",
    "location_id": "finance_office", 
    "location_label": "财务处",
    "security_level": "L3",
    "description": "重要文件投递",
    "status": "executing",
    "progress": "2/3",
    "current_target": 2,
    "locker_id": "L001",
    "created_at": "2024-12-20T14:30:21Z"
  }
}
```

#### 3.1. 到达任务信息
```json
{
  "arrived_task": {
    "task_id": "T20241220143021ABC123",
    "location_id": "finance_office",
    "receiver": "emp002",
    "arrived_at": 1703062221.123,
    "waiting_time": 5,
    "timeout_remaining": 5,
    "status": "arrived"
  }
}
```
**字段说明:**
- `task_id`: 到达任务的ID
- `location_id`: 到达的位置ID
- `receiver`: 接收人用户ID
- `arrived_at`: 到达时间戳
- `waiting_time`: 已等待时间（秒）
- `timeout_remaining`: 剩余超时时间（秒）
- `status`: 固定为"arrived"

#### 4. 执行队列详细信息
```json
{
  "execution_queue": {
    "all_tasks": [
      {
        "task_id": "T20241220143021ABC123",
        "status": "executing",
        "execution_order": 2
      }
    ],
    "completed_in_queue": [
      {
        "task_id": "T20241220143020ABC122", 
        "status": "completed_in_queue",
        "execution_order": 1
      }
    ],
    "total_in_queue": 3,
    "completed_count": 1,
    "remaining_count": 2
  }
}
```

#### 5. 已完成任务历史
```json
{
  "completed_tasks": [
    {
      "task_id": "T20241220143019ABC121",
      "user_id": "emp001", 
      "receiver": "emp003",
      "location_id": "meeting_room_a",
      "location_label": "会议室A",
      "security_level": "L2", 
      "description": "会议资料",
      "status": "completed",
      "created_at": "2024-12-20T14:30:19Z",
      "locker_id": "L002"
    }
  ]
}
```

#### 6. 执行状态信息
```json
{
  "current_execution": {
    "active": true,
    "current_queue_level": "L3",
    "completed_count": 1,
    "total_count": 3,
    "waiting_for_next": false,
    "command_sent": true,
    "started": true,
    "progress": "1/3",
    "current_task_index": 1,
    "remaining_tasks": 2
  }
}
```

#### 7. 系统状态信息
```json
{
  "ros2_bridge": {
    "available": true,
    "status": "Connected successfully",
    "url": "http://localhost:8080"
  },
  "summary": {
    "total_pending_tasks": 6,
    "executing_tasks": 1, 
    "completed_tasks": 5,
    "execution_queue_tasks": 3,
    "total_tasks": 15
  }
}
```

#### 8. 汇总统计信息
```json
{
  "summary": {
    "total_pending_tasks": 6,      // 所有队列中等待的任务总数
    "executing_tasks": 1,          // 当前正在执行的任务数（0或1）
    "completed_tasks": 5,          // 历史完成的任务总数
    "execution_queue_tasks": 3,    // 执行队列中的任务总数
    "total_tasks": 15              // 系统中所有任务的总数
  }
}
```

#### 9. 降级策略信息
```json
{
  "downgrade_strategy": {
    "current_strategy": "step_by_step",
    "strategy_name": "逐级降级 (L3→L2→L1→L0)",
    "rules": {
      "L3": "L2",
      "L2": "L1",
      "L1": "L0", 
      "L0": null
    },
    "available_strategies": [
      "step_by_step",
      "flat_high",
      "all_step"
    ]
  }
}
```
**字段说明:**
- `current_strategy`: 当前使用的降级策略
- `strategy_name`: 策略的中文描述
- `rules`: 具体的降级规则映射
- `available_strategies`: 所有可用的策略选项

## 降级机制详解

### 降级触发条件
当机器人到达目标位置后，如果用户未在规定时间内（默认10秒）完成取件，系统会自动触发降级机制。

### 降级策略系统
系统支持三种可配置的降级策略，可根据业务需求灵活切换：

#### 1. 逐级降级策略 (step_by_step) - 默认策略
```
L3 → L2 → L1 → L0
L0 任务超时不再降级
```
- **适用场景**: 标准业务场景，确保任务逐步降级
- **特点**: 每个优先级都有机会被重新执行

#### 2. 高优先级平级回退策略 (flat_high)
```
L3 → L1 (跳过L2)
L2 → L1 
L1 → L0
L0 任务超时不再降级
```
- **适用场景**: 简化高优先级任务处理，减少中间等级
- **特点**: L3和L2任务都直接降级到L1，避免在L2停留

#### 3. 全部降一级策略 (all_step)
```
L3 → L2
L2 → L1  
L1 → L0
L0 → L0 (保持原级别)
```
- **适用场景**: 严格按优先级逐级处理，L0任务循环重试
- **特点**: L0任务超时后重新加入L0队列，不会丢失

### 降级后的处理
1. **内存队列更新**: 任务在内存队列中的`security_level`字段会更新为降级后的级别
2. **持久化状态**: JSON文件中任务的`security_level`字段**不会**自动更新，保持原始等级
3. **状态重置**: 任务状态重置为`pending`，等待重新调度
4. **队列重入**: 任务根据当前降级策略加入对应的目标队列，等待执行
5. **策略日志**: 所有降级操作都会记录使用的具体策略和降级路径

### 降级监控
通过队列状态API可以监控降级任务：
- 查看各队列中的任务分布情况
- 监控`downgrade_strategy`字段了解当前策略
- 通过任务执行日志追踪降级历史
- L0队列通常包含降级任务（具体取决于当前策略）

### 降级策略配置
系统提供API接口来动态配置降级策略：

#### 获取当前降级策略
```http
GET /api/tasks/downgrade/strategy
```

**响应示例:**
```json
{
  "current_strategy": "step_by_step",
  "strategy_name": "逐级降级 (L3→L2→L1→L0)",
  "rules": {
    "L3": "L2",
    "L2": "L1", 
    "L1": "L0",
    "L0": null
  },
  "available_strategies": [
    "step_by_step",
    "flat_high", 
    "all_step"
  ]
}
```

#### 设置降级策略
```http
POST /api/tasks/downgrade/strategy
```

**请求参数:**
```json
{
  "strategy": "flat_high"
}
```

**可选策略值:**
- `step_by_step`: 逐级降级策略
- `flat_high`: 高优先级平级回退策略  
- `all_step`: 全部降一级策略

**响应示例:**
```json
{
  "success": true,
  "message": "Downgrade strategy changed from step_by_step to flat_high"
}
```

### 降级策略对比表

| 策略类型 | L3降级目标 | L2降级目标 | L1降级目标 | L0降级目标 | 适用场景 |
|---------|-----------|-----------|-----------|-----------|----------|
| 逐级降级 | L2 | L1 | L0 | 不降级 | 标准业务场景 |
| 高优先级平级 | L1 | L1 | L0 | 不降级 | 简化高优先级处理 |
| 全部降一级 | L2 | L1 | L0 | 保持L0 | 严格优先级+循环重试 |

## 增强智能调度系统

### 系统架构
系统采用分离式架构，将数据收集和智能计算分开处理：

**后端 (FastAPI)**: 
- 数据收集器和协调器
- 收集任务等待时间、超时历史、安全等级等信息
- 将完整任务详情发送给ROS2节点进行智能计算
- 接收并应用ROS2返回的优化结果

**ROS2节点**: 
- 智能调度计算引擎
- 执行多因子权重计算和增强TSP优化
- 返回优化后的任务顺序和详细优先级信息

### 增强权重计算

#### 权重因子系统
系统综合考虑以下五个关键因子来计算任务优先级：

1. **地理距离成本** (权重: 1.0)
   - 计算从当前位置到目标地点的欧氏距离
   - 距离越近，成本越低，优先级越高
   - 公式: `min(distance/10.0, 10.0) * 1.0`

2. **等待时间优势** (权重: 0.5)
   - 任务创建到当前时刻的等待时间
   - 等待时间越长，优先级越高
   - 公式: `min(waiting_minutes * 0.1, 5.0) * 0.5`

3. **超时历史优势** (权重: 10.0)
   - 检查任务是否有过超时降级历史
   - 有超时历史的任务获得最高优先级加成
   - 逻辑: `original_security_level != current_security_level`

4. **任务类型调整** (权重: 2.0)
   - call任务优先级稍低于send任务
   - 考虑不同任务类型的处理复杂度
   - call任务增加额外成本

5. **原始安全等级优势** (权重: 5.0)
   - 基于任务的原始安全等级分配优先级
   - L3: 3.0分优势 > L2: 2.0分优势 > L1: 1.0分优势 > L0: 0分优势
   - 确保高等级任务即使降级后仍保持相对优先权

#### 综合优先级计算
```python
# 最终成本计算 (越低越优先)
final_cost = distance_cost + type_cost - waiting_advantage - timeout_advantage - level_advantage

# 最终优先级分数 (0-10分，越高越优先)
final_priority = 10.0 - min(max(final_cost, 0.0), 10.0)
```

### 增强TSP算法

#### 传统TSP vs 增强TSP
- **传统TSP**: 仅考虑地理距离，寻找最短路径
- **增强TSP**: 综合距离和优先级成本，寻找最优平衡

#### 算法实现
1. **小规模精确算法** (≤6个任务): 使用暴力法遍历所有排列
2. **大规模启发式算法** (>6个任务): 使用贪心算法快速求解
3. **加权成本矩阵**: `总成本 = 地理距离 + 优先级成本`

### 通信协议

#### 后端发送任务信息
话题: `/enhanced_multi_nav_command`
```json
{
  "tasks": [
    {
      "task_id": "T20241222123456ABC123",
      "location_name": "办公室A",
      "location_id": "office_a", 
      "task_type": "send",
      "security_level": "L1",
      "original_security_level": "L2",
      "waiting_time_seconds": 1800,
      "has_timeout_history": true,
      "receiver": "emp002",
      "created_at": "2024-12-22T12:34:56",
      "description": "重要文件投递"
    }
  ],
  "timestamp": "2024-12-22T12:34:56",
  "scheduler_version": "enhanced_v1.0"
}
```

#### ROS2返回优化结果
API: `POST /api/tasks/robot/enhanced_optimized_order`
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

### 优化效果
- **智能优先级**: 不仅考虑距离，还综合等待时间、超时历史等因素
- **公平调度**: 等待时间长的任务自动获得优先级提升
- **容错恢复**: 超时任务在重新调度时获得更高优先级
- **类型感知**: 根据任务类型特点进行差异化调度
- **等级保持**: 原始高等级任务即使降级后仍保持相对优势

## 传统TSP路径优化 (兼容模式)

### 基础优化流程
1. **发送任务**: FastAPI后端将多个目标点发送给ROS2节点
2. **路径优化**: ROS2节点使用传统TSP算法优化访问顺序
3. **顺序同步**: 优化后的顺序通过`/api/tasks/robot/optimized_order`接口同步回后端
4. **队列重排**: 后端根据优化顺序重新排列执行队列中的任务

### 基础优化效果
- **距离最短**: 机器人按最短路径访问所有目标点
- **时间节省**: 减少不必要的往返路径
- **能耗优化**: 降低机器人运行能耗
- **执行效率**: 提高整体任务执行效率

### 增强优化效果对比

| 对比维度 | 传统TSP优化 | 增强智能调度 |
|---------|------------|-------------|
| 优化目标 | 仅距离最短 | 距离+优先级平衡 |
| 考虑因素 | 地理位置 | 地理位置+等待时间+超时历史+任务类型+安全等级 |
| 调度公平性 | 无 | 等待时间长的任务自动提升优先级 |
| 容错能力 | 无 | 超时任务重新调度时获得优先级提升 |
| 业务感知 | 无 | 根据任务类型和安全等级差异化处理 |
| 优化效果 | 路径最短 | 综合效益最优 |
- **能耗优化**: 降低机器人运行能耗
- **执行效率**: 提高整体任务执行效率

## 前端集成指南

### 实时状态监控
建议前端每2-3秒轮询`/api/tasks/queue/status`接口，获取最新的队列状态信息。

### 关键状态处理
1. **等待取件**: 当`arrived_task`不为null时，显示取件提醒
2. **超时警告**: 根据`timeout_remaining`字段显示倒计时
3. **降级通知**: 监控各队列变化，提醒降级任务，特别关注L0队列
4. **执行进度**: 使用`current_execution.progress`显示执行进度
5. **策略显示**: 在界面中显示当前降级策略，方便用户了解系统行为

### 用户交互流程
1. **任务创建** → 调用任务创建API
2. **队列启动** → 调用`/api/tasks/start`
3. **状态监控** → 轮询状态API
4. **到达通知** → 机器人调用到达API
5. **用户取件** → 调用认证和取件API
6. **继续执行** → 调用`/api/tasks/next`（如需要）

## 增强调度系统使用指南

### 开发者集成

#### 1. 启用增强调度
增强调度系统是自动启用的，当系统检测到队列中有任务时：
- 后端自动收集任务详细信息
- 使用`/enhanced_multi_nav_command`话题发送到ROS2
- ROS2节点执行智能权重计算和优化
- 通过`/api/tasks/robot/enhanced_optimized_order`接口返回结果

#### 2. 监控调度决策
通过日志和API响应监控调度过程：

```python
# 查看权重计算日志
# ROS2节点会输出详细的权重计算过程
"Task T123 priority calculation: distance_cost=1.50, waiting_adv=0.90, timeout_adv=10.00, type_cost=0.00, level_adv=2.00, final_priority=8.40"

# 查看优化结果
{
  "optimized_tasks": [
    {
      "task_id": "T123",
      "final_priority": 8.4,
      "optimization_reason": "timeout_history_long_waiting"
    }
  ]
}
```

#### 3. 自定义权重配置
在ROS2节点中可以调整权重因子：

```python
# 在 optimized_multi_nav.py 中修改权重配置
WAITING_TIME_WEIGHT = 0.5      # 等待时间权重
TIMEOUT_HISTORY_WEIGHT = 10.0  # 超时历史权重  
DISTANCE_WEIGHT = 1.0          # 距离权重
TASK_TYPE_WEIGHT = 2.0         # 任务类型权重
LEVEL_WEIGHT = 5.0             # 原始安全等级权重
```

### 调度策略最佳实践

#### 1. 任务优先级设计
- **L3**: 用于真正紧急的任务，如安全事件、VIP请求
- **L2**: 用于重要但非紧急的任务，如管理层文件
- **L1**: 用于日常普通任务，如部门间文件传递
- **L0**: 用于降级任务，系统自动管理

#### 2. 超时时间配置
根据业务需求调整取件超时时间：
- 重要任务可以设置较长的超时时间
- 普通任务使用默认的10秒超时
- 考虑用户响应时间和业务紧急程度

#### 3. 降级策略选择
- **标准业务**: 使用`step_by_step`逐级降级
- **简化管理**: 使用`flat_high`减少中间等级
- **严格控制**: 使用`all_step`确保任务不丢失

### 性能优化建议

#### 1. 批量处理
- 尽量让同一优先级的多个任务同时启动
- 利用TSP优化减少总体路径长度
- 避免频繁的单任务启动

#### 2. 监控调优
- 定期检查L0队列，了解降级任务情况
- 根据历史数据调整权重因子
- 监控系统响应时间和优化效果

#### 3. 网络优化
- 确保ROS2桥接服务稳定运行
- 优化后端与ROS2节点之间的通信延迟
- 实现适当的重试和容错机制

## 特性

- ✅ **四级优先级队列**: 支持L3/L2/L1/L0四个优先级，包含降级队列
- ✅ **智能优先级抢占**: 高优先级任务可以中断低优先级队列执行
- ✅ **任务到达与取件**: 支持到达状态管理和用户认证取件
- ✅ **灵活降级策略**: 支持三种可配置的降级策略，满足不同业务需求
- ✅ **动态策略切换**: 支持运行时动态修改降级策略，无需重启系统
- ✅ **增强智能调度**: 多因子权重计算，综合考虑距离、等待时间、超时历史等
- ✅ **智能TSP优化**: 不仅优化距离，还考虑任务优先级的增强TSP算法
- ✅ **分离式架构**: 后端负责数据收集，ROS2负责智能计算，职责明确
- ✅ **双向通信协议**: 完整的任务信息传递和优化结果反馈机制
- ✅ **优化原因追踪**: 提供详细的调度决策原因，便于理解和调试
- ✅ **任务不丢失保证**: 被中断的任务会正确回退并重新参与调度
- ✅ **详细状态监控**: 提供全面的队列状态、执行进度和任务历史信息
- ✅ **手动控制执行**: 队列完成后需要手动触发下一优先级队列，避免意外执行
- ✅ **容错机制**: 包含延时控制和错误处理，确保系统稳定性
- ✅ **前端友好API**: 提供完整的状态信息和清晰的接口文档
- ✅ **语音识别集成**: 支持通过语音指令进行地点选择和任务创建
