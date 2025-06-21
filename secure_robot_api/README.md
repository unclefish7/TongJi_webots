# Secure Robot API - 任务调度系统

这是一个基于优先级的机器人任务调度API系统，支持多优先级队列管理和智能任务调度。

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

#### 4. TSP路径优化
系统集成了ROS2的TSP（旅行商问题）优化：

1. **路径优化**: ROS2节点会对多个目标点进行路径优化
2. **顺序同步**: 优化后的执行顺序会同步回FastAPI后端
3. **队列重排**: 后端根据优化顺序重新排列执行队列中的任务

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
- 返回启动结果和调度详情

**响应示例:**
```json
{
  "success": true,
  "message": "L3 queue started successfully with 3 targets: ['T20241220143021ABC123', 'T20241220143022DEF456', 'T20241220143023GHI789']"
}
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

#### 接收ROS2优化顺序
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

## TSP路径优化

### 优化流程
1. **发送任务**: FastAPI后端将多个目标点发送给ROS2节点
2. **路径优化**: ROS2节点使用TSP算法优化访问顺序
3. **顺序同步**: 优化后的顺序通过`/api/tasks/robot/optimized_order`接口同步回后端
4. **队列重排**: 后端根据优化顺序重新排列执行队列中的任务

### 优化效果
- **距离最短**: 机器人按最短路径访问所有目标点
- **时间节省**: 减少不必要的往返路径
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

## 特性

- ✅ **四级优先级队列**: 支持L3/L2/L1/L0四个优先级，包含降级队列
- ✅ **智能优先级抢占**: 高优先级任务可以中断低优先级队列执行
- ✅ **任务到达与取件**: 支持到达状态管理和用户认证取件
- ✅ **灵活降级策略**: 支持三种可配置的降级策略，满足不同业务需求
- ✅ **动态策略切换**: 支持运行时动态修改降级策略，无需重启系统
- ✅ **TSP路径优化**: 集成ROS2路径优化，提高配送效率
- ✅ **任务不丢失保证**: 被中断的任务会正确回退并重新参与调度
- ✅ **详细状态监控**: 提供全面的队列状态、执行进度和任务历史信息
- ✅ **手动控制执行**: 队列完成后需要手动触发下一优先级队列，避免意外执行
- ✅ **容错机制**: 包含延时控制和错误处理，确保系统稳定性
- ✅ **前端友好API**: 提供完整的状态信息和清晰的接口文档
