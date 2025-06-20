# Secure Robot API - 任务调度系统

这是一个基于优先级的机器人任务调度API系统，支持多优先级队列管理和智能任务调度。

## 任务调度方法

### 优先级队列系统

系统支持三个优先级级别的任务队列：

- **L3 (最高优先级)**: 紧急任务，优先级值 = 3
- **L2 (中等优先级)**: 普通重要任务，优先级值 = 2  
- **L1 (最低优先级)**: 一般任务，优先级值 = 1

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

#### 3. 任务回退与重新调度
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

#### 继续下一个任务
```http
POST /api/tasks/next
```
- 继续执行当前队列中的下一个任务
- 只能在当前队列内使用，不能跨优先级

## 队列状态监控

### 获取队列状态信息

通过以下API可以获取详细的队列状态信息：

```http
GET /api/tasks/queue/status
```

### 返回信息详解

#### 1. 基本队列信息
```json
{
  "queues": {
    "L3": 2,  // L3队列中等待的任务数量
    "L2": 1,  // L2队列中等待的任务数量  
    "L1": 3   // L1队列中等待的任务数量
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
    "L1": [...]
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

### 任务状态说明

- **pending**: 等待执行的任务
- **executing**: 当前正在执行的任务
- **waiting_in_queue**: 执行队列中等待执行的任务
- **completed_in_queue**: 执行队列中已完成的任务
- **completed**: 已完成的任务
- **failed**: 执行失败的任务

### 使用示例

#### 监控队列状态
```bash
curl -X 'GET' \
  'http://localhost:8000/api/tasks/queue/status' \
  -H 'accept: application/json'
```

#### 启动任务执行
```bash
curl -X 'POST' \
  'http://localhost:8000/api/tasks/start' \
  -H 'accept: application/json'
```

#### 继续下一个任务
```bash
curl -X 'POST' \
  'http://localhost:8000/api/tasks/next' \
  -H 'accept: application/json'
```

## 特性

- ✅ **智能优先级抢占**: 高优先级任务可以中断低优先级队列执行
- ✅ **任务不丢失保证**: 被中断的任务会正确回退并重新参与调度
- ✅ **详细状态监控**: 提供全面的队列状态、执行进度和任务历史信息
- ✅ **手动控制执行**: 队列完成后需要手动触发下一优先级队列，避免意外执行
- ✅ **容错机制**: 包含延时控制和错误处理，确保系统稳定性
