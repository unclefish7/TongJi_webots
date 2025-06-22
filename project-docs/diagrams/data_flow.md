# 系统数据流图

本文档展示了智能配送机器人系统的数据流向和处理过程。

## 1. 系统整体数据流图

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                               系统数据流架构图                                        │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│ ┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────────────┐ │
│ │   用户端     │────▶│   前端界面   │────▶│   API网关    │────▶│      业务逻辑        │ │
│ │   Client    │     │  Frontend   │     │ API Gateway │     │  Business Logic     │ │
│ │             │     │             │     │             │     │                     │ │
│ │ ┌─────────┐ │     │ ┌─────────┐ │     │ ┌─────────┐ │     │ ┌─────────────────┐ │ │
│ │ │用户输入  │ │────▶│ │界面交互  │ │────▶│ │请求处理  │ │────▶│ │任务管理          │ │ │
│ │ │数据     │ │     │ │数据     │ │     │ │数据     │ │     │ │数据             │ │ │
│ │ └─────────┘ │     │ └─────────┘ │     │ └─────────┘ │     │ └─────────────────┘ │ │
│ │ ┌─────────┐ │     │ ┌─────────┐ │     │ ┌─────────┐ │     │ ┌─────────────────┐ │ │
│ │ │语音指令  │ │────▶│ │状态显示  │ │◀────│ │响应数据  │ │◀────│ │路径规划          │ │ │
│ │ │数据     │ │     │ │数据     │ │     │ │         │ │     │ │数据             │ │ │
│ │ └─────────┘ │     │ └─────────┘ │     │ └─────────┘ │     │ └─────────────────┘ │ │
│ └─────────────┘     └─────────────┘     └─────────────┘     └─────────────────────┘ │
│         ▲                   ▲                   ▲                         │         │
│         │                   │                   │                         ▼         │
│         │                   │                   │           ┌─────────────────────┐ │
│         │                   │                   │           │      数据持久化      │ │
│         │                   │                   │           │   Data Persistence  │ │
│         │                   │                   │           │                     │ │
│         │                   │                   │           │ ┌─────────────────┐ │ │
│         │                   │                   │           │ │SQLite数据库      │ │ │
│         │                   │                   │           │ │任务/用户数据     │ │ │
│         │                   │                   │           │ └─────────────────┘ │ │
│         │                   │                   │           │ ┌─────────────────┐ │ │
│         │                   │                   │           │ │文件存储          │ │ │
│         │                   │                   │           │ │日志/配置数据     │ │ │
│         │                   │                   │           │ └─────────────────┘ │ │
│         │                   │                   │           └─────────────────────┘ │
│         │                   │                   │                         │         │
│         │                   │                   │                         ▼         │
│ ┌─────────────────────────────────────────────────────────────────────────────┐     │
│ │                          ROS2通信层                                         │     │
│ │                       ROS2 Communication                                   │     │
│ │                                                                             │     │
│ │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │     │
│ │ │话题消息      │  │服务调用      │  │动作指令      │  │参数配置              │ │     │
│ │ │Topic Msgs   │  │Service Call │  │Action Cmd   │  │Parameter Config     │ │     │
│ │ │             │  │             │  │             │  │                     │ │     │
│ │ │/robot_pose  │  │/path_plan   │  │/navigate    │  │/robot_params        │ │     │
│ │ │/robot_status│  │/map_update  │  │/dock_robot  │  │/nav_params          │ │     │
│ │ │/sensor_data │  │/emergency   │  │/pickup      │  │/safety_params       │ │     │
│ │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │     │
│ └─────────────────────────────────────────────────────────────────────────────┘     │
│                                       │                                             │
│                                       ▼                                             │
│ ┌─────────────────────────────────────────────────────────────────────────────┐     │
│ │                        机器人控制系统                                        │     │
│ │                      Robot Control System                                  │     │
│ │                                                                             │     │
│ │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │     │
│ │ │导航数据      │  │传感器数据    │  │执行器数据    │  │状态监控数据          │ │     │
│ │ │Nav Data     │  │Sensor Data  │  │Actuator Data│  │Status Monitor Data  │ │     │
│ │ │             │  │             │  │             │  │                     │ │     │
│ │ │路径点坐标    │  │激光雷达      │  │轮速控制      │  │电池电量              │ │     │
│ │ │地图数据      │  │IMU数据      │  │舵机角度      │  │系统温度              │ │     │
│ │ │目标位置      │  │摄像头图像    │  │制动器状态    │  │网络连接              │ │     │
│ │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │     │
│ └─────────────────────────────────────────────────────────────────────────────┘     │
│                                       │                                             │
│                                       ▼                                             │
│ ┌─────────────────────────────────────────────────────────────────────────────┐     │
│ │                         Webots仿真环境                                      │     │
│ │                       Webots Simulation                                     │     │
│ │                                                                             │     │
│ │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │     │
│ │ │物理仿真数据  │  │环境模型数据  │  │机器人模型    │  │传感器仿真数据        │ │ ◀───┘
│ │ │Physics Data │  │World Model  │  │Robot Model  │  │Sensor Simulation    │ │
│ │ │             │  │             │  │             │  │                     │ │
│ │ │碰撞检测      │  │建筑物布局    │  │运动学模型    │  │虚拟激光雷达          │ │
│ │ │摩擦力计算    │  │障碍物分布    │  │动力学特性    │  │虚拟IMU              │ │
│ │ │重力影响      │  │地面材质      │  │外观渲染      │  │虚拟摄像头            │ │
│ │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│ └─────────────────────────────────────────────────────────────────────────────┘
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## 2. 数据类型和结构

### 2.1 用户输入数据结构

```python
# 任务创建数据
TaskCreateData = {
    "pickup_location": str,      # 取货地点 "A101"
    "delivery_location": str,    # 送货地点 "B205"  
    "priority": str,            # 优先级 "high"|"medium"|"low"
    "scheduled_time": datetime, # 预定时间
    "special_instructions": str, # 特殊说明
    "contact_info": {           # 联系信息
        "name": str,
        "phone": str,
        "email": str
    }
}

# 用户认证数据
AuthData = {
    "username": str,
    "password": str,
    "remember_me": bool,
    "two_factor_code": str      # 可选的两因子认证码
}

# 语音指令数据
VoiceCommandData = {
    "audio_data": bytes,        # 音频数据
    "sample_rate": int,         # 采样率
    "encoding": str,            # 编码格式
    "language": str,            # 语言代码
    "user_id": str             # 用户标识
}
```

### 2.2 业务处理数据结构

```python
# 任务数据模型
TaskModel = {
    "id": str,                  # 任务唯一标识
    "status": str,             # 状态: created|queued|in_progress|completed|failed
    "user_id": str,            # 创建用户ID
    "pickup_location": Location,
    "delivery_location": Location,
    "priority": int,           # 优先级数值
    "created_at": datetime,
    "updated_at": datetime,
    "scheduled_time": datetime,
    "started_at": datetime,
    "completed_at": datetime,
    "estimated_duration": int,  # 预估耗时(秒)
    "actual_duration": int,    # 实际耗时(秒)
    "robot_id": str,           # 分配的机器人ID
    "path_data": PathData,     # 路径规划数据
    "error_message": str       # 错误信息
}

# 位置数据模型
Location = {
    "id": str,                 # 位置标识
    "name": str,               # 位置名称
    "coordinates": {           # 坐标信息
        "x": float,
        "y": float,
        "z": float,
        "orientation": float
    },
    "floor": int,              # 楼层
    "building": str,           # 建筑物
    "accessible": bool,        # 是否可达
    "restrictions": list       # 访问限制
}

# 路径规划数据
PathData = {
    "waypoints": list,         # 路径点列表
    "total_distance": float,   # 总距离
    "estimated_time": int,     # 预估时间
    "complexity": str,         # 路径复杂度
    "safety_score": float,     # 安全评分
    "alternative_paths": list  # 备选路径
}
```

### 2.3 ROS2消息数据结构

```python
# 机器人状态消息
RobotStatusMsg = {
    "header": {
        "stamp": time,
        "frame_id": str
    },
    "robot_id": str,
    "pose": {
        "position": {"x": float, "y": float, "z": float},
        "orientation": {"x": float, "y": float, "z": float, "w": float}
    },
    "velocity": {
        "linear": {"x": float, "y": float, "z": float},
        "angular": {"x": float, "y": float, "z": float}
    },
    "battery_level": float,    # 电池电量百分比
    "system_status": str,      # 系统状态
    "current_task_id": str,    # 当前任务ID
    "error_code": int,         # 错误代码
    "sensors": {               # 传感器数据
        "lidar_working": bool,
        "camera_working": bool,
        "imu_working": bool
    }
}

# 导航目标消息
NavigationGoalMsg = {
    "header": {
        "stamp": time,
        "frame_id": str
    },
    "goal_id": str,
    "target_pose": {
        "position": {"x": float, "y": float, "z": float},
        "orientation": {"x": float, "y": float, "z": float, "w": float}
    },
    "behavior": str,           # 导航行为类型
    "timeout": int,            # 超时时间
    "precision": float         # 精度要求
}

# 传感器数据消息
SensorDataMsg = {
    "header": {
        "stamp": time,
        "frame_id": str
    },
    "lidar_scan": {
        "angle_min": float,
        "angle_max": float,
        "angle_increment": float,
        "range_min": float,
        "range_max": float,
        "ranges": list          # 距离数据数组
    },
    "imu_data": {
        "orientation": {"x": float, "y": float, "z": float, "w": float},
        "angular_velocity": {"x": float, "y": float, "z": float},
        "linear_acceleration": {"x": float, "y": float, "z": float}
    },
    "odometry": {
        "pose": {"position": {}, "orientation": {}},
        "twist": {"linear": {}, "angular": {}}
    }
}
```

## 3. 数据流处理管道

### 3.1 任务处理流水线

```
用户请求 → 数据验证 → 权限检查 → 业务逻辑 → 数据持久化 → ROS2通信 → 状态反馈

详细流程:
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│   用户请求    │───▶│   数据验证    │───▶│   权限检查    │
│  User Request│    │Data Validation│    │Permission Chk│
└──────────────┘    └──────────────┘    └──────────────┘
        │                   ▲                   ▲
        ▼                   │                   │
┌──────────────┐           失败返回            失败返回
│   状态反馈    │◀─────────────────────────────────┘
│Status Feedback│
└──────────────┘
        ▲
        │
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  ROS2通信     │◀───│  数据持久化   │◀───│   业务逻辑    │
│ROS2 Comm     │    │Data Persist  │    │Business Logic│
└──────────────┘    └──────────────┘    └──────────────┘
```

### 3.2 实时数据流处理

```python
# 实时数据流处理架构
class RealTimeDataProcessor:
    """实时数据流处理器"""
    
    def __init__(self):
        self.data_sources = []      # 数据源列表
        self.processors = []        # 处理器链
        self.output_handlers = []   # 输出处理器
        
    async def process_stream(self, data_stream):
        """处理数据流"""
        async for data in data_stream:
            # 数据预处理
            processed_data = await self.preprocess(data)
            
            # 数据处理链
            for processor in self.processors:
                processed_data = await processor.process(processed_data)
            
            # 输出处理
            for handler in self.output_handlers:
                await handler.handle(processed_data)
    
    async def preprocess(self, data):
        """数据预处理"""
        # 数据清洗
        cleaned_data = self.clean_data(data)
        
        # 数据标准化
        normalized_data = self.normalize_data(cleaned_data)
        
        # 数据验证
        if not self.validate_data(normalized_data):
            raise DataValidationError("Invalid data format")
        
        return normalized_data
```

## 4. 数据一致性保证

### 4.1 事务处理机制

```python
class TaskTransactionManager:
    """任务事务管理器"""
    
    async def create_task_transaction(self, task_data):
        """创建任务的事务处理"""
        async with self.db.transaction() as tx:
            try:
                # 1. 创建任务记录
                task = await tx.create_task(task_data)
                
                # 2. 更新用户任务计数
                await tx.increment_user_task_count(task_data.user_id)
                
                # 3. 预约机器人资源
                robot = await tx.reserve_robot(task.scheduled_time)
                
                # 4. 创建路径规划记录
                path = await tx.create_path_plan(task.id, task_data.locations)
                
                # 5. 提交事务
                await tx.commit()
                
                # 6. 发送ROS2消息(事务外)
                await self.ros2_client.publish_task_created(task)
                
                return task
                
            except Exception as e:
                # 回滚事务
                await tx.rollback()
                raise TaskCreationError(f"Failed to create task: {e}")
```

### 4.2 数据同步机制

```python
class DataSynchronizer:
    """数据同步器"""
    
    def __init__(self):
        self.sync_queue = asyncio.Queue()
        self.sync_handlers = {}
        
    async def sync_data(self, source, target, data_type):
        """同步数据"""
        sync_task = {
            'source': source,
            'target': target,
            'data_type': data_type,
            'timestamp': datetime.utcnow()
        }
        
        await self.sync_queue.put(sync_task)
    
    async def process_sync_queue(self):
        """处理同步队列"""
        while True:
            sync_task = await self.sync_queue.get()
            
            try:
                handler = self.sync_handlers[sync_task['data_type']]
                await handler.sync(sync_task)
                
            except Exception as e:
                logger.error(f"Data sync failed: {e}")
                # 重试机制
                await self.retry_sync(sync_task)
```

## 5. 数据监控和分析

### 5.1 数据质量监控

```python
class DataQualityMonitor:
    """数据质量监控器"""
    
    def __init__(self):
        self.quality_metrics = {}
        self.alert_thresholds = {}
        
    async def monitor_data_quality(self, data_batch):
        """监控数据质量"""
        quality_report = {
            'completeness': self.check_completeness(data_batch),
            'accuracy': self.check_accuracy(data_batch),
            'consistency': self.check_consistency(data_batch),
            'timeliness': self.check_timeliness(data_batch)
        }
        
        # 检查是否超过阈值
        for metric, value in quality_report.items():
            threshold = self.alert_thresholds.get(metric, 0.95)
            if value < threshold:
                await self.send_quality_alert(metric, value, threshold)
        
        return quality_report
    
    def check_completeness(self, data_batch):
        """检查数据完整性"""
        total_fields = 0
        missing_fields = 0
        
        for record in data_batch:
            for field in record.required_fields:
                total_fields += 1
                if not record.get(field):
                    missing_fields += 1
        
        return 1 - (missing_fields / total_fields) if total_fields > 0 else 1
```

### 5.2 数据分析和报告

```python
class DataAnalyzer:
    """数据分析器"""
    
    async def generate_daily_report(self, date):
        """生成日报"""
        report = {
            'date': date,
            'task_statistics': await self.analyze_task_data(date),
            'robot_performance': await self.analyze_robot_data(date),
            'system_health': await self.analyze_system_data(date),
            'user_behavior': await self.analyze_user_data(date)
        }
        
        return report
    
    async def analyze_task_data(self, date):
        """分析任务数据"""
        tasks = await self.db.get_tasks_by_date(date)
        
        return {
            'total_tasks': len(tasks),
            'completed_tasks': len([t for t in tasks if t.status == 'completed']),
            'failed_tasks': len([t for t in tasks if t.status == 'failed']),
            'average_duration': sum(t.duration for t in tasks) / len(tasks),
            'peak_hours': self.calculate_peak_hours(tasks),
            'popular_locations': self.calculate_popular_locations(tasks)
        }
```

## 6. 数据备份和恢复

### 6.1 自动备份策略

```python
class DataBackupManager:
    """数据备份管理器"""
    
    def __init__(self):
        self.backup_schedule = {
            'incremental': '0 */6 * * *',  # 每6小时增量备份
            'full': '0 2 * * 0',           # 每周日全量备份
        }
        
    async def create_backup(self, backup_type='incremental'):
        """创建备份"""
        timestamp = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
        backup_path = f"backups/{backup_type}_{timestamp}"
        
        if backup_type == 'full':
            await self.create_full_backup(backup_path)
        else:
            await self.create_incremental_backup(backup_path)
        
        # 验证备份完整性
        if await self.verify_backup(backup_path):
            logger.info(f"Backup created successfully: {backup_path}")
            return backup_path
        else:
            raise BackupError(f"Backup verification failed: {backup_path}")
    
    async def restore_from_backup(self, backup_path):
        """从备份恢复"""
        # 停止服务
        await self.stop_services()
        
        try:
            # 恢复数据
            await self.restore_database(backup_path)
            await self.restore_files(backup_path)
            
            # 验证恢复
            if await self.verify_restore():
                logger.info("Data restored successfully")
            else:
                raise RestoreError("Restore verification failed")
                
        finally:
            # 重启服务
            await self.start_services()
```

## 7. 数据安全和隐私

### 7.1 数据加密

```python
class DataEncryption:
    """数据加密服务"""
    
    def __init__(self, encryption_key):
        self.cipher = Fernet(encryption_key)
        
    def encrypt_sensitive_data(self, data):
        """加密敏感数据"""
        if isinstance(data, dict):
            encrypted_data = {}
            for key, value in data.items():
                if key in self.sensitive_fields:
                    encrypted_data[key] = self.cipher.encrypt(str(value).encode())
                else:
                    encrypted_data[key] = value
            return encrypted_data
        
        return self.cipher.encrypt(str(data).encode())
    
    def decrypt_sensitive_data(self, encrypted_data):
        """解密敏感数据"""
        if isinstance(encrypted_data, dict):
            decrypted_data = {}
            for key, value in encrypted_data.items():
                if key in self.sensitive_fields:
                    decrypted_data[key] = self.cipher.decrypt(value).decode()
                else:
                    decrypted_data[key] = value
            return decrypted_data
        
        return self.cipher.decrypt(encrypted_data).decode()
```

### 7.2 数据脱敏

```python
class DataAnonymizer:
    """数据脱敏器"""
    
    def anonymize_user_data(self, user_data):
        """用户数据脱敏"""
        anonymized = user_data.copy()
        
        # 姓名脱敏
        if 'name' in anonymized:
            anonymized['name'] = self.mask_name(anonymized['name'])
        
        # 电话脱敏
        if 'phone' in anonymized:
            anonymized['phone'] = self.mask_phone(anonymized['phone'])
        
        # 邮箱脱敏
        if 'email' in anonymized:
            anonymized['email'] = self.mask_email(anonymized['email'])
        
        return anonymized
    
    def mask_name(self, name):
        """姓名脱敏"""
        return name[0] + '*' * (len(name) - 1)
    
    def mask_phone(self, phone):
        """电话脱敏"""
        return phone[:3] + '*' * 4 + phone[-4:]
    
    def mask_email(self, email):
        """邮箱脱敏"""
        local, domain = email.split('@')
        return local[0] + '*' * (len(local) - 1) + '@' + domain
```
