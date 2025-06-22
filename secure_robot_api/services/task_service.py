import json
import os
import random
import string
import time
from typing import Dict, List, Optional
from datetime import datetime
from services.auth_service import consume_auth, get_user_by_id
from services.schema_service import create_entity_from_schema, validate_entity_against_schema
import requests
from threading import Lock
from collections import deque

def load_locations() -> List[Dict]:
    """从 locations.json 加载位置数据"""
    locations_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'locations.json')
    try:
        with open(locations_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def load_tasks() -> List[Dict]:
    """从 tasks.json 加载任务数据"""
    tasks_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'tasks.json')
    try:
        with open(tasks_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def save_tasks(tasks: List[Dict]) -> None:
    """保存任务数据到 tasks.json"""
    tasks_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'tasks.json')
    try:
        with open(tasks_path, 'w', encoding='utf-8') as f:
            json.dump(tasks, f, ensure_ascii=False, indent=2)
    except Exception as e:
        raise Exception(f"Failed to save tasks: {str(e)}")

def load_lockers() -> List[Dict]:
    """从 lockers.json 加载柜子数据"""
    lockers_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'lockers.json')
    try:
        with open(lockers_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def save_lockers(lockers: List[Dict]) -> None:
    """保存柜子数据到 lockers.json"""
    lockers_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'lockers.json')
    try:
        with open(lockers_path, 'w', encoding='utf-8') as f:
            json.dump(lockers, f, ensure_ascii=False, indent=2)
    except Exception as e:
        raise Exception(f"Failed to save lockers: {str(e)}")

def get_location_by_id(location_id: str) -> Optional[Dict]:
    """根据位置ID获取位置信息"""
    locations = load_locations()
    for location in locations:
        if location.get('location_id') == location_id:
            return location
    return None

def get_task_by_id(task_id: str) -> Optional[Dict]:
    """根据任务ID获取任务信息"""
    tasks = load_tasks()
    for task in tasks:
        if task.get('task_id') == task_id:
            return task
    return None

def get_available_locker() -> Optional[Dict]:
    """获取一个可用的柜子"""
    lockers = load_lockers()
    for locker in lockers:
        if locker.get('status') == 'available':
            return locker
    return None

def allocate_locker(locker_id: str) -> tuple[bool, str]:
    """
    分配柜子（将状态改为 in_use）
    
    Args:
        locker_id: 柜子ID
    
    Returns:
        tuple[bool, str]: (是否成功, 错误信息)
    """
    from services.log_service import log_locker_operation
    
    lockers = load_lockers()
    
    for locker in lockers:
        if locker.get('locker_id') == locker_id:
            if locker.get('status') == 'available':
                locker['status'] = 'in_use'
                try:
                    save_lockers(lockers)
                    log_locker_operation(locker_id, "allocated", success=True)
                    return True, ""
                except Exception as e:
                    log_locker_operation(locker_id, "allocation failed", success=False)
                    return False, f"Failed to allocate locker: {str(e)}"
            else:
                return False, f"Locker {locker_id} is not available"
    
    return False, f"Locker {locker_id} not found"

def release_locker(locker_id: str) -> tuple[bool, str]:
    """
    释放柜子（将状态改为 available）
    
    Args:
        locker_id: 柜子ID
    
    Returns:
        tuple[bool, str]: (是否成功, 错误信息)
    """
    from services.log_service import log_locker_operation
    
    lockers = load_lockers()
    
    for locker in lockers:
        if locker.get('locker_id') == locker_id:
            if locker.get('status') == 'in_use':
                locker['status'] = 'available'
                try:
                    save_lockers(lockers)
                    log_locker_operation(locker_id, "released", success=True)
                    return True, ""
                except Exception as e:
                    log_locker_operation(locker_id, "release failed", success=False)
                    return False, f"Failed to release locker: {str(e)}"
            else:
                return False, f"Locker {locker_id} is not in use"
    
    return False, f"Locker {locker_id} not found"

def generate_task_id() -> str:
    """生成任务ID：时间戳+随机码"""
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    random_chars = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
    return f"T{timestamp}{random_chars}"

# 定义错误编码
class TaskErrorCodes:
    SUCCESS = "TASK_000"
    INITIATOR_NOT_FOUND = "TASK_001"
    RECEIVER_NOT_FOUND = "TASK_002"
    LOCATION_NOT_FOUND = "TASK_003"
    INVALID_SECURITY_LEVEL = "TASK_004"
    INSUFFICIENT_AUTH = "TASK_005"
    NO_AVAILABLE_LOCKERS = "TASK_006"
    LOCKER_ALLOCATION_FAILED = "TASK_007"
    VALIDATION_FAILED = "TASK_008"
    SAVE_FAILED = "TASK_009"
    TASK_NOT_FOUND = "TASK_010"
    STATUS_UPDATE_FAILED = "TASK_011"
    LOCKER_RELEASE_FAILED = "TASK_012"

def validate_task_creation(user_id: str, receiver: str, location_id: str, security_level: str, task_type: str) -> tuple[bool, str, str]:
    """
    验证任务创建的前置条件
    
    Args:
        user_id: 发起人ID
        receiver: 接收人ID
        location_id: 目标位置ID
        security_level: 安全等级
        task_type: 任务类型 (call/send)
    
    Returns:
        tuple[bool, str, str]: (是否验证通过, 错误编码, 错误信息)
    """
    # 验证发起人是否存在
    initiator = get_user_by_id(user_id)
    if not initiator:
        return False, TaskErrorCodes.INITIATOR_NOT_FOUND, f"Initiator user '{user_id}' not found"
    
    # 验证接收人是否存在
    receiver_user = get_user_by_id(receiver)
    if not receiver_user:
        return False, TaskErrorCodes.RECEIVER_NOT_FOUND, f"Receiver user '{receiver}' not found"
    
    # 验证位置是否存在
    location = get_location_by_id(location_id)
    if not location:
        return False, TaskErrorCodes.LOCATION_NOT_FOUND, f"Target location '{location_id}' not found"
    
    # 验证任务类型
    if task_type not in ["call", "send"]:
        return False, TaskErrorCodes.VALIDATION_FAILED, f"Invalid task type '{task_type}'. Must be 'call' or 'send'"
    
    # 移除安全等级验证，现在接受任何security_level值
    
    return True, TaskErrorCodes.SUCCESS, ""

def create_task(user_id: str, receiver: str, location_id: str, security_level: str, task_type: str, description: Optional[str] = None) -> tuple[bool, str, str, Optional[str], Optional[Dict], Optional[str]]:
    """
    创建新任务并添加到队列
    
    Args:
        user_id: 发起人ID
        receiver: 接收人ID
        location_id: 目标位置ID
        security_level: 安全等级
        task_type: 任务类型 (call/send)
        description: 任务描述
    
    Returns:
        tuple[bool, str, str, Optional[str], Optional[Dict], Optional[str]]: 
        (是否成功, 错误编码, 消息, 任务ID, 任务详细信息, 柜子ID)
    """
    from services.log_service import log_task_creation, log_error
    
    # 验证任务创建条件
    is_valid, error_code, error_msg = validate_task_creation(user_id, receiver, location_id, security_level, task_type)
    if not is_valid:
        log_task_creation("", user_id, receiver, location_id, security_level, "", False, error_code)
        return False, error_code, error_msg, None, None, None
    
    # 根据任务类型决定是否需要柜子分配
    locker_allocated = False
    locker_id = None
    
    try:
        # 步骤1：仅对send任务分配柜子
        if task_type == "send":
            # 查找并分配可用柜子
            available_locker = get_available_locker()
            if not available_locker:
                log_task_creation("", user_id, receiver, location_id, security_level, "", False, TaskErrorCodes.NO_AVAILABLE_LOCKERS)
                return False, TaskErrorCodes.NO_AVAILABLE_LOCKERS, "No available lockers for send task creation. Please try again later.", None, None, None
            
            locker_id = available_locker.get('locker_id')
            
            # 分配柜子
            allocation_success, allocation_error = allocate_locker(locker_id)
            if not allocation_success:
                log_task_creation("", user_id, receiver, location_id, security_level, locker_id, False, TaskErrorCodes.LOCKER_ALLOCATION_FAILED)
                return False, TaskErrorCodes.LOCKER_ALLOCATION_FAILED, f"Failed to allocate locker {locker_id}: {allocation_error}", None, None, None
            
            locker_allocated = True
        
        # 步骤2：基于schema构建任务数据
        provided_values = {
            "description": description,
            "initiator": user_id,
            "receiver": receiver,
            "location_id": location_id,
            "security_level": security_level,
            "original_security_level": security_level,  # 保存原始安全等级用于跟踪降级历史
            "task_type": task_type,
            "locker_id": locker_id  # call任务时为None
        }
        
        task_data = create_entity_from_schema("Task", provided_values)
        
        # 验证生成的任务数据
        is_valid, validation_error = validate_entity_against_schema("Task", task_data)
        if not is_valid:
            # 如果验证失败，回滚柜子分配（仅send任务）
            if locker_allocated and locker_id:
                release_locker(locker_id)
            log_task_creation("", user_id, receiver, location_id, security_level, locker_id, False, TaskErrorCodes.VALIDATION_FAILED)
            return False, TaskErrorCodes.VALIDATION_FAILED, f"Task data validation failed: {validation_error}", None, None, None
        
        # 步骤3：保存任务到存储文件
        try:
            save_task_to_storage(task_data)
        except Exception as save_error:
            # 如果保存失败，回滚柜子分配（仅send任务）
            if locker_allocated and locker_id:
                release_locker(locker_id)
            log_task_creation("", user_id, receiver, location_id, security_level, locker_id, False, TaskErrorCodes.SAVE_FAILED)
            return False, TaskErrorCodes.SAVE_FAILED, f"Failed to save task: {str(save_error)}", None, None, None
        
        task_id = task_data.get("task_id")
        
        # 记录成功的任务创建日志
        log_task_creation(task_id, user_id, receiver, location_id, security_level, locker_id, True)
        
        # 获取用户和位置信息用于成功消息
        initiator = get_user_by_id(user_id)
        receiver_user = get_user_by_id(receiver)
        location = get_location_by_id(location_id)
        
        # 构建成功消息
        locker_info = f", Assigned Locker: {locker_id}" if locker_id else ""
        success_message = f"Task {task_id} ({task_type}) created successfully. Initiator: {initiator.get('name')} ({user_id}), Receiver: {receiver_user.get('name')} ({receiver}), Location: {location.get('label')} ({location_id}), Security Level: {security_level}{locker_info}"
        
        # 在成功创建任务后，添加到队列
        add_task_to_queue(task_data)
    
        return True, TaskErrorCodes.SUCCESS, success_message, task_id, task_data, locker_id
        
    except Exception as e:
        # 如果发生任何未预期的异常，回滚柜子分配（仅send任务）
        if locker_allocated and locker_id:
            release_locker(locker_id)
        log_error(f"Unexpected error during task creation: {str(e)}", user_id)
        return False, TaskErrorCodes.SAVE_FAILED, f"Unexpected error during task creation: {str(e)}", None, None, None

def save_task_to_storage(task_data: Dict) -> None:
    """
    保存任务到存储文件
    
    Args:
        task_data: 任务数据
    """
    # 加载现有任务
    tasks = load_tasks()
    
    # 添加新任务
    tasks.append(task_data)
    
    # 保存回文件
    save_tasks(tasks)

# 移除不再需要的认证相关函数
# def rollback_auth_consumption(user_id: str, security_level: str) -> None:
# def rollback_transaction(user_id: str, security_level: str, locker_id: Optional[str]) -> None:

def update_task_status(task_id: str, new_status: str, timestamp_field: Optional[str] = None) -> tuple[bool, str]:
    """
    更新任务状态
    
    Args:
        task_id: 任务ID
        new_status: 新状态
        timestamp_field: 要更新的时间戳字段 (accepted/delivered)
    
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_status_change, log_error
    
    # 验证状态值
    valid_statuses = ["pending", "authenticating", "delivering", "arrived", "completed", "failed"]
    if new_status not in valid_statuses:
        return False, f"Invalid status: {new_status}"
    
    # 加载任务
    tasks = load_tasks()
    
    # 查找并更新任务
    task_found = False
    old_status = ""
    for task in tasks:
        if task.get('task_id') == task_id:
            old_status = task.get('status', '')
            task['status'] = new_status
            
            # 更新时间戳
            if timestamp_field and timestamp_field in ['accepted', 'delivered']:
                if 'timestamps' not in task:
                    task['timestamps'] = {}
                task['timestamps'][timestamp_field] = datetime.now().isoformat()
            
            task_found = True
            break
    
    if not task_found:
        return False, f"Task not found: {task_id}"
    
    # 保存更新
    try:
        save_tasks(tasks)
        log_task_status_change(task_id, old_status, new_status)
        return True, f"Task {task_id} status updated to {new_status}"
    except Exception as e:
        log_error(f"Failed to update task {task_id}: {str(e)}", related_task=task_id)
        return False, f"Failed to update task {task_id}: {str(e)}"
        return False, f"Failed to update task: {str(e)}"

def get_tasks_by_user(user_id: str, role: str = "all") -> List[Dict]:
    """
    获取用户相关的任务
    
    Args:
        user_id: 用户ID
        role: 角色筛选 ("initiator", "receiver", "all")
    
    Returns:
        List[Dict]: 任务列表
    """
    tasks = load_tasks()
    result = []
    
    for task in tasks:
        if role == "all":
            if task.get('initiator') == user_id or task.get('receiver') == user_id:
                result.append(task)
        elif role == "initiator":
            if task.get('initiator') == user_id:
                result.append(task)
        elif role == "receiver":
            if task.get('receiver') == user_id:
                result.append(task)
    
    return result

def complete_task(task_id: str) -> tuple[bool, str, str, Optional[Dict]]:
    """
    完成任务并释放柜子（仅send任务）
    
    Args:
        task_id: 任务ID
    
    Returns:
        tuple[bool, str, str, Optional[Dict]]: (是否成功, 错误编码, 消息, 任务信息)
    """
    from services.log_service import log_locker_operation, log_error
    
    # 获取任务信息
    task = get_task_by_id(task_id)
    if not task:
        return False, TaskErrorCodes.TASK_NOT_FOUND, f"Task '{task_id}' not found", None
    
    # 更新任务状态为 completed
    status_updated, status_message = update_task_status(task_id, "completed", "delivered")
    if not status_updated:
        return False, TaskErrorCodes.STATUS_UPDATE_FAILED, f"Failed to update task status: {status_message}", None
    
    # 仅对send任务释放柜子
    warning_message = ""
    task_type = task.get('task_type', 'send')  # 默认为send以保持向后兼容
    locker_id = task.get('locker_id')
    
    if task_type == "send" and locker_id:
        locker_released, locker_message = release_locker(locker_id)
        if locker_released:
            log_locker_operation(locker_id, "released", task_id, True)
        else:
            log_locker_operation(locker_id, "release failed", task_id, False)
            warning_message = f" Warning: Failed to release locker {locker_id}: {locker_message}"
    
    # 获取更新后的任务信息
    updated_task = get_task_by_id(task_id)
    
    success_message = f"Task {task_id} ({task_type}) completed successfully{warning_message}"
    return True, TaskErrorCodes.SUCCESS, success_message, updated_task

def fail_task(task_id: str, reason: str = "") -> tuple[bool, str, str, Optional[Dict]]:
    """
    标记任务失败并释放柜子（仅send任务）
    
    Args:
        task_id: 任务ID
        reason: 失败原因
    
    Returns:
        tuple[bool, str, str, Optional[Dict]]: (是否成功, 错误编码, 消息, 任务信息)
    """
    from services.log_service import log_locker_operation, log_error
    
    # 获取任务信息
    task = get_task_by_id(task_id)
    if not task:
        return False, TaskErrorCodes.TASK_NOT_FOUND, f"Task '{task_id}' not found", None
    
    # 更新任务状态为 failed
    status_updated, status_message = update_task_status(task_id, "failed")
    if not status_updated:
        return False, TaskErrorCodes.STATUS_UPDATE_FAILED, f"Failed to update task status: {status_message}", None
    
    # 仅对send任务释放柜子
    warning_message = ""
    task_type = task.get('task_type', 'send')  # 默认为send以保持向后兼容
    locker_id = task.get('locker_id')
    
    if task_type == "send" and locker_id:
        locker_released, locker_message = release_locker(locker_id)
        if locker_released:
            log_locker_operation(locker_id, "released", task_id, True)
        else:
            log_locker_operation(locker_id, "release failed", task_id, False)
            warning_message = f" Warning: Could not release locker {locker_id}: {locker_message}"
    
    # 获取更新后的任务信息
    updated_task = get_task_by_id(task_id)
    
    failure_message = f"Task {task_id} ({task_type}) marked as failed{warning_message}"
    if reason:
        failure_message += f". Reason: {reason}"
        # 记录失败原因日志
        from services.log_service import log_error
        log_error(f"Task {task_id} failed: {reason}", related_task=task_id)
    
    return True, TaskErrorCodes.SUCCESS, failure_message, updated_task

# 任务队列管理相关代码

# 降级策略配置
class DowngradeStrategy:
    """降级策略枚举"""
    STEP_BY_STEP = "step_by_step"      # 逐级降级: L3→L2→L1→L0
    FLAT_HIGH_PRIORITY = "flat_high"    # L3、L2平级回退到L1: L3→L1, L2→L1, L1→L0
    ALL_STEP_DOWN = "all_step"          # 所有等级都降一级: L3→L2, L2→L1, L1→L0, L0→保持L0

# 全局降级策略配置 - 可以通过修改这个变量来改变降级行为
CURRENT_DOWNGRADE_STRATEGY = DowngradeStrategy.STEP_BY_STEP

# 降级策略映射表
DOWNGRADE_RULES = {
    DowngradeStrategy.STEP_BY_STEP: {
        "L3": "L3",  # L3降级到L3
        "L2": "L2",  # L2降级到L2
        "L1": "L0",  # L1降级到L0
        "L0": "L0"   # L0不再降级
    },
    DowngradeStrategy.FLAT_HIGH_PRIORITY: {
        "L3": "L1",  # L3直接降级到L1
        "L2": "L1",  # L2直接降级到L1
        "L1": "L0",  # L1降级到L0
        "L0": "L0"   # L0不再降级
    },
    DowngradeStrategy.ALL_STEP_DOWN: {
        "L3": "L2",  # L3降级到L2
        "L2": "L1",  # L2降级到L1
        "L1": "L0",  # L1降级到L0
        "L0": "L0"   # L0保持L0（不变）
    }
}

def get_downgrade_target(current_level: str) -> Optional[str]:
    """
    根据当前降级策略获取降级目标等级
    
    Args:
        current_level: 当前安全等级
        
    Returns:
        Optional[str]: 降级目标等级，None表示不降级
    """
    strategy_rules = DOWNGRADE_RULES.get(CURRENT_DOWNGRADE_STRATEGY, {})
    return strategy_rules.get(current_level)

def set_downgrade_strategy(strategy: str) -> tuple[bool, str]:
    """
    设置降级策略
    
    Args:
        strategy: 降级策略名称
        
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    global CURRENT_DOWNGRADE_STRATEGY
    
    if strategy not in [DowngradeStrategy.STEP_BY_STEP, DowngradeStrategy.FLAT_HIGH_PRIORITY, DowngradeStrategy.ALL_STEP_DOWN]:
        return False, f"Invalid downgrade strategy: {strategy}. Valid options: {list(DOWNGRADE_RULES.keys())}"
    
    old_strategy = CURRENT_DOWNGRADE_STRATEGY
    CURRENT_DOWNGRADE_STRATEGY = strategy
    
    from services.log_service import log_task_operation
    log_task_operation("system", "downgrade_strategy_changed", f"from_{old_strategy}_to_{strategy}")
    
    return True, f"Downgrade strategy changed from {old_strategy} to {strategy}"

def get_current_downgrade_strategy() -> Dict:
    """
    获取当前降级策略的详细信息
    
    Returns:
        Dict: 包含策略名称和规则的详细信息
    """
    return {
        "current_strategy": CURRENT_DOWNGRADE_STRATEGY,
        "strategy_name": {
            DowngradeStrategy.STEP_BY_STEP: "逐级降级 (L3→L2→L1→L0)",
            DowngradeStrategy.FLAT_HIGH_PRIORITY: "高优先级平级回退 (L3→L1, L2→L1, L1→L0)",
            DowngradeStrategy.ALL_STEP_DOWN: "全部降一级 (L3→L2, L2→L1, L1→L0, L0→L0)"
        }.get(CURRENT_DOWNGRADE_STRATEGY, "未知策略"),
        "rules": DOWNGRADE_RULES.get(CURRENT_DOWNGRADE_STRATEGY, {}),
        "available_strategies": list(DOWNGRADE_RULES.keys())
    }

# 任务队列（按优先级）
task_queues = {
    "L3": deque(),  # 最高优先级
    "L2": deque(),  # 中等优先级  
    "L1": deque(),  # 最低优先级
    "L0": deque()   # 降级队列（只接收从L1降级的任务）
}

# 队列锁
queue_lock = Lock()

# 当前执行的任务状态
current_execution = {
    "active": False,
    "current_queue_level": None,  # 当前执行的队列级别
    "queue_tasks": [],           # 当前队列中的所有任务
    "completed_count": 0,        # 已完成的任务数量
    "total_count": 0,           # 总任务数量
    "waiting_for_next": False,
    "command_sent": False,  # 标记是否已发送命令
    "started": False,       # 标记是否已开始执行
    "arrived_task": None,   # 当前到达等待取件的任务
    "arrived_time": None,   # 到达时间
    "pickup_timeout": 10    # 取件超时时间（秒）
}

# ROS2桥接配置
ROS2_BRIDGE_URL = "http://localhost:8080/publish_topic"
ROS2_HEALTH_URL = "http://localhost:8080/health"

def check_ros2_bridge_connection() -> tuple[bool, str]:
    """
    检查ROS2桥接服务是否可用
    
    Returns:
        tuple[bool, str]: (是否可用, 状态信息)
    """
    try:
        response = requests.get(
            ROS2_HEALTH_URL,
            timeout=3.0
        )
        
        if response.status_code == 200:
            data = response.json()
            if data.get('status') == 'healthy':
                return True, "ROS2 bridge is healthy"
            else:
                return False, f"ROS2 bridge unhealthy: {data}"
        else:
            return False, f"ROS2 bridge responded with status {response.status_code}"
            
    except requests.exceptions.ConnectionError:
        return False, "ROS2 bridge service is not running (connection refused)"
    except requests.exceptions.Timeout:
        return False, "ROS2 bridge service timeout"
    except Exception as e:
        return False, f"Error checking ROS2 bridge: {str(e)}"

def add_task_to_queue(task_data: Dict) -> None:
    """
    将任务添加到对应优先级队列（仅入队，不自动启动）
    
    Args:
        task_data: 任务数据
    """
    security_level = task_data.get("security_level")
    
    with queue_lock:
        if security_level in task_queues:
            task_queues[security_level].append(task_data)
            update_task_status(task_data["task_id"], "pending")
            
    # 注意：不再自动启动任务，需要手动调用start_task_execution

def start_task_execution() -> tuple[bool, str]:
    """
    手动启动任务执行（API接口调用）
    支持优先级抢占：如果新队列优先级更高或同等，会中断当前执行
    
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_operation
    
    # 检查是否有任务正在等待取件
    if current_execution["arrived_task"] is not None:
        # 检查是否超时
        if current_execution["arrived_time"] is not None:
            elapsed_time = time.time() - current_execution["arrived_time"]
            if elapsed_time < current_execution["pickup_timeout"]:
                return False, f"Cannot start new task: current task is waiting for pickup ({int(current_execution['pickup_timeout'] - elapsed_time)}s remaining)"
            else:
                # 超时，需要降级当前到达的任务
                _handle_pickup_timeout()
    
    # 首先检查ROS2桥接服务是否可用
    bridge_available, bridge_status = check_ros2_bridge_connection()
    if not bridge_available:
        return False, f"Cannot start task: {bridge_status}. Please ensure ROS2 bridge node is running."
    
    with queue_lock:
        # 获取最高优先级的非空队列
        new_queue_level, new_queue_tasks = get_highest_priority_queue()
        if not new_queue_level:
            return False, "No tasks in any queue"
        
        # 检查是否需要进行优先级抢占
        should_preempt = False
        preempt_message = ""
        
        if current_execution["active"]:
            current_queue_level = current_execution["current_queue_level"]
            
            # 获取优先级数值（L3=3, L2=2, L1=1, L0=0）
            def get_priority_value(level):
                return {"L3": 3, "L2": 2, "L1": 1, "L0": 0}.get(level, 0)
            
            current_priority = get_priority_value(current_queue_level)
            new_priority = get_priority_value(new_queue_level)
            
            # 如果新队列优先级高于或等于当前队列，执行抢占
            if new_priority >= current_priority:
                should_preempt = True
                preempt_message = f"Preempting {current_queue_level} queue (priority {current_priority}) with {new_queue_level} queue (priority {new_priority})"
                
                # 回退当前未完成的任务到等待队列
                remaining_tasks = current_execution["queue_tasks"][current_execution["completed_count"]:]
                if remaining_tasks:
                    # 将未完成的任务状态重置为pending
                    for task in remaining_tasks:
                        update_task_status(task["task_id"], "pending")
                        log_task_operation(task["task_id"], "preempted", f"returned_to_{current_queue_level}_queue")
                    
                    # 将未完成的任务加回到原队列
                    task_queues[current_queue_level].extend(remaining_tasks)
                
                # 重置当前执行状态
                current_execution.update({
                    "active": False,
                    "current_queue_level": None,
                    "queue_tasks": [],
                    "completed_count": 0,
                    "total_count": 0,
                    "waiting_for_next": False,
                    "command_sent": False,
                    "started": False,
                    "arrived_task": None,
                    "arrived_time": None
                })
                
                log_task_operation(f"{current_queue_level}_queue", "preempted", preempt_message)
                
                # 重新获取最高优先级队列（包含刚才回退的任务）
                new_queue_level, new_queue_tasks = get_highest_priority_queue()
                if not new_queue_level:
                    return False, "No tasks in any queue after preemption"
            else:
                # 当前队列优先级更高，不允许抢占
                return False, f"Cannot start {new_queue_level} queue (priority {new_priority}): {current_queue_level} queue (priority {current_priority}) is running with higher priority"
        
        # 构建新队列中所有任务的详细信息（传递给ROS2进行权重计算）
        task_details = []
        task_ids = []
        for task in new_queue_tasks:
            location = get_location_by_id(task["location_id"])
            if location:
                # 计算任务等待时间
                created_time = task.get("timestamps", {}).get("created") or task.get("created_at", "")
                waiting_time = 0
                if created_time:
                    try:
                        created_dt = datetime.fromisoformat(created_time.replace('Z', '+00:00'))
                        waiting_time = (datetime.now() - created_dt.replace(tzinfo=None)).total_seconds()
                    except:
                        waiting_time = 0
                
                # 检查任务是否有超时历史（通过安全等级变化推断）
                has_timeout_history = task.get("security_level") != task.get("original_security_level", task.get("security_level"))
                
                # 构建任务详细信息（只传递数据，权重计算交给ROS2节点）
                task_detail = {
                    "task_id": task["task_id"],
                    "location_name": location["label"],
                    "location_id": task["location_id"],
                    "task_type": task.get("task_type", "send"),
                    "security_level": task.get("security_level", "L0"),
                    "original_security_level": task.get("original_security_level", task.get("security_level", "L0")),
                    "waiting_time_seconds": max(int(waiting_time), 0),  # 确保非负数
                    "has_timeout_history": bool(has_timeout_history),  # 确保是布尔值
                    "receiver": task.get("receiver", ""),
                    "created_at": created_time or "",
                    "description": task.get("description", "")
                }
                
                task_details.append(task_detail)
                task_ids.append(task["task_id"])
            else:
                log_task_operation(task["task_id"], "start_failed", "location_not_found")
        
        if not task_details:
            return False, f"No valid task details found for {new_queue_level} queue tasks"
        
        # 如果发生了抢占，等待一段时间让ROS2系统稳定
        if should_preempt:
            log_task_operation(f"{new_queue_level}_queue", "preemption_delay", "waiting_for_system_stabilization")
            time.sleep(0.5)  # 等待500毫秒让系统稳定
        
        # 步骤1：发送增强的多目标导航命令到ROS2（包含任务详细信息）
        # 记录发送的任务详细信息
        for i, task_detail in enumerate(task_details):
            print(f"Task {i+1}: {task_detail['task_id']} at {task_detail['location_name']} (waiting: {task_detail['waiting_time_seconds']}s, timeout_history: {task_detail['has_timeout_history']})")
        
        nav_success, nav_error = send_enhanced_multi_nav_command_with_retry(task_details)
        if not nav_success:
            log_task_operation(f"{new_queue_level}_queue", "start_failed", "ros2_enhanced_nav_command_failed")
            return False, f"Failed to send enhanced navigation command for {new_queue_level} queue: {nav_error}"
        
        # 步骤2：如果发生了抢占，增加额外延时后再发送next指令
        if should_preempt:
            time.sleep(0.3)  # 抢占后额外等待300毫秒
        
        # 立即发送第一个next指令启动机器人
        next_success, next_error = send_next_command_with_retry()
        if not next_success:
            log_task_operation(f"{new_queue_level}_queue", "next_command_failed", "initial_start_command")
        
        # 更新执行状态 - 队列已开始执行
        current_execution.update({
            "active": True,
            "current_queue_level": new_queue_level,
            "queue_tasks": new_queue_tasks.copy(),
            "completed_count": 0,
            "total_count": len(new_queue_tasks),
            "waiting_for_next": False,
            "command_sent": True,
            "started": True
        })
        
        # 清空已启动的队列
        task_queues[new_queue_level].clear()
        
        # 更新所有任务状态为delivering
        for task in new_queue_tasks:
            update_task_status(task["task_id"], "delivering")
            log_task_operation(task["task_id"], "started_execution", f"{new_queue_level}_batch_started")
        
        success_message = f"{new_queue_level} queue started successfully with {len(task_details)} enhanced tasks: {task_ids}"
        if should_preempt:
            # 统计重新调度的任务
            total_tasks_in_queue = len(task_queues[new_queue_level]) + len(new_queue_tasks)
            success_message = f"PREEMPTED: {preempt_message}. Rescheduled {total_tasks_in_queue} total tasks in {new_queue_level} queue. {success_message}"
        
        return True, success_message

def send_multi_nav_command_with_retry(targets: List[str], max_retries: int = 3) -> tuple[bool, str]:
    """
    发送多目标导航命令到ROS2（带重试机制）
    
    Args:
        targets: 目标点名称列表
        max_retries: 最大重试次数
        
    Returns:
        tuple[bool, str]: (是否发送成功, 错误信息)
    """
    last_error = ""
    
    for attempt in range(max_retries):
        try:
            # 构建ROS2消息 - 严格按照原始格式
            ros_data = {
                "topic": "/multi_nav_command",  # 恢复为原始话题名
                "type": "std_msgs/String",
                "data": {
                    "data": json.dumps(targets, ensure_ascii=False)  # 支持中文字符
                }
            }
            
            # 发送HTTP请求到ROS2桥接
            response = requests.post(
                ROS2_BRIDGE_URL,
                json=ros_data,
                headers={'Content-Type': 'application/json'},
                timeout=5.0
            )
            
            if response.status_code == 200:
                return True, ""
            else:
                try:
                    error_data = response.json()
                    last_error = f"HTTP {response.status_code}: {error_data.get('error', response.text)}"
                except:
                    last_error = f"HTTP {response.status_code}: {response.text}"
                
        except requests.exceptions.ConnectionError as e:
            last_error = f"Connection error: ROS2 bridge service not available (attempt {attempt + 1}/{max_retries})"
        except requests.exceptions.Timeout as e:
            last_error = f"Timeout error: ROS2 bridge took too long to respond (attempt {attempt + 1}/{max_retries})"
        except Exception as e:
            last_error = f"Unexpected error: {str(e)} (attempt {attempt + 1}/{max_retries})"
        
        # 如果不是最后一次尝试，等待后重试
        if attempt < max_retries - 1:
            time.sleep(1)
    
    return False, last_error

def send_next_command_with_retry(max_retries: int = 3) -> tuple[bool, str]:
    """
    发送next指令到ROS2（带重试机制）
    
    Args:
        max_retries: 最大重试次数
        
    Returns:
        tuple[bool, str]: (是否发送成功, 错误信息)
    """
    last_error = ""
    
    for attempt in range(max_retries):
        try:
            # 构建ROS2消息 - 严格按照原始格式
            ros_data = {
                "topic": "/next",  # 恢复为原始话题名
                "type": "std_msgs/String", 
                "data": {
                    "data": "start"  # 修改为 "start" 而不是 "continue"
                }
            }
            
            # 发送HTTP请求到ROS2桥接
            response = requests.post(
                ROS2_BRIDGE_URL,
                json=ros_data,
                headers={'Content-Type': 'application/json'},
                timeout=5.0
            )
            
            if response.status_code == 200:
                return True, ""
            else:
                try:
                    error_data = response.json()
                    last_error = f"HTTP {response.status_code}: {error_data.get('error', response.text)}"
                except:
                    last_error = f"HTTP {response.status_code}: {response.text}"
                
        except requests.exceptions.ConnectionError as e:
            last_error = f"Connection error: ROS2 bridge service not available (attempt {attempt + 1}/{max_retries})"
        except requests.exceptions.Timeout as e:
            last_error = f"Timeout error: ROS2 bridge took too long to respond (attempt {attempt + 1}/{max_retries})"
        except Exception as e:
            last_error = f"Unexpected error: {str(e)} (attempt {attempt + 1}/{max_retries})"
        
        # 如果不是最后一次尝试，等待后重试
        if attempt < max_retries - 1:
            time.sleep(1)
    
    return False, last_error

def send_multi_nav_command(targets: List[str]) -> bool:
    """
    发送多目标导航命令到ROS2（保持向后兼容）
    
    Args:
        targets: 目标点名称列表
        
    Returns:
        是否发送成功
    """
    success, _ = send_multi_nav_command_with_retry(targets)
    return success

def send_next_command() -> bool:
    """
    发送next指令到ROS2（保持向后兼容）
    
    Returns:
        是否发送成功
    """
    success, _ = send_next_command_with_retry()
    return success

def get_highest_priority_queue() -> tuple[Optional[str], List[Dict]]:
    """
    获取最高优先级的非空队列
    L0队列只有在L1-L3队列都为空时才会被选中
    
    Returns:
        tuple[Optional[str], List[Dict]]: (队列级别, 队列中的任务列表)
    """
    # 按优先级顺序检查队列（L0队列最后检查）
    for level in ["L3", "L2", "L1"]:
        if task_queues[level]:
            return level, list(task_queues[level])
    
    # 只有在L1-L3队列都为空时才检查L0队列
    if task_queues["L0"]:
        return "L0", list(task_queues["L0"])
    
    return None, []

# 移除 start_next_queue() 函数，因为不再需要自动启动下一个队列

def complete_current_task() -> None:
    """
    完成当前任务（机器人到达一个目标点时调用）
    """
    from services.log_service import log_task_operation
    
    if not current_execution["active"]:
        return
    
    current_execution["completed_count"] += 1
    
    # 获取当前完成的任务
    if current_execution["completed_count"] <= len(current_execution["queue_tasks"]):
        completed_task = current_execution["queue_tasks"][current_execution["completed_count"] - 1]
        task_id = completed_task["task_id"]
        
        # 完成单个任务
        complete_task(task_id)
        log_task_operation(task_id, "completed_execution", f"task_{current_execution['completed_count']}")
    
    # 检查是否完成了当前队列的所有任务
    if current_execution["completed_count"] >= current_execution["total_count"]:
        # 当前队列完成，重置执行状态
        queue_level = current_execution["current_queue_level"]
        current_execution.update({
            "active": False,
            "current_queue_level": None,
            "queue_tasks": [],
            "completed_count": 0,
            "total_count": 0,
            "waiting_for_next": False,
            "command_sent": False,
            "started": False,
            "arrived_task": None,
            "arrived_time": None
        })
        
        log_task_operation(f"{queue_level}_queue", "completed_all_tasks", f"total_{current_execution['total_count']}")
        
        # 不再自动启动下一个优先级队列，需要手动调用start接口

def remove_task_from_queue(task_id: str) -> tuple[bool, str]:
    """
    从队列中移除指定任务
    
    Args:
        task_id: 任务ID
        
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_operation
    
    with queue_lock:
        # 搜索所有队列
        for level, queue in task_queues.items():
            for i, task in enumerate(queue):
                if task.get("task_id") == task_id:
                    # 移除任务
                    del queue[i]
                    
                    # 更新任务状态为取消
                    update_task_status(task_id, "failed")
                    
                    # 如果移除的任务在当前执行的队列中
                    if (current_execution["active"] and 
                        current_execution["current_queue_level"] == level):
                        
                        # 从当前执行队列中移除该任务
                        current_execution["queue_tasks"] = [
                            t for t in current_execution["queue_tasks"] 
                            if t.get("task_id") != task_id
                        ]
                        current_execution["total_count"] = len(current_execution["queue_tasks"])
                        
                        # 如果当前执行队列变空，停止执行并启动下一个队列
                        if current_execution["total_count"] == 0:
                            current_execution.update({
                                "active": False,
                                "current_queue_level": None,
                                "queue_tasks": [],
                                "completed_count": 0,
                                "total_count": 0,
                                "waiting_for_next": False,
                                "command_sent": False,
                                "started": False,
                                "arrived_task": None,
                                "arrived_time": None
                            })
                            # 不再自动启动下一个队列，需要手动调用start接口
                    else:
                        # 如果移除的是待执行队列中的任务，重新发布该队列
                        if task_queues[level]:  # 队列还有其他任务
                            republish_queue_targets(level)
                    
                    log_task_operation(task_id, "removed_from_queue", level)
                    return True, f"Task {task_id} removed from {level} queue"
        
        return False, f"Task {task_id} not found in any queue"

def republish_queue_targets(queue_level: str) -> None:
    """
    重新发布指定队列的所有目标点到ROS2
    
    Args:
        queue_level: 队列级别 (L1/L2/L3)
    """
    from services.log_service import log_task_operation
    
    if not task_queues[queue_level]:
        return
    
    # 构建该队列中所有任务的目标点列表
    targets = []
    for task in task_queues[queue_level]:
        location = get_location_by_id(task["location_id"])
        if location:
            targets.append(location["label"])
    
    if targets:
        success = send_multi_nav_command(targets)
        if success:
            log_task_operation(f"{queue_level}_queue", "republished_targets", f"count_{len(targets)}")
        else:
            log_task_operation(f"{queue_level}_queue", "republish_failed", f"count_{len(targets)}")

def send_next_to_robot() -> tuple[bool, str]:
    """
    发送next指令让机器人继续
    
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    # 检查是否有任务正在等待取件
    if current_execution["arrived_task"] is not None:
        # 检查是否超时
        if current_execution["arrived_time"] is not None:
            elapsed_time = time.time() - current_execution["arrived_time"]
            if elapsed_time < current_execution["pickup_timeout"]:
                return False, f"Cannot send next command: current task is waiting for pickup ({int(current_execution['pickup_timeout'] - elapsed_time)}s remaining)"
            else:
                # 超时，需要降级当前到达的任务
                _handle_pickup_timeout()
    
    if not current_execution["active"]:
        return False, "No active task queue"

    if not current_execution["waiting_for_next"]:
        return False, "Robot is not waiting for next command"
    
    # 检查是否还有更多任务需要执行
    if current_execution["completed_count"] >= current_execution["total_count"]:
        # 所有任务都已经到达完毕，结束当前执行
        queue_level = current_execution["current_queue_level"]
        current_execution.update({
            "active": False,
            "current_queue_level": None,
            "queue_tasks": [],
            "completed_count": 0,
            "total_count": 0,
            "waiting_for_next": False,
            "command_sent": False,
            "started": False,
            "arrived_task": None,
            "arrived_time": None
        })
        
        from services.log_service import log_task_operation
        log_task_operation(f"{queue_level}_queue", "completed_all_tasks", f"all_{current_execution.get('total_count', 0)}_tasks_processed")
        return True, f"{queue_level} queue completed - all tasks have been processed"
    
    # 检查ROS2桥接服务
    bridge_available, bridge_status = check_ros2_bridge_connection()
    if not bridge_available:
        return False, f"Cannot send next command: {bridge_status}"
    
    # 发送next指令
    success, error = send_next_command_with_retry()
    if success:
        current_execution["waiting_for_next"] = False
        queue_level = current_execution["current_queue_level"]
        task_count = current_execution["completed_count"] + 1
        # log_task_operation(f"{queue_level}_queue", "next_command_sent", f"target_{task_count}")
        return True, f"Next command sent for {queue_level} queue, continuing to target {task_count}"
    else:
        return False, f"Failed to send next command to robot: {error}"

def handle_robot_arrival() -> tuple[bool, str]:
    """
    处理机器人到达通知
    - send任务：设置为arrived状态，等待取件
    - call任务：直接完成任务
    
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_operation
    
    if not current_execution["active"]:
        return False, "No active task queue"
    
    queue_level = current_execution["current_queue_level"]
    current_task_index = current_execution["completed_count"]
    
    # 检查是否所有任务都已经处理完毕
    if current_task_index >= current_execution["total_count"]:
        # 所有任务都已经到达过，这种情况不应该发生
        return False, f"All tasks in {queue_level} queue have already been processed"
    
    # 获取当前任务
    current_task = current_execution["queue_tasks"][current_task_index]
    task_type = current_task.get("task_type", "send")  # 默认为send以保持向后兼容
    
    # 完成当前任务计数（表示已到达）
    current_execution["completed_count"] += 1
    
    if task_type == "call":
        # call任务：直接完成，不需要等待取件
        complete_task(current_task["task_id"])
        log_task_operation(current_task["task_id"], "arrived_and_completed", f"call_task_auto_completed")
        log_task_operation(f"{queue_level}_queue", "call_task_completed", f"task_{current_task_index + 1}")
        
        # 检查是否是最后一个任务
        if current_execution["completed_count"] >= current_execution["total_count"]:
            # 所有任务都已完成，结束当前执行
            current_execution.update({
                "active": False,
                "current_queue_level": None,
                "queue_tasks": [],
                "completed_count": 0,
                "total_count": 0,
                "waiting_for_next": False,
                "command_sent": False,
                "started": False,
                "arrived_task": None,
                "arrived_time": None
            })
            
            log_task_operation(f"{queue_level}_queue", "completed_all_tasks", f"final_call_task_{current_task['task_id']}_completed")
            return True, f"Call task {current_task['task_id']} completed automatically, {queue_level} queue finished"
        else:
            # 还有更多任务，设置waiting_for_next状态
            current_execution["waiting_for_next"] = True
            return True, f"Call task {current_task['task_id']} completed automatically at target {current_task_index + 1}/{current_execution['total_count']}"
            
    else:
        # send任务：设置为arrived状态，等待取件
        current_execution["arrived_task"] = current_task
        current_execution["arrived_time"] = time.time()
        current_execution["waiting_for_next"] = True
        
        # 更新任务状态为arrived
        update_task_status(current_task["task_id"], "arrived")
        
        log_task_operation(current_task["task_id"], "arrived_at_target", f"waiting_for_pickup")
        log_task_operation(f"{queue_level}_queue", "arrived_at_target", f"task_{current_task_index + 1}")
        
        # 检查是否是最后一个任务
        if current_execution["completed_count"] >= current_execution["total_count"]:
            return True, f"Robot arrived at final target {current_task_index + 1}/{current_execution['total_count']}, send task {current_task['task_id']} is waiting for pickup (last task in queue)"
        else:
            return True, f"Robot arrived at target {current_task_index + 1}/{current_execution['total_count']}, send task {current_task['task_id']} is waiting for pickup"

def get_queue_status() -> Dict:
    """
    获取当前队列状态
    
    Returns:
        队列状态信息，包含任务详情
    """
    with queue_lock:
        # 检查ROS2桥接状态
        bridge_available, bridge_status = check_ros2_bridge_connection()
        
        # 获取所有队列中的任务详情（等待执行的任务）
        queue_details = {}
        current_executing_task = None
        
        # 构建任务详情的辅助函数
        def build_task_detail(task, status_override=None):
            location = get_location_by_id(task.get('location_id', ''))
            location_label = location.get('label', task.get('location_id', '未知位置')) if location else task.get('location_id', '未知位置')
            
            return {
                "task_id": task.get("task_id", ""),
                "task_type": task.get("task_type", "send"),  # 添加任务类型字段
                "user_id": task.get("initiator", task.get("user_id", "")),
                "receiver": task.get("receiver", ""),
                "location_id": task.get("location_id", ""),
                "location_label": location_label,
                "security_level": task.get("security_level", ""),
                "description": task.get("description", ""),
                "status": status_override or task.get("status", ""),
                "created_at": task.get("timestamps", {}).get("created", task.get("created_at", "")),
                "locker_id": task.get("locker_id", "")  # call任务可能为空
            }
        
        for level in task_queues:
            queue_details[level] = []
            for task in task_queues[level]:  # 直接遍历任务对象而不是task_id
                if task:
                    task_detail = build_task_detail(task)
                    queue_details[level].append(task_detail)
        
        # 获取已完成的任务（从存储文件中加载）
        completed_tasks = []
        all_stored_tasks = load_tasks()
        for task in all_stored_tasks:
            if task.get("status") in ["completed", "failed"]:
                task_detail = build_task_detail(task)
                completed_tasks.append(task_detail)
        
        # 执行队列中的任务详情
        execution_queue_tasks = []
        execution_completed_tasks = []
        
        # 如果有正在执行的队列，获取当前正在执行的任务详情
        if current_execution["active"] and current_execution.get("queue_tasks"):
            # 根据 completed_count 确定当前正在执行的任务
            current_task_index = current_execution["completed_count"]
            
            # 构建执行队列中所有任务的详情
            for i, executing_task in enumerate(current_execution["queue_tasks"]):
                task_detail = build_task_detail(executing_task)
                
                if i < current_task_index:
                    # 已完成的任务
                    task_detail["status"] = "completed_in_queue"
                    task_detail["execution_order"] = i + 1
                    execution_completed_tasks.append(task_detail)
                elif i == current_task_index:
                    # 当前正在执行的任务
                    task_detail["status"] = "executing"
                    task_detail["progress"] = f"{current_execution['completed_count'] + 1}/{current_execution['total_count']}"
                    task_detail["current_target"] = current_task_index + 1
                    task_detail["execution_order"] = i + 1
                    current_executing_task = task_detail
                    execution_queue_tasks.append(task_detail)
                else:
                    # 等待执行的任务
                    task_detail["status"] = "waiting_in_queue"
                    task_detail["execution_order"] = i + 1
                    execution_queue_tasks.append(task_detail)
        
        # 构建到达任务信息
        arrived_task_info = None
        if current_execution["arrived_task"] is not None:
            arrived_task = current_execution["arrived_task"]
            elapsed_time = time.time() - (current_execution["arrived_time"] or 0)
            remaining_time = max(0, current_execution["pickup_timeout"] - elapsed_time)
            
            arrived_task_info = {
                "task_id": arrived_task.get("task_id", ""),
                "location_id": arrived_task.get("location_id", ""),
                "receiver": arrived_task.get("receiver", ""),
                "arrived_at": current_execution["arrived_time"],
                "waiting_time": int(elapsed_time),
                "timeout_remaining": int(remaining_time),
                "status": "arrived"
            }
        
        status = {
            "queues": {
                level: len(queue) for level, queue in task_queues.items()
            },
            "queue_details": queue_details,
            "current_executing_task": current_executing_task,
            "arrived_task": arrived_task_info,
            "execution_queue": {
                "all_tasks": execution_queue_tasks,
                "completed_in_queue": execution_completed_tasks,
                "total_in_queue": len(current_execution.get("queue_tasks", [])),
                "completed_count": len(execution_completed_tasks),
                "remaining_count": len(execution_queue_tasks) - (1 if current_executing_task else 0)
            },
            "completed_tasks": completed_tasks,
            "current_execution": {
                "active": current_execution["active"],
                "current_queue_level": current_execution["current_queue_level"],
                "completed_count": current_execution["completed_count"],
                "total_count": current_execution["total_count"],
                "waiting_for_next": current_execution["waiting_for_next"],
                "command_sent": current_execution["command_sent"],
                "started": current_execution["started"],
                "progress": f"{current_execution['completed_count']}/{current_execution['total_count']}" if current_execution["active"] else "0/0",
                "current_task_index": current_execution["completed_count"] if current_execution["active"] else None,
                "remaining_tasks": current_execution["total_count"] - current_execution["completed_count"] if current_execution["active"] else 0
            },
            "ros2_bridge": {
                "available": bridge_available,
                "status": bridge_status,
                "url": ROS2_BRIDGE_URL
            },
            "summary": {
                "total_pending_tasks": sum(len(queue) for queue in task_queues.values()),
                "executing_tasks": 1 if current_executing_task else 0,
                "completed_tasks": len(completed_tasks),
                "execution_queue_tasks": len(execution_queue_tasks),
                "total_tasks": sum(len(queue) for queue in task_queues.values()) + len(execution_queue_tasks) + len(completed_tasks)
            }
        }
        
        # 更新队列状态，包含降级策略信息
        status["downgrade_strategy"] = get_current_downgrade_strategy()
        
        return status

def _handle_pickup_timeout():
    """
    处理取件超时，将任务降级到下一优先级队列
    使用配置的降级策略进行降级
    """
    from services.log_service import log_task_operation
    
    if current_execution["arrived_task"] is None:
        return
    
    arrived_task = current_execution["arrived_task"]
    original_level = arrived_task.get("security_level", "")
    
    # 使用配置的降级策略确定降级目标队列
    downgrade_target = get_downgrade_target(original_level)
    
    if downgrade_target and downgrade_target != original_level:
        # 更新任务的安全等级为降级后的等级，但保持原始等级不变
        arrived_task["security_level"] = downgrade_target
        # 确保原始安全等级字段存在且不被修改
        if "original_security_level" not in arrived_task:
            arrived_task["original_security_level"] = original_level
        
        # 重置任务状态为pending
        update_task_status(arrived_task["task_id"], "pending")
        
        # 将任务加入降级队列
        task_queues[downgrade_target].append(arrived_task)
        
        log_task_operation(arrived_task["task_id"], "pickup_timeout", 
                         f"downgrade_from_{original_level}_to_{downgrade_target}_strategy_{CURRENT_DOWNGRADE_STRATEGY}")
    elif downgrade_target == original_level:
        # L0级任务在ALL_STEP_DOWN策略下保持原级别，重新加入队列
        update_task_status(arrived_task["task_id"], "pending")
        task_queues[original_level].append(arrived_task)
        
        log_task_operation(arrived_task["task_id"], "pickup_timeout", 
                         f"{original_level}_timeout_readded_to_same_level_strategy_{CURRENT_DOWNGRADE_STRATEGY}")
    else:
        # 无法降级，记录日志
        log_task_operation(arrived_task["task_id"], "pickup_timeout", 
                         f"{original_level}_no_downgrade_available_strategy_{CURRENT_DOWNGRADE_STRATEGY}")
    
    # 清除到达状态
    current_execution["arrived_task"] = None
    current_execution["arrived_time"] = None
    
    # 检查是否是最后一个任务，如果是则结束当前执行
    if (current_execution["active"] and 
        current_execution["completed_count"] >= current_execution["total_count"]):
        # 最后一个任务已超时处理完成，结束当前执行
        queue_level = current_execution["current_queue_level"]
        current_execution.update({
            "active": False,
            "current_queue_level": None,
            "queue_tasks": [],
            "completed_count": 0,
            "total_count": 0,
            "waiting_for_next": False,
            "command_sent": False,
            "started": False,
            "arrived_task": None,
            "arrived_time": None
        })
        
        log_task_operation(f"{queue_level}_queue", "completed_all_tasks", f"final_task_timeout_processed")

def clear_arrived_task(task_id: str) -> tuple[bool, str]:
    """
    清除到达状态（用户完成取件时调用，仅适用于send任务）
    
    Args:
        task_id: 任务ID
        
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_operation
    
    if (current_execution["arrived_task"] is not None and 
        current_execution["arrived_task"]["task_id"] == task_id):
        
        # 验证是否为send任务
        task_type = current_execution["arrived_task"].get("task_type", "send")
        if task_type != "send":
            return False, f"Task {task_id} is a {task_type} task and does not require pickup"
        
        # 清除到达状态
        current_execution["arrived_task"] = None
        current_execution["arrived_time"] = None
        
        # 检查是否是最后一个任务
        if (current_execution["active"] and 
            current_execution["completed_count"] >= current_execution["total_count"]):
            # 最后一个任务已取件完成，结束当前执行
            queue_level = current_execution["current_queue_level"]
            current_execution.update({
                "active": False,
                "current_queue_level": None,
                "queue_tasks": [],
                "completed_count": 0,
                "total_count": 0,
                "waiting_for_next": False,
                "command_sent": False,
                "started": False,
                "arrived_task": None,
                "arrived_time": None
            })
            
            log_task_operation(f"{queue_level}_queue", "completed_all_tasks", f"final_task_{task_id}_pickup_completed")
            log_task_operation(task_id, "pickup_completed", "arrived_state_cleared_final_task")
            return True, f"Arrived state cleared for final send task {task_id}, {queue_level} queue completed"
        else:
            log_task_operation(task_id, "pickup_completed", "arrived_state_cleared")
            return True, f"Arrived state cleared for send task {task_id}"
    
    return False, f"Send task {task_id} is not in arrived state"

def handle_robot_optimized_order(optimized_tasks: List[Dict]) -> tuple[bool, str]:
    """
    处理机器人优化后的任务执行顺序（支持增强的任务信息）
    
    Args:
        optimized_tasks: 优化后的任务列表，包含task_id和优化后的顺序信息
        
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_operation
    
    try:
        # 检查当前是否有活跃的执行队列
        if not current_execution["active"]:
            return False, "No active task queue to reorder"
        
        current_queue_level = current_execution["current_queue_level"]
        current_queue_tasks = current_execution["queue_tasks"]
        
        # 创建任务ID到任务的映射
        task_id_to_task = {}
        for task in current_queue_tasks:
            task_id_to_task[task["task_id"]] = task
        
        # 根据优化后的顺序重新排列任务
        reordered_tasks = []
        for optimized_task in optimized_tasks:
            task_id = optimized_task.get("task_id")
            if task_id in task_id_to_task:
                reordered_tasks.append(task_id_to_task[task_id])
            else:
                log_task_operation(f"{current_queue_level}_queue", "reorder_warning", f"task_{task_id}_not_found")
        
        if len(reordered_tasks) != len(current_queue_tasks):
            return False, f"Task count mismatch: original {len(current_queue_tasks)}, reordered {len(reordered_tasks)}"
        
        # 更新执行队列中的任务顺序
        current_execution["queue_tasks"] = reordered_tasks
        
        # 记录重新排序操作
        original_task_ids = [task["task_id"] for task in current_queue_tasks]
        reordered_task_ids = [task["task_id"] for task in reordered_tasks]
        
        # 记录优化信息（如果有权重信息的话）
        optimization_info = []
        for optimized_task in optimized_tasks:
            if "final_priority" in optimized_task:
                optimization_info.append(f"{optimized_task['task_id']}(priority:{optimized_task['final_priority']})")
        
        optimization_details = f"optimization_details: {optimization_info}" if optimization_info else ""
        
        log_task_operation(f"{current_queue_level}_queue", "tasks_reordered", 
                         f"original_order: {original_task_ids}, optimized_order: {reordered_task_ids}, {optimization_details}")
        
        return True, f"Successfully reordered {len(reordered_tasks)} tasks in {current_queue_level} queue based on enhanced TSP optimization"
        
    except Exception as e:
        return False, f"Error reordering tasks: {str(e)}"



def send_enhanced_multi_nav_command_with_retry(task_details: List[Dict], max_retries: int = 3) -> tuple[bool, str]:
    """
    发送增强的多目标导航命令到ROS2（包含任务详细信息，带重试机制）
    
    Args:
        task_details: 任务详细信息列表，包含位置、权重、等待时间等
        max_retries: 最大重试次数
        
    Returns:
        tuple[bool, str]: (是否发送成功, 错误信息)
    """
    last_error = ""
    
    for attempt in range(max_retries):
        try:
            # 构建增强的ROS2消息格式
            enhanced_data = {
                "tasks": task_details,
                "timestamp": datetime.now().isoformat(),
                "scheduler_version": "enhanced_v1.0"
            }
            
            ros_data = {
                "topic": "/enhanced_multi_nav_command",  # 使用新的话题名
                "type": "std_msgs/String",
                "data": {
                    "data": json.dumps(enhanced_data, ensure_ascii=False)  # 支持中文字符
                }
            }
            
            # 发送HTTP请求到ROS2桥接
            response = requests.post(
                ROS2_BRIDGE_URL,
                json=ros_data,
                headers={'Content-Type': 'application/json'},
                timeout=5.0
            )
            
            if response.status_code == 200:
                return True, ""
            else:
                try:
                    error_data = response.json()
                    last_error = f"HTTP {response.status_code}: {error_data.get('error', response.text)}"
                except:
                    last_error = f"HTTP {response.status_code}: {response.text}"
                
        except requests.exceptions.ConnectionError as e:
            last_error = f"Connection error: ROS2 bridge service not available (attempt {attempt + 1}/{max_retries})"
        except requests.exceptions.Timeout as e:
            last_error = f"Timeout error: ROS2 bridge took too long to respond (attempt {attempt + 1}/{max_retries})"
        except Exception as e:
            last_error = f"Unexpected error: {str(e)} (attempt {attempt + 1}/{max_retries})"
        
        # 如果不是最后一次尝试，等待后重试
        if attempt < max_retries - 1:
            time.sleep(1)
    
    return False, last_error
