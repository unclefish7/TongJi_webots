import json
import os
import random
import string
from typing import Dict, List, Optional
from datetime import datetime
from services.auth_service import consume_auth, get_user_by_id
from services.schema_service import create_entity_from_schema, validate_entity_against_schema
import requests
from threading import Lock
from collections import deque
import time

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

def validate_task_creation(user_id: str, receiver: str, location_id: str, security_level: str) -> tuple[bool, str, str]:
    """
    验证任务创建的前置条件
    
    Args:
        user_id: 发起人ID
        receiver: 接收人ID
        location_id: 目标位置ID
        security_level: 安全等级
    
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
    
    # 移除安全等级验证，现在接受任何security_level值
    
    return True, TaskErrorCodes.SUCCESS, ""

def create_task(user_id: str, receiver: str, location_id: str, security_level: str, description: Optional[str] = None) -> tuple[bool, str, str, Optional[str], Optional[Dict], Optional[str]]:
    """
    创建新任务并添加到队列
    
    Args:
        user_id: 发起人ID
        receiver: 接收人ID
        location_id: 目标位置ID
        security_level: 安全等级
        description: 任务描述
    
    Returns:
        tuple[bool, str, str, Optional[str], Optional[Dict], Optional[str]]: 
        (是否成功, 错误编码, 消息, 任务ID, 任务详细信息, 柜子ID)
    """
    from services.log_service import log_task_creation, log_error
    
    # 验证任务创建条件
    is_valid, error_code, error_msg = validate_task_creation(user_id, receiver, location_id, security_level)
    if not is_valid:
        log_task_creation("", user_id, receiver, location_id, security_level, "", False, error_code)
        return False, error_code, error_msg, None, None, None
    
    # 简化操作：只需要柜子分配
    locker_allocated = False
    locker_id = None
    
    try:
        # 步骤1：查找并分配可用柜子
        available_locker = get_available_locker()
        if not available_locker:
            log_task_creation("", user_id, receiver, location_id, security_level, "", False, TaskErrorCodes.NO_AVAILABLE_LOCKERS)
            return False, TaskErrorCodes.NO_AVAILABLE_LOCKERS, "No available lockers for task creation. Please try again later.", None, None, None
        
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
            "locker_id": locker_id
        }
        
        task_data = create_entity_from_schema("Task", provided_values)
        
        # 验证生成的任务数据
        is_valid, validation_error = validate_entity_against_schema("Task", task_data)
        if not is_valid:
            # 如果验证失败，只需要回滚柜子分配
            release_locker(locker_id)
            log_task_creation("", user_id, receiver, location_id, security_level, locker_id, False, TaskErrorCodes.VALIDATION_FAILED)
            return False, TaskErrorCodes.VALIDATION_FAILED, f"Task data validation failed: {validation_error}", None, None, None
        
        # 步骤3：保存任务到存储文件
        try:
            save_task_to_storage(task_data)
        except Exception as save_error:
            # 如果保存失败，只需要回滚柜子分配
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
        
        success_message = f"Task {task_id} created successfully. Initiator: {initiator.get('name')} ({user_id}), Receiver: {receiver_user.get('name')} ({receiver}), Location: {location.get('label')} ({location_id}), Security Level: {security_level}, Assigned Locker: {locker_id}"
        
        # 在成功创建任务后，添加到队列
        add_task_to_queue(task_data)
    
        return True, TaskErrorCodes.SUCCESS, success_message, task_id, task_data, locker_id
        
    except Exception as e:
        # 如果发生任何未预期的异常，只需要回滚柜子分配
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
    valid_statuses = ["pending", "authenticating", "delivering", "completed", "failed"]
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
    完成任务并释放柜子
    
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
    
    # 释放柜子
    locker_id = task.get('locker_id')
    warning_message = ""
    if locker_id:
        locker_released, locker_message = release_locker(locker_id)
        if locker_released:
            log_locker_operation(locker_id, "released", task_id, True)
        else:
            log_locker_operation(locker_id, "release failed", task_id, False)
            warning_message = f" Warning: Failed to release locker {locker_id}: {locker_message}"
    
    # 获取更新后的任务信息
    updated_task = get_task_by_id(task_id)
    
    success_message = f"Task {task_id} completed successfully{warning_message}"
    return True, TaskErrorCodes.SUCCESS, success_message, updated_task

def fail_task(task_id: str, reason: str = "") -> tuple[bool, str, str, Optional[Dict]]:
    """
    标记任务失败并释放柜子
    
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
    
    # 释放柜子
    locker_id = task.get('locker_id')
    warning_message = ""
    if locker_id:
        locker_released, locker_message = release_locker(locker_id)
        if locker_released:
            log_locker_operation(locker_id, "released", task_id, True)
        else:
            log_locker_operation(locker_id, "release failed", task_id, False)
            warning_message = f" Warning: Could not release locker {locker_id}: {locker_message}"
    
    # 获取更新后的任务信息
    updated_task = get_task_by_id(task_id)
    
    failure_message = f"Task {task_id} marked as failed{warning_message}"
    if reason:
        failure_message += f". Reason: {reason}"
        # 记录失败原因日志
        from services.log_service import log_error
        log_error(f"Task {task_id} failed: {reason}", related_task=task_id)
    
    return True, TaskErrorCodes.SUCCESS, failure_message, updated_task

# 任务队列管理相关代码

# 任务队列（按优先级）
task_queues = {
    "L3": deque(),  # 最高优先级
    "L2": deque(),  # 中等优先级  
    "L1": deque()   # 最低优先级
}

# 队列锁
queue_lock = Lock()

# 当前执行的任务状态
current_execution = {
    "active": False,
    "task_id": None,
    "security_level": None,
    "current_target_index": 0,
    "total_targets": 0,
    "waiting_for_next": False,
    "command_sent": False,  # 标记是否已发送命令
    "started": False        # 标记是否已开始执行
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
    
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_operation
    
    # 首先检查ROS2桥接服务是否可用
    bridge_available, bridge_status = check_ros2_bridge_connection()
    if not bridge_available:
        return False, f"Cannot start task: {bridge_status}. Please ensure ROS2 bridge node is running."
    
    with queue_lock:
        if current_execution["active"]:
            return False, "Another task is already running"
        
        # 获取最高优先级任务
        next_task = get_highest_priority_task()
        if not next_task:
            return False, "No tasks in queue"
        
        task_id = next_task["task_id"]
        security_level = next_task["security_level"]
        location_id = next_task["location_id"]
        
        # 获取位置信息
        location = get_location_by_id(location_id)
        if not location:
            log_task_operation(task_id, "start_failed", "location_not_found")
            return False, f"Location not found for task {task_id}"
        
        # 构建目标点列表（当前只有一个目标点）
        targets = [location["label"]]
        
        # 步骤1：发送多目标导航命令到ROS2
        nav_success, nav_error = send_multi_nav_command_with_retry(targets)
        if not nav_success:
            # 如果发送失败，将任务放回队列
            task_queues[security_level].appendleft(next_task)
            log_task_operation(task_id, "start_failed", "ros2_nav_command_failed")
            return False, f"Failed to send navigation command for task {task_id}: {nav_error}"
        
        # 步骤2：立即发送第一个next指令启动机器人
        next_success, next_error = send_next_command_with_retry()
        if not next_success:
            # 如果next指令发送失败，记录警告但继续执行（机器人可能仍会启动）
            log_task_operation(task_id, "next_command_failed", "initial_start_command")
        
        # 更新执行状态 - 任务已开始执行
        current_execution.update({
            "active": True,
            "task_id": task_id,
            "security_level": security_level,
            "current_target_index": 0,
            "total_targets": len(targets),
            "waiting_for_next": False,  # 不等待，已经发送了启动指令
            "command_sent": True,
            "started": True
        })
        
        # 更新任务状态
        update_task_status(task_id, "delivering")
        log_task_operation(task_id, "started_execution", f"{security_level}_manual_started")
        
        return True, f"Task {task_id} started successfully"

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
                    
                    # 如果移除的是当前正在执行的任务
                    if current_execution["task_id"] == task_id:
                        current_execution["active"] = False
                        current_execution["task_id"] = None
                        current_execution["security_level"] = None
                        current_execution["waiting_for_next"] = False
                        
                        # 启动下一个任务
                        start_next_task()
                    else:
                        # 重新发送当前优先级的队列（不包含已移除的任务）
                        refresh_current_queue()
                    
                    log_task_operation(task_id, "removed_from_queue", level)
                    return True, f"Task {task_id} removed from {level} queue"
        
        return False, f"Task {task_id} not found in any queue"

def get_highest_priority_task() -> Optional[Dict]:
    """
    获取最高优先级的任务
    
    Returns:
        最高优先级的任务，如果没有则返回None
    """
    # 按优先级顺序检查队列
    for level in ["L3", "L2", "L1"]:
        if task_queues[level]:
            return task_queues[level].popleft()
    return None

def start_next_task() -> None:
    """
    启动下一个最高优先级任务（内部使用，任务完成后自动调用）
    """
    from services.log_service import log_task_operation
    
    if current_execution["active"]:
        return
    
    # 获取最高优先级任务
    next_task = get_highest_priority_task()
    if not next_task:
        return
    
    task_id = next_task["task_id"]
    security_level = next_task["security_level"]
    location_id = next_task["location_id"]
    
    # 获取位置信息
    location = get_location_by_id(location_id)
    if not location:
        log_task_operation(task_id, "start_failed", "location_not_found")
        return
    
    # 构建目标点列表（当前只有一个目标点）
    targets = [location["label"]]
    
    # 步骤1：发送多目标导航命令到ROS2
    nav_success = send_multi_nav_command(targets)
    if not nav_success:
        # 如果发送失败，将任务放回队列
        task_queues[security_level].appendleft(next_task)
        log_task_operation(task_id, "start_failed", "ros2_nav_command_failed")
        return
    
    # 步骤2：立即发送第一个next指令启动机器人
    next_success = send_next_command()
    if not next_success:
        # 如果next指令发送失败，记录警告但继续执行（机器人可能仍会启动）
        log_task_operation(task_id, "next_command_failed", "initial_start_command")
    
    # 更新执行状态 - 任务已开始执行
    current_execution.update({
        "active": True,
        "task_id": task_id,
        "security_level": security_level,
        "current_target_index": 0,
        "total_targets": len(targets),
        "waiting_for_next": False,  # 不等待，已经发送了启动指令
        "command_sent": True,
        "started": True
    })
    
    # 更新任务状态
    update_task_status(task_id, "delivering")
    log_task_operation(task_id, "started_execution", f"{security_level}_auto_started")

def complete_current_task() -> None:
    """
    完成当前任务
    """
    from services.log_service import log_task_operation
    
    if current_execution["active"]:
        task_id = current_execution["task_id"]
        
        # 完成任务
        complete_task(task_id)
        
        # 重置执行状态
        current_execution.update({
            "active": False,
            "task_id": None,
            "security_level": None,
            "current_target_index": 0,
            "total_targets": 0,
            "waiting_for_next": False,
            "command_sent": False,
            "started": False
        })
        
        log_task_operation(task_id, "completed_execution", "")
        
        # 启动下一个任务
        start_next_task()

def send_next_to_robot() -> tuple[bool, str]:
    """
    发送next指令让机器人继续
    
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    if not current_execution["active"]:
        return False, "No active task"
    
    if not current_execution["waiting_for_next"]:
        return False, "Robot is not waiting for next command"
    
    # 检查ROS2桥接服务
    bridge_available, bridge_status = check_ros2_bridge_connection()
    if not bridge_available:
        return False, f"Cannot send next command: {bridge_status}"
    
    # 发送next指令
    success, error = send_next_command_with_retry()
    if success:
        current_execution["waiting_for_next"] = False
        task_id = current_execution["task_id"]
        log_task_operation(task_id, "next_command_sent", f"target_{current_execution['current_target_index']}")
        return True, f"Next command sent for task {task_id}, continuing to next target"
    else:
        return False, f"Failed to send next command to robot: {error}"

def refresh_current_queue() -> None:
    """
    重新发送当前优先级队列到ROS2（用于任务取消后的队列更新）
    """
    # 找到当前最高优先级的非空队列
    current_targets = []
    for level in ["L3", "L2", "L1"]:
        if task_queues[level]:
            # 收集该级别所有任务的目标点
            for task in task_queues[level]:
                location = get_location_by_id(task["location_id"])
                if location:
                    current_targets.append(location["label"])
            break
    
    if current_targets:
        send_multi_nav_command(current_targets)

def handle_robot_arrival() -> tuple[bool, str]:
    """
    处理机器人到达通知
    
    Returns:
        tuple[bool, str]: (是否成功, 消息)
    """
    from services.log_service import log_task_operation
    
    if not current_execution["active"]:
        return False, "No active task"
    
    task_id = current_execution["task_id"]
    
    # 检查是否完成了所有目标点
    if current_execution["current_target_index"] >= current_execution["total_targets"] - 1:
        # 任务完成
        complete_current_task()
        return True, f"Task {task_id} completed successfully"
    else:
        # 还有更多目标点，等待next指令
        current_execution["waiting_for_next"] = True
        current_execution["current_target_index"] += 1
        
        log_task_operation(task_id, "arrived_at_intermediate", str(current_execution["current_target_index"]))
        return True, f"Robot arrived at target {current_execution['current_target_index']}, waiting for next command"

def get_queue_status() -> Dict:
    """
    获取当前队列状态
    
    Returns:
        队列状态信息
    """
    with queue_lock:
        # 检查ROS2桥接状态
        bridge_available, bridge_status = check_ros2_bridge_connection()
        
        status = {
            "queues": {
                level: len(queue) for level, queue in task_queues.items()
            },
            "current_execution": current_execution.copy(),
            "ros2_bridge": {
                "available": bridge_available,
                "status": bridge_status,
                "url": ROS2_BRIDGE_URL
            }
        }
        return status
