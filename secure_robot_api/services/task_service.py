import json
import os
from typing import Dict, List, Optional
from datetime import datetime
from services.auth_service import consume_auth, get_user_by_id
from services.schema_service import create_entity_from_schema, validate_entity_against_schema

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
    lockers = load_lockers()
    
    for locker in lockers:
        if locker.get('locker_id') == locker_id:
            if locker.get('status') == 'available':
                locker['status'] = 'in_use'
                try:
                    save_lockers(lockers)
                    return True, ""
                except Exception as e:
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
    lockers = load_lockers()
    
    for locker in lockers:
        if locker.get('locker_id') == locker_id:
            if locker.get('status') == 'in_use':
                locker['status'] = 'available'
                try:
                    save_lockers(lockers)
                    return True, ""
                except Exception as e:
                    return False, f"Failed to release locker: {str(e)}"
            else:
                return False, f"Locker {locker_id} is not in use"
    
    return False, f"Locker {locker_id} not found"

def generate_task_id() -> str:
    """生成任务ID"""
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    return f"T{timestamp}"

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
    
    # 验证安全等级格式
    if security_level not in ["L1", "L2", "L3"]:
        return False, TaskErrorCodes.INVALID_SECURITY_LEVEL, f"Invalid security level '{security_level}'. Valid values: L1, L2, L3"
    
    return True, TaskErrorCodes.SUCCESS, ""

def create_task(user_id: str, receiver: str, location_id: str, security_level: str, description: Optional[str] = None) -> tuple[bool, str, str, Optional[str], Optional[Dict], Optional[str]]:
    """
    创建新任务
    
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
    # 验证任务创建条件
    is_valid, error_code, error_msg = validate_task_creation(user_id, receiver, location_id, security_level)
    if not is_valid:
        return False, error_code, error_msg, None, None, None
    
    # 尝试消费认证记录
    auth_consumed = consume_auth(user_id, security_level)
    if not auth_consumed:
        return False, TaskErrorCodes.INSUFFICIENT_AUTH, f"Insufficient authentication for security level {security_level}. Please authenticate first.", None, None, None
    
    # 查找可用柜子
    available_locker = get_available_locker()
    if not available_locker:
        return False, TaskErrorCodes.NO_AVAILABLE_LOCKERS, "No available lockers for task creation. Please try again later.", None, None, None
    
    locker_id = available_locker.get('locker_id')
    
    # 分配柜子
    allocation_success, allocation_error = allocate_locker(locker_id)
    if not allocation_success:
        return False, TaskErrorCodes.LOCKER_ALLOCATION_FAILED, f"Failed to allocate locker {locker_id}: {allocation_error}", None, None, None
    
    # 基于schema构建任务数据
    try:
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
            # 如果验证失败，释放柜子
            release_locker(locker_id)
            return False, TaskErrorCodes.VALIDATION_FAILED, f"Task data validation failed: {validation_error}", None, None, None
        
        # 保存任务到存储文件
        save_task_to_storage(task_data)
        
        task_id = task_data.get("task_id")
        
        # 获取用户和位置信息用于成功消息
        initiator = get_user_by_id(user_id)
        receiver_user = get_user_by_id(receiver)
        location = get_location_by_id(location_id)
        
        success_message = f"Task {task_id} created successfully. Initiator: {initiator.get('name')} ({user_id}), Receiver: {receiver_user.get('name')} ({receiver}), Location: {location.get('label')} ({location_id}), Security Level: {security_level}, Assigned Locker: {locker_id}"
        
        return True, TaskErrorCodes.SUCCESS, success_message, task_id, task_data, locker_id
        
    except Exception as e:
        # 如果创建失败，释放柜子
        release_locker(locker_id)
        return False, TaskErrorCodes.SAVE_FAILED, f"Failed to create task: {str(e)}", None, None, None

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
    # 验证状态值
    valid_statuses = ["pending", "authenticating", "delivering", "completed", "failed"]
    if new_status not in valid_statuses:
        return False, f"Invalid status: {new_status}"
    
    # 加载任务
    tasks = load_tasks()
    
    # 查找并更新任务
    task_found = False
    for task in tasks:
        if task.get('task_id') == task_id:
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
        return True, f"Task {task_id} status updated to {new_status}"
    except Exception as e:
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
        if not locker_released:
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
        if not locker_released:
            warning_message = f" Warning: Could not release locker {locker_id}: {locker_message}"
    
    # 获取更新后的任务信息
    updated_task = get_task_by_id(task_id)
    
    failure_message = f"Task {task_id} marked as failed{warning_message}"
    if reason:
        failure_message += f". Reason: {reason}"
    
    return True, TaskErrorCodes.SUCCESS, failure_message, updated_task
