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

def generate_task_id() -> str:
    """生成任务ID"""
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    return f"T{timestamp}"

def validate_task_creation(user_id: str, receiver: str, location_id: str, security_level: str) -> tuple[bool, str]:
    """
    验证任务创建的前置条件
    
    Args:
        user_id: 发起人ID
        receiver: 接收人ID
        location_id: 目标位置ID
        security_level: 安全等级
    
    Returns:
        tuple[bool, str]: (是否验证通过, 错误信息)
    """
    # 验证发起人是否存在
    initiator = get_user_by_id(user_id)
    if not initiator:
        return False, "Initiator user not found"
    
    # 验证接收人是否存在
    receiver_user = get_user_by_id(receiver)
    if not receiver_user:
        return False, "Receiver user not found"
    
    # 验证位置是否存在
    location = get_location_by_id(location_id)
    if not location:
        return False, "Target location not found"
    
    # 验证安全等级格式
    if security_level not in ["L1", "L2", "L3"]:
        return False, "Invalid security level"
    
    return True, ""

def create_task(user_id: str, receiver: str, location_id: str, security_level: str, description: Optional[str] = None) -> tuple[bool, str, Optional[str]]:
    """
    创建新任务
    
    Args:
        user_id: 发起人ID
        receiver: 接收人ID
        location_id: 目标位置ID
        security_level: 安全等级
        description: 任务描述
    
    Returns:
        tuple[bool, str, Optional[str]]: (是否成功, 消息, 任务ID)
    """
    # 验证任务创建条件
    is_valid, error_msg = validate_task_creation(user_id, receiver, location_id, security_level)
    if not is_valid:
        return False, error_msg, None
    
    # 尝试消费认证记录
    auth_consumed = consume_auth(user_id, security_level)
    if not auth_consumed:
        return False, f"Insufficient authentication for security level {security_level}", None
    
    # 基于schema构建任务数据
    try:
        provided_values = {
            "description": description,
            "initiator": user_id,
            "receiver": receiver,
            "location_id": location_id,
            "security_level": security_level
        }
        
        task_data = create_entity_from_schema("Task", provided_values)
        
        # 验证生成的任务数据
        is_valid, validation_error = validate_entity_against_schema("Task", task_data)
        if not is_valid:
            return False, f"Task data validation failed: {validation_error}", None
        
        # 保存任务到存储文件
        save_task_to_storage(task_data)
        
        task_id = task_data.get("task_id")
        return True, f"Task created successfully with security level {security_level}", task_id
        
    except Exception as e:
        return False, f"Failed to create task: {str(e)}", None

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
