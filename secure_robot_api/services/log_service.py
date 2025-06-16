import json
import os
from typing import Dict, List, Optional
from datetime import datetime
from services.schema_service import create_entity_from_schema, validate_entity_against_schema

def load_logs() -> List[Dict]:
    """从 logs.json 加载日志数据"""
    logs_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'logs.json')
    try:
        with open(logs_path, 'r', encoding='utf-8') as f:
            content = f.read().strip()
            if not content:
                return []
            return json.loads(content)
    except (FileNotFoundError, json.JSONDecodeError):
        return []

def save_logs(logs: List[Dict]) -> None:
    """保存日志数据到 logs.json"""
    logs_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'logs.json')
    try:
        with open(logs_path, 'w', encoding='utf-8') as f:
            json.dump(logs, f, ensure_ascii=False, indent=2)
    except Exception as e:
        raise Exception(f"Failed to save logs: {str(e)}")

def create_log(log_type: str, message: str, related_user: Optional[str] = None, 
               related_task: Optional[str] = None, related_locker: Optional[str] = None,
               auth_mode: Optional[List[str]] = None, result: Optional[str] = None) -> bool:
    """
    创建日志记录
    
    Args:
        log_type: 日志类型 (access/auth/delivery/system/error)
        message: 日志消息
        related_user: 相关用户ID
        related_task: 相关任务ID
        related_locker: 相关柜子ID
        auth_mode: 认证方式列表
        result: 操作结果 (success/failed/partial/info)
    
    Returns:
        bool: 是否成功创建日志
    """
    try:
        # 基于schema构建日志数据
        provided_values = {
            "log_type": log_type,
            "message": message,
            "related_user": related_user,
            "related_task": related_task,
            "related_locker": related_locker,
            "auth_mode": auth_mode,
            "result": result,
            "timestamp": datetime.now().isoformat()
        }
        
        log_data = create_entity_from_schema("Log", provided_values)
        
        # 验证生成的日志数据
        is_valid, validation_error = validate_entity_against_schema("Log", log_data)
        if not is_valid:
            print(f"Warning: Log data validation failed: {validation_error}")
            return False
        
        # 加载现有日志
        logs = load_logs()
        
        # 添加新日志
        logs.append(log_data)
        
        # 保存日志
        save_logs(logs)
        
        return True
        
    except Exception as e:
        print(f"Error creating log: {str(e)}")
        return False

def log_auth_attempt(user_id: str, requested_level: str, methods: List[str], 
                    success: bool, verified_level: str = "") -> bool:
    """记录认证尝试日志"""
    result = "success" if success else "failed"
    message = f"User {user_id} authentication attempt for level {requested_level}"
    if success:
        message += f", verified at level {verified_level}"
    else:
        message += " failed"
    
    return create_log(
        log_type="auth",
        message=message,
        related_user=user_id,
        auth_mode=methods,
        result=result
    )

def log_task_creation(task_id: str, initiator: str, receiver: str, location_id: str, 
                     security_level: str, locker_id: str, success: bool, 
                     error_code: str = "") -> bool:
    """记录任务创建日志"""
    result = "success" if success else "failed"
    message = f"Task creation attempt by {initiator} for {receiver} at {location_id}"
    if success:
        message += f", task {task_id} created with locker {locker_id}"
    else:
        message += f" failed with code {error_code}"
    
    return create_log(
        log_type="delivery",
        message=message,
        related_user=initiator,
        related_task=task_id if success else None,
        related_locker=locker_id if success else None,
        result=result
    )

def log_task_status_change(task_id: str, old_status: str, new_status: str, 
                          user_id: Optional[str] = None) -> bool:
    """记录任务状态变更日志"""
    message = f"Task {task_id} status changed from {old_status} to {new_status}"
    if user_id:
        message += f" by user {user_id}"
    
    return create_log(
        log_type="delivery",
        message=message,
        related_user=user_id,
        related_task=task_id,
        result="success"
    )

def log_locker_operation(locker_id: str, operation: str, task_id: Optional[str] = None,
                        success: bool = True) -> bool:
    """记录柜子操作日志"""
    result = "success" if success else "failed"
    message = f"Locker {locker_id} {operation}"
    if task_id:
        message += f" for task {task_id}"
    
    return create_log(
        log_type="system",
        message=message,
        related_task=task_id,
        related_locker=locker_id,
        result=result
    )

def log_task_operation(task_id: str, operation: str, details: str = "", 
                      success: bool = True, user_id: Optional[str] = None) -> bool:
    """记录任务操作日志"""
    result = "success" if success else "failed"
    message = f"Task {task_id} operation: {operation}"
    if details:
        message += f" - {details}"
    
    return create_log(
        log_type="delivery",
        message=message,
        related_user=user_id,
        related_task=task_id,
        result=result
    )

def log_error(message: str, related_user: Optional[str] = None, 
              related_task: Optional[str] = None) -> bool:
    """记录错误日志"""
    return create_log(
        log_type="error",
        message=message,
        related_user=related_user,
        related_task=related_task,
        result="failed"
    )
