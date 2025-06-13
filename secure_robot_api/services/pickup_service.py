import json
import os
from typing import Dict, List, Optional, Tuple
from datetime import datetime
from services.auth_service import is_pickup_auth_valid, get_pickup_auth_status, LEVEL_ORDER
from services.task_service import load_tasks, save_tasks, load_lockers, save_lockers, get_task_by_id
from services.log_service import create_log

def get_user_pickup_tasks(user_id: str) -> List[Dict]:
    """
    获取用户可取件的任务列表
    
    Args:
        user_id: 用户ID
    
    Returns:
        List[Dict]: 可取件任务列表
    """
    tasks = load_tasks()
    pickup_tasks = []
    
    for task in tasks:
        if (task.get('receiver') == user_id and 
            task.get('status') == 'arrived'):
            pickup_tasks.append({
                'task_id': task.get('task_id'),
                'description': task.get('description', ''),
                'security_level': task.get('security_level'),
                'locker_id': task.get('locker_id'),
                'initiator': task.get('initiator'),
                'timestamps': task.get('timestamps', {})
            })
    
    return pickup_tasks

def validate_pickup_request(user_id: str, task_id: str) -> Tuple[bool, str, Optional[Dict]]:
    """
    验证取件请求
    
    Args:
        user_id: 用户ID
        task_id: 任务ID
    
    Returns:
        Tuple[bool, str, Optional[Dict]]: (是否有效, 错误信息, 任务信息)
    """
    # 查找任务
    task = get_task_by_id(task_id)
    if not task:
        return False, f"任务 '{task_id}' 不存在", None
    
    # 验证接收人
    if task.get('receiver') != user_id:
        return False, f"任务 '{task_id}' 的接收人不是当前用户", None
    
    # 验证任务状态
    if task.get('status') != 'arrived':
        current_status = task.get('status', 'unknown')
        return False, f"任务 '{task_id}' 状态为 '{current_status}'，无法取件（需要状态为 'arrived'）", None
    
    return True, "", task

def validate_pickup_auth(user_id: str, required_level: str) -> Tuple[bool, str, Optional[Dict]]:
    """
    验证取件认证
    
    Args:
        user_id: 用户ID
        required_level: 所需认证等级
    
    Returns:
        Tuple[bool, str, Optional[Dict]]: (是否有效, 错误信息, 认证信息)
    """
    # 获取认证状态
    auth_status = get_pickup_auth_status(user_id)
    if not auth_status:
        return False, "未找到有效的取件认证，请先进行认证", None
    
    # 检查认证等级
    verified_level = auth_status.get('verified_level')
    required_level_num = LEVEL_ORDER.get(required_level, 0)
    verified_level_num = LEVEL_ORDER.get(verified_level, 0)
    
    if verified_level_num < required_level_num:
        return False, f"认证等级不足，需要 {required_level} 级认证，当前为 {verified_level} 级", None
    
    # 检查是否过期（这个检查在get_pickup_auth_status中已经做了，这里是双重保险）
    expires_at = datetime.fromisoformat(auth_status.get('expires_at'))
    if datetime.now() > expires_at:
        return False, "取件认证已过期，请重新认证", None
    
    return True, "", auth_status

def execute_pickup(user_id: str, task_id: str) -> Tuple[bool, str, str]:
    """
    执行取件操作
    
    Args:
        user_id: 用户ID
        task_id: 任务ID
    
    Returns:
        Tuple[bool, str, str]: (是否成功, 状态码, 消息)
    """
    try:
        # 验证取件请求
        is_valid, error_msg, task = validate_pickup_request(user_id, task_id)
        if not is_valid:
            return False, "PICKUP_001", error_msg
        
        # 验证取件认证
        auth_valid, auth_error, auth_info = validate_pickup_auth(user_id, task.get('security_level'))
        if not auth_valid:
            return False, "PICKUP_002", auth_error
        
        # 更新任务状态
        success = update_task_status_to_completed(task_id)
        if not success:
            return False, "PICKUP_003", f"更新任务 '{task_id}' 状态失败"
        
        # 释放柜子
        locker_id = task.get('locker_id')
        if locker_id:
            locker_success = release_locker_for_pickup(locker_id)
            if not locker_success:
                # 任务已完成，但柜子释放失败，记录警告
                warning_msg = f"任务完成但柜子 '{locker_id}' 释放失败"
                create_log(
                    log_type="error",
                    message=warning_msg,
                    related_user=user_id,
                    related_task=task_id,
                    related_locker=locker_id,
                    result="partial"
                )
        
        # 记录成功取件日志
        create_log(
            log_type="access",
            message=f"用户 {user_id} 成功取件，任务 {task_id}",
            related_user=user_id,
            related_task=task_id,
            related_locker=locker_id,
            auth_mode=auth_info.get('methods', []),
            result="success"
        )
        
        return True, "PICKUP_000", f"取件成功，任务 '{task_id}' 已完成"
        
    except Exception as e:
        error_msg = f"取件操作出现异常: {str(e)}"
        create_log(
            log_type="error",
            message=error_msg,
            related_user=user_id,
            related_task=task_id,
            result="failed"
        )
        return False, "PICKUP_999", error_msg

def update_task_status_to_completed(task_id: str) -> bool:
    """
    更新任务状态为已完成
    
    Args:
        task_id: 任务ID
    
    Returns:
        bool: 是否成功
    """
    try:
        tasks = load_tasks()
        
        for task in tasks:
            if task.get('task_id') == task_id:
                task['status'] = 'completed'
                # 更新交付时间戳
                if 'timestamps' not in task:
                    task['timestamps'] = {}
                task['timestamps']['delivered'] = datetime.now().isoformat()
                break
        else:
            return False
        
        save_tasks(tasks)
        return True
        
    except Exception as e:
        print(f"更新任务状态失败: {str(e)}")
        return False

def release_locker_for_pickup(locker_id: str) -> bool:
    """
    释放取件后的柜子
    
    Args:
        locker_id: 柜子ID
    
    Returns:
        bool: 是否成功
    """
    try:
        lockers = load_lockers()
        
        for locker in lockers:
            if locker.get('locker_id') == locker_id:
                locker['status'] = 'available'
                break
        else:
            return False
        
        save_lockers(lockers)
        return True
        
    except Exception as e:
        print(f"释放柜子失败: {str(e)}")
        return False
