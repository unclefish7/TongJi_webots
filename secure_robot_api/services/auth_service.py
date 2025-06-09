import json
import os
from typing import Dict, List, Tuple, Optional
from datetime import datetime

# 认证等级顺序
LEVEL_ORDER = {"L1": 1, "L2": 2, "L3": 3}

# 运行时认证缓存 - 修改为列表结构支持多认证记录
auth_session_cache: Dict[str, List[Dict]] = {}

def load_users() -> List[Dict]:
    """从 users.json 加载用户数据"""
    users_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'users.json')
    try:
        with open(users_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return []

def get_user_by_id(user_id: str) -> Optional[Dict]:
    """根据用户ID获取用户信息"""
    users = load_users()
    for user in users:
        if user.get('user_id') == user_id:
            return user
    return None

def verify_auth(user_id: str, requested_level: str, provided: Dict[str, str]) -> Tuple[bool, str, List[str]]:
    """
    验证用户认证 - 基于请求的认证等级进行验证
    
    Args:
        user_id: 用户ID（员工卡号）
        requested_level: 请求的认证等级 (L1/L2/L3)
        provided: 提供的认证信息，可能包含 l2_auth、l3_auth
    
    Returns:
        Tuple[bool, str, List[str]]: (是否认证成功, 验证等级, 使用的认证方式)
    """
    from services.log_service import log_auth_attempt
    
    # 获取用户信息
    user = get_user_by_id(user_id)
    if not user:
        log_auth_attempt(user_id, requested_level, ['ID'], False)
        return False, "", []
    
    user_auth_level = user.get('auth_level', '')
    requested_level_num = LEVEL_ORDER.get(requested_level, 0)
    user_level_num = LEVEL_ORDER.get(user_auth_level, 0)
    
    # 检查请求的认证等级是否超过用户本身的等级
    if requested_level_num > user_level_num:
        log_auth_attempt(user_id, requested_level, ['ID'], False)
        return False, "", []
    
    methods_used = ['ID']  # 基础ID验证
    
    # L1: 只需 user_id 匹配即可
    if requested_level == 'L1':
        success, verified_level = True, 'L1'
    
    # L2: 需要验证 L2级认证信息
    elif requested_level == 'L2':
        provided_l2_auth = provided.get('l2_auth', '')
        user_l2_auth = user.get('l2_auth', '')
        
        if provided_l2_auth and provided_l2_auth == user_l2_auth:
            methods_used.append('L2')
            success, verified_level = True, 'L2'
        else:
            success, verified_level = False, ''
    
    # L3: 需要验证 L2级认证信息 + L3级认证信息
    elif requested_level == 'L3':
        provided_l2_auth = provided.get('l2_auth', '')
        provided_l3_auth = provided.get('l3_auth', '')
        user_l2_auth = user.get('l2_auth', '')
        user_l3_auth = user.get('l3_auth', '')
        
        l2_valid = provided_l2_auth and provided_l2_auth == user_l2_auth
        l3_valid = provided_l3_auth and provided_l3_auth == user_l3_auth
        
        if l2_valid and l3_valid:
            methods_used.extend(['L2', 'L3'])
            success, verified_level = True, 'L3'
        else:
            success, verified_level = False, ''
    
    else:
        # 未知认证等级
        success, verified_level = False, ''
    
    # 记录认证尝试日志
    log_auth_attempt(user_id, requested_level, methods_used, success, verified_level)
    
    # 如果认证成功，添加到缓存
    if success:
        add_auth_record(user_id, verified_level, methods_used)
    
    return success, verified_level, methods_used

def add_auth_record(user_id: str, level: str, methods: List[str]) -> None:
    """添加认证记录到缓存"""
    if user_id not in auth_session_cache:
        auth_session_cache[user_id] = []
    
    auth_record = {
        "level": level,
        "methods": methods,
        "timestamp": datetime.now().isoformat(),
        "used": False
    }
    
    auth_session_cache[user_id].append(auth_record)

def consume_auth(user_id: str, required_level: str) -> bool:
    """
    消费认证记录
    
    Args:
        user_id: 用户ID
        required_level: 所需认证等级
    
    Returns:
        bool: 是否成功消费认证记录
    """
    if user_id not in auth_session_cache:
        return False
    
    user_records = auth_session_cache[user_id]
    required_level_num = LEVEL_ORDER.get(required_level, 0)
    
    # 找到所有未使用且等级满足要求的认证记录
    valid_records = []
    for i, record in enumerate(user_records):
        if (not record["used"] and 
            LEVEL_ORDER.get(record["level"], 0) >= required_level_num):
            valid_records.append((i, record))
    
    if not valid_records:
        return False
    
    # 按等级排序，优先消费等级最低的记录
    valid_records.sort(key=lambda x: LEVEL_ORDER.get(x[1]["level"], 0))
    
    # 消费第一个（等级最低的）记录
    record_index, record = valid_records[0]
    auth_session_cache[user_id][record_index]["used"] = True
    
    return True

def rollback_auth_consumption(user_id: str, security_level: str) -> bool:
    """
    回滚认证记录消费 - 将最近消费的认证记录标记为未使用
    
    Args:
        user_id: 用户ID
        security_level: 安全等级
    
    Returns:
        bool: 是否成功回滚
    """
    if user_id not in auth_session_cache:
        return False
    
    user_records = auth_session_cache[user_id]
    required_level_num = LEVEL_ORDER.get(security_level, 0)
    
    # 找到最近使用的满足要求的认证记录并恢复
    # 从后往前查找，因为最近的操作在列表末尾
    for i in range(len(user_records) - 1, -1, -1):
        record = user_records[i]
        if (record["used"] and 
            LEVEL_ORDER.get(record["level"], 0) >= required_level_num):
            record["used"] = False
            return True
    
    return False

def get_auth_records(user_id: str) -> List[Dict]:
    """获取用户的所有认证记录"""
    return auth_session_cache.get(user_id, [])

def clear_used_auth_records(user_id: str) -> None:
    """清理已使用的认证记录"""
    if user_id in auth_session_cache:
        auth_session_cache[user_id] = [
            record for record in auth_session_cache[user_id] 
            if not record["used"]
        ]
        
        # 如果没有记录了，删除用户条目
        if not auth_session_cache[user_id]:
            del auth_session_cache[user_id]

def get_auth_session_status(user_id: str) -> Dict[str, any]:
    """
    获取用户认证会话状态（用于调试和监控）
    
    Args:
        user_id: 用户ID
    
    Returns:
        Dict: 认证状态信息
    """
    if user_id not in auth_session_cache:
        return {
            "user_id": user_id,
            "total_records": 0,
            "available_records": 0,
            "used_records": 0,
            "records": []
        }
    
    records = auth_session_cache[user_id]
    available_count = sum(1 for r in records if not r["used"])
    used_count = sum(1 for r in records if r["used"])
    
    return {
        "user_id": user_id,
        "total_records": len(records),
        "available_records": available_count,
        "used_records": used_count,
        "records": records
    }
