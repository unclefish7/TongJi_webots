import json
import random
import os
from datetime import datetime, timedelta
from typing import Dict, Any, List, Optional

# 全局配置
LOGS_COUNT = 100

def load_schema() -> Dict[str, Any]:
    """加载 schema 配置"""
    schema_path = 'schema/schema_reference.json'
    with open(schema_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def load_related_data() -> Dict[str, List]:
    """加载相关数据"""
    data = {
        'users': [],
        'tasks': [],
        'lockers': []
    }
    
    try:
        with open('storage/users.json', 'r', encoding='utf-8') as f:
            data['users'] = json.load(f)
    except FileNotFoundError:
        print("警告: users.json 不存在")
    
    try:
        with open('storage/tasks.json', 'r', encoding='utf-8') as f:
            data['tasks'] = json.load(f)
    except FileNotFoundError:
        print("警告: tasks.json 不存在")
    
    try:
        with open('storage/lockers.json', 'r', encoding='utf-8') as f:
            data['lockers'] = json.load(f)
    except FileNotFoundError:
        print("警告: lockers.json 不存在")
    
    return data

def generate_log_id(base_time: datetime) -> str:
    """根据 schema 格式生成日志ID"""
    return f"LOG{base_time.strftime('%Y%m%d%H%M%S')}"

def generate_log_message(log_type: str, result: Optional[str] = None) -> str:
    """根据日志类型生成相应的消息"""
    messages = {
        "access": [
            "用户尝试访问安全区域",
            "用户进入受限位置",
            "访问权限验证",
            "门禁系统触发"
        ],
        "auth": [
            "用户身份认证请求",
            "多因子认证验证",
            "权限等级检查",
            "认证失败重试"
        ],
        "delivery": [
            "机器人开始配送任务",
            "配送任务状态更新",
            "到达目标位置",
            "配送完成确认"
        ],
        "system": [
            "系统启动完成",
            "定期状态检查",
            "系统维护操作",
            "配置更新"
        ],
        "error": [
            "系统异常检测",
            "网络连接中断",
            "硬件故障报告",
            "数据同步失败"
        ]
    }
    
    base_message = random.choice(messages.get(log_type, ["未知操作"]))
    
    if result:
        result_suffix = {
            "success": "操作成功",
            "failed": "操作失败",
            "partial": "部分成功",
            "info": "信息记录"
        }
        base_message += f" - {result_suffix.get(result, '')}"
    
    return base_message

def generate_logs(count: int) -> list:
    """生成日志数据"""
    schema = load_schema()
    log_schema = schema['Log']
    
    related_data = load_related_data()
    
    user_ids = [user['user_id'] for user in related_data['users']]
    task_ids = [task['task_id'] for task in related_data['tasks']]
    locker_ids = [locker['locker_id'] for locker in related_data['lockers']]
    
    logs = []
    used_ids = set()
    
    # 生成日志时间范围（最近30天）
    end_time = datetime.now()
    start_time = end_time - timedelta(days=30)
    
    for _ in range(count):
        # 生成随机时间
        random_seconds = random.randint(0, int((end_time - start_time).total_seconds()))
        log_time = start_time + timedelta(seconds=random_seconds)
        
        # 生成唯一的日志ID
        while True:
            log_id = generate_log_id(log_time)
            if log_id not in used_ids:
                used_ids.add(log_id)
                break
            else:
                log_time += timedelta(seconds=1)
        
        log_type = random.choice(log_schema['log_type']['values'])
        result = random.choice(log_schema['result']['values'])
        
        log = {
            "log_id": log_id,
            "log_type": log_type,
            "message": generate_log_message(log_type, result),
            "result": result,
            "timestamp": log_time.isoformat()
        }
        
        # 可选的关联字段
        if user_ids and random.random() > 0.3:  # 70% 概率关联用户
            log["related_user"] = random.choice(user_ids)
        
        if task_ids and random.random() > 0.6:  # 40% 概率关联任务
            log["related_task"] = random.choice(task_ids)
        
        if locker_ids and random.random() > 0.7:  # 30% 概率关联柜体
            log["related_locker"] = random.choice(locker_ids)
        
        # 为 access/auth 类型日志添加认证方式
        if log_type in ["access", "auth"] and random.random() > 0.2:
            auth_modes = log_schema['auth_mode']['items']['values']
            num_modes = random.randint(1, min(3, len(auth_modes)))
            log["auth_mode"] = random.sample(auth_modes, num_modes)
        
        logs.append(log)
    
    # 按时间排序
    logs.sort(key=lambda x: x['timestamp'])
    
    return logs

def main():
    """主函数"""
    logs = generate_logs(LOGS_COUNT)
    
    # 保存到文件
    output_path = 'storage/logs.json'
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(logs, f, ensure_ascii=False, indent=2)
    
    print(f"已生成 {len(logs)} 个日志数据到 {output_path}")

if __name__ == "__main__":
    main()
