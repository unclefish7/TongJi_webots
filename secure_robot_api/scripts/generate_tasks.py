import json
import random
import os
from datetime import datetime, timedelta
from typing import Dict, Any, List

# 全局配置
TASKS_COUNT = 30

def load_schema() -> Dict[str, Any]:
    """加载 schema 配置"""
    schema_path = 'schema/schema_reference.json'
    with open(schema_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def load_users() -> List[Dict]:
    """加载用户数据"""
    users_path = 'storage/users.json'
    try:
        with open(users_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        print("警告: users.json 不存在，请先运行 generate_users.py")
        return []

def load_locations() -> List[Dict]:
    """加载位置数据"""
    locations_path = 'storage/locations.json'
    try:
        with open(locations_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        print("警告: locations.json 不存在，请先运行 generate_locations.py")
        return []

def generate_task_id(base_time: datetime) -> str:
    """根据 schema 格式生成任务ID"""
    return f"T{base_time.strftime('%Y%m%d%H%M%S')}"

def generate_timestamps(status: str, created_time: datetime) -> Dict:
    """根据任务状态生成相应的时间戳"""
    timestamps = {
        "created": created_time.isoformat()
    }
    
    if status in ["authenticating", "delivering", "completed", "failed"]:
        accepted_time = created_time + timedelta(minutes=random.randint(1, 30))
        timestamps["accepted"] = accepted_time.isoformat()
        
        if status in ["completed", "failed"]:
            delivered_time = accepted_time + timedelta(minutes=random.randint(5, 120))
            timestamps["delivered"] = delivered_time.isoformat()
    
    return timestamps

def generate_tasks(count: int) -> list:
    """生成任务数据"""
    schema = load_schema()
    task_schema = schema['Task']
    
    users = load_users()
    locations = load_locations()
    
    if not users or not locations:
        return []
    
    user_ids = [user['user_id'] for user in users]
    location_ids = [loc['location_id'] for loc in locations]
    
    tasks = []
    used_ids = set()
    
    # 生成任务时间范围（最近30天）
    end_time = datetime.now()
    start_time = end_time - timedelta(days=30)
    
    for _ in range(count):
        # 生成随机创建时间
        random_seconds = random.randint(0, int((end_time - start_time).total_seconds()))
        created_time = start_time + timedelta(seconds=random_seconds)
        
        # 生成唯一的任务ID
        while True:
            task_id = generate_task_id(created_time)
            if task_id not in used_ids:
                used_ids.add(task_id)
                break
            else:
                # 如果ID重复，稍微调整时间
                created_time += timedelta(seconds=1)
        
        # 随机选择发起人和接收人（确保不同）
        initiator = random.choice(user_ids)
        receiver = random.choice([uid for uid in user_ids if uid != initiator])
        
        status = random.choice(task_schema['status']['values'])
        
        task = {
            "task_id": task_id,
            "initiator": initiator,
            "receiver": receiver,
            "location_id": random.choice(location_ids),
            "security_level": random.choice(task_schema['security_level']['values']),
            "status": status,
            "timestamps": generate_timestamps(status, created_time)
        }
        
        tasks.append(task)
    
    # 按创建时间排序
    tasks.sort(key=lambda x: x['timestamps']['created'])
    
    return tasks

def main():
    """主函数"""
    tasks = generate_tasks(TASKS_COUNT)
    
    if not tasks:
        print("无法生成任务数据，请确保用户和位置数据已生成")
        return
    
    # 保存到文件
    output_path = 'storage/tasks.json'
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(tasks, f, ensure_ascii=False, indent=2)
    
    print(f"已生成 {len(tasks)} 个任务数据到 {output_path}")

if __name__ == "__main__":
    main()
