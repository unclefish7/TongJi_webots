import json
import random
import os
from typing import Dict, Any

# 全局配置
LOCKERS_COUNT = 3

def load_schema() -> Dict[str, Any]:
    """加载 schema 配置"""
    schema_path = 'schema/schema_reference.json'
    with open(schema_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def generate_lockers(count: int) -> list:
    """生成柜体数据"""
    schema = load_schema()
    locker_schema = schema['Locker']
    
    lockers = []
    used_ids = set()
    
    for _ in range(count):
        # 生成唯一的柜体ID
        while True:
            locker_id = f"LOCK{random.randint(1000, 9999)}"
            if locker_id not in used_ids:
                used_ids.add(locker_id)
                break
        
        locker = {
            "locker_id": locker_id,
            "status": random.choice(locker_schema['status']['values'])
        }
        
        lockers.append(locker)
    
    return lockers

def main():
    """主函数"""
    lockers = generate_lockers(LOCKERS_COUNT)
    
    # 保存到文件
    output_path = 'storage/lockers.json'
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(lockers, f, ensure_ascii=False, indent=2)
    
    print(f"已生成 {len(lockers)} 个柜体数据到 {output_path}")

if __name__ == "__main__":
    main()
