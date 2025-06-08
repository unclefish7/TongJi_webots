import json
import random
import os
from typing import Dict, Any

# 全局配置
USERS_COUNT = 5

def load_schema() -> Dict[str, Any]:
    """加载 schema 配置"""
    schema_path = 'schema/schema_reference.json'
    with open(schema_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def generate_user_id() -> str:
    """生成用户ID（工号）"""
    return f"EMP{random.randint(1000, 9999)}"

def generate_chinese_name() -> str:
    """生成中文姓名"""
    surnames = ["张", "李", "王", "刘", "陈", "杨", "赵", "黄", "周", "吴", "徐", "孙", "马", "朱", "胡", "林", "郭", "何", "高", "罗"]
    given_names = ["伟", "芳", "娜", "敏", "静", "强", "磊", "军", "洋", "勇", "艳", "杰", "娟", "涛", "明", "超", "秀英", "华", "建华", "丽"]
    return random.choice(surnames) + random.choice(given_names)

def generate_users(count: int) -> list:
    """生成用户数据"""
    schema = load_schema()
    user_schema = schema['User']
    
    users = []
    used_ids = set()
    
    for _ in range(count):
        # 生成唯一的用户ID
        while True:
            user_id = generate_user_id()
            if user_id not in used_ids:
                used_ids.add(user_id)
                break
        
        user = {
            "user_id": user_id,
            "name": generate_chinese_name(),
            "role": random.choice(user_schema['role']['values']),
            "auth_level": random.choice(user_schema['auth_level']['values']),
        }
        
        # 可选字段
        if random.random() > 0.3:  # 70% 概率生成 OTP
            user["otp"] = f"{random.randint(100000, 999999)}"
        
        if random.random() > 0.2:  # 80% 概率生成 face_id
            user["face_id"] = f"FACE_{user_id}_{random.randint(1000, 9999)}"
        
        users.append(user)
    
    return users

def main():
    """主函数"""
    users = generate_users(USERS_COUNT)
    
    # 保存到文件
    output_path = 'storage/users.json'
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(users, f, ensure_ascii=False, indent=2)
    
    print(f"已生成 {len(users)} 个用户数据到 {output_path}")

if __name__ == "__main__":
    main()
