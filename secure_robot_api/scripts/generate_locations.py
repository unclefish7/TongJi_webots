import json
import random
import os
from typing import Dict, Any

# 全局配置
LOCATIONS_COUNT = 15

def load_schema() -> Dict[str, Any]:
    """加载 schema 配置"""
    schema_path = 'schema/schema_reference.json'
    with open(schema_path, 'r', encoding='utf-8') as f:
        return json.load(f)

def generate_locations(count: int) -> list:
    """生成位置数据"""
    schema = load_schema()
    location_schema = schema['Location']
    
    # 基于 map.json 的位置数据（硬编码）
    map_locations = [
        {"label": "经理室", "pose": {"x": 1.9446754456, "y": -19.5046272278, "z": 0.0}},
        {"label": "财务处", "pose": {"x": 1.4331240654, "y": -11.2010211945, "z": 0.0}},
        {"label": "大会议室", "pose": {"x": 1.3147621155, "y": -8.6426315308, "z": 0.0}},
        {"label": "等候处", "pose": {"x": -10.8124036789, "y": -21.7672996521, "z": 0.0}},
        {"label": "前台", "pose": {"x": -10.0897378922, "y": -13.9185628891, "z": 0.0}},
        {"label": "小会议室", "pose": {"x": -3.3445196152, "y": -21.2436790466, "z": 0.0}},
        {"label": "茶水间", "pose": {"x": -3.4624409676, "y": -9.6876792908, "z": 0.0}},
        {"label": "休息室", "pose": {"x": -3.4624409676, "y": -9.6876792908, "z": 0.0}},
        {"label": "男厕所", "pose": {"x": -9.8163862228, "y": -10.1237754822, "z": 0.0}},
        {"label": "女厕所", "pose": {"x": -9.6707010269, "y": -6.1619634628, "z": 0.0}},
        {"label": "办公B", "pose": {"x": -7.4183931351, "y": 1.7043571472, "z": 0.0}},
        {"label": "办公A", "pose": {"x": -2.7720179558, "y": 2.0250015259, "z": 0.0}}
    ]
    
    locations = []
    used_ids = set()
    
    # 为所有地图位置生成数据
    for map_location in map_locations:
        # 生成唯一的位置ID
        while True:
            location_id = f"LOC{random.randint(100, 999)}"
            if location_id not in used_ids:
                used_ids.add(location_id)
                break
        
        location = {
            "location_id": location_id,
            "label": map_location["label"]
        }
        
        # 使用硬编码的坐标，转换为二维坐标
        location["coordinates"] = {
            "x": round(map_location["pose"]["x"], 2),
            "y": round(map_location["pose"]["y"], 2)
        }
        
        locations.append(location)
    
    return locations

def main():
    """主函数"""
    locations = generate_locations(LOCATIONS_COUNT)
    
    # 保存到文件
    output_path = 'storage/locations.json'
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(locations, f, ensure_ascii=False, indent=2)
    
    print(f"已生成 {len(locations)} 个位置数据到 {output_path}")

if __name__ == "__main__":
    main()
