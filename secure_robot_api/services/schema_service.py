import json
import os
from typing import Dict, Any, Optional
from datetime import datetime

def load_schema() -> Dict[str, Any]:
    """从 schema_reference.json 加载schema定义"""
    schema_path = os.path.join(os.path.dirname(__file__), '..', 'schema', 'schema_reference.json')
    try:
        with open(schema_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return {}

def generate_field_value(field_name: str, field_config: Dict[str, Any], provided_values: Dict[str, Any] = None) -> Any:
    """
    根据schema字段配置生成字段值
    
    Args:
        field_name: 字段名
        field_config: 字段配置
        provided_values: 用户提供的值
    
    Returns:
        生成的字段值
    """
    if provided_values and field_name in provided_values:
        return provided_values[field_name]
    
    field_type = field_config.get('type')
    
    # 生成器字段
    if 'generator' in field_config:
        generator = field_config.get('generator')
        if generator == 'timestamp+prefix':
            prefix = field_config.get('prefix', '')
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            return f"{prefix}{timestamp}"
    
    # 对象类型字段
    if field_type == 'object':
        obj = {}
        fields = field_config.get('fields', {})
        for sub_field_name, sub_field_config in fields.items():
            if sub_field_config.get('nullable', False) and sub_field_name not in (provided_values or {}):
                obj[sub_field_name] = None
            elif sub_field_config.get('format') == 'iso8601':
                if sub_field_name == 'created':
                    obj[sub_field_name] = datetime.now().isoformat()
                else:
                    obj[sub_field_name] = None
            else:
                obj[sub_field_name] = generate_field_value(sub_field_name, sub_field_config, provided_values)
        return obj
    
    # 枚举类型的默认值
    if field_type == 'enum':
        values = field_config.get('values', [])
        if 'status' in field_name.lower() and 'pending' in values:
            return 'pending'
        return values[0] if values else None
    
    # 其他类型的默认值
    if field_config.get('required', False):
        if field_type == 'string':
            return ""
        elif field_type == 'array':
            return []
    
    return None

def create_entity_from_schema(entity_name: str, provided_values: Dict[str, Any]) -> Dict[str, Any]:
    """
    根据schema定义创建实体数据
    
    Args:
        entity_name: 实体名称（如 Task, User 等）
        provided_values: 用户提供的值
    
    Returns:
        根据schema生成的完整实体数据
    """
    schema = load_schema()
    entity_schema = schema.get(entity_name, {})
    
    if not entity_schema:
        raise ValueError(f"Schema not found for entity: {entity_name}")
    
    entity_data = {}
    
    for field_name, field_config in entity_schema.items():
        value = generate_field_value(field_name, field_config, provided_values)
        if value is not None:
            entity_data[field_name] = value
    
    return entity_data

def validate_entity_against_schema(entity_name: str, entity_data: Dict[str, Any]) -> tuple[bool, str]:
    """
    根据schema验证实体数据
    
    Args:
        entity_name: 实体名称
        entity_data: 实体数据
    
    Returns:
        tuple[bool, str]: (是否有效, 错误信息)
    """
    schema = load_schema()
    entity_schema = schema.get(entity_name, {})
    
    if not entity_schema:
        return False, f"Schema not found for entity: {entity_name}"
    
    for field_name, field_config in entity_schema.items():
        if field_config.get('required', False) and field_name not in entity_data:
            return False, f"Required field missing: {field_name}"
        
        if field_name in entity_data:
            field_type = field_config.get('type')
            field_value = entity_data[field_name]
            
            # 枚举类型验证
            if field_type == 'enum':
                valid_values = field_config.get('values', [])
                if field_value not in valid_values:
                    return False, f"Invalid value for {field_name}: {field_value}. Valid values: {valid_values}"
    
    return True, ""
