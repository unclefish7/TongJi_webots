import pytest
import json
import os
from typing import Generator

@pytest.fixture(autouse=True)
def reset_test_data() -> Generator[None, None, None]:
    """
    自动执行的 fixture，每个测试前后重置测试数据
    
    autouse=True: 自动应用到所有测试，无需显式调用
    Generator: 使用 yield 分隔测试前和测试后的操作
    """
    
    # ===== 测试前执行 =====
    base_path = os.path.join(os.path.dirname(__file__), '..', 'storage')
    
    # 备份原始数据文件
    backup_files = ['tasks.json', 'lockers.json']
    backups = {}
    
    for filename in backup_files:
        file_path = os.path.join(base_path, filename)
        if os.path.exists(file_path):
            with open(file_path, 'r', encoding='utf-8') as f:
                backups[filename] = json.load(f)
    
    # yield 将控制权交给测试函数
    yield
    
    # ===== 测试后执行 =====
    # 恢复原始数据
    for filename, data in backups.items():
        file_path = os.path.join(base_path, filename)
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    
    # 清理认证缓存
    try:
        from services.auth_service import auth_session_cache
        auth_session_cache.clear()
    except ImportError:
        pass

@pytest.fixture
def sample_task_data():
    """示例任务数据 - 需要显式调用"""
    return {
        "user_id": "EMP9875",
        "receiver": "EMP4958", 
        "location_id": "LOC101",
        "security_level": "L1",
        "description": "测试任务"
    }

@pytest.fixture
def sample_auth_data():
    """示例认证数据 - 需要显式调用"""
    return {
        "l1": {
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        },
        "l2": {
            "user_id": "EMP4958",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        },
        "l3": {
            "user_id": "EMP7064",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121",
                "l3_auth": "FACE_EMP7064_4944"
            }
        }
    }

@pytest.fixture(scope="session")
def api_client():
    """会话级别的测试客户端，整个测试会话只创建一次"""
    from fastapi.testclient import TestClient
    from main import app
    return TestClient(app)

@pytest.fixture(scope="function")
def clean_auth_cache():
    """函数级别的认证缓存清理"""
    # 测试前清理
    try:
        from services.auth_service import auth_session_cache
        auth_session_cache.clear()
    except ImportError:
        pass
    
    yield
    
    # 测试后清理
    try:
        from services.auth_service import auth_session_cache
        auth_session_cache.clear()
    except ImportError:
        pass
