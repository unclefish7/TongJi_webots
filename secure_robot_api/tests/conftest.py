import pytest
import json
import os
import sys
from typing import Generator

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

@pytest.fixture(autouse=True)
def reset_test_data() -> Generator[None, None, None]:
    """
    自动执行的 fixture，每个测试前后重置测试数据
    
    autouse=True: 自动应用到所有测试，无需显式调用
    Generator: 使用 yield 分隔测试前和测试后的操作
    """
    
    # ===== 测试前执行 =====
    base_path = os.path.join(os.path.dirname(__file__), '..', 'storage')
    
    # 备份原始数据文件（包括logs.json）
    backup_files = ['tasks.json', 'lockers.json', 'logs.json']
    backups = {}
    
    for filename in backup_files:
        file_path = os.path.join(base_path, filename)
        if os.path.exists(file_path):
            with open(file_path, 'r', encoding='utf-8') as f:
                backups[filename] = json.load(f)
        else:
            # 如果文件不存在，创建空备份
            backups[filename] = []
    
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

@pytest.fixture(scope="session", autouse=True)
def session_setup_and_cleanup():
    """
    会话级别的 fixture，在整个测试会话开始前和结束后执行
    """
    # ===== 会话开始前 =====
    print("\n" + "="*80)
    print("开始测试会话")
    print("="*80)
    
    # 备份会话开始时的日志状态
    logs_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'logs.json')
    session_log_backup = []
    
    try:
        if os.path.exists(logs_path):
            with open(logs_path, 'r', encoding='utf-8') as f:
                session_log_backup = json.load(f)
    except Exception:
        session_log_backup = []
    
    yield  # 执行所有测试
    
    # ===== 会话结束后 =====
    print("\n" + "="*80)
    print("测试会话结束 - 打印本次会话生成的日志内容:")
    print("="*80)
    
    try:
        # 读取当前日志
        current_logs = []
        if os.path.exists(logs_path):
            with open(logs_path, 'r', encoding='utf-8') as f:
                current_logs = json.load(f)
        
        # 计算新增的日志（本次会话生成的）
        session_logs = current_logs[len(session_log_backup):]
        
        if not session_logs:
            print("本次测试会话没有生成新的日志记录")
        else:
            print(f"本次会话共生成 {len(session_logs)} 条新日志记录:\n")
            
            # 按类型分组显示
            log_types = {}
            for log in session_logs:
                log_type = log.get('log_type', 'unknown')
                if log_type not in log_types:
                    log_types[log_type] = []
                log_types[log_type].append(log)
            
            # 显示统计信息
            print("日志类型统计:")
            for log_type, logs in log_types.items():
                success_count = len([l for l in logs if l.get('result') == 'success'])
                failed_count = len([l for l in logs if l.get('result') == 'failed'])
                print(f"  {log_type}: {len(logs)} 条 (成功: {success_count}, 失败: {failed_count})")
            
            print("\n详细日志记录:")
            print("-" * 80)
            
            for i, log in enumerate(session_logs, 1):
                print(f"日志 {i}:")
                print(f"  ID: {log.get('log_id', 'N/A')}")
                print(f"  类型: {log.get('log_type', 'N/A')}")
                print(f"  用户: {log.get('related_user', 'N/A')}")
                print(f"  任务: {log.get('related_task', 'N/A')}")
                print(f"  柜子: {log.get('related_locker', 'N/A')}")
                print(f"  消息: {log.get('message', 'N/A')}")
                print(f"  认证方式: {log.get('auth_mode', 'N/A')}")
                print(f"  结果: {log.get('result', 'N/A')}")
                print(f"  时间: {log.get('timestamp', 'N/A')}")
                print("-" * 60)
        
        # 恢复会话开始时的日志状态
        with open(logs_path, 'w', encoding='utf-8') as f:
            json.dump(session_log_backup, f, ensure_ascii=False, indent=2)
        
        print(f"✓ 日志文件已恢复到会话开始前的状态 ({len(session_log_backup)} 条记录)")
        
    except FileNotFoundError:
        print("logs.json 文件不存在")
    except json.JSONDecodeError as e:
        print(f"logs.json 文件格式错误: {e}")
    except Exception as e:
        print(f"处理日志文件时出错: {e}")
    
    print("="*80)

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

@pytest.fixture
def print_logs_content():
    """
    手动调用的 fixture，用于在测试中打印当前日志内容
    """
    def _print_logs():
        logs_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'logs.json')
        try:
            with open(logs_path, 'r', encoding='utf-8') as f:
                logs = json.load(f)
            
            print(f"\n当前日志内容 ({len(logs)} 条记录):")
            for i, log in enumerate(logs, 1):
                print(f"{i}. [{log.get('log_type')}] {log.get('message')} - {log.get('timestamp')}")
        except Exception as e:
            print(f"无法读取日志: {e}")
    
    return _print_logs
