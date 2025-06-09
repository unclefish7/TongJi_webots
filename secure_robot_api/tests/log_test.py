import pytest
import json
import os
import sys

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fastapi.testclient import TestClient
from main import app

# 使用同步测试客户端进行测试
client = TestClient(app)

def safe_print(*args, **kwargs):
    """安全的打印函数，确保在测试中可见"""
    print(*args, **kwargs, file=sys.stderr, flush=True)

class TestLogGeneration:
    """日志生成测试类"""
    
    def test_log_generation_during_auth(self, print_logs_content):
        """测试认证过程中的日志生成"""
        safe_print("\n=== 测试认证日志生成 ===")
        
        # 执行多次认证操作
        auth_operations = [
            {
                "user_id": "EMP9875",
                "requested_level": "L1",
                "provided": {},
                "description": "L1认证"
            },
            {
                "user_id": "EMP4958", 
                "requested_level": "L2",
                "provided": {"l2_auth": "495481"},
                "description": "L2认证"
            },
            {
                "user_id": "EMP7064",
                "requested_level": "L3", 
                "provided": {"l2_auth": "546121", "l3_auth": "FACE_EMP7064_4944"},
                "description": "L3认证"
            },
            {
                "user_id": "INVALID_USER",
                "requested_level": "L1",
                "provided": {},
                "description": "失败的认证"
            }
        ]
        
        for auth_op in auth_operations:
            safe_print(f"\n执行{auth_op['description']}...")
            response = client.post("/api/auth/verify", json={
                "user_id": auth_op["user_id"],
                "requested_level": auth_op["requested_level"],
                "provided": auth_op["provided"]
            })
            safe_print(f"响应状态: {response.status_code}")
        
        # 打印当前日志内容
        print_logs_content()
    
    def test_log_generation_during_task_operations(self, print_logs_content):
        """测试任务操作过程中的日志生成"""
        safe_print("\n=== 测试任务操作日志生成 ===")
        
        # 先进行认证
        safe_print("1. 执行认证...")
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        })
        assert auth_response.status_code == 200
        
        # 创建任务
        safe_print("2. 创建任务...")
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1",
            "description": "测试任务日志"
        })
        
        if task_response.status_code == 200:
            task_id = task_response.json()["task_id"]
            safe_print(f"任务创建成功: {task_id}")
            
            # 完成任务
            safe_print("3. 完成任务...")
            complete_response = client.post("/api/task/complete", json={
                "task_id": task_id
            })
            safe_print(f"完成任务响应: {complete_response.status_code}")
        else:
            safe_print(f"任务创建失败: {task_response.status_code}")
            safe_print(task_response.json())
        
        # 尝试创建一个会失败的任务
        safe_print("4. 创建失败任务（无认证）...")
        fail_task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1",
            "description": "应该失败的任务"
        })
        safe_print(f"失败任务响应: {fail_task_response.status_code}")
        
        # 打印当前日志内容
        print_logs_content()
    
    def test_log_file_structure(self):
        """测试日志文件结构是否正确"""
        safe_print("\n=== 测试日志文件结构 ===")
        
        # 先执行一些操作生成日志
        client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        })
        
        # 检查日志文件结构
        logs_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'logs.json')
        
        assert os.path.exists(logs_path), "日志文件不存在"
        
        with open(logs_path, 'r', encoding='utf-8') as f:
            logs = json.load(f)
        
        assert isinstance(logs, list), "日志文件应该是数组格式"
        
        if logs:
            # 检查第一条日志的结构
            first_log = logs[0]
            required_fields = ['log_id', 'log_type', 'message', 'timestamp']
            
            for field in required_fields:
                assert field in first_log, f"日志缺少必要字段: {field}"
            
            # 检查日志类型是否有效
            valid_log_types = ['access', 'auth', 'delivery', 'system', 'error']
            assert first_log['log_type'] in valid_log_types, f"无效的日志类型: {first_log['log_type']}"
            
            safe_print(f"✓ 日志文件结构正确，共有 {len(logs)} 条记录")
            safe_print(f"✓ 第一条日志: {first_log['log_type']} - {first_log['message']}")
    
    def test_different_log_types(self, print_logs_content):
        """测试不同类型的日志生成"""
        safe_print("\n=== 测试不同类型日志生成 ===")
        
        operations = [
            ("认证操作", lambda: client.post("/api/auth/verify", json={
                "user_id": "EMP9875", "requested_level": "L1", "provided": {}
            })),
            ("任务查询", lambda: client.get("/api/task/ping")),
            ("无效认证", lambda: client.post("/api/auth/verify", json={
                "user_id": "INVALID", "requested_level": "L1", "provided": {}
            })),
        ]
        
        for desc, operation in operations:
            safe_print(f"执行 {desc}...")
            try:
                response = operation()
                safe_print(f"  状态码: {response.status_code}")
            except Exception as e:
                safe_print(f"  异常: {e}")
        
        # 打印生成的日志
        print_logs_content()
    
    @pytest.mark.parametrize("log_count_before", [0])
    def test_log_count_increase(self, log_count_before):
        """测试日志数量确实在增加"""
        safe_print("\n=== 测试日志数量变化 ===")
        
        # 获取操作前的日志数量
        logs_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'logs.json')
        try:
            with open(logs_path, 'r', encoding='utf-8') as f:
                logs_before = json.load(f)
            count_before = len(logs_before)
        except:
            count_before = 0
        
        safe_print(f"操作前日志数量: {count_before}")
        
        # 执行一些操作
        client.post("/api/auth/verify", json={
            "user_id": "EMP9875", "requested_level": "L1", "provided": {}
        })
        
        client.post("/api/auth/verify", json={
            "user_id": "EMP4958", "requested_level": "L2", 
            "provided": {"l2_auth": "495481"}
        })
        
        # 获取操作后的日志数量
        with open(logs_path, 'r', encoding='utf-8') as f:
            logs_after = json.load(f)
        count_after = len(logs_after)
        
        safe_print(f"操作后日志数量: {count_after}")
        safe_print(f"新增日志数量: {count_after - count_before}")
        
        assert count_after > count_before, "日志数量应该有所增加"
    
    def test_session_log_isolation(self):
        """测试会话日志隔离性"""
        safe_print("\n=== 测试会话日志隔离 ===")
        
        # 记录测试开始前的日志数量
        logs_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'logs.json')
        with open(logs_path, 'r', encoding='utf-8') as f:
            initial_logs = json.load(f)
        initial_count = len(initial_logs)
        
        safe_print(f"测试开始前日志数量: {initial_count}")
        
        # 执行一些操作生成新日志
        for i in range(3):
            client.post("/api/auth/verify", json={
                "user_id": "EMP9875", "requested_level": "L1", "provided": {}
            })
        
        # 检查日志确实增加了
        with open(logs_path, 'r', encoding='utf-8') as f:
            current_logs = json.load(f)
        current_count = len(current_logs)
        
        safe_print(f"操作后日志数量: {current_count}")
        safe_print(f"新增日志数量: {current_count - initial_count}")
        
        assert current_count > initial_count, "应该生成了新的日志记录"
        
        # 这个测试结束后，conftest.py 会自动回滚日志
