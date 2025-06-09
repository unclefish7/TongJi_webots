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

class TestTaskAPI:
    """任务API测试类"""
    
    def setup_method(self):
        """每个测试方法前的设置"""
        # 清理任务文件
        tasks_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'tasks.json')
        with open(tasks_path, 'w', encoding='utf-8') as f:
            json.dump([], f)
    
    def test_ping(self):
        """测试ping接口"""
        response = client.get("/api/task/ping")
        assert response.status_code == 200
        data = response.json()
        assert "正常运行" in data["message"]
    
    def test_create_task_success_l1(self):
        """测试L1任务创建成功"""
        # 先进行L1认证
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        })
        assert auth_response.status_code == 200
        
        # 创建任务
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1",
            "description": "测试任务L1"
        })
        
        assert task_response.status_code == 200
        data = task_response.json()
        assert data["success"] == True
        assert data["code"] == "TASK_000"
        assert data["task_id"] is not None
        assert data["task"] is not None
        assert data["locker_id"] is not None
        assert "created successfully" in data["message"]
    
    def test_create_task_success_l2(self):
        """测试L2任务创建成功"""
        # 先进行L2认证
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP4958",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        })
        assert auth_response.status_code == 200
        
        # 创建任务
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP4958",
            "receiver": "EMP7064",
            "location_id": "LOC448",
            "security_level": "L2",
            "description": "测试任务L2"
        })
        
        assert task_response.status_code == 200
        data = task_response.json()
        assert data["success"] == True
        assert data["code"] == "TASK_000"
        assert data["task_id"] is not None
        assert data["task"] is not None
        assert data["locker_id"] is not None
    
    def test_create_task_success_l3(self):
        """测试L3任务创建成功"""
        # 先进行L3认证
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP7064",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121",
                "l3_auth": "FACE_EMP7064_4944"
            }
        })
        assert auth_response.status_code == 200
        
        # 创建任务
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP7064",
            "receiver": "EMP6330",
            "location_id": "LOC896",
            "security_level": "L3",
            "description": "测试任务L3"
        })
        
        assert task_response.status_code == 200
        data = task_response.json()
        assert data["success"] == True
        assert data["code"] == "TASK_000"
    
    def test_create_task_initiator_not_found(self):
        """测试发起人不存在"""
        task_response = client.post("/api/task/create", json={
            "user_id": "INVALID_USER",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        
        assert task_response.status_code == 404
        data = task_response.json()
        assert data["detail"]["code"] == "TASK_001"
        assert "not found" in data["detail"]["message"]
    
    def test_create_task_receiver_not_found(self):
        """测试接收人不存在"""
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "INVALID_USER",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        
        assert task_response.status_code == 404
        data = task_response.json()
        assert data["detail"]["code"] == "TASK_002"
        assert "not found" in data["detail"]["message"]
    
    def test_create_task_location_not_found(self):
        """测试位置不存在"""
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "INVALID_LOC",
            "security_level": "L1"
        })
        
        assert task_response.status_code == 404
        data = task_response.json()
        assert data["detail"]["code"] == "TASK_003"
        assert "not found" in data["detail"]["message"]
    
    def test_create_task_invalid_security_level(self):
        """测试无效安全等级"""
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "INVALID"
        })
        
        assert task_response.status_code == 400
        data = task_response.json()
        assert data["detail"]["code"] == "TASK_004"
        assert "Invalid security level" in data["detail"]["message"]
    
    def test_create_task_insufficient_auth(self):
        """测试认证不足"""
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        
        assert task_response.status_code == 403
        data = task_response.json()
        assert data["detail"]["code"] == "TASK_005"
        assert "Insufficient authentication" in data["detail"]["message"]
    
    def test_create_task_no_available_lockers(self):
        """测试无可用柜子"""
        # 先进行认证
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        })
        assert auth_response.status_code == 200
        
        # 将所有柜子设置为已使用
        lockers_path = os.path.join(os.path.dirname(__file__), '..', 'storage', 'lockers.json')
        with open(lockers_path, 'r', encoding='utf-8') as f:
            lockers = json.load(f)
        
        for locker in lockers:
            locker['status'] = 'in_use'
        
        with open(lockers_path, 'w', encoding='utf-8') as f:
            json.dump(lockers, f, ensure_ascii=False, indent=2)
        
        # 尝试创建任务
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        
        assert task_response.status_code == 503
        data = task_response.json()
        assert data["detail"]["code"] == "TASK_006"
        assert "No available lockers" in data["detail"]["message"]
        
        # 恢复柜子状态
        for locker in lockers:
            locker['status'] = 'available'
        
        with open(lockers_path, 'w', encoding='utf-8') as f:
            json.dump(lockers, f, ensure_ascii=False, indent=2)
    
    def test_complete_task_success(self):
        """测试完成任务成功"""
        # 先创建一个任务
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        })
        assert auth_response.status_code == 200
        
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        assert task_response.status_code == 200
        task_id = task_response.json()["task_id"]
        
        # 完成任务
        complete_response = client.post("/api/task/complete", json={
            "task_id": task_id
        })
        
        assert complete_response.status_code == 200
        data = complete_response.json()
        assert data["success"] == True
        assert data["code"] == "TASK_000"
        assert data["task"]["status"] == "completed"
        assert data["task"]["timestamps"]["delivered"] is not None
    
    def test_complete_task_not_found(self):
        """测试完成不存在的任务"""
        complete_response = client.post("/api/task/complete", json={
            "task_id": "INVALID_TASK"
        })
        
        assert complete_response.status_code == 404
        data = complete_response.json()
        assert data["detail"]["code"] == "TASK_010"
        assert "not found" in data["detail"]["message"]
    
    def test_fail_task_success(self):
        """测试标记任务失败成功"""
        # 先创建一个任务
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        })
        assert auth_response.status_code == 200
        
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        assert task_response.status_code == 200
        task_id = task_response.json()["task_id"]
        
        # 标记任务失败
        fail_response = client.post("/api/task/fail", json={
            "task_id": task_id,
            "reason": "测试失败原因"
        })
        
        assert fail_response.status_code == 200
        data = fail_response.json()
        assert data["success"] == True
        assert data["code"] == "TASK_000"
        assert data["task"]["status"] == "failed"
        assert "测试失败原因" in data["message"]
    
    def test_fail_task_not_found(self):
        """测试标记不存在的任务失败"""
        fail_response = client.post("/api/task/fail", json={
            "task_id": "INVALID_TASK",
            "reason": "测试"
        })
        
        assert fail_response.status_code == 404
        data = fail_response.json()
        assert data["detail"]["code"] == "TASK_010"
        assert "not found" in data["detail"]["message"]
    
    def test_auth_consumption_rollback(self):
        """测试认证消费回滚机制"""
        # 先进行认证
        auth_response = client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1",
            "provided": {}
        })
        assert auth_response.status_code == 200
        
        # 尝试创建任务（使用不存在的接收人，应该回滚认证）
        task_response = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "INVALID_USER",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        assert task_response.status_code == 404
        
        # 再次尝试创建任务（如果回滚成功，认证记录应该可以再次使用）
        task_response2 = client.post("/api/task/create", json={
            "user_id": "EMP9875",
            "receiver": "EMP4958",
            "location_id": "LOC101",
            "security_level": "L1"
        })
        assert task_response2.status_code == 200
