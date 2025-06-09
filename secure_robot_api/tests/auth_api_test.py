import pytest
import sys
import os

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fastapi.testclient import TestClient
from main import app

# 使用同步测试客户端进行测试
client = TestClient(app)

class TestAuthAPI:
    """认证API测试类"""
    
    def test_l1_auth_success(self):
        """测试L1认证成功"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP9875",
            "requested_level": "L1", 
            "provided": {}
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == True
        assert data["verified_level"] == "L1"
        assert "ID" in data["methods"]
    
    def test_l2_auth_success(self):
        """测试L2认证成功"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP4958",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == True
        assert data["verified_level"] == "L2"
        assert "ID" in data["methods"]
        assert "L2" in data["methods"]
    
    def test_l3_auth_success(self):
        """测试L3认证成功"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP7064",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121",
                "l3_auth": "FACE_EMP7064_4944"
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == True
        assert data["verified_level"] == "L3"
        assert "ID" in data["methods"]
        assert "L2" in data["methods"]
        assert "L3" in data["methods"]
    
    def test_auth_user_not_found(self):
        """测试用户不存在"""
        response = client.post("/api/auth/verify", json={
            "user_id": "INVALID_USER",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert response.status_code == 401
        data = response.json()
        assert "Authentication failed" in data["detail"]
    
    def test_auth_level_exceeds_user_level(self):
        """测试请求等级超过用户等级"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP4958",  # L2用户
            "requested_level": "L3",  # 请求L3认证
            "provided": {
                "l2_auth": "495481",
                "l3_auth": "FACE_EMP4958_3970"
            }
        })
        
        assert response.status_code == 401
        data = response.json()
        assert "Authentication failed" in data["detail"]
    
    def test_l2_auth_wrong_credentials(self):
        """测试L2认证凭证错误"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP4958",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "wrong_otp"
            }
        })
        
        assert response.status_code == 401
        data = response.json()
        assert "Authentication failed" in data["detail"]
    
    def test_l3_auth_missing_l3_credentials(self):
        """测试L3认证缺少L3凭证"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP7064",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121"
                # 缺少 l3_auth
            }
        })
        
        assert response.status_code == 401
        data = response.json()
        assert "Authentication failed" in data["detail"]
    
    def test_l3_auth_wrong_l3_credentials(self):
        """测试L3认证L3凭证错误"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP7064",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121",
                "l3_auth": "wrong_face_id"
            }
        })
        
        assert response.status_code == 401
        data = response.json()
        assert "Authentication failed" in data["detail"]
    
    def test_downgrade_auth_success(self):
        """测试L3用户请求L2认证成功"""
        response = client.post("/api/auth/verify", json={
            "user_id": "EMP7064",  # L3用户
            "requested_level": "L2",  # 请求L2认证
            "provided": {
                "l2_auth": "546121"
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == True
        assert data["verified_level"] == "L2"
        assert "ID" in data["methods"]
        assert "L2" in data["methods"]
        assert "L3" not in data["methods"]
