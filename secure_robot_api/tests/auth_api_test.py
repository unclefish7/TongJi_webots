import pytest
import sys
import os
from datetime import datetime, timedelta

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
    
    # ============ 新增：Purpose认证接口测试 ============
    
    def test_purpose_auth_send_l1_success(self):
        """测试寄件L1认证成功"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "send",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == True
        assert data["verified_level"] == "L1"
        assert "ID" in data["methods"]
        # 寄件认证不应返回有效的过期时间
        assert data.get("expires_at") is None
    
    def test_purpose_auth_pickup_l1_success(self):
        """测试取件L1认证成功"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "pickup",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == True
        assert data["verified_level"] == "L1"
        assert "ID" in data["methods"]
        # 取件认证应返回有效的过期时间
        assert data.get("expires_at") is not None
        
        # 验证过期时间格式和合理性
        expires_at = data["expires_at"]
        expires_datetime = datetime.fromisoformat(expires_at)
        now = datetime.now()
        time_diff = expires_datetime - now
        
        # 应该在4-6分钟之间（5分钟±1分钟误差）
        assert 4 * 60 < time_diff.total_seconds() < 6 * 60
    
    def test_purpose_auth_send_l2_success(self):
        """测试寄件L2认证成功"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "send",
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
        assert data.get("expires_at") is None
    
    def test_purpose_auth_pickup_l2_success(self):
        """测试取件L2认证成功"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "pickup",
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
        assert data.get("expires_at") is not None
    
    def test_purpose_auth_send_l3_success(self):
        """测试寄件L3认证成功"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP7064",
            "purpose": "send",
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
        assert data.get("expires_at") is None
    
    def test_purpose_auth_pickup_l3_success(self):
        """测试取件L3认证成功"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP7064",
            "purpose": "pickup",
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
        assert data.get("expires_at") is not None
    
    def test_purpose_auth_user_not_found(self):
        """测试用户不存在的错误处理"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "INVALID_USER",
            "purpose": "send",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == False
        assert "不存在" in data["message"]
    
    def test_purpose_auth_level_exceeds_user_level(self):
        """测试请求等级超过用户等级"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",  # L2用户
            "purpose": "send",
            "requested_level": "L3",  # 请求L3认证
            "provided": {
                "l2_auth": "495481",
                "l3_auth": "fake_face"
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == False
        assert "最大认证等级" in data["message"]
    
    def test_purpose_auth_wrong_l2_credentials(self):
        """测试L2认证凭证错误"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "send",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "wrong_otp"
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == False
        assert "L2 级认证凭证无效" in data["message"]
    
    def test_purpose_auth_missing_l2_credentials(self):
        """测试L2认证缺少凭证"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "send",
            "requested_level": "L2",
            "provided": {}  # 缺少l2_auth
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == False
        assert "L2 级认证凭证无效" in data["message"]
    
    def test_purpose_auth_l3_missing_l2_credentials(self):
        """测试L3认证缺少L2凭证"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP7064",
            "purpose": "send",
            "requested_level": "L3",
            "provided": {
                "l3_auth": "FACE_EMP7064_4944"
                # 缺少l2_auth
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == False
        assert "L2 级认证凭证无效" in data["message"]
    
    def test_purpose_auth_l3_missing_l3_credentials(self):
        """测试L3认证缺少L3凭证"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP7064",
            "purpose": "send",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121"
                # 缺少l3_auth
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == False
        assert "L3 级认证凭证无效" in data["message"]
    
    def test_purpose_auth_l3_wrong_l3_credentials(self):
        """测试L3认证L3凭证错误"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP7064",
            "purpose": "pickup",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121",
                "l3_auth": "wrong_face_id"
            }
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["verified"] == False
        assert "L3 级认证凭证无效" in data["message"]
    
    def test_purpose_auth_invalid_purpose_value(self):
        """测试无效的purpose值"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "invalid_purpose",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert response.status_code == 422  # Validation error
    
    def test_purpose_auth_invalid_level_value(self):
        """测试无效的认证等级值"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "send",
            "requested_level": "L4",  # 无效等级
            "provided": {}
        })
        
        assert response.status_code == 422  # Validation error
    
    def test_purpose_auth_downgrade_success(self):
        """测试L3用户请求L2认证成功"""
        response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP7064",  # L3用户
            "purpose": "send",
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
    
    def test_purpose_auth_multiple_pickup_same_user(self):
        """测试同一用户多次取件认证（应该覆盖之前的认证）"""
        # 第一次取件认证
        response1 = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "pickup",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert response1.status_code == 200
        data1 = response1.json()
        assert data1["verified"] == True
        expires_at_1 = data1["expires_at"]
        
        # 第二次取件认证（应该覆盖第一次）
        response2 = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "pickup",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert response2.status_code == 200
        data2 = response2.json()
        assert data2["verified"] == True
        expires_at_2 = data2["expires_at"]
        
        # 第二次的过期时间应该比第一次晚
        expires_1 = datetime.fromisoformat(expires_at_1)
        expires_2 = datetime.fromisoformat(expires_at_2)
        assert expires_2 > expires_1
    
    def test_purpose_auth_send_and_pickup_different_cache(self):
        """测试寄件和取件认证使用不同缓存"""
        # 先进行寄件认证
        send_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "send",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        })
        
        assert send_response.status_code == 200
        send_data = send_response.json()
        assert send_data["verified"] == True
        assert send_data.get("expires_at") is None
        
        # 再进行取件认证
        pickup_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "pickup",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        })
        
        assert pickup_response.status_code == 200
        pickup_data = pickup_response.json()
        assert pickup_data["verified"] == True
        assert pickup_data.get("expires_at") is not None
    
    def test_purpose_auth_cache_behavior_verification(self):
        """测试认证缓存行为验证"""
        from services.auth_service import auth_session_cache, pickup_auth_cache
        
        # 清理缓存
        auth_session_cache.clear()
        pickup_auth_cache.clear()
        
        # 初始状态缓存应该为空
        assert len(auth_session_cache) == 0
        assert len(pickup_auth_cache) == 0
        
        # 寄件认证
        send_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "send",
            "requested_level": "L1",
            "provided": {}
        })
        
        assert send_response.status_code == 200
        assert len(auth_session_cache) == 1  # 寄件缓存增加
        assert len(pickup_auth_cache) == 0   # 取件缓存不变
        assert "EMP9875" in auth_session_cache
        
        # 取件认证
        pickup_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "pickup",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        })
        
        assert pickup_response.status_code == 200
        assert len(auth_session_cache) == 1  # 寄件缓存不变
        assert len(pickup_auth_cache) == 1   # 取件缓存增加
        assert "EMP4958" in pickup_auth_cache
    
    # ============ 新增：缓存状态接口测试 ============
    
    def test_cache_status_initial_empty(self):
        """测试初始状态缓存为空"""
        from services.auth_service import auth_session_cache, pickup_auth_cache
        
        # 清理缓存
        auth_session_cache.clear()
        pickup_auth_cache.clear()
        
        response = client.get("/api/auth/cache_status")
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        
        cache_data = data["data"]
        assert cache_data["send_auth_cache"]["users"] == 0
        assert cache_data["send_auth_cache"]["total_records"] == 0
        assert cache_data["send_auth_cache"]["available_records"] == 0
        assert cache_data["pickup_auth_cache"]["users"] == 0
    
    def test_cache_status_after_send_auth(self):
        """测试寄件认证后的缓存状态"""
        from services.auth_service import auth_session_cache, pickup_auth_cache
        
        # 清理缓存
        auth_session_cache.clear()
        pickup_auth_cache.clear()
        
        # 执行寄件认证
        auth_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "send",
            "requested_level": "L1",
            "provided": {}
        })
        assert auth_response.status_code == 200
        
        # 检查缓存状态
        status_response = client.get("/api/auth/cache_status")
        assert status_response.status_code == 200
        
        data = status_response.json()
        cache_data = data["data"]
        
        assert cache_data["send_auth_cache"]["users"] == 1
        assert cache_data["send_auth_cache"]["total_records"] == 1
        assert cache_data["send_auth_cache"]["available_records"] == 1
        assert cache_data["pickup_auth_cache"]["users"] == 0
    
    def test_cache_status_after_pickup_auth(self):
        """测试取件认证后的缓存状态"""
        from services.auth_service import auth_session_cache, pickup_auth_cache
        
        # 清理缓存
        auth_session_cache.clear()
        pickup_auth_cache.clear()
        
        # 执行取件认证
        auth_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "pickup",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        })
        assert auth_response.status_code == 200
        
        # 检查缓存状态
        status_response = client.get("/api/auth/cache_status")
        assert status_response.status_code == 200
        
        data = status_response.json()
        cache_data = data["data"]
        
        assert cache_data["send_auth_cache"]["users"] == 0
        assert cache_data["pickup_auth_cache"]["users"] == 1
    
    def test_cache_status_mixed_auth_types(self):
        """测试混合认证类型的缓存状态"""
        from services.auth_service import auth_session_cache, pickup_auth_cache
        
        # 清理缓存
        auth_session_cache.clear()
        pickup_auth_cache.clear()
        
        # 执行寄件认证
        send_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP9875",
            "purpose": "send",
            "requested_level": "L1",
            "provided": {}
        })
        assert send_response.status_code == 200
        
        # 执行取件认证
        pickup_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "pickup",
            "requested_level": "L2",
            "provided": {
                "l2_auth": "495481"
            }
        })
        assert pickup_response.status_code == 200
        
        # 再执行一次寄件认证
        send_response2 = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP7064",
            "purpose": "send",
            "requested_level": "L3",
            "provided": {
                "l2_auth": "546121",
                "l3_auth": "FACE_EMP7064_4944"
            }
        })
        assert send_response2.status_code == 200
        
        # 检查缓存状态
        status_response = client.get("/api/auth/cache_status")
        assert status_response.status_code == 200
        
        data = status_response.json()
        cache_data = data["data"]
        
        # 应该有2个寄件用户，1个取件用户
        assert cache_data["send_auth_cache"]["users"] == 2
        assert cache_data["send_auth_cache"]["total_records"] == 2
        assert cache_data["send_auth_cache"]["available_records"] == 2
        assert cache_data["pickup_auth_cache"]["users"] == 1
    
    def test_comprehensive_purpose_auth_scenarios(self):
        """综合认证场景测试"""
        # 测试用户数据
        test_cases = [
            # (user_id, max_level, test_level, l2_auth, l3_auth, should_succeed)
            ("EMP9875", "L1", "L1", None, None, True),
            ("EMP4958", "L2", "L1", None, None, True),
            ("EMP4958", "L2", "L2", "495481", None, True),
            ("EMP7064", "L3", "L1", None, None, True),
            ("EMP7064", "L3", "L2", "546121", None, True),
            ("EMP7064", "L3", "L3", "546121", "FACE_EMP7064_4944", True),
            ("EMP9875", "L1", "L2", "fake", None, False),  # 超过用户等级
            ("EMP4958", "L2", "L3", "495481", "fake", False),  # 超过用户等级
        ]
        
        for user_id, max_level, test_level, l2_auth, l3_auth, should_succeed in test_cases:
            provided = {}
            if l2_auth:
                provided["l2_auth"] = l2_auth
            if l3_auth:
                provided["l3_auth"] = l3_auth
            
            for purpose in ["send", "pickup"]:
                response = client.post("/api/auth/verify_purpose", json={
                    "user_id": user_id,
                    "purpose": purpose,
                    "requested_level": test_level,
                    "provided": provided
                })
                
                assert response.status_code == 200
                data = response.json()
                
                if should_succeed:
                    assert data["verified"] == True, f"Failed: {user_id} {test_level} {purpose}"
                    assert data["verified_level"] == test_level
                    if purpose == "pickup":
                        assert data.get("expires_at") is not None
                    else:
                        assert data.get("expires_at") is None
                else:
                    assert data["verified"] == False, f"Should have failed: {user_id} {test_level} {purpose}"
                    assert "message" in data
