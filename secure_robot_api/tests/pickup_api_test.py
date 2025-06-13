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

class TestPickupAPI:
    """取件API测试类"""
    
    def setup_method(self):
        """每个测试方法前的准备工作"""
        # 清理认证缓存
        from services.auth_service import auth_session_cache, pickup_auth_cache
        auth_session_cache.clear()
        pickup_auth_cache.clear()
        
        # 准备测试数据 - 创建一个已到达的任务
        self.test_task_data = {
            "user_id": "EMP4958",  # 发起人
            "receiver": "EMP9875",  # 接收人  
            "location_id": "LOC101",
            "security_level": "L1",
            "description": "测试取件任务"
        }
    
    def create_test_task_and_set_arrived(self):
        """创建测试任务并设置为已到达状态"""
        # 先为发起人进行认证
        auth_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "send", 
            "requested_level": "L2",
            "provided": {"l2_auth": "495481"}
        })
        assert auth_response.status_code == 200
        
        # 创建任务
        task_response = client.post("/api/task/create", json=self.test_task_data)
        assert task_response.status_code == 200
        task_data = task_response.json()
        task_id = task_data["task_id"]
        
        # 手动设置任务状态为arrived
        from services.task_service import load_tasks, save_tasks
        tasks = load_tasks()
        for task in tasks:
            if task.get('task_id') == task_id:
                task['status'] = 'arrived'
                break
        save_tasks(tasks)
        
        return task_id, task_data
    
    def authenticate_user_for_pickup(self, user_id: str, level: str = "L1", provided: dict = None):
        """为用户进行取件认证"""
        if provided is None:
            provided = {}
        
        auth_response = client.post("/api/auth/verify_purpose", json={
            "user_id": user_id,
            "purpose": "pickup",
            "requested_level": level,
            "provided": provided
        })
        assert auth_response.status_code == 200
        return auth_response.json()
    
    def test_get_pickup_tasks_empty(self):
        """测试查询空的取件任务列表"""
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        assert data["user_id"] == "EMP9875"
        assert data["total_tasks"] == 0
        assert len(data["tasks"]) == 0
        assert data["auth_status"]["authenticated"] == False
    
    def test_get_pickup_tasks_with_auth_only(self):
        """测试只有认证但无可取件任务"""
        # 为用户进行取件认证
        self.authenticate_user_for_pickup("EMP9875")
        
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        assert data["total_tasks"] == 0
        assert data["auth_status"]["authenticated"] == True
        assert data["auth_status"]["level"] == "L1"
        assert data["auth_status"]["expires_at"] is not None
    
    def test_get_pickup_tasks_with_available_task(self):
        """测试查询有可取件任务的情况"""
        # 创建测试任务并设置为arrived状态
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 为接收人进行认证
        self.authenticate_user_for_pickup("EMP9875")
        
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        assert data["user_id"] == "EMP9875"
        assert data["total_tasks"] == 1
        assert len(data["tasks"]) == 1
        
        task = data["tasks"][0]
        assert task["task_id"] == task_id
        assert task["security_level"] == "L1"
        assert task["description"] == "测试取件任务"
        assert task["locker_id"] is not None
        
        assert data["auth_status"]["authenticated"] == True
        assert data["auth_status"]["level"] == "L1"
    
    def test_get_pickup_tasks_different_receiver(self):
        """测试查询其他用户的任务（应该查不到）"""
        # 创建测试任务（接收人是EMP9875）
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 用其他用户查询
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP4958"})
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        assert data["user_id"] == "EMP4958"
        assert data["total_tasks"] == 0  # 不是接收人，查不到任务
    
    def test_pickup_execute_success(self):
        """测试成功执行取件操作"""
        # 创建测试任务并设置为arrived状态
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 为接收人进行认证
        self.authenticate_user_for_pickup("EMP9875", "L1")
        
        # 执行取件
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": task_id
        })
        
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        assert data["code"] == "PICKUP_000"
        assert "取件成功" in data["message"]
        assert data["task_id"] == task_id
        
        # 验证任务状态已更新为completed
        from services.task_service import get_task_by_id
        updated_task = get_task_by_id(task_id)
        assert updated_task["status"] == "completed"
        assert updated_task["timestamps"]["delivered"] is not None
        
        # 验证柜子状态已释放
        from services.task_service import load_lockers
        lockers = load_lockers()
        locker_id = task_data["locker_id"]
        for locker in lockers:
            if locker["locker_id"] == locker_id:
                assert locker["status"] == "available"
                break
        else:
            pytest.fail("未找到对应的柜子")
    
    def test_pickup_execute_task_not_found(self):
        """测试取件不存在的任务"""
        self.authenticate_user_for_pickup("EMP9875")
        
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": "INVALID_TASK"
        })
        
        assert response.status_code == 404
        data = response.json()
        assert data["detail"]["code"] == "PICKUP_001"
        assert "不存在" in data["detail"]["message"]
    
    def test_pickup_execute_wrong_receiver(self):
        """测试非接收人尝试取件"""
        # 创建测试任务（接收人是EMP9875）
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 其他用户进行认证并尝试取件
        self.authenticate_user_for_pickup("EMP4958", "L2", {"l2_auth": "495481"})
        
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP4958",  # 不是接收人
            "task_id": task_id
        })
        
        assert response.status_code == 404
        data = response.json()
        assert data["detail"]["code"] == "PICKUP_001"
        assert "接收人不是当前用户" in data["detail"]["message"]
    
    def test_pickup_execute_wrong_status(self):
        """测试取件状态不正确的任务"""
        # 创建任务但不设置为arrived状态（保持pending）
        auth_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "send",
            "requested_level": "L2", 
            "provided": {"l2_auth": "495481"}
        })
        assert auth_response.status_code == 200
        
        task_response = client.post("/api/task/create", json=self.test_task_data)
        assert task_response.status_code == 200
        task_id = task_response.json()["task_id"]
        
        # 为接收人认证并尝试取件
        self.authenticate_user_for_pickup("EMP9875")
        
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": task_id
        })
        
        assert response.status_code == 404
        data = response.json()
        assert data["detail"]["code"] == "PICKUP_001"
        assert "状态为 'pending'" in data["detail"]["message"]
        assert "需要状态为 'arrived'" in data["detail"]["message"]
    
    def test_pickup_execute_no_auth(self):
        """测试未认证用户尝试取件"""
        # 创建测试任务
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 不进行认证直接尝试取件
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": task_id
        })
        
        assert response.status_code == 403
        data = response.json()
        assert data["detail"]["code"] == "PICKUP_002"
        assert "未找到有效的取件认证" in data["detail"]["message"]
    
    def test_pickup_execute_insufficient_auth_level(self):
        """测试认证等级不足"""
        # 创建需要L2等级的任务
        l2_task_data = self.test_task_data.copy()
        l2_task_data["security_level"] = "L2"
        
        # 为发起人进行L2认证
        auth_response = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",
            "purpose": "send",
            "requested_level": "L2",
            "provided": {"l2_auth": "495481"}
        })
        assert auth_response.status_code == 200
        
        # 创建L2任务
        task_response = client.post("/api/task/create", json=l2_task_data)
        assert task_response.status_code == 200
        task_id = task_response.json()["task_id"]
        
        # 设置任务状态为arrived
        from services.task_service import load_tasks, save_tasks
        tasks = load_tasks()
        for task in tasks:
            if task.get('task_id') == task_id:
                task['status'] = 'arrived'
                break
        save_tasks(tasks)
        
        # 接收人只进行L1认证
        self.authenticate_user_for_pickup("EMP9875", "L1")
        
        # 尝试取件
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": task_id
        })
        
        assert response.status_code == 403
        data = response.json()
        assert data["detail"]["code"] == "PICKUP_002"
        assert "认证等级不足" in data["detail"]["message"]
        assert "需要 L2 级认证，当前为 L1 级" in data["detail"]["message"]
    
    def test_pickup_execute_expired_auth(self):
        """测试过期认证的取件尝试"""
        # 创建测试任务
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 为接收人进行认证
        self.authenticate_user_for_pickup("EMP9875")
        
        # 手动设置认证为过期状态
        from services.auth_service import pickup_auth_cache
        from datetime import datetime, timedelta
        if "EMP9875" in pickup_auth_cache:
            expired_time = (datetime.now() - timedelta(minutes=1)).isoformat()
            pickup_auth_cache["EMP9875"]["expires_at"] = expired_time
        
        # 尝试取件
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": task_id
        })
        
        assert response.status_code == 403
        data = response.json()
        assert data["detail"]["code"] == "PICKUP_002"
        assert "未找到有效的取件认证" in data["detail"]["message"]
    
    def test_pickup_multiple_tasks_different_levels(self):
        """测试多个不同安全等级的取件任务"""
        # 清理所有现有任务，确保测试环境干净
        from services.task_service import load_tasks, save_tasks
        save_tasks([])  # 清空任务列表
        
        # 释放所有柜子，确保有足够的柜子可用
        from services.task_service import load_lockers, save_lockers
        lockers = load_lockers()
        for locker in lockers:
            locker['status'] = 'available'
        save_lockers(lockers)
        
        # 创建第一个L1任务
        auth_response1 = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",  # L2用户
            "purpose": "send",
            "requested_level": "L1",  # 只需要L1等级
            "provided": {}  # L1认证不需要额外凭证
        })
        assert auth_response1.status_code == 200, f"L1认证失败: {auth_response1.json()}"
        
        l1_task_data = {
            "user_id": "EMP4958",
            "receiver": "EMP9875",
            "location_id": "LOC101", 
            "security_level": "L1",
            "description": "L1测试任务"
        }
        
        task_response1 = client.post("/api/task/create", json=l1_task_data)
        assert task_response1.status_code == 200, f"L1任务创建失败: {task_response1.json()}"
        task_id1 = task_response1.json()["task_id"]
        
        # 设置L1任务为arrived状态
        tasks = load_tasks()
        l1_task_found = False
        for task in tasks:
            if task.get('task_id') == task_id1:
                task['status'] = 'arrived'
                l1_task_found = True
                break
        assert l1_task_found, f"未找到L1任务 {task_id1} 进行状态更新"
        save_tasks(tasks)
        
        # 验证L1任务状态已正确设置并保存
        from services.task_service import get_task_by_id
        l1_task = get_task_by_id(task_id1)
        assert l1_task is not None, f"未能读取到L1任务 {task_id1}"
        assert l1_task.get('status') == 'arrived', f"L1任务状态未设置为arrived: {l1_task.get('status')}"
        
        # 确保有足够的时间差生成不同的任务ID
        import time
        time.sleep(1)
        
        # 创建第二个L2任务
        auth_response2 = client.post("/api/auth/verify_purpose", json={
            "user_id": "EMP4958",  # L2用户
            "purpose": "send",
            "requested_level": "L2",
            "provided": {"l2_auth": "495481"}
        })
        assert auth_response2.status_code == 200, f"L2认证失败: {auth_response2.json()}"
        
        l2_task_data = {
            "user_id": "EMP4958",
            "receiver": "EMP9875",
            "location_id": "LOC101", 
            "security_level": "L2",
            "description": "L2测试任务"
        }
        
        # 检查柜子可用性
        lockers_before = load_lockers()
        available_lockers = [l for l in lockers_before if l.get('status') == 'available']
        assert len(available_lockers) > 0, "没有可用柜子"
        
        task_response2 = client.post("/api/task/create", json=l2_task_data)
        assert task_response2.status_code == 200, f"L2任务创建失败: {task_response2.json()}"
        task_id2 = task_response2.json()["task_id"]
        
        # 验证两个任务ID不同
        assert task_id1 != task_id2, f"任务ID冲突: {task_id1} == {task_id2}"
        
        # 使用获取单个任务的方式验证L2任务创建成功
        l2_task = get_task_by_id(task_id2)
        assert l2_task is not None, f"未能读取到L2任务 {task_id2}"
        
        # 重新加载所有任务，确保任务列表是最新的
        tasks = load_tasks()
        
        # 检查任务列表中同时存在两个任务
        task_ids = [t.get('task_id') for t in tasks]
        assert task_id1 in task_ids, f"任务列表中找不到L1任务 {task_id1}"
        assert task_id2 in task_ids, f"任务列表中找不到L2任务 {task_id2}"
        
        # 设置L2任务为arrived状态
        l2_task_found = False
        for task in tasks:
            if task.get('task_id') == task_id2:
                task['status'] = 'arrived'
                l2_task_found = True
                break
        assert l2_task_found, f"未找到L2任务 {task_id2} 进行状态更新"
        save_tasks(tasks)
        
        # 验证L2任务状态已更新
        l2_task_updated = get_task_by_id(task_id2)
        assert l2_task_updated is not None, f"更新后未能读取到L2任务 {task_id2}"
        assert l2_task_updated.get('status') == 'arrived', f"L2任务状态未设置为arrived: {l2_task_updated.get('status')}"
        
        # 重新加载所有任务，并筛选出已到达状态的任务
        all_tasks = load_tasks()
        arrived_tasks = [t for t in all_tasks if t.get('status') == 'arrived' and t.get('receiver') == 'EMP9875']
        
        # 详细打印任务列表（用于调试）
        task_info = "\n".join([f"任务ID: {t.get('task_id')}, 状态: {t.get('status')}, 安全等级: {t.get('security_level')}" for t in all_tasks])
        assert len(arrived_tasks) == 2, f"期望2个arrived任务，实际找到{len(arrived_tasks)}个任务：{[t.get('task_id') for t in arrived_tasks]}\n所有任务:\n{task_info}"
        
        # 验证任务安全等级
        arrived_levels = [t.get('security_level') for t in arrived_tasks]
        assert "L1" in arrived_levels, f"未找到L1级别的arrived任务，安全等级列表: {arrived_levels}"
        assert "L2" in arrived_levels, f"未找到L2级别的arrived任务，安全等级列表: {arrived_levels}"
        
        # 查询取件任务API
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        
        assert response.status_code == 200, f"查询取件任务失败: {response.json()}"
        data = response.json()
        assert data["success"] == True, f"取件任务查询未成功: {data}"
        assert data["total_tasks"] == 2, f"期望2个任务，实际返回{data['total_tasks']}个。任务列表: {data['tasks']}"
        
        # 验证API返回的任务安全等级
        returned_levels = [task["security_level"] for task in data["tasks"]]
        assert "L1" in returned_levels, f"API返回的任务中未找到L1任务，安全等级列表: {returned_levels}"
        assert "L2" in returned_levels, f"API返回的任务中未找到L2任务，安全等级列表: {returned_levels}"
    
    def test_pickup_complete_workflow(self):
        """测试完整的取件工作流程"""
        # 1. 创建任务
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 2. 查询取件任务（无认证）
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 1
        assert data["auth_status"]["authenticated"] == False
        
        # 3. 进行取件认证
        auth_data = self.authenticate_user_for_pickup("EMP9875", "L1")
        assert auth_data["verified"] == True
        
        # 4. 再次查询取件任务（有认证）
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 1
        assert data["auth_status"]["authenticated"] == True
        
        # 5. 执行取件
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": task_id
        })
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        
        # 6. 再次查询应该没有可取件任务了
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 0  # 任务已完成，不再显示
        assert summary["tasks_by_level"]["L3"] == 0
    
    def test_pickup_complete_workflow(self):
        """测试完整的取件工作流程"""
        # 1. 创建任务
        task_id, task_data = self.create_test_task_and_set_arrived()
        
        # 2. 查询取件任务（无认证）
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 1
        assert data["auth_status"]["authenticated"] == False
        
        # 3. 进行取件认证
        auth_data = self.authenticate_user_for_pickup("EMP9875", "L1")
        assert auth_data["verified"] == True
        
        # 4. 再次查询取件任务（有认证）
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 1
        assert data["auth_status"]["authenticated"] == True
        
        # 5. 执行取件
        response = client.post("/api/pickup/execute", json={
            "user_id": "EMP9875",
            "task_id": task_id
        })
        assert response.status_code == 200
        data = response.json()
        assert data["success"] == True
        
        # 6. 再次查询应该没有可取件任务了
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 0  # 任务已完成，不再显示
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 0  # 任务已完成，不再显示
        
        # 6. 再次查询应该没有可取件任务了
        response = client.get("/api/pickup/tasks", params={"user_id": "EMP9875"})
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 0  # 任务已完成，不再显示
        assert response.status_code == 200
        data = response.json()
        assert data["total_tasks"] == 0  # 任务已完成，不再显示
