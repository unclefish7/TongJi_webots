from fastapi import APIRouter, HTTPException, Query
from services.pickup_service import get_user_pickup_tasks, execute_pickup
from services.auth_service import get_pickup_auth_status
from models import PickupTasksResponse, PickupExecuteRequest, PickupExecuteResponse

router = APIRouter(tags=["pickup"])

@router.get("/tasks", response_model=PickupTasksResponse)
async def get_pickup_tasks(user_id: str = Query(..., description="用户ID")):
    """
    查询可取件任务
    
    - **user_id**: 用户ID（查询参数）
    
    查询条件：
    - Task.receiver == user_id
    - Task.status == "arrived"
    
    返回字段包括：task_id、description、security_level、locker_id
    """
    try:
        # 获取用户的可取件任务
        pickup_tasks = get_user_pickup_tasks(user_id)
        
        # 获取用户认证状态
        auth_status = get_pickup_auth_status(user_id)
        auth_info = None
        if auth_status:
            auth_info = {
                "authenticated": True,
                "level": auth_status.get('verified_level'),
                "expires_at": auth_status.get('expires_at'),
                "methods": auth_status.get('methods', [])
            }
        else:
            auth_info = {
                "authenticated": False,
                "level": None,
                "expires_at": None,
                "methods": []
            }
        
        return PickupTasksResponse(
            success=True,
            user_id=user_id,
            total_tasks=len(pickup_tasks),
            tasks=pickup_tasks,
            auth_status=auth_info
        )
        
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"查询取件任务失败: {str(e)}"
        )

@router.post("/execute", response_model=PickupExecuteResponse)
async def execute_pickup_task(request: PickupExecuteRequest):
    """
    执行取件操作
    
    - **user_id**: 用户ID
    - **task_id**: 任务ID
    
    处理流程：
    1. 查找该任务，确认 receiver 是否为当前用户、状态为 "arrived"
    2. 从 pickup_auth_cache 中查找该用户的认证信息
    3. 检查是否已过期，且 verified_level ≥ task.security_level
    4. 若通过校验：
       - 更新 Task.status = "completed"
       - 更新对应 Locker.status = "available"
       - 写入访问日志
       - 返回成功信息
    5. 若认证不足或过期，返回 403 + 提示重新认证
    """
    try:
        success, code, message = execute_pickup(request.user_id, request.task_id)
        
        if not success:
            # 根据错误码确定HTTP状态码
            status_code_map = {
                "PICKUP_001": 404,  # 任务不存在或不符合条件
                "PICKUP_002": 403,  # 认证不足或过期
                "PICKUP_003": 500,  # 系统错误
                "PICKUP_999": 500   # 未知错误
            }
            
            status_code = status_code_map.get(code, 400)
            
            raise HTTPException(
                status_code=status_code,
                detail={
                    "code": code,
                    "message": message
                }
            )
        
        return PickupExecuteResponse(
            success=True,
            code=code,
            message=message,
            task_id=request.task_id
        )
        
    except HTTPException:
        # 重新抛出HTTP异常
        raise
    except Exception as e:
        # 处理其他异常
        raise HTTPException(
            status_code=500,
            detail={
                "code": "PICKUP_999",
                "message": f"取件操作异常: {str(e)}"
            }
        )
