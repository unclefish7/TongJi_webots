# api/task_api.py
from fastapi import APIRouter, HTTPException
from datetime import datetime
from services.auth_service import consume_auth, get_user_by_id
from models import PingResponse, TaskCreateRequest, TaskCreateResponse

router = APIRouter()

@router.get("/ping", response_model=PingResponse)
def ping():
    return {"message": "Secure Robot API 正常运行"}

tags=["tasks"]

def generate_task_id() -> str:
    """生成任务ID"""
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    return f"T{timestamp}"

@router.post("/create", response_model=TaskCreateResponse)
async def create_task(request: TaskCreateRequest):
    """
    创建新任务
    
    - **user_id**: 发起人ID
    - **receiver**: 接收人ID  
    - **location_id**: 目标位置ID
    - **security_level**: 任务安全等级 (L1/L2/L3)
    - **description**: 任务描述（可选）
    
    注意：创建任务前会消费一次认证记录，每个认证记录只能使用一次
    """
    try:
        # 验证发起人是否存在
        initiator = get_user_by_id(request.user_id)
        if not initiator:
            raise HTTPException(
                status_code=404,
                detail="Initiator user not found"
            )
        
        # 验证接收人是否存在
        receiver = get_user_by_id(request.receiver)
        if not receiver:
            raise HTTPException(
                status_code=404,
                detail="Receiver user not found"
            )
        
        # 尝试消费认证记录
        auth_consumed = consume_auth(request.user_id, request.security_level)
        if not auth_consumed:
            raise HTTPException(
                status_code=403,
                detail=f"Insufficient authentication for security level {request.security_level}"
            )
        
        # 生成任务ID
        task_id = generate_task_id()
        
        # 这里可以添加任务保存逻辑（如保存到JSON文件）
        # 暂时只返回成功响应
        
        return TaskCreateResponse(
            success=True,
            task_id=task_id,
            message=f"Task created successfully with security level {request.security_level}"
        )
    
    except HTTPException:
        # 重新抛出HTTP异常
        raise
    except Exception as e:
        # 处理其他异常
        raise HTTPException(
            status_code=500,
            detail=f"Task creation error: {str(e)}"
        )
