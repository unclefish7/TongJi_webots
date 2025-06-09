# api/task_api.py
from fastapi import APIRouter, HTTPException
from services.task_service import create_task
from models import PingResponse, TaskCreateRequest, TaskCreateResponse

router = APIRouter(tags=["tasks"])

@router.get("/ping", response_model=PingResponse)
def ping():
    return {"message": "Secure Robot API 正常运行"}

@router.post("/create", response_model=TaskCreateResponse)
async def create_task_endpoint(request: TaskCreateRequest):
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
        success, message, task_id = create_task(
            user_id=request.user_id,
            receiver=request.receiver,
            location_id=request.location_id,
            security_level=request.security_level,
            description=request.description
        )
        
        if not success:
            # 根据错误信息确定HTTP状态码
            if "not found" in message:
                status_code = 404
            elif "Insufficient authentication" in message:
                status_code = 403
            elif "Invalid" in message:
                status_code = 400
            else:
                status_code = 400
            
            raise HTTPException(
                status_code=status_code,
                detail=message
            )
        
        return TaskCreateResponse(
            success=True,
            task_id=task_id,
            message=message
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
