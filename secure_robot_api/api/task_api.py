# api/task_api.py
from fastapi import APIRouter, HTTPException
from services.task_service import create_task, complete_task, fail_task, TaskErrorCodes
from services.auth_service import load_users
from typing import Dict, List, Any
import json
import os
from models import (
    PingResponse, TaskCreateRequest, TaskCreateResponse,
    TaskCompleteRequest, TaskCompleteResponse,
    TaskFailRequest, TaskFailResponse
)

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
    
    返回编码说明：
    - TASK_000: 成功
    - TASK_001: 发起人不存在
    - TASK_002: 接收人不存在
    - TASK_003: 目标位置不存在
    - TASK_004: 安全等级无效
    - TASK_005: 认证不足
    - TASK_006: 无可用柜子
    - TASK_007: 柜子分配失败
    - TASK_008: 数据验证失败
    - TASK_009: 保存失败
    """
    try:
        success, code, message, task_id, task_data, locker_id = create_task(
            user_id=request.user_id,
            receiver=request.receiver,
            location_id=request.location_id,
            security_level=request.security_level,
            description=request.description
        )
        
        if not success:
            # 根据错误编码确定HTTP状态码
            status_code_map = {
                TaskErrorCodes.INITIATOR_NOT_FOUND: 404,
                TaskErrorCodes.RECEIVER_NOT_FOUND: 404,
                TaskErrorCodes.LOCATION_NOT_FOUND: 404,
                TaskErrorCodes.INVALID_SECURITY_LEVEL: 400,
                TaskErrorCodes.INSUFFICIENT_AUTH: 403,
                TaskErrorCodes.NO_AVAILABLE_LOCKERS: 503,
                TaskErrorCodes.LOCKER_ALLOCATION_FAILED: 500,
                TaskErrorCodes.VALIDATION_FAILED: 400,
                TaskErrorCodes.SAVE_FAILED: 500
            }
            
            status_code = status_code_map.get(code, 400)
            
            raise HTTPException(
                status_code=status_code,
                detail={
                    "code": code,
                    "message": message
                }
            )
        
        return TaskCreateResponse(
            success=True,
            code=code,
            message=message,
            task_id=task_id,
            task=task_data,
            locker_id=locker_id
        )
    
    except HTTPException:
        # 重新抛出HTTP异常
        raise
    except Exception as e:
        # 处理其他异常
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999",
                "message": f"Unexpected error: {str(e)}"
            }
        )

@router.post("/complete", response_model=TaskCompleteResponse)
async def complete_task_endpoint(request: TaskCompleteRequest):
    """
    完成任务
    
    - **task_id**: 任务ID
    """
    try:
        success, code, message, task_data = complete_task(request.task_id)
        
        if not success:
            status_code = 404 if code == TaskErrorCodes.TASK_NOT_FOUND else 500
            raise HTTPException(
                status_code=status_code,
                detail={
                    "code": code,
                    "message": message
                }
            )
        
        return TaskCompleteResponse(
            success=True,
            code=code,
            message=message,
            task=task_data
        )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999",
                "message": f"Unexpected error: {str(e)}"
            }
        )

@router.post("/fail", response_model=TaskFailResponse)
async def fail_task_endpoint(request: TaskFailRequest):
    """
    标记任务失败
    
    - **task_id**: 任务ID
    - **reason**: 失败原因（可选）
    """
    try:
        success, code, message, task_data = fail_task(request.task_id, request.reason)
        
        if not success:
            status_code = 404 if code == TaskErrorCodes.TASK_NOT_FOUND else 500
            raise HTTPException(
                status_code=status_code,
                detail={
                    "code": code,
                    "message": message
                }
            )
        
        return TaskFailResponse(
            success=True,
            code=code,
            message=message,
            task=task_data
        )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999",
                "message": f"Unexpected error: {str(e)}"
            }
        )

@router.get("/tasks/user/{user_id}")
async def get_user_tasks(user_id: str, status: str = None):
    """获取指定用户的任务列表"""
    try:
        # 这里先返回示例数据，后续可以从数据库获取真实数据
        tasks = []
        return {"tasks": tasks, "total": len(tasks)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取用户任务失败: {str(e)}")

@router.get("/tasks/user/{user_id}/pending-pickup")
async def get_user_pending_pickup_tasks(user_id: str):
    """获取指定用户的待取件任务"""
    try:
        # 这里先返回示例数据，后续可以从数据库获取真实数据
        tasks = []
        return {"tasks": tasks, "total": len(tasks)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取待取件任务失败: {str(e)}")
