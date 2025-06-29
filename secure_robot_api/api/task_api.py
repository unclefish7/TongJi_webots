# api/task_api.py
from fastapi import APIRouter, HTTPException
from services.task_service import (
    create_task, complete_task, fail_task, TaskErrorCodes,
    remove_task_from_queue, handle_robot_arrival, send_next_to_robot,
    get_queue_status, start_task_execution, check_ros2_bridge_connection,
    clear_arrived_task, handle_robot_optimized_order, set_downgrade_strategy,
    get_current_downgrade_strategy
)
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
    - **task_type**: 任务类型 (call-呼叫任务/send-寄送任务)
    - **description**: 任务描述（可选）
    
    返回编码说明：
    - TASK_000: 成功
    - TASK_001: 发起人不存在
    - TASK_002: 接收人不存在
    - TASK_003: 目标位置不存在
    - TASK_006: 无可用柜子（仅send任务）
    - TASK_007: 柜子分配失败（仅send任务）
    - TASK_008: 数据验证失败
    - TASK_009: 保存失败
    """
    try:
        success, code, message, task_id, task_data, locker_id = create_task(
            user_id=request.user_id,
            receiver=request.receiver,
            location_id=request.location_id,
            security_level=request.security_level,
            task_type=request.task_type,
            description=request.description
        )
        
        if not success:
            # 根据错误编码确定HTTP状态码
            status_code_map = {
                TaskErrorCodes.INITIATOR_NOT_FOUND: 404,
                TaskErrorCodes.RECEIVER_NOT_FOUND: 404,
                TaskErrorCodes.LOCATION_NOT_FOUND: 404,
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

@router.post("/cancel/{task_id}")
async def cancel_task(task_id: str):
    """
    取消任务（从队列中移除）
    
    - **task_id**: 要取消的任务ID
    """
    try:
        success, message = remove_task_from_queue(task_id)
        
        if not success:
            raise HTTPException(
                status_code=404,
                detail={
                    "code": "TASK_010",
                    "message": message
                }
            )
        
        return {
            "success": True,
            "code": "TASK_000",
            "message": message
        }
    
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

@router.post("/robot/arrived")
async def robot_arrived():
    """
    机器人到达通知接口（由ROS2桥接调用）
    """
    try:
        success, message = handle_robot_arrival()
        
        return {
            "success": success,
            "message": message
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999", 
                "message": f"Error handling robot arrival: {str(e)}"
            }
        )

@router.post("/robot/next")
async def send_next():
    """
    发送next指令让机器人继续
    """
    try:
        success, message = send_next_to_robot()
        
        if not success:
            raise HTTPException(
                status_code=400,
                detail={
                    "code": "TASK_011",
                    "message": message
                }
            )
        
        return {
            "success": True,
            "message": message
        }
    
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

@router.get("/queue/status")
async def get_queue_status_endpoint():
    """
    获取当前任务队列状态
    """
    try:
        status = get_queue_status()
        return {
            "success": True,
            "data": status
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999",
                "message": f"Error getting queue status: {str(e)}"
            }
        )

@router.get("/ros2/status")
async def check_ros2_status():
    """
    检查ROS2桥接服务状态
    """
    try:
        available, status = check_ros2_bridge_connection()
        
        return {
            "success": True,
            "data": {
                "available": available,
                "status": status,
                "bridge_url": "http://localhost:8080"
            }
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999",
                "message": f"Error checking ROS2 status: {str(e)}"
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

@router.post("/start")
async def start_task():
    """
    启动队列中的下一个任务
    
    手动启动最高优先级的待执行任务。
    创建任务后，任务会进入对应优先级队列，但不会自动执行。
    需要调用此接口来启动任务执行。
    """
    try:
        success, message = start_task_execution()
        
        if not success:
            raise HTTPException(
                status_code=400,
                detail={
                    "code": "TASK_013",
                    "message": message
                }
            )
        
        return {
            "success": True,
            "code": "TASK_000",
            "message": message
        }
    
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

@router.post("/robot/optimized_order")
async def robot_optimized_order(request: dict):
    """
    机器人通知优化后的任务执行顺序（由ROS2桥接调用）
    
    请求体包含:
    - original_order: 原始任务顺序（位置名称列表）
    - optimized_order: 优化后的任务顺序（位置名称列表）
    """
    try:
        original_order = request.get("original_order", [])
        optimized_order = request.get("optimized_order", [])
        
        success, message = handle_robot_optimized_order(original_order, optimized_order)
        
        return {
            "success": success,
            "message": message,
            "original_order": original_order,
            "optimized_order": optimized_order
        }
    
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999", 
                "message": f"Error handling optimized order: {str(e)}"
            }
        )

@router.get("/downgrade/strategy")
async def get_downgrade_strategy():
    """
    获取当前降级策略信息
    
    返回当前使用的降级策略、规则和可用策略列表
    """
    try:
        strategy_info = get_current_downgrade_strategy()
        return {
            "success": True,
            "data": strategy_info
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999",
                "message": f"Error getting downgrade strategy: {str(e)}"
            }
        )

@router.post("/downgrade/strategy")
async def set_downgrade_strategy_endpoint(request: Dict[str, str]):
    """
    设置降级策略
    
    - **strategy**: 降级策略名称
      - "step_by_step": 只有L1降级 (L3→L3, L2→L2, L1→L0, L0→L0)
      - "flat_high": 高优先级平级回退 (L3→L1, L2→L1, L1→L0)  
      - "all_step": 全部降一级 (L3→L2, L2→L1, L1→L0, L0→L0)
    """
    try:
        strategy = request.get("strategy")
        if not strategy:
            raise HTTPException(
                status_code=400,
                detail={
                    "code": "TASK_400",
                    "message": "Strategy parameter is required"
                }
            )
        
        success, message = set_downgrade_strategy(strategy)
        
        if success:
            return {
                "success": True,
                "message": message,
                "current_strategy": get_current_downgrade_strategy()
            }
        else:
            raise HTTPException(
                status_code=400,
                detail={
                    "code": "TASK_400",
                    "message": message
                }
            )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail={
                "code": "TASK_999",
                "message": f"Error setting downgrade strategy: {str(e)}"
            }
        )

@router.post("/pickup/complete/{task_id}")
async def complete_pickup(task_id: str):
    """
    完成取件操作（清除到达状态，仅适用于send任务）
    
    - **task_id**: 要完成取件的任务ID
    """
    try:
        success, message = clear_arrived_task(task_id)
        
        if not success:
            raise HTTPException(
                status_code=400,
                detail={
                    "code": "TASK_014",
                    "message": message
                }
            )
        
        return {
            "success": True,
            "code": "TASK_000",
            "message": message
        }
    
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

@router.post("/robot/enhanced_optimized_order")
async def robot_enhanced_optimized_order(request: Dict[str, Any]):
    """
    处理机器人发送的增强优化任务顺序（支持包含权重的任务信息）
    
    请求格式：
    {
        "optimized_tasks": [
            {
                "task_id": "任务ID",
                "final_priority": 5.2,
                "distance_from_current": 15.2,
                "optimization_reason": "waiting_time_priority"
            }
        ]
    }
    """
    try:
        optimized_tasks = request.get("optimized_tasks", [])
        
        if not optimized_tasks:
            raise HTTPException(status_code=400, detail="Missing optimized_tasks")
        
        # 记录接收到的优化结果（调试用）
        print(f"Received enhanced optimized order with {len(optimized_tasks)} tasks:")
        for i, task in enumerate(optimized_tasks):
            task_id = task.get("task_id", "NO_ID")
            priority = task.get("final_priority", "N/A")
            distance = task.get("distance_from_current", "N/A")
            reason = task.get("optimization_reason", "N/A")
            print(f"  Task {i+1}: {task_id} (priority: {priority}, distance: {distance}m, reason: {reason})")
        
        success, message = handle_robot_optimized_order(optimized_tasks)
        
        if success:
            return {
                "success": True,
                "message": message,
                "processed_tasks": len(optimized_tasks)
            }
        else:
            raise HTTPException(status_code=400, detail=message)
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")
