from fastapi import APIRouter, HTTPException
from typing import List, Dict, Optional, Any
from datetime import datetime, timedelta
from pydantic import BaseModel
import json
import os

router = APIRouter()

# 机器人任务数据模型
class RobotTask(BaseModel):
    id: str
    type: str  # 'delivery' | 'pickup' | 'call'
    location: str
    user_id: str
    description: Optional[str] = None
    priority: int = 1
    created_at: str
    status: str = 'pending'  # 'pending' | 'in_progress' | 'completed' | 'failed'
    estimated_arrival: Optional[str] = None

class AddTaskRequest(BaseModel):
    type: str
    location: str
    user_id: str
    description: Optional[str] = None
    priority: Optional[int] = 1

class QueueStatus(BaseModel):
    total_tasks: int
    pending_tasks: int
    current_task: Optional[RobotTask] = None
    robot_status: str = 'idle'  # 'idle' | 'moving' | 'waiting' | 'error'
    current_location: Optional[str] = None

# 内存中的任务队列
task_queue: List[RobotTask] = []
current_task: Optional[RobotTask] = None
robot_status = 'idle'

@router.post("/robot/queue/add", response_model=Dict[str, Any])
async def add_task_to_queue(request: AddTaskRequest):
    """添加任务到机器人队列"""
    try:
        # 生成任务ID
        task_id = f"{request.type}_{request.user_id}_{int(datetime.now().timestamp())}"
        
        # 创建任务
        task = RobotTask(
            id=task_id,
            type=request.type,
            location=request.location,
            user_id=request.user_id,
            description=request.description or f"{request.type}任务到{request.location}",
            priority=request.priority or (1 if request.type == 'pickup' else 2 if request.type == 'delivery' else 3),
            created_at=datetime.now().isoformat(),
            status='pending'
        )
        
        # 添加到队列
        task_queue.append(task)
        
        # 按优先级排序
        task_queue.sort(key=lambda x: x.priority)
        
        # 记录日志
        from services.log_service import create_log
        create_log(
            log_type="system",
            message=f"Robot task added: {task.type} to {task.location} by {task.user_id}",
            related_user=request.user_id,
            result="success"
        )
        
        return {
            "success": True,
            "message": "任务已添加到队列",
            "task_id": task_id,
            "queue_position": len(task_queue),
            "task": task.dict()
        }
        
    except Exception as e:
        from services.log_service import log_error
        log_error(f"Failed to add robot task: {str(e)}", request.user_id)
        raise HTTPException(status_code=500, detail=f"添加任务失败: {str(e)}")

@router.get("/robot/queue/status", response_model=QueueStatus)
async def get_queue_status():
    """获取机器人队列状态"""
    try:
        return QueueStatus(
            total_tasks=len(task_queue) + (1 if current_task else 0),
            pending_tasks=len(task_queue),
            current_task=current_task,
            robot_status=robot_status,
            current_location=current_task.location if current_task else None
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取队列状态失败: {str(e)}")

@router.get("/robot/queue/tasks", response_model=List[RobotTask])
async def get_all_tasks():
    """获取所有任务"""
    try:
        all_tasks = []
        if current_task:
            all_tasks.append(current_task)
        all_tasks.extend(task_queue)
        return all_tasks
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取任务列表失败: {str(e)}")

@router.delete("/robot/queue/task/{task_id}")
async def remove_task(task_id: str):
    """移除指定任务"""
    try:
        global task_queue
        
        # 从队列中移除
        original_length = len(task_queue)
        task_queue = [task for task in task_queue if task.id != task_id]
        
        if len(task_queue) < original_length:
            return {"success": True, "message": "任务已移除"}
        else:
            return {"success": False, "message": "未找到指定任务"}
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"移除任务失败: {str(e)}")

@router.post("/robot/queue/clear")
async def clear_queue():
    """清空队列"""
    try:
        global task_queue
        task_queue.clear()
        
        return {"success": True, "message": "队列已清空"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"清空队列失败: {str(e)}")

@router.post("/robot/task/start/{task_id}")
async def start_task(task_id: str):
    """开始执行任务"""
    try:
        global current_task, robot_status, task_queue
        
        # 从队列中取出任务
        task_index = -1
        for i, task in enumerate(task_queue):
            if task.id == task_id:
                task_index = i
                break
        
        if task_index == -1:
            return {"success": False, "message": "未找到指定任务"}
        
        # 设置为当前任务
        current_task = task_queue.pop(task_index)
        current_task.status = 'in_progress'
        robot_status = 'moving'
        
        # 记录日志
        from services.log_service import create_log
        create_log(
            log_type="system",
            message=f"Robot task started: {current_task.type} to {current_task.location}",
            related_user=current_task.user_id,
            result="success"
        )
        
        return {
            "success": True,
            "message": "任务已开始执行",
            "task": current_task.dict()
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"开始任务失败: {str(e)}")

@router.post("/robot/task/complete")
async def complete_current_task():
    """完成当前任务"""
    try:
        global current_task, robot_status
        
        if not current_task:
            return {"success": False, "message": "没有正在执行的任务"}
        
        # 记录日志
        from services.log_service import create_log
        create_log(
            log_type="system",
            message=f"Robot task completed: {current_task.type} at {current_task.location}",
            related_user=current_task.user_id,
            result="success"
        )
        
        completed_task = current_task
        completed_task.status = 'completed'
        current_task = None
        robot_status = 'idle'
        
        return {
            "success": True,
            "message": "任务已完成",
            "completed_task": completed_task.dict()
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"完成任务失败: {str(e)}")

@router.post("/robot/task/fail")
async def fail_current_task(reason: Optional[str] = None):
    """任务失败"""
    try:
        global current_task, robot_status
        
        if not current_task:
            return {"success": False, "message": "没有正在执行的任务"}
        
        # 记录日志
        from services.log_service import create_log
        create_log(
            log_type="error",
            message=f"Robot task failed: {current_task.type} at {current_task.location}. Reason: {reason or 'Unknown'}",
            related_user=current_task.user_id,
            result="failed"
        )
        
        failed_task = current_task
        failed_task.status = 'failed'
        current_task = None
        robot_status = 'error'
        
        return {
            "success": True,
            "message": "任务已标记为失败",
            "failed_task": failed_task.dict()
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"任务失败处理失败: {str(e)}")

@router.post("/robot/status/update")
async def update_robot_status(status: str, location: Optional[str] = None):
    """更新机器人状态"""
    try:
        global robot_status
        
        valid_statuses = ['idle', 'moving', 'waiting', 'error']
        if status not in valid_statuses:
            return {"success": False, "message": f"无效的状态: {status}"}
        
        robot_status = status
        
        # 如果提供了位置信息，更新当前任务的位置
        if location and current_task:
            current_task.estimated_arrival = datetime.now().isoformat()
        
        return {
            "success": True,
            "message": "机器人状态已更新",
            "status": robot_status
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"更新状态失败: {str(e)}")

@router.post("/queue/{task_id}/start")
async def start_task(task_id: str):
    """开始执行指定任务"""
    try:
        # 这里先返回成功，实际实现时需要调用任务处理逻辑
        return {"success": True, "message": f"任务 {task_id} 已开始执行"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"开始任务失败: {str(e)}")

@router.post("/queue/{task_id}/fail")
async def fail_task(task_id: str, request: dict):
    """标记任务失败"""
    try:
        reason = request.get("reason", "未知原因")
        # 这里先返回成功，实际实现时需要调用任务处理逻辑
        return {"success": True, "message": f"任务 {task_id} 已标记为失败: {reason}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"标记任务失败时出错: {str(e)}")

@router.get("/queue")
async def get_task_queue():
    """获取当前任务队列"""
    try:
        # 这里先返回示例数据，实际实现需要从数据库或缓存获取
        tasks = [
            {
                "id": "task_001",
                "type": "delivery",
                "location": "经理室",
                "user_id": "EMP7064",
                "description": "配送包裹到经理室",
                "priority": 2,
                "status": "pending",
                "created_at": "2025-06-15T10:00:00Z"
            }
        ]
        return {"tasks": tasks, "total": len(tasks)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取任务队列失败: {str(e)}")
