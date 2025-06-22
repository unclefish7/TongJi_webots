from pydantic import BaseModel
from typing import Dict, List, Optional, Literal

class PingResponse(BaseModel):
    message: str

class AuthRequest(BaseModel):
    user_id: str
    purpose: Optional[str] = None  # 用途：send/pickup，可选字段
    requested_level: str  # 请求的认证等级 L1/L2/L3
    provided: Dict[str, str]  # 可能包含 l2_auth、l3_auth

class AuthResponse(BaseModel):
    success: bool  # 改为success以匹配frontend期望
    verified_level: Optional[str] = None
    methods: Optional[List[str]] = None
    expires_at: Optional[str] = None
    error: Optional[str] = None

class TaskCreateRequest(BaseModel):
    user_id: str  # 发起人ID
    receiver: str  # 接收人ID
    location_id: str  # 目标位置ID
    security_level: str  # 任务安全等级 L1/L2/L3
    task_type: Literal["call", "send"]  # 任务类型：call-呼叫任务，send-寄送任务
    description: Optional[str] = None  # 任务描述

class TaskCreateResponse(BaseModel):
    success: bool
    code: str  # 响应编码
    message: str  # 详细说明
    task_id: Optional[str] = None
    task: Optional[Dict] = None  # 完整任务信息
    locker_id: Optional[str] = None  # 分配的柜子ID

class TaskStatusUpdateRequest(BaseModel):
    task_id: str
    new_status: str
    timestamp_field: Optional[str] = None  # accepted/delivered

class TaskStatusUpdateResponse(BaseModel):
    success: bool
    code: str
    message: str

class TaskQueryResponse(BaseModel):
    tasks: List[Dict]
    total: int

class Task(BaseModel):
    task_id: str
    task_type: str  # 任务类型：call/send
    description: Optional[str]
    initiator: str
    receiver: str
    location_id: str
    locker_id: Optional[str] = None  # 柜子ID字段改为可选（仅send任务需要）
    security_level: str
    status: str
    timestamps: Dict[str, Optional[str]]

class TaskCompleteRequest(BaseModel):
    task_id: str

class TaskCompleteResponse(BaseModel):
    success: bool
    code: str
    message: str
    task: Optional[Dict] = None

class TaskFailRequest(BaseModel):
    task_id: str
    reason: Optional[str] = None

class TaskFailResponse(BaseModel):
    success: bool
    code: str
    message: str
    task: Optional[Dict] = None

class PurposeAuthRequest(BaseModel):
    user_id: str
    purpose: Literal["send", "pickup"]  # 本次认证的用途
    requested_level: Literal["L1", "L2", "L3"]
    provided: Dict[str, str]  # 提供的认证信息，如 l2_auth, l3_auth

class PurposeAuthResponse(BaseModel):
    verified: bool
    verified_level: Optional[str] = None
    methods: Optional[List[str]] = None
    expires_at: Optional[str] = None  # 仅在 purpose=pickup 时返回
    message: Optional[str] = None

class PickupTasksResponse(BaseModel):
    success: bool
    user_id: str
    total_tasks: int
    tasks: List[Dict]
    auth_status: Optional[Dict] = None

class PickupExecuteRequest(BaseModel):
    user_id: str
    task_id: str

class PickupExecuteResponse(BaseModel):
    success: bool
    code: str
    message: str
    task_id: Optional[str] = None
    locker_id: Optional[str] = None  # 分配的柜子ID
    pickup_time: Optional[str] = None  # 取件时间
    auth_status: Optional[Dict] = None  # 认证状态

class LockerStatusResponse(BaseModel):
    success: bool
    total_lockers: int
    lockers: List[Dict]
    message: Optional[str] = None