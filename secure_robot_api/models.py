from pydantic import BaseModel
from typing import Dict, List, Optional

class PingResponse(BaseModel):
    message: str

class AuthRequest(BaseModel):
    user_id: str
    requested_level: str  # 请求的认证等级 L1/L2/L3
    provided: Dict[str, str]  # 可能包含 l2_auth、l3_auth

class AuthResponse(BaseModel):
    verified: bool
    verified_level: str
    methods: List[str]

class TaskCreateRequest(BaseModel):
    user_id: str  # 发起人ID
    receiver: str  # 接收人ID
    location_id: str  # 目标位置ID
    security_level: str  # 任务安全等级 L1/L2/L3
    description: Optional[str] = None  # 任务描述

class TaskCreateResponse(BaseModel):
    success: bool
    task_id: Optional[str] = None
    message: str