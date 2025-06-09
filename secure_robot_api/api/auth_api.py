from fastapi import APIRouter, HTTPException
from services.auth_service import verify_auth_with_purpose
from models import AuthRequest, AuthResponse

router = APIRouter(tags=["authentication"])

@router.post("/verify", response_model=AuthResponse)
async def verify_authentication(request: AuthRequest):
    """
    基于指定认证等级的用户认证验证
    
    - **user_id**: 用户ID（员工卡号）
    - **requested_level**: 请求的认证等级 (L1/L2/L3)
    - **provided**: 提供的认证信息，可能包含 l2_auth、l3_auth
    - **purpose**: 本次认证用途（"send" 表示寄件，"pickup" 表示取件）
    
    认证逻辑：
    1. 验证请求的认证等级不能超过用户本身最大等级
    2. 根据请求等级验证相应凭证：
       - L1：只需 user_id 匹配即可
       - L2：需验证 user_id + l2_auth
       - L3：需验证 user_id + l2_auth + l3_auth
    3. 根据认证用途写入不同缓存结构：
       - "send"：写入 `auth_session_cache`，可用于一次寄件任务，认证后即标记 `used=False`
       - "pickup"：写入 `pickup_auth_cache`，有效期为 5 分钟，在此期间可多次取件
    
    注意：所有认证请求均为无登录状态、无 session，仅根据本次提交的认证信息判断
    """
    try:
        success, verified_level, methods, expires_at = verify_auth_with_purpose(
            request.user_id, 
            request.requested_level, 
            request.provided,
            request.purpose
        )
        
        if not success:
            # 认证失败
            return AuthResponse(
                verified=False,
                message="认证失败，凭证无效或等级不足"
            )
        
        # 认证成功
        response_data = {
            "verified": True,
            "verified_level": verified_level,
            "methods": methods
        }
        
        # 如果是取件认证，返回过期时间
        if request.purpose == "pickup" and expires_at:
            response_data["expires_at"] = expires_at
        
        return AuthResponse(**response_data)
    
    except Exception as e:
        # 处理其他异常
        raise HTTPException(
            status_code=500,
            detail=f"Authentication service error: {str(e)}"
        )
