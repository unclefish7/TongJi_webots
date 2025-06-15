from fastapi import APIRouter, HTTPException
from services.auth_service import verify_auth, verify_purpose_auth, get_auth_cache_status
from models import AuthRequest, AuthResponse, PurposeAuthRequest, PurposeAuthResponse

router = APIRouter(tags=["authentication"])

@router.post("/verify", response_model=AuthResponse)
async def verify_authentication(request: AuthRequest):
    """
    基于指定认证等级的用户认证验证
    
    - **user_id**: 用户ID（员工卡号）
    - **requested_level**: 请求的认证等级 (L1/L2/L3)
    - **provided**: 提供的认证信息，可能包含 l2_auth、l3_auth
    
    认证逻辑：
    1. 验证请求的认证等级不能超过用户本身的最大等级
    2. 根据请求的认证等级验证相应的凭证：
       - L1：只需 user_id 匹配即可
       - L2：需要验证 user_id + l2_auth
       - L3：需要验证 user_id + l2_auth + l3_auth
    
    注意：每次请求都是独立的一次性认证，不依赖登录状态或会话
    """
    try:
        success, verified_level, methods = verify_auth(
            request.user_id, 
            request.requested_level, 
            request.provided
        )
        
        if not success:
            # 认证失败
            raise HTTPException(
                status_code=401, 
                detail=f"Authentication failed for level {request.requested_level}"
            )
        
        return AuthResponse(
            success=success,
            verified_level=verified_level,
            methods=methods
        )
    
    except HTTPException:
        # 重新抛出HTTP异常
        raise
    except Exception as e:
        # 处理其他异常
        raise HTTPException(
            status_code=500,
            detail=f"Authentication service error: {str(e)}"
        )

@router.post("/verify_purpose", response_model=PurposeAuthResponse)
async def verify_purpose_authentication(request: PurposeAuthRequest):
    """
    基于用途的用户认证验证
    
    - **user_id**: 用户ID（员工卡号）
    - **purpose**: 本次认证的用途 ("send" 表示寄件，"pickup" 表示取件)
    - **requested_level**: 请求的认证等级 (L1/L2/L3)
    - **provided**: 提供的认证信息，可能包含 l2_auth、l3_auth
    
    认证逻辑：
    1. 校验用户是否存在，以及其 auth_level
    2. 按 requested_level 判断需要的认证信息：
       - L1：user_id 匹配即可
       - L2：需提供 l2_auth
       - L3：需提供 l2_auth + l3_auth
    3. 若认证失败，返回 verified=False 与提示 message
    
    认证结果缓存策略：
    - purpose == "send"：写入 auth_session_cache，结构包含 verified_level、used=False、timestamp
    - purpose == "pickup"：写入 pickup_auth_cache，结构包含 verified_level、started_at、expires_at（默认有效期 5 分钟）
    
    注意：所有认证均为"无状态的一次性认证"，无登录逻辑
    """
    try:
        success, verified_level, methods, expires_at, error_message = verify_purpose_auth(
            user_id=request.user_id,
            purpose=request.purpose,
            requested_level=request.requested_level,
            provided=request.provided
        )
        
        if not success:
            # 认证失败，返回错误信息
            return PurposeAuthResponse(
                verified=False,
                message=error_message or "认证失败，凭证无效或等级不足"
            )
        
        # 认证成功，构建响应数据
        response_data = {
            "verified": True,
            "verified_level": verified_level,
            "methods": methods
        }
        
        # 如果是取件认证，返回过期时间
        if request.purpose == "pickup" and expires_at:
            response_data["expires_at"] = expires_at
        
        return PurposeAuthResponse(**response_data)
    
    except Exception as e:
        # 处理其他异常
        raise HTTPException(
            status_code=500,
            detail=f"Authentication service error: {str(e)}"
        )

@router.get("/cache_status")
async def get_authentication_cache_status():
    """
    获取认证缓存状态（用于调试和监控）
    
    返回寄件认证缓存和取件认证缓存的状态信息
    """
    try:
        status = get_auth_cache_status()
        return {
            "success": True,
            "data": status
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get cache status: {str(e)}"
        )
