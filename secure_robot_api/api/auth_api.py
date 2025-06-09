from fastapi import APIRouter, HTTPException
from services.auth_service import verify_auth
from models import AuthRequest, AuthResponse

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
            verified=success,
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
