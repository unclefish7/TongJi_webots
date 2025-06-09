from fastapi import APIRouter, HTTPException
from services.auth_service import verify_auth
from models import AuthRequest, AuthResponse

router = APIRouter(tags=["authentication"])

@router.post("/verify", response_model=AuthResponse)
async def verify_authentication(request: AuthRequest):
    """
    一次性用户认证验证
    
    - **user_id**: 用户ID（员工卡号）
    - **provided**: 提供的认证信息，可能包含 otp、face_id
    
    认证模式（基于用户的 auth_level）：
    - L1：只需 user_id 匹配即可
    - L2：需要验证 user_id + otp
    - L3：需要验证 user_id + otp + face_id
    
    注意：每次请求都是独立的一次性认证，不依赖登录状态或会话
    """
    try:
        success, verified_level, methods = verify_auth(request.user_id, request.provided)
        
        if not success:
            # 认证失败
            raise HTTPException(
                status_code=401, 
                detail="Authentication failed"
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
