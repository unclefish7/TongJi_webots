from fastapi import APIRouter, HTTPException
from typing import List, Optional
from services.auth_service import load_users, get_user_by_id

router = APIRouter()

@router.get("", response_model=List[dict])
async def get_all_users():
    """获取所有用户列表"""
    try:
        users = load_users()
        return users
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取用户列表失败: {str(e)}")

@router.get("/{user_id}", response_model=dict)
async def get_user(user_id: str):
    """根据用户ID获取用户信息"""
    try:
        user = get_user_by_id(user_id)
        if not user:
            raise HTTPException(status_code=404, detail="用户不存在")
        return user
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取用户信息失败: {str(e)}")
