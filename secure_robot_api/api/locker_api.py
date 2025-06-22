from fastapi import APIRouter, HTTPException
from services.task_service import get_all_lockers_status
from models import LockerStatusResponse

router = APIRouter(tags=["locker"])

@router.get("/status", response_model=LockerStatusResponse)
async def get_lockers_status():
    """
    获取所有储物柜的状态
    
    返回所有储物柜的状态信息，包括：
    - locker_id: 柜子ID
    - status: 柜子状态（available/occupied等）
    - 其他相关信息
    """
    try:
        lockers = get_all_lockers_status()
        
        return LockerStatusResponse(
            success=True,
            total_lockers=len(lockers),
            lockers=lockers,
            message="成功获取所有储物柜状态"
        )
        
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"获取储物柜状态失败: {str(e)}"
        )
