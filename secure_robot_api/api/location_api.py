# secure_robot_api/api/location_api.py
from fastapi import APIRouter, HTTPException
from typing import List, Dict, Any
from services.task_service import load_locations

router = APIRouter()

@router.get("/", response_model=List[Dict[str, Any]])
async def get_all_locations():
    """获取所有可用位置"""
    try:
        locations = load_locations()
        return locations
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to load locations: {str(e)}")

@router.get("/{location_id}", response_model=Dict[str, Any])
async def get_location_by_id(location_id: str):
    """根据位置ID获取位置信息"""
    try:
        from services.task_service import get_location_by_id
        location = get_location_by_id(location_id)
        if not location:
            raise HTTPException(status_code=404, detail=f"Location {location_id} not found")
        return location
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get location: {str(e)}")
