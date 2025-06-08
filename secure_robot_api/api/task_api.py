# api/task_api.py
from fastapi import APIRouter
# secure_robot_api/api/task_api.py
from models import PingResponse

router = APIRouter()

@router.get("/ping", response_model=PingResponse)
def ping():
    return {"message": "Secure Robot API 正常运行"}
