# secure_robot_api/main.py
from fastapi import FastAPI
from api.task_api import router as task_router

app = FastAPI(title="Secure Robot API")
app.include_router(task_router, prefix="/api/task")

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)

# 启动： uvicorn main:app --reload
