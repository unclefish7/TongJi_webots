# secure_robot_api/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api.task_api import router as task_router
from api.auth_api import router as auth_router
from api.pickup_api import router as pickup_router
from api.user_api import router as user_router

app = FastAPI(title="Secure Robot API")

# 配置 CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],  # Vue dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(task_router, prefix="/api")
app.include_router(auth_router, prefix="/api")
app.include_router(pickup_router, prefix="/api")
app.include_router(user_router)

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
