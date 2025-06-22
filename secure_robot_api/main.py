# secure_robot_api/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api.task_api import router as task_router
from api.auth_api import router as auth_router
from api.pickup_api import router as pickup_router
from api.user_api import router as user_router
from api.location_api import router as location_router
from api.voice_api import router as voice_router

import subprocess
import requests
import time
import os
import signal

app = FastAPI(title="Secure Robot API")

# 配置 CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],  # Vue dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(task_router, prefix="/api/tasks")
app.include_router(auth_router, prefix="/api/auth")
app.include_router(pickup_router, prefix="/api/pickup")
app.include_router(user_router, prefix="/api/user")
app.include_router(location_router, prefix="/api/locations")
app.include_router(voice_router, prefix="/api/voice")

# Ollama进程句柄
ollama_process = None

def check_ollama_installed():
    """检查Ollama是否已安装"""
    try:
        result = subprocess.run(["ollama", "--version"], 
                              capture_output=True, text=True, timeout=5)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False

def check_ollama_running():
    """检查Ollama服务是否正在运行"""
    try:
        response = requests.get("http://localhost:11434/api/tags", timeout=3)
        return response.status_code == 200
    except:
        return False

def start_ollama_service():
    """启动Ollama服务"""
    global ollama_process
    
    print("🤖 正在启动Ollama服务...")
    try:
        # 在后台启动ollama serve
        ollama_process = subprocess.Popen(
            ["ollama", "serve"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid if os.name != 'nt' else None
        )
        
        # 等待服务启动
        for i in range(10):  # 最多等待10秒
            time.sleep(1)
            if check_ollama_running():
                print("✅ Ollama服务启动成功")
                return True
            print(f"⏳ 等待Ollama启动... ({i+1}/10)")
        
        print("❌ Ollama服务启动超时")
        return False
        
    except FileNotFoundError:
        print("❌ 找不到ollama命令，请先安装Ollama")
        return False
    except Exception as e:
        print(f"❌ 启动Ollama服务失败: {e}")
        return False

def stop_ollama_service():
    """停止Ollama服务"""
    global ollama_process
    
    if ollama_process:
        try:
            if os.name != 'nt':
                # Linux/macOS: 杀死进程组
                os.killpg(os.getpgid(ollama_process.pid), signal.SIGTERM)
            else:
                # Windows: 杀死进程
                ollama_process.terminate()
            
            ollama_process.wait(timeout=5)
            print("🛑 Ollama服务已停止")
        except:
            if ollama_process:
                ollama_process.kill()
        finally:
            ollama_process = None

def setup_ollama():
    """设置和启动Ollama服务"""
    print("🔍 检查Ollama状态...")
    
    # 检查是否已安装
    if not check_ollama_installed():
        print("⚠️  未检测到Ollama，跳过启动")
        print("💡 提示: 运行 ./setup_offline_llm.sh 安装Ollama")
        return False
    
    # 检查是否已在运行
    if check_ollama_running():
        print("✅ Ollama服务已在运行")
        return True
    
    # 尝试启动服务
    return start_ollama_service()

# 信号处理函数
def signal_handler(signum, frame):
    """处理程序退出信号"""
    print("\n🔄 正在优雅关闭服务...")
    stop_ollama_service()
    exit(0)

# 注册信号处理器
if os.name != 'nt':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

if __name__ == "__main__":
    import uvicorn
    
    print("🚀 启动机器人语音识别API服务")
    print("=" * 50)
    
    # 设置和启动Ollama服务
    ollama_status = setup_ollama()
    if ollama_status:
        print("🎯 系统将优先使用离线LLM进行语义分析")
    else:
        print("⚠️  系统将使用在线LLM或关键词匹配")
    
    print("=" * 50)
    print("🌐 API服务地址: http://localhost:8000")
    print("📚 API文档地址: http://localhost:8000/docs")
    print("🎤 语音识别端点: /api/voice/recognize")
    print("=" * 50)
    print("按 Ctrl+C 停止服务")
    print()
    
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    except KeyboardInterrupt:
        print("\n🔄 收到停止信号...")
    finally:
        stop_ollama_service()
        print("👋 服务已关闭")
