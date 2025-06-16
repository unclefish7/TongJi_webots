#!/bin/bash

# 机器人配送系统启动脚本
echo "=== 启动机器人配送系统 ==="

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查Python环境
echo -e "${YELLOW}检查Python环境...${NC}"
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}错误: 未找到Python3${NC}"
    exit 1
fi

# 检查Node.js环境
echo -e "${YELLOW}检查Node.js环境...${NC}"
if ! command -v npm &> /dev/null; then
    echo -e "${RED}错误: 未找到npm${NC}"
    exit 1
fi

# 启动后端API服务
echo -e "${YELLOW}启动后端API服务...${NC}"
cd /home/jerry/Documents/webots/secure_robot_api

# 安装Python依赖（如果需要）
if [ ! -d "venv" ]; then
    echo "创建Python虚拟环境..."
    python3 -m venv venv
fi

source venv/bin/activate
pip install -r requirements.txt

# 启动FastAPI服务
echo -e "${GREEN}启动FastAPI服务器 (端口8000)...${NC}"
uvicorn main:app --host 0.0.0.0 --port 8000 --reload &
BACKEND_PID=$!

# 等待后端启动
sleep 3

# 测试后端连接
echo -e "${YELLOW}测试后端API连接...${NC}"
if curl -s http://localhost:8000/api/tasks/ping > /dev/null; then
    echo -e "${GREEN}✓ 后端API启动成功${NC}"
else
    echo -e "${RED}✗ 后端API启动失败${NC}"
    kill $BACKEND_PID 2>/dev/null
    exit 1
fi

# 启动前端服务
echo -e "${YELLOW}启动前端服务...${NC}"
cd /home/jerry/Documents/webots/robot-delivery-system

# 安装Node.js依赖（如果需要）
if [ ! -d "node_modules" ]; then
    echo "安装前端依赖..."
    npm install
fi

# 启动Vue.js开发服务器
echo -e "${GREEN}启动Vue.js开发服务器 (端口5173)...${NC}"
npm run dev &
FRONTEND_PID=$!

# 等待前端启动
sleep 5

# 测试前端连接
echo -e "${YELLOW}测试前端连接...${NC}"
if curl -s http://localhost:5173 > /dev/null; then
    echo -e "${GREEN}✓ 前端服务启动成功${NC}"
else
    echo -e "${RED}✗ 前端服务启动失败${NC}"
    kill $BACKEND_PID $FRONTEND_PID 2>/dev/null
    exit 1
fi

# 显示启动信息
echo ""
echo -e "${GREEN}=== 系统启动完成 ===${NC}"
echo -e "后端API: ${GREEN}http://localhost:8000${NC}"
echo -e "前端界面: ${GREEN}http://localhost:5173${NC}"
echo -e "API测试页面: ${GREEN}http://localhost:5173/test${NC}"
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止所有服务${NC}"

# 保存进程ID
echo $BACKEND_PID > /tmp/robot_backend.pid
echo $FRONTEND_PID > /tmp/robot_frontend.pid

# 等待用户中断
wait_for_interrupt() {
    while true; do
        sleep 1
    done
}

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}正在停止服务...${NC}"
    
    # 停止前端
    if [ -f /tmp/robot_frontend.pid ]; then
        kill $(cat /tmp/robot_frontend.pid) 2>/dev/null
        rm /tmp/robot_frontend.pid
    fi
    
    # 停止后端
    if [ -f /tmp/robot_backend.pid ]; then
        kill $(cat /tmp/robot_backend.pid) 2>/dev/null
        rm /tmp/robot_backend.pid
    fi
    
    echo -e "${GREEN}所有服务已停止${NC}"
    exit 0
}

# 注册信号处理
trap cleanup SIGINT SIGTERM

# 等待中断
wait_for_interrupt
