#!/bin/bash

# 停止机器人配送系统脚本
echo "=== 停止机器人配送系统 ==="

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 停止前端服务
if [ -f /tmp/robot_frontend.pid ]; then
    echo -e "${YELLOW}停止前端服务...${NC}"
    kill $(cat /tmp/robot_frontend.pid) 2>/dev/null
    rm /tmp/robot_frontend.pid
    echo -e "${GREEN}✓ 前端服务已停止${NC}"
else
    echo -e "${YELLOW}未找到前端进程ID文件${NC}"
fi

# 停止后端服务
if [ -f /tmp/robot_backend.pid ]; then
    echo -e "${YELLOW}停止后端服务...${NC}"
    kill $(cat /tmp/robot_backend.pid) 2>/dev/null
    rm /tmp/robot_backend.pid
    echo -e "${GREEN}✓ 后端服务已停止${NC}"
else
    echo -e "${YELLOW}未找到后端进程ID文件${NC}"
fi

# 强制停止所有相关进程
echo -e "${YELLOW}检查并清理残留进程...${NC}"

# 停止uvicorn进程
pkill -f "uvicorn main:app" 2>/dev/null && echo -e "${GREEN}✓ 清理uvicorn进程${NC}"

# 停止npm dev进程
pkill -f "npm run dev" 2>/dev/null && echo -e "${GREEN}✓ 清理npm进程${NC}"

# 停止vite进程
pkill -f "vite" 2>/dev/null && echo -e "${GREEN}✓ 清理vite进程${NC}"

echo -e "${GREEN}=== 系统已完全停止 ===${NC}"
