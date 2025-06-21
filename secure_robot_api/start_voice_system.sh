#!/bin/bash

# 语音识别系统启动脚本

echo "=========================================="
echo "启动机器人语音识别系统"
echo "=========================================="

# 检查Python环境
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到Python3"
    exit 1
fi

# 检查pip
if ! command -v pip &> /dev/null; then
    echo "错误: 未找到pip"
    exit 1
fi

# 检查Node.js
if ! command -v node &> /dev/null; then
    echo "错误: 未找到Node.js"
    exit 1
fi

# 检查npm
if ! command -v npm &> /dev/null; then
    echo "错误: 未找到npm"
    exit 1
fi

echo "✓ 环境检查通过"

# 检查离线LLM服务
echo ""
echo "检查离线LLM服务..."
if curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "✓ 离线LLM服务运行中 (Ollama)"
    echo "  将优先使用离线LLM进行语义分析"
else
    echo "⚠ 离线LLM服务未启动"
    echo "  建议运行: ./setup_offline_llm.sh"
    echo "  或手动启动: ollama serve"
    echo "  系统将使用在线LLM或关键词匹配"
fi

# 安装Python依赖
echo ""
echo "安装Python依赖..."
pip install -r requirements.txt
pip install -r requirements_voice.txt

# 检查环境变量
if [ -z "$OPENAI_API_KEY" ]; then
    echo ""
    echo "警告: 未设置OPENAI_API_KEY环境变量"
    echo "语音识别将使用传统关键词匹配模式"
    echo ""
    echo "要启用LLM语义分析，请设置环境变量："
    echo "export OPENAI_API_KEY=your_api_key_here"
    echo ""
else
    echo "✓ 检测到OpenAI API密钥"
fi

# 检查Vosk模型
if [ ! -d "models/vosk-model-small-cn-0.22" ] && [ ! -d "models/vosk-model-cn-0.22" ]; then
    echo ""
    echo "警告: 未找到Vosk中文语音模型"
    echo "正在下载语音模型..."
    if [ -f "setup_vosk.sh" ]; then
        ./setup_vosk.sh
    else
        echo "错误: 未找到setup_vosk.sh脚本"
        exit 1
    fi
fi

echo "✓ Vosk模型检查完成"

# 安装前端依赖
echo ""
echo "安装前端依赖..."
cd ..
cd robot-delivery-system
npm install
cd ..

echo "✓ 前端依赖安装完成"

# 启动后端服务
cd secure_robot_api
conda activate webots
echo ""
echo "启动后端API服务..."
echo "服务地址: http://localhost:8000"
echo "API文档: http://localhost:8000/docs"
echo ""
echo "按Ctrl+C停止服务"
echo ""

# 启动FastAPI服务
uvicorn main:app --reload --host 0.0.0.0 --port 8000
