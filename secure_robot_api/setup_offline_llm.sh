#!/bin/bash

# Ollama离线LLM部署脚本

echo "=========================================="
echo "部署离线LLM服务 (Ollama)"
echo "=========================================="

# 检查系统架构
ARCH=$(uname -m)
OS=$(uname -s | tr '[:upper:]' '[:lower:]')

echo "检测到系统: $OS ($ARCH)"

# 安装Ollama
install_ollama() {
    echo ""
    echo "正在安装Ollama..."
    
    if command -v ollama &> /dev/null; then
        echo "✓ Ollama已安装"
        ollama --version
        return 0
    fi
    
    # 下载并安装Ollama
    curl -fsSL https://ollama.com/install.sh | sh
    
    if [ $? -eq 0 ]; then
        echo "✓ Ollama安装成功"
    else
        echo "✗ Ollama安装失败"
        exit 1
    fi
}

# 启动Ollama服务
start_ollama_service() {
    echo ""
    echo "启动Ollama服务..."
    
    # 检查服务是否已运行
    if pgrep -f "ollama serve" > /dev/null; then
        echo "✓ Ollama服务已在运行"
        return 0
    fi
    
    # 启动服务（后台运行）
    nohup ollama serve > /dev/null 2>&1 &
    
    # 等待服务启动
    echo "等待服务启动..."
    sleep 5
    
    # 检查服务状态
    if curl -s http://localhost:11434/api/tags > /dev/null; then
        echo "✓ Ollama服务启动成功"
    else
        echo "✗ Ollama服务启动失败"
        exit 1
    fi
}

# 下载推荐的模型
download_models() {
    echo ""
    echo "下载推荐的中文模型..."
    
    # 推荐的模型列表（按优先级排序）
    MODELS=(
        "qwen2:7b"      # 通义千问7B - 推荐
        "llama3.1:8b"   # Llama 3.1 8B - 备用
        "qwen2:1.5b"    # 通义千问1.5B - 轻量级
    )
    
    echo "可用的模型选项："
    for i in "${!MODELS[@]}"; do
        echo "$((i+1)). ${MODELS[$i]}"
    done
    
    echo ""
    echo "推荐选择 1 (qwen2:7b) - 中文效果最好"
    echo "如果硬件资源有限，可选择 3 (qwen2:1.5b)"
    echo ""
    
    read -p "请选择要下载的模型 (1-${#MODELS[@]}): " choice
    
    if [[ $choice =~ ^[1-${#MODELS[@]}]$ ]]; then
        selected_model=${MODELS[$((choice-1))]}
        echo ""
        echo "正在下载模型: $selected_model"
        echo "注意: 模型文件较大，下载可能需要几分钟到几十分钟..."
        
        ollama pull "$selected_model"
        
        if [ $? -eq 0 ]; then
            echo "✓ 模型 $selected_model 下载成功"
        else
            echo "✗ 模型下载失败"
            exit 1
        fi
    else
        echo "无效选择，下载默认模型 qwen2:7b"
        ollama pull qwen2:7b
    fi
}

# 测试模型
test_model() {
    echo ""
    echo "测试模型..."
    
    # 获取已安装的模型
    models=$(ollama list | grep -v "NAME" | awk '{print $1}' | head -1)
    
    if [ -z "$models" ]; then
        echo "✗ 未找到已安装的模型"
        return 1
    fi
    
    echo "使用模型: $models"
    echo "测试提示: 你好，请用中文回答"
    
    # 测试模型响应
    response=$(ollama run "$models" "你好，请用一句话介绍你自己" --timeout 30s)
    
    if [ $? -eq 0 ]; then
        echo "✓ 模型测试成功"
        echo "模型响应: $response"
    else
        echo "✗ 模型测试失败"
        return 1
    fi
}

# 创建系统服务（可选）
create_systemd_service() {
    echo ""
    read -p "是否创建系统服务以便开机自启动? (y/n): " create_service
    
    if [[ $create_service =~ ^[Yy]$ ]]; then
        echo "创建systemd服务..."
        
        sudo tee /etc/systemd/system/ollama.service > /dev/null <<EOF
[Unit]
Description=Ollama Server
After=network-online.target

[Service]
ExecStart=/usr/local/bin/ollama serve
User=ollama
Group=ollama
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
EOF
        
        # 创建ollama用户
        sudo useradd -r -s /bin/false -m -d /usr/share/ollama ollama 2>/dev/null || true
        
        # 启用服务
        sudo systemctl daemon-reload
        sudo systemctl enable ollama
        sudo systemctl start ollama
        
        echo "✓ 系统服务创建成功"
    fi
}

# 主函数
main() {
    install_ollama
    start_ollama_service
    download_models
    test_model
    create_systemd_service
    
    echo ""
    echo "=========================================="
    echo "离线LLM服务部署完成!"
    echo "=========================================="
    echo ""
    echo "服务信息:"
    echo "- API地址: http://localhost:11434"
    echo "- 已安装模型: $(ollama list | grep -v NAME | wc -l) 个"
    echo ""
    echo "常用命令:"
    echo "- 查看模型: ollama list"
    echo "- 运行模型: ollama run <model_name>"
    echo "- 停止服务: pkill ollama"
    echo "- 启动服务: ollama serve"
    echo ""
    echo "现在可以测试语音识别系统的离线LLM功能了!"
}

# 执行主函数
main
