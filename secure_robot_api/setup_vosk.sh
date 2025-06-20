#!/bin/bash
# 下载并配置Vosk中文模型

echo "正在下载Vosk中文轻量模型..."

# 创建模型目录
mkdir -p /home/jerry/Documents/webots/secure_robot_api/models

cd /home/jerry/Documents/webots/secure_robot_api/models

# 下载中文小模型（约40MB，适合轻量部署）
if [ ! -d "vosk-model-small-cn-0.22" ]; then
    echo "下载中文小模型..."
    wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip
    unzip vosk-model-small-cn-0.22.zip
    rm vosk-model-small-cn-0.22.zip
    echo "中文小模型下载完成"
fi

# 如果需要更好的识别效果，可以下载大模型（约1.8GB）
# if [ ! -d "vosk-model-cn-0.22" ]; then
#     echo "下载中文大模型..."
#     wget https://alphacephei.com/vosk/models/vosk-model-cn-0.22.zip
#     unzip vosk-model-cn-0.22.zip
#     rm vosk-model-cn-0.22.zip
#     echo "中文大模型下载完成"
# fi

echo "模型下载配置完成！"
