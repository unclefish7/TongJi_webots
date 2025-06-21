# 语音识别系统使用指南

## 系统概述

本系统集成了语音识别和智能语义分析功能，支持通过语音指令选择机器人配送目的地。

### 技术架构
- **语音识别**: Vosk (本地离线识别)
- **语义分析**: OpenAI GPT (智能理解用户意图)
- **回退机制**: 关键词匹配 (当LLM不可用时)

## 快速开始

### 1. 环境准备

```bash
# 克隆项目
cd /home/jerry/Documents/webots/secure_robot_api

# 安装依赖
pip install -r requirements.txt
pip install -r requirements_voice.txt

# 下载语音模型
./setup_vosk.sh

# 设置环境变量
export OPENAI_API_KEY="your_api_key_here"
```

### 2. 启动系统

```bash
# 方式1: 使用启动脚本
./start_voice_system.sh

# 方式2: 手动启动
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 3. 测试功能

```bash
# 运行测试脚本
python tests/test_voice_llm.py
```

## 功能特性

### 支持的地点
- 经理室 (关键词: 经理室, 经理, 老板)
- 财务处 (关键词: 财务处, 财务, 会计)
- 等候处 (关键词: 等候处, 等候, 等待)
- 前台 (关键词: 前台, 接待)
- 休息室 (关键词: 休息室, 休息, 茶水间)
- 小办公区 (关键词: 小办公区, 小办公, 办公区)
- 大办公区 (关键词: 大办公区, 大办公, 主办公区)
- 小会议室 (关键词: 小会议室, 小会议, 会议室)
- 大会议室 (关键词: 大会议室, 大会议, 主会议室)

### 语音指令示例
- "我要去经理室"
- "帮我送到财务处"
- "请送到前台"
- "我想去大办公区"
- "送到小会议室吧"
- "麻烦送到休息室"

## API接口

### 语音识别接口
```http
POST /api/voice/recognize
Content-Type: multipart/form-data

{
  "audio": "音频文件"
}
```

### 响应格式
```json
{
  "success": true,
  "text": "识别到的文本",
  "location": "匹配的地点",
  "matched_keywords": ["匹配的关键词"],
  "analysis_method": "LLM", // 或 "Keywords"
  "available_locations": ["所有可用地点"]
}
```

### 获取地点列表
```http
GET /api/voice/locations
```

### 测试文本提取
```http
POST /api/voice/test-recognition?text=测试文本
```

## 配置说明

### 环境变量
- `OPENAI_API_KEY`: OpenAI API密钥 (必需，用于LLM分析)
- `OPENAI_BASE_URL`: API端点 (可选，默认为官方端点)

### 模型文件
- Vosk语音模型存放在 `models/` 目录
- 支持中文小模型和大模型

## 前端集成

### Vue组件使用
```vue
<template>
  <VoiceRecorder @location-selected="handleLocationSelected" />
</template>

<script setup>
import VoiceRecorder from '@/components/VoiceRecorder.vue'

const handleLocationSelected = (location: string) => {
  console.log('选择的地点:', location)
}
</script>
```

### npm依赖
前端项目的npm依赖已在`package.json`中配置：
- `lodash-es`: 工具库
- `@types/lodash-es`: TypeScript类型定义
- `element-plus`: UI组件库
- `axios`: HTTP客户端

安装前端依赖：
```bash
cd robot-delivery-system
npm install
```

## 故障排除

### 常见问题

1. **语音识别失败**
   - 检查Vosk模型是否正确下载
   - 确认音频格式支持 (WAV, MP3, M4A)
   - 检查音频文件大小和质量

2. **LLM分析不工作**
   - 验证OPENAI_API_KEY是否设置
   - 检查网络连接和API配额
   - 查看系统日志获取详细错误信息

3. **权限问题**
   - 确保脚本有执行权限: `chmod +x *.sh`
   - 检查文件读写权限

### 日志查看
```bash
# 查看系统日志
tail -f /var/log/voice_system.log

# 或在控制台查看实时日志
uvicorn main:app --reload --log-level debug
```

## 部署说明

### 生产环境
1. 使用HTTPS协议
2. 配置反向代理 (Nginx)
3. 设置环境变量管理
4. 配置日志轮转
5. 添加监控和告警

### Docker部署
```dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY requirements*.txt ./
RUN pip install -r requirements.txt -r requirements_voice.txt

COPY . .
EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

## 开发指南

### 添加新地点
1. 在`voice_service.py`中更新`location_keywords`字典
2. 重启服务
3. 测试新地点识别

### 自定义LLM提示词
修改`_analyze_with_llm`方法中的`prompt`变量来优化识别效果。

### 扩展音频格式支持
在`preprocess_audio`方法中添加新的音频格式处理逻辑。
