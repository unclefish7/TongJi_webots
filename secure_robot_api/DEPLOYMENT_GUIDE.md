🤖 语音识别系统 - 集成离线LLM语义分析

## 📋 更新内容

### 🔧 修复问题
- 修复前端API路径错误 (/api/recognize → /api/voice/recognize)
- 解决语音识别接口404错误

### 🚀 新增功能
- 集成离线LLM服务 (基于Ollama)
- 实现三层智能分析机制：
  1. 离线LLM (优先) - 完全本地，无需网络
  2. 在线LLM (备选) - OpenAI API
  3. 关键词匹配 (兜底) - 传统匹配
- 前端显示分析方法 (离线AI/在线AI/关键词匹配)
- 支持中文大语言模型 (qwen2:7b, llama3.1:8b)

### 🛠️ 技术改进
- 添加智能回退机制，确保高可用性
- 优化语音识别准确率
- 支持完全离线运行，避免网络依赖
- 零API成本，高稳定性

## 🚀 环境准备与运行指南

### 1. 系统要求
- Linux/macOS/Windows
- Python 3.8+
- Node.js 16+
- 4GB+ RAM (推荐8GB+)
- 10GB+ 磁盘空间 (用于LLM模型)

### 2. 克隆项目
```bash
git clone <your-repo-url>
cd webots/secure_robot_api
```

### 3. 安装Python依赖
```bash
# 安装基础依赖
pip install -r requirements.txt

# 安装语音识别依赖
pip install -r requirements_voice.txt
```

### 4. 设置语音识别模型
```bash
# 下载Vosk中文语音模型
./setup_vosk.sh
```

### 5. 部署离线LLM (推荐)
```bash
# 一键安装Ollama和中文模型
./setup_offline_llm.sh

# 选择推荐的qwen2:7b模型 (约4GB)
# 或选择轻量级qwen2:1.5b模型 (约1GB)
```

### 6. 配置环境变量 (可选)
```bash
# 如果需要使用在线LLM作为备选
cp .env.example .env
# 编辑.env文件，设置OPENAI_API_KEY (可选)
```

### 7. 安装前端依赖
```bash
cd ../robot-delivery-system
npm install
cd ../secure_robot_api
```

### 8. 启动系统
```bash
# 方式1: 使用一键启动脚本
./start_voice_system.sh

# 方式2: 手动启动后端
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 9. 启动前端 (另开终端)
```bash
cd ../robot-delivery-system
npm run dev
```

### 10. 测试功能
```bash
# 测试离线LLM
python tests/test_offline_llm.py

# 测试语音识别
python tests/test_voice_llm.py
```

## 📱 使用方法

1. 打开浏览器访问前端页面
2. 点击"开始录音"按钮
3. 说出目的地指令，如：
   - "我要去经理室"
   - "帮我送到财务处"
   - "请送到前台"
4. 系统会显示识别结果和使用的分析方法
5. 点击"选择此地点"确认

## 🎯 系统特点

### 智能分析优先级
1. **离线LLM分析** - 最快最稳定，完全本地运行
2. **在线LLM分析** - 备选方案，需要API密钥
3. **关键词匹配** - 兜底保障，始终可用

### 支持的地点
- 经理室、财务处、等候处、前台、休息室
- 小办公区、大办公区、小会议室、大会议室

### 前端显示效果
- 🟢 离线AI分析
- 🔵 在线AI分析  
- ⚪ 关键词匹配

## 🛠️ 故障排除

### 语音识别失败
```bash
# 检查Vosk模型
ls models/

# 重新下载模型
./setup_vosk.sh
```

### 离线LLM不工作
```bash
# 检查Ollama服务
curl http://localhost:11434/api/tags

# 重启Ollama
pkill ollama && ollama serve
```

### API路径错误
确保后端API正确启动在 http://localhost:8000

## 📊 性能对比

| 功能 | 离线LLM | 在线LLM | 关键词匹配 |
|------|---------|---------|-----------|
| 响应速度 | ⚡ 快 | 🐌 慢 | ⚡ 极快 |
| 准确率 | 🎯 很高 | 🎯 很高 | 📊 中等 |
| 成本 | 💰 免费 | 💸 付费 | 💰 免费 |
| 稳定性 | 🛡️ 极高 | ⚠️ 依赖网络 | 🛡️ 极高 |

## 📞 技术支持

如有问题，请检查：
1. 系统要求是否满足
2. 依赖是否正确安装
3. 服务是否正常启动
4. 端口是否被占用

建议按顺序执行安装步骤，遇到问题可查看各脚本的输出日志。
