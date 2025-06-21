# 🤖 离线LLM语音识别系统部署指南

## 📋 概览

现在您的语音识别系统支持三层智能分析：
1. **离线LLM** (Ollama) - 最优先，完全离线
2. **在线LLM** (OpenAI) - 备选方案  
3. **关键词匹配** - 最后回退

## 🚀 快速部署

### 1. 修复API路径问题
✅ **已修复** - 前端API路径已更新为正确的 `/api/voice/*`

### 2. 部署离线LLM服务

```bash
# 运行自动安装脚本
./setup_offline_llm.sh
```

脚本会自动：
- 安装Ollama
- 启动服务
- 下载中文模型 (推荐qwen2:7b)
- 测试模型功能

### 3. 启动系统

```bash
# 启动语音识别系统
./start_voice_system.sh
```

### 4. 测试功能

```bash
# 测试离线LLM
python tests/test_offline_llm.py
```

## 🎯 优势对比

| 分析方法 | 响应速度 | 准确率 | 成本 | 稳定性 |
|---------|---------|--------|------|--------|
| 离线LLM | ⚡ 快 | 🎯 很高 | 💰 免费 | 🛡️ 极高 |
| 在线LLM | 🐌 慢 | 🎯 很高 | 💸 付费 | ⚠️ 依赖网络 |
| 关键词匹配 | ⚡ 极快 | 📊 中等 | 💰 免费 | 🛡️ 极高 |

## 📱 前端显示效果

系统会在识别结果中显示使用的分析方法：
- 🟢 **离线AI分析** - 使用本地LLM
- 🔵 **在线AI分析** - 使用OpenAI API  
- ⚪ **关键词匹配** - 传统匹配

## ⚙️ 系统工作流程

```
用户语音 → Vosk识别 → 文本分析
                          ↓
                    1. 离线LLM分析 ✅
                          ↓ (失败)
                    2. 在线LLM分析 ⚠️
                          ↓ (失败)  
                    3. 关键词匹配 🔄
                          ↓
                      返回地点结果
```

## 🔧 常用命令

```bash
# 查看Ollama状态
curl http://localhost:11434/api/tags

# 启动Ollama服务
ollama serve

# 查看已安装模型
ollama list

# 手动测试模型
ollama run qwen2:7b "你好"

# 停止Ollama服务
pkill ollama
```

## 🛠️ 故障排除

### Ollama服务无法启动
```bash
# 检查端口占用
lsof -i :11434

# 重启服务
pkill ollama && ollama serve
```

### 模型下载失败
```bash
# 手动下载推荐模型
ollama pull qwen2:7b

# 或下载轻量级模型
ollama pull qwen2:1.5b
```

### API路径错误
确保前端请求路径为：
- ✅ `/api/voice/recognize`
- ✅ `/api/voice/locations`  
- ✅ `/api/voice/test-recognition`

## 🎉 完成！

现在您的语音识别系统具备：
- ✅ 离线LLM智能分析
- ✅ 多层回退机制
- ✅ 完全离线运行
- ✅ 高稳定性和准确率

测试语音指令示例：
- "我要去经理室"
- "帮我送到财务处"  
- "请送到前台"
- "我想去大办公区"
