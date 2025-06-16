# 机器人配送系统

一个完整的机器人配送系统，包含前端Web界面和后端API服务。

## 系统架构

### 后端 (FastAPI)
- **端口**: 8000
- **技术栈**: Python + FastAPI + SQLite
- **主要功能**:
  - 任务管理 (创建、执行、完成、取消)
  - 用户认证 (L1/L2/L3 多级认证)
  - 取件管理
  - 队列状态管理
  - ROS2桥接

### 前端 (Vue.js)
- **端口**: 5173
- **技术栈**: Vue 3 + TypeScript + Element Plus
- **主要功能**:
  - 寄件包裹
  - 取件包裹
  - 呼叫机器人
  - 任务队列监控
  - 柜门状态显示
  - 用户认证界面

## API路由结构

### 任务API (`/api/tasks`)
- `GET /api/tasks/ping` - 服务状态检查
- `POST /api/tasks/create` - 创建新任务
- `POST /api/tasks/complete` - 完成任务
- `POST /api/tasks/fail` - 标记任务失败
- `POST /api/tasks/cancel/{task_id}` - 取消任务
- `POST /api/tasks/start` - 启动下一个任务
- `GET /api/tasks/queue/status` - 获取队列状态
- `GET /api/tasks/ros2/status` - 检查ROS2状态

### 认证API (`/api/auth`)
- `POST /api/auth/verify` - 基础认证验证
- `POST /api/auth/verify_purpose` - 用途认证验证
- `GET /api/auth/cache_status` - 认证缓存状态

### 取件API (`/api/pickup`)
- `GET /api/pickup/tasks` - 查询可取件任务
- `POST /api/pickup/execute` - 执行取件操作

### 用户API (`/api/users`)
- `GET /api/users` - 获取所有用户
- `GET /api/users/{user_id}` - 获取指定用户信息

## 快速启动

### 1. 启动整个系统
```bash
cd /home/jerry/Documents/webots
./start_system.sh
```

系统启动后可访问：
- 前端界面: http://localhost:5173
- 后端API: http://localhost:8000
- API测试页面: http://localhost:5173/test
- API文档: http://localhost:8000/docs

### 2. 停止系统
```bash
cd /home/jerry/Documents/webots
./stop_system.sh
```

### 3. 测试API连接
```bash
cd /home/jerry/Documents/webots
./test_api_connection.sh
```

## 使用流程

### 寄件流程
1. 访问 http://localhost:5173/send
2. 进行身份认证 (根据安全等级)
3. 填写寄件信息 (接收人、目标位置、安全等级)
4. 系统分配柜门并创建任务
5. 在任务队列中启动任务

### 取件流程
1. 访问 http://localhost:5173/receive
2. 进行身份认证 (取件用途)
3. 查看可取件任务列表
4. 点击取件执行操作

### 呼叫机器人
1. 访问 http://localhost:5173/call
2. 选择机器人和位置
3. 设置优先级
4. 创建呼叫任务

### 任务队列管理
1. 在主界面右侧查看任务队列
2. 可以手动启动下一个任务
3. 查看任务状态和进度
4. 取消等待中的任务

## 开发调试

### 单独启动后端
```bash
cd /home/jerry/Documents/webots/secure_robot_api
source venv/bin/activate
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 单独启动前端
```bash
cd /home/jerry/Documents/webots/robot-delivery-system
npm run dev
```

### API测试
使用内置的API测试页面：http://localhost:5173/test

或使用命令行测试：
```bash
curl -X GET http://localhost:8000/api/tasks/ping
curl -X GET http://localhost:8000/api/tasks/queue/status
```

## 数据存储

系统使用JSON文件存储数据：
- `secure_robot_api/storage/users.json` - 用户信息
- `secure_robot_api/storage/locations.json` - 位置信息  
- `secure_robot_api/storage/lockers.json` - 柜门信息
- `secure_robot_api/storage/tasks.json` - 任务数据

## 配置文件

### 后端配置
- CORS设置：允许前端访问
- API前缀：统一的 `/api` 路径
- 端口：8000

### 前端配置  
- API基础URL：http://localhost:8000
- 开发服务器端口：5173
- 自动刷新间隔：5秒

## 故障排除

### 常见问题

1. **端口被占用**
   ```bash
   sudo lsof -i :8000  # 检查8000端口
   sudo lsof -i :5173  # 检查5173端口
   ```

2. **Python依赖问题**
   ```bash
   cd secure_robot_api
   pip install -r requirements.txt
   ```

3. **Node.js依赖问题**
   ```bash
   cd robot-delivery-system
   rm -rf node_modules package-lock.json
   npm install
   ```

4. **API连接失败**
   - 检查后端服务是否启动
   - 检查防火墙设置
   - 验证API路径是否正确

### 日志查看
- 后端日志：启动脚本会显示uvicorn日志
- 前端日志：浏览器开发者工具Console
- 网络请求：浏览器开发者工具Network面板

## 扩展功能

系统支持以下扩展：
- ROS2机器人集成
- 多机器人调度
- 实时位置追踪
- 更多认证方式
- 移动端应用

## 技术支持

如遇问题，请检查：
1. 系统要求 (Python 3.8+, Node.js 16+)
2. 端口可用性
3. 网络连接
4. 服务日志输出
