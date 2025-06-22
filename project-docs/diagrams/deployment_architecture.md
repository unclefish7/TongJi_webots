# 部署架构图

本文档展示了智能配送机器人系统的部署架构和环境配置。

## 1. 系统部署架构图

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              部署架构图                                               │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│ ┌─────────────────────────────────────────────────────────────────────────────────┐ │
│ │                            负载均衡层                                           │ │
│ │                        Load Balancer Layer                                     │ │
│ │                                                                                 │ │
│ │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │ │
│ │  │   Nginx     │    │   HAProxy   │    │     CDN     │    │   SSL终端   │      │ │
│ │  │反向代理服务  │    │负载均衡器    │    │内容分发网络  │    │  SSL Term   │      │ │
│ │  │             │    │             │    │             │    │             │      │ │
│ │  │ - 静态文件   │    │ - 健康检查   │    │ - 静态资源   │    │ - HTTPS     │      │ │
│ │  │ - 请求路由   │    │ - 故障转移   │    │ - 图片/视频  │    │ - 证书管理   │      │ │
│ │  │ - 缓存      │    │ - 会话保持   │    │ - 全球加速   │    │ - 安全策略   │      │ │
│ │  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘      │ │
│ └─────────────────────────────────────────────────────────────────────────────────┘ │
│                                       │                                             │
│                                       ▼                                             │
│ ┌─────────────────────────────────────────────────────────────────────────────────┐ │
│ │                            应用服务层                                           │ │
│ │                        Application Service Layer                               │ │
│ │                                                                                 │ │
│ │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │ │
│ │  │  Web前端    │    │  API服务    │    │  WebSocket  │    │  语音服务    │      │ │
│ │  │ Frontend    │    │ API Server  │    │   服务      │    │Voice Service│      │ │
│ │  │             │    │             │    │             │    │             │      │ │
│ │  │ - Vue.js    │    │ - FastAPI   │    │ - 实时通信   │    │ - Vosk ASR  │      │ │
│ │  │ - TypeScript│    │ - Python    │    │ - 状态推送   │    │ - TTS合成   │      │ │
│ │  │ - Vite构建  │    │ - 异步处理   │    │ - 连接管理   │    │ - 语音处理   │      │ │
│ │  │ - Nginx服务 │    │ - JWT认证   │    │ - 消息广播   │    │ - 指令识别   │      │ │
│ │  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘      │ │
│ │                                                                                 │ │
│ │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │ │
│ │  │  任务管理    │    │  路径规划    │    │  安全监控    │    │  数据分析    │      │ │
│ │  │TaskManager  │    │PathPlanner  │    │SecurityMonitor│  │DataAnalyzer │      │ │
│ │  │             │    │             │    │             │    │             │      │ │
│ │  │ - 调度算法   │    │ - A*算法    │    │ - 异常检测   │    │ - 统计分析   │      │ │
│ │  │ - 队列管理   │    │ - 地图服务   │    │ - 入侵检测   │    │ - 报表生成   │      │ │
│ │  │ - 状态跟踪   │    │ - 避障算法   │    │ - 审计日志   │    │ - 预测分析   │      │ │
│ │  │ - 优先级    │    │ - 优化算法   │    │ - 告警通知   │    │ - 可视化    │      │ │
│ │  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘      │ │
│ └─────────────────────────────────────────────────────────────────────────────────┘ │
│                                       │                                             │
│                                       ▼                                             │
│ ┌─────────────────────────────────────────────────────────────────────────────────┐ │
│ │                           ROS2通信层                                            │ │
│ │                        ROS2 Communication Layer                                │ │
│ │                                                                                 │ │
│ │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │ │
│ │  │  ROS2桥接   │    │  导航栈     │    │  SLAM系统   │    │  传感器融合  │      │ │
│ │  │ROS2 Bridge  │    │Nav2 Stack   │    │SLAM System  │    │SensorFusion │      │ │
│ │  │             │    │             │    │             │    │             │      │ │
│ │  │ - 消息转换   │    │ - 路径规划   │    │ - 地图构建   │    │ - 数据融合   │      │ │
│ │  │ - 话题代理   │    │ - 局部规划   │    │ - 定位算法   │    │ - 传感器校准 │      │ │
│ │  │ - 服务调用   │    │ - 控制器    │    │ - 回环检测   │    │ - 故障检测   │      │ │
│ │  │ - 参数管理   │    │ - 恢复行为   │    │ - 地图更新   │    │ - 数据滤波   │      │ │
│ │  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘      │ │
│ └─────────────────────────────────────────────────────────────────────────────────┘ │
│                                       │                                             │
│                                       ▼                                             │
│ ┌─────────────────────────────────────────────────────────────────────────────────┐ │
│ │                          仿真环境层                                             │ │
│ │                       Simulation Environment Layer                             │ │
│ │                                                                                 │ │
│ │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │ │
│ │  │ Webots仿真  │    │  机器人模型  │    │  环境模型    │    │  传感器模拟  │      │ │
│ │  │Webots Sim   │    │Robot Model  │    │World Model  │    │Sensor Sim   │      │ │
│ │  │             │    │             │    │             │    │             │      │ │
│ │  │ - 物理引擎   │    │ - 运动学    │    │ - 建筑布局   │    │ - LiDAR仿真 │      │ │
│ │  │ - 图形渲染   │    │ - 动力学    │    │ - 障碍物    │    │ - 摄像头仿真 │      │ │
│ │  │ - 场景管理   │    │ - 外观设计   │    │ - 材质属性   │    │ - IMU仿真   │      │ │
│ │  │ - 时间同步   │    │ - 传感器集成 │    │ - 光照效果   │    │ - 噪声模拟   │      │ │
│ │  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘      │ │
│ └─────────────────────────────────────────────────────────────────────────────────┘ │
│                                       │                                             │
│                                       ▼                                             │
│ ┌─────────────────────────────────────────────────────────────────────────────────┐ │
│ │                           数据存储层                                            │ │
│ │                        Data Storage Layer                                      │ │
│ │                                                                                 │ │
│ │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │ │
│ │  │ SQLite数据库│    │  Redis缓存  │    │  文件存储    │    │  日志系统    │      │ │
│ │  │SQLite DB    │    │Redis Cache  │    │File Storage │    │Log System   │      │ │
│ │  │             │    │             │    │             │    │             │      │ │
│ │  │ - 任务数据   │    │ - 会话缓存   │    │ - 配置文件   │    │ - 操作日志   │      │ │
│ │  │ - 用户数据   │    │ - 查询缓存   │    │ - 地图文件   │    │ - 错误日志   │      │ │
│ │  │ - 系统配置   │    │ - 临时数据   │    │ - 媒体文件   │    │ - 访问日志   │      │ │
│ │  │ - 历史记录   │    │ - 状态数据   │    │ - 备份文件   │    │ - 性能日志   │      │ │
│ │  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘      │ │
│ └─────────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                     │
│ ┌─────────────────────────────────────────────────────────────────────────────────┐ │
│ │                           监控告警层                                            │ │
│ │                        Monitoring & Alerting Layer                             │ │
│ │                                                                                 │ │
│ │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐      │ │
│ │  │ 性能监控    │    │  健康检查    │    │  日志聚合    │    │  告警通知    │      │ │
│ │  │Performance  │    │Health Check │    │Log Aggregate│    │Alert Notify │      │ │
│ │  │             │    │             │    │             │    │             │      │ │
│ │  │ - CPU/内存  │    │ - 服务状态   │    │ - 日志收集   │    │ - 邮件通知   │      │ │
│ │  │ - 网络I/O   │    │ - 接口检查   │    │ - 日志分析   │    │ - 短信通知   │      │ │
│ │  │ - 数据库性能 │    │ - 依赖检查   │    │ - 异常检测   │    │ - 钉钉通知   │      │ │
│ │  │ - 响应时间   │    │ - 资源检查   │    │ - 趋势分析   │    │ - 升级机制   │      │ │
│ │  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘      │ │
│ └─────────────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## 2. 容器化部署架构

### 2.1 Docker容器组织结构

```yaml
# docker-compose.yml
version: '3.8'

services:
  # 前端服务
  frontend:
    build: ./robot-delivery-system
    ports:
      - "3000:3000"
    environment:
      - NODE_ENV=production
      - VITE_API_BASE_URL=http://api:8000
    depends_on:
      - api
    networks:
      - app-network
    volumes:
      - ./robot-delivery-system/dist:/usr/share/nginx/html:ro
    restart: unless-stopped

  # API服务
  api:
    build: ./secure_robot_api
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=sqlite:///./storage/robot_delivery.db
      - REDIS_URL=redis://redis:6379
      - JWT_SECRET_KEY=${JWT_SECRET_KEY}
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    depends_on:
      - redis
      - db
    networks:
      - app-network
      - ros-network
    volumes:
      - ./secure_robot_api/storage:/app/storage
      - ./secure_robot_api/logs:/app/logs
    restart: unless-stopped

  # Redis缓存
  redis:
    image: redis:6-alpine
    ports:
      - "6379:6379"
    command: redis-server --appendonly yes
    networks:
      - app-network
    volumes:
      - redis-data:/data
    restart: unless-stopped

  # 数据库
  db:
    image: sqlite:latest
    networks:
      - app-network
    volumes:
      - db-data:/var/lib/sqlite
    restart: unless-stopped

  # ROS2服务
  ros2-bridge:
    build: ./ros2_ws
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    networks:
      - ros-network
    volumes:
      - ./ros2_ws/src:/ros2_ws/src
      - ./ros2_ws/install:/ros2_ws/install
    restart: unless-stopped

  # Webots仿真
  webots:
    build: ./tongji_webot_project
    ports:
      - "1234:1234"  # Webots端口
    environment:
      - DISPLAY=${DISPLAY}
      - WEBOTS_HOME=/usr/local/webots
    networks:
      - ros-network
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./tongji_webot_project/worlds:/usr/local/webots/projects/worlds
    restart: unless-stopped

  # 监控服务
  prometheus:
    image: prom/prometheus
    ports:
      - "9090:9090"
    networks:
      - monitoring
    volumes:
      - ./monitoring/prometheus.yml:/etc/prometheus/prometheus.yml
      - prometheus-data:/prometheus
    restart: unless-stopped

  # 可视化监控
  grafana:
    image: grafana/grafana
    ports:
      - "3001:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=${GRAFANA_PASSWORD}
    networks:
      - monitoring
    volumes:
      - grafana-data:/var/lib/grafana
      - ./monitoring/grafana:/etc/grafana/provisioning
    restart: unless-stopped

  # 负载均衡
  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    networks:
      - app-network
    volumes:
      - ./nginx/nginx.conf:/etc/nginx/nginx.conf
      - ./nginx/ssl:/etc/nginx/ssl
      - ./nginx/logs:/var/log/nginx
    depends_on:
      - frontend
      - api
    restart: unless-stopped

networks:
  app-network:
    driver: bridge
  ros-network:
    driver: bridge
  monitoring:
    driver: bridge

volumes:
  redis-data:
  db-data:
  prometheus-data:
  grafana-data:
```

### 2.2 Kubernetes部署配置

```yaml
# k8s-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: robot-delivery-api
  namespace: robot-system
spec:
  replicas: 3
  selector:
    matchLabels:
      app: robot-delivery-api
  template:
    metadata:
      labels:
        app: robot-delivery-api
    spec:
      containers:
      - name: api
        image: robot-delivery/api:latest
        ports:
        - containerPort: 8000
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: db-secret
              key: url
        - name: REDIS_URL
          value: "redis://redis-service:6379"
        resources:
          requests:
            memory: "512Mi"
            cpu: "250m"
          limits:
            memory: "1Gi"
            cpu: "500m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5

---
apiVersion: v1
kind: Service
metadata:
  name: api-service
  namespace: robot-system
spec:
  selector:
    app: robot-delivery-api
  ports:
  - port: 8000
    targetPort: 8000
  type: ClusterIP

---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: robot-delivery-ingress
  namespace: robot-system
  annotations:
    kubernetes.io/ingress.class: nginx
    cert-manager.io/cluster-issuer: letsencrypt-prod
spec:
  tls:
  - hosts:
    - api.robot-delivery.com
    secretName: api-tls
  rules:
  - host: api.robot-delivery.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: api-service
            port:
              number: 8000
```

## 3. 环境配置管理

### 3.1 多环境配置

```bash
# 开发环境 (.env.dev)
NODE_ENV=development
API_BASE_URL=http://localhost:8000
DATABASE_URL=sqlite:///./dev.db
REDIS_URL=redis://localhost:6379
LOG_LEVEL=DEBUG
WEBOTS_MODE=development

# 测试环境 (.env.test)
NODE_ENV=test
API_BASE_URL=http://test-api.internal:8000
DATABASE_URL=sqlite:///./test.db
REDIS_URL=redis://test-redis.internal:6379
LOG_LEVEL=INFO
WEBOTS_MODE=test

# 生产环境 (.env.prod)
NODE_ENV=production
API_BASE_URL=https://api.robot-delivery.com
DATABASE_URL=postgresql://user:pass@prod-db.internal:5432/robot_delivery
REDIS_URL=redis://prod-redis.internal:6379
LOG_LEVEL=WARNING
WEBOTS_MODE=production
```

### 3.2 配置管理脚本

```python
# config/environment.py
import os
from typing import Dict, Any
from dataclasses import dataclass

@dataclass
class DatabaseConfig:
    url: str
    pool_size: int = 10
    max_overflow: int = 20
    pool_timeout: int = 30

@dataclass
class RedisConfig:
    url: str
    max_connections: int = 100
    socket_timeout: int = 5

@dataclass
class ROS2Config:
    domain_id: int
    middleware: str = "rmw_fastrtps_cpp"
    discovery_timeout: int = 30

class Environment:
    def __init__(self, env_name: str = None):
        self.env_name = env_name or os.getenv('NODE_ENV', 'development')
        self.load_config()
    
    def load_config(self):
        """加载环境配置"""
        config_file = f".env.{self.env_name}"
        if os.path.exists(config_file):
            self.load_env_file(config_file)
        
        self.database = DatabaseConfig(
            url=os.getenv('DATABASE_URL', 'sqlite:///./robot_delivery.db')
        )
        
        self.redis = RedisConfig(
            url=os.getenv('REDIS_URL', 'redis://localhost:6379')
        )
        
        self.ros2 = ROS2Config(
            domain_id=int(os.getenv('ROS_DOMAIN_ID', '0'))
        )
    
    def load_env_file(self, file_path: str):
        """加载环境变量文件"""
        with open(file_path, 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    key, value = line.strip().split('=', 1)
                    os.environ[key] = value

# 全局配置实例
config = Environment()
```

## 4. 部署脚本和自动化

### 4.1 一键部署脚本

```bash
#!/bin/bash
# deploy.sh - 一键部署脚本

set -e

# 配置变量
ENVIRONMENT=${1:-production}
VERSION=${2:-latest}
DEPLOY_DIR="/opt/robot-delivery"
BACKUP_DIR="/opt/backups"

echo "开始部署智能配送机器人系统..."
echo "环境: $ENVIRONMENT"
echo "版本: $VERSION"

# 检查系统要求
check_requirements() {
    echo "检查系统要求..."
    
    # 检查Docker
    if ! command -v docker &> /dev/null; then
        echo "错误: Docker未安装"
        exit 1
    fi
    
    # 检查Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        echo "错误: Docker Compose未安装"
        exit 1
    fi
    
    # 检查磁盘空间
    AVAILABLE_SPACE=$(df / | tail -1 | awk '{print $4}')
    if [ $AVAILABLE_SPACE -lt 5000000 ]; then
        echo "警告: 磁盘空间不足5GB"
    fi
    
    echo "系统要求检查完成"
}

# 备份现有数据
backup_data() {
    echo "备份现有数据..."
    
    if [ -d "$DEPLOY_DIR" ]; then
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        BACKUP_PATH="$BACKUP_DIR/backup_$TIMESTAMP"
        
        mkdir -p "$BACKUP_PATH"
        
        # 备份数据库
        if [ -f "$DEPLOY_DIR/storage/robot_delivery.db" ]; then
            cp "$DEPLOY_DIR/storage/robot_delivery.db" "$BACKUP_PATH/"
        fi
        
        # 备份配置文件
        if [ -f "$DEPLOY_DIR/.env" ]; then
            cp "$DEPLOY_DIR/.env" "$BACKUP_PATH/"
        fi
        
        echo "数据备份到: $BACKUP_PATH"
    fi
}

# 下载和更新代码
update_code() {
    echo "更新代码..."
    
    if [ ! -d "$DEPLOY_DIR" ]; then
        mkdir -p "$DEPLOY_DIR"
        cd "$DEPLOY_DIR"
        git clone https://github.com/your-org/robot-delivery-system.git .
    else
        cd "$DEPLOY_DIR"
        git fetch origin
        git checkout $VERSION
        git pull origin $VERSION
    fi
    
    echo "代码更新完成"
}

# 构建和部署服务
deploy_services() {
    echo "构建和部署服务..."
    
    cd "$DEPLOY_DIR"
    
    # 复制环境配置
    cp ".env.$ENVIRONMENT" ".env"
    
    # 构建镜像
    docker-compose build
    
    # 停止旧服务
    docker-compose down
    
    # 启动新服务
    docker-compose up -d
    
    echo "服务部署完成"
}

# 健康检查
health_check() {
    echo "执行健康检查..."
    
    # 等待服务启动
    sleep 30
    
    # 检查API服务
    if curl -f http://localhost:8000/health > /dev/null 2>&1; then
        echo "✓ API服务正常"
    else
        echo "✗ API服务异常"
        exit 1
    fi
    
    # 检查前端服务
    if curl -f http://localhost:3000 > /dev/null 2>&1; then
        echo "✓ 前端服务正常"
    else
        echo "✗ 前端服务异常"
        exit 1
    fi
    
    # 检查数据库连接
    if docker-compose exec -T api python -c "from app.database import test_connection; test_connection()" > /dev/null 2>&1; then
        echo "✓ 数据库连接正常"
    else
        echo "✗ 数据库连接异常"
        exit 1
    fi
    
    echo "健康检查通过"
}

# 清理旧版本
cleanup() {
    echo "清理旧版本..."
    
    # 清理旧镜像
    docker image prune -f
    
    # 清理旧容器
    docker container prune -f
    
    # 清理旧备份(保留最近10个)
    if [ -d "$BACKUP_DIR" ]; then
        ls -t "$BACKUP_DIR" | tail -n +11 | xargs -I {} rm -rf "$BACKUP_DIR/{}"
    fi
    
    echo "清理完成"
}

# 主流程
main() {
    check_requirements
    backup_data
    update_code
    deploy_services
    health_check
    cleanup
    
    echo "🎉 部署完成!"
    echo "访问地址:"
    echo "  前端: http://localhost:3000"
    echo "  API: http://localhost:8000"
    echo "  监控: http://localhost:3001"
}

# 错误处理
trap 'echo "❌ 部署失败!"; exit 1' ERR

# 执行主流程
main
```

### 4.2 CI/CD流水线配置

```yaml
# .github/workflows/deploy.yml
name: Deploy Robot Delivery System

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.9'
    
    - name: Install dependencies
      run: |
        cd secure_robot_api
        pip install -r requirements.txt
    
    - name: Run tests
      run: |
        cd secure_robot_api
        pytest tests/ -v --cov=app --cov-report=xml
    
    - name: Upload coverage
      uses: codecov/codecov-action@v3
      with:
        file: ./secure_robot_api/coverage.xml

  build:
    needs: test
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    
    - name: Setup Node.js
      uses: actions/setup-node@v3
      with:
        node-version: '18'
    
    - name: Build frontend
      run: |
        cd robot-delivery-system
        npm install
        npm run build
    
    - name: Build Docker images
      run: |
        docker build -t robot-delivery/api:${{ github.sha }} ./secure_robot_api
        docker build -t robot-delivery/frontend:${{ github.sha }} ./robot-delivery-system
    
    - name: Push to registry
      run: |
        echo ${{ secrets.DOCKER_PASSWORD }} | docker login -u ${{ secrets.DOCKER_USERNAME }} --password-stdin
        docker push robot-delivery/api:${{ github.sha }}
        docker push robot-delivery/frontend:${{ github.sha }}

  deploy:
    needs: build
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/main'
    steps:
    - uses: actions/checkout@v3
    
    - name: Deploy to staging
      run: |
        ssh ${{ secrets.DEPLOY_USER }}@${{ secrets.DEPLOY_HOST }} "
          cd /opt/robot-delivery &&
          ./deploy.sh staging ${{ github.sha }}
        "
    
    - name: Run integration tests
      run: |
        # 集成测试脚本
        ./scripts/integration_test.sh staging
    
    - name: Deploy to production
      if: success()
      run: |
        ssh ${{ secrets.DEPLOY_USER }}@${{ secrets.DEPLOY_HOST }} "
          cd /opt/robot-delivery &&
          ./deploy.sh production ${{ github.sha }}
        "
```

## 5. 性能优化和扩展

### 5.1 水平扩展配置

```yaml
# horizontal-scaling.yml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: api-hpa
  namespace: robot-system
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: robot-delivery-api
  minReplicas: 2
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
  behavior:
    scaleUp:
      stabilizationWindowSeconds: 60
      policies:
      - type: Percent
        value: 100
        periodSeconds: 15
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
      - type: Percent
        value: 10
        periodSeconds: 60
```

### 5.2 数据库集群配置

```yaml
# postgres-cluster.yml
apiVersion: postgresql.cnpg.io/v1
kind: Cluster
metadata:
  name: postgres-cluster
  namespace: robot-system
spec:
  instances: 3
  primaryUpdateStrategy: unsupervised
  
  postgresql:
    parameters:
      max_connections: "200"
      shared_buffers: "256MB"
      effective_cache_size: "1GB"
      maintenance_work_mem: "64MB"
      checkpoint_completion_target: "0.9"
      wal_buffers: "16MB"
      default_statistics_target: "100"
      random_page_cost: "1.1"
      effective_io_concurrency: "200"
  
  bootstrap:
    initdb:
      database: robot_delivery
      owner: app_user
      secret:
        name: postgres-credentials
  
  storage:
    size: 100Gi
    storageClass: fast-ssd
  
  monitoring:
    enabled: true
```

## 6. 安全加固

### 6.1 网络安全配置

```yaml
# network-policy.yml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: robot-delivery-network-policy
  namespace: robot-system
spec:
  podSelector:
    matchLabels:
      app: robot-delivery-api
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - namespaceSelector:
        matchLabels:
          name: ingress-nginx
    - podSelector:
        matchLabels:
          app: robot-delivery-frontend
    ports:
    - protocol: TCP
      port: 8000
  egress:
  - to:
    - podSelector:
        matchLabels:
          app: postgres
    ports:
    - protocol: TCP
      port: 5432
  - to:
    - podSelector:
        matchLabels:
          app: redis
    ports:
    - protocol: TCP
      port: 6379
```

### 6.2 密钥管理

```yaml
# sealed-secrets.yml
apiVersion: bitnami.com/v1alpha1
kind: SealedSecret
metadata:
  name: robot-delivery-secrets
  namespace: robot-system
spec:
  encryptedData:
    database-url: AgBy3i4OJSWK+PiTySYZZA9rO5...
    jwt-secret: AgAKAoiQm+/LCCmqmcb09b5Sx...
    redis-password: AgAoKpEHpNvX8KmLc5fEqO8Lf...
  template:
    metadata:
      name: robot-delivery-secrets
      namespace: robot-system
    type: Opaque
```

这样，我们已经完成了完整的部署架构图文档，涵盖了从单机部署到容器化、Kubernetes集群部署的各种场景，包括监控、安全、扩展等各个方面的配置。
