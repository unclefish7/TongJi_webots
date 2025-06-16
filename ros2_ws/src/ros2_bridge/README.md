# ROS 2 Navigation Result Bridge

这是一个ROS 2 Humble的Python桥接节点包，包含两个主要功能：

1. **导航结果桥接** - 监听导航结果并发送HTTP通知
2. **动态话题发布桥接** - 提供HTTP接口动态发布ROS 2话题消息

## 节点功能

### 1. nav_result_bridge
- **节点名称**: `nav_result_bridge`
- **订阅话题**: `/navigate_to_pose/result` (类型: `nav2_msgs/action/NavigateToPose_Result`)
- **功能**: 当导航成功时（`error_code == 0`），向 `http://localhost:8000/api/robot/arrived` 发送JSON消息

### 2. topic_bridge_node  
- **节点名称**: `topic_bridge_node`
- **HTTP服务器**: 监听端口 8080
- **功能**: 提供REST API动态发布ROS 2话题消息
- **支持的消息类型**: std_msgs、geometry_msgs、sensor_msgs等标准消息包

## 文件结构

```
ros2_bridge/
├── CMakeLists.txt              # CMake构建文件
├── package.xml                 # ROS 2包配置
├── requirements.txt            # Python依赖
├── start_bridge.sh             # 启动脚本
├── README.md                   # 本文件
├── launch/                     # Launch文件目录
│   ├── ros2_bridge.launch.py   # 默认launch文件（包含两个节点）
│   └── nav_bridge_custom.launch.py  # 自定义配置launch文件
└── ros2_bridge/                # Python包目录
    ├── __init__.py             # Python包初始化文件
    ├── nav_result_bridge.py    # 导航结果桥接节点
    └── publish_msg.py          # 动态话题发布节点
```

## 安装和使用

### 1. 安装Python依赖

```bash
cd /home/jerry/Documents/webots/ros2_ws/src/ros2_bridge
pip3 install -r requirements.txt

# 安装额外的Python包
pip3 install aiohttp
```

### 2. 安装ROS 2依赖

确保已安装以下ROS 2包：
```bash
sudo apt install ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav2-msgs
```

### 3. 编译ROS 2包

```bash
cd /home/jerry/Documents/webots/ros2_ws
colcon build --packages-select ros2_bridge
source install/setup.bash
```

### 4. 运行节点

方法一：使用launch文件启动所有节点
```bash
ros2 launch ros2_bridge ros2_bridge.launch.py
```

方法二：单独运行话题发布节点
```bash
ros2 run ros2_bridge publish_msg.py
```

方法三：单独运行导航桥接节点  
```bash
ros2 run ros2_bridge nav_result_bridge.py
```

## API 使用说明

### 动态话题发布API

**端点**: `POST http://localhost:8080/publish_topic`

**请求格式**:
```json
{
  "topic": "/topic_name",              // 要发布的话题名
  "type": "std_msgs/String",           // 消息类型
  "data": { "data": "hello world" }    // 消息内容
}
```

**响应格式**:
```json
{
  "success": true
}
```

### 使用示例

1. **发布字符串消息**:
```bash
curl -X POST http://localhost:8080/publish_topic \
  -H "Content-Type: application/json" \
  -d '{"topic":"/test_string","type":"std_msgs/String","data":{"data":"Hello ROS 2!"}}'
```

2. **发布位姿消息**:
```bash
curl -X POST http://localhost:8080/publish_topic \
  -H "Content-Type: application/json" \
  -d '{
    "topic": "/target_pose",
    "type": "geometry_msgs/PoseStamped",
    "data": {
      "header": {
        "frame_id": "map"
      },
      "pose": {
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      }
    }
  }'
```

3. **健康检查**:
```bash
curl http://localhost:8080/health
```

## Launch文件参数

### ros2_bridge.launch.py
- `api_url`: 导航API服务器URL (默认: http://localhost:8000/api/robot/arrived)
- `log_level`: 日志级别 (默认: info)
- `http_port`: HTTP服务器端口 (默认: 8080)

### 使用示例
```bash
# 使用自定义参数启动
ros2 launch ros2_bridge ros2_bridge.launch.py api_url:=http://192.168.1.100:8000/api/arrived log_level:=debug http_port:=8080
```

## 依赖

### ROS 2 包
- ROS 2 Humble
- rclpy
- std_msgs
- geometry_msgs  
- sensor_msgs
- nav2_msgs
- launch
- launch_ros

### Python 包
- aiohttp (HTTP服务器)
- requests (HTTP客户端)

## 注意事项

1. 确保ROS 2环境已正确配置
2. 确保所需的消息类型包已安装
3. HTTP服务器默认监听所有接口(0.0.0.0)，注意安全性
4. 节点支持动态加载消息类型，但需要相应的ROS 2包已安装
5. 错误会记录到日志但不会中断服务

## 故障排除

如果遇到问题，请检查：

1. ROS 2环境是否正确source
2. 所需的消息包是否已安装
3. aiohttp是否已安装
4. 端口8080是否被占用
5. 检查节点日志输出获取详细错误信息
