# 机器人实时地图显示功能

本功能为机器人配送系统提供了类似rviz2的实时地图显示和机器人位置跟踪能力。

## 功能特性

### 1. ROS实时地图模式
- 订阅 `/map` 话题显示占用栅格地图
- 订阅 `/tf` 话题跟踪机器人实时位置
- 支持地图缩放、平移操作
- 实时显示机器人朝向和移动轨迹

### 2. 模拟地图模式
- 使用语义地图配置显示建筑结构
- 模拟机器人移动和位置更新
- 支持点击地图位置控制机器人移动
- 预设语义位置快速导航

## 安装依赖

首先确保已安装必要的npm包：

```bash
cd robot-delivery-system
npm install
```

## ROS环境配置

### 1. 安装ROS Bridge
```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-server
```

### 2. 启动ROS Bridge服务
使用提供的启动脚本：
```bash
./start_ros_bridge.sh
```

或手动启动：
```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. 确保地图和机器人数据可用
- 地图服务器应发布 `/map` 话题
- 机器人应发布TF变换信息
- 可选：发布 `/robot_state` 话题作为备选位置信息源

## 使用方法

### 启动前端应用
```bash
cd robot-delivery-system
npm run dev
```

### 访问地图功能
1. 打开浏览器访问前端应用
2. 点击主页面的"实时地图"按钮
3. 选择使用模式：
   - **ROS实时地图模式**: 连接实际ROS系统
   - **模拟地图模式**: 使用预设地图进行演示

### 地图操作
- **缩放**: 鼠标滚轮
- **平移**: 鼠标拖拽
- **居中**: 点击"居中机器人"或"居中视图"按钮
- **机器人控制** (模拟模式): 点击地图任意位置

## 配置自定义地图

### 修改语义地图配置
编辑 `src/services/simulatedMap.ts` 中的 `defaultMapConfig`：

```typescript
export const defaultMapConfig: SimulatedMapConfig = {
  width: 800,        // 地图宽度(像素)
  height: 600,       // 地图高度(像素)
  resolution: 0.05,  // 分辨率(米/像素)
  origin: { x: -20, y: -15 }, // 地图原点世界坐标
  locations: [
    {
      id: 'room_101',
      name: '办公室101',
      x: 5,     // 世界坐标X
      y: 3,     // 世界坐标Y
      type: 'room',
      width: 4, // 房间宽度
      height: 3 // 房间高度
    },
    // ... 更多位置
  ],
  walls: [
    { start: { x: 0, y: 0 }, end: { x: 20, y: 0 } },
    // ... 更多墙体
  ]
}
```

### 修改ROS话题配置
编辑 `src/services/rosConnection.ts` 中的话题名称：

```typescript
// 地图话题
this.mapTopic = new ROSLIB.Topic({
  ros: this.ros,
  name: '/map',  // 修改为你的地图话题名
  messageType: 'nav_msgs/OccupancyGrid'
})

// TF话题
this.tfTopic = new ROSLIB.Topic({
  ros: this.ros,
  name: '/tf',   // 修改为你的TF话题名
  messageType: 'tf2_msgs/TFMessage'
})
```

## 故障排除

### 连接ROS失败
1. 确认ROS Bridge服务正在运行
2. 检查WebSocket地址是否正确 (默认: ws://localhost:9090)
3. 确认防火墙设置允许端口9090通信

### 地图不显示
1. 确认 `/map` 话题正在发布
2. 检查地图数据格式是否正确
3. 尝试使用 `ros2 topic echo /map` 验证话题数据

### 机器人位置不更新
1. 确认 `/tf` 话题正在发布机器人变换
2. 检查机器人frame_id配置 (默认查找: base_link, robot_base)
3. 可以配置备选的 `/robot_state` 话题

### 模拟模式问题
1. 确认浏览器控制台无JavaScript错误
2. 检查地图配置中的坐标范围是否合理
3. 尝试刷新页面重新加载

## 扩展开发

### 添加新的机器人状态显示
在 `RealTimeMapViewer.vue` 或 `SimulatedMapViewer.vue` 中修改 `drawRobot` 函数。

### 添加路径规划可视化
订阅路径话题并在地图上绘制规划路径。

### 添加传感器数据显示
订阅激光雷达或相机数据并在地图上叠加显示。

## 技术架构

- **前端框架**: Vue 3 + TypeScript
- **UI组件**: Element Plus
- **ROS通信**: roslib.js
- **地图渲染**: HTML5 Canvas
- **状态管理**: Vue Composition API

## 文件结构

```
src/
├── components/
│   ├── RealTimeMapViewer.vue    # ROS实时地图组件
│   └── SimulatedMapViewer.vue   # 模拟地图组件
├── services/
│   ├── rosConnection.ts         # ROS连接服务
│   └── simulatedMap.ts          # 模拟地图服务
├── views/
│   └── MapDashboard.vue         # 地图主页面
└── router/
    └── index.ts                 # 路由配置
```
