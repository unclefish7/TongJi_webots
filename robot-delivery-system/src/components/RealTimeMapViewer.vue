<template>
  <div class="map-viewer">
    <div class="map-controls">
      <el-button @click="connectToROS" :disabled="isConnected" type="primary">
        {{ isConnected ? '已连接ROS' : '连接ROS' }}
      </el-button>
      <el-button @click="requestMap" :disabled="!isConnected">获取地图</el-button>
      <el-button @click="toggleRobotTracking" :disabled="!isConnected">
        {{ isTrackingRobot ? '停止跟踪' : '开始跟踪' }}
      </el-button>
      <el-button @click="centerOnRobot" :disabled="!robotPosition">居中机器人</el-button>
    </div>
    
    <div class="map-info" v-if="mapData">
      <span>地图尺寸: {{ mapData.width }} x {{ mapData.height }}</span>
      <span>分辨率: {{ mapData.resolution.toFixed(3) }} m/px</span>
      <span v-if="robotPosition">
        机器人位置: ({{ robotPosition.x.toFixed(2) }}, {{ robotPosition.y.toFixed(2) }})
      </span>
    </div>

    <div class="map-container" ref="mapContainer">
      <canvas 
        ref="mapCanvas" 
        @wheel="handleZoom"
        @mousedown="handleMouseDown"
        @mousemove="handleMouseMove"
        @mouseup="handleMouseUp"
        @mouseleave="handleMouseUp"
      ></canvas>
    </div>

    <div class="status-info">
      <el-tag :type="isConnected ? 'success' : 'danger'">
        {{ isConnected ? 'ROS已连接' : 'ROS未连接' }}
      </el-tag>
      <el-tag v-if="mapData" type="info">地图已加载</el-tag>
      <el-tag v-if="robotPosition" type="warning">机器人在线</el-tag>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, nextTick } from 'vue'
import { ElButton, ElTag, ElMessage } from 'element-plus'
import { rosConnection, type MapData, type RobotPosition } from '@/services/rosConnection'

// 响应式数据
const isConnected = ref(false)
const isTrackingRobot = ref(false)
const mapData = ref<MapData | null>(null)
const robotPosition = ref<RobotPosition | null>(null)

// DOM引用
const mapContainer = ref<HTMLDivElement>()
const mapCanvas = ref<HTMLCanvasElement>()

// 画布状态
const scale = ref(1)
const offsetX = ref(0)
const offsetY = ref(0)
const isDragging = ref(false)
const lastMouseX = ref(0)
const lastMouseY = ref(0)

// 连接到ROS
const connectToROS = async () => {
  try {
    await rosConnection.connect()
    isConnected.value = true
    
    // 设置回调函数
    rosConnection.onMapReceived((map: MapData) => {
      mapData.value = map
      nextTick(() => {
        drawMap()
      })
    })
    
    rosConnection.onRobotPositionUpdate((position: RobotPosition) => {
      robotPosition.value = position
      if (isTrackingRobot.value) {
        nextTick(() => {
          drawMap()
        })
      }
    })
    
    ElMessage.success('成功连接到ROS')
  } catch (error) {
    ElMessage.error('连接ROS失败: ' + error)
  }
}

// 请求地图数据
const requestMap = async () => {
  try {
    const map = await rosConnection.getMapOnce()
    mapData.value = map
    nextTick(() => {
      drawMap()
    })
    ElMessage.success('地图加载成功')
  } catch (error) {
    ElMessage.error('获取地图失败: ' + error)
  }
}

// 切换机器人跟踪
const toggleRobotTracking = () => {
  isTrackingRobot.value = !isTrackingRobot.value
  if (isTrackingRobot.value) {
    ElMessage.info('开始跟踪机器人位置')
  } else {
    ElMessage.info('停止跟踪机器人位置')
  }
}

// 居中机器人
const centerOnRobot = () => {
  if (!robotPosition.value || !mapData.value || !mapCanvas.value) return
  
  const canvas = mapCanvas.value
  const robot = robotPosition.value
  const map = mapData.value
  
  // 将世界坐标转换为地图坐标
  const mapX = (robot.x - map.origin.position.x) / map.resolution
  const mapY = (robot.y - map.origin.position.y) / map.resolution
  
  // 居中显示
  offsetX.value = canvas.width / 2 - mapX * scale.value
  offsetY.value = canvas.height / 2 - mapY * scale.value
  
  drawMap()
}

// 绘制地图
const drawMap = () => {
  if (!mapCanvas.value || !mapData.value) return
  
  const canvas = mapCanvas.value
  const ctx = canvas.getContext('2d')
  if (!ctx) return
  
  // 清空画布
  ctx.clearRect(0, 0, canvas.width, canvas.height)
  
  // 保存上下文
  ctx.save()
  
  // 应用变换
  ctx.translate(offsetX.value, offsetY.value)
  ctx.scale(scale.value, scale.value)
  
  // 绘制地图
  drawOccupancyGrid(ctx, mapData.value)
  
  // 绘制机器人
  if (robotPosition.value) {
    drawRobot(ctx, robotPosition.value, mapData.value)
  }
  
  // 恢复上下文
  ctx.restore()
}

// 绘制占用栅格地图
const drawOccupancyGrid = (ctx: CanvasRenderingContext2D, map: MapData) => {
  const imageData = ctx.createImageData(map.width, map.height)
  const data = imageData.data
  
  for (let i = 0; i < map.data.length; i++) {
    const value = map.data[i]
    let color = 128 // 未知区域为灰色
    
    if (value === 0) {
      color = 255 // 自由空间为白色
    } else if (value === 100) {
      color = 0 // 障碍物为黑色
    }
    
    const pixelIndex = i * 4
    data[pixelIndex] = color     // R
    data[pixelIndex + 1] = color // G
    data[pixelIndex + 2] = color // B
    data[pixelIndex + 3] = 255   // A
  }
  
  // 翻转Y轴（ROS地图Y轴向上，Canvas Y轴向下）
  const tempCanvas = document.createElement('canvas')
  tempCanvas.width = map.width
  tempCanvas.height = map.height
  const tempCtx = tempCanvas.getContext('2d')!
  
  tempCtx.putImageData(imageData, 0, 0)
  ctx.scale(1, -1)
  ctx.translate(0, -map.height)
  ctx.drawImage(tempCanvas, 0, 0)
}

// 绘制机器人
const drawRobot = (ctx: CanvasRenderingContext2D, robot: RobotPosition, map: MapData) => {
  // 将世界坐标转换为地图坐标
  const mapX = (robot.x - map.origin.position.x) / map.resolution
  const mapY = (robot.y - map.origin.position.y) / map.resolution
  
  // 计算机器人朝向角度
  const quaternion = robot.orientation
  const yaw = Math.atan2(
    2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
  )
  
  ctx.save()
  ctx.translate(mapX, mapY)
  ctx.rotate(-yaw) // 负号因为Y轴翻转
  
  // 绘制机器人本体（圆形）
  ctx.fillStyle = 'red'
  ctx.beginPath()
  ctx.arc(0, 0, 5 / map.resolution, 0, 2 * Math.PI)
  ctx.fill()
  
  // 绘制机器人朝向箭头
  ctx.strokeStyle = 'red'
  ctx.lineWidth = 2 / map.resolution
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(8 / map.resolution, 0)
  ctx.moveTo(6 / map.resolution, -2 / map.resolution)
  ctx.lineTo(8 / map.resolution, 0)
  ctx.lineTo(6 / map.resolution, 2 / map.resolution)
  ctx.stroke()
  
  ctx.restore()
}

// 鼠标滚轮缩放
const handleZoom = (event: WheelEvent) => {
  event.preventDefault()
  
  const canvas = mapCanvas.value
  if (!canvas) return
  
  const rect = canvas.getBoundingClientRect()
  const mouseX = event.clientX - rect.left
  const mouseY = event.clientY - rect.top
  
  const scaleFactor = event.deltaY > 0 ? 0.9 : 1.1
  const newScale = Math.max(0.1, Math.min(10, scale.value * scaleFactor))
  
  // 缩放中心为鼠标位置
  const scaleChange = newScale / scale.value
  offsetX.value = mouseX - (mouseX - offsetX.value) * scaleChange
  offsetY.value = mouseY - (mouseY - offsetY.value) * scaleChange
  
  scale.value = newScale
  drawMap()
}

// 鼠标拖拽
const handleMouseDown = (event: MouseEvent) => {
  isDragging.value = true
  lastMouseX.value = event.clientX
  lastMouseY.value = event.clientY
}

const handleMouseMove = (event: MouseEvent) => {
  if (!isDragging.value) return
  
  const deltaX = event.clientX - lastMouseX.value
  const deltaY = event.clientY - lastMouseY.value
  
  offsetX.value += deltaX
  offsetY.value += deltaY
  
  lastMouseX.value = event.clientX
  lastMouseY.value = event.clientY
  
  drawMap()
}

const handleMouseUp = () => {
  isDragging.value = false
}

// 调整画布大小
const resizeCanvas = () => {
  if (!mapCanvas.value || !mapContainer.value) return
  
  const container = mapContainer.value
  mapCanvas.value.width = container.clientWidth
  mapCanvas.value.height = container.clientHeight
  
  drawMap()
}

// 生命周期
onMounted(() => {
  nextTick(() => {
    resizeCanvas()
    window.addEventListener('resize', resizeCanvas)
  })
})

onUnmounted(() => {
  window.removeEventListener('resize', resizeCanvas)
  rosConnection.disconnect()
})
</script>

<style scoped>
.map-viewer {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background: #f5f5f5;
}

.map-controls {
  padding: 16px;
  background: white;
  border-bottom: 1px solid #e0e0e0;
  display: flex;
  gap: 12px;
  align-items: center;
}

.map-info {
  padding: 8px 16px;
  background: #f0f0f0;
  border-bottom: 1px solid #e0e0e0;
  display: flex;
  gap: 20px;
  font-size: 14px;
  color: #666;
}

.map-container {
  flex: 1;
  position: relative;
  overflow: hidden;
}

.map-container canvas {
  display: block;
  cursor: grab;
  background: #f8f8f8;
}

.map-container canvas:active {
  cursor: grabbing;
}

.status-info {
  padding: 12px 16px;
  background: white;
  border-top: 1px solid #e0e0e0;
  display: flex;
  gap: 12px;
  align-items: center;
}
</style>
