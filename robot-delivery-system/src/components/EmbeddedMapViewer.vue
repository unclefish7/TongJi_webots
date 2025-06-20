<template>
  <div class="embedded-map-viewer">
    <!-- 地图控制栏 -->
    <div class="map-controls">
      <el-button-group size="small">
        <el-button @click="connectToROS" :disabled="isConnected" type="primary">
          {{ isConnected ? '已连接' : '连接ROS' }}
        </el-button>

        <el-button @click="toggleRobotTracking" :disabled="!isConnected" :type="isTrackingRobot ? 'warning' : 'info'">
          {{ isTrackingRobot ? '停止追踪' : '开始追踪' }}
        </el-button>
        <!-- <el-button @click="centerOnRobot" :disabled="!robotPosition" size="small">
          <el-icon><Aim /></el-icon>
        </el-button> -->
        <el-button @click="resetView" size="small" type="info">
          重置视图
        </el-button>
      </el-button-group>
    </div>

    <!-- 状态信息 -->
    <div class="map-status" v-if="mapData">
      <el-tag :type="isConnected ? 'success' : 'danger'" size="small">
        {{ isConnected ? 'ROS已连接' : 'ROS未连接' }}
      </el-tag>
      <span v-if="robotPosition" class="robot-info">
        机器人: ({{ robotPosition.x.toFixed(1) }}, {{ robotPosition.y.toFixed(1) }})
      </span>
    </div>

    <!-- 地图画布 -->
    <div class="map-canvas-container" ref="mapContainer">
      <canvas 
        ref="mapCanvas" 
        @wheel="handleZoom"
        @mousedown="handleMouseDown"
        @mousemove="handleMouseMove"
        @mouseup="handleMouseUp"
        @mouseleave="handleMouseUp"
      ></canvas>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, nextTick } from 'vue'
import { ElButton, ElButtonGroup, ElTag, ElMessage, ElIcon } from 'element-plus'
import { Aim } from '@element-plus/icons-vue'
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
const scale = ref(2) // 进一步减小初始缩放比例，从5改为2
const offsetX = ref(0)
const offsetY = ref(0)
const isDragging = ref(false)
const lastMouseX = ref(0)
const lastMouseY = ref(0)

// 追踪定时器
let trackingInterval: number | null = null
// 渲染优化
let animationFrameId: number | null = null
let needsRedraw = false
// 拖拽优化
let dragAnimationId: number | null = null
let isDragScheduled = false

// 连接到ROS
const connectToROS = async () => {
  try {
    await rosConnection.connect()
    isConnected.value = true
    
    // 设置回调函数
    rosConnection.onMapReceived((map: MapData) => {
      mapData.value = map
      nextTick(() => {
        scheduleRedraw()
      })
    })
    
    rosConnection.onRobotPositionUpdate((position: RobotPosition) => {
      robotPosition.value = position
      nextTick(() => {
        scheduleRedraw()
      })
    })
    
    // resetView() 
    ElMessage.success('成功连接到ROS')
  } catch (error) {
    ElMessage.error('连接ROS失败')
    console.error(error)
  }
}

// 请求地图数据
const requestMap = async () => {
  try {
    const map = await rosConnection.getMapOnce()
    mapData.value = map
    
    // 自动调整视图以适配新地图 - 视角中心设置在地图中心
    if (mapCanvas.value && mapContainer.value) {
      const container = mapContainer.value
      const scaleX = (container.clientWidth * 0.6) / map.width  // 适当增大显示比例
      const scaleY = (container.clientHeight * 0.6) / map.height 
      const autoScale = Math.min(scaleX, scaleY, 2.0) // 增加最大缩放限制
      
      scale.value = Math.max(0.1, autoScale) // 设置合理的最小缩放
      
      // 将视角中心设置在地图中心点
      const mapCenterX = map.width / 2
      const mapCenterY = map.height / 2
      
      offsetX.value = container.clientWidth / 2 - mapCenterX * scale.value
      offsetY.value = container.clientHeight / 2 - mapCenterY * scale.value
    }
    
    nextTick(() => {
      scheduleRedraw()
    })
    ElMessage.success('地图加载成功并已自动适配显示')
  } catch (error) {
    ElMessage.error('获取地图失败')
    console.error(error)
  }
}

// 切换机器人跟踪
const toggleRobotTracking = () => {
  isTrackingRobot.value = !isTrackingRobot.value
  
  if (isTrackingRobot.value) {
    // 开始ROS追踪，每秒自动获取机器人位置
    rosConnection.startTracking()
    ElMessage.info('开始追踪机器人位置 - 每秒更新')
  } else {
    // 停止追踪
    rosConnection.stopTracking()
    if (trackingInterval) {
      clearInterval(trackingInterval)
      trackingInterval = null
    }
    ElMessage.info('停止追踪机器人位置')
  }
}

// 居中机器人
const centerOnRobot = () => {
  if (!robotPosition.value || !mapCanvas.value || !mapData.value) return
  
  const canvas = mapCanvas.value
  const robot = robotPosition.value
  const map = mapData.value
  
  const mapX = (robot.x - map.origin.position.x) / map.resolution
  const mapY = (robot.y - map.origin.position.y) / map.resolution
  
  offsetX.value = canvas.width / 2 - mapX * scale.value
  offsetY.value = canvas.height / 2 - mapY * scale.value
  
  scheduleRedraw()
}

// 重置视图到合适的缩放和位置
const resetView = () => {
  // 重置偏移和缩放
  offsetX.value = 0
  offsetY.value = 0
  
  // 重新计算适合的视图
  nextTick(() => {
    resizeCanvas()
    ElMessage.info('视图已重置')
  })
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
  
  // 绘制ROS地图
  drawOccupancyGrid(ctx, mapData.value)
  if (robotPosition.value) {
    drawROSRobot(ctx, robotPosition.value, mapData.value)
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
  
  // 翻转Y轴
  const tempCanvas = document.createElement('canvas')
  tempCanvas.width = map.width
  tempCanvas.height = map.height
  const tempCtx = tempCanvas.getContext('2d')!
  
  tempCtx.putImageData(imageData, 0, 0)
  ctx.scale(1, -1)
  ctx.translate(0, -map.height)
  ctx.drawImage(tempCanvas, 0, 0)
}

// 绘制ROS机器人（专门针对ROS地图大幅缩小）
const drawROSRobot = (ctx: CanvasRenderingContext2D, robot: RobotPosition, map: MapData) => {
  const mapX = (robot.x - map.origin.position.x) / map.resolution
  const mapY = (robot.y - map.origin.position.y) / map.resolution
  
  const quaternion = robot.orientation
  const yaw = Math.atan2(
    2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
  )
  
  ctx.save()
  ctx.translate(mapX, mapY)
  ctx.rotate(-yaw)
  
  // 专门为ROS地图优化的机器人尺寸 - 大幅缩小但保证可见
  const baseSize = Math.max(0.5, 1.0 / scale.value) // 减小基础尺寸
  const robotRadius = baseSize / map.resolution/ 3.0
  const arrowLength = (baseSize * 1.2) / map.resolution // 减小箭头长度
  const arrowWidth = (baseSize * 0.3) / map.resolution // 减小箭头宽度
  const lineWidth = Math.max(0.5, (baseSize * 0.15) / map.resolution) // 减小线宽
  
  // 绘制机器人本体
  ctx.fillStyle = 'red'
  ctx.beginPath()
  ctx.arc(0, 0, robotRadius, 0, 2 * Math.PI)
  ctx.fill()
  
  // 绘制机器人边框
  ctx.strokeStyle = 'red'
  ctx.lineWidth = lineWidth
  ctx.beginPath()
  ctx.arc(0, 0, robotRadius, 0, 2 * Math.PI)
  ctx.stroke()
  
  // 绘制朝向箭头
  ctx.strokeStyle = 'red'
  ctx.lineWidth = lineWidth
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(arrowLength, 0)
  ctx.moveTo(arrowLength - arrowWidth, -arrowWidth)
  ctx.lineTo(arrowLength, 0)
  ctx.lineTo(arrowLength - arrowWidth, arrowWidth)
  ctx.stroke()
  
  ctx.restore()
}

// 鼠标事件处理（性能优化版本）
const handleZoom = (event: WheelEvent) => {
  event.preventDefault()
  
  const canvas = mapCanvas.value
  if (!canvas) return
  
  const rect = canvas.getBoundingClientRect()
  const mouseX = event.clientX - rect.left
  const mouseY = event.clientY - rect.top
  
  const scaleFactor = event.deltaY > 0 ? 0.9 : 1.1
  const newScale = Math.max(0.01, Math.min(50, scale.value * scaleFactor)) // 从0.05/100改为0.01/50
  
  const scaleChange = newScale / scale.value
  offsetX.value = mouseX - (mouseX - offsetX.value) * scaleChange
  offsetY.value = mouseY - (mouseY - offsetY.value) * scaleChange
  
  scale.value = newScale
  scheduleRedraw() // 使用优化的重绘
}

const handleMouseDown = (event: MouseEvent) => {
  isDragging.value = true
  lastMouseX.value = event.clientX
  lastMouseY.value = event.clientY
  
  // 添加鼠标样式
  if (mapCanvas.value) {
    mapCanvas.value.style.cursor = 'grabbing'
  }
}

const handleMouseMove = (event: MouseEvent) => {
  if (!isDragging.value) return
  
  const deltaX = event.clientX - lastMouseX.value
  const deltaY = event.clientY - lastMouseY.value
  
  offsetX.value += deltaX
  offsetY.value += deltaY
  
  lastMouseX.value = event.clientX
  lastMouseY.value = event.clientY
  
  // 优化拖拽性能：使用专门的拖拽重绘调度
  scheduleDragRedraw()
}

// 专门为拖拽优化的重绘调度函数
const scheduleDragRedraw = () => {
  if (isDragScheduled) return
  isDragScheduled = true
  
  if (dragAnimationId) {
    cancelAnimationFrame(dragAnimationId)
  }
  
  dragAnimationId = requestAnimationFrame(() => {
    if (isDragScheduled) {
      drawMap()
      isDragScheduled = false
    }
    dragAnimationId = null
  })
}

const handleMouseUp = () => {
  isDragging.value = false
  
  // 清理拖拽动画
  if (dragAnimationId) {
    cancelAnimationFrame(dragAnimationId)
    dragAnimationId = null
  }
  isDragScheduled = false
  
  // 恢复鼠标样式
  if (mapCanvas.value) {
    mapCanvas.value.style.cursor = 'grab'
  }
}

// 调整画布大小
const resizeCanvas = () => {
  if (!mapCanvas.value || !mapContainer.value) return
  
  const container = mapContainer.value
  mapCanvas.value.width = container.clientWidth
  mapCanvas.value.height = container.clientHeight
  
  // 初始化视图 - ROS地图自动适配，视角中心在地图中心
  if (offsetX.value === 0 && offsetY.value === 0 && mapData.value) {
    const mapWidth = mapData.value.width
    const mapHeight = mapData.value.height
    
    const scaleX = (container.clientWidth * 0.6) / mapWidth // 与requestMap保持一致
    const scaleY = (container.clientHeight * 0.6) / mapHeight
    const autoScale = Math.min(scaleX, scaleY, 2.0)
    
    scale.value = Math.max(0.1, autoScale)
    
    // 将视角中心设置在地图中心点
    const mapCenterX = mapWidth / 2
    const mapCenterY = mapHeight / 2
    
    offsetX.value = container.clientWidth / 2 - mapCenterX * scale.value
    offsetY.value = container.clientHeight / 2 - mapCenterY * scale.value
  }
  
  scheduleRedraw()
}

// 优化的绘制函数，避免频繁重绘
const scheduleRedraw = () => {
  if (needsRedraw) return
  needsRedraw = true
  
  if (animationFrameId) {
    cancelAnimationFrame(animationFrameId)
  }
  
  animationFrameId = requestAnimationFrame(() => {
    if (needsRedraw) {
      drawMap()
      needsRedraw = false
    }
    animationFrameId = null
  })
}

// 生命周期
onMounted(() => {
  nextTick(() => {
    resizeCanvas()
    // window.addEventListener('resize', resizeCanvas)
    
    // 自动连接ROS并开始实时追踪
    // setTimeout(() => {
    //   connectToROS().then(() => {
    //     if (isConnected.value) {
    //       requestMap()
    //       setTimeout(() => {
    //         toggleRobotTracking()
    //       }, 1000) // 等待地图加载后开始追踪
    //     }
    //   })
    // }, 500) // 延迟半秒确保组件完全初始化
  })
})

onUnmounted(() => {
  window.removeEventListener('resize', resizeCanvas)
  if (trackingInterval) {
    clearInterval(trackingInterval)
  }
  if (animationFrameId) {
    cancelAnimationFrame(animationFrameId)
  }
  if (dragAnimationId) {
    cancelAnimationFrame(dragAnimationId)
  }
  rosConnection.disconnect()
})
</script>

<style scoped>
.embedded-map-viewer {
  display: flex;
  flex-direction: column;
  height: 100%;
  background: white;
}

.map-controls {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 12px;
  border-bottom: 1px solid #e0e0e0;
  background: #fafafa;
}

.map-status {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 4px 12px;
  background: #f9f9f9;
  border-bottom: 1px solid #e0e0e0;
  font-size: 12px;
}

.robot-info {
  color: #666;
  font-size: 11px;
}

.map-canvas-container {
  flex: 1;
  position: relative;
  overflow: hidden;
  min-height: 300px;
}

.map-canvas-container canvas {
  display: block;
  cursor: grab;
  background: #f8f8f8;
  width: 100%;
  height: 100%;
}

.map-canvas-container canvas:active {
  cursor: grabbing;
}
</style>
