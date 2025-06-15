<template>
  <div class="simulated-map-viewer">
    <div class="map-controls">
      <el-button @click="resetRobotPosition" type="primary">重置机器人位置</el-button>
      <el-button @click="moveToRandomLocation" type="success">随机移动</el-button>
      <el-input-number 
        v-model="targetX" 
        placeholder="目标X坐标" 
        :precision="1"
        :step="0.5"
        size="small"
        style="width: 120px;"
      />
      <el-input-number 
        v-model="targetY" 
        placeholder="目标Y坐标" 
        :precision="1"
        :step="0.5"
        size="small"
        style="width: 120px;"
      />
      <el-button @click="moveToTarget" :disabled="!targetX || !targetY">移动到目标</el-button>
      <el-button @click="centerView">居中视图</el-button>
    </div>
    
    <div class="map-info">
      <span>地图尺寸: {{ mapConfig.width }} x {{ mapConfig.height }}</span>
      <span>分辨率: {{ mapConfig.resolution }} m/px</span>
      <span>机器人位置: ({{ robotPosition.x.toFixed(2) }}, {{ robotPosition.y.toFixed(2) }})</span>
      <span>机器人朝向: {{ (robotPosition.theta * 180 / Math.PI).toFixed(1) }}°</span>
    </div>

    <div class="map-container" ref="mapContainer">
      <canvas 
        ref="mapCanvas" 
        @wheel="handleZoom"
        @mousedown="handleMouseDown"
        @mousemove="handleMouseMove"
        @mouseup="handleMouseUp"
        @mouseleave="handleMouseUp"
        @click="handleCanvasClick"
      ></canvas>
    </div>

    <div class="location-list">
      <h4>语义位置</h4>
      <div class="locations">
        <el-tag 
          v-for="location in mapConfig.locations" 
          :key="location.id"
          :type="getLocationTagType(location.type)"
          @click="moveToLocation(location)"
          class="location-tag"
        >
          {{ location.name }}
        </el-tag>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, nextTick } from 'vue'
import { ElButton, ElTag, ElInputNumber, ElMessage } from 'element-plus'
import { 
  defaultMapConfig, 
  simulatedRobot, 
  type SimulatedMapConfig, 
  type SemanticLocation 
} from '@/services/simulatedMap'

// 响应式数据
const mapConfig = ref<SimulatedMapConfig>(defaultMapConfig)
const robotPosition = ref({ x: 8, y: 8, theta: 0 })
const targetX = ref<number>()
const targetY = ref<number>()

// DOM引用
const mapContainer = ref<HTMLDivElement>()
const mapCanvas = ref<HTMLCanvasElement>()

// 画布状态
const scale = ref(20) // 初始缩放比例
const offsetX = ref(0)
const offsetY = ref(0)
const isDragging = ref(false)
const lastMouseX = ref(0)
const lastMouseY = ref(0)

// 重置机器人位置
const resetRobotPosition = () => {
  simulatedRobot.setPosition(8, 8, 0)
  ElMessage.success('机器人位置已重置')
}

// 移动到随机位置
const moveToRandomLocation = () => {
  const x = Math.random() * 18 + 1
  const y = Math.random() * 13 + 1
  simulatedRobot.setTargetPosition(x, y)
  ElMessage.info(`机器人正在移动到 (${x.toFixed(1)}, ${y.toFixed(1)})`)
}

// 移动到指定目标
const moveToTarget = () => {
  if (targetX.value !== undefined && targetY.value !== undefined) {
    simulatedRobot.setTargetPosition(targetX.value, targetY.value)
    ElMessage.info(`机器人正在移动到 (${targetX.value}, ${targetY.value})`)
  }
}

// 移动到语义位置
const moveToLocation = (location: SemanticLocation) => {
  simulatedRobot.setTargetPosition(location.x, location.y)
  ElMessage.info(`机器人正在移动到 ${location.name}`)
}

// 居中视图
const centerView = () => {
  if (!mapCanvas.value) return
  
  const canvas = mapCanvas.value
  const robot = robotPosition.value
  
  offsetX.value = canvas.width / 2 - robot.x * scale.value
  offsetY.value = canvas.height / 2 - robot.y * scale.value
  
  drawMap()
}

// 获取位置标签类型
const getLocationTagType = (type: string): 'primary' | 'success' | 'info' | 'warning' | 'danger' => {
  switch (type) {
    case 'room': return 'primary'
    case 'corridor': return 'info'
    case 'door': return 'warning'
    case 'elevator': return 'success'
    case 'stairs': return 'danger'
    default: return 'info'
  }
}

// 绘制地图
const drawMap = () => {
  if (!mapCanvas.value) return
  
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
  
  // 绘制网格
  drawGrid(ctx)
  
  // 绘制墙体
  drawWalls(ctx)
  
  // 绘制语义位置
  drawLocations(ctx)
  
  // 绘制机器人
  drawRobot(ctx)
  
  // 恢复上下文
  ctx.restore()
}

// 绘制网格
const drawGrid = (ctx: CanvasRenderingContext2D) => {
  ctx.strokeStyle = '#f0f0f0'
  ctx.lineWidth = 0.02
  
  for (let x = 0; x <= mapConfig.value.width * mapConfig.value.resolution; x += 1) {
    ctx.beginPath()
    ctx.moveTo(x, 0)
    ctx.lineTo(x, mapConfig.value.height * mapConfig.value.resolution)
    ctx.stroke()
  }
  
  for (let y = 0; y <= mapConfig.value.height * mapConfig.value.resolution; y += 1) {
    ctx.beginPath()
    ctx.moveTo(0, y)
    ctx.lineTo(mapConfig.value.width * mapConfig.value.resolution, y)
    ctx.stroke()
  }
}

// 绘制墙体
const drawWalls = (ctx: CanvasRenderingContext2D) => {
  ctx.strokeStyle = '#333'
  ctx.lineWidth = 0.1
  
  mapConfig.value.walls.forEach(wall => {
    ctx.beginPath()
    ctx.moveTo(wall.start.x, wall.start.y)
    ctx.lineTo(wall.end.x, wall.end.y)
    ctx.stroke()
  })
}

// 绘制语义位置
const drawLocations = (ctx: CanvasRenderingContext2D) => {
  mapConfig.value.locations.forEach(location => {
    const colors = {
      room: '#e3f2fd',
      corridor: '#f3e5f5',
      door: '#fff3e0',
      elevator: '#e8f5e8',
      stairs: '#ffebee',
      landmark: '#fafafa'
    }
    
    const borderColors = {
      room: '#2196f3',
      corridor: '#9c27b0',
      door: '#ff9800',
      elevator: '#4caf50',
      stairs: '#f44336',
      landmark: '#666'
    }
    
    if (location.width && location.height) {
      // 绘制矩形区域
      ctx.fillStyle = colors[location.type] || colors.landmark
      ctx.fillRect(
        location.x - location.width / 2,
        location.y - location.height / 2,
        location.width,
        location.height
      )
      
      ctx.strokeStyle = borderColors[location.type] || borderColors.landmark
      ctx.lineWidth = 0.05
      ctx.strokeRect(
        location.x - location.width / 2,
        location.y - location.height / 2,
        location.width,
        location.height
      )
    } else {
      // 绘制点位置
      ctx.fillStyle = borderColors[location.type] || borderColors.landmark
      ctx.beginPath()
      ctx.arc(location.x, location.y, 0.2, 0, 2 * Math.PI)
      ctx.fill()
    }
    
    // 绘制标签
    ctx.fillStyle = '#333'
    ctx.font = '0.3px Arial'
    ctx.textAlign = 'center'
    ctx.fillText(location.name, location.x, location.y + (location.height ? location.height / 2 + 0.5 : 0.5))
  })
}

// 绘制机器人
const drawRobot = (ctx: CanvasRenderingContext2D) => {
  const robot = robotPosition.value
  
  ctx.save()
  ctx.translate(robot.x, robot.y)
  ctx.rotate(robot.theta)
  
  // 绘制机器人本体（圆形）
  ctx.fillStyle = '#ff4444'
  ctx.beginPath()
  ctx.arc(0, 0, 0.3, 0, 2 * Math.PI)
  ctx.fill()
  
  // 绘制机器人边框
  ctx.strokeStyle = '#cc0000'
  ctx.lineWidth = 0.05
  ctx.stroke()
  
  // 绘制朝向箭头
  ctx.strokeStyle = '#cc0000'
  ctx.lineWidth = 0.08
  ctx.beginPath()
  ctx.moveTo(0, 0)
  ctx.lineTo(0.4, 0)
  ctx.moveTo(0.3, -0.1)
  ctx.lineTo(0.4, 0)
  ctx.lineTo(0.3, 0.1)
  ctx.stroke()
  
  ctx.restore()
  
  // 绘制机器人路径轨迹（可选）
  // TODO: 可以添加路径历史记录
}

// 画布点击事件
const handleCanvasClick = (event: MouseEvent) => {
  if (!mapCanvas.value) return
  
  const canvas = mapCanvas.value
  const rect = canvas.getBoundingClientRect()
  const x = (event.clientX - rect.left - offsetX.value) / scale.value
  const y = (event.clientY - rect.top - offsetY.value) / scale.value
  
  // 检查点击位置是否在地图范围内
  if (x >= 0 && x <= mapConfig.value.width * mapConfig.value.resolution && 
      y >= 0 && y <= mapConfig.value.height * mapConfig.value.resolution) {
    simulatedRobot.setTargetPosition(x, y)
    ElMessage.info(`机器人正在移动到 (${x.toFixed(2)}, ${y.toFixed(2)})`)
  }
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
  const newScale = Math.max(5, Math.min(100, scale.value * scaleFactor))
  
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
  
  // 初始化视图居中
  if (scale.value && offsetX.value === 0 && offsetY.value === 0) {
    offsetX.value = container.clientWidth / 2 - (mapConfig.value.width * mapConfig.value.resolution * scale.value) / 2
    offsetY.value = container.clientHeight / 2 - (mapConfig.value.height * mapConfig.value.resolution * scale.value) / 2
  }
  
  drawMap()
}

// 生命周期
onMounted(() => {
  // 订阅机器人位置更新
  simulatedRobot.onPositionUpdate((position) => {
    robotPosition.value = position
    nextTick(() => {
      drawMap()
    })
  })
  
  nextTick(() => {
    resizeCanvas()
    window.addEventListener('resize', resizeCanvas)
  })
})

onUnmounted(() => {
  window.removeEventListener('resize', resizeCanvas)
  simulatedRobot.stop()
})
</script>

<style scoped>
.simulated-map-viewer {
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
  flex-wrap: wrap;
}

.map-info {
  padding: 8px 16px;
  background: #f0f0f0;
  border-bottom: 1px solid #e0e0e0;
  display: flex;
  gap: 20px;
  font-size: 14px;
  color: #666;
  flex-wrap: wrap;
}

.map-container {
  flex: 1;
  position: relative;
  overflow: hidden;
}

.map-container canvas {
  display: block;
  cursor: grab;
  background: white;
}

.map-container canvas:active {
  cursor: grabbing;
}

.location-list {
  padding: 12px 16px;
  background: white;
  border-top: 1px solid #e0e0e0;
  max-height: 120px;
  overflow-y: auto;
}

.location-list h4 {
  margin: 0 0 8px 0;
  font-size: 14px;
  color: #666;
}

.locations {
  display: flex;
  gap: 8px;
  flex-wrap: wrap;
}

.location-tag {
  cursor: pointer;
  transition: transform 0.2s;
}

.location-tag:hover {
  transform: scale(1.05);
}
</style>
