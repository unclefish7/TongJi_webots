<template>
    <div class="embedded-map-viewer">
        <!-- 地图控制栏 -->
        <div class="map-controls">
            <el-button-group size="small">
                <el-button @click="connectToROS" :disabled="isConnected" :type="isConnected ? 'success' : 'primary'">
                    {{ isConnected ? '已连接' : '连接ROS' }}
                </el-button>
                <el-button @click="toggleRobotTracking" :disabled="!isConnected"
                    :type="isTrackingRobot ? 'warning' : 'primary'">
                    {{ isTrackingRobot ? '停止追踪' : '开始追踪' }}
                </el-button>
                <el-button @click="onClickCenterOnRobot" :disabled="!robotPosition" size="small">
                    <el-icon>
                        <Aim />
                    </el-icon>
                    定位机器人
                </el-button>
                <el-button @click="resetView" size="small" type="danger">
                    重置视图
                </el-button>
            </el-button-group>
        </div>

        <!-- 状态信息 -->
        <div class="map-status" v-if="mapData">
            <span v-if="robotPosition" class="robot-info">
                机器人: ({{ robotPosition.x.toFixed(2) }}, {{ robotPosition.y.toFixed(2) }})
                角度: {{ robotYawDegrees?.toFixed(1) }}°
            </span>
            <span class="map-info">
                缩放: {{ scale.toFixed(2) }}x | 旋转: {{ rotation.toFixed(0) }}°
            </span>
        </div>

        <!-- 地图容器 - 双层画布架构 -->
        <div class="map-container" ref="mapContainer">
            <!-- 地图层 - 静态，使用Canvas绘制 -->
            <canvas ref="mapCanvas" class="map-layer"></canvas>

            <!-- 机器人与交互层 - 动态重绘 -->
            <canvas ref="robotCanvas" class="robot-layer" @wheel.prevent="handleZoom" @mousedown="handleMouseDown"
                @mousemove="handleMouseMove" @mouseup="handleMouseUp" @mouseleave="handleMouseUp"></canvas>

            <!-- 缩放控件 -->
            <div class="zoom-controls">
                <div class="zoom-slider-container">
                    <button class="zoom-btn" @click="zoomOut" title="缩小">
                        <svg viewBox="0 0 24 24" width="16" height="16">
                            <path fill="currentColor" d="M19,13H5V11H19V13Z" />
                        </svg>
                    </button>
                    <div class="zoom-slider-wrapper">
                        <input type="range" class="zoom-slider" :min="minScale" :max="maxScale" :step="0.05"
                            v-model.number="scale" @input="onZoomSliderChange" />
                        <div class="zoom-value">{{ scale.toFixed(1) }}x</div>
                    </div>
                    <button class="zoom-btn" @click="zoomIn" title="放大">
                        <svg viewBox="0 0 24 24" width="16" height="16">
                            <path fill="currentColor" d="M19,13H13V19H11V13H5V11H11V5H13V11H19V13Z" />
                        </svg>
                    </button>
                </div>
            </div>
        </div>
    </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, nextTick, computed } from 'vue'
import { ElButton, ElButtonGroup, ElMessage, ElIcon } from 'element-plus'
import { Aim } from '@element-plus/icons-vue'
import { rosConnection, type MapData, type RobotPosition } from '@/services/rosConnection'
import { throttle } from 'lodash-es'

// --- 机器人位置微调偏移量（以米为单位，确保缩放一致性） ---
const ROBOT_POSITION_OFFSET = {
    x: 0,  // X轴偏移：负数向左，正数向右（米）
    y: 0     // Y轴偏移：正数向北，负数向南（米）
}

// --- 地图 X 方向拉伸倍数 ---
const MAP_X_STRETCH = 1.1

// --- 响应式状态 ---
const isConnected = ref(false)
const isTrackingRobot = ref(false)
const mapData = ref<MapData | null>(null)
const robotPosition = ref<RobotPosition | null>(null)

// --- DOM 引用 ---
const mapContainer = ref<HTMLDivElement>()
const mapCanvas = ref<HTMLCanvasElement>()
const robotCanvas = ref<HTMLCanvasElement>()

// --- 视图变换状态 ---
const scale = ref(1.0)
const minScale = ref(0.1)
const maxScale = ref(10.0)
const offsetX = ref(0)
const offsetY = ref(0)
const rotation = ref(0) // 角度制

// --- 交互状态 ---
const isDragging = ref(false)
const lastMouseX = ref(0)
const lastMouseY = ref(0)

// --- 性能优化 ---
let mapBitmap: ImageBitmap | null = null
let pendingFrame: number | null = null

// --- 计算属性 ---
const robotYawRadians = computed(() => {
    if (!robotPosition.value) return 0
    const q = robotPosition.value.orientation
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
})

const robotYawDegrees = computed(() => {
    return (robotYawRadians.value * 180 / Math.PI + 360) % 360
})

// --- ROS 连接与数据处理 ---
const connectToROS = async () => {
    try {
        console.log('开始连接ROS服务...')
        await rosConnection.connect()
        isConnected.value = true
        console.log('ROS连接成功')

        rosConnection.onMapReceived(async (map: MapData) => {
            console.log('接收到地图数据:', map)
            mapData.value = map
            await createMapBitmap(map)
            await nextTick()
            resetView()
        })

        rosConnection.onRobotPositionUpdate(throttle((position: RobotPosition) => {
            console.log('接收到机器人位置数据:', position)
            const isFirstRobotPosition = !robotPosition.value
            robotPosition.value = position

            // 第一次接收到机器人位置时，自动居中显示
            if (isFirstRobotPosition && mapData.value) {
                console.log('第一次接收机器人位置，自动居中')
                nextTick(() => {
                    centerOnRobot()
                    ElMessage.success('机器人位置已定位')
                })
            } else if (isTrackingRobot.value) {
                centerOnRobot(false) // 追踪时平滑移动，不带动画
            }
            scheduleRedraw()
        }, 50)) // 20Hz

        // 开始追踪机器人位置
        rosConnection.startTracking()
        console.log('ROS事件监听器已设置，开始追踪机器人位置')
        ElMessage.success('成功连接到ROS')
    } catch (error) {
        ElMessage.error('连接ROS失败')
        console.error('ROS连接错误:', error)
    }
}

// --- 核心渲染逻辑 ---

// 创建地图位图缓存 (Y轴翻转，使之与Canvas坐标系匹配)
const createMapBitmap = async (map: MapData) => {
    const { width, height, data } = map
    const canvas = document.createElement('canvas')
    canvas.width = width
    canvas.height = height
    const ctx = canvas.getContext('2d')!
    const imageData = ctx.createImageData(width, height)
    const pixelData = imageData.data

    for (let i = 0; i < data.length; i++) {
        const value = data[i]
        let color = 205 // 未知区域
        if (value === 0) color = 254 // 自由空间
        else if (value === 100) color = 0 // 障碍物

        const pixelIndex = i * 4
        pixelData[pixelIndex] = color
        pixelData[pixelIndex + 1] = color
        pixelData[pixelIndex + 2] = color
        pixelData[pixelIndex + 3] = 255
    }
    ctx.putImageData(imageData, 0, 0)

    // Y-flip the image data to match canvas coordinates (Y-down)
    const finalCanvas = document.createElement('canvas')
    finalCanvas.width = width * MAP_X_STRETCH
    finalCanvas.height = height
    const finalCtx = finalCanvas.getContext('2d')!
    finalCtx.scale(MAP_X_STRETCH, -1)
    finalCtx.translate(0, -height)
    finalCtx.drawImage(canvas, 0, 0)

    if (mapBitmap) mapBitmap.close()
    mapBitmap = await createImageBitmap(finalCanvas)
}

// 应用核心变换 (所有图层的基准)
const applyTransformations = (ctx: CanvasRenderingContext2D) => {
    if (!mapContainer.value || !mapData.value) return
    const container = mapContainer.value
    const mapCenterPx = { x: mapData.value.width / 2, y: mapData.value.height / 2 }

    // 1. 移动到容器中心
    ctx.translate(container.clientWidth / 2, container.clientHeight / 2)
    // 2. 应用拖拽平移
    ctx.translate(offsetX.value, offsetY.value)
    // 3. 应用旋转
    ctx.rotate(rotation.value * Math.PI / 180)
    // 4. 应用缩放
    ctx.scale(scale.value, scale.value)
    // 5. 将地图中心对准原点
    ctx.translate(-mapCenterPx.x, -mapCenterPx.y)
}

// 绘制静态地图层
const drawStaticMap = () => {
    if (!mapCanvas.value || !mapBitmap) return
    const canvas = mapCanvas.value
    const ctx = canvas.getContext('2d')!

    ctx.clearRect(0, 0, canvas.width, canvas.height)
    ctx.save()
    applyTransformations(ctx)
    ctx.imageSmoothingEnabled = false // 像素风
    ctx.drawImage(mapBitmap, 0, 0)
    ctx.restore()
}

// 绘制机器人层
const drawRobotLayer = () => {
    if (!robotCanvas.value || !robotPosition.value || !mapData.value) return
    const canvas = robotCanvas.value
    const ctx = canvas.getContext('2d')!

    ctx.clearRect(0, 0, canvas.width, canvas.height)
    ctx.save()
    applyTransformations(ctx) // 应用完全相同的变换

    const { resolution, origin, height } = mapData.value
    const { x, y } = robotPosition.value

    // 应用微调偏移量（在世界坐标系中）
    const adjustedX = x + ROBOT_POSITION_OFFSET.x
    const adjustedY = y + ROBOT_POSITION_OFFSET.y

    // 将调整后的ROS世界坐标 (米) 转换为地图像素坐标
    const robotMapX = ((adjustedX - origin.position.x) / resolution) * MAP_X_STRETCH
    const robotMapY = height - ((adjustedY - origin.position.y) / resolution) // Y轴翻转

    // 调试信息：确保坐标计算正确
    console.log('Robot position:', { x, y })
    console.log('Adjusted position:', { adjustedX, adjustedY })
    console.log('Map origin:', origin.position)
    console.log('Robot map coordinates:', { robotMapX, robotMapY })

    // --- 绘制机器人 ---
    ctx.save()
    ctx.translate(robotMapX, robotMapY)

    // 绘制一个醒目的圆圈代表机器人位置，排除绘制复杂图形可能带来的问题
    const robotRadius = Math.max(5, 8 / scale.value) // 确保在任何缩放级别都可见
    ctx.fillStyle = 'rgba(239, 68, 68, 0.9)' // 使用醒目的、略带透明的红色
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.9)'
    ctx.lineWidth = Math.max(1, 2 / scale.value)
    ctx.beginPath()
    ctx.arc(0, 0, robotRadius, 0, 2 * Math.PI)
    ctx.fill()
    ctx.stroke()

    // 绘制方向线
    ctx.rotate(-robotYawRadians.value) // 应用旋转
    ctx.beginPath()
    ctx.moveTo(robotRadius, 0)
    ctx.lineTo(robotRadius * 2, 0)
    ctx.strokeStyle = '#111827' // 深色方向线
    ctx.stroke()

    ctx.restore()
    // --- 结束绘制 ---

    ctx.restore()
}

// 调度重绘 (rAF)
const scheduleRedraw = () => {
    if (pendingFrame) return
    pendingFrame = requestAnimationFrame(() => {
        drawStaticMap()
        drawRobotLayer()
        pendingFrame = null
    })
}

// --- 交互事件处理 ---

const handleMouseDown = (event: MouseEvent) => {
    if (isTrackingRobot.value) {
        isTrackingRobot.value = false
        ElMessage.info('手动操作已取消机器人追踪')
    }
    isDragging.value = true
    lastMouseX.value = event.clientX
    lastMouseY.value = event.clientY
    robotCanvas.value!.style.cursor = 'grabbing'
}

const handleMouseMove = (event: MouseEvent) => {
    if (!isDragging.value) return
    const dx = event.clientX - lastMouseX.value
    const dy = event.clientY - lastMouseY.value
    lastMouseX.value = event.clientX
    lastMouseY.value = event.clientY

    offsetX.value += dx
    offsetY.value += dy
    scheduleRedraw()
}

const handleMouseUp = () => {
    isDragging.value = false
    robotCanvas.value!.style.cursor = 'grab'
    constrainView()
    scheduleRedraw()
}

// 以鼠标为中心缩放
const handleZoom = (event: WheelEvent) => {
    if (!mapContainer.value) return
    if (isTrackingRobot.value) {
        isTrackingRobot.value = false
        ElMessage.info('手动操作已取消机器人追踪')
    }
    const rect = mapContainer.value.getBoundingClientRect()
    const mouseX = event.clientX - rect.left
    const mouseY = event.clientY - rect.top

    const containerCenterX = rect.width / 2
    const containerCenterY = rect.height / 2

    // 1. 计算鼠标相对于当前视图中心的位置
    const mouseRelativeToCenter = {
        x: mouseX - containerCenterX - offsetX.value,
        y: mouseY - containerCenterY - offsetY.value
    }

    // 2. 计算新旧缩放比例
    const oldScale = scale.value
    const zoomFactor = 1.1
    const newScale = event.deltaY < 0 ? oldScale * zoomFactor : oldScale / zoomFactor
    scale.value = Math.max(minScale.value, Math.min(newScale, maxScale.value))
    const scaleRatio = scale.value / oldScale

    // 3. 调整偏移量，以保持鼠标下的点不动
    offsetX.value += mouseRelativeToCenter.x - mouseRelativeToCenter.x * scaleRatio
    offsetY.value += mouseRelativeToCenter.y - mouseRelativeToCenter.y * scaleRatio

    constrainView()
    scheduleRedraw()
}

// --- 控制函数 ---

const toggleRobotTracking = () => {
    isTrackingRobot.value = !isTrackingRobot.value
    if (isTrackingRobot.value) {
        ElMessage.success('已开启机器人追踪')
        centerOnRobot()
    } else {
        ElMessage.info('已关闭机器人追踪')
    }
}

// 将机器人置于视图中心
const centerOnRobot = (redraw = true) => {
    if (!robotPosition.value || !mapData.value) return

    const { resolution, origin, width, height } = mapData.value

    // 应用相同的微调偏移量
    const adjustedX = robotPosition.value.x + ROBOT_POSITION_OFFSET.x
    const adjustedY = robotPosition.value.y + ROBOT_POSITION_OFFSET.y

    // 使用与drawRobotLayer完全相同的坐标转换
    const robotMapX = ((adjustedX - origin.position.x) / resolution) * MAP_X_STRETCH
    const robotMapY = height - ((adjustedY - origin.position.y) / resolution) // Y轴翻转

    const mapCenterPx = { x: width / 2, y: height / 2 }

    // 计算机器人相对于地图中心的像素偏移
    const robotOffsetX = robotMapX - mapCenterPx.x
    const robotOffsetY = robotMapY - mapCenterPx.y

    // 考虑当前的缩放和旋转，计算需要的视图偏移
    const rad = rotation.value * Math.PI / 180
    const cos = Math.cos(rad)
    const sin = Math.sin(rad)

    // 将机器人在地图中的偏移，转换为需要的视图偏移（取反，因为我们要让机器人居中）
    const rotatedOffsetX = robotOffsetX * cos - robotOffsetY * sin
    const rotatedOffsetY = robotOffsetX * sin + robotOffsetY * cos

    // 应用缩放并取反（因为要将机器人移到中心）
    offsetX.value = -rotatedOffsetX * scale.value
    offsetY.value = -rotatedOffsetY * scale.value

    if (redraw) {
        constrainView()
        scheduleRedraw()
    }
}

const onClickCenterOnRobot = (evt: MouseEvent) => {
    centerOnRobot(true)  // 保留你原来的逻辑
}


// 重置视图到初始状态
const resetView = () => {
    if (!mapData.value || !mapContainer.value) return
    const container = mapContainer.value
    const map = mapData.value

    // 计算合适的初始缩放，使地图能完整显示
    const scaleX = container.clientWidth / map.width
    const scaleY = container.clientHeight / map.height
    scale.value = Math.min(scaleX, scaleY) * 0.9

    // 设置缩放范围
    minScale.value = scale.value * 0.2
    maxScale.value = scale.value * 20

    // 重置状态
    rotation.value = -90 // 默认旋转-90度，使地图朝上
    offsetX.value = 0
    offsetY.value = 0
    isTrackingRobot.value = false

    constrainView()
    scheduleRedraw()
}

// 限制视图拖动范围
const constrainView = () => {
    // (可选) 这是一个简化的实现，可以根据需要变得更复杂
    // 目标是防止地图完全移出视野
    if (!mapData.value || !mapContainer.value) return
    const map = mapData.value
    const container = mapContainer.value

    const effectiveMapWidth = map.width * scale.value
    const effectiveMapHeight = map.height * scale.value

    const limitX = (effectiveMapWidth / 2) + (container.clientWidth / 2) * 0.8
    const limitY = (effectiveMapHeight / 2) + (container.clientHeight / 2) * 0.8

    offsetX.value = Math.max(-limitX, Math.min(limitX, offsetX.value))
    offsetY.value = Math.max(-limitY, Math.min(limitY, offsetY.value))
}

// 缩放控件
const onZoomSliderChange = (event: Event) => {
    const target = event.target as HTMLInputElement
    scale.value = parseFloat(target.value)
    constrainView()
    scheduleRedraw()
}
const zoomIn = () => {
    scale.value = Math.min(maxScale.value, scale.value * 1.25)
    constrainView()
    scheduleRedraw()
}
const zoomOut = () => {
    scale.value = Math.max(minScale.value, scale.value / 1.25)
    constrainView()
    scheduleRedraw()
}

// --- 生命周期钩子 ---

const resizeCanvas = () => {
    if (!mapContainer.value || !mapCanvas.value || !robotCanvas.value) return
    const container = mapContainer.value
    mapCanvas.value.width = container.clientWidth
    mapCanvas.value.height = container.clientHeight
    robotCanvas.value.width = container.clientWidth
    robotCanvas.value.height = container.clientHeight
    scheduleRedraw()
}

onMounted(() => {
    nextTick(() => {
        resizeCanvas()
        window.addEventListener('resize', resizeCanvas)
    })
})

onUnmounted(() => {
    window.removeEventListener('resize', resizeCanvas)
    if (pendingFrame) cancelAnimationFrame(pendingFrame)
    if (mapBitmap) mapBitmap.close()
    rosConnection.disconnect()
})

</script>

<style scoped>
.embedded-map-viewer {
    display: flex;
    flex-direction: column;
    height: 100%;
    background: #f0f2f5;
    border-radius: 12px;
    overflow: hidden;
    box-shadow: 0 8px 24px rgba(0, 0, 0, 0.1);
}

.map-controls {
    display: flex;
    justify-content: center;
    /* 中心对齐按钮组 */
    align-items: center;
    padding: 12px 16px;
    background: #ffffff;
    border-bottom: 1px solid #e5e7eb;
    flex-shrink: 0;
}

.map-status {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 16px;
    padding: 8px 16px;
    background: rgba(255, 255, 255, 0.8);
    backdrop-filter: blur(5px);
    border-bottom: 1px solid #e5e7eb;
    font-size: 12px;
    color: #4b5563;
    flex-shrink: 0;
}

.robot-info,
.map-info {
    font-family: 'Monaco', 'Menlo', 'Courier New', monospace;
    font-weight: 500;
    background-color: #f3f4f6;
    padding: 4px 8px;
    border-radius: 4px;
}

.map-container {
    flex-grow: 1;
    position: relative;
    overflow: hidden;
    background-color: #e5e7eb;
    background-image:
        linear-gradient(45deg, #d1d5db 25%, transparent 25%),
        linear-gradient(-45deg, #d1d5db 25%, transparent 25%),
        linear-gradient(45deg, transparent 75%, #d1d5db 75%),
        linear-gradient(-45deg, transparent 75%, #d1d5db 75%);
    background-size: 20px 20px;
}

.map-layer,
.robot-layer {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
}

.map-layer {
    pointer-events: none;
    z-index: 1;
}

.robot-layer {
    cursor: grab;
    z-index: 2;
}

.robot-layer:active {
    cursor: grabbing;
}

/* 缩放控件样式 */
.zoom-controls {
    position: absolute;
    bottom: 20px;
    right: 20px;
    z-index: 10;
    background: rgba(255, 255, 255, 0.9);
    backdrop-filter: blur(8px);
    border-radius: 8px;
    padding: 8px;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
    border: 1px solid rgba(0, 0, 0, 0.05);
}

.zoom-slider-container {
    display: flex;
    align-items: center;
    gap: 8px;
}

.zoom-btn {
    width: 28px;
    height: 28px;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    background-color: #ffffff;
    color: #4b5563;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: all 0.2s ease;
}

.zoom-btn:hover {
    background-color: #f3f4f6;
    border-color: #9ca3af;
}

.zoom-slider-wrapper {
    display: flex;
    align-items: center;
    gap: 8px;
    width: 150px;
}

.zoom-slider {
    width: 100%;
    height: 4px;
    border-radius: 2px;
    background: #d1d5db;
    outline: none;
    cursor: pointer;
    -webkit-appearance: none;
    appearance: none;
}

.zoom-slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 16px;
    height: 16px;
    border-radius: 50%;
    background: #3b82f6;
    cursor: pointer;
    border: 2px solid #ffffff;
    box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
    transition: transform 0.2s ease;
}

.zoom-slider::-webkit-slider-thumb:hover {
    transform: scale(1.1);
}

.zoom-slider::-moz-range-thumb {
    width: 12px;
    height: 12px;
    border-radius: 50%;
    background: #3b82f6;
    cursor: pointer;
    border: 2px solid #ffffff;
}

.zoom-value {
    font-size: 12px;
    font-weight: 600;
    color: #4b5563;
    font-family: monospace;
    min-width: 40px;
    text-align: center;
}
</style>
