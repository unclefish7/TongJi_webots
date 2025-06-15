<template>
  <div class="dashboard-container">
    <!-- 顶部导航栏 -->
    <el-header class="dashboard-header">
      <div class="header-left">
        <h1>智能配送机器人管理系统</h1>
      </div>
      <div class="header-center">
        <!-- 用户认证状态显示 -->
        <div class="auth-status">
          <div v-if="isAuthenticated" class="user-info">
            <el-avatar :size="32">
              {{ currentUser?.name?.charAt(0) }}
            </el-avatar>
            <div class="user-details">
              <div class="user-name">{{ currentUser?.name }}</div>
              <el-tag :type="getAuthLevelType(currentUser?.auth_level || '')" size="small">
                {{ currentUser?.auth_level }} 权限
              </el-tag>
            </div>
            <el-button @click="showUserSwitchModal = true" size="small" type="info" plain>
              切换用户
            </el-button>
          </div>
          <div v-else class="no-auth">
            <el-alert 
              title="未认证" 
              description="请先进行身份认证以使用系统功能" 
              type="warning" 
              :closable="false"
            />
            <el-button @click="showUserSwitchModal = true" type="primary" size="small">
              身份认证
            </el-button>
          </div>
        </div>
      </div>
      <div class="header-right">
        <el-button type="primary" @click="$router.push('/call')">
          <el-icon><Phone /></el-icon>
          呼叫机器人
        </el-button>
        <el-button type="success" @click="$router.push('/send')">
          <el-icon><Box /></el-icon>
          寄送包裹
        </el-button>
        <el-button type="warning" @click="$router.push('/receive')">
          <el-icon><Download /></el-icon>
          取件
        </el-button>
      </div>
    </el-header>
    <el-container class="main-container">
      <!-- 左侧状态栏 -->
      <el-aside width="20%" class="status-sidebar">
        <!-- API连接状态 -->
        <ConnectionStatus />
        
        <!-- 机器人状态 -->
        <el-card class="status-card">
          <template #header>
            <div class="card-header">
              <el-icon><User /></el-icon>
              <span>机器人状态</span>
            </div>
          </template>
          <div class="robot-list">
            <div v-for="robot in robotStore.robots" :key="robot.id" class="robot-item">
              <div class="robot-info">
                <div class="robot-name">{{ robot.name }}</div>
                <div class="robot-details">
                  <el-tag :type="getStatusType(robot.status)" size="small">
                    {{ getStatusText(robot.status) }}
                  </el-tag>
                  <div class="battery">
                    <el-icon><CircleCheck /></el-icon>
                    {{ robot.battery }}%
                  </div>
                </div>
                <div v-if="robot.currentTask" class="robot-task">
                  {{ robot.currentTask }}
                </div>
              </div>
              <el-progress
                :percentage="robot.battery"
                :stroke-width="4"
                :show-text="false"
                :color="getBatteryColor(robot.battery)"
              />
            </div>
          </div>
        </el-card>

        <!-- 机器人任务队列 -->
        <RobotTaskQueue />
      </el-aside>

      <!-- 主要内容区域 -->
      <el-main class="map-container">
        <!-- 地图区域 -->
        <el-card class="map-card">
          <template #header>
            <div class="card-header">
              <el-icon><Location /></el-icon>
              <span>实时地图</span>
            </div>
          </template>
          <div class="map-content-wrapper">
            <EmbeddedMapViewer />
          </div>
        </el-card>
      </el-main>
    </el-container>

    <!-- 用户切换/认证模态框 -->
    <UserAuthModal
      v-model="showUserSwitchModal"
      purpose="send"
      required-level="L1"
      @auth-success="handleAuthSuccess"
    />
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, nextTick } from 'vue'
import { useRobotStore } from '@/stores/robot'
import { authService } from '@/services/authService'
import {
  Phone,
  Box,
  Download,
  User,
  Monitor,
  Location,
  ZoomIn,
  ZoomOut,
  Refresh,
  List,
  CircleCheck,
} from '@element-plus/icons-vue'
import ConnectionStatus from '@/components/ConnectionStatus.vue'
import EmbeddedMapViewer from '@/components/EmbeddedMapViewer.vue'
import RobotTaskQueue from '@/components/RobotTaskQueue.vue'
import UserAuthModal from '@/components/UserAuthModal.vue'

const robotStore = useRobotStore()

// 认证相关状态
const showUserSwitchModal = ref(false)

// 认证相关计算属性
const isAuthenticated = computed(() => authService.isAuthenticated())
const currentUser = computed(() => authService.getCurrentUser())

// 方法
const getAuthLevelType = (level: string) => {
  const types = {
    'L1': 'success',
    'L2': 'warning', 
    'L3': 'danger'
  }
  return types[level as keyof typeof types] || 'info'
}

const handleAuthSuccess = (user: any, authResult: any) => {
  console.log('用户认证成功:', user, authResult)
  showUserSwitchModal.value = false
  
  // 触发响应式更新
  nextTick(() => {
    // 强制重新计算computed属性
    console.log('认证状态已更新:', authService.isAuthenticated())
  })
}

// 地图相关数据
const mapContainer = ref()
const floors = ref([
  {
    id: 1,
    name: '1楼',
    style: { left: '50px', top: '50px', width: '200px', height: '100px' },
    rooms: [
      {
        id: '101',
        name: '101',
        style: { left: '10px', top: '10px', width: '80px', height: '30px' },
      },
      {
        id: '102',
        name: '102',
        style: { left: '100px', top: '10px', width: '80px', height: '30px' },
      },
    ],
  },
  {
    id: 2,
    name: '2楼',
    style: { left: '300px', top: '50px', width: '200px', height: '100px' },
    rooms: [
      {
        id: '201',
        name: '201',
        style: { left: '10px', top: '10px', width: '80px', height: '30px' },
      },
      {
        id: '202',
        name: '202',
        style: { left: '100px', top: '10px', width: '80px', height: '30px' },
      },
    ],
  },
  {
    id: 3,
    name: '3楼',
    style: { left: '50px', top: '200px', width: '200px', height: '100px' },
    rooms: [
      {
        id: '301',
        name: '301',
        style: { left: '10px', top: '10px', width: '80px', height: '30px' },
      },
      {
        id: '302',
        name: '302',
        style: { left: '100px', top: '10px', width: '80px', height: '30px' },
      },
    ],
  },
])

const activePaths = ref([
  {
    id: 1,
    d: 'M 100 150 Q 200 200 300 300',
  },
])

// 计算属性
const onlineRobots = computed(() => robotStore.robots.length)
const availableRobots = computed(() => robotStore.availableRobots.length)
const busyRobots = computed(() => robotStore.robots.filter((r) => r.status === 'busy').length)
const availableCompartments = computed(() => robotStore.availableCompartments.length)
const occupiedCompartments = computed(
  () => robotStore.compartments.filter((c) => c.isOccupied).length,
)

// 方法
const getStatusType = (status: string) => {
  const types = {
    idle: 'success',
    busy: 'warning',
    error: 'danger',
    charging: 'info',
  }
  return types[status as keyof typeof types] || 'info'
}

const getStatusText = (status: string) => {
  const texts = {
    idle: '空闲',
    busy: '忙碌',
    error: '故障',
    charging: '充电中',
  }
  return texts[status as keyof typeof texts] || status
}

const getBatteryColor = (battery: number) => {
  if (battery > 60) return '#67c23a'
  if (battery > 30) return '#e6a23c'
  return '#f56c6c'
}

const resetView = () => {
  console.log('重置视图')
}

onMounted(() => {
  // 初始化地图
  console.log('地图初始化完成')
})
</script>

<style scoped>
.dashboard-container {
  height: 100vh;
  width: 100vw;
  display: flex;
  flex-direction: column;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
}

.dashboard-header {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  border-bottom: 1px solid rgba(255, 255, 255, 0.2);
  display: flex;
  justify-content: space-between;
  align-items: center;
  /* padding: 0 30px; */
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  backdrop-filter: blur(10px);
}

.header-left h1 {
  margin: 0;
  color: #ffffff;
  font-size: 24px;
  font-weight: 700;
  text-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
}

.header-center {
  flex: 1;
  display: flex;
  justify-content: center;
  align-items: center;
  margin: 0 2rem;
}

.auth-status {
  display: flex;
  align-items: center;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 12px;
  padding: 0.5rem 1rem;
  backdrop-filter: blur(10px);
}

.user-info {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.user-details {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.user-name {
  color: white;
  font-weight: 600;
  font-size: 0.9rem;
}

.no-auth {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.no-auth .el-alert {
  margin: 0;
}

.header-right {
  display: flex;
  gap: 12px;
}

.header-right .el-button {
  border-radius: 25px;
  padding: 12px 20px;
  font-weight: 600;
  border: none;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  transition: all 0.3s ease;
}

.header-right .el-button:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(0, 0, 0, 0.25);
}

.main-container {
  flex: 1;
  padding: 25px;
  gap: 25px;
  background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
  min-height: 0;
}

.status-sidebar {
  display: flex;
  flex-direction: column;
  gap: 15px;
  padding-right: 15px;
  height: calc(100vh - 80px); /* 减去头部高度 */
  overflow-y: auto;
  
  /* 让内容填满整个高度 */
  > .status-card {
    flex-shrink: 0;
  }
  
  /* 让任务队列组件占据剩余空间 */
  > .task-queue-card {
    flex: 1;
    min-height: 300px;
    overflow: hidden;
  }
}

.status-card {
  height: fit-content;
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
  border: none;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  transition: all 0.3s ease;
}

.status-card:hover {
  transform: translateY(-3px);
  box-shadow: 0 12px 35px rgba(0, 0, 0, 0.15);
}

.card-header {
  display: flex;
  align-items: center;
  gap: 10px;
  font-weight: 700;
  color: #2c3e50;
  font-size: 16px;
}

.card-header > .el-icon {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  padding: 8px;
  border-radius: 8px;
  font-size: 16px;
}

.card-header .el-button .el-icon {
  background: transparent;
  padding: 0;
  border-radius: 0;
  color: inherit;
}

.robot-list {
  display: flex;
  flex-direction: column;
  gap: 15px;
}

.robot-item {
  padding: 16px;
  border: 1px solid rgba(100, 108, 154, 0.2);
  border-radius: 12px;
  background: linear-gradient(135deg, #ffecd2 0%, #fcb69f 100%);
  transition: all 0.3s ease;
  position: relative;
  overflow: hidden;
}

.robot-item::before {
  content: '';
  position: absolute;
  top: 0;
  left: -100%;
  width: 100%;
  height: 100%;
  background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.3), transparent);
  transition: left 0.5s ease;
}

.robot-item:hover::before {
  left: 100%;
}

.robot-item:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.15);
}

.robot-info {
  margin-bottom: 12px;
}

.robot-name {
  font-weight: 700;
  margin-bottom: 6px;
  color: #2c3e50;
  font-size: 16px;
}

.robot-details {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 6px;
}

.battery {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 13px;
  color: #34495e;
  font-weight: 600;
}

.robot-task {
  font-size: 13px;
  color: #7f8c8d;
  margin-top: 6px;
  font-style: italic;
}

.map-container {
  flex: 1;
  min-width: 0;
}

.map-card {
  height: 100%;
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
  border: none;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
}

.map-card .card-header {
  justify-content: space-between;
}

.map-content-wrapper {
  height: calc(100vh - 200px); /* 增加高度，减少上方留白 */
  min-height: 500px; /* 增加最小高度 */
  position: relative;
  overflow: hidden;
  border: 2px solid rgba(100, 108, 154, 0.2);
  border-radius: 12px;
}

.map-background {
  width: 100%;
  height: 100%;
  position: relative;
  background-image:
    linear-gradient(rgba(100, 108, 154, 0.1) 1px, transparent 1px),
    linear-gradient(90deg, rgba(100, 108, 154, 0.1) 1px, transparent 1px);
  background-size: 25px 25px;
}

.floor {
  position: absolute;
  border: 2px solid #667eea;
  border-radius: 10px;
  background: rgba(102, 126, 234, 0.15);
  backdrop-filter: blur(5px);
  transition: all 0.3s ease;
}

.floor:hover {
  background: rgba(102, 126, 234, 0.25);
  transform: scale(1.02);
}

.floor-label {
  position: absolute;
  top: -30px;
  left: 0;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  padding: 6px 12px;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 700;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
}

.room {
  position: absolute;
  border: 2px solid #27ae60;
  border-radius: 6px;
  background: rgba(39, 174, 96, 0.15);
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 13px;
  font-weight: 700;
  color: #27ae60;
  transition: all 0.3s ease;
}

.room:hover {
  background: rgba(39, 174, 96, 0.3);
  transform: scale(1.1);
}

.robot-marker {
  position: absolute;
  z-index: 100;
  transform: translate(-50%, -50%);
  transition: all 0.3s ease;
}

.robot-marker:hover {
  transform: translate(-50%, -50%) scale(1.2);
}

.robot-icon {
  width: 36px;
  height: 36px;
  border-radius: 50%;
  background: linear-gradient(135deg, #27ae60 0%, #2ecc71 100%);
  color: white;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 18px;
  animation: pulse 2s infinite;
  box-shadow: 0 4px 15px rgba(39, 174, 96, 0.4);
}

.robot-busy .robot-icon {
  background: linear-gradient(135deg, #f39c12 0%, #e67e22 100%);
  animation: spin 2s linear infinite;
  box-shadow: 0 4px 15px rgba(243, 156, 18, 0.4);
}

.path-overlay {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.task-footer {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  border-top: 1px solid rgba(255, 255, 255, 0.2);
  padding: 25px;
}

.task-card {
  height: 100%;
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.15);
  border: none;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
}

.task-badge {
  margin-left: auto;
}

/* Element Plus 组件样式覆盖 */
:deep(.el-table) {
  background: transparent;
  border-radius: 10px;
  overflow: hidden;
}

:deep(.el-table th) {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  font-weight: 700;
  border: none;
}

:deep(.el-table td) {
  border-color: rgba(100, 108, 154, 0.1);
}

:deep(.el-table tr:hover > td) {
  background: rgba(102, 126, 234, 0.1);
}

:deep(.el-progress-bar__outer) {
  border-radius: 10px;
  background: rgba(100, 108, 154, 0.2);
}

:deep(.el-progress-bar__inner) {
  border-radius: 10px;
}

:deep(.el-tag) {
  border-radius: 15px;
  border: none;
  font-weight: 600;
}

:deep(.el-descriptions-item__label) {
  font-weight: 700;
  color: #2c3e50;
}

:deep(.el-descriptions-item__content) {
  color: #34495e;
  font-weight: 600;
}

/* 响应式设计 */
@media (max-width: 1200px) {
  .main-container {
    flex-direction: column;
  }

  .status-sidebar {
    width: 100% !important;
    flex-direction: row;
    overflow-x: auto;
  }

  .status-card {
    min-width: 280px;
  }
}

@media (max-width: 768px) {
  .dashboard-header {
    flex-direction: column;
    gap: 15px;
    padding: 20px;
  }

  .header-right {
    flex-wrap: wrap;
    justify-content: center;
  }

  .status-sidebar {
    flex-direction: column;
  }

  .map-content {
    height: 250px;
  }
}

@keyframes pulse {
  0% {
    box-shadow: 0 0 0 0 rgba(39, 174, 96, 0.7);
  }
  70% {
    box-shadow: 0 0 0 15px rgba(39, 174, 96, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(39, 174, 96, 0);
  }
}

@keyframes spin {
  from {
    transform: rotate(0deg);
  }
  to {
    transform: rotate(360deg);
  }
}
</style>
