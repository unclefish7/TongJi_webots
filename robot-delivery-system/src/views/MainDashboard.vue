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
          <div v-if="userStore.isAuthenticated" class="user-info">
            <el-avatar :size="32">
              {{ userStore.currentUser?.name?.charAt(0) }}
            </el-avatar>
            <div class="user-details">
              <div class="user-name">{{ userStore.currentUser?.name }}</div>
              <el-tag :type="userStore.getAuthLevelType(userStore.currentUser?.auth_level || '')" size="small">
                {{ userStore.currentUser?.auth_level }} 权限
              </el-tag>
            </div>
            <el-dropdown @command="switchUser" trigger="click">
              <el-button size="small" type="info" plain>
                切换用户
                <el-icon><ArrowDown /></el-icon>
              </el-button>
              <template #dropdown>
                <el-dropdown-menu>
                  <el-dropdown-item 
                    v-for="user in userStore.authenticatedUsers" 
                    :key="user.user_id"
                    :command="user.user_id"
                    :disabled="user.user_id === userStore.selectedUserId"
                  >
                    <div class="user-option">
                      <span>{{ user.name }} ({{ user.user_id }})</span>
                      <el-tag :type="userStore.getAuthLevelType(user.auth_level)" size="small">
                        {{ user.auth_level }}
                      </el-tag>
                    </div>
                  </el-dropdown-item>
                  <el-dropdown-item divided command="add-auth">
                    <el-button type="primary" text>
                      <el-icon><Plus /></el-icon>
                      新增认证
                    </el-button>
                  </el-dropdown-item>
                </el-dropdown-menu>
              </template>
            </el-dropdown>
          </div>
          <div v-else class="no-auth">
            <el-alert 
              title="未认证, 请先进行身份认证" 
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
      <el-aside width="25%" class="status-sidebar">
        <!-- 机器人任务队列 -->
        <RobotTaskQueue class="task-queue-card" />

        <!-- 智能柜门状态 -->
        <LockerStatusPanel class="status-card" />
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
import { ref, onMounted } from 'vue'
import { useRobotStore } from '@/stores/robot'
import { useUserStore } from '@/stores/user'
import {
  Phone,
  Box,
  Download,
  User as UserIcon,
  Monitor,
  Location,
  ZoomIn,
  ZoomOut,
  Refresh,
  List,
  CircleCheck,
  ArrowDown,
  Plus,
} from '@element-plus/icons-vue'
import EmbeddedMapViewer from '@/components/EmbeddedMapViewer.vue'
import RobotTaskQueue from '@/components/RobotTaskQueue.vue'
import LockerStatusPanel from '@/components/LockerStatusPanel.vue'
import UserAuthModal from '@/components/UserAuthModal.vue'

const robotStore = useRobotStore()
const userStore = useUserStore()

// 认证相关状态
const showUserSwitchModal = ref(false)

// 切换用户
const switchUser = (userId: string) => {
  if (userId === 'add-auth') {
    // 处理新增认证命令
    showUserSwitchModal.value = true
    return
  }
  
  userStore.switchUser(userId)
}

const handleAuthSuccess = async (user: any, authResult: any) => {
  console.log('用户认证成功:', user, authResult)
  
  // 认证成功后自动切换到新用户
  if (user && authResult.success) {
    await userStore.handleAuthSuccess(user)
  }
  
  // 不自动关闭认证模态框，让用户手动关闭
  // showUserSwitchModal.value = false
}

onMounted(async () => {
  // 初始化用户状态
  await userStore.initialize()
  console.log('主界面初始化完成')
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
  padding: 0.5rem 0.5rem;
  /* backdrop-filter: blur(10px); */
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
  /* margin: 0; */
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
  background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
  min-height: 0;
}

.status-sidebar {
  display: flex;
  flex-direction: column;
  gap: 15px;
  padding: 15px;
  height: calc(100vh - 80px);
  overflow: hidden;
}

.status-card {
  flex-shrink: 0;
  height: auto;
  max-height: 40%;
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
  border: none;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  transition: all 0.3s ease;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.task-queue-card {
  flex: 1;
  min-height: 300px;
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
  border: none;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  transition: all 0.3s ease;
  overflow: hidden;
}

.status-card:hover {
  transform: translateY(-3px);
  box-shadow: 0 12px 35px rgba(0, 0, 0, 0.15);
}

.card-header {
  display: flex;
  align-items: center;
  /* gap: 10px; */
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

.card-header span {
  flex: 1;
  margin-left: 5px;
  font-family: monospace;
  font-weight: 800;
}

.card-header .el-button .el-icon {
  background: transparent;
  padding: 0;
  border-radius: 0;
  color: inherit;
}


.map-container {
  flex: 1;
}

.map-card {
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
  border: none;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  margin-top: 0;
}

.map-card .card-header {
  justify-content: space-between;
}

.map-content-wrapper {
  height: 80vh; 
  min-height: 500px; 
  position: relative;
  overflow: scroll;
  /* border: 2px solid rgba(100, 108, 154, 0.2); */
  border-radius: 12px;
}

.path-overlay {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.task-queue-card{
    height: 100%;
    margin-top: 1vh;
    /* margin-left: 1vw; */
    height: 50vh;
    border-radius: 15px;
    box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
    border: none;
    background: rgba(255, 255, 255, 0.95);
    backdrop-filter: blur(10px);
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


.status-card {
  height: fit-content;
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
  border: none;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
}

/* 用户选项样式 */
.user-option {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 8px;
  width: 100%;
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
