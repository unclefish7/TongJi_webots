<template>
  <el-card class="task-queue-card">
    <template #header>
      <div class="card-header">
        <el-icon><List /></el-icon>
        <span>任务队列</span>
        <div class="queue-stats" v-if="queueStatus">
          <el-tag size="small" type="info">L0: {{ queueStatus.queues?.L0 || 0 }}</el-tag>
          <el-tag size="small" type="success">L1: {{ queueStatus.queues?.L1 || 0 }}</el-tag>
          <el-tag size="small" type="warning">L2: {{ queueStatus.queues?.L2 || 0 }}</el-tag>
          <el-tag size="small" type="danger">L3: {{ queueStatus.queues?.L3 || 0 }}</el-tag>
          <el-tag size="small" type="success" v-if="queueStatus.current_execution?.active">执行中</el-tag>
        </div>
        <el-button 
          @click="refreshQueue" 
          size="small" 
          type="primary" 
          plain
          class="refresh-btn"
        >
          <el-icon><Refresh /></el-icon>
          刷新
        </el-button>
      </div>
    </template>

    <!-- 队列控制按钮 - 固定在顶部 -->
    <div class="queue-controls">
      <div class="control-buttons">
        <el-button 
          @click="startNextTask" 
          type="primary" 
          size="small"
          :loading="isStarting"
          :disabled="!canStartTask"
        >
          <el-icon><VideoPlay /></el-icon>
          开始执行任务队列
        </el-button>
        
        <el-button 
          @click="sendNextCommand" 
          type="success" 
          size="small"
          :loading="isSendingNext"
          :disabled="!canContinueTask"
        >
          <el-icon><Right /></el-icon>
          继续下一任务
        </el-button>
      </div>
    </div>

    <div class="queue-container">
      <!-- 可滚动内容区域 -->
      <div class="scrollable-content">
        <!-- 执行状态信息 -->
        <div class="execution-status" v-if="queueStatus?.current_execution">
          <div class="status-header">
            <h4>执行状态</h4>
            <el-tag 
              :type="queueStatus.current_execution.active ? 'success' : 'info'" 
              size="small"
            >
              {{ queueStatus.current_execution.active ? '执行中' : '空闲' }}
            </el-tag>
          </div>
          
          <div v-if="queueStatus.current_execution.active" class="execution-details">
            <div class="execution-info">
              <span>当前队列: {{ queueStatus.current_execution.current_queue_level }}</span>
              <span>进度: {{ queueStatus.current_execution.progress }}</span>
              <span>剩余任务: {{ queueStatus.current_execution.remaining_tasks }}</span>
            </div>
            
            <!-- 当前执行任务 - 移除进度条 -->
            <div v-if="queueStatus.current_executing_task" class="current-task">
              <h5>当前执行任务</h5>
              <div class="task-card current-executing">
                <div class="task-header">
                  <el-tag :type="getTaskTypeColor(queueStatus.current_executing_task.security_level)" size="small">
                    {{ queueStatus.current_executing_task.security_level }}
                  </el-tag>
                  <el-tag type="warning" size="small">执行中</el-tag>
                </div>
                <div class="task-content">
                  <div class="task-info">
                    <span>任务ID: {{ queueStatus.current_executing_task.task_id }}</span>
                    <span>目标: {{ getLocationDisplayName(queueStatus.current_executing_task.location_id) }}</span>
                    <span>发起人: {{ getUserDisplayName(queueStatus.current_executing_task.user_id) }}</span>
                    <span>接收人: {{ getUserDisplayName(queueStatus.current_executing_task.receiver) }}</span>
                    <span>描述: {{ queueStatus.current_executing_task.description || '无' }}</span>
                    <span v-if="queueStatus.current_executing_task.progress">
                      执行进度: {{ queueStatus.current_executing_task.progress }}
                    </span>
                  </div>
                </div>
              </div>
            </div>

            <!-- 到达任务信息 -->
            <div v-if="queueStatus.arrived_task" class="arrived-task">
              <h5>等待取件任务</h5>
              <div class="task-card arrived">
                <div class="task-header">
                  <el-tag type="success" size="small">已到达</el-tag>
                  <el-tag type="warning" size="small">
                    剩余时间: {{ queueStatus.arrived_task.timeout_remaining }}秒
                  </el-tag>
                </div>
                <div class="task-content">
                  <div class="task-info">
                    <span>任务ID: {{ queueStatus.arrived_task.task_id }}</span>
                    <span>位置: {{ getLocationDisplayName(queueStatus.arrived_task.location_id) }}</span>
                    <span>接收人: {{ getUserDisplayName(queueStatus.arrived_task.receiver) }}</span>
                    <span>等待时间: {{ queueStatus.arrived_task.waiting_time }}秒</span>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- 执行队列详情 -->
        <div class="execution-queue" v-if="queueStatus?.execution_queue?.all_tasks?.length > 0">
          <h4>执行队列详情</h4>
          <div class="execution-summary">
            <span>总任务数: {{ queueStatus.execution_queue.total_in_queue }}</span>
            <span>已完成: {{ queueStatus.execution_queue.completed_count }}</span>
            <span>剩余: {{ queueStatus.execution_queue.remaining_count }}</span>
          </div>
          
          <div class="execution-task-list">
            <div
              v-for="task in queueStatus.execution_queue.all_tasks"
              :key="task.task_id"
              class="task-item execution-task"
              :class="{ 
                'task-completed': task.status === 'completed_in_queue',
                'task-executing': task.status === 'executing',
                'task-waiting': task.status === 'waiting_in_queue'
              }"
            >
              <div class="task-header">
                <div class="task-order">{{ task.execution_order }}</div>
                <el-tag 
                  :type="getExecutionStatusColor(task.status)" 
                  size="small"
                >
                  {{ getExecutionStatusName(task.status) }}
                </el-tag>
              </div>
              
              <div class="task-content">
                <div class="task-info">
                  <span>{{ getLocationDisplayName(task.location_id) }}</span>
                  <span>{{ getUserDisplayName(task.receiver) }}</span>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- 等待队列详情 -->
        <div class="queue-details" v-if="queueStatus?.queue_details">
          <div class="queue-level" v-for="(level, levelName) in queueStatus.queue_details" :key="levelName">
            <div class="level-header" v-if="level.length > 0">
              <h4>{{ levelName }} 队列 ({{ level.length }}个任务)</h4>
              <el-tag :type="getQueueLevelColor(String(levelName))" size="small">
                {{ getQueueLevelName(String(levelName)) }}
              </el-tag>
            </div>
            
            <div class="task-list">
              <div
                v-for="(task, index) in level"
                :key="task.task_id"
                class="task-item"
              >
                <div class="task-header">
                  <div class="task-index">#{{ index + 1 }}</div>
                  <el-tag 
                    :type="getTaskTypeColor(task.security_level)" 
                    size="small"
                  >
                    {{ task.security_level }}
                  </el-tag>
                  <el-tag 
                    :type="getStatusTypeColor(task.status)" 
                    size="small"
                    class="task-status"
                  >
                    {{ getStatusName(task.status) }}
                  </el-tag>
                </div>
                
                <div class="task-content">
                  <div class="task-location">
                    <el-icon><Location /></el-icon>
                    目标: {{ getLocationDisplayName(task.location_id) }}
                  </div>
                  <div class="task-user">
                    <el-icon><User /></el-icon>
                    发起人: {{ getUserDisplayName(task.user_id) }}
                  </div>
                  <div class="task-user">
                    <el-icon><User /></el-icon>
                    接收人: {{ getUserDisplayName(task.receiver) }}
                  </div>
                  <div class="task-description">
                    {{ task.description || '无描述' }}
                  </div>
                  <div class="task-time">
                    <el-icon><Clock /></el-icon>
                    创建时间: {{ formatTime(task.created_at) }}
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- 空队列提示 -->
        <div v-if="getTotalQueueCount() === 0" class="empty-queue">
          <el-empty description="队列为空，暂无任务" />
        </div>
      </div>
    </div>
  </el-card>
</template>

<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { taskApiService } from '@/services/taskApiService'
import { nameMapService } from '@/services/nameMapService'
import { 
  List, 
  Refresh, 
  Location, 
  User, 
  Clock,
  VideoPlay,
  Right
} from '@element-plus/icons-vue'

// 状态数据
const queueStatus = ref<any>(null)
const isRefreshing = ref(false)
const isStarting = ref(false)
const isSendingNext = ref(false)

// 计算属性
const canStartTask = computed(() => {
  // 总是可以点击，让后端处理是否能启动的逻辑
  return true
})

const canContinueTask = computed(() => {
  return queueStatus.value?.current_execution?.active && 
         queueStatus.value?.current_execution?.waiting_for_next
})

// 辅助函数
const getTotalQueueCount = () => {
  if (!queueStatus.value?.queues) return 0
  return Object.values(queueStatus.value.queues).reduce((total: number, count: any) => total + (count || 0), 0)
}

// 名称映射函数
const getUserDisplayName = (userId: string): string => {
  return nameMapService.getUserName(userId)
}

const getLocationDisplayName = (locationId: string): string => {
  return nameMapService.getLocationName(locationId)
}

// 初始化
onMounted(async () => {
  // 初始化名称映射服务
  await nameMapService.initialize()
  
  await refreshQueue()
  
  // 定期刷新队列状态
  setInterval(refreshQueue, 8000)
})

// 方法
const refreshQueue = async () => {
  try {
    isRefreshing.value = true
    const response = await taskApiService.getQueueStatus()
    
    if (response.success && response.data) {
      queueStatus.value = response.data
      console.log('队列状态已更新:', response.data)
    } else {
      ElMessage.error('获取任务队列失败')
    }
  } catch (error: any) {
    console.error('获取任务队列失败:', error)
    ElMessage.error(error.message || '获取任务队列失败')
  } finally {
    isRefreshing.value = false
  }
}

const startNextTask = async () => {
  try {
    isStarting.value = true
    
    const result = await taskApiService.startTask()
    if (result.success) {
      ElMessage.success(result.message || '任务已启动')
      await refreshQueue()
    } else {
      // 如实显示后端返回的错误信息
      ElMessage.error(result.message || '启动任务失败')
    }
  } catch (error: any) {
    console.error('启动任务失败:', error)
    // 显示详细的错误信息
    const errorMessage = error.response?.data?.message || 
                        error.message || 
                        '启动任务失败'
    ElMessage.error(errorMessage)
  } finally {
    isStarting.value = false
  }
}

const sendNextCommand = async () => {
  try {
    isSendingNext.value = true
    
    const result = await taskApiService.sendNext()
    if (result.success) {
      ElMessage.success(result.message || '已发送继续指令')
      await refreshQueue()
    } else {
      ElMessage.error(result.message || '发送继续指令失败')
    }
  } catch (error: any) {
    console.error('发送继续指令失败:', error)
    ElMessage.error(error.message || '发送继续指令失败')
  } finally {
    isSendingNext.value = false
  }
}

// 工具方法
const getTaskTypeColor = (securityLevel: string) => {
  const colors = {
    L0: 'info',
    L1: 'success',
    L2: 'warning', 
    L3: 'danger'
  }
  return colors[securityLevel as keyof typeof colors] || 'info'
}

const getQueueLevelColor = (level: string) => {
  const colors = {
    L0: 'info',
    L1: 'success',
    L2: 'warning',
    L3: 'danger'
  }
  return colors[level as keyof typeof colors] || 'info'
}

const getQueueLevelName = (level: string) => {
  const names = {
    L0: '降级队列',
    L1: '普通优先级',
    L2: '中等优先级',
    L3: '高级优先级'
  }
  return names[level as keyof typeof names] || level
}

const getStatusTypeColor = (status: string) => {
  const colors = {
    pending: 'info',
    executing: 'warning',
    arrived: 'success',
    completed: 'success',
    completed_in_queue: 'success',
    waiting_in_queue: 'info',
    failed: 'danger'
  }
  return colors[status as keyof typeof colors] || 'info'
}

const getStatusName = (status: string) => {
  const names = {
    pending: '等待中',
    executing: '执行中',
    arrived: '已到达',
    completed: '已完成',
    completed_in_queue: '已完成',
    waiting_in_queue: '等待中',
    failed: '失败'
  }
  return names[status as keyof typeof names] || status
}

const getExecutionStatusColor = (status: string) => {
  const colors = {
    completed_in_queue: 'success',
    executing: 'warning',
    waiting_in_queue: 'info'
  }
  return colors[status as keyof typeof colors] || 'info'
}

const getExecutionStatusName = (status: string) => {
  const names = {
    completed_in_queue: '已完成',
    executing: '执行中',
    waiting_in_queue: '等待中'
  }
  return names[status as keyof typeof names] || status
}

const getTaskProgressPercentage = (task: any) => {
  if (!task.progress) return 0
  
  // 解析形如 "2/3" 的进度字符串
  const match = task.progress.match(/(\d+)\/(\d+)/)
  if (match) {
    const current = parseInt(match[1])
    const total = parseInt(match[2])
    return Math.floor((current / total) * 100)
  }
  return 0
}

const formatTime = (timeStr: string) => {
  if (!timeStr) return '未知'
  return new Date(timeStr).toLocaleString('zh-CN')
}
</script>

<style scoped>
.task-queue-card {
  height: 100%;
  margin-bottom: 1rem;
  display: flex;
  flex-direction: column;
}

.task-queue-card :deep(.el-card__body) {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  padding: 16px 20px;
}

.card-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  font-weight: 600;
  flex-shrink: 0;
}

.card-header > span {
  flex: 1;
  margin-left: 8px;
}

.queue-stats {
  display: flex;
  gap: 8px;
  align-items: center;
}

.refresh-btn {
  margin-left: 12px;
}

.queue-container {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  min-height: 0; /* 允许flex子元素收缩 */
}

.queue-controls {
  margin-bottom: 16px;
  padding: 12px;
  background: #fafafa;
  border-radius: 6px;
  flex-shrink: 0; /* 防止压缩 */
}

.control-buttons {
  display: flex;
  gap: 8px;
}

.scrollable-content {
  flex: 1;
  overflow-y: auto;
  min-height: 0;
  padding-right: 4px; /* 为滚动条留出空间 */
}

.execution-status {
  margin-bottom: 16px;
  padding: 12px;
  background: #f8f9fa;
  border-radius: 6px;
  /* 移除flex-shrink: 0，让它可以根据内容调整大小 */
}

.status-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 8px;
}

.queue-details {
  /* 移除flex和overflow设置，让它自然布局 */
}

.queue-level {
  margin-bottom: 16px;
}

.task-list {
  /* 移除flex和overflow设置，让它自然布局 */
}

.status-header h4 {
  margin: 0;
  font-size: 14px;
  color: #666;
}

.execution-details {
  margin-top: 12px;
}

.execution-info {
  display: flex;
  gap: 16px;
  margin-bottom: 12px;
  font-size: 12px;
  color: #666;
}

.current-task h5,
.arrived-task h5 {
  margin: 12px 0 8px 0;
  font-size: 13px;
  color: #333;
}

.task-card {
  border: 1px solid #e4e7ed;
  border-radius: 6px;
  padding: 12px;
  margin-bottom: 8px;
}

.task-card.current-executing {
  border-color: #f56c6c;
  background: #fef2f2;
}

.task-card.arrived {
  border-color: #67c23a;
  background: #f0f9ff;
}

.level-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 8px;
  padding: 8px 12px;
  background: #f5f7fa;
  border-radius: 4px;
}

.level-header h4 {
  margin: 0;
  font-size: 14px;
  color: #333;
}

.task-list {
  /* 移除flex和overflow设置，让它自然布局 */
}

.task-item {
  border: 1px solid #e4e7ed;
  border-radius: 6px;
  padding: 12px;
  margin-bottom: 8px;
  background: white;
  transition: all 0.3s ease;
}

.task-item:hover {
  border-color: #c6e2ff;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.task-item.task-active {
  border-color: #409eff;
  background: #ecf5ff;
}

.task-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 8px;
}

.task-index {
  font-weight: bold;
  color: #666;
  font-size: 12px;
}

.task-order {
  background: #409eff;
  color: white;
  border-radius: 50%;
  width: 20px;
  height: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 12px;
  font-weight: bold;
}

.task-content {
  font-size: 13px;
  line-height: 1.6;
}

.task-location,
.task-user,
.task-description,
.task-time {
  display: flex;
  align-items: center;
  margin-bottom: 4px;
  color: #666;
}

.task-location .el-icon,
.task-user .el-icon,
.task-time .el-icon {
  margin-right: 6px;
  color: #909399;
}

.task-info {
  display: flex;
  flex-direction: column;
  gap: 4px;
  font-size: 12px;
  color: #666;
}

.task-info span {
  display: block;
}

.execution-queue {
  margin-top: 16px;
  padding: 12px;
  background: #f8f9fa;
  border-radius: 6px;
}

.execution-queue h4 {
  margin: 0 0 8px 0;
  font-size: 14px;
  color: #333;
}

.execution-summary {
  display: flex;
  gap: 16px;
  margin-bottom: 12px;
  font-size: 12px;
  color: #666;
}

.execution-task-list {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.execution-task {
  padding: 8px 12px;
  margin-bottom: 0;
}

.execution-task.task-completed {
  background: #f0f9ff;
  border-color: #67c23a;
}

.execution-task.task-executing {
  background: #fef2f2;
  border-color: #f56c6c;
}

.execution-task.task-waiting {
  background: #f5f7fa;
  border-color: #e4e7ed;
}

.empty-queue {
  display: flex;
  align-items: center;
  justify-content: center;
  min-height: 200px;
  margin: 20px 0;
}

.task-status {
  margin-left: 8px;
}

/* 响应式设计 */
@media (max-width: 768px) {
  .card-header {
    flex-direction: column;
    gap: 8px;
    align-items: flex-start;
  }

  .queue-stats {
    order: 1;
  }

  .refresh-btn {
    margin-left: 0;
    order: 2;
  }

  .execution-info {
    flex-direction: column;
    gap: 4px;
  }

  .control-buttons {
    flex-direction: column;
  }
}
</style>
