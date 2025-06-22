<template>
  <el-card class="task-queue-card">
    <template #header>
      <div class="card-header">
        <el-icon><List /></el-icon>
        <span>任务队列</span>
        <div class="queue-stats" v-if="queueStatus">
          <el-tag size="small" type="info">L1: {{ queueStatus.queues?.L1 || 0 }}</el-tag>
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

    <div class="queue-container">
      <!-- 队列控制按钮 -->
      <div class="queue-controls" v-if="queueStatus">
        <div class="control-buttons">
          <el-button 
            v-if="!queueStatus.current_execution?.active && getTotalQueueCount() > 0"
            @click="startNextTask" 
            type="primary" 
            size="small"
            :loading="isStarting"
          >
            <el-icon><Refresh /></el-icon>
            启动下一个任务
          </el-button>
          
          <el-button 
            v-if="queueStatus.current_execution?.active"
            @click="sendNextCommand" 
            type="success" 
            size="small"
            :loading="isSendingNext"
          >
            <el-icon><Right /></el-icon>
            继续下一个任务
          </el-button>
          
          <el-button 
            v-if="queueStatus.current_execution?.active"
            @click="handleRobotArrival" 
            type="warning" 
            size="small"
            :loading="isHandlingArrival"
          >
            <el-icon><Check /></el-icon>
            机器人已到达
          </el-button>
        </div>
      </div>

      <div v-if="taskQueue.length === 0" class="empty-queue">
        <el-empty description="队列为空，暂无任务" />
      </div>
      
      <div v-else class="task-list">
        <div
          v-for="(task, index) in taskQueue"
          :key="task.task_id"
          class="task-item"
          :class="{ 'task-active': task.status === 'executing' }"
        >
          <div class="task-header">
            <div class="task-index">#{{ index + 1 }}</div>
            <el-tag 
              :type="getTaskTypeColor(task.security_level)" 
              size="small"
            >
              {{ getTaskTypeName(task.security_level) }}
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
              目标: {{ task.location_id }}
            </div>
            <div class="task-user">
              <el-icon><User /></el-icon>
              发起人: {{ task.user_id }}
            </div>
            <div class="task-user">
              <el-icon><User /></el-icon>
              接收人: {{ task.receiver }}
            </div>
            <div class="task-description">
              {{ task.description || '无描述' }}
            </div>
            <div class="task-time">
              <el-icon><Clock /></el-icon>
              创建时间: {{ formatTime(task.created_at) }}
            </div>
          </div>

          <div class="task-actions" v-if="task.status === 'pending' && index === 0">
            <el-button 
              @click="startTask(task)" 
              size="small" 
              type="primary"
            >
              启动任务
            </el-button>
            <el-button 
              @click="cancelTask(task)" 
              size="small" 
              type="danger"
            >
              取消任务
            </el-button>
          </div>

          <div class="task-progress" v-if="task.status === 'executing'">
            <el-progress 
              :percentage="getTaskProgress(task)" 
              :status="getProgressStatus(task)"
            />
            <div class="progress-text">
              {{ getProgressText(task) }}
            </div>
          </div>
        </div>
      </div>
    </div>
  </el-card>
</template>

<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { taskApiService, type TaskData } from '@/services/taskApiService'
import { 
  List, 
  Refresh, 
  Location, 
  User, 
  Clock,
  CircleCheck,
  Warning,
  Right,
  Check
} from '@element-plus/icons-vue'

// 状态数据
const taskQueue = ref<TaskData[]>([])
const isRefreshing = ref(false)
const isStarting = ref(false)
const isSendingNext = ref(false)
const isHandlingArrival = ref(false)
const queueStatus = ref<any>(null)

// 辅助函数
const getTotalQueueCount = () => {
  if (!queueStatus.value?.queues) return 0
  return Object.values(queueStatus.value.queues).reduce((total: number, count: any) => total + (count || 0), 0)
}

// 初始化
onMounted(async () => {
  await refreshQueue()
  
  // 定期刷新队列状态（降低频率）
  setInterval(refreshQueue, 10000)
})

// 方法
const refreshQueue = async () => {
  try {
    isRefreshing.value = true
    const statusResponse = await taskApiService.getQueueStatus()
    
    if (statusResponse.success && statusResponse.data) {
      queueStatus.value = statusResponse.data
      
      // 合并所有优先级的任务到一个数组中，用于显示
      const allTasks: TaskData[] = []
      
      // 添加所有队列中的任务
      if (statusResponse.data.queue_summary) {
        Object.values(statusResponse.data.queue_summary).forEach((levelTasks: any) => {
          if (Array.isArray(levelTasks)) {
            allTasks.push(...levelTasks)
          }
        })
      }
      
      // 如果有正在执行的任务，添加到列表开头
      if (statusResponse.data.current_executing_task) {
        allTasks.unshift(statusResponse.data.current_executing_task)
      }
      
      taskQueue.value = allTasks
    } else {
      taskQueue.value = []
      ElMessage.error('获取任务队列失败')
    }
  } catch (error: any) {
    console.error('获取任务队列失败:', error)
    ElMessage.error(error.message || '获取任务队列失败')
    taskQueue.value = []
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
      ElMessage.error(result.message || '启动任务失败')
    }
  } catch (error: any) {
    console.error('启动任务失败:', error)
    ElMessage.error(error.message || '启动任务失败')
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

const handleRobotArrival = async () => {
  try {
    isHandlingArrival.value = true
    
    const result = await taskApiService.robotArrived()
    if (result.success) {
      ElMessage.success(result.message || '已处理机器人到达')
      await refreshQueue()
    } else {
      ElMessage.error(result.message || '处理机器人到达失败')
    }
  } catch (error: any) {
    console.error('处理机器人到达失败:', error)
    ElMessage.error(error.message || '处理机器人到达失败')
  } finally {
    isHandlingArrival.value = false
  }
}

const startTask = async (task: TaskData) => {
  try {
    await ElMessageBox.confirm(
      `确认启动下一个任务？`,
      '确认操作',
      {
        confirmButtonText: '确认',
        cancelButtonText: '取消',
        type: 'info',
      }
    )

    const result = await taskApiService.startTask()
    if (result.success) {
      ElMessage.success(result.message || '任务已启动')
      await refreshQueue()
    } else {
      ElMessage.error(result.message || '启动任务失败')
    }
  } catch (error: any) {
    if (error !== 'cancel') {
      console.error('启动任务失败:', error)
      ElMessage.error(error.message || '启动任务失败')
    }
  }
}

const cancelTask = async (task: TaskData) => {
  try {
    await ElMessageBox.confirm(
      `确认取消任务 "${task.description}"？`,
      '确认取消',
      {
        confirmButtonText: '确认',
        cancelButtonText: '取消',
        type: 'warning',
      }
    )

    const result = await taskApiService.cancelTask(task.task_id)
    if (result.success) {
      ElMessage.success(result.message || '任务已取消')
      await refreshQueue()
    } else {
      ElMessage.error(result.message || '取消任务失败')
    }
  } catch (error: any) {
    if (error !== 'cancel') {
      console.error('取消任务失败:', error)
      ElMessage.error(error.message || '取消任务失败')
    }
  }
}

// 工具方法
const getTaskTypeColor = (securityLevel: string) => {
  const colors = {
    L1: 'success',
    L2: 'warning', 
    L3: 'danger'
  }
  return colors[securityLevel as keyof typeof colors] || 'info'
}

const getTaskTypeName = (securityLevel: string) => {
  const names = {
    L1: 'L1级别',
    L2: 'L2级别',
    L3: 'L3级别'
  }
  return names[securityLevel as keyof typeof names] || securityLevel
}

const getStatusTypeColor = (status: string) => {
  const colors = {
    pending: 'info',
    executing: 'warning',
    completed: 'success',
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
    failed: '失败'
  }
  return names[status as keyof typeof names] || status
}

const getTaskProgress = (task: TaskData) => {
  // 模拟进度计算
  if (task.status === 'executing') {
    const elapsed = Date.now() - new Date(task.updated_at || task.created_at).getTime()
    const estimated = 5 * 60 * 1000 // 5分钟预估
    return Math.min(Math.floor((elapsed / estimated) * 100), 95)
  }
  return 0
}

const getProgressStatus = (task: TaskData) => {
  const progress = getTaskProgress(task)
  if (progress > 80) return 'warning'
  return 'success'
}

const getProgressText = (task: TaskData) => {
  if (task.status === 'executing') return '正在执行任务...'
  if (task.status === 'arrived') return '机器人已到达目标位置'
  return '执行中...'
}

const formatTime = (timeStr: string) => {
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
  margin-left: 0.5rem;
}

.queue-stats {
  display: flex;
  gap: 0.5rem;
  margin-left: auto;
  margin-right: 1rem;
}

.refresh-btn {
  margin-left: auto;
}

.queue-container {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  min-height: 0;
}

.queue-controls {
  flex-shrink: 0;
  margin-bottom: 1rem;
  padding: 0.5rem;
  background-color: #f8f9fa;
  border-radius: 8px;
}

.control-buttons {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.empty-queue {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  text-align: center;
  padding: 2rem 0;
}

.task-list {
  flex: 1;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 1rem;
  padding-right: 8px;
}

.task-item {
  border: 1px solid #e4e7ed;
  border-radius: 8px;
  padding: 0.8rem;
  transition: all 0.3s;
  background: #fff;
  margin-bottom: 0.5rem;
  flex-shrink: 0;
}

.task-item:hover {
  border-color: #409eff;
  box-shadow: 0 2px 8px rgba(64, 158, 255, 0.1);
}

.task-active {
  border-color: #e6a23c;
  background-color: #fdf6ec;
}

.task-header {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-bottom: 0.5rem;
  flex-wrap: wrap;
}

.task-index {
  font-weight: 600;
  color: #606266;
  font-size: 0.85rem;
  flex-shrink: 0;
}

.task-status {
  margin-left: auto;
  flex-shrink: 0;
}

.task-content {
  margin: 0.5rem 0;
  display: flex;
  flex-direction: column;
  gap: 0.3rem;
  font-size: 0.85rem;
}

.task-location,
.task-user,
.task-description,
.task-time {
  display: flex;
  align-items: center;
  gap: 0.3rem;
  line-height: 1.4;
  word-break: break-word;
}

.task-description {
  color: #909399;
  font-style: italic;
  font-size: 0.8rem;
}

.task-actions {
  display: flex;
  gap: 0.5rem;
  margin-top: 0.5rem;
  flex-wrap: wrap;
}
.task-time {
  display: flex;
  align-items: center;
  gap: 0.25rem;
  font-size: 0.9rem;
  color: #606266;
}

.task-description {
  color: #303133;
  font-size: 0.9rem;
  margin: 0.25rem 0;
}

.task-actions {
  display: flex;
  gap: 0.5rem;
  margin-top: 0.5rem;
}

.task-progress {
  margin-top: 0.5rem;
}

.progress-text {
  font-size: 0.8rem;
  color: #909399;
  margin-top: 0.25rem;
  text-align: center;
}
</style>
