<template>
  <el-card class="task-queue-card">
    <template #header>
      <div class="card-header">
        <el-icon><List /></el-icon>
        <span>机器人任务队列</span>
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
      <div v-if="taskQueue.length === 0" class="empty-queue">
        <el-empty description="队列为空，暂无任务" />
      </div>
      
      <div v-else class="task-list">
        <div
          v-for="(task, index) in taskQueue"
          :key="task.id"
          class="task-item"
          :class="{ 'task-active': task.status === 'executing' }"
        >
          <div class="task-header">
            <div class="task-index">#{{ index + 1 }}</div>
            <el-tag 
              :type="getTaskTypeColor(task.type)" 
              size="small"
            >
              {{ getTaskTypeName(task.type) }}
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
              目标: {{ task.location }}
            </div>
            <div class="task-user">
              <el-icon><User /></el-icon>
              用户: {{ task.user_id }}
            </div>
            <div class="task-description">
              {{ task.description }}
            </div>
            <div class="task-time">
              <el-icon><Clock /></el-icon>
              创建时间: {{ formatTime(task.created_at) }}
            </div>
          </div>

          <div class="task-actions" v-if="task.status === 'pending'">
            <el-button 
              @click="startTask(task)" 
              size="small" 
              type="primary"
            >
              开始执行
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
import { robotService } from '@/services/robotService'
import { 
  List, 
  Refresh, 
  Location, 
  User, 
  Clock,
  CircleCheck,
  Warning
} from '@element-plus/icons-vue'

// 状态数据
const taskQueue = ref<any[]>([])
const isRefreshing = ref(false)

// 初始化
onMounted(async () => {
  await refreshQueue()
  
  // 定期刷新队列状态
  setInterval(refreshQueue, 5000)
})

// 方法
const refreshQueue = async () => {
  try {
    isRefreshing.value = true
    const queue = await robotService.getTaskQueue()
    taskQueue.value = queue.tasks || []
  } catch (error) {
    console.error('获取任务队列失败:', error)
    ElMessage.error('获取任务队列失败')
  } finally {
    isRefreshing.value = false
  }
}

const startTask = async (task: any) => {
  try {
    await ElMessageBox.confirm(
      `确认开始执行任务 "${task.description}"？`,
      '确认操作',
      {
        confirmButtonText: '确认',
        cancelButtonText: '取消',
        type: 'info',
      }
    )

    await robotService.startTask(task.id)
    ElMessage.success('任务已开始执行')
    await refreshQueue()
  } catch (error) {
    if (error !== 'cancel') {
      console.error('开始任务失败:', error)
      ElMessage.error('开始任务失败')
    }
  }
}

const cancelTask = async (task: any) => {
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

    await robotService.failTask(task.id, '用户取消')
    ElMessage.success('任务已取消')
    await refreshQueue()
  } catch (error) {
    if (error !== 'cancel') {
      console.error('取消任务失败:', error)
      ElMessage.error('取消任务失败')
    }
  }
}

// 工具方法
const getTaskTypeColor = (type: string) => {
  const colors = {
    delivery: 'success',
    pickup: 'warning',
    call: 'info'
  }
  return colors[type as keyof typeof colors] || 'info'
}

const getTaskTypeName = (type: string) => {
  const names = {
    delivery: '配送',
    pickup: '取件',
    call: '呼叫'
  }
  return names[type as keyof typeof names] || type
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
    completed: '已完成',
    failed: '失败'
  }
  return names[status as keyof typeof names] || status
}

const getTaskProgress = (task: any) => {
  // 模拟进度计算
  if (task.status === 'executing') {
    const elapsed = Date.now() - new Date(task.updated_at || task.created_at).getTime()
    const estimated = 5 * 60 * 1000 // 5分钟预估
    return Math.min(Math.floor((elapsed / estimated) * 100), 95)
  }
  return 0
}

const getProgressStatus = (task: any) => {
  const progress = getTaskProgress(task)
  if (progress > 80) return 'warning'
  return 'success'
}

const getProgressText = (task: any) => {
  if (task.type === 'delivery') return '正在前往目标位置...'
  if (task.type === 'pickup') return '正在前往取件位置...'
  if (task.type === 'call') return '正在响应用户呼叫...'
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
}

.card-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  font-weight: 600;
}

.card-header > span {
  flex: 1;
  margin-left: 0.5rem;
}

.refresh-btn {
  margin-left: auto;
}

.queue-container {
  max-height: calc(100vh - 350px); /* 适应侧边栏高度 */
  overflow-y: auto;
  padding: 10px 0;
}

.empty-queue {
  text-align: center;
  padding: 2rem 0;
}

.task-list {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.task-item {
  border: 1px solid #e4e7ed;
  border-radius: 8px;
  padding: 1rem;
  transition: all 0.3s;
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
}

.task-index {
  font-weight: 600;
  color: #606266;
  font-size: 0.9rem;
}

.task-status {
  margin-left: auto;
}

.task-content {
  margin: 0.5rem 0;
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.task-location,
.task-user,
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
