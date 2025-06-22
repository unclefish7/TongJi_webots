<template>
  <el-card class="locker-status-card">
    <template #header>
      <div class="card-header">
        <el-icon><Grid /></el-icon>
        <span>智能柜门状态</span>
        <el-button 
          @click="refreshStatus" 
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

    <div class="lockers-grid">
      <div
        v-for="locker in lockers"
        :key="locker.locker_id"
        class="locker-item"
        :class="[
          `locker-${locker.status}`,
          { 'locker-highlighted': highlightedLocker === locker.locker_id }
        ]"
        @click="showLockerDetails(locker)"
      >
        <div class="locker-id">{{ locker.locker_id }}</div>
        <div class="locker-status">
          <el-tag 
            :type="getStatusColor(locker.status)" 
            size="small"
            effect="dark"
          >
            {{ getStatusText(locker.status) }}
          </el-tag>
        </div>
        <div class="locker-info" v-if="locker.task_id">
          <div class="task-id">任务: {{ locker.task_id.slice(-8) }}</div>
          <div class="security-level" v-if="locker.security_level">
            <el-tag size="mini" :type="getSecurityColor(locker.security_level)">
              {{ locker.security_level }}
            </el-tag>
          </div>
        </div>
      </div>
    </div>

    <!-- 柜门详情对话框 -->
    <el-dialog
      v-model="showDetailsDialog"
      title="柜门详情"
      width="60%"
      :close-on-click-modal="false"
    >
      <div v-if="selectedLocker" class="locker-details">
        <el-descriptions :column="1" border>
          <el-descriptions-item label="柜门编号">
            {{ selectedLocker.locker_id }}
          </el-descriptions-item>
          <el-descriptions-item label="状态">
            <el-tag :type="getStatusColor(selectedLocker.status)">
              {{ getStatusText(selectedLocker.status) }}
            </el-tag>
          </el-descriptions-item>
          <el-descriptions-item label="关联任务" v-if="selectedLocker.task_id">
            {{ selectedLocker.task_id }}
          </el-descriptions-item>
          <el-descriptions-item label="用户ID" v-if="selectedLocker.user_id">
            {{ selectedLocker.user_id }}
          </el-descriptions-item>
          <el-descriptions-item label="安全等级" v-if="selectedLocker.security_level">
            <el-tag :type="getSecurityColor(selectedLocker.security_level)">
              {{ selectedLocker.security_level }}
            </el-tag>
          </el-descriptions-item>
          <el-descriptions-item label="更新时间" v-if="selectedLocker.updated_at">
            {{ formatTime(selectedLocker.updated_at) }}
          </el-descriptions-item>
        </el-descriptions>
      </div>
      
      <template #footer>
        <el-button @click="showDetailsDialog = false">关闭</el-button>
      </template>
    </el-dialog>
  </el-card>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { ElMessage } from 'element-plus'
import { systemStatusService, type LockerStatus } from '@/services/systemStatusService'
import { lockerApiService } from '@/services/lockerApiService'
import { Grid, Refresh } from '@element-plus/icons-vue'

// Props
interface Props {
  highlightedLocker?: string
}

const props = withDefaults(defineProps<Props>(), {
  highlightedLocker: ''
})

// 状态数据
const lockers = ref<LockerStatus[]>([])
const selectedLocker = ref<LockerStatus | null>(null)
const showDetailsDialog = ref(false)

// 初始化
onMounted(async () => {
  await refreshStatus()
  
  // 订阅状态更新
  systemStatusService.onStatusUpdate(handleStatusUpdate)
  
  // 使用系统状态服务的响应式数据
  lockers.value = systemStatusService.lockers.value
})

onUnmounted(() => {
  // 取消订阅
  systemStatusService.offStatusUpdate(handleStatusUpdate)
})

// 方法
const refreshStatus = async () => {
  try {
    const response = await lockerApiService.getLockerStatus()
    if (response.success) {
      lockers.value = response.lockers
    } else {
      ElMessage.error(response.message || '获取柜门状态失败')
    }
  } catch (error) {
    console.error('刷新柜门状态失败:', error)
    ElMessage.error('刷新柜门状态失败')
  }
}

const showLockerDetails = (locker: LockerStatus) => {
  selectedLocker.value = locker
  showDetailsDialog.value = true
}

const handleStatusUpdate = (type: string, data: any) => {
  if (type === 'locker') {
    // 更新对应的柜门状态
    const index = lockers.value.findIndex(l => l.locker_id === data.locker_id)
    if (index !== -1) {
      lockers.value[index] = { ...data }
    }
  }
}

// 工具方法
const getStatusColor = (status: string) => {
  const colors = {
    available: 'success',
    occupied: 'warning',
    reserved: 'info'
  }
  return colors[status as keyof typeof colors] || 'info'
}

const getStatusText = (status: string) => {
  const texts = {
    available: '可用',
    occupied: '占用',
    reserved: '预约'
  }
  return texts[status as keyof typeof texts] || status
}

const getSecurityColor = (level: string) => {
  const colors = {
    L1: 'success',
    L2: 'warning',
    L3: 'danger'
  }
  return colors[level as keyof typeof colors] || 'info'
}

const formatTime = (timeStr: string) => {
  return new Date(timeStr).toLocaleString('zh-CN')
}
</script>

<style scoped>
.locker-status-card {
  height: 80%;
  margin-bottom: 1rem;
}

.locker-status-card {
  height: 100%;
  display: flex;
  flex-direction: column;
}

.locker-status-card :deep(.el-card__body) {
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

.refresh-btn {
  margin-left: auto;
}

.lockers-grid {
  flex: 1;
  overflow-y: auto;
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(140px, 1fr));
  gap: 0.8rem;
  padding: 10px 8px 10px 0;
  align-content: start;
}

.locker-item {
  border: 2px solid #e4e7ed;
  border-radius: 8px;
  padding: 0.8rem;
  text-align: center;
  cursor: pointer;
  transition: all 0.3s;
  background: #fff;
  min-height: 100px;
  display: flex;
  flex-direction: column;
  justify-content: center;
}

.locker-item:hover {
  border-color: #409eff;
  box-shadow: 0 2px 8px rgba(64, 158, 255, 0.2);
}

.locker-available {
  border-color: #67c23a;
  background-color: #f0f9ff;
}

.locker-occupied {
  border-color: #e6a23c;
  background-color: #fdf6ec;
}

.locker-reserved {
  border-color: #909399;
  background-color: #f4f4f5;
}

.locker-highlighted {
  border-color: #409eff !important;
  background-color: #ecf5ff !important;
  box-shadow: 0 0 10px rgba(64, 158, 255, 0.3) !important;
}

.locker-id {
  font-weight: 600;
  font-size: 1rem;
  margin-bottom: 0.4rem;
  color: #303133;
}

.locker-status {
  margin-bottom: 0.4rem;
}

.locker-info {
  font-size: 0.75rem;
  color: #606266;
  line-height: 1.2;
}

.task-id {
  margin-bottom: 0.25rem;
}

.security-level {
  margin-top: 0.25rem;
}

.locker-details {
  /* padding: 1rem 0; */
  /* width: 50%; */
}
</style>
