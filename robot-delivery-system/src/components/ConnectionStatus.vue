<template>
  <el-card class="connection-status-card" :class="{ 'connected': isConnected, 'disconnected': !isConnected }">
    <template #header>
      <div class="status-header">
        <el-icon><Monitor /></el-icon>
        <span>API连接状态</span>
        <el-button @click="testConnection" :loading="isTesting" size="small" type="primary">
          刷新
        </el-button>
      </div>
    </template>
    <div class="status-content">
      <div class="status-indicator">
        <el-icon class="status-icon" :class="{ 'online': isConnected, 'offline': !isConnected }">
          <CircleCheck v-if="isConnected" />
          <CircleClose v-else />
        </el-icon>
        <span class="status-text">{{ isConnected ? 'API服务正常' : 'API服务离线' }}</span>
      </div>
      <div class="status-details">
        <p>后端地址: {{ apiUrl }}</p>
        <p>最后检查: {{ lastChecked }}</p>
      </div>
    </div>
  </el-card>
</template>

<script setup lang="ts">
import { ref, onMounted } from 'vue'
import { ElMessage } from 'element-plus'
import { useRobotStore } from '@/stores/robot'
import { API_BASE_URL } from '@/services/api'
import { Monitor, CircleCheck, CircleClose } from '@element-plus/icons-vue'

const robotStore = useRobotStore()

const isConnected = ref(false)
const isTesting = ref(false)
const lastChecked = ref('')
const apiUrl = ref(API_BASE_URL)

const testConnection = async () => {
  isTesting.value = true
  try {
    const result = await robotStore.testApiConnection()
    isConnected.value = result
    lastChecked.value = new Date().toLocaleString('zh-CN')
    
    // 删除烦人的连接成功提示，只在连接失败时提示
    if (!result) {
      ElMessage.error('API连接失败')
    }
  } catch (error) {
    isConnected.value = false
    lastChecked.value = new Date().toLocaleString('zh-CN')
    ElMessage.error('API连接测试失败')
  } finally {
    isTesting.value = false
  }
}

onMounted(() => {
  testConnection()
  // 每1秒自动检查一次连接
  setInterval(testConnection, 1000)
})
</script>

<style scoped>
.connection-status-card {
  transition: all 0.3s ease;
  border-radius: 12px;
  margin-bottom: 1vh;
}

.connection-status-card.connected {
  border-left: 4px solid #67c23a;
  background: linear-gradient(135deg, #f0f9ff 0%, #ecfdf5 100%);
}

.connection-status-card.disconnected {
  border-left: 4px solid #f56c6c;
  background: linear-gradient(135deg, #fef2f2 0%, #fff1f1 100%);
}

.status-header {
  display: flex;
  align-items: center;
  gap: 0.8vw;
  justify-content: space-between;
  font-weight: 600;
  font-size: 0.9vw;
}

.status-content {
  padding: 1vh 0;
}

.status-indicator {
  display: flex;
  align-items: center;
  gap: 1vw;
  margin-bottom: 1vh;
}

.status-icon {
  font-size: 1.2vw;
  transition: all 0.3s ease;
}

.status-icon.online {
  color: #67c23a;
}

.status-icon.offline {
  color: #f56c6c;
}

.status-text {
  font-weight: 600;
  font-size: 0.9vw;
}

.status-details {
  font-size: 0.7vw;
  color: #606266;
  line-height: 1.5;
}

.status-details p {
  margin: 0.5vh 0;
}
</style>