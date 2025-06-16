<template>
  <div class="api-test-container">
    <el-page-header @back="$router.back()">
      <template #content>
        <span class="page-title">API连接测试</span>
      </template>
    </el-page-header>

    <div class="test-content">
      <el-row :gutter="20">
        <!-- 基础API测试 -->
        <el-col :span="12">
          <el-card class="test-card">
            <template #header>
              <div class="card-header">
                <span>基础API测试</span>
                <el-button @click="runBasicTests" type="primary" size="small" :loading="testing.basic">
                  运行测试
                </el-button>
              </div>
            </template>
            
            <div class="test-results">
              <div v-for="(test, index) in basicTests" :key="index" class="test-item">
                <div class="test-name">{{ test.name }}</div>
                <el-tag :type="getTestResultType(test.status)" size="small">
                  {{ test.status }}
                </el-tag>
                <div v-if="test.error" class="test-error">{{ test.error }}</div>
              </div>
            </div>
          </el-card>
        </el-col>

        <!-- 任务流程测试 -->
        <el-col :span="12">
          <el-card class="test-card">
            <template #header>
              <div class="card-header">
                <span>任务流程测试</span>
                <el-button @click="runTaskFlowTests" type="primary" size="small" :loading="testing.taskFlow">
                  运行测试
                </el-button>
              </div>
            </template>
            
            <div class="test-results">
              <div v-for="(test, index) in taskFlowTests" :key="index" class="test-item">
                <div class="test-name">{{ test.name }}</div>
                <el-tag :type="getTestResultType(test.status)" size="small">
                  {{ test.status }}
                </el-tag>
                <div v-if="test.error" class="test-error">{{ test.error }}</div>
                <div v-if="test.data" class="test-data">
                  <pre>{{ JSON.stringify(test.data, null, 2) }}</pre>
                </div>
              </div>
            </div>
          </el-card>
        </el-col>
      </el-row>

      <!-- 实时状态监控 -->
      <el-row style="margin-top: 20px;">
        <el-col :span="24">
          <el-card class="status-card">
            <template #header>
              <div class="card-header">
                <span>实时状态监控</span>
                <el-button @click="startStatusMonitoring" type="success" size="small" :loading="monitoring">
                  {{ monitoring ? '监控中...' : '开始监控' }}
                </el-button>
              </div>
            </template>
            
            <div class="status-content">
              <el-row :gutter="20">
                <el-col :span="8">
                  <div class="status-section">
                    <h4>队列状态</h4>
                    <div v-if="liveStatus.queue">
                      <p>总任务: {{ liveStatus.queue.total_tasks }}</p>
                      <p>等待任务: {{ liveStatus.queue.pending_tasks }}</p>
                      <p>执行中: {{ liveStatus.queue.executing_tasks }}</p>
                    </div>
                  </div>
                </el-col>
                
                <el-col :span="8">
                  <div class="status-section">
                    <h4>ROS2状态</h4>
                    <div v-if="liveStatus.ros2">
                      <p>可用: {{ liveStatus.ros2.available ? '是' : '否' }}</p>
                      <p>状态: {{ liveStatus.ros2.status }}</p>
                    </div>
                  </div>
                </el-col>
                
                <el-col :span="8">
                  <div class="status-section">
                    <h4>系统状态</h4>
                    <p>更新时间: {{ liveStatus.lastUpdate }}</p>
                    <p>连接状态: <el-tag :type="liveStatus.connected ? 'success' : 'danger'" size="small">
                      {{ liveStatus.connected ? '已连接' : '断开连接' }}
                    </el-tag></p>
                  </div>
                </el-col>
              </el-row>
            </div>
          </el-card>
        </el-col>
      </el-row>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, reactive, onUnmounted } from 'vue'
import { useRouter } from 'vue-router'
import { ElMessage } from 'element-plus'
import { taskApiService } from '@/services/taskApiService'
import { authApiService } from '@/services/authApiService'
import { pickupApiService } from '@/services/pickupApiService'
import { userApiService } from '@/services/userApiService'

const router = useRouter()

// 测试状态
const testing = reactive({
  basic: false,
  taskFlow: false
})

const monitoring = ref(false)
let monitoringInterval: number | null = null

// 测试结果
const basicTests = ref<any[]>([])
const taskFlowTests = ref<any[]>([])

// 实时状态
const liveStatus = reactive({
  queue: null,
  ros2: null,
  connected: false,
  lastUpdate: ''
})

// 基础API测试
const runBasicTests = async () => {
  testing.basic = true
  basicTests.value = []
  
  const tests = [
    {
      name: 'Ping测试',
      run: () => taskApiService.ping()
    },
    {
      name: '获取用户列表',
      run: () => userApiService.getAllUsers()
    },
    {
      name: '获取队列状态',
      run: () => taskApiService.getQueueStatus()
    },
    {
      name: '检查ROS2状态',
      run: () => taskApiService.checkROS2Status()
    },
    {
      name: '认证缓存状态',
      run: () => authApiService.getCacheStatus()
    }
  ]
  
  for (const test of tests) {
    const testResult = {
      name: test.name,
      status: 'running',
      error: null,
      data: null
    }
    
    basicTests.value.push(testResult)
    
    try {
      const result = await test.run()
      testResult.status = 'success'
      testResult.data = result
    } catch (error: any) {
      testResult.status = 'failed'
      testResult.error = error.message || '未知错误'
    }
  }
  
  testing.basic = false
}

// 任务流程测试
const runTaskFlowTests = async () => {
  testing.taskFlow = true
  taskFlowTests.value = []
  
  let createdTaskId: string | null = null
  
  const tests = [
    {
      name: '创建测试任务',
      run: async () => {
        const result = await taskApiService.createTask({
          user_id: 'E001',
          receiver: 'E002',
          location_id: 'A101',
          security_level: 'L1',
          description: 'API测试任务'
        })
        if (result.success && result.task_id) {
          createdTaskId = result.task_id
        }
        return result
      }
    },
    {
      name: '启动任务',
      run: () => taskApiService.startTask()
    },
    {
      name: '获取更新后的队列状态',
      run: () => taskApiService.getQueueStatus()
    },
    {
      name: '取消任务',
      run: () => createdTaskId ? taskApiService.cancelTask(createdTaskId) : Promise.reject('没有任务ID')
    }
  ]
  
  for (const test of tests) {
    const testResult = {
      name: test.name,
      status: 'running',
      error: null,
      data: null
    }
    
    taskFlowTests.value.push(testResult)
    
    try {
      const result = await test.run()
      testResult.status = 'success'
      testResult.data = result
    } catch (error: any) {
      testResult.status = 'failed'
      testResult.error = error.message || '未知错误'
    }
    
    // 在测试步骤之间添加延迟
    await new Promise(resolve => setTimeout(resolve, 1000))
  }
  
  testing.taskFlow = false
}

// 开始状态监控
const startStatusMonitoring = () => {
  if (monitoring.value) {
    stopStatusMonitoring()
    return
  }
  
  monitoring.value = true
  
  const updateStatus = async () => {
    try {
      // 获取队列状态
      const queueResult = await taskApiService.getQueueStatus()
      if (queueResult.success) {
        liveStatus.queue = queueResult.data
      }
      
      // 获取ROS2状态
      const ros2Result = await taskApiService.checkROS2Status()
      if (ros2Result.success) {
        liveStatus.ros2 = ros2Result.data
      }
      
      liveStatus.connected = true
      liveStatus.lastUpdate = new Date().toLocaleString()
      
    } catch (error) {
      liveStatus.connected = false
      console.error('状态更新失败:', error)
    }
  }
  
  // 立即更新一次
  updateStatus()
  
  // 每3秒更新一次
  monitoringInterval = window.setInterval(updateStatus, 3000)
}

// 停止状态监控
const stopStatusMonitoring = () => {
  monitoring.value = false
  if (monitoringInterval) {
    clearInterval(monitoringInterval)
    monitoringInterval = null
  }
}

// 获取测试结果类型
const getTestResultType = (status: string) => {
  switch (status) {
    case 'success': return 'success'
    case 'failed': return 'danger'
    case 'running': return 'warning'
    default: return 'info'
  }
}

// 组件卸载时清理
onUnmounted(() => {
  stopStatusMonitoring()
})
</script>

<style scoped>
.api-test-container {
  padding: 20px;
  max-width: 1200px;
  margin: 0 auto;
}

.page-title {
  font-size: 1.5rem;
  font-weight: 600;
}

.test-content {
  margin-top: 20px;
}

.test-card, .status-card {
  height: 100%;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-weight: 600;
}

.test-results {
  max-height: 400px;
  overflow-y: auto;
}

.test-item {
  padding: 10px 0;
  border-bottom: 1px solid #eee;
}

.test-item:last-child {
  border-bottom: none;
}

.test-name {
  font-weight: 500;
  margin-bottom: 5px;
}

.test-error {
  color: #f56c6c;
  font-size: 0.9rem;
  margin-top: 5px;
}

.test-data {
  margin-top: 10px;
  max-height: 200px;
  overflow-y: auto;
}

.test-data pre {
  background: #f5f5f5;
  padding: 10px;
  border-radius: 4px;
  font-size: 0.8rem;
}

.status-content {
  padding: 10px 0;
}

.status-section h4 {
  margin: 0 0 10px 0;
  color: #303133;
}

.status-section p {
  margin: 5px 0;
  color: #606266;
}
</style>
