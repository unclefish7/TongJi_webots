<template>
  <div class="call-robot-container">
    <el-page-header @back="$router.back()">
      <template #content>
        <span class="page-title">呼叫机器人</span>
      </template>
    </el-page-header>

    <div class="call-content">
      <div class="content-layout">
        <!-- 左侧呼叫表单 -->
        <div class="form-section">
          <el-card class="call-form-card">
            <template #header>
              <div class="card-header">
                <el-icon><Phone /></el-icon>
                <span>呼叫信息</span>
              </div>
            </template>

            <el-form :model="callForm" :rules="callRules" ref="callFormRef" label-width="7vw">
              <!-- 当前用户信息 -->
              <el-form-item label="呼叫用户">
                <div class="current-user-info">
                  <div v-if="userStore.isAuthenticated" class="user-display">
                    <el-avatar :size="24">
                      {{ userStore.currentUser?.name?.charAt(0) }}
                    </el-avatar>
                    <div class="user-details">
                      <span class="user-name">{{ userStore.currentUser?.name }}</span>
                      <el-tag :type="userStore.getAuthLevelType(userStore.currentUser?.auth_level || '')" size="small">
                        {{ userStore.currentUser?.auth_level }}
                      </el-tag>
                    </div>
                  </div>
                  <div v-else class="no-user">
                    <el-alert
                      title="未认证用户"
                      description="请返回主页进行身份认证"
                      type="warning"
                      :closable="false"
                    />
                    <el-button @click="$router.push('/')" type="primary" size="small">
                      返回主页认证
                    </el-button>
                  </div>
                </div>
              </el-form-item>

              <!-- 选择机器人 -->
              <el-form-item label="选择机器人" prop="robotId">
                <el-select
                  v-model="callForm.robotId"
                  placeholder="请选择可用的机器人"
                  style="width: 100%"
                >
                  <el-option
                    v-for="robot in robotStore.availableRobots"
                    :key="robot.id"
                    :label="robot.name"
                    :value="robot.id"
                  >
                    <div class="robot-option">
                      <div class="robot-option-info">
                        <span>{{ robot.name }}</span>
                        <el-tag :type="getStatusType(robot.status)" size="small">
                          {{ getStatusText(robot.status) }}
                        </el-tag>
                      </div>
                      <div class="robot-option-details">
                        <span>位置: ({{ robot.position.x }}, {{ robot.position.y }})</span>
                        <span>电量: {{ robot.battery }}%</span>
                      </div>
                    </div>
                  </el-option>
                </el-select>
              </el-form-item>              <!-- 呼叫位置 -->
              <el-form-item label="呼叫位置" prop="location">
                <el-select
                  v-model="callForm.location"
                  placeholder="请选择呼叫位置"
                  style="width: 100%"
                >
                  <el-option
                    v-for="location in locationOptions"
                    :key="location.value"
                    :label="location.label"
                    :value="location.value"
                  />
                </el-select>
              </el-form-item>

              <!-- 呼叫类型 -->
              <el-form-item label="呼叫类型" prop="priority">
                <el-radio-group v-model="callForm.priority" class="priority-radio-group">
                  <el-radio value="normal" class="priority-radio">
                    <div class="priority-option">
                      <div class="priority-title">普通呼叫</div>
                      <div class="priority-desc">标准响应时间，适用于一般需求</div>
                    </div>
                  </el-radio>
                  <el-radio value="urgent" class="priority-radio">
                    <div class="priority-option">
                      <div class="priority-title">紧急呼叫</div>
                      <div class="priority-desc">优先响应，适用于紧急情况</div>
                    </div>
                  </el-radio>
                </el-radio-group>
              </el-form-item>

              <!-- 备注信息 -->
              <el-form-item label="备注信息">
                <el-input
                  v-model="callForm.notes"
                  type="textarea"
                  :rows="3"
                  placeholder="请输入备注信息（可选）"
                />
              </el-form-item>

              <!-- 操作按钮 -->
              <el-form-item class="submit-section">
                <el-button
                  type="primary"
                  @click="submitCall"
                  :loading="isSubmitting"
                  size="large"
                  class="submit-btn"
                >
                  <el-icon><Bell /></el-icon>
                  发起呼叫
                </el-button>
                <el-button @click="resetForm" size="large">重置</el-button>
              </el-form-item>
            </el-form>
          </el-card>
        </div>

        <!-- 右侧状态面板 -->
        <div class="status-section">
          <!-- 机器人状态 -->
          <el-card class="status-card">
            <template #header>
              <div class="card-header">
                <el-icon><User /></el-icon>
                <span>机器人状态</span>
              </div>
            </template>
            <div class="robot-status-list">
              <div v-for="robot in robotStore.robots" :key="robot.id" class="robot-status-item">
                <div class="robot-info">
                  <div class="robot-name">{{ robot.name }}</div>
                  <el-tag :type="getStatusType(robot.status)" size="small">
                    {{ getStatusText(robot.status) }}
                  </el-tag>
                </div>
                <div class="robot-position">
                  位置: ({{ robot.position.x }}, {{ robot.position.y }})
                </div>
                <div class="robot-battery">
                  <el-progress
                    :percentage="robot.battery"
                    :stroke-width="6"
                    :color="getBatteryColor(robot.battery)"
                  />
                </div>
                <div v-if="robot.currentTask" class="robot-task">
                  {{ robot.currentTask }}
                </div>
              </div>
            </div>
          </el-card>

          <!-- 柜门状态 -->
          <el-card class="status-card">
            <template #header>
              <div class="card-header">
                <el-icon><Grid /></el-icon>
                <span>柜门状态</span>
              </div>
            </template>
            <div class="compartment-grid">
              <div
                v-for="compartment in robotStore.compartments"
                :key="compartment.id"
                class="compartment-item"
                :class="{ occupied: compartment.isOccupied }"
              >
                <div class="compartment-number">
                  {{ compartment.floor }}F-{{ compartment.compartmentNumber }}
                </div>
                <div class="compartment-level">
                  <el-tag :type="getLevelType(compartment.securityLevel)" size="small">
                    {{ compartment.securityLevel }}
                  </el-tag>
                </div>
                <div v-if="compartment.isOccupied" class="compartment-content">
                  {{ compartment.content }}
                </div>
                <div v-else class="compartment-status-text">空闲</div>
              </div>
            </div>
          </el-card>
        </div>
      </div>
    </div>

    <!-- 呼叫成功对话框 -->
    <el-dialog
      v-model="showSuccessDialog"
      title="呼叫成功"
      width="50vw"
      :before-close="handleSuccessClose"
    >
      <el-result icon="success" title="呼叫发送成功">
        <template #sub-title>
          <p>
            机器人 <strong>{{ selectedRobotName }}</strong> 已接收到您的呼叫
          </p>
          <p>
            预计到达时间: <strong>{{ estimatedArrival }}</strong>
          </p>
        </template>
        <template #extra>
          <el-button type="primary" @click="handleSuccessClose">确定</el-button>
          <el-button @click="callAnother">再次呼叫</el-button>
        </template>
      </el-result>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, reactive, onMounted } from 'vue'
import { useRouter } from 'vue-router'
import { ElMessage } from 'element-plus'
import { useRobotStore } from '@/stores/robot'
import { useUserStore } from '@/stores/user'
import { locationApiService } from '@/services/locationApiService'
import { taskApiService } from '@/services/taskApiService'
import { Phone, Bell, User, Grid } from '@element-plus/icons-vue'

const router = useRouter()
const robotStore = useRobotStore()
const userStore = useUserStore()

// 表单数据
const callForm = reactive({
  robotId: '',
  location: '', // 改为字符串类型
  priority: 'normal',
  notes: '',
})

// 表单验证规则
const callRules = {
  robotId: [{ required: true, message: '请选择机器人', trigger: 'change' }],
  location: [{ required: true, message: '请选择呼叫位置', trigger: 'change' }],
  priority: [{ required: true, message: '请选择呼叫类型', trigger: 'change' }],
}

// 位置选项（从地图服务获取）
const locationOptions = ref<any[]>([])

// 初始化地点数据
onMounted(async () => {
  try {
    const locations = await locationApiService.getAllLocations()
    locationOptions.value = locations.map((location: any) => ({
      value: location.location_id,  // 使用 location_id 作为值
      label: location.label,        // 使用 label 作为显示文本
      location: location
    }))
  } catch (error) {
    console.error('获取地点数据失败:', error)
    ElMessage.error('获取地点数据失败')
  }
})

// 状态
const callFormRef = ref()
const isSubmitting = ref(false)
const showSuccessDialog = ref(false)
const selectedRobotName = ref('')
const estimatedArrival = ref('')

// 计算属性
const locationText = computed(() => {
  if (callForm.location.length === 2) {
    return `${callForm.location[0]} ${callForm.location[1]}`
  }
  return ''
})

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

const getLevelType = (level: string) => {
  const types = {
    L1: 'success',
    L2: 'warning',
    L3: 'danger',
  }
  return types[level as keyof typeof types] || 'info'
}

const submitCall = async () => {
  if (!callFormRef.value) return

  try {
    await callFormRef.value.validate()
    isSubmitting.value = true

    // 检查用户认证状态
    if (!userStore.isAuthenticated) {
      ElMessage.error('请先进行身份认证')
      router.push('/')
      return
    }

    const user = userStore.currentUser
    if (!user) {
      ElMessage.error('用户信息异常')
      router.push('/')
      return
    }

    // 获取选中的机器人信息
    const selectedRobot = robotStore.robots.find((r) => r.id === callForm.robotId)
    if (!selectedRobot) {
      ElMessage.error('所选机器人不存在')
      return
    }

    selectedRobotName.value = selectedRobot.name

    // 创建呼叫任务
    try {
      const taskRequest = {
        user_id: user.user_id,
        receiver: user.user_id, // 呼叫任务的接收人就是发起人
        location_id: callForm.location,
        security_level: 'L1' as 'L1' | 'L2' | 'L3', // 呼叫任务默认L1级别
        task_type: 'call' as 'call' | 'send', // 呼叫任务类型
        description: `${callForm.priority === 'urgent' ? '紧急' : '普通'}呼叫机器人到${callForm.location}${callForm.notes ? ` - ${callForm.notes}` : ''}`
      }

      const result = await taskApiService.createTask(taskRequest)
      
      if (!result.success) {
        throw new Error(result.message)
      }

      // 计算预计到达时间
      const arrivalMinutes = callForm.priority === 'urgent' ? 3 : 5
      const arrivalTime = new Date()
      arrivalTime.setMinutes(arrivalTime.getMinutes() + arrivalMinutes)
      estimatedArrival.value = arrivalTime.toLocaleTimeString('zh-CN', {
        hour: '2-digit',
        minute: '2-digit',
      })

      showSuccessDialog.value = true
      ElMessage.success('呼叫任务创建成功')
    } catch (taskError) {
      console.error('创建呼叫任务失败:', taskError)
      ElMessage.error('创建呼叫任务失败，请重试')
      return
    }
  } catch (error) {
    console.error('呼叫失败:', error)
    ElMessage.error('呼叫失败，请重试')
  } finally {
    isSubmitting.value = false
  }
}

const resetForm = () => {
  if (callFormRef.value) {
    callFormRef.value.resetFields()
  }
}

const handleSuccessClose = () => {
  showSuccessDialog.value = false
  router.push('/')
}

const callAnother = () => {
  showSuccessDialog.value = false
  resetForm()
}
</script>

<style scoped>
.call-robot-container {
  padding: 2vw;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  min-height: 100vh;
  width: 100vw;
  position: relative;
}

.page-title {
  font-size: 1.2vw;
  font-weight: 600;
  color: white;
  text-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
}

.call-content {
  margin-top: 2vh;
}

.content-layout {
  display: grid;
  grid-template-columns: 1fr 22vw;
  gap: 1.5vw;
  max-width: 100vw;
  margin: 0 auto;
}

.form-section {
  grid-column: 1;
}

.status-section {
  grid-column: 2;
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
}

.call-form-card,
.status-card {
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  border: none;
  border-radius: 16px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;
  animation: slideInUp 0.6s ease-out;
}

.call-form-card:hover,
.status-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15);
}

.card-header {
  display: flex;
  align-items: center;
  gap: 0.8vw;
  font-weight: 600;
  font-size: 0.9vw;
  background: linear-gradient(45deg, #667eea, #764ba2);
  background-clip: text;
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}

.card-header .el-icon {
  color: #667eea;
  font-size: 1.1vw;
}

/* 表单样式 */
.priority-radio-group {
  width: 100%;
}

.priority-radio {
  width: 100%;
  margin-bottom: 1.5vh;
  border: 2px solid #e4e7ed;
  border-radius: 12px;
  padding: 2vh 1.2vw;
  transition: all 0.3s ease;
  background: linear-gradient(135deg, #f8f9ff 0%, #ffffff 100%);
  min-height: 8vh;
}

.priority-radio:hover {
  border-color: #667eea;
  transform: translateX(8px);
  box-shadow: 0 4px 16px rgba(102, 126, 234, 0.2);
}

.priority-radio.is-checked {
  border-color: #667eea;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

.priority-radio.is-checked .priority-option {
  color: white;
}

.priority-option {
  margin-left: 1vw;
  transition: color 0.3s ease;
}

.priority-title {
  font-weight: 600;
  font-size: 0.9vw;
  margin-bottom: 0.5vh;
}

.priority-desc {
  font-size: 0.7vw;
  opacity: 0.8;
}

.submit-section {
  margin-top: 3vh;
  text-align: center;
}

.submit-btn {
  min-width: 10vw;
  height: 4vh;
  font-size: 0.9vw;
  border-radius: 25px;
  background: linear-gradient(45deg, #667eea, #764ba2);
  border: none;
  transition: all 0.3s ease;
}

.submit-btn:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 25px rgba(102, 126, 234, 0.4);
}

/* 机器人选择器样式 */
.robot-option {
  display: flex;
  flex-direction: column;
  gap: 0.5vh;
  padding: 1vh 0;
}

.robot-option-info {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-weight: 600;
  font-size: 0.8vw;
}

.robot-option-details {
  display: flex;
  justify-content: space-between;
  font-size: 0.7vw;
  color: #606266;
}

/* 机器人状态列表 */
.robot-status-list {
  display: flex;
  flex-direction: column;
  gap: 1vh;
  max-height: 25vh;
  overflow-y: auto;
}

.robot-status-item {
  padding: 1.5vh 1vw;
  border: 2px solid #e4e7ed;
  border-radius: 12px;
  background: linear-gradient(135deg, #f0f9ff 0%, #ffffff 100%);
  transition: all 0.3s ease;
  animation: fadeInScale 0.5s ease-out;
}

.robot-status-item:hover {
  transform: scale(1.02);
  border-color: #667eea;
  box-shadow: 0 4px 16px rgba(102, 126, 234, 0.2);
}

.robot-info {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1vh;
}

.robot-name {
  font-weight: 600;
  font-size: 0.8vw;
  color: #303133;
}

.robot-position {
  font-size: 0.7vw;
  color: #606266;
  margin-bottom: 1vh;
}

.robot-battery {
  margin-bottom: 1vh;
}

.robot-task {
  font-size: 0.7vw;
  color: #909399;
  font-style: italic;
}

/* 柜门状态样式 */
.compartment-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1vw;
  max-height: 30vh;
  overflow-y: auto;
}

.compartment-item {
  padding: 1.5vh 1vw;
  border: 2px solid #e4e7ed;
  border-radius: 12px;
  background: linear-gradient(135deg, #f0f9ff 0%, #ffffff 100%);
  text-align: center;
  transition: all 0.3s ease;
  animation: fadeInScale 0.5s ease-out;
}

.compartment-item:hover {
  transform: scale(1.05);
}

.compartment-item.occupied {
  background: linear-gradient(135deg, #fef0f0 0%, #fee2e2 100%);
  border-color: #f56c6c;
}

.compartment-number {
  font-weight: 600;
  font-size: 0.7vw;
  margin-bottom: 0.5vh;
}

.compartment-level {
  margin-bottom: 0.5vh;
}

.compartment-content,
.compartment-status-text {
  font-size: 0.6vw;
  color: #606266;
}

/* 动画效果 */
@keyframes slideInUp {
  from {
    opacity: 0;
    transform: translateY(30px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

@keyframes fadeInScale {
  from {
    opacity: 0;
    transform: scale(0.9);
  }
  to {
    opacity: 1;
    transform: scale(1);
  }
}

/* 响应式设计 */
@media (max-width: 1200px) {
  .content-layout {
    grid-template-columns: 1fr;
    gap: 3vw;
  }

  .status-section {
    grid-column: 1;
    flex-direction: row;
    gap: 3vw;
  }

  .page-title {
    font-size: 3vw;
  }

  .card-header {
    font-size: 2vw;
  }

  .priority-title {
    font-size: 1.8vw;
  }

  .priority-desc {
    font-size: 1.4vw;
  }

  .submit-btn {
    min-width: 20vw;
    font-size: 2vw;
  }
}

@media (max-width: 768px) {
  .call-robot-container {
    padding: 3vw;
  }

  .page-title {
    font-size: 4vw;
  }

  .content-layout {
    grid-template-columns: 1fr;
    gap: 4vw;
  }

  .status-section {
    flex-direction: column;
    gap: 2vh;
  }

  .compartment-grid {
    grid-template-columns: 1fr;
  }

  .submit-btn {
    min-width: 30vw;
    font-size: 3vw;
    height: 6vh;
  }
}

/* Element Plus 组件覆盖样式 */
:deep(.el-form-item__label) {
  font-weight: 600;
  color: #333;
  font-size: 0.8vw;
}

:deep(.el-button) {
  border-radius: 8px;
  transition: all 0.3s ease;
}

:deep(.el-button:hover) {
  transform: translateY(-1px);
}

:deep(.el-card__header) {
  background: linear-gradient(135deg, #f8f9ff 0%, #ffffff 100%);
  border-bottom: 1px solid rgba(102, 126, 234, 0.1);
}

:deep(.el-select-dropdown__item) {
  padding: 1vh 0;
}

:deep(.el-tag) {
  border-radius: 12px;
  font-weight: 500;
}

:deep(.el-radio__input.is-checked .el-radio__inner) {
  background-color: white;
  border-color: white;
}

:deep(.el-radio__input.is-checked + .el-radio__label) {
  color: white;
}

:deep(.el-progress-bar__outer) {
  border-radius: 10px;
}

:deep(.el-progress-bar__inner) {
  border-radius: 10px;
}

/* 滚动条样式 */
.robot-status-list::-webkit-scrollbar,
.compartment-grid::-webkit-scrollbar {
  width: 6px;
}

.robot-status-list::-webkit-scrollbar-track,
.compartment-grid::-webkit-scrollbar-track {
  background: rgba(0, 0, 0, 0.1);
  border-radius: 3px;
}

.robot-status-list::-webkit-scrollbar-thumb,
.compartment-grid::-webkit-scrollbar-thumb {
  background: rgba(102, 126, 234, 0.5);
  border-radius: 3px;
}

.robot-status-list::-webkit-scrollbar-thumb:hover,
.compartment-grid::-webkit-scrollbar-thumb:hover {
  background: rgba(102, 126, 234, 0.7);
}

.current-user-info {
  background: rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  padding: 1vh;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.2);
}

.user-display {
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
  font-weight: 600;
  color: #333;
  font-size: 0.9rem;
}

.no-user {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}
</style>
