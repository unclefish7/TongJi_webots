<template>
  <div class="send-package-container">
    <el-page-header @back="$router.back()">
      <template #content>
        <span class="page-title">寄送包裹</span>
      </template>
    </el-page-header>

    <div class="send-content">
      <div class="content-layout">
        <!-- 主要表单区域 -->
        <div class="form-section">
          <el-card class="send-form-card">
            <template #header>
              <div class="card-header">
                <el-icon><Box /></el-icon>
                <span>包裹信息</span>
              </div>
            </template>

            <el-form :model="sendForm" :rules="sendRules" ref="sendFormRef" label-width="8vw">
              <!-- 身份认证 -->
              <el-form-item label="身份认证" prop="authenticated">
                <div class="auth-section">
                  <div v-if="!isAuthenticated" class="auth-required">
                    <el-alert
                      title="需要身份认证"
                      :description="`寄送${sendForm.securityLevel}级别包裹需要进行身份认证`"
                      type="warning"
                      :closable="false"
                    />
                    <el-button type="primary" @click="showAuthModal = true" style="margin-top: 1vh">
                      <el-icon><Lock /></el-icon>
                      进行身份认证
                    </el-button>
                  </div>
                  <div v-else class="auth-success">
                    <el-alert
                      title="认证成功"
                      :description="`已通过${userAuthLevel}级别身份认证`"
                      type="success"
                      :closable="false"
                    />
                    <el-button
                      type="info"
                      @click="showAuthModal = true"
                      size="small"
                      style="margin-top: 1vh"
                    >
                      重新认证
                    </el-button>
                  </div>
                </div>
              </el-form-item>

              <!-- 安全等级选择 -->
              <el-form-item label="安全等级" prop="securityLevel">
                <el-radio-group
                  v-model="sendForm.securityLevel"
                  @change="onSecurityLevelChange"
                  class="security-radio-group"
                >
                  <el-radio value="L1" class="security-radio">
                    <div class="security-option">
                      <div class="security-title">L1 - 基础安全</div>
                      <div class="security-desc">普通包裹，基础安全保护</div>
                      <div class="security-features">
                        <el-tag size="small" type="success">基础加密</el-tag>
                        <el-tag size="small" type="info">标准配送</el-tag>
                      </div>
                    </div>
                  </el-radio>
                  <el-radio value="L2" class="security-radio">
                    <div class="security-option">
                      <div class="security-title">L2 - 中等安全</div>
                      <div class="security-desc">重要包裹，增强安全保护</div>
                      <div class="security-features">
                        <el-tag size="small" type="warning">强化加密</el-tag>
                        <el-tag size="small" type="warning">身份验证</el-tag>
                        <el-tag size="small" type="info">跟踪记录</el-tag>
                      </div>
                    </div>
                  </el-radio>
                  <el-radio value="L3" class="security-radio">
                    <div class="security-option">
                      <div class="security-title">L3 - 高级安全</div>
                      <div class="security-desc">机密包裹，最高安全保护</div>
                      <div class="security-features">
                        <el-tag size="small" type="danger">军用加密</el-tag>
                        <el-tag size="small" type="danger">多重认证</el-tag>
                        <el-tag size="small" type="warning">全程监控</el-tag>
                        <el-tag size="small" type="info">专用配送</el-tag>
                      </div>
                    </div>
                  </el-radio>
                </el-radio-group>
              </el-form-item>              <!-- 收件人选择 -->
              <el-form-item label="收件人" prop="receiver">
                <el-select
                  v-model="sendForm.receiver"
                  placeholder="请选择收件人"
                  style="width: 100%"
                  filterable
                  loading-text="加载中..."
                  :loading="isLoadingUsers"
                >
                  <el-option
                    v-for="user in userOptions"
                    :key="user.value"
                    :label="user.label"
                    :value="user.value"
                  >
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                      <span>{{ user.label }}</span>
                      <el-tag size="small" :type="getLevelType(user.authLevel)">{{ user.authLevel }}</el-tag>
                    </div>
                  </el-option>
                </el-select>
              </el-form-item>

              <!-- 收件地址 -->
              <el-form-item label="收件地址" prop="destination">
                <el-select
                  v-model="sendForm.destination"
                  placeholder="请选择收件地址"
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

              <!-- L3级别收件人信息 -->
              <el-form-item
                v-if="sendForm.securityLevel === 'L3'"
                label="收件人信息"
                prop="recipientInfo"
                class="recipient-form"
              >
                <div class="recipient-inputs">
                  <div class="recipient-row">
                    <el-input
                      v-model="sendForm.recipientInfo.name"
                      placeholder="收件人姓名"
                      style="margin-bottom: 1vh"
                    />
                    <el-input
                      v-model="sendForm.recipientInfo.phone"
                      placeholder="联系电话"
                      style="margin-bottom: 1vh"
                    />
                  </div>
                  <el-input v-model="sendForm.recipientInfo.idNumber" placeholder="身份证号码" />
                </div>
              </el-form-item>

              <!-- 包裹描述 -->
              <el-form-item label="包裹描述" prop="description">
                <el-input
                  v-model="sendForm.description"
                  type="textarea"
                  :rows="3"
                  placeholder="请描述包裹内容（避免敏感信息）"
                />
              </el-form-item>

              <!-- 操作按钮 -->
              <el-form-item class="submit-section">
                <el-button
                  type="primary"
                  @click="submitSend"
                  :loading="isSubmitting"
                  :disabled="!canSubmit"
                  size="large"
                  class="submit-btn"
                >
                  <el-icon><Upload /></el-icon>
                  确认寄送
                </el-button>
                <el-button @click="resetForm" size="large">重置</el-button>
              </el-form-item>
            </el-form>
          </el-card>
        </div>

        <!-- 右侧状态面板 -->
        <div class="status-section">
          <!-- 柜门状态 -->
          <el-card class="status-card">
            <template #header>
              <div class="card-header">
                <el-icon><Grid /></el-icon>
                <span>柜门状态</span>
              </div>
            </template>
            <div class="compartment-status">
              <div class="status-summary">
                <el-descriptions :column="1" size="small">
                  <el-descriptions-item label="总柜门数">{{
                    robotStore.compartments.length
                  }}</el-descriptions-item>
                  <el-descriptions-item label="可用柜门">{{
                    availableCompartments.length
                  }}</el-descriptions-item>
                  <el-descriptions-item label="已占用">{{
                    occupiedCompartments.length
                  }}</el-descriptions-item>
                </el-descriptions>
              </div>

              <el-divider />

              <div class="compartment-grid">
                <div
                  v-for="compartment in robotStore.compartments"
                  :key="compartment.id"
                  class="compartment-item"
                  :class="{
                    occupied: compartment.isOccupied,
                    available:
                      !compartment.isOccupied &&
                      compartment.securityLevel === sendForm.securityLevel,
                    selected: selectedCompartment?.id === compartment.id,
                  }"
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
            </div>
          </el-card>
        </div>

        <!-- 左下角：安全等级说明 -->
        <div class="security-info-section">
          <el-card class="info-card">
            <template #header>
              <div class="card-header">
                <el-icon><InfoFilled /></el-icon>
                <span>安全等级说明</span>
              </div>
            </template>
            <div class="security-info">
              <div class="security-item">
                <el-tag type="success" size="small">L1</el-tag>
                <div class="security-description">
                  <p><strong>基础安全等级</strong></p>
                  <ul>
                    <li>适用于普通文件、日常用品</li>
                    <li>基础加密传输</li>
                    <li>标准配送流程</li>
                  </ul>
                </div>
              </div>

              <div class="security-item">
                <el-tag type="warning" size="small">L2</el-tag>
                <div class="security-description">
                  <p><strong>中等安全等级</strong></p>
                  <ul>
                    <li>适用于重要文件、贵重物品</li>
                    <li>强化加密传输</li>
                    <li>需要身份验证</li>
                    <li>全程跟踪记录</li>
                  </ul>
                </div>
              </div>

              <div class="security-item">
                <el-tag type="danger" size="small">L3</el-tag>
                <div class="security-description">
                  <p><strong>高级安全等级</strong></p>
                  <ul>
                    <li>适用于机密文件、核心资料</li>
                    <li>军用级加密传输</li>
                    <li>多重身份认证</li>
                    <li>全程监控录像</li>
                    <li>专用配送路线</li>
                  </ul>
                </div>
              </div>
            </div>
          </el-card>
        </div>
      </div>
    </div>    <!-- 身份认证弹窗 -->
    <UserAuthModal
      v-model="showAuthModal"
      purpose="send"
      :required-level="sendForm.securityLevel"
      @auth-success="handleAuthSuccess"
    />

    <!-- 寄送成功对话框 -->
    <el-dialog
      v-model="showSuccessDialog"
      title="寄送成功"
      width="50vw"
      :before-close="handleSuccessClose"
    >
      <el-result icon="success" title="包裹寄送成功">
        <template #sub-title>
          <div class="success-info">
            <p>您的包裹已成功投递到柜门</p>
            <div class="compartment-info">
              <el-descriptions title="柜门信息" :column="2" border>
                <el-descriptions-item label="柜门编号">
                  {{ selectedCompartment?.floor }}F-{{ selectedCompartment?.compartmentNumber }}
                </el-descriptions-item>
                <el-descriptions-item label="安全等级">
                  {{ selectedCompartment?.securityLevel }}
                </el-descriptions-item>
                <el-descriptions-item label="寄送时间">
                  {{ sendTime }}
                </el-descriptions-item>
                <el-descriptions-item label="预计送达">
                  {{ estimatedDelivery }}
                </el-descriptions-item>
              </el-descriptions>
            </div>
            <el-alert
              title="柜门已打开"
              description="请将包裹放入指定柜门，关闭柜门后配送将自动开始"
              type="success"
              :closable="false"
              style="margin-top: 2vh"
            />
          </div>
        </template>
        <template #extra>
          <el-button type="primary" @click="handleSuccessClose">确定</el-button>
          <el-button @click="sendAnother">再次寄送</el-button>
        </template>
      </el-result>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, reactive, watch, onMounted } from 'vue'
import { useRouter } from 'vue-router'
import { ElMessage } from 'element-plus'
import { useRobotStore } from '@/stores/robot'
import { authService } from '@/services/authService'
import { locationApiService } from '@/services/locationApiService'
import { taskApiService } from '@/services/taskApiService'
import { systemStatusService } from '@/services/systemStatusService'
import { userApiService } from '@/services/userApiService'
import UserAuthModal from '@/components/UserAuthModal.vue'
import { Box, Lock, Upload, Grid, InfoFilled } from '@element-plus/icons-vue'

const router = useRouter()
const robotStore = useRobotStore()

// 表单数据
const sendForm = reactive({
  securityLevel: 'L1',
  receiver: '', // 新增收件人字段
  destination: '', // 改为字符串类型
  recipientInfo: {
    name: '',
    phone: '',
    idNumber: '',
  },
  description: '',
  authenticated: false,
})

// 表单验证规则
const sendRules = computed(() => ({
  securityLevel: [{ required: true, message: '请选择安全等级', trigger: 'change' }],
  receiver: [{ required: true, message: '请选择收件人', trigger: 'change' }],
  destination: [{ required: true, message: '请选择收件地址', trigger: 'change' }],
  description: [{ required: true, message: '请输入包裹描述', trigger: 'blur' }],
  ...(sendForm.securityLevel === 'L3' && {
    recipientInfo: [{ required: true, message: '请填写完整的收件人信息', trigger: 'blur' }],
  }),
}))

// 位置选项（从地图服务获取）
const locationOptions = ref<any[]>([])

// 用户选项（从用户服务获取）
const userOptions = ref<any[]>([])
const isLoadingUsers = ref(false)

// 初始化地点数据和用户数据
onMounted(async () => {
  // 加载地点数据
  try {
    console.log('开始获取地点数据...')
    const locations = await locationApiService.getAllLocations()
    console.log('获取到的地点数据:', locations)
    
    locationOptions.value = locations.map((location: any) => ({
      value: location.location_id,  // 使用 location_id 作为值
      label: location.label,        // 使用 label 作为显示文本
      location: location
    }))
    
    console.log('处理后的地点选项:', locationOptions.value)
  } catch (error) {
    console.error('获取地点数据失败:', error)
    ElMessage.error('获取地点数据失败')
  }

  // 加载用户数据
  try {
    console.log('开始获取用户数据...')
    isLoadingUsers.value = true
    const users = await userApiService.getAllUsers()
    console.log('获取到的用户数据:', users)
    
    userOptions.value = users.map((user: any) => ({
      value: user.user_id,     // 使用 user_id 作为值
      label: user.name,        // 使用 name 作为显示文本
      authLevel: user.auth_level,  // 保存认证等级用于显示
      user: user
    }))
    
    console.log('处理后的用户选项:', userOptions.value)
  } catch (error) {
    console.error('获取用户数据失败:', error)
    ElMessage.error('获取用户数据失败')
  } finally {
    isLoadingUsers.value = false
  }
})

// 状态
const sendFormRef = ref()
const showAuthModal = ref(false)
const showSuccessDialog = ref(false)
const isSubmitting = ref(false)
const selectedCompartment = ref<any>(null)
const sendTime = ref('')
const estimatedDelivery = ref('')

// 计算属性
const isAuthenticated = computed(() => authService.isAuthenticated())
const userAuthLevel = computed(() => authService.getCurrentUser()?.auth_level || null)
const currentUser = computed(() => authService.getCurrentUser())

const availableCompartments = computed(() => robotStore.compartments.filter((c) => !c.isOccupied))

const occupiedCompartments = computed(() => robotStore.compartments.filter((c) => c.isOccupied))

const canSubmit = computed(() => {
  const hasAuth = isAuthenticated.value
  const hasRequiredInfo =
    sendForm.securityLevel !== 'L3' ||
    (sendForm.recipientInfo.name && sendForm.recipientInfo.phone && sendForm.recipientInfo.idNumber)
  return hasAuth && hasRequiredInfo && sendForm.receiver.length > 0 && sendForm.destination.length > 0 && sendForm.description
})

// 监听安全等级变化
watch(
  () => sendForm.securityLevel,
  (newLevel) => {
    // 检查当前认证等级是否满足要求
    if (isAuthenticated.value) {
      const authLevelNum = parseInt(userAuthLevel.value?.replace('L', '') || '0')
      const requiredLevelNum = parseInt(newLevel.replace('L', ''))

      if (authLevelNum < requiredLevelNum) {
        ElMessage.warning(`${newLevel}级别包裹需要${newLevel}级别或以上认证`)
      }
    }
  },
)

// 方法
const getLevelType = (level: string) => {
  const types = {
    L1: 'success',
    L2: 'warning',
    L3: 'danger',
  }
  return types[level as keyof typeof types] || 'info'
}

const onSecurityLevelChange = () => {
  // 清空收件人选择和信息
  sendForm.receiver = ''
  sendForm.recipientInfo = {
    name: '',
    phone: '',
    idNumber: '',
  }
}

const handleAuthSuccess = (user: any, authResult: any) => {
  ElMessage.success(`身份认证成功，获得${user.auth_level}级别权限`)
  console.log('认证成功:', user, authResult)
}

const submitSend = async () => {
  if (!sendFormRef.value) return

  try {
    await sendFormRef.value.validate()
    isSubmitting.value = true

    // 检查认证状态
    if (!isAuthenticated.value) {
      ElMessage.error('请先进行身份认证')
      return
    }

    // 检查认证等级
    const currentAuthLevel = userAuthLevel.value
    if (!currentAuthLevel) {
      ElMessage.error('无法获取认证等级')
      return
    }

    const authLevelNum = parseInt(currentAuthLevel.replace('L', ''))
    const requiredLevelNum = parseInt(sendForm.securityLevel.replace('L', ''))

    if (authLevelNum < requiredLevelNum) {
      ElMessage.error(`权限不足，需要${sendForm.securityLevel}级别认证`)
      return
    }

    // 获取当前用户信息
    const user = currentUser.value
    if (!user) {
      ElMessage.error('用户信息异常')
      return
    }

    // 获取目标地点信息（现在 destination 已经是 location_id 了）
    const targetLocationId = Array.isArray(sendForm.destination) ? sendForm.destination[0] : sendForm.destination
    if (!targetLocationId) {
      ElMessage.error('请选择收件地址')
      return
    }

    // 创建寄送任务
    const taskRequest = {
      user_id: user.user_id,
      receiver: sendForm.receiver, // 使用选择的收件人ID
      location_id: targetLocationId,
      security_level: sendForm.securityLevel as 'L1' | 'L2' | 'L3',
      description: sendForm.description
    }

    const taskResult = await taskApiService.createTask(taskRequest)

    if (taskResult.success && taskResult.task_id) {
      // 更新柜门状态
      if (taskResult.locker_id) {
        systemStatusService.updateLockerStatus(
          taskResult.locker_id,
          'occupied',
          taskResult.task_id
        )
      }

      // 设置成功信息
      selectedCompartment.value = {
        lockerId: taskResult.locker_id,
        compartmentId: 'A1', // 模拟柜门编号
        taskId: taskResult.task_id
      }

      sendTime.value = new Date().toLocaleString('zh-CN')
      const deliveryTime = new Date()
      deliveryTime.setMinutes(
        deliveryTime.getMinutes() + (sendForm.securityLevel === 'L3' ? 10 : 5),
      )
      estimatedDelivery.value = deliveryTime.toLocaleString('zh-CN')

      showSuccessDialog.value = true
      ElMessage.success('包裹寄送成功')
    } else {
      ElMessage.error(taskResult.message || '寄送失败')
    }
  } catch (error) {
    console.error('寄送失败:', error)
    ElMessage.error('寄送失败，请重试')
  } finally {
    isSubmitting.value = false
  }
}

const resetForm = () => {
  if (sendFormRef.value) {
    sendFormRef.value.resetFields()
  }
  sendForm.receiver = ''
  sendForm.recipientInfo = {
    name: '',
    phone: '',
    idNumber: '',
  }
}

const handleSuccessClose = () => {
  showSuccessDialog.value = false
  router.push('/')
}

const sendAnother = () => {
  showSuccessDialog.value = false
  resetForm()
}
</script>

<style scoped>
.send-package-container {
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

.send-content {
  margin-top: 2vh;
}

.content-layout {
  display: grid;
  grid-template-columns: 1fr 22vw;
  grid-template-rows: auto auto;
  gap: 1.5vw;
  height: 85vh;
  position: relative;
}

.form-section {
  grid-column: 1;
  grid-row: 1;
}

.status-section {
  grid-column: 2;
  grid-row: 1 / span 2;
}

.security-info-section {
  grid-column: 1;
  grid-row: 2;
  align-self: start;
}

.send-form-card,
.status-card,
.info-card {
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  border: none;
  border-radius: 16px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;
  animation: slideInUp 0.6s ease-out;
}

.send-form-card:hover,
.status-card:hover,
.info-card:hover {
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
.security-radio-group {
  width: 100%;
}

.security-radio {
  width: 100%;
  margin-bottom: 1.5vh;
  border: 2px solid #e4e7ed;
  border-radius: 12px;
  padding: 2.5vh 1.2vw;
  transition: all 0.3s ease;
  background: linear-gradient(135deg, #f8f9ff 0%, #ffffff 100%);
  min-height: 10vh;
}

.security-radio:hover {
  border-color: #667eea;
  transform: translateX(8px);
  box-shadow: 0 4px 16px rgba(102, 126, 234, 0.2);
}

.security-radio.is-checked {
  border-color: #667eea;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

.security-radio.is-checked .security-option {
  color: white;
}

.security-option {
  margin-left: 1vw;
  transition: color 0.3s ease;
}

.security-title {
  font-weight: 600;
  font-size: 0.9vw;
  margin-bottom: 0.5vh;
}

.security-desc {
  font-size: 0.7vw;
  opacity: 0.8;
  margin-bottom: 1vh;
}

.security-features {
  display: flex;
  gap: 0.5vw;
  flex-wrap: wrap;
}

.auth-section {
  width: 100%;
}

.auth-required,
.auth-success {
  padding: 2vh 1.5vw;
  border-radius: 12px;
  background: linear-gradient(135deg, #f0f9ff 0%, #e0f2fe 100%);
  border: 1px solid rgba(59, 130, 246, 0.2);
  transition: all 0.3s ease;
}

.auth-required:hover,
.auth-success:hover {
  transform: scale(1.02);
  box-shadow: 0 4px 16px rgba(59, 130, 246, 0.2);
}

.recipient-form {
  animation: slideInDown 0.5s ease-out;
}

.recipient-inputs {
  width: 100%;
}

.recipient-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1vw;
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

/* 柜门状态样式 */
.compartment-status {
  padding: 1vh 0;
}

.compartment-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1vw;
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

.compartment-item.available {
  background: linear-gradient(135deg, #f0f9ff 0%, #dbeafe 100%);
  border-color: #409eff;
  box-shadow: 0 4px 16px rgba(64, 158, 255, 0.2);
}

.compartment-item.selected {
  background: linear-gradient(135deg, #e1f3d8 0%, #dcfce7 100%);
  border-color: #67c23a;
  box-shadow: 0 4px 20px rgba(103, 194, 58, 0.3);
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

/* 安全等级说明样式 */
.security-info {
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
}

.security-item {
  display: flex;
  gap: 1vw;
  align-items: flex-start;
  padding: 1vh 0;
  border-bottom: 1px solid rgba(0, 0, 0, 0.05);
  transition: all 0.3s ease;
}

.security-item:hover {
  transform: translateX(8px);
  background: rgba(102, 126, 234, 0.05);
  padding: 1vh;
  border-radius: 8px;
}

.security-item:last-child {
  border-bottom: none;
}

.security-description {
  flex: 1;
}

.security-description p {
  margin: 0 0 1vh 0;
  font-size: 0.7vw;
  font-weight: 600;
  color: #303133;
}

.security-description ul {
  margin: 0;
  padding-left: 1.5vw;
  font-size: 0.6vw;
  color: #606266;
}

.security-description li {
  margin-bottom: 0.5vh;
  transition: color 0.3s ease;
}

.security-item:hover .security-description li {
  color: #333;
}

/* 成功对话框样式 */
.success-info {
  text-align: left;
}

.compartment-info {
  margin: 2vh 0;
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

@keyframes slideInDown {
  from {
    opacity: 0;
    transform: translateY(-20px);
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
    grid-template-rows: auto auto auto;
  }

  .status-section {
    grid-column: 1;
    grid-row: 2;
  }

  .security-info-section {
    grid-column: 1;
    grid-row: 3;
  }

  .page-title {
    font-size: 3vw;
  }

  .card-header {
    font-size: 2vw;
  }

  .security-title {
    font-size: 1.8vw;
  }

  .security-desc {
    font-size: 1.4vw;
  }

  .submit-btn {
    min-width: 20vw;
    font-size: 2vw;
  }
}

@media (max-width: 768px) {
  .send-package-container {
    padding: 3vw;
  }

  .page-title {
    font-size: 4vw;
  }

  .content-layout {
    gap: 3vw;
  }

  .recipient-row {
    grid-template-columns: 1fr;
    gap: 2vw;
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
  font-size: 0.9vw;
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

:deep(.el-alert) {
  border-radius: 8px;
  border: none;
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

:deep(.el-descriptions__label) {
  font-weight: 600;
}
</style>
