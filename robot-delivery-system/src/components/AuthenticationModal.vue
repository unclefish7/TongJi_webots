<template>
  <el-dialog v-model="visible" title="身份认证" width="500px" :before-close="handleClose">
    <div class="auth-container">
      <el-steps :active="currentStep" finish-status="success" align-center>
        <el-step title="选择认证方式" />
        <el-step title="身份验证" />
        <el-step title="认证完成" />
      </el-steps>

      <!-- 步骤1：选择认证方式 -->
      <div v-if="currentStep === 0" class="step-content">
        <el-card class="auth-method-card">
          <template #header>
            <span>请选择身份认证方式</span>
          </template>
          <el-radio-group v-model="selectedAuthMethod" class="auth-methods">
            <el-radio value="card" class="auth-method-item">
              <div class="method-content">
                <el-icon><CreditCard /></el-icon>
                <div>
                  <div class="method-title">员工卡认证</div>
                  <div class="method-desc">L1级别 - 基础权限</div>
                </div>
              </div>
            </el-radio>
            <el-radio value="face" class="auth-method-item">
              <div class="method-content">
                <el-icon><User /></el-icon>
                <div>
                  <div class="method-title">人脸识别</div>
                  <div class="method-desc">L2级别 - 中等权限</div>
                </div>
              </div>
            </el-radio>
            <el-radio value="fingerprint" class="auth-method-item">
              <div class="method-content">
                <el-icon><Key /></el-icon>
                <div>
                  <div class="method-title">指纹识别</div>
                  <div class="method-desc">L3级别 - 高级权限</div>
                </div>
              </div>
            </el-radio>
          </el-radio-group>
        </el-card>
      </div>

      <!-- 步骤2：身份验证 -->
      <div v-if="currentStep === 1" class="step-content">
        <el-card class="verification-card">
          <template #header>
            <span>{{ getAuthMethodTitle() }}</span>
          </template>
          <div class="verification-content">
            <div v-if="selectedAuthMethod === 'card'" class="verification-method">
              <el-icon class="verification-icon"><CreditCard /></el-icon>
              <p>请将员工卡放置在读卡器上</p>
              <el-progress :percentage="verificationProgress" :status="verificationStatus" />
            </div>
            <div v-else-if="selectedAuthMethod === 'face'" class="verification-method">
              <el-icon class="verification-icon"><User /></el-icon>
              <p>请正视摄像头进行人脸识别</p>
              <el-progress :percentage="verificationProgress" :status="verificationStatus" />
            </div>
            <div v-else-if="selectedAuthMethod === 'fingerprint'" class="verification-method">
              <el-icon class="verification-icon"><Key /></el-icon>
              <p>请将手指放置在指纹扫描器上</p>
              <el-progress :percentage="verificationProgress" :status="verificationStatus" />
            </div>
          </div>
        </el-card>
      </div>

      <!-- 步骤3：认证完成 -->
      <div v-if="currentStep === 2" class="step-content">
        <el-result
          :icon="authResult.success ? 'success' : 'error'"
          :title="authResult.title"
          :sub-title="authResult.message"
        >
          <template #extra>
            <div v-if="authResult.success" class="auth-success-info">
              <el-descriptions title="认证信息" :column="1" border>
                <el-descriptions-item label="用户姓名">{{
                  authResult.userInfo?.name
                }}</el-descriptions-item>
                <el-descriptions-item label="认证等级">{{
                  authResult.userInfo?.level
                }}</el-descriptions-item>
                <el-descriptions-item label="部门">{{
                  authResult.userInfo?.department
                }}</el-descriptions-item>
                <el-descriptions-item label="有效期">{{
                  authResult.userInfo?.expiry
                }}</el-descriptions-item>
              </el-descriptions>
            </div>
          </template>
        </el-result>
      </div>
    </div>

    <template #footer>
      <div class="dialog-footer">
        <el-button v-if="currentStep > 0 && currentStep < 2" @click="prevStep">上一步</el-button>
        <el-button
          v-if="currentStep === 0"
          type="primary"
          :disabled="!selectedAuthMethod"
          @click="nextStep"
        >
          下一步
        </el-button>
        <el-button
          v-if="currentStep === 1"
          type="primary"
          :loading="isVerifying"
          @click="startVerification"
        >
          开始验证
        </el-button>
        <el-button v-if="currentStep === 2" type="primary" @click="confirm">
          {{ authResult.success ? '确认' : '重试' }}
        </el-button>
        <el-button @click="handleClose">取消</el-button>
      </div>
    </template>
  </el-dialog>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'
import { CreditCard, User, Key } from '@element-plus/icons-vue'
import { useRobotStore } from '@/stores/robot'

interface Props {
  modelValue: boolean
}

interface Emits {
  (e: 'update:modelValue', value: boolean): void
  (e: 'auth-success', level: 'L1' | 'L2' | 'L3', userInfo: any): void
}

const props = defineProps<Props>()
const emit = defineEmits<Emits>()

const robotStore = useRobotStore()

const visible = computed({
  get: () => props.modelValue,
  set: (value) => emit('update:modelValue', value),
})

const currentStep = ref(0)
const selectedAuthMethod = ref('')
const isVerifying = ref(false)
const verificationProgress = ref(0)
const verificationStatus = ref<'success' | 'exception' | undefined>(undefined)

const authResult = ref({
  success: false,
  title: '',
  message: '',
  userInfo: null as any,
})

const getAuthMethodTitle = () => {
  const titles = {
    card: '员工卡认证',
    face: '人脸识别',
    fingerprint: '指纹识别',
  }
  return titles[selectedAuthMethod.value as keyof typeof titles] || ''
}

const getAuthLevel = () => {
  const levels = {
    card: 'L1',
    face: 'L2',
    fingerprint: 'L3',
  }
  return levels[selectedAuthMethod.value as keyof typeof levels] as 'L1' | 'L2' | 'L3'
}

const nextStep = () => {
  currentStep.value++
}

const prevStep = () => {
  currentStep.value--
}

const startVerification = async () => {
  isVerifying.value = true
  verificationProgress.value = 0
  verificationStatus.value = undefined

  // 模拟验证过程
  const interval = setInterval(() => {
    verificationProgress.value += 10
    if (verificationProgress.value >= 100) {
      clearInterval(interval)
      finishVerification()
    }
  }, 200)
}

const finishVerification = () => {
  isVerifying.value = false

  // 模拟认证结果（实际应该调用真实的认证API）
  const success = Math.random() > 0.2 // 80% 成功率

  if (success) {
    verificationStatus.value = 'success'
    const level = getAuthLevel()
    const userInfo = {
      name: '张三',
      level: level,
      department: level === 'L1' ? '普通员工' : level === 'L2' ? '部门主管' : '系统管理员',
      expiry: '2024-12-31',
    }

    authResult.value = {
      success: true,
      title: '认证成功',
      message: `您已获得${level}级别访问权限`,
      userInfo,
    }

    robotStore.setUserAuthLevel(level)
  } else {
    verificationStatus.value = 'exception'
    authResult.value = {
      success: false,
      title: '认证失败',
      message: '身份验证失败，请重试',
      userInfo: null,
    }
  }

  currentStep.value = 2
}

const confirm = () => {
  if (authResult.value.success) {
    emit('auth-success', getAuthLevel(), authResult.value.userInfo)
    handleClose()
  } else {
    // 重试
    reset()
  }
}

const reset = () => {
  currentStep.value = 0
  selectedAuthMethod.value = ''
  isVerifying.value = false
  verificationProgress.value = 0
  verificationStatus.value = undefined
  authResult.value = {
    success: false,
    title: '',
    message: '',
    userInfo: null,
  }
}

const handleClose = () => {
  reset()
  visible.value = false
}
</script>

<style scoped>
.auth-container {
  padding: 2vh 0;
  background: linear-gradient(135deg, rgba(102, 126, 234, 0.05) 0%, rgba(118, 75, 162, 0.05) 100%);
  border-radius: 16px;
}

.step-content {
  margin-top: 3vh;
  min-height: 35vh;
  animation: slideInUp 0.6s ease-out;
}

.auth-method-card,
.verification-card {
  margin: 2vh 0;
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  border: none;
  border-radius: 16px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;
}

.auth-method-card:hover,
.verification-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15);
}

.auth-methods {
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
  padding: 2vh;
}

.auth-method-item {
  width: 100%;
  padding: 2vh 1.5vw;
  border: 2px solid #e4e7ed;
  border-radius: 12px;
  transition: all 0.3s ease;
  background: linear-gradient(135deg, #f8f9ff 0%, #ffffff 100%);
  position: relative;
  overflow: hidden;
}

.auth-method-item::before {
  content: '';
  position: absolute;
  top: 0;
  left: -100%;
  width: 100%;
  height: 100%;
  background: linear-gradient(90deg, transparent, rgba(102, 126, 234, 0.1), transparent);
  transition: left 0.5s ease;
}

.auth-method-item:hover {
  border-color: #667eea;
  transform: translateX(8px);
  box-shadow: 0 4px 16px rgba(102, 126, 234, 0.2);
}

.auth-method-item:hover::before {
  left: 100%;
}

.auth-method-item.is-checked {
  border-color: #667eea;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  transform: scale(1.02);
}

.method-content {
  display: flex;
  align-items: center;
  gap: 1.5vw;
  position: relative;
  z-index: 1;
}

.method-content .el-icon {
  font-size: 2vw;
  color: #667eea;
  transition: all 0.3s ease;
}

.auth-method-item.is-checked .method-content .el-icon {
  color: white;
  transform: scale(1.1);
}

.method-title {
  font-weight: 600;
  font-size: 1vw;
  color: #303133;
  margin-bottom: 0.5vh;
  transition: color 0.3s ease;
}

.auth-method-item.is-checked .method-title {
  color: white;
}

.method-desc {
  font-size: 0.8vw;
  color: #606266;
  transition: color 0.3s ease;
}

.auth-method-item.is-checked .method-desc {
  color: rgba(255, 255, 255, 0.9);
}

.verification-content {
  text-align: center;
  padding: 4vh 2vw;
  background: linear-gradient(135deg, #f0f9ff 0%, #e0f2fe 100%);
  border-radius: 12px;
  margin: 2vh;
}

.verification-method {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2vh;
  animation: pulse 2s infinite;
}

.verification-icon {
  font-size: 4vw;
  color: #667eea;
  background: linear-gradient(45deg, #667eea, #764ba2);
  background-clip: text;
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  animation: iconFloat 3s ease-in-out infinite;
}

.verification-method p {
  font-size: 1vw;
  color: #606266;
  margin: 0;
  font-weight: 500;
}

.auth-success-info {
  max-width: 30vw;
  margin: 0 auto;
  animation: fadeInScale 0.8s ease-out;
}

.dialog-footer {
  text-align: right;
  padding: 2vh 0;
  border-top: 1px solid rgba(102, 126, 234, 0.1);
  margin-top: 2vh;
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

@keyframes pulse {
  0%,
  100% {
    transform: scale(1);
  }
  50% {
    transform: scale(1.02);
  }
}

@keyframes iconFloat {
  0%,
  100% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(-10px);
  }
}

/* 响应式设计 */
@media (max-width: 1200px) {
  .method-content .el-icon {
    font-size: 3vw;
  }

  .method-title {
    font-size: 2vw;
  }

  .method-desc {
    font-size: 1.5vw;
  }

  .verification-icon {
    font-size: 6vw;
  }

  .verification-method p {
    font-size: 1.8vw;
  }

  .auth-success-info {
    max-width: 60vw;
  }
}

@media (max-width: 768px) {
  .step-content {
    min-height: 40vh;
  }

  .auth-methods {
    gap: 2vh;
  }

  .method-content {
    gap: 3vw;
  }

  .method-content .el-icon {
    font-size: 4vw;
  }

  .method-title {
    font-size: 3vw;
  }

  .method-desc {
    font-size: 2.5vw;
  }

  .verification-icon {
    font-size: 8vw;
  }

  .verification-method p {
    font-size: 3vw;
  }

  .auth-success-info {
    max-width: 80vw;
  }
}

/* Element Plus 组件覆盖样式 */
:deep(.el-dialog) {
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  border-radius: 16px;
  border: none;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.2);
}

:deep(.el-dialog__header) {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border-top-left-radius: 16px;
  border-top-right-radius: 16px;
  padding: 2vh 2vw;
}

:deep(.el-dialog__title) {
  font-weight: 600;
  font-size: 1.1vw;
  color: white;
}

:deep(.el-dialog__headerbtn .el-dialog__close) {
  color: white;
  font-size: 1.2vw;
}

:deep(.el-dialog__headerbtn .el-dialog__close:hover) {
  color: rgba(255, 255, 255, 0.8);
}

:deep(.el-steps) {
  margin: 2vh 0;
}

:deep(.el-step__title) {
  font-size: 0.9vw;
  font-weight: 500;
}

:deep(.el-step__description) {
  font-size: 0.7vw;
}

:deep(.el-card__header) {
  background: linear-gradient(135deg, #f8f9ff 0%, #ffffff 100%);
  border-bottom: 1px solid rgba(102, 126, 234, 0.1);
  font-weight: 600;
  color: #667eea;
}

:deep(.el-button) {
  border-radius: 8px;
  transition: all 0.3s ease;
  font-weight: 500;
}

:deep(.el-button:hover) {
  transform: translateY(-1px);
}

:deep(.el-button--primary) {
  background: linear-gradient(45deg, #667eea, #764ba2);
  border: none;
}

:deep(.el-button--primary:hover) {
  background: linear-gradient(45deg, #5a6fd8, #6a4190);
  box-shadow: 0 4px 16px rgba(102, 126, 234, 0.4);
}

:deep(.el-progress-bar__outer) {
  border-radius: 10px;
  background: rgba(102, 126, 234, 0.1);
}

:deep(.el-progress-bar__inner) {
  border-radius: 10px;
  background: linear-gradient(45deg, #667eea, #764ba2);
}

:deep(.el-descriptions__label) {
  font-weight: 600;
  color: #667eea;
}

:deep(.el-radio__input.is-checked .el-radio__inner) {
  background-color: white;
  border-color: white;
}

:deep(.el-radio__input.is-checked + .el-radio__label) {
  color: white;
}

:deep(.el-result__title) {
  color: #667eea;
  font-weight: 600;
}
</style>
