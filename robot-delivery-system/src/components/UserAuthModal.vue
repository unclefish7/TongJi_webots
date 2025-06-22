<template>
  <el-dialog 
    v-model="visible" 
    title="身份认证" 
    width="60vw" 
    :before-close="handleClose"
    :close-on-click-modal="false"
    destroy-on-close
  >
    <div class="auth-container">
      <el-steps :active="currentStep" finish-status="success" align-center>
        <el-step title="选择用户" />
        <el-step title="身份验证" />
        <el-step title="认证完成" />
      </el-steps>

      <!-- 步骤1：选择用户 -->
      <div v-if="currentStep === 0" class="step-content">
        <el-card class="user-selection-card">
          <template #header>
            <span>请选择登录用户（演示模式）</span>
          </template>
          
          <div class="user-grid">
            <el-card 
              v-for="user in availableUsers" 
              :key="user.user_id"
              :class="['user-card', { 'selected': selectedUser?.user_id === user.user_id }]"
              @click="selectUser(user)"
              shadow="hover"
            >
              <div class="user-info">
                <el-avatar :size="50" class="user-avatar">
                  {{ user.name.charAt(0) }}
                </el-avatar>
                <div class="user-details">
                  <div class="user-name">{{ user.name }}</div>
                  <div class="user-id">ID: {{ user.user_id }}</div>
                  <div class="user-level">权限: {{ user.auth_level }}</div>
                  <div class="user-location">办公地点: {{ user.office_location }}</div>
                </div>
              </div>
            </el-card>
          </div>
          
          <div class="loading-message" v-if="loadingUsers">
            <el-icon class="is-loading"><Loading /></el-icon>
            加载用户列表...
          </div>
        </el-card>
      </div>

      <!-- 步骤2：身份验证 -->
      <div v-if="currentStep === 1" class="step-content">
        <el-card class="verification-card">
          <template #header>
            <span>{{ selectedUser?.name }} - {{ authPurpose === 'send' ? '寄件' : '取件' }}认证</span>
          </template>
          
          <div class="user-summary">
            <el-descriptions :column="2" border>
              <el-descriptions-item label="用户姓名">{{ selectedUser?.name }}</el-descriptions-item>
              <el-descriptions-item label="员工卡号">{{ selectedUser?.user_id }}</el-descriptions-item>
              <el-descriptions-item label="权限等级">{{ selectedUser?.auth_level }}</el-descriptions-item>
              <el-descriptions-item label="办公地点">{{ selectedUser?.office_location }}</el-descriptions-item>
            </el-descriptions>
          </div>

          <el-divider />

          <el-form :model="authForm" label-width="120px">
            <el-form-item label="认证等级">
              <el-select v-model="authForm.requested_level" placeholder="选择所需认证等级">
                <el-option label="L1 - 基础权限" value="L1" />
                <el-option label="L2 - 中等权限" value="L2" :disabled="!canUseLevel('L2')" />
                <el-option label="L3 - 高级权限" value="L3" :disabled="!canUseLevel('L3')" />
              </el-select>
            </el-form-item>

            <el-form-item 
              v-if="authForm.requested_level === 'L2' || authForm.requested_level === 'L3'"
              label="L2认证密码"
            >
              <el-input 
                v-model="authForm.l2_auth" 
                type="password" 
                placeholder="请输入L2级认证密码"
                show-password
              />
            </el-form-item>

            <el-form-item 
              v-if="authForm.requested_level === 'L3'"
              label="L3认证密码"
            >
              <el-input 
                v-model="authForm.l3_auth" 
                type="password" 
                placeholder="请输入L3级认证密码"
                show-password
              />
            </el-form-item>
          </el-form>

          <div class="auth-tips">
            <el-alert 
              title="认证提示" 
              type="info" 
              :closable="false"
              show-icon
            >
              <template #default>
                <div>当前用户最高权限等级：<strong>{{ selectedUser?.auth_level }}</strong></div>
                <div v-if="authForm.requested_level === 'L1'">L1级认证只需要员工卡号即可</div>
                <div v-if="authForm.requested_level === 'L2'">L2级认证需要员工卡号 + L2认证密码</div>
                <div v-if="authForm.requested_level === 'L3'">L3级认证需要员工卡号 + L2认证密码 + L3认证密码</div>
              </template>
            </el-alert>
          </div>
        </el-card>
      </div>

      <!-- 步骤3：认证结果 -->
      <div v-if="currentStep === 2" class="step-content">
        <el-card class="result-card">
          <div class="result-content">
            <el-result
              :icon="authResult.success ? 'success' : 'error'"
              :title="authResult.success ? '认证成功' : '认证失败'"
              :sub-title="authResult.message"
            >
              <template #extra>
                <div v-if="authResult.success" class="success-info">
                  <el-descriptions :column="1" border>
                    <el-descriptions-item label="认证等级">{{ authResult.verified_level }}</el-descriptions-item>
                    <el-descriptions-item label="认证方式">{{ authResult.methods?.join(', ') }}</el-descriptions-item>
                    <el-descriptions-item v-if="authResult.expires_at" label="有效期至">
                      {{ formatDateTime(authResult.expires_at) }}
                    </el-descriptions-item>
                  </el-descriptions>
                </div>
              </template>
            </el-result>
          </div>
        </el-card>
      </div>
    </div>

    <template #footer>
      <div class="dialog-footer">
        <el-button 
          v-if="currentStep === 0" 
          type="primary" 
          :disabled="!selectedUser"
          @click="nextStep"
        >
          下一步
        </el-button>
        <el-button 
          v-if="currentStep === 1" 
          @click="prevStep"
        >
          上一步
        </el-button>
        <el-button 
          v-if="currentStep === 1" 
          type="primary" 
          :loading="authenticating"
          @click="performAuth"
        >
          开始认证
        </el-button>
        <el-button 
          v-if="currentStep === 2" 
          type="primary" 
          @click="handleClose"
        >
          <el-icon><Check /></el-icon>
          确认
        </el-button>
      </div>
    </template>
  </el-dialog>
</template>

<script setup lang="ts">
import { ref, reactive, onMounted, computed, watch } from 'vue'
import { ElMessage } from 'element-plus'
import { Loading, Check } from '@element-plus/icons-vue'
import { authService, type User, type AuthResponse } from '@/services/authService'
import { authApiService } from '@/services/authApiService'

// Props
interface Props {
  modelValue: boolean
  purpose: 'send' | 'pickup'
  requiredLevel?: string
}

const props = withDefaults(defineProps<Props>(), {
  requiredLevel: 'L1'
})

// Emits
const emit = defineEmits<{
  'update:modelValue': [value: boolean]
  'auth-success': [user: User, authResult: AuthResponse]
  'auth-failed': [error: string]
}>()

// 响应式数据
const visible = computed({
  get: () => props.modelValue,
  set: (value) => emit('update:modelValue', value)
})

const currentStep = ref(0)
const availableUsers = ref<User[]>([])
const selectedUser = ref<User | null>(null)
const loadingUsers = ref(false)
const authenticating = ref(false)
const authPurpose = computed(() => props.purpose)

const authForm = reactive({
  requested_level: props.requiredLevel,
  l2_auth: '',
  l3_auth: ''
})

const authResult = reactive<AuthResponse & { message?: string }>({
  success: false,
  message: ''
})

// 计算属性
const canUseLevel = (level: string) => {
  if (!selectedUser.value) return false
  const levelOrder = { 'L1': 1, 'L2': 2, 'L3': 3 }
  const userLevel = levelOrder[selectedUser.value.auth_level as keyof typeof levelOrder] || 0
  const requiredLevel = levelOrder[level as keyof typeof levelOrder] || 0
  return userLevel >= requiredLevel
}

// 监听模态框开启，当打开时重置状态
watch(visible, (newValue, oldValue) => {
  if (newValue && !oldValue) {
    // 模态框从关闭变为打开时，重置到初始状态
    resetToStart()
  }
})

// 重置到初始状态
const resetToStart = () => {
  currentStep.value = 0
  selectedUser.value = null
  authForm.l2_auth = ''
  authForm.l3_auth = ''
  authForm.requested_level = props.requiredLevel
  Object.assign(authResult, { success: false, message: '' })
}

// 方法
const loadUsers = async () => {
  loadingUsers.value = true
  try {
    availableUsers.value = await authService.getAllUsers()
  } catch (error) {
    console.error('加载用户列表失败:', error)
    ElMessage.error('加载用户列表失败')
  } finally {
    loadingUsers.value = false
  }
}

const selectUser = (user: User) => {
  selectedUser.value = user
  authForm.requested_level = props.requiredLevel
}

const nextStep = () => {
  if (currentStep.value < 2) {
    currentStep.value++
  }
}

const prevStep = () => {
  if (currentStep.value > 0) {
    currentStep.value--
  }
}

const performAuth = async () => {
  if (!selectedUser.value) return

  authenticating.value = true
  try {
    const authRequest = {
      user_id: selectedUser.value.user_id,
      purpose: props.purpose,
      requested_level: authForm.requested_level as "L1" | "L2" | "L3", // 类型断言,
      provided: {
        l2_auth: authForm.l2_auth,
        l3_auth: authForm.l3_auth
      }
    }

    // 使用基于用途的认证API
    const result = await authApiService.verifyPurposeAuth(authRequest)
    
    if (result.verified) {
      authResult.success = true
      authResult.message = `认证成功！获得 ${result.verified_level} 级权限`
      authResult.verified_level = result.verified_level
      authResult.methods = result.methods
      authResult.expires_at = result.expires_at
      
      // 认证成功后自动通知切换用户
      emit('auth-success', selectedUser.value, {
        success: true,
        verified_level: result.verified_level,
        methods: result.methods,
        expires_at: result.expires_at
      })
    } else {
      authResult.success = false
      authResult.message = result.message || '认证失败'
      emit('auth-failed', authResult.message)
    }
    
    currentStep.value = 2
  } catch (error) {
    console.error('认证过程出错:', error)
    authResult.success = false
    authResult.message = '认证过程出错，请重试'
    emit('auth-failed', authResult.message)
    currentStep.value = 2
  } finally {
    authenticating.value = false
  }
}

const confirmAuth = () => {
  // 不自动关闭，让用户手动关闭或继续操作
  // visible.value = false
}

const handleClose = () => {
  visible.value = false
  // 不在关闭时重置状态，而是在打开时重置
}

const formatDateTime = (dateStr: string) => {
  return new Date(dateStr).toLocaleString('zh-CN')
}

// 生命周期
onMounted(() => {
  loadUsers()
})
</script>

<style scoped>
.auth-container {
  padding: 20px 0;
}

.step-content {
  margin-top: 30px;
  min-height: 300px;
}

.user-selection-card {
  min-height: 400px;
}

.user-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
  gap: 16px;
  margin-top: 20px;
}

.user-card {
  cursor: pointer;
  transition: all 0.3s ease;
  border: 2px solid transparent;
}

.user-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

.user-card.selected {
  border-color: var(--el-color-primary);
}

.user-info {
  display: flex;
  align-items: center;
  gap: 12px;
}

.user-avatar {
  background: var(--el-color-primary);
  color: white;
  font-weight: bold;
}

.user-details {
  flex: 1;
}

.user-name {
  font-size: 16px;
  font-weight: bold;
  color: var(--el-text-color-primary);
  margin-bottom: 4px;
}

.user-id, .user-level, .user-location {
  font-size: 12px;
  color: var(--el-text-color-secondary);
  margin-bottom: 2px;
}

.loading-message {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  padding: 40px;
  color: var(--el-text-color-secondary);
}

.verification-card {
  min-height: 400px;
}

.user-summary {
  margin-bottom: 20px;
}

.auth-tips {
  margin-top: 20px;
}

.result-card {
  min-height: 300px;
}

.result-content {
  padding: 20px;
}

.success-info {
  max-width: 400px;
  margin: 0 auto;
}

.dialog-footer {
  display: flex;
  justify-content: flex-end;
  gap: 12px;
}
</style>
