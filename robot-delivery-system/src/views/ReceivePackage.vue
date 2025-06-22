<template>
  <div class="receive-package-container">
    <el-page-header @back="$router.back()">
      <template #content>
        <span class="page-title">取件服务</span>
      </template>
    </el-page-header>

    <div class="receive-content">
      <!-- 身份认证步骤 -->
      <el-card v-if="!isAuthenticated" class="auth-card">
        <template #header>
          <div class="card-header">
            <el-icon><Lock /></el-icon>
            <span>取件身份认证</span>
          </div>
        </template>
        <div class="auth-prompt">
          <el-empty description="请先进行取件身份认证以查看您的包裹">
            <template #description>
              <div>
                <p>取件需要进行专门的身份认证</p>
                <p>认证有效期：30分钟</p>
              </div>
            </template>
            <el-button type="primary" @click="showAuthModal = true">
              <el-icon><Lock /></el-icon>
              开始取件认证
            </el-button>
          </el-empty>
        </div>
      </el-card>
      <!-- 认证成功后的内容 -->
      <template v-else>
        <!-- 认证状态提示 -->
        <el-card class="auth-info-card" v-if="pickupAuthInfo">
          <div class="auth-info">
            <div class="auth-user">
              <el-avatar :size="32">{{ currentUser?.name?.charAt(0) }}</el-avatar>
              <div class="user-details">
                <span class="user-name">{{ currentUser?.name }}</span>
                <el-tag :type="getAuthLevelType(pickupAuthInfo.verified_level)" size="small">
                  {{ pickupAuthInfo.verified_level }} 级取件权限
                </el-tag>
              </div>
            </div>
            <div class="auth-time">
              <span>认证时间: {{ authTime }}</span>
              <span v-if="pickupAuthInfo.expires_at">
                有效期至: {{ new Date(pickupAuthInfo.expires_at).toLocaleString('zh-CN') }}
              </span>
            </div>
            <el-button @click="showAuthModal = true" size="small" type="info" plain>
              重新认证
            </el-button>
          </div>
        </el-card>
        
        <div class="content-layout">
          <!-- 左侧包裹列表 -->
          <div class="package-section">
            <el-card class="package-list-card">
              <template #header>
                <div class="card-header">
                  <div class="header-left">
                    <el-icon><Box /></el-icon>
                    <span>我的包裹 ({{ userPackages.length }})</span>
                  </div>
                  <el-button
                    @click="refreshPackages"
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

              <div v-if="userPackages.length === 0" class="no-packages">
                <el-empty description="暂无待取包裹">
                  <el-button @click="$router.push('/send')">去寄件</el-button>
                </el-empty>
              </div>

              <div v-else class="package-list">
                <div
                  v-for="pkg in userPackages"
                  :key="pkg.id"
                  class="package-item"
                  :class="{ selected: selectedPackage?.id === pkg.id }"
                  @click="selectPackage(pkg)"
                >
                  <div class="package-info">
                    <div class="package-header">
                      <div class="package-id">包裹 #{{ pkg.id.toUpperCase() }}</div>
                      <el-tag :type="getLevelType(pkg.securityLevel)" size="small">
                        {{ pkg.securityLevel }}
                      </el-tag>
                    </div>

                    <div class="package-details">
                      <div class="detail-row">
                        <el-icon><Location /></el-icon>
                        <span>柜门位置: {{ pkg.floor }}F-{{ pkg.compartmentNumber }}</span>
                      </div>
                      <div class="detail-row">
                        <el-icon><Clock /></el-icon>
                        <span>到达时间: {{ pkg.arrivalTime }}</span>
                      </div>
                      <div v-if="pkg.content" class="detail-row">
                        <el-icon><Document /></el-icon>
                        <span>包裹内容: {{ pkg.content }}</span>
                      </div>
                      <div v-if="pkg.sender" class="detail-row">
                        <el-icon><User /></el-icon>
                        <span>寄件人: {{ pkg.sender }}</span>
                      </div>
                    </div>

                    <div class="package-actions">
                      <el-button
                        type="primary"
                        size="small"
                        @click.stop="openCompartment(pkg)"
                        :loading="pkg.id === openingPackageId"
                        class="action-btn"
                      >
                        <el-icon><Unlock /></el-icon>
                        取件
                      </el-button>
                      <el-button
                        size="small"
                        @click.stop="viewPackageDetails(pkg)"
                        class="action-btn"
                      >
                        详情
                      </el-button>
                    </div>
                  </div>
                </div>
              </div>
            </el-card>
          </div>

          <!-- 右侧状态面板 -->
          <div class="status-section">
            <!-- 用户信息 -->
            <el-card class="user-info-card">
              <template #header>
                <div class="card-header">
                  <div class="header-left">
                    <el-icon><User /></el-icon>
                    <span>用户信息</span>
                  </div>
                </div>
              </template>
              <el-descriptions :column="1" size="small">
                <el-descriptions-item label="认证等级">
                  <el-tag :type="getLevelType(userAuthLevel || 'L1')">
                    {{ userAuthLevel }}
                  </el-tag>
                </el-descriptions-item>
                <el-descriptions-item label="可取包裹">{{
                  userPackages.length
                }}</el-descriptions-item>
                <el-descriptions-item label="认证时间">{{ authTime }}</el-descriptions-item>
              </el-descriptions>
              <el-button
                @click="showAuthModal = true"
                size="small"
                type="info"
                plain
                style="margin-top: 1vh"
                class="reauth-btn"
              >
                重新认证
              </el-button>
            </el-card>

            

            <!-- 取件说明 -->
            <el-card class="help-card">
              <template #header>
                <div class="card-header">
                  <div class="header-left">
                    <el-icon><QuestionFilled /></el-icon>
                    <span>取件说明</span>
                  </div>
                </div>
              </template>
              <div class="help-content">
                <el-steps direction="vertical" :active="4" finish-status="success">
                  <el-step title="身份认证" description="使用员工卡/人脸/指纹进行身份认证" />
                  <el-step title="查看包裹" description="系统显示您可取的包裹列表" />
                  <el-step title="选择包裹" description="点击要取的包裹" />
                  <el-step title="开启柜门" description="系统自动开启对应柜门" />
                  <el-step title="取出包裹" description="取出包裹并关闭柜门" />
                </el-steps>
              </div>
            </el-card>
          </div>
        </div>
      </template>
    </div>

    <!-- 身份认证弹窗 -->
    <UserAuthModal
      v-model="showAuthModal"
      purpose="pickup"
      required-level="L1"
      @auth-success="handleAuthSuccess"
    />

    <!-- 取件成功对话框 -->
    <el-dialog
      v-model="showSuccessDialog"
      title="取件成功"
      width="400px"
      :before-close="handleSuccessClose"
    >
      <el-result icon="success" title="包裹取出成功">
        <template #sub-title>
          <div class="success-info">
            <p>
              柜门 <strong>{{ successCompartment }}</strong> 已打开
            </p>
            <p>请取出您的包裹并关闭柜门</p>
          </div>
        </template>
        <template #extra>
          <el-button type="primary" @click="handleSuccessClose">确定</el-button>
        </template>
      </el-result>
    </el-dialog>

    <!-- 包裹详情对话框 -->
    <el-dialog v-model="showDetailsDialog" title="包裹详情" width="500px">
      <div v-if="selectedPackageDetails" class="package-details-content">
        <el-descriptions title="包裹信息" :column="2" border>
          <el-descriptions-item label="包裹编号">{{
            selectedPackageDetails.id.toUpperCase()
          }}</el-descriptions-item>
          <el-descriptions-item label="安全等级">
            <el-tag :type="getLevelType(selectedPackageDetails.securityLevel)">
              {{ selectedPackageDetails.securityLevel }}
            </el-tag>
          </el-descriptions-item>
          <el-descriptions-item label="柜门位置">
            {{ selectedPackageDetails.floor }}F-{{ selectedPackageDetails.compartmentNumber }}
          </el-descriptions-item>
          <el-descriptions-item label="到达时间">{{
            selectedPackageDetails.arrivalTime
          }}</el-descriptions-item>
          <el-descriptions-item label="包裹内容" :span="2">{{
            selectedPackageDetails.content
          }}</el-descriptions-item>
          <el-descriptions-item label="寄件人" :span="2">{{
            selectedPackageDetails.sender
          }}</el-descriptions-item>
        </el-descriptions>

        <el-timeline style="margin-top: 20px">
          <el-timeline-item
            v-for="(activity, index) in selectedPackageDetails.timeline"
            :key="index"
            :timestamp="activity.timestamp"
            :type="activity.type"
          >
            {{ activity.content }}
          </el-timeline-item>
        </el-timeline>
      </div>
      <template #footer>
        <el-button @click="showDetailsDialog = false">关闭</el-button>
        <el-button type="primary" @click="openCompartmentFromDetails">取件</el-button>
      </template>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted } from 'vue'
import { useRouter } from 'vue-router'
import { ElMessage, ElMessageBox } from 'element-plus'
import { useRobotStore } from '@/stores/robot'
import { authService, type User } from '@/services/authService'
import { pickupApiService } from '@/services/pickupApiService'
import { systemStatusService } from '@/services/systemStatusService'
import UserAuthModal from '@/components/UserAuthModal.vue'
import {
  Lock,
  Box,
  Refresh,
  Location,
  Clock,
  Document,
  User as UserIcon,
  Unlock,
  Grid,
  QuestionFilled,
  Loading,
} from '@element-plus/icons-vue'

const router = useRouter()
const robotStore = useRobotStore()

// 状态
const showAuthModal = ref(false)
const showSuccessDialog = ref(false)
const showDetailsDialog = ref(false)
const selectedPackage = ref<any>(null)
const selectedPackageDetails = ref<any>(null)
const openingPackageId = ref('')
const openingCompartmentId = ref('')
const successCompartment = ref('')
const authTime = ref('')
const currentPickupUser = ref<User | null>(null)
const pickupAuthInfo = ref<any>(null)

// 模拟用户包裹数据（这将从后端API获取）
const userPackages = ref<any[]>([])

// 计算属性 - 基于取件认证缓存
const isAuthenticated = computed(() => currentPickupUser.value !== null)
const userAuthLevel = computed(() => currentPickupUser.value?.auth_level || null)
const currentUser = computed(() => currentPickupUser.value)

// 检查取件认证状态
const checkPickupAuth = async () => {
  // 这里可以实现检查多个用户的取件认证
  // 暂时简化为检查当前选中用户
  if (currentPickupUser.value) {
    const hasAuth = await authService.hasPickupAuth(currentPickupUser.value.user_id)
    if (!hasAuth) {
      currentPickupUser.value = null
      pickupAuthInfo.value = null
    } else {
      pickupAuthInfo.value = await authService.getPickupAuthInfo(currentPickupUser.value.user_id)
    }
  }
}

// 初始化和数据加载
onMounted(async () => {
  // 检查是否有取件认证
  await checkPickupAuth()
  
  if (currentPickupUser.value) {
    await refreshPackages()
  }
})

// 方法
const getLevelType = (level: string) => {
  const types = {
    L1: 'success',
    L2: 'warning',
    L3: 'danger',
  }
  return types[level as keyof typeof types] || 'info'
}

const getAuthLevelType = (level: string) => {
  const types = {
    'L1': 'success',
    'L2': 'warning', 
    'L3': 'danger'
  }
  return types[level as keyof typeof types] || 'info'
}

const isUserPackage = (compartment: any) => {
  return userPackages.value.some((pkg) => pkg.id === compartment.id && compartment.isOccupied)
}

const selectPackage = (pkg: any) => {
  selectedPackage.value = pkg
}

const handleAuthSuccess = async (user: User, authResult: any) => {
  console.log('取件认证成功:', user, authResult)
  
  // 设置当前取件用户
  currentPickupUser.value = user
  pickupAuthInfo.value = authResult
  authTime.value = new Date().toLocaleString('zh-CN')
  
  showAuthModal.value = false
  
  // 刷新包裹列表
  await refreshPackages()
  
  ElMessage.success(`取件认证成功，${user.name} 可以进行取件操作`)
}

const refreshPackages = async () => {
  if (!currentPickupUser.value) {
    ElMessage.warning('请先进行取件认证')
    return
  }

  try {
    // 检查认证是否仍然有效
    await checkPickupAuth()
    
    if (!currentPickupUser.value) {
      ElMessage.warning('认证已过期，请重新认证')
      showAuthModal.value = true
      return
    }

    const user = currentPickupUser.value
    
    // 获取用户的取件任务列表
    const response = await pickupApiService.getPickupTasks(user.user_id)
    
    if (response.success) {
      // 将取件任务转换为用户包裹格式
      userPackages.value = response.tasks.map(task => ({
        id: task.task_id,
        floor: 1, // 模拟楼层
        compartmentNumber: task.locker_id || 'A1',
        securityLevel: task.security_level,
        content: task.description || '包裹',
        sender: '未知', // 新API中没有sender信息，可能需要额外获取
        arrivalTime: new Date().toLocaleString('zh-CN'), // 新API中没有时间信息
        timeline: [
          { timestamp: new Date().toLocaleString('zh-CN'), content: '包裹已寄出', type: 'primary' },
          { timestamp: new Date().toLocaleString('zh-CN'), content: '包裹已到达', type: 'success' },
        ],
      }))
    } else {
      userPackages.value = []
      ElMessage.warning('获取取件任务失败')
    }
    
    ElMessage.success('包裹列表已刷新')
  } catch (error) {
    console.error('刷新包裹列表失败:', error)
    ElMessage.error('刷新包裹列表失败')
  }
}

const openCompartment = async (pkg: any) => {
  try {
    await ElMessageBox.confirm(
      `确认要取出柜门 ${pkg.floor}F-${pkg.compartmentNumber} 的包裹吗？`,
      '确认取件',
      {
        confirmButtonText: '确认',
        cancelButtonText: '取消',
        type: 'warning',
      },
    )

    openingPackageId.value = pkg.id
    openingCompartmentId.value = pkg.id

    const user = currentUser.value
    if (!user) {
      ElMessage.error('用户信息异常')
      return
    }

    // 执行取件操作
    const executeRequest = {
      user_id: user.user_id,
      task_id: pkg.id
    }

    const result = await pickupApiService.executePickup(executeRequest)
    
    if (result.success) {
      // 更新柜门状态
      if (pkg.compartmentNumber) {
        systemStatusService.updateLockerStatus(
          pkg.compartmentNumber,
          'available'
        )
      }

      successCompartment.value = `${pkg.floor}F-${pkg.compartmentNumber}`
      showSuccessDialog.value = true

      // 从用户包裹列表中移除
      const index = userPackages.value.findIndex((p) => p.id === pkg.id)
      if (index > -1) {
        userPackages.value.splice(index, 1)
      }

      ElMessage.success(result.message || '取件成功')
    } else {
      ElMessage.error(result.message || '取件失败')
    }
  } catch (error) {
    console.error('取件失败:', error)
    ElMessage.error('取件失败，请重试')
  } finally {
    openingPackageId.value = ''
    openingCompartmentId.value = ''
  }
}

const viewPackageDetails = (pkg: any) => {
  selectedPackageDetails.value = pkg
  showDetailsDialog.value = true
}

const openCompartmentFromDetails = () => {
  showDetailsDialog.value = false
  if (selectedPackageDetails.value) {
    openCompartment(selectedPackageDetails.value)
  }
}

const handleSuccessClose = () => {
  showSuccessDialog.value = false
}
</script>

<style scoped>
.receive-package-container {
  padding: 2vw;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  min-height: 100vh;
  width: 100vw;
}

.page-title {
  font-size: 1.4vw;
  font-weight: 600;
  color: white;
  text-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
}

.receive-content {
  margin-top: 2vh;
}

/* 认证信息卡片样式 */
.auth-info-card {
  margin-bottom: 1.5vh;
  border-radius: 12px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

.auth-info {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 10px;
}

.auth-user {
  display: flex;
  align-items: center;
  gap: 12px;
}

.user-details {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.user-name {
  font-weight: 600;
  color: #303133;
}

.auth-time {
  display: flex;
  flex-direction: column;
  gap: 4px;
  color: #606266;
  font-size: 14px;
}

/* 用户选项样式 */
.user-option {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 8px;
  width: 100%;
}

.content-layout {
  display: grid;
  grid-template-columns: 1fr 25vw;
  gap: 1.5vw;
}

.package-section {
  grid-column: 1;
}

.status-section {
  grid-column: 2;
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
}

.auth-card,
.package-list-card,
.user-info-card,
.compartment-status-card,
.help-card {
  background: rgba(255, 255, 255, 0.95);
  backdrop-filter: blur(10px);
  border: none;
  border-radius: 16px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;
  animation: slideInUp 0.6s ease-out;
}

.auth-card:hover,
.package-list-card:hover,
.user-info-card:hover,
.compartment-status-card:hover,
.help-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15);
}

.card-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  font-weight: 600;
}

.header-left {
  display: flex;
  align-items: center;
  gap: 0.8vw;
  font-size: 1vw;
  background: linear-gradient(45deg, #667eea, #764ba2);
  background-clip: text;
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}

.header-left .el-icon {
  color: #667eea;
  font-size: 1.2vw;
}

.refresh-btn {
  border-radius: 8px;
  font-size: 0.8vw;
  transition: all 0.3s ease;
}

.refresh-btn:hover {
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(64, 158, 255, 0.3);
}

.auth-prompt {
  padding: 4vh 2vw;
  text-align: center;
}

.no-packages {
  padding: 4vh 2vw;
  text-align: center;
}

.package-list {
  display: flex;
  flex-direction: column;
  gap: 1.5vh;
}

.package-item {
  padding: 2vh 1.5vw;
  border: 2px solid #e4e7ed;
  border-radius: 12px;
  background: linear-gradient(135deg, #ffffff 0%, #f8f9ff 100%);
  cursor: pointer;
  transition: all 0.3s ease;
  animation: fadeInScale 0.5s ease-out;
}

.package-item:hover {
  border-color: #409eff;
  box-shadow: 0 4px 16px rgba(64, 158, 255, 0.2);
  transform: translateY(-2px);
}

.package-item.selected {
  border-color: #409eff;
  background: linear-gradient(135deg, #f0f9ff 0%, #e0f2fe 100%);
  box-shadow: 0 6px 20px rgba(64, 158, 255, 0.3);
  transform: translateY(-2px);
}

.package-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1.2vh;
}

.package-id {
  font-weight: 600;
  font-size: 1vw;
  color: #303133;
}

.package-details {
  margin-bottom: 1.5vh;
}

.detail-row {
  display: flex;
  align-items: center;
  gap: 0.8vw;
  margin-bottom: 0.6vh;
  font-size: 0.8vw;
  color: #606266;
  transition: color 0.3s ease;
}

.package-item:hover .detail-row {
  color: #333;
}

.package-actions {
  display: flex;
  gap: 1vw;
}

.action-btn {
  border-radius: 8px;
  font-size: 0.8vw;
  transition: all 0.3s ease;
}

.action-btn:hover {
  transform: translateY(-1px);
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
  position: relative;
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

.compartment-item.user-package {
  background: linear-gradient(135deg, #f0f9ff 0%, #dbeafe 100%);
  border-color: #409eff;
  box-shadow: 0 4px 16px rgba(64, 158, 255, 0.2);
  animation: pulse 2s infinite;
}

.compartment-item.opening {
  background: linear-gradient(135deg, #e1f3d8 0%, #dcfce7 100%);
  border-color: #67c23a;
}

.compartment-number {
  font-weight: 600;
  font-size: 0.8vw;
  margin-bottom: 0.6vh;
}

.compartment-level {
  margin-bottom: 0.6vh;
}

.compartment-status {
  font-size: 0.7vw;
}

.occupied-text {
  color: #f56c6c;
}

.available-text {
  color: #909399;
}

.opening-indicator {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background: rgba(103, 194, 58, 0.9);
  color: white;
  font-size: 0.7vw;
  border-radius: 12px;
  animation: pulse 1s infinite;
}

.help-content {
  padding: 1vh 0;
}

.reauth-btn {
  border-radius: 8px;
  font-size: 0.8vw;
  transition: all 0.3s ease;
}

.reauth-btn:hover {
  transform: translateY(-1px);
}

.success-info {
  text-align: center;
  margin: 2vh 0;
  font-size: 0.9vw;
}

.package-details-content {
  max-height: 50vh;
  overflow-y: auto;
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
    box-shadow: 0 4px 16px rgba(64, 158, 255, 0.2);
  }
  50% {
    box-shadow: 0 6px 24px rgba(64, 158, 255, 0.4);
  }
}

/* 响应式设计 */
@media (max-width: 1200px) {
  .content-layout {
    grid-template-columns: 1fr;
    gap: 2vh;
  }

  .status-section {
    grid-column: 1;
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 1.5vw;
  }

  .page-title {
    font-size: 2.5vw;
  }

  .header-left {
    font-size: 1.5vw;
  }

  .package-id {
    font-size: 1.5vw;
  }

  .detail-row {
    font-size: 1.2vw;
  }
}

@media (max-width: 768px) {
  .receive-package-container {
    padding: 3vw;
  }

  .page-title {
    font-size: 4vw;
  }

  .status-section {
    grid-template-columns: 1fr;
  }

  .compartment-grid {
    grid-template-columns: 1fr;
  }

  .package-actions {
    flex-direction: column;
    gap: 1vh;
  }
}

/* Element Plus 组件覆盖样式 */
:deep(.el-card__header) {
  background: linear-gradient(135deg, #f8f9ff 0%, #ffffff 100%);
  border-bottom: 1px solid rgba(102, 126, 234, 0.1);
}

:deep(.el-button) {
  border-radius: 8px;
  transition: all 0.3s ease;
}

:deep(.el-button:hover) {
  transform: translateY(-1px);
}

:deep(.el-tag) {
  border-radius: 12px;
  font-weight: 500;
}

:deep(.el-descriptions__label) {
  font-weight: 600;
  font-size: 0.8vw;
}

:deep(.el-descriptions__content) {
  font-size: 0.8vw;
}

:deep(.el-step__title) {
  font-size: 0.8vw;
}

:deep(.el-step__description) {
  font-size: 0.7vw;
}

:deep(.el-empty__description p) {
  font-size: 0.9vw;
}
</style>
