import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { authService, type User } from '@/services/authService'

export const useUserStore = defineStore('user', () => {
  // 状态
  const authenticatedUsers = ref<User[]>([])
  const selectedUserId = ref<string>('')
  const loading = ref(false)

  // 计算属性
  const isAuthenticated = computed(() => selectedUserId.value !== '')
  const currentUser = computed(() => {
    return authenticatedUsers.value.find(user => user.user_id === selectedUserId.value) || null
  })

  // 获取认证等级类型
  const getAuthLevelType = (level: string) => {
    const types = {
      'L1': 'success',
      'L2': 'warning', 
      'L3': 'danger'
    }
    return types[level as keyof typeof types] || 'info'
  }

  // 刷新已认证用户列表
  const refreshAuthenticatedUsers = async () => {
    loading.value = true
    try {
      console.log('开始刷新认证用户列表...')
      const users = await authService.getAuthenticatedSendUsers()
      console.log('获取到的认证用户:', users)
      authenticatedUsers.value = users
      
      // 如果当前选中的用户仍在已认证用户列表中，保持选择
      if (selectedUserId.value && users.some(user => user.user_id === selectedUserId.value)) {
        console.log('当前选中用户仍在认证列表中，保持选择')
        // 保持原有选择，同时同步到 authService
        const user = users.find(u => u.user_id === selectedUserId.value)
        if (user) {
          authService.setCurrentUser(user)
        }
      } else if (users.length > 0) {
        // 如果当前没有选中用户，或选中的用户不在列表中，选择第一个
        selectedUserId.value = users[0].user_id
        authService.setCurrentUser(users[0])
        console.log('自动选择第一个认证用户:', users[0].name)
      } else {
        // 没有任何认证用户
        selectedUserId.value = ''
        console.log('没有认证用户')
      }
    } catch (error) {
      console.error('刷新认证用户列表失败:', error)
    } finally {
      loading.value = false
    }
  }

  // 切换用户
  const switchUser = (userId: string) => {
    const user = authenticatedUsers.value.find(u => u.user_id === userId)
    if (user) {
      selectedUserId.value = userId
      authService.setCurrentUser(user)
      console.log('切换到用户:', user.name)
    }
  }

  // 添加认证成功后的处理
  const handleAuthSuccess = async (user: User) => {
    console.log('用户认证成功:', user)
    
    // 先刷新已认证用户列表
    await refreshAuthenticatedUsers()
    
    // 认证成功后自动切换到新用户
    if (user) {
      selectedUserId.value = user.user_id
      authService.setCurrentUser(user)
      console.log('自动切换到新认证用户:', user.name)
    }
  }

  // 初始化
  const initialize = async () => {
    await refreshAuthenticatedUsers()
    
    // 如果有已认证用户且当前没有选中用户，自动选择第一个
    if (authenticatedUsers.value.length > 0 && !selectedUserId.value) {
      selectedUserId.value = authenticatedUsers.value[0].user_id
      authService.setCurrentUser(authenticatedUsers.value[0])
      console.log('自动选择第一个已认证用户:', authenticatedUsers.value[0].name)
    }
  }

  return {
    // 状态
    authenticatedUsers,
    selectedUserId,
    loading,
    
    // 计算属性
    isAuthenticated,
    currentUser,
    
    // 方法
    getAuthLevelType,
    refreshAuthenticatedUsers,
    switchUser,
    handleAuthSuccess,
    initialize
  }
})
