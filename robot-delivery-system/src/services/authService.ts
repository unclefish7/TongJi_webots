import axios from 'axios'
import { authApiService } from './authApiService'
import { userApiService } from './userApiService'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 用户接口定义
export interface User {
  user_id: string
  name: string
  auth_level: string
  office_location?: string
  department?: string
  l2_auth?: string
  l3_auth?: string
  pickup_auth_info?: {
    verified_level: string
    started_at: string
    expires_at: string
    methods: string[]
  }
}

// 认证请求接口
export interface AuthRequest {
  user_id: string
  purpose: 'send' | 'pickup'
  requested_level: 'L1' | 'L2' | 'L3'
  provided: {
    l2_auth?: string
    l3_auth?: string
  }
}

// 认证响应接口
export interface AuthResponse {
  success: boolean
  verified_level?: string
  methods?: string[]
  expires_at?: string
  error?: string
}

// 用户信息接口（保持向后兼容）
export interface UserInfo {
  name: string
  level: 'L1' | 'L2' | 'L3'
  department: string
  expiry: string
}

// 认证缓存接口
export interface AuthCacheDetails {
  send_auth_cache: {
    [userId: string]: {
      total_records: number
      records: Array<{
        level: string
        used: boolean
        timestamp: string
        methods: string[]
      }>
    }
  }
  pickup_auth_cache: {
    [userId: string]: {
      verified_level: string
      started_at: string
      expires_at: string
      methods: string[]
    }
  }
  cache_summary: {
    send_cache_users: number
    pickup_cache_users: number
    send_cache_total_records: number
    send_cache_available_records: number
  }
}

// 认证服务类
export class AuthService {
  private static instance: AuthService
  private currentUser: User | null = null
  private authCache: Map<string, any> = new Map() // 缓存认证结果
  private authToken: string | null = null
  private authCacheDetails: AuthCacheDetails | null = null

  static getInstance(): AuthService {
    if (!AuthService.instance) {
      AuthService.instance = new AuthService()
    }
    return AuthService.instance
  }

  // 获取所有用户列表（用于前端用户选择）
  async getAllUsers(): Promise<User[]> {
    try {
      const response = await axios.get(`${API_BASE_URL}/api/user`)
      return response.data
    } catch (error) {
      console.error('获取用户列表失败:', error)
      throw error
    }
  }

  // 根据用户ID获取用户信息
  async getUserById(userId: string): Promise<User | null> {
    try {
      const response = await axios.get(`${API_BASE_URL}/api/user/${userId}`)
      return response.data
    } catch (error) {
      console.error('获取用户信息失败:', error)
      return null
    }
  }

  // 获取认证缓存详情
  async refreshAuthCacheDetails(): Promise<void> {
    try {
      const response = await authApiService.getCacheDetails()
      if (response.success) {
        this.authCacheDetails = response.data
      }
    } catch (error) {
      console.error('刷新认证缓存失败:', error)
    }
  }

  // 获取已认证的发送用户列表
  async getAuthenticatedSendUsers(): Promise<User[]> {
    await this.refreshAuthCacheDetails()
    
    if (!this.authCacheDetails) {
      return []
    }

    const authenticatedUsers: User[] = []
    const sendCache = this.authCacheDetails.send_auth_cache

    for (const userId in sendCache) {
      const userCache = sendCache[userId]
      // 检查是否有可用的认证记录
      const hasAvailableAuth = userCache.records.some(record => !record.used)
      
      if (hasAvailableAuth) {
        const user = await this.getUserById(userId)
        if (user) {
          authenticatedUsers.push(user)
        }
      }
    }

    return authenticatedUsers
  }

  // 获取已认证的取件用户列表
  async getAuthenticatedPickupUsers(): Promise<User[]> {
    await this.refreshAuthCacheDetails()
    
    if (!this.authCacheDetails) {
      return []
    }

    const authenticatedUsers: User[] = []
    const pickupCache = this.authCacheDetails.pickup_auth_cache

    for (const userId in pickupCache) {
      const userCache = pickupCache[userId]
      
      // 检查是否过期
      const expiresAt = new Date(userCache.expires_at)
      const now = new Date()
      
      if (now < expiresAt) {
        const user = await this.getUserById(userId)
        if (user) {
          // 添加认证信息到用户对象
          authenticatedUsers.push({
            ...user,
            pickup_auth_info: {
              verified_level: userCache.verified_level,
              started_at: userCache.started_at,
              expires_at: userCache.expires_at,
              methods: userCache.methods
            }
          })
        }
      }
    }

    return authenticatedUsers
  }

  // 检查用户是否有发送认证
  async hasSendAuth(userId: string): Promise<boolean> {
    await this.refreshAuthCacheDetails()
    
    if (!this.authCacheDetails) {
      return false
    }

    const userCache = this.authCacheDetails.send_auth_cache[userId]
    if (!userCache) {
      return false
    }

    // 检查是否有可用的认证记录
    return userCache.records.some(record => !record.used)
  }

  // 检查用户是否有取件认证
  async hasPickupAuth(userId: string): Promise<boolean> {
    await this.refreshAuthCacheDetails()
    
    if (!this.authCacheDetails) {
      return false
    }

    const userCache = this.authCacheDetails.pickup_auth_cache[userId]
    if (!userCache) {
      return false
    }

    // 检查是否过期
    const expiresAt = new Date(userCache.expires_at)
    const now = new Date()
    
    return now < expiresAt
  }

  // 获取用户的取件认证信息
  async getPickupAuthInfo(userId: string): Promise<any> {
    await this.refreshAuthCacheDetails()
    
    if (!this.authCacheDetails) {
      return null
    }

    const userCache = this.authCacheDetails.pickup_auth_cache[userId]
    if (!userCache) {
      return null
    }

    // 检查是否过期
    const expiresAt = new Date(userCache.expires_at)
    const now = new Date()
    
    if (now >= expiresAt) {
      return null
    }

    return userCache
  }

  // 进行身份认证
  async authenticate(authRequest: AuthRequest): Promise<AuthResponse> {
    try {
      const response = await authApiService.verifyPurposeAuth(authRequest)
      
      if (response.verified) {
        // 认证成功后刷新缓存
        await this.refreshAuthCacheDetails()
        
        return {
          success: true,
          verified_level: response.verified_level,
          methods: response.methods,
          expires_at: response.expires_at
        }
      } else {
        return {
          success: false,
          error: response.message
        }
      }
    } catch (error) {
      console.error('认证失败:', error)
      return {
        success: false,
        error: '认证过程中发生错误'
      }
    }
  }

  // 检查取件认证状态
  async checkPickupAuth(userId: string, requiredLevel: string): Promise<boolean> {
    const hasAuth = await this.hasPickupAuth(userId)
    if (!hasAuth) {
      return false
    }

    const authInfo = await this.getPickupAuthInfo(userId)
    if (!authInfo) {
      return false
    }

    // 检查认证等级是否满足要求
    const levelOrder = { 'L1': 1, 'L2': 2, 'L3': 3 }
    const userLevel = levelOrder[authInfo.verified_level as keyof typeof levelOrder] || 0
    const requiredLevelNum = levelOrder[requiredLevel as keyof typeof levelOrder] || 0

    return userLevel >= requiredLevelNum
  }

  // 获取当前用户
  getCurrentUser(): User | null {
    return this.currentUser
  }

  // 获取认证令牌
  getAuthToken(): string | null {
    return this.authToken
  }

  // 检查是否已认证
  isAuthenticated(): boolean {
    return this.currentUser !== null && this.authToken !== null
  }

  // 登出
  logout(): void {
    this.currentUser = null
    this.authToken = null
  }

  // 获取用户办公地点
  getUserLocation(): string | null {
    return this.currentUser?.office_location || null
  }

  // 检查用户权限等级
  hasAuthLevel(requiredLevel: string): boolean {
    // 直接返回最高权限
    return true
  }

  // 设置当前用户（用于演示模式）
  setCurrentUser(user: User): void {
    this.currentUser = user
    this.authToken = `${user.user_id}_${Date.now()}`
  }

  // 兼容性方法 - 保持原有接口
  static async verifyAuth(request: AuthRequest): Promise<AuthResponse> {
    const service = AuthService.getInstance()
    return service.authenticate(request)
  }

  // 兼容性方法 - 模拟认证
  static async performAuth(
    method: 'card' | 'face' | 'fingerprint',
    userId: string
  ): Promise<{ success: boolean; userInfo?: UserInfo }> {
    const service = AuthService.getInstance()
    
    // 直接设置最高权限用户
    const user: User = {
      user_id: userId,
      name: `用户${userId}`,
      auth_level: 'L3',
      office_location: 'A区',
      department: '管理部门'
    }
    
    service.setCurrentUser(user)
    
    return {
      success: true,
      userInfo: {
        name: user.name,
        level: 'L3',
        department: user.department || '管理部门',
        expiry: new Date(Date.now() + 24 * 60 * 60 * 1000).toISOString()
      }
    }
  }

  // 静态权限检查方法（兼容性）
  static hasPermission(userLevel: string | null, requiredLevel: string): boolean {
    // 直接返回最高权限
    return true
  }
}

// 导出单例实例
export const authService = AuthService.getInstance()