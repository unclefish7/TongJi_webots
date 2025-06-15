import axios from 'axios'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 用户接口定义
export interface User {
  user_id: string
  name: string
  auth_level: string
  office_location: string
  l2_auth?: string
  l3_auth?: string
}

// 认证请求接口
export interface AuthRequest {
  user_id: string
  purpose: 'send' | 'pickup'
  requested_level: string
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

// 认证服务类
export class AuthService {
  private static instance: AuthService
  private currentUser: User | null = null
  private authToken: string | null = null

  static getInstance(): AuthService {
    if (!AuthService.instance) {
      AuthService.instance = new AuthService()
    }
    return AuthService.instance
  }

  // 获取所有用户列表（用于前端用户选择）
  async getAllUsers(): Promise<User[]> {
    try {
      const response = await axios.get(`${API_BASE_URL}/users`)
      return response.data
    } catch (error) {
      console.error('获取用户列表失败:', error)
      throw error
    }
  }

  // 根据用户ID获取用户信息
  async getUserById(userId: string): Promise<User | null> {
    try {
      const response = await axios.get(`${API_BASE_URL}/users/${userId}`)
      return response.data
    } catch (error) {
      console.error('获取用户信息失败:', error)
      return null
    }
  }

  // 进行身份认证
  async authenticate(authRequest: AuthRequest): Promise<AuthResponse> {
    try {
      const response = await axios.post(`${API_BASE_URL}/api/verify`, authRequest)
      
      if (response.data.success) {
        // 认证成功，保存用户信息
        this.currentUser = await this.getUserById(authRequest.user_id)
        this.authToken = `${authRequest.user_id}_${Date.now()}`
      }
      
      return response.data
    } catch (error: any) {
      console.error('身份认证失败:', error)
      return {
        success: false,
        error: error.response?.data?.detail || '认证请求失败'
      }
    }
  }

  // 检查取件认证状态
  async checkPickupAuth(userId: string, requiredLevel: string): Promise<boolean> {
    try {
      const response = await axios.get(`${API_BASE_URL}/auth/pickup-status/${userId}`, {
        params: { required_level: requiredLevel }
      })
      return response.data.valid
    } catch (error) {
      console.error('检查取件认证失败:', error)
      return false
    }
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
    if (!this.currentUser) return false
    
    const levelOrder = { 'L1': 1, 'L2': 2, 'L3': 3 }
    const userLevel = levelOrder[this.currentUser.auth_level as keyof typeof levelOrder] || 0
    const required = levelOrder[requiredLevel as keyof typeof levelOrder] || 0
    
    return userLevel >= required
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
    const user = await service.getUserById(userId)
    
    if (!user) {
      return { success: false }
    }

    // 模拟认证成功
    service.setCurrentUser(user)
    
    return {
      success: true,
      userInfo: {
        name: user.name,
        level: user.auth_level as 'L1' | 'L2' | 'L3',
        department: user.office_location,
        expiry: new Date(Date.now() + 24 * 60 * 60 * 1000).toISOString()
      }
    }
  }

  // 静态权限检查方法（兼容性）
  static hasPermission(userLevel: string | null, requiredLevel: string): boolean {
    if (!userLevel) return false
    
    const levelOrder = { 'L1': 1, 'L2': 2, 'L3': 3 }
    const userLevelNum = levelOrder[userLevel as keyof typeof levelOrder] || 0
    const requiredLevelNum = levelOrder[requiredLevel as keyof typeof levelOrder] || 0
    
    return userLevelNum >= requiredLevelNum
  }
}

// 导出单例实例
export const authService = AuthService.getInstance()