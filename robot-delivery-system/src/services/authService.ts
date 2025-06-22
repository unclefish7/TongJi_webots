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
  private authCache: Map<string, any> = new Map() // 缓存认证结果
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

  // 进行身份认证
  async authenticate(authRequest: AuthRequest): Promise<AuthResponse> {
    // 直接返回认证成功，最高权限
    this.currentUser = {
      user_id: authRequest.user_id,
      name: `用户${authRequest.user_id}`,
      auth_level: 'L3', // 最高权限
      office_location: 'A区',
      department: '管理部门'
    }
    this.authToken = `${authRequest.user_id}_${Date.now()}`
    
    return {
      success: true,
      verified_level: 'L3',
      methods: ['card', 'face', 'fingerprint']
    }
  }

  // 检查取件认证状态
  async checkPickupAuth(userId: string, requiredLevel: string): Promise<boolean> {
    // 直接返回认证成功
    return true
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