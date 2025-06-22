import api from './api'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 认证相关类型定义
export interface AuthRequest {
  user_id: string
  requested_level: 'L1' | 'L2' | 'L3'
  provided?: {
    l2_auth?: string
    l3_auth?: string
  }
}

export interface AuthResponse {
  success: boolean
  verified_level?: string
  methods?: string[]
  expires_at?: string
  error?: string
}

export interface PurposeAuthRequest {
  user_id: string
  purpose: 'send' | 'pickup'
  requested_level: 'L1' | 'L2' | 'L3'
  provided?: {
    l2_auth?: string
    l3_auth?: string
  }
}

export interface PurposeAuthResponse {
  verified: boolean
  verified_level?: string
  methods?: string[]
  expires_at?: string
  message?: string
}

// 认证API服务类
export class AuthApiService {
  private static instance: AuthApiService

  static getInstance(): AuthApiService {
    if (!AuthApiService.instance) {
      AuthApiService.instance = new AuthApiService()
    }
    return AuthApiService.instance
  }

  /**
   * 基于指定认证等级的用户认证验证
   */
  async verifyAuth(request: AuthRequest): Promise<AuthResponse> {
    try {
      const response = await api.post('/api/auth/verify', request)
      return response.data
    } catch (error: any) {
      console.error('认证验证失败:', error)
      return {
        success: false,
        error: error.response?.data?.detail || '认证失败'
      }
    }
  }

  /**
   * 基于用途的用户认证验证
   */
  async verifyPurposeAuth(request: PurposeAuthRequest): Promise<PurposeAuthResponse> {
    try {
      const response = await api.post('/api/auth/verify_purpose', request)
      return response.data
    } catch (error: any) {
      console.error('用途认证验证失败:', error)
      return {
        verified: false,
        message: error.response?.data?.detail || '认证失败'
      }
    }
  }

  /**
   * 获取认证缓存状态
   */
  async getCacheStatus(): Promise<any> {
    // 直接返回模拟的缓存状态
    return {
      success: true,
      data: {
        auth_session_cache: {},
        pickup_auth_cache: {}
      }
    }
  }
}

// 导出单例实例
export const authApiService = AuthApiService.getInstance()
