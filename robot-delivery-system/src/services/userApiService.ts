import api from './api'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 用户相关类型定义
export interface User {
  user_id: string
  name: string
  auth_level: string
  department?: string
  role?: string
  phone?: string
  email?: string
}

// 用户API服务类
export class UserApiService {
  private static instance: UserApiService

  static getInstance(): UserApiService {
    if (!UserApiService.instance) {
      UserApiService.instance = new UserApiService()
    }
    return UserApiService.instance
  }

  /**
   * 获取所有用户列表
   */
  async getAllUsers(): Promise<User[]> {
    try {
      const response = await api.get('/api/user')
      return response.data
    } catch (error: any) {
      console.error('获取用户列表失败:', error)
      return []
    }
  }

  /**
   * 根据用户ID获取用户信息
   */
  async getUserById(userId: string): Promise<User | null> {
    try {
      const response = await api.get(`/api/user/${userId}`)
      return response.data
    } catch (error: any) {
      if (error.response?.status === 404) {
        return null
      }
      console.error('获取用户信息失败:', error)
      return null
    }
  }
}

// 导出单例实例
export const userApiService = UserApiService.getInstance()
