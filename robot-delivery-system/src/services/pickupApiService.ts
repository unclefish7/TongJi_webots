import api from './api'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 取件相关的类型定义
export interface PickupTaskData {
  task_id: string
  description?: string
  security_level: string
  locker_id?: string
}

export interface PickupAuthInfo {
  authenticated: boolean
  level?: string
  expires_at?: string
  methods?: string[]
}

export interface PickupTasksResponse {
  success: boolean
  user_id: string
  total_tasks: number
  tasks: PickupTaskData[]
  auth_status: PickupAuthInfo
}

export interface PickupExecuteRequest {
  user_id: string
  task_id: string
}

export interface PickupExecuteResponse {
  success: boolean
  code: string
  message: string
  task_id?: string
}

// 取件服务类
export class PickupApiService {
  private static instance: PickupApiService

  static getInstance(): PickupApiService {
    if (!PickupApiService.instance) {
      PickupApiService.instance = new PickupApiService()
    }
    return PickupApiService.instance
  }

  /**
   * 查询可取件任务
   */
  async getPickupTasks(userId: string): Promise<PickupTasksResponse> {
    try {
      const response = await api.get('/api/pickup/tasks', {
        params: { user_id: userId }
      })
      return response.data
    } catch (error: any) {
      console.error('获取取件任务失败:', error)
      return {
        success: false,
        user_id: userId,
        total_tasks: 0,
        tasks: [],
        auth_status: {
          authenticated: false,
          methods: []
        }
      }
    }
  }

  /**
   * 执行取件操作
   */
  async executePickup(request: PickupExecuteRequest): Promise<PickupExecuteResponse> {
    try {
      const response = await api.post('/api/pickup/execute', request)
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        code: 'PICKUP_999',
        message: `执行取件操作失败: ${error.message}`
      }
    }
  }
}

// 导出单例实例
export const pickupApiService = PickupApiService.getInstance()
