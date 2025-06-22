import api from './api'

// 柜门状态相关的类型定义
export interface LockerStatus {
  locker_id: string
  status: 'available' | 'occupied' | 'maintenance' | 'reserved'
  task_id?: string
  user_id?: string
  security_level?: string
  updated_at?: string
}

export interface LockerStatusResponse {
  success: boolean
  total_lockers: number
  lockers: LockerStatus[]
  message: string
}

// 柜门状态API服务类
export class LockerApiService {
  private static instance: LockerApiService

  static getInstance(): LockerApiService {
    if (!LockerApiService.instance) {
      LockerApiService.instance = new LockerApiService()
    }
    return LockerApiService.instance
  }

  // 获取所有柜门状态
  async getLockerStatus(): Promise<LockerStatusResponse> {
    try {
      const response = await api.get('/api/lockers/status')
      return response.data
    } catch (error: any) {
      console.error('获取柜门状态失败:', error)
      return {
        success: false,
        total_lockers: 0,
        lockers: [],
        message: error.response?.data?.detail || '获取柜门状态失败'
      }
    }
  }
}

// 导出单例实例
export const lockerApiService = LockerApiService.getInstance()
