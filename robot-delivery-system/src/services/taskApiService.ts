import api from './api'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 新任务接口类型定义
export interface TaskApiRequest {
  user_id: string
  receiver: string
  location_id: string
  security_level: 'L1' | 'L2' | 'L3'
  description?: string
}

export interface TaskApiResponse {
  success: boolean
  code: string
  message: string
  task_id?: string
  task?: TaskData
  locker_id?: string
}

export interface TaskData {
  task_id: string
  user_id: string
  receiver: string
  location_id: string
  security_level: string
  description?: string
  status: string
  created_at: string
  updated_at?: string
  locker_id?: string
}

export interface QueueStatusResponse {
  success: boolean
  data: {
    total_tasks: number
    pending_tasks: number
    executing_tasks: number
    high_priority_count: number
    medium_priority_count: number
    low_priority_count: number
    current_executing_task?: TaskData
    next_pending_task?: TaskData
    queue_summary: {
      high_priority: TaskData[]
      medium_priority: TaskData[]
      low_priority: TaskData[]
    }
  }
}

export interface ROS2StatusResponse {
  success: boolean
  data: {
    available: boolean
    status: string
    bridge_url: string
  }
}

// 新任务API服务类
export class TaskApiService {
  private static instance: TaskApiService

  static getInstance(): TaskApiService {
    if (!TaskApiService.instance) {
      TaskApiService.instance = new TaskApiService()
    }
    return TaskApiService.instance
  }

  /**
   * 创建新任务
   */
  async createTask(request: TaskApiRequest): Promise<TaskApiResponse> {
    try {
      const response = await api.post('/api/tasks/create', request)
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        code: 'TASK_999',
        message: `创建任务失败: ${error.message}`
      }
    }
  }

  /**
   * 完成任务
   */
  async completeTask(taskId: string): Promise<TaskApiResponse> {
    try {
      const response = await api.post('/api/tasks/complete', {
        task_id: taskId
      })
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        code: 'TASK_999',
        message: `完成任务失败: ${error.message}`
      }
    }
  }

  /**
   * 标记任务失败
   */
  async failTask(taskId: string, reason?: string): Promise<TaskApiResponse> {
    try {
      const response = await api.post('/api/tasks/fail', {
        task_id: taskId,
        reason
      })
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        code: 'TASK_999',
        message: `标记任务失败: ${error.message}`
      }
    }
  }

  /**
   * 取消任务（从队列中移除）
   */
  async cancelTask(taskId: string): Promise<{ success: boolean; code: string; message: string }> {
    try {
      const response = await api.post(`/api/tasks/cancel/${taskId}`)
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        code: 'TASK_999',
        message: `取消任务失败: ${error.message}`
      }
    }
  }

  /**
   * 启动队列中的下一个任务
   */
  async startTask(): Promise<{ success: boolean; code: string; message: string }> {
    try {
      const response = await api.post('/api/tasks/start')
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        code: 'TASK_999',
        message: `启动任务失败: ${error.message}`
      }
    }
  }

  /**
   * 机器人到达通知
   */
  async robotArrived(): Promise<{ success: boolean; message: string }> {
    try {
      const response = await api.post('/api/tasks/robot/arrived')
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        message: `机器人到达通知失败: ${error.message}`
      }
    }
  }

  /**
   * 发送next指令让机器人继续
   */
  async sendNext(): Promise<{ success: boolean; message: string }> {
    try {
      const response = await api.post('/api/tasks/robot/next')
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        message: `发送next指令失败: ${error.message}`
      }
    }
  }

  /**
   * 获取当前任务队列状态
   */
  async getQueueStatus(): Promise<QueueStatusResponse> {
    try {
      const response = await api.get('/api/tasks/queue/status')
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        data: {
          total_tasks: 0,
          pending_tasks: 0,
          executing_tasks: 0,
          high_priority_count: 0,
          medium_priority_count: 0,
          low_priority_count: 0,
          queue_summary: {
            high_priority: [],
            medium_priority: [],
            low_priority: []
          }
        }
      }
    }
  }

  /**
   * 检查ROS2桥接服务状态
   */
  async checkROS2Status(): Promise<ROS2StatusResponse> {
    try {
      const response = await api.get('/api/tasks/ros2/status')
      return response.data
    } catch (error: any) {
      if (error.response?.data) {
        throw error.response.data
      }
      throw {
        success: false,
        data: {
          available: false,
          status: 'unavailable',
          bridge_url: 'http://localhost:8080'
        }
      }
    }
  }

  /**
   * 获取指定用户的任务列表
   */
  async getUserTasks(userId: string, status?: string): Promise<{ tasks: TaskData[]; total: number }> {
    try {
      const params = status ? `?status=${status}` : ''
      const response = await api.get(`/api/tasks/user/${userId}${params}`)
      return response.data
    } catch (error: any) {
      console.error('获取用户任务失败:', error)
      return { tasks: [], total: 0 }
    }
  }

  /**
   * 获取指定用户的待取件任务
   */
  async getUserPendingPickupTasks(userId: string): Promise<{ tasks: TaskData[]; total: number }> {
    try {
      const response = await api.get(`/api/tasks/user/${userId}/pending-pickup`)
      return response.data
    } catch (error: any) {
      console.error('获取待取件任务失败:', error)
      return { tasks: [], total: 0 }
    }
  }

  /**
   * Ping测试
   */
  async ping(): Promise<{ message: string }> {
    try {
      const response = await api.get('/api/tasks/ping')
      return response.data
    } catch (error: any) {
      throw {
        message: `API连接失败: ${error.message}`
      }
    }
  }
}

// 导出单例实例
export const taskApiService = TaskApiService.getInstance()
