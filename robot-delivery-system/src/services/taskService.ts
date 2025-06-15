import axios from 'axios'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 任务相关的类型定义
export interface TaskCreateRequest {
  user_id: string
  receiver: string
  location_id: string
  security_level: 'L1' | 'L2' | 'L3'
  description?: string
}

export interface TaskCreateResponse {
  success: boolean
  code: string
  message: string
  task_id?: string
  task?: any
  locker_id?: string
}

export interface TaskCompleteRequest {
  task_id: string
  user_id: string
}

export interface TaskCompleteResponse {
  success: boolean
  code: string
  message: string
  task?: any
}

export interface TaskFailRequest {
  task_id: string
  reason?: string
}

export interface TaskFailResponse {
  success: boolean
  code: string
  message: string
  task?: any
}

export interface Task {
  task_id: string
  initiator: string
  receiver: string
  location_id: string
  security_level: string
  status: string
  description?: string
  locker_id?: string
  created_at: string
  updated_at: string
}

// 任务服务类
export class TaskService {
  private static instance: TaskService

  static getInstance(): TaskService {
    if (!TaskService.instance) {
      TaskService.instance = new TaskService()
    }
    return TaskService.instance
  }

  /**
   * 创建新任务
   * @param request 任务创建请求
   * @returns 创建结果
   */
  async createTask(request: TaskCreateRequest): Promise<TaskCreateResponse> {
    try {
      const response = await axios.post(`${API_BASE_URL}/tasks/create`, request)
      return response.data
    } catch (error: any) {
      console.error('创建任务失败:', error)
      return {
        success: false,
        code: 'CREATE_FAILED',
        message: error.response?.data?.detail || '创建任务失败'
      }
    }
  }

  /**
   * 完成任务
   * @param request 任务完成请求
   * @returns 完成结果
   */
  async completeTask(request: TaskCompleteRequest): Promise<TaskCompleteResponse> {
    try {
      const response = await axios.post(`${API_BASE_URL}/tasks/${request.task_id}/complete`, {
        user_id: request.user_id
      })
      return response.data
    } catch (error: any) {
      console.error('完成任务失败:', error)
      return {
        success: false,
        code: 'COMPLETE_FAILED',
        message: error.response?.data?.detail || '完成任务失败'
      }
    }
  }

  /**
   * 任务失败
   * @param request 任务失败请求
   * @returns 失败结果
   */
  async failTask(request: TaskFailRequest): Promise<TaskFailResponse> {
    try {
      const response = await axios.post(`${API_BASE_URL}/tasks/${request.task_id}/fail`, {
        reason: request.reason
      })
      return response.data
    } catch (error: any) {
      console.error('任务失败处理失败:', error)
      return {
        success: false,
        code: 'FAIL_FAILED',
        message: error.response?.data?.detail || '任务失败处理失败'
      }
    }
  }

  /**
   * 获取用户的任务列表
   * @param userId 用户ID
   * @param status 任务状态过滤
   * @returns 任务列表
   */
  async getUserTasks(userId: string, status?: string): Promise<Task[]> {
    try {
      const params = status ? { status } : {}
      const response = await axios.get(`${API_BASE_URL}/tasks/user/${userId}`, { params })
      return response.data
    } catch (error) {
      console.error('获取用户任务失败:', error)
      return []
    }
  }

  /**
   * 获取待取件任务
   * @param userId 用户ID
   * @returns 待取件任务列表
   */
  async getPendingPickupTasks(userId: string): Promise<Task[]> {
    try {
      const response = await axios.get(`${API_BASE_URL}/tasks/user/${userId}/pending-pickup`)
      return response.data
    } catch (error) {
      console.error('获取待取件任务失败:', error)
      return []
    }
  }

  /**
   * 根据任务ID获取任务详情
   * @param taskId 任务ID
   * @returns 任务详情
   */
  async getTaskById(taskId: string): Promise<Task | null> {
    try {
      const response = await axios.get(`${API_BASE_URL}/tasks/${taskId}`)
      return response.data
    } catch (error) {
      console.error('获取任务详情失败:', error)
      return null
    }
  }

  /**
   * 获取所有任务列表（管理员功能）
   * @returns 所有任务列表
   */
  async getAllTasks(): Promise<Task[]> {
    try {
      const response = await axios.get(`${API_BASE_URL}/tasks/all`)
      return response.data
    } catch (error) {
      console.error('获取所有任务失败:', error)
      return []
    }
  }

  // 兼容性方法 - 保持原有接口
  static async createTask(request: TaskCreateRequest): Promise<TaskCreateResponse> {
    const service = TaskService.getInstance()
    return service.createTask(request)
  }

  static async completeTask(request: TaskCompleteRequest): Promise<TaskCompleteResponse> {
    const service = TaskService.getInstance()
    return service.completeTask(request)
  }

  static async failTask(request: TaskFailRequest): Promise<TaskFailResponse> {
    const service = TaskService.getInstance()
    return service.failTask(request)
  }
}

// 导出单例实例
export const taskService = TaskService.getInstance()
