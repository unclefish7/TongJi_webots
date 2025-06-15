import axios from 'axios'
import { rosConnection } from './rosConnection'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 机器人任务接口
export interface RobotTask {
  id: string
  type: 'delivery' | 'pickup' | 'call'
  location: string
  user_id: string
  description?: string
  priority: number
  created_at: string
  updated_at?: string
  status: 'pending' | 'in_progress' | 'executing' | 'completed' | 'failed'
  estimated_arrival?: string
}

// 队列状态接口
export interface QueueStatus {
  total_tasks: number
  pending_tasks: number
  current_task: RobotTask | null
  robot_status: 'idle' | 'moving' | 'waiting' | 'error'
  current_location?: string
}

// 机器人服务类
export class RobotService {
  private static instance: RobotService
  private taskQueue: RobotTask[] = []
  private currentTask: RobotTask | null = null
  private robotStatus: 'idle' | 'moving' | 'waiting' | 'error' = 'idle'
  private statusUpdateCallbacks: ((status: QueueStatus) => void)[] = []

  static getInstance(): RobotService {
    if (!RobotService.instance) {
      RobotService.instance = new RobotService()
    }
    return RobotService.instance
  }

  constructor() {
    // 初始化时启动队列处理
    this.startQueueProcessor()
  }

  /**
   * 添加寄件任务到队列
   * @param taskId 任务ID
   * @param userId 用户ID
   * @param location 目标位置
   * @param description 任务描述
   */
  async addDeliveryTask(taskId: string, userId: string, location: string, description?: string): Promise<void> {
    const task: RobotTask = {
      id: `delivery_${taskId}`,
      type: 'delivery',
      location,
      user_id: userId,
      description: description || `配送任务到${location}`,
      priority: 2,
      created_at: new Date().toISOString(),
      status: 'pending'
    }

    try {
      // 调用后端API添加任务
      await axios.post(`${API_BASE_URL}/api/queue/add`, {
        task_id: task.id,
        type: task.type,
        location: task.location,
        user_id: task.user_id,
        description: task.description,
        priority: task.priority
      })
      
      console.log(`配送任务已添加到后端队列: ${task.id} -> ${location}`)
    } catch (error) {
      console.error('添加配送任务到后端失败:', error)
      // 即使后端失败，也添加到本地队列作为备选
    }

    // 添加到本地队列
    this.taskQueue.push(task)
    this.sortQueue()
    this.notifyStatusUpdate()
    
    console.log(`添加配送任务: ${task.id} -> ${location}`)
  }

  /**
   * 添加取件任务到队列
   * @param taskId 任务ID
   * @param userId 用户ID
   * @param location 目标位置
   */
  async addPickupTask(taskId: string, userId: string, location: string): Promise<void> {
    const task: RobotTask = {
      id: `pickup_${taskId}`,
      type: 'pickup',
      location,
      user_id: userId,
      description: `从${location}取件`,
      priority: 1,
      created_at: new Date().toISOString(),
      status: 'pending'
    }

    try {
      // 调用后端API添加任务
      await axios.post(`${API_BASE_URL}/api/queue/add`, {
        task_id: task.id,
        type: task.type,
        location: task.location,
        user_id: task.user_id,
        description: task.description,
        priority: task.priority
      })
      
      console.log(`取件任务已添加到后端队列: ${task.id} -> ${location}`)
    } catch (error) {
      console.error('添加取件任务到后端失败:', error)
      // 即使后端失败，也添加到本地队列作为备选
    }

    // 添加到本地队列
    this.taskQueue.push(task)
    this.sortQueue()
    this.notifyStatusUpdate()
    
    console.log(`添加取件任务: ${task.id} -> ${location}`)
  }

  /**
   * 添加呼叫任务到队列
   * @param userId 用户ID
   * @param location 用户位置
   */
  async addCallTask(userId: string, location: string): Promise<void> {
    const task: RobotTask = {
      id: `call_${userId}_${Date.now()}`,
      type: 'call',
      location,
      user_id: userId,
      description: `用户呼叫服务 - ${location}`,
      priority: 3,
      created_at: new Date().toISOString(),
      status: 'pending'
    }

    try {
      // 调用后端API添加任务
      await axios.post(`${API_BASE_URL}/api/queue/add`, {
        task_id: task.id,
        type: task.type,
        location: task.location,
        user_id: task.user_id,
        description: task.description,
        priority: task.priority
      })
      
      console.log(`呼叫任务已添加到后端队列: ${task.id} -> ${location}`)
    } catch (error) {
      console.error('添加呼叫任务到后端失败:', error)
      // 即使后端失败，也添加到本地队列作为备选
    }

    // 添加到本地队列
    this.taskQueue.push(task)
    this.sortQueue()
    this.notifyStatusUpdate()
    
    console.log(`添加呼叫任务: ${task.id} -> ${location}`)
  }

  /**
   * 获取当前队列状态
   */
  getQueueStatus(): QueueStatus {
    return {
      total_tasks: this.taskQueue.length + (this.currentTask ? 1 : 0),
      pending_tasks: this.taskQueue.length,
      current_task: this.currentTask,
      robot_status: this.robotStatus
    }
  }

  /**
   * 获取队列中的所有任务
   */
  getAllTasks(): RobotTask[] {
    const tasks = [...this.taskQueue]
    if (this.currentTask) {
      tasks.unshift(this.currentTask)
    }
    return tasks
  }

  /**
   * 移除指定任务
   * @param taskId 任务ID
   */
  removeTask(taskId: string): boolean {
    const index = this.taskQueue.findIndex(task => task.id === taskId)
    if (index !== -1) {
      this.taskQueue.splice(index, 1)
      this.notifyStatusUpdate()
      return true
    }
    return false
  }

  /**
   * 清空队列
   */
  clearQueue(): void {
    this.taskQueue = []
    this.notifyStatusUpdate()
  }

  /**
   * 订阅状态更新
   * @param callback 回调函数
   */
  onStatusUpdate(callback: (status: QueueStatus) => void): void {
    this.statusUpdateCallbacks.push(callback)
  }

  /**
   * 取消订阅状态更新
   * @param callback 回调函数
   */
  offStatusUpdate(callback: (status: QueueStatus) => void): void {
    const index = this.statusUpdateCallbacks.indexOf(callback)
    if (index !== -1) {
      this.statusUpdateCallbacks.splice(index, 1)
    }
  }

  /**
   * 手动执行下一个任务
   */
  async executeNextTask(): Promise<void> {
    if (this.currentTask || this.taskQueue.length === 0) {
      return
    }

    this.currentTask = this.taskQueue.shift()!
    this.currentTask.status = 'in_progress'
    this.robotStatus = 'moving'
    this.notifyStatusUpdate()

    console.log(`开始执行任务: ${this.currentTask.id} -> ${this.currentTask.location}`)

    try {
      // 发送导航指令到ROS
      await this.sendNavigationCommand(this.currentTask.location)
      
      // 模拟导航过程
      await this.waitForNavigation()
      
      // 到达目标后等待操作
      this.robotStatus = 'waiting'
      this.notifyStatusUpdate()
      
      console.log(`机器人已到达 ${this.currentTask.location}，等待用户操作...`)
      
      // 等待1分钟后自动完成任务
      setTimeout(() => {
        this.completeCurrentTask()
      }, 60000) // 1分钟
      
    } catch (error) {
      console.error('任务执行失败:', error)
      this.currentTask.status = 'failed'
      this.robotStatus = 'error'
      this.notifyStatusUpdate()
      
      // 错误后等待一段时间再继续
      setTimeout(() => {
        this.completeCurrentTask()
      }, 5000)
    }
  }

  /**
   * 完成当前任务
   */
  completeCurrentTask(): void {
    if (this.currentTask) {
      console.log(`任务完成: ${this.currentTask.id}`)
      this.currentTask.status = 'completed'
      this.currentTask = null
      this.robotStatus = 'idle'
      this.notifyStatusUpdate()
      
      // 继续执行下一个任务
      setTimeout(() => {
        this.executeNextTask()
      }, 2000) // 2秒后执行下一个任务
    }
  }

  /**
   * 发送导航指令到ROS
   * @param location 目标位置
   */
  private async sendNavigationCommand(location: string): Promise<void> {
    try {
      // 发送多目标导航指令
      await rosConnection.publishMultiNavCommand([location])
      
      // 发送启动信号
      setTimeout(() => {
        rosConnection.publishNextSignal('start')
      }, 1000)
      
    } catch (error) {
      console.error('发送导航指令失败:', error)
      throw error
    }
  }

  /**
   * 等待导航完成
   */
  private async waitForNavigation(): Promise<void> {
    return new Promise((resolve) => {
      // 模拟导航时间 (实际应该监听ROS的导航状态)
      const estimatedTime = 10000 + Math.random() * 15000 // 10-25秒
      setTimeout(resolve, estimatedTime)
    })
  }

  /**
   * 队列排序（按优先级）
   */
  private sortQueue(): void {
    this.taskQueue.sort((a, b) => a.priority - b.priority)
  }

  /**
   * 通知状态更新
   */
  private notifyStatusUpdate(): void {
    const status = this.getQueueStatus()
    this.statusUpdateCallbacks.forEach(callback => {
      try {
        callback(status)
      } catch (error) {
        console.error('状态更新回调错误:', error)
      }
    })
  }

  /**
   * 启动队列处理器
   */
  private startQueueProcessor(): void {
    // 每5秒检查一次队列
    setInterval(() => {
      if (this.robotStatus === 'idle' && this.taskQueue.length > 0) {
        this.executeNextTask()
      }
    }, 5000)
  }

  /**
   * 获取任务队列
   */
  /**
   * 获取任务队列（从后端API获取真实数据）
   */
  async getTaskQueue(): Promise<{ tasks: RobotTask[] }> {
    try {
      const response = await axios.get(`${API_BASE_URL}/api/queue`)
      return {
        tasks: response.data.tasks || []
      }
    } catch (error) {
      console.error('获取后端任务队列失败，返回本地队列:', error)
      // 后端失败时返回本地队列作为备选
      return {
        tasks: [...this.taskQueue]
      }
    }
  }

  /**
   * 开始执行任务（调用后端API）
   */
  async startTask(taskId: string): Promise<void> {
    try {
      await axios.post(`${API_BASE_URL}/api/queue/${taskId}/start`)
      
      // 同时更新本地队列
      const task = this.taskQueue.find(t => t.id === taskId)
      if (task) {
        task.status = 'executing'
        task.updated_at = new Date().toISOString()
        this.notifyStatusUpdate()
      }
      
      console.log(`开始执行任务: ${taskId}`)
    } catch (error) {
      console.error('开始任务失败:', error)
      throw error
    }
  }

  /**
   * 标记任务失败（调用后端API）
   */
  async failTask(taskId: string, reason?: string): Promise<void> {
    try {
      await axios.post(`${API_BASE_URL}/api/queue/${taskId}/fail`, { reason })
      
      // 同时更新本地队列
      const taskIndex = this.taskQueue.findIndex(t => t.id === taskId)
      if (taskIndex !== -1) {
        this.taskQueue[taskIndex].status = 'failed'
        this.taskQueue[taskIndex].updated_at = new Date().toISOString()
        this.notifyStatusUpdate()
      }
      
      console.log(`任务失败: ${taskId}, 原因: ${reason}`)
    } catch (error) {
      console.error('标记任务失败时出错:', error)
      throw error
    }
  }
}

// 导出单例实例
export const robotService = RobotService.getInstance()
