import axios from 'axios'
import { ref, reactive } from 'vue'
import { taskApiService } from './taskApiService'
import { pickupApiService } from './pickupApiService'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 柜门状态类型定义
export interface LockerStatus {
  locker_id: string
  status: 'available' | 'occupied'
  task_id?: string
  user_id?: string
  security_level?: string
  updated_at?: string
}

// 任务状态类型定义
export interface TaskStatus {
  task_id: string
  status: 'pending' | 'executing' | 'arrived' | 'completed' | 'failed'
  user_id: string
  receiver: string
  location_id: string
  security_level: string
  description?: string
  locker_id?: string
  created_at: string
  updated_at?: string
}

// 系统状态管理服务
export class SystemStatusService {
  private static instance: SystemStatusService
  
  // 响应式状态
  public lockers = ref<LockerStatus[]>([])
  public tasks = ref<TaskStatus[]>([])
  public queueStatus = ref<any>(null)
  public robotStatus = ref<string>('idle')
  
  // 状态更新回调
  private statusCallbacks: ((type: string, data: any) => void)[] = []

  static getInstance(): SystemStatusService {
    if (!SystemStatusService.instance) {
      SystemStatusService.instance = new SystemStatusService()
    }
    return SystemStatusService.instance
  }

  constructor() {
    // 启动定期状态更新
    this.startPeriodicUpdate()
  }

  /**
   * 获取所有柜门状态
   */
  async getLockerStatuses(): Promise<LockerStatus[]> {
    try {
      // 这里暂时返回模拟数据，实际应该调用后端API
      const mockLockers: LockerStatus[] = [
        { locker_id: 'L001', status: 'available' },
        { locker_id: 'L002', status: 'available' },
        { locker_id: 'L003', status: 'available' },
        { locker_id: 'L004', status: 'available' },
        { locker_id: 'L005', status: 'available' },
      ]
      
      this.lockers.value = mockLockers
      return mockLockers
    } catch (error) {
      console.error('获取柜门状态失败:', error)
      return []
    }
  }

  /**
   * 获取队列状态
   */
  async getQueueStatus(): Promise<any> {
    try {
      const response = await taskApiService.getQueueStatus()
      if (response.success) {
        this.queueStatus.value = response.data
        return response.data
      }
      return null
    } catch (error) {
      console.error('获取队列状态失败:', error)
      return null
    }
  }

  /**
   * 获取ROS2状态
   */
  async getROS2Status(): Promise<any> {
    try {
      const response = await taskApiService.checkROS2Status()
      if (response.success) {
        const status = response.data
        this.robotStatus.value = status.available ? 'connected' : 'disconnected'
        return status
      }
      return { available: false, status: 'error' }
    } catch (error) {
      console.error('获取ROS2状态失败:', error)
      this.robotStatus.value = 'error'
      return { available: false, status: 'error' }
    }
  }

  /**
   * 更新柜门状态
   */
  updateLockerStatus(lockerId: string, status: LockerStatus['status'], taskId?: string) {
    const locker = this.lockers.value.find(l => l.locker_id === lockerId)
    if (locker) {
      locker.status = status
      locker.task_id = taskId
      locker.updated_at = new Date().toISOString()
      this.notifyStatusUpdate('locker', locker)
    }
  }

  /**
   * 更新任务状态
   */
  updateTaskStatus(taskId: string, status: TaskStatus['status']) {
    const task = this.tasks.value.find(t => t.task_id === taskId)
    if (task) {
      task.status = status
      task.updated_at = new Date().toISOString()
      this.notifyStatusUpdate('task', task)
    }
  }

  /**
   * 订阅状态更新
   */
  onStatusUpdate(callback: (type: string, data: any) => void) {
    this.statusCallbacks.push(callback)
  }

  /**
   * 取消订阅状态更新
   */
  offStatusUpdate(callback: (type: string, data: any) => void) {
    const index = this.statusCallbacks.indexOf(callback)
    if (index !== -1) {
      this.statusCallbacks.splice(index, 1)
    }
  }

  /**
   * 启动定期状态更新
   */
  private startPeriodicUpdate() {
    // 每15秒更新一次状态（降低频率）
    setInterval(async () => {
      await this.getQueueStatus()
      await this.getROS2Status()
      await this.getLockerStatuses()
    }, 15000)
  }

  /**
   * 通知状态更新
   */
  private notifyStatusUpdate(type: string, data: any) {
    this.statusCallbacks.forEach(callback => {
      try {
        callback(type, data)
      } catch (error) {
        console.error('状态更新回调错误:', error)
      }
    })
  }

  /**
   * 手动刷新所有状态
   */
  async refreshAllStatus() {
    try {
      await Promise.all([
        this.getQueueStatus(),
        this.getROS2Status(),
        this.getLockerStatuses()
      ])
    } catch (error) {
      console.error('刷新状态失败:', error)
    }
  }
}

// 导出单例实例
export const systemStatusService = SystemStatusService.getInstance()
