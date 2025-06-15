import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import { TaskService } from '@/services/taskService'

export interface RobotStatus {
  id: string
  name: string
  status: 'idle' | 'busy' | 'error' | 'charging'
  position: { x: number; y: number }
  battery: number
  currentTask?: string
}

export interface CompartmentStatus {
  id: string
  floor: number
  compartmentNumber: number
  isOccupied: boolean
  securityLevel: 'L1' | 'L2' | 'L3'
  content?: string
  recipient?: string
}

export const useRobotStore = defineStore('robot', () => {
  // 机器人状态
  const robots = ref<RobotStatus[]>([
    {
      id: 'robot1',
      name: '配送机器人-001',
      status: 'idle',
      position: { x: 15, y: 25 },
      battery: 85,
    },
    {
      id: 'robot2',
      name: '配送机器人-002',
      status: 'busy',
      position: { x: 55, y: 25 },
      battery: 72,
      currentTask: '正在配送至3楼301房间',
    },
  ])

  // 柜门状态
  const compartments = ref<CompartmentStatus[]>([
    { id: 'c1', floor: 1, compartmentNumber: 1, isOccupied: false, securityLevel: 'L1' },
    { id: 'c2', floor: 1, compartmentNumber: 2, isOccupied: true, securityLevel: 'L2', content: '文件包裹', recipient: '张三' },
    { id: 'c3', floor: 1, compartmentNumber: 3, isOccupied: false, securityLevel: 'L3' },
    { id: 'c4', floor: 2, compartmentNumber: 1, isOccupied: true, securityLevel: 'L1', content: '普通包裹', recipient: '李四' },
    { id: 'c5', floor: 2, compartmentNumber: 2, isOccupied: false, securityLevel: 'L2' },
    { id: 'c6', floor: 3, compartmentNumber: 1, isOccupied: false, securityLevel: 'L3' },
  ])

  // 当前用户认证等级和信息
  const userAuthLevel = ref<'L1' | 'L2' | 'L3' | null>(null)
  const currentUser = ref<string>('EMP9875') // 默认用户ID

  // 计算属性
  const availableRobots = computed(() => 
    robots.value.filter(robot => robot.status === 'idle')
  )

  const availableCompartments = computed(() => 
    compartments.value.filter(comp => !comp.isOccupied)
  )

  const userCompartments = computed(() => {
    if (!userAuthLevel.value) return []
    return compartments.value.filter(comp => 
      comp.isOccupied && comp.recipient === getCurrentUser()
    )
  })

  // 方法
  function updateRobotStatus(robotId: string, status: Partial<RobotStatus>) {
    const robot = robots.value.find(r => r.id === robotId)
    if (robot) {
      Object.assign(robot, status)
    }
  }

  function updateCompartmentStatus(compartmentId: string, status: Partial<CompartmentStatus>) {
    const compartment = compartments.value.find(c => c.id === compartmentId)
    if (compartment) {
      Object.assign(compartment, status)
    }
  }

  function setUserAuthLevel(level: 'L1' | 'L2' | 'L3' | null) {
    userAuthLevel.value = level
  }

  function getCurrentUser() {
    return currentUser.value
  }

  function setCurrentUser(userId: string) {
    currentUser.value = userId
  }

  // 使用后端API的呼叫机器人功能
  async function callRobot(callInfo: {
    robotId: string
    location: string
    priority: 'normal' | 'urgent'
    notes?: string
  }): Promise<boolean> {
    try {
      // 检查权限
      if (!userAuthLevel.value) {
        throw new Error('需要身份认证')
      }

      const requiredLevel = callInfo.priority === 'urgent' ? 'L2' : 'L1'
      // TODO: Implement proper authentication check
      // if (!authService.hasAuthLevel(requiredLevel)) {
      //   throw new Error(`需要${requiredLevel}级别权限`)
      // }

      // TODO: Integrate with new TaskService API
      // const response = await TaskService.callRobot({
      //   user_id: currentUser.value,
      //   location: callInfo.location,
      //   priority: callInfo.priority,
      //   notes: callInfo.notes
      // })

      // For now, simulate success
      const response = { success: true }

      if (response.success) {
        // 更新本地机器人状态
        updateRobotStatus(callInfo.robotId, {
          status: 'busy',
          currentTask: `响应${callInfo.priority === 'urgent' ? '紧急' : '普通'}呼叫，前往${callInfo.location}`
        })

        return true
      } else {
        // throw new Error(response.message || '呼叫失败')
        throw new Error('呼叫失败')
      }
    } catch (error) {
      console.error('Call robot failed:', error)
      return false
    }
  }

  // 使用后端API的发送包裹功能
  async function sendPackage(packageInfo: {
    destination: string
    securityLevel: 'L1' | 'L2' | 'L3'
    recipient?: string
    description?: string
  }): Promise<CompartmentStatus | null> {
    try {
      // 检查权限
      if (!userAuthLevel.value) {
        throw new Error('需要身份认证')
      }

      // TODO: Implement proper authentication check
      // if (!authService.hasAuthLevel(packageInfo.securityLevel)) {
      //   throw new Error(`需要${packageInfo.securityLevel}级别权限`)
      // }

      // TODO: Integrate with new TaskService API
      // const response = await TaskService.sendPackage({
      //   user_id: currentUser.value,
      //   destination: packageInfo.destination,
      //   securityLevel: packageInfo.securityLevel,
      //   recipient: packageInfo.recipient,
      //   description: packageInfo.description
      // })

      // For now, simulate success
      const response = { success: true, lockerId: 'L001' }

      if (response.success && response.lockerId) {
        // 找到对应的柜门并更新状态
        const compartment = availableCompartments.value.find(
          comp => comp.securityLevel === packageInfo.securityLevel
        )
        
        if (compartment) {
          updateCompartmentStatus(compartment.id, {
            isOccupied: true,
            content: '待寄送包裹',
            recipient: packageInfo.recipient || '未指定'
          })
          return compartment
        }
      } else {
        throw new Error('寄送失败')
      }
    } catch (error) {
      console.error('Send package failed:', error)
      return null
    }

    return null
  }

  // 测试API连接
  async function testApiConnection(): Promise<boolean> {
    try {
      // TODO: Implement ping functionality in TaskService
      // await TaskService.ping()
      return true
    } catch (error) {
      console.error('API connection test failed:', error)
      return false
    }
  }

  return {
    robots,
    compartments,
    userAuthLevel,
    currentUser,
    availableRobots,
    availableCompartments,
    userCompartments,
    updateRobotStatus,
    updateCompartmentStatus,
    setUserAuthLevel,
    callRobot,
    sendPackage,
    getCurrentUser,
    setCurrentUser,
    testApiConnection
  }
})
