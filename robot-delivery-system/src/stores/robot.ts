import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

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

export const useRobotStore = defineStore('robot', () => {  // 机器人状态
  const robots = ref<RobotStatus[]>([
    {
      id: 'robot1',
      name: '配送机器人-001',
      status: 'idle',
      position: { x: 15, y: 25 }, // 使用百分比位置
      battery: 85,
    },
    {
      id: 'robot2',
      name: '配送机器人-002',
      status: 'busy',
      position: { x: 55, y: 25 }, // 使用百分比位置
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

  // 当前用户认证等级
  const userAuthLevel = ref<'L1' | 'L2' | 'L3' | null>(null)

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
    // 模拟获取当前用户，实际应该从认证系统获取
    return '当前用户'
  }

  function callRobot(callInfo: {
    robotId: string
    location: string
    priority: 'normal' | 'urgent'
  }) {
    const robot = robots.value.find(r => r.id === callInfo.robotId)
    if (robot && robot.status === 'idle') {
      updateRobotStatus(callInfo.robotId, {
        status: 'busy',
        currentTask: `响应${callInfo.priority === 'urgent' ? '紧急' : '普通'}呼叫，前往${callInfo.location}`
      })
      return true
    }
    return false
  }

  function sendPackage(packageInfo: {
    destination: string
    securityLevel: 'L1' | 'L2' | 'L3'
    recipient?: string
  }) {
    const availableCompartment = availableCompartments.value.find(
      comp => comp.securityLevel === packageInfo.securityLevel
    )
    
    if (availableCompartment) {
      updateCompartmentStatus(availableCompartment.id, {
        isOccupied: true,
        content: '待寄送包裹',
        recipient: packageInfo.recipient || '未指定'
      })
      return availableCompartment
    }
    return null
  }

  return {
    robots,
    compartments,
    userAuthLevel,
    availableRobots,
    availableCompartments,
    userCompartments,
    updateRobotStatus,
    updateCompartmentStatus,
    setUserAuthLevel,
    callRobot,
    sendPackage,
    getCurrentUser
  }
})
