export interface SemanticLocation {
  id: string
  name: string
  x: number
  y: number
  type: 'room' | 'corridor' | 'door' | 'elevator' | 'stairs' | 'landmark'
  width?: number
  height?: number
}

export interface SimulatedMapConfig {
  width: number
  height: number
  resolution: number // 米/像素
  origin: {
    x: number
    y: number
  }
  locations: SemanticLocation[]
  walls: Array<{
    start: { x: number, y: number }
    end: { x: number, y: number }
  }>
}

// 示例地图配置
export const defaultMapConfig: SimulatedMapConfig = {
  width: 800,
  height: 600,
  resolution: 0.05, // 5cm per pixel
  origin: { x: -20, y: -15 }, // 地图原点在世界坐标系中的位置
  locations: [
    {
      id: 'room_101',
      name: '办公室101',
      x: 5,
      y: 3,
      type: 'room',
      width: 4,
      height: 3
    },
    {
      id: 'room_102',
      name: '办公室102',
      x: 12,
      y: 3,
      type: 'room',
      width: 4,
      height: 3
    },
    {
      id: 'corridor_main',
      name: '主走廊',
      x: 8,
      y: 0,
      type: 'corridor',
      width: 2,
      height: 15
    },
    {
      id: 'elevator_01',
      name: '电梯1',
      x: 18,
      y: 8,
      type: 'elevator',
      width: 2,
      height: 2
    },
    {
      id: 'door_101',
      name: '101门',
      x: 7,
      y: 3,
      type: 'door'
    },
    {
      id: 'door_102',
      name: '102门',
      x: 14,
      y: 3,
      type: 'door'
    }
  ],
  walls: [
    // 外墙
    { start: { x: 0, y: 0 }, end: { x: 20, y: 0 } },
    { start: { x: 20, y: 0 }, end: { x: 20, y: 15 } },
    { start: { x: 20, y: 15 }, end: { x: 0, y: 15 } },
    { start: { x: 0, y: 15 }, end: { x: 0, y: 0 } },
    
    // 房间墙体
    { start: { x: 3, y: 1 }, end: { x: 3, y: 6 } },
    { start: { x: 3, y: 1 }, end: { x: 9, y: 1 } },
    { start: { x: 9, y: 1 }, end: { x: 9, y: 6 } },
    { start: { x: 3, y: 6 }, end: { x: 9, y: 6 } },
    
    { start: { x: 10, y: 1 }, end: { x: 10, y: 6 } },
    { start: { x: 10, y: 1 }, end: { x: 16, y: 1 } },
    { start: { x: 16, y: 1 }, end: { x: 16, y: 6 } },
    { start: { x: 10, y: 6 }, end: { x: 16, y: 6 } }
  ]
}

export class SimulatedRobotState {
  private position = { x: 8, y: 8, theta: 0 }
  private velocity = { linear: 0, angular: 0 }
  private targetPosition: { x: number, y: number } | null = null
  private moveSpeed = 1.0 // m/s
  private rotateSpeed = 1.0 // rad/s
  private callbacks: Array<(position: { x: number, y: number, theta: number }) => void> = []
  private simulationInterval: number | null = null

  constructor() {
    this.startSimulation()
  }

  private startSimulation() {
    this.simulationInterval = window.setInterval(() => {
      this.updatePosition()
      this.notifyCallbacks()
    }, 100) // 10Hz更新频率
  }

  private updatePosition() {
    if (!this.targetPosition) return

    const dx = this.targetPosition.x - this.position.x
    const dy = this.targetPosition.y - this.position.y
    const distance = Math.sqrt(dx * dx + dy * dy)

    if (distance < 0.1) {
      // 到达目标点
      this.targetPosition = null
      this.velocity = { linear: 0, angular: 0 }
      return
    }

    // 计算目标角度
    const targetTheta = Math.atan2(dy, dx)
    let deltaTheta = targetTheta - this.position.theta

    // 标准化角度到[-π, π]
    while (deltaTheta > Math.PI) deltaTheta -= 2 * Math.PI
    while (deltaTheta < -Math.PI) deltaTheta += 2 * Math.PI

    // 旋转到目标方向
    if (Math.abs(deltaTheta) > 0.1) {
      this.position.theta += Math.sign(deltaTheta) * this.rotateSpeed * 0.1
      this.velocity.angular = Math.sign(deltaTheta) * this.rotateSpeed
      this.velocity.linear = 0
    } else {
      // 向前移动
      const moveDistance = Math.min(this.moveSpeed * 0.1, distance)
      this.position.x += moveDistance * Math.cos(this.position.theta)
      this.position.y += moveDistance * Math.sin(this.position.theta)
      this.velocity.linear = this.moveSpeed
      this.velocity.angular = 0
    }
  }

  private notifyCallbacks() {
    this.callbacks.forEach(callback => {
      callback({ ...this.position })
    })
  }

  setTargetPosition(x: number, y: number) {
    this.targetPosition = { x, y }
  }

  getCurrentPosition() {
    return { ...this.position }
  }

  onPositionUpdate(callback: (position: { x: number, y: number, theta: number }) => void) {
    this.callbacks.push(callback)
  }

  setPosition(x: number, y: number, theta: number = 0) {
    this.position = { x, y, theta }
    this.notifyCallbacks()
  }

  stop() {
    if (this.simulationInterval) {
      clearInterval(this.simulationInterval)
      this.simulationInterval = null
    }
  }
}

export const simulatedRobot = new SimulatedRobotState()
