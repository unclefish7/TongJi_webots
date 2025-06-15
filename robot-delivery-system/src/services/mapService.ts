// 地点位置接口
export interface LocationPose {
  position: {
    x: number
    y: number
    z: number
  }
  orientation: {
    x: number
    y: number
    z: number
    w: number
  }
}

// 地点接口（添加id字段）
export interface Location {
  id: string
  name: string
  pose: LocationPose
}

// 地图数据（从map.json加载）
const MAP_DATA: Record<string, LocationPose> = {
  "经理室": {
    "position": {
      "x": 0.7234129905700684,
      "y": -19.521406173706055,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": -0.01465866819676752,
      "w": 0.9998925559512367
    }
  },
  "财务处": {
    "position": {
      "x": -3.352997303009033,
      "y": -20.595783233642578,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": -0.6960923899624771,
      "w": 0.7179522161232784
    }
  },
  "等候处": {
    "position": {
      "x": -9.018957138061523,
      "y": -20.48737335205078,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": -0.8842058555876726,
      "w": 0.4670974255382617
    }
  },
  "前台": {
    "position": {
      "x": -10.979203224182129,
      "y": -16.93120002746582,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.7083596545028485,
      "w": 0.7058516840474386
    }
  },
  "休息室": {
    "position": {
      "x": -5.760888576507568,
      "y": -4.985156059265137,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0009870039753736553,
      "w": 0.9999995129114577
    }
  },
  "小办公区": {
    "position": {
      "x": -6.7908034324646,
      "y": 7.634155750274658,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": -0.999998231646026,
      "w": 0.001880612884459631
    }
  },
  "大办公区": {
    "position": {
      "x": -3.678215503692627,
      "y": 7.6719160079956055,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.00864396609606628,
      "w": 0.9999626402271887
    }
  },
  "大会议室": {
    "position": {
      "x": 1.7801513671875,
      "y": -8.649852752685547,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0023330187157100724,
      "w": 0.9999972785081328
    }
  },
  "小会议室": {
    "position": {
      "x": 1.1165165901184082,
      "y": -11.299249649047852,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.03669462856751698,
      "w": 0.9993265253330825
    }
  }
}

// 地图服务类
export class MapService {
  private static instance: MapService

  static getInstance(): MapService {
    if (!MapService.instance) {
      MapService.instance = new MapService()
    }
    return MapService.instance
  }

  // 获取所有可用地点
  getAllLocations(): Location[] {
    return Object.entries(MAP_DATA).map(([name, pose]) => ({
      id: name, // 使用name作为id
      name,
      pose
    }))
  }

  // 获取地点名称列表
  getLocationNames(): string[] {
    return Object.keys(MAP_DATA)
  }

  // 根据名称获取地点信息
  getLocationByName(name: string): Location | null {
    const pose = MAP_DATA[name]
    if (!pose) {
      return null
    }
    return { id: name, name, pose }
  }

  // 检查地点是否存在
  isValidLocation(name: string): boolean {
    return name in MAP_DATA
  }

  // 获取地点的位置坐标
  getLocationPosition(name: string): { x: number, y: number, z: number } | null {
    const location = this.getLocationByName(name)
    return location?.pose.position || null
  }

  // 获取地点的方向信息
  getLocationOrientation(name: string): { x: number, y: number, z: number, w: number } | null {
    const location = this.getLocationByName(name)
    return location?.pose.orientation || null
  }

  // 计算两个地点之间的距离
  calculateDistance(location1: string, location2: string): number | null {
    const pos1 = this.getLocationPosition(location1)
    const pos2 = this.getLocationPosition(location2)
    
    if (!pos1 || !pos2) {
      return null
    }
    
    const dx = pos1.x - pos2.x
    const dy = pos1.y - pos2.y
    const dz = pos1.z - pos2.z
    
    return Math.sqrt(dx * dx + dy * dy + dz * dz)
  }

  // 获取最近的地点
  getNearestLocation(targetX: number, targetY: number, targetZ: number = 0): Location | null {
    let nearestLocation: Location | null = null
    let minDistance = Infinity
    
    for (const [name, pose] of Object.entries(MAP_DATA)) {
      const dx = pose.position.x - targetX
      const dy = pose.position.y - targetY
      const dz = pose.position.z - targetZ
      const distance = Math.sqrt(dx * dx + dy * dy + dz * dz)
      
      if (distance < minDistance) {
        minDistance = distance
        nearestLocation = { id: name, name, pose }
      }
    }
    
    return nearestLocation
  }

  // 获取地点的显示信息（用于UI显示）
  getLocationDisplayInfo(name: string): { name: string, coordinates: string } | null {
    const location = this.getLocationByName(name)
    if (!location) {
      return null
    }
    
    const { x, y } = location.pose.position
    return {
      name,
      coordinates: `(${x.toFixed(1)}, ${y.toFixed(1)})`
    }
  }

  // 验证地点列表中的所有地点是否都存在
  validateLocationList(locations: string[]): { valid: boolean, invalidLocations: string[] } {
    const invalidLocations = locations.filter(location => !this.isValidLocation(location))
    return {
      valid: invalidLocations.length === 0,
      invalidLocations
    }
  }
}

// 导出单例实例
export const mapService = MapService.getInstance()
