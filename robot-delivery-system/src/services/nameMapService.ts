import { userApiService } from './userApiService'
import { locationApiService } from './locationApiService'

/**
 * 名称映射服务 - 将ID转换为人类可读的名称
 */
export class NameMapService {
  private static instance: NameMapService
  private userMap: Map<string, string> = new Map()
  private locationMap: Map<string, string> = new Map()
  private initialized = false

  static getInstance(): NameMapService {
    if (!NameMapService.instance) {
      NameMapService.instance = new NameMapService()
    }
    return NameMapService.instance
  }

  /**
   * 初始化名称映射缓存
   */
  async initialize(): Promise<void> {
    if (this.initialized) return

    try {
      // 加载用户映射
      const users = await userApiService.getAllUsers()
      for (const user of users) {
        this.userMap.set(user.user_id, user.name || user.user_id)
      }

      // 加载地点映射
      const locations = await locationApiService.getAllLocations()
      for (const location of locations) {
        this.locationMap.set(location.location_id, location.label || location.location_id)
      }

      this.initialized = true
      console.log('名称映射服务初始化完成')
      console.log('用户映射:', Array.from(this.userMap.entries()))
      console.log('地点映射:', Array.from(this.locationMap.entries()))
    } catch (error) {
      console.error('初始化名称映射失败:', error)
    }
  }

  /**
   * 获取用户显示名称
   */
  getUserName(userId: string): string {
    return this.userMap.get(userId) || userId
  }

  /**
   * 获取地点显示名称
   */
  getLocationName(locationId: string): string {
    return this.locationMap.get(locationId) || locationId
  }

  /**
   * 批量获取用户名称
   */
  getBatchUserNames(userIds: string[]): Map<string, string> {
    const result = new Map<string, string>()
    for (const userId of userIds) {
      result.set(userId, this.getUserName(userId))
    }
    return result
  }

  /**
   * 批量获取地点名称
   */
  getBatchLocationNames(locationIds: string[]): Map<string, string> {
    const result = new Map<string, string>()
    for (const locationId of locationIds) {
      result.set(locationId, this.getLocationName(locationId))
    }
    return result
  }

  /**
   * 强制刷新缓存
   */
  async refresh(): Promise<void> {
    this.initialized = false
    this.userMap.clear()
    this.locationMap.clear()
    await this.initialize()
  }
}

export const nameMapService = NameMapService.getInstance()
