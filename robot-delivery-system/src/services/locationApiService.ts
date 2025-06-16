// robot-delivery-system/src/services/locationApiService.ts
import axios, { type AxiosResponse } from 'axios'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

export interface LocationData {
  location_id: string
  label: string
  coordinates: {
    x: number
    y: number
  }
}

export interface LocationApiResponse {
  success: boolean
  data?: LocationData[]
  message?: string
}

class LocationApiService {
  // 获取所有位置
  async getAllLocations(): Promise<LocationData[]> {
    try {
      console.log('正在获取位置数据...')
      const response: AxiosResponse<LocationData[]> = await axios.get(`${API_BASE_URL}/api/locations/`)
      console.log('位置数据API响应:', response.data)
      return response.data || []
    } catch (error: any) {
      console.error('获取位置数据失败:', error)
      if (error.response) {
        console.error('错误响应:', error.response.data)
        console.error('状态码:', error.response.status)
      }
      throw new Error(`获取位置数据失败: ${error.message}`)
    }
  }

  // 根据ID获取位置
  async getLocationById(locationId: string): Promise<LocationData | null> {
    try {
      const response: AxiosResponse<LocationData> = await axios.get(`${API_BASE_URL}/api/locations/${locationId}`)
      return response.data || null
    } catch (error: any) {
      console.error(`获取位置 ${locationId} 失败:`, error)
      return null
    }
  }
}

export const locationApiService = new LocationApiService()
export default locationApiService
