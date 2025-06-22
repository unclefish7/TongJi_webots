// API服务统一导出文件
export { default as api } from './api'
export { authApiService } from './authApiService'
export { taskApiService } from './taskApiService'
export { pickupApiService } from './pickupApiService'
export { lockerApiService } from './lockerApiService'
export { userApiService } from './userApiService'
export { authService } from './authService'

// 类型定义统一导出
export type { TaskApiRequest, TaskApiResponse, TaskData } from './taskApiService'
export type { PickupTaskData, PickupTasksResponse, PickupExecuteRequest } from './pickupApiService'
export type { LockerStatus, LockerStatusResponse } from './lockerApiService'
export type { User, AuthRequest, AuthResponse } from './authService'
export type { AuthRequest as ApiAuthRequest, PurposeAuthRequest, PurposeAuthResponse } from './authApiService'
