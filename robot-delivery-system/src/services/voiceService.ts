import axios from 'axios'

// 后端API基础URL
const API_BASE_URL = 'http://localhost:8000'

// 语音识别结果接口
export interface VoiceRecognitionResult {
  success: boolean
  text: string
  location: string | null
  matched_keywords: string[]
  available_locations: string[]
  filename?: string
  file_size?: number
  content_type?: string
}

// 录音配置
export interface RecordingConfig {
  sampleRate: number
  channels: number
  bitDepth: number
  format: string
}

export class VoiceRecognitionService {
  private static instance: VoiceRecognitionService
  private mediaRecorder: MediaRecorder | null = null
  private audioChunks: Blob[] = []
  private isRecording: boolean = false
  private stream: MediaStream | null = null

  static getInstance(): VoiceRecognitionService {
    if (!VoiceRecognitionService.instance) {
      VoiceRecognitionService.instance = new VoiceRecognitionService()
    }
    return VoiceRecognitionService.instance
  }

  /**
   * 检查浏览器是否支持语音录制
   */
  isSupported(): boolean {
    return !!(navigator.mediaDevices && 
             'getUserMedia' in navigator.mediaDevices && 
             typeof MediaRecorder !== 'undefined' && 
             MediaRecorder.isTypeSupported)
  }

  /**
   * 请求麦克风权限
   */
  async requestMicrophonePermission(): Promise<boolean> {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true })
      stream.getTracks().forEach(track => track.stop()) // 立即停止，只是测试权限
      return true
    } catch (error) {
      console.error('麦克风权限请求失败:', error)
      return false
    }
  }

  /**
   * 开始录音
   */
  async startRecording(): Promise<void> {
    if (this.isRecording) {
      throw new Error('已经在录音中')
    }

    if (!this.isSupported()) {
      throw new Error('浏览器不支持录音功能')
    }

    try {
      // 获取麦克风权限
      this.stream = await navigator.mediaDevices.getUserMedia({
        audio: {
          sampleRate: 16000, // 16kHz采样率
          channelCount: 1,   // 单声道
          echoCancellation: true,
          noiseSuppression: true,
          autoGainControl: true
        }
      })

      // 创建MediaRecorder
      const options: MediaRecorderOptions = {
        mimeType: 'audio/webm;codecs=opus' // 优先使用webm格式
      }

      // 检查支持的格式
      if (!MediaRecorder.isTypeSupported(options.mimeType!)) {
        // fallback到wav格式
        if (MediaRecorder.isTypeSupported('audio/wav')) {
          options.mimeType = 'audio/wav'
        } else if (MediaRecorder.isTypeSupported('audio/mp4')) {
          options.mimeType = 'audio/mp4'
        } else {
          delete options.mimeType // 使用默认格式
        }
      }

      this.mediaRecorder = new MediaRecorder(this.stream, options)
      this.audioChunks = []

      // 监听录音数据
      this.mediaRecorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          this.audioChunks.push(event.data)
        }
      }

      // 开始录音
      this.mediaRecorder.start(100) // 每100ms收集一次数据
      this.isRecording = true

      console.log('开始录音...')
    } catch (error) {
      console.error('开始录音失败:', error)
      throw error
    }
  }

  /**
   * 停止录音并返回音频文件
   */
  async stopRecording(): Promise<File> {
    return new Promise((resolve, reject) => {
      if (!this.isRecording || !this.mediaRecorder) {
        reject(new Error('当前没有在录音'))
        return
      }

      this.mediaRecorder.onstop = () => {
        try {
          // 合并音频数据
          const audioBlob = new Blob(this.audioChunks, { 
            type: this.mediaRecorder?.mimeType || 'audio/wav' 
          })

          // 创建File对象
          const audioFile = new File(
            [audioBlob], 
            `voice-command-${Date.now()}.${this.getFileExtension(this.mediaRecorder?.mimeType)}`,
            { type: audioBlob.type }
          )

          // 停止媒体流
          if (this.stream) {
            this.stream.getTracks().forEach(track => track.stop())
            this.stream = null
          }

          this.isRecording = false
          console.log('录音完成, 文件大小:', audioFile.size, 'bytes')

          resolve(audioFile)
        } catch (error) {
          reject(error)
        }
      }

      this.mediaRecorder.onerror = (event) => {
        reject(new Error('录音过程中出错'))
      }

      // 停止录音
      this.mediaRecorder.stop()
    })
  }

  /**
   * 取消录音
   */
  cancelRecording(): void {
    if (this.isRecording && this.mediaRecorder) {
      this.mediaRecorder.stop()
      
      if (this.stream) {
        this.stream.getTracks().forEach(track => track.stop())
        this.stream = null
      }
      
      this.isRecording = false
      this.audioChunks = []
      console.log('录音已取消')
    }
  }

  /**
   * 获取当前是否正在录音
   */
  getIsRecording(): boolean {
    return this.isRecording
  }

  /**
   * 上传音频文件进行语音识别
   */
  async recognizeAudio(audioFile: File): Promise<VoiceRecognitionResult> {
    try {
      const formData = new FormData()
      formData.append('audio', audioFile)

      const response = await axios.post(`${API_BASE_URL}/api/recognize`, formData, {
        headers: {
          'Content-Type': 'multipart/form-data'
        },
        timeout: 30000 // 30秒超时
      })

      return response.data
    } catch (error: any) {
      console.error('语音识别失败:', error)
      throw new Error(error.response?.data?.detail || '语音识别服务异常')
    }
  }

  /**
   * 获取可用地点列表
   */
  async getAvailableLocations(): Promise<{ locations: Record<string, string[]>, location_names: string[] }> {
    try {
      const response = await axios.get(`${API_BASE_URL}/api/locations`)
      return response.data
    } catch (error) {
      console.error('获取地点列表失败:', error)
      throw error
    }
  }

  /**
   * 测试文本地点提取
   */
  async testTextExtraction(text: string): Promise<any> {
    try {
      const response = await axios.post(`${API_BASE_URL}/api/test-recognition?text=${encodeURIComponent(text)}`)
      return response.data
    } catch (error) {
      console.error('测试文本提取失败:', error)
      throw error
    }
  }

  /**
   * 根据MIME类型获取文件扩展名
   */
  private getFileExtension(mimeType?: string): string {
    if (!mimeType) return 'wav'
    
    if (mimeType.includes('webm')) return 'webm'
    if (mimeType.includes('mp4')) return 'm4a'
    if (mimeType.includes('wav')) return 'wav'
    if (mimeType.includes('mp3')) return 'mp3'
    
    return 'wav'
  }
}

// 导出单例实例
export const voiceService = VoiceRecognitionService.getInstance()
