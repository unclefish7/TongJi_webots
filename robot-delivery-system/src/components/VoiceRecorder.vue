<template>
  <div class="voice-recorder">
    <!-- 录音按钮 -->
    <el-button
      :type="isRecording ? 'danger' : 'primary'"
      :loading="isProcessing"
      @click="toggleRecording"
      :disabled="!isSupported"
      size="large"
      class="record-button"
    >
      <el-icon>
        <Microphone v-if="!isRecording" />
        <VideoPause v-else />
      </el-icon>
      {{ getButtonText() }}
    </el-button>

    <!-- 录音状态显示 -->
    <div v-if="isRecording" class="recording-status">
      <div class="recording-indicator">
        <div class="pulse"></div>
        <span>正在录音... {{ recordingTime }}s</span>
      </div>
      <el-button @click="cancelRecording" size="small" type="info" plain>
        取消录音
      </el-button>
    </div>

    <!-- 识别结果显示 -->
    <div v-if="recognitionResult" class="recognition-result">
      <el-card class="result-card">
        <template #header>
          <div class="card-header">
            <el-icon><ChatRound /></el-icon>
            <span>语音识别结果</span>
          </div>
        </template>
        
        <div class="result-content">
          <div class="recognized-text">
            <strong>识别文本:</strong> "{{ recognitionResult.text }}"
          </div>
          
          <div v-if="recognitionResult.location" class="detected-location">
            <strong>识别地点:</strong> 
            <el-tag type="success" size="large">{{ recognitionResult.location }}</el-tag>
          </div>
          
          <div v-else class="no-location">
            <el-tag type="warning">未识别到有效地点</el-tag>
          </div>
          
          <div v-if="recognitionResult.matched_keywords.length > 0" class="matched-keywords">
            <strong>匹配关键词:</strong>
            <el-tag 
              v-for="keyword in recognitionResult.matched_keywords" 
              :key="keyword"
              size="small"
              class="keyword-tag"
            >
              {{ keyword }}
            </el-tag>
          </div>
        </div>

        <div class="result-actions">
          <el-button 
            v-if="recognitionResult.location" 
            type="primary" 
            @click="selectLocation"
          >
            选择此地点
          </el-button>
          <el-button @click="clearResult" type="info" plain>
            重新录音
          </el-button>
        </div>
      </el-card>
    </div>

    <!-- 帮助信息 -->
    <div class="help-info">
      <el-alert
        title="语音指令示例"
        type="info"
        :closable="false"
        class="help-alert"
      >
        <template #default>
          <div class="help-examples">
            <p>您可以说：</p>
            <ul>
              <li>"帮我送到一楼快递柜"</li>
              <li>"送到二楼办公室"</li>
              <li>"我要寄到三楼前台"</li>
              <li>"送到经理室"</li>
            </ul>
          </div>
        </template>
      </el-alert>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { ElMessage } from 'element-plus'
import { Microphone, VideoPause, ChatRound } from '@element-plus/icons-vue'
import { voiceService, type VoiceRecognitionResult } from '@/services/voiceService'

// Props
interface Props {
  onLocationSelected?: (location: string) => void
}

const props = withDefaults(defineProps<Props>(), {
  onLocationSelected: () => {}
})

// 发射事件
const emit = defineEmits<{
  locationSelected: [location: string]
}>()

// 状态管理
const isSupported = ref(false)
const isRecording = ref(false)
const isProcessing = ref(false)
const recordingTime = ref(0)
const recognitionResult = ref<VoiceRecognitionResult | null>(null)

// 定时器
let recordingTimer: number | null = null

// 生命周期
onMounted(async () => {
  // 检查浏览器支持
  isSupported.value = voiceService.isSupported()
  
  if (!isSupported.value) {
    ElMessage.error('您的浏览器不支持语音录制功能')
    return
  }

  // 请求麦克风权限（静默检查）
  try {
    await voiceService.requestMicrophonePermission()
  } catch (error) {
    console.warn('麦克风权限检查:', error)
  }
})

onUnmounted(() => {
  if (recordingTimer) {
    clearInterval(recordingTimer)
  }
  if (isRecording.value) {
    voiceService.cancelRecording()
  }
})

// 方法
const getButtonText = (): string => {
  if (!isSupported.value) return '不支持录音'
  if (isProcessing.value) return '处理中...'
  if (isRecording.value) return '停止录音'
  return '开始录音'
}

const toggleRecording = async (): Promise<void> => {
  if (isRecording.value) {
    await stopRecording()
  } else {
    await startRecording()
  }
}

const startRecording = async (): Promise<void> => {
  try {
    // 清除之前的结果
    recognitionResult.value = null
    
    // 开始录音
    await voiceService.startRecording()
    isRecording.value = true
    recordingTime.value = 0
    
    // 开始计时
    recordingTimer = setInterval(() => {
      recordingTime.value++
      
      // 最大录音时长30秒
      if (recordingTime.value >= 30) {
        stopRecording()
        ElMessage.warning('录音时长超过30秒，自动停止')
      }
    }, 1000)
    
    ElMessage.success('开始录音，请说出您的指令')
  } catch (error: any) {
    ElMessage.error(error.message || '开始录音失败')
    console.error('开始录音失败:', error)
  }
}

const stopRecording = async (): Promise<void> => {
  try {
    isProcessing.value = true
    
    // 停止计时
    if (recordingTimer) {
      clearInterval(recordingTimer)
      recordingTimer = null
    }
    
    // 停止录音并获取音频文件
    const audioFile = await voiceService.stopRecording()
    isRecording.value = false
    
    ElMessage.info('录音完成，正在识别...')
    
    // 发送给后端进行识别
    const result = await voiceService.recognizeAudio(audioFile)
    recognitionResult.value = result
    
    if (result.success && result.text) {
      if (result.location) {
        ElMessage.success(`识别成功！检测到地点: ${result.location}`)
      } else {
        ElMessage.warning(`识别到文本"${result.text}"，但未检测到有效地点`)
      }
    } else {
      ElMessage.error('语音识别失败，请重试')
    }
    
  } catch (error: any) {
    ElMessage.error(error.message || '语音识别失败')
    console.error('语音识别失败:', error)
  } finally {
    isProcessing.value = false
    isRecording.value = false
  }
}

const cancelRecording = (): void => {
  voiceService.cancelRecording()
  isRecording.value = false
  
  if (recordingTimer) {
    clearInterval(recordingTimer)
    recordingTimer = null
  }
  
  recordingTime.value = 0
  ElMessage.info('录音已取消')
}

const selectLocation = (): void => {
  if (recognitionResult.value?.location) {
    emit('locationSelected', recognitionResult.value.location)
    props.onLocationSelected?.(recognitionResult.value.location)
    ElMessage.success(`已选择地点: ${recognitionResult.value.location}`)
  }
}

const clearResult = (): void => {
  recognitionResult.value = null
}
</script>

<style scoped>
.voice-recorder {
  width: 100%;
  display: flex;
  flex-direction: column;
  gap: 20px;
  align-items: center;
}

.record-button {
  min-width: 140px;
  height: 50px;
  border-radius: 25px;
  font-size: 16px;
  font-weight: bold;
  transition: all 0.3s ease;
}

.record-button:hover {
  transform: translateY(-2px);
  box-shadow: 0 8px 20px rgba(0, 0, 0, 0.15);
}

.recording-status {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 10px;
  padding: 15px;
  background: linear-gradient(135deg, #ffebee 0%, #ffcdd2 100%);
  border-radius: 12px;
  border: 2px solid #f44336;
}

.recording-indicator {
  display: flex;
  align-items: center;
  gap: 10px;
  font-weight: bold;
  color: #d32f2f;
}

.pulse {
  width: 12px;
  height: 12px;
  background: #f44336;
  border-radius: 50%;
  animation: pulse 1s infinite;
}

@keyframes pulse {
  0% {
    transform: scale(0.8);
    opacity: 1;
  }
  50% {
    transform: scale(1.2);
    opacity: 0.7;
  }
  100% {
    transform: scale(0.8);
    opacity: 1;
  }
}

.recognition-result {
  width: 100%;
  max-width: 600px;
}

.result-card {
  border-radius: 15px;
  box-shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
  border: none;
}

.card-header {
  display: flex;
  align-items: center;
  gap: 10px;
  font-weight: 700;
  color: #2c3e50;
}

.result-content {
  display: flex;
  flex-direction: column;
  gap: 15px;
}

.recognized-text {
  padding: 10px;
  background: #f8f9fa;
  border-radius: 8px;
  border-left: 4px solid #007bff;
}

.detected-location {
  display: flex;
  align-items: center;
  gap: 10px;
  flex-wrap: wrap;
}

.no-location {
  display: flex;
  align-items: center;
  gap: 10px;
}

.matched-keywords {
  display: flex;
  align-items: center;
  gap: 10px;
  flex-wrap: wrap;
}

.keyword-tag {
  margin: 2px;
}

.result-actions {
  display: flex;
  gap: 10px;
  justify-content: center;
  margin-top: 15px;
}

.help-info {
  width: 100%;
  max-width: 500px;
}

.help-alert {
  border-radius: 10px;
}

.help-examples {
  font-size: 14px;
}

.help-examples ul {
  margin: 10px 0 0 0;
  padding-left: 20px;
}

.help-examples li {
  margin: 5px 0;
  color: #606266;
}
</style>
