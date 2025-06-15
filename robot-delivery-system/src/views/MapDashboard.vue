<template>
  <div class="map-dashboard">
    <div class="dashboard-header">
      <h2>机器人实时地图监控</h2>
      <div class="mode-selector">
        <el-radio-group v-model="mapMode" @change="handleModeChange">
          <el-radio-button value="ros">ROS实时地图</el-radio-button>
          <el-radio-button value="simulated">模拟地图</el-radio-button>
        </el-radio-group>
      </div>
    </div>

    <div class="map-content">
      <RealTimeMapViewer v-if="mapMode === 'ros'" />
      <SimulatedMapViewer v-else />
    </div>

    <div class="instructions" v-if="showInstructions">
      <el-card class="instruction-card">
        <template #header>
          <div class="card-header">
            <span>使用说明</span>
            <el-button type="text" @click="showInstructions = false">关闭</el-button>
          </div>
        </template>
        
        <div class="instruction-content">
          <div class="mode-instruction">
            <h4>ROS实时地图模式</h4>
            <ul>
              <li>需要启动 rosbridge_server: <code>ros2 launch rosbridge_server rosbridge_websocket_launch.xml</code></li>
              <li>确保发布了 /map 话题和 /tf 话题</li>
              <li>支持实时显示占用栅格地图和机器人位置</li>
              <li>可以通过鼠标滚轮缩放，拖拽平移</li>
            </ul>
          </div>
          
          <div class="mode-instruction">
            <h4>模拟地图模式</h4>
            <ul>
              <li>使用预定义的语义地图进行模拟</li>
              <li>点击地图任意位置让机器人移动到该位置</li>
              <li>可以点击语义位置标签快速导航</li>
              <li>支持手动输入坐标进行精确控制</li>
            </ul>
          </div>
          
          <div class="mode-instruction">
            <h4>通用操作</h4>
            <ul>
              <li>鼠标滚轮：缩放地图</li>
              <li>鼠标拖拽：平移地图</li>
              <li>居中按钮：将视图居中到机器人位置</li>
            </ul>
          </div>
        </div>
      </el-card>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import { ElRadioGroup, ElRadioButton, ElCard, ElButton, ElMessage } from 'element-plus'
import RealTimeMapViewer from '@/components/RealTimeMapViewer.vue'
import SimulatedMapViewer from '@/components/SimulatedMapViewer.vue'

// 响应式数据
const mapMode = ref<'ros' | 'simulated'>('simulated')
const showInstructions = ref(true)

// 处理模式切换
const handleModeChange = (mode: string | number | boolean | undefined) => {
  if (mode === 'ros') {
    ElMessage.info('切换到ROS实时地图模式，请确保ROS环境已正确配置')
  } else if (mode === 'simulated') {
    ElMessage.info('切换到模拟地图模式')
  }
}
</script>

<style scoped>
.map-dashboard {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background: #f5f5f5;
}

.dashboard-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 24px;
  background: white;
  border-bottom: 1px solid #e0e0e0;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
}

.dashboard-header h2 {
  margin: 0;
  color: #2c3e50;
  font-size: 24px;
}

.mode-selector {
  display: flex;
  align-items: center;
  gap: 12px;
}

.map-content {
  flex: 1;
  position: relative;
}

.instructions {
  position: fixed;
  top: 20px;
  right: 20px;
  width: 400px;
  z-index: 1000;
}

.instruction-card {
  max-height: 80vh;
  overflow-y: auto;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.instruction-content {
  line-height: 1.6;
}

.mode-instruction {
  margin-bottom: 20px;
}

.mode-instruction h4 {
  margin: 0 0 8px 0;
  color: #409eff;
}

.mode-instruction ul {
  margin: 8px 0;
  padding-left: 20px;
}

.mode-instruction li {
  margin-bottom: 4px;
}

.mode-instruction code {
  background: #f5f5f5;
  padding: 2px 6px;
  border-radius: 3px;
  font-family: 'Courier New', monospace;
  font-size: 12px;
}
</style>
