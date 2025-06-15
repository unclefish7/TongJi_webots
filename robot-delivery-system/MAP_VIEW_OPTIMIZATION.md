# 地图视角和机器人尺寸优化更新

## 主要改动

### 1. 地图视角优化 🎯
- **地图中心定位**: 接收ROS地图后，视角中心自动定位到地图的中心点
- **合理缩放比例**: 地图显示比例调整为容器尺寸的60%，确保地图清晰可见
- **一致性保证**: `requestMap()` 和 `resizeCanvas()` 函数使用相同的视角设置逻辑

### 2. 机器人图标优化 🤖
- **缩小尺寸**: 大幅减小机器人图标尺寸，避免在地图上过于突出
- **保证可见**: 虽然缩小但仍保证在各种缩放级别下都能清晰看到
- **边框加强**: 添加深红色边框，提高机器人图标的可识别性

## 具体参数调整

### 地图视角参数
```typescript
// 显示比例从8%增加到60%
const scaleX = (container.clientWidth * 0.6) / map.width
const scaleY = (container.clientHeight * 0.6) / map.height

// 最大缩放从0.3增加到2.0
const autoScale = Math.min(scaleX, scaleY, 2.0)

// 视角中心定位到地图中心
const mapCenterX = mapWidth / 2
const mapCenterY = mapHeight / 2
offsetX.value = container.clientWidth / 2 - mapCenterX * scale.value
offsetY.value = container.clientHeight / 2 - mapCenterY * scale.value
```

### 机器人尺寸参数
```typescript
// 基础尺寸从2.0/5.0减小到0.5/1.0
const baseSize = Math.max(0.5, 1.0 / scale.value)

// 箭头长度从1.5减小到1.2
const arrowLength = (baseSize * 1.2) / map.resolution

// 箭头宽度从0.4减小到0.3
const arrowWidth = (baseSize * 0.3) / map.resolution

// 线宽从1.0/0.2减小到0.5/0.15
const lineWidth = Math.max(0.5, (baseSize * 0.15) / map.resolution)
```

## 预期效果

### 地图显示 📍
- ✅ 接收ROS地图后，视角自动居中到地图中心
- ✅ 地图以合适的比例显示，既不会太小也不会太大
- ✅ 用户可以清晰看到整个地图的结构和细节

### 机器人图标 🎯
- ✅ 机器人图标大小适中，不会遮挡地图细节
- ✅ 红色填充 + 深红色边框，确保在各种背景下都清晰可见
- ✅ 朝向箭头清晰显示机器人的方向

### 用户体验 🚀
- ✅ 获取地图后无需手动调整视角
- ✅ 机器人位置一目了然但不会干扰地图查看
- ✅ 保持了所有交互功能（缩放、拖拽、居中等）

## 测试建议

1. **连接ROS并获取地图**
   - 验证地图是否居中显示
   - 检查地图显示比例是否合适

2. **查看机器人图标**
   - 确认机器人图标大小是否合适
   - 验证机器人图标是否清晰可见

3. **测试交互功能**
   - 验证拖拽、缩放功能正常
   - 测试机器人居中功能
   - 检查机器人追踪功能

现在地图视角会自动居中到地图中心，机器人图标也调整为合适的大小！
