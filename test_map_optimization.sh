#!/bin/bash

echo "=== 测试地图显示优化 ==="

# 检查前端服务是否运行
if ! curl -s http://localhost:5173 > /dev/null; then
    echo "启动前端服务..."
    cd /home/jerry/Documents/webots/robot-delivery-system
    npm run dev &
    sleep 5
fi

echo "✓ 前端服务已启动"
echo "📍 请访问 http://localhost:5173 测试以下功能："
echo ""
echo "🔧 测试步骤："
echo "1. 打开主页面，查看嵌入的地图组件"
echo "2. 选择'模拟'模式（默认）"
echo "3. 观察地图缩放比例是否合适"
echo "4. 检查机器人图标大小是否合理"
echo "5. 测试'重置视图'按钮"
echo "6. 如果有ROS环境，测试'ROS'模式"
echo ""
echo "🎯 预期效果："
echo "- 地图应显示在合适的大小（不会太大占满整个框）"
echo "- 机器人图标应该很小，清晰可见但不突兀"
echo "- 可以通过滚轮缩放到很小的比例查看全貌"
echo "- '重置视图'按钮应该能恢复到最佳显示状态"
echo ""
echo "📊 关键参数调整："
echo "- 初始缩放: 5 → 2"
echo "- 地图显示比例: 0.5 → 0.3 (ROS), 0.8 → 0.6 (模拟)"
echo "- 机器人基础尺寸: 大幅缩小"
echo "- 最小缩放限制: 0.5 → 0.1"
