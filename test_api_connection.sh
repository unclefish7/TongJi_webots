#!/bin/bash

# API连接测试脚本
echo "=== 前后端API连接测试 ==="

API_BASE="http://localhost:8000"

echo "1. 测试任务API..."

# 测试ping接口
echo "测试 Ping 接口..."
curl -X GET "${API_BASE}/api/tasks/ping" \
  -H "Content-Type: application/json" \
  | python3 -m json.tool

echo -e "\n2. 测试队列状态接口..."
curl -X GET "${API_BASE}/api/tasks/queue/status" \
  -H "Content-Type: application/json" \
  | python3 -m json.tool

echo -e "\n3. 测试创建任务接口..."
curl -X POST "${API_BASE}/api/tasks/create" \
  -H "Content-Type: application/json" \
  -d '{
    "user_id": "E001",
    "receiver": "E002", 
    "location_id": "A101",
    "security_level": "L1",
    "description": "测试任务"
  }' \
  | python3 -m json.tool

echo -e "\n4. 测试用户API..."
curl -X GET "${API_BASE}/api/users" \
  -H "Content-Type: application/json" \
  | python3 -m json.tool

echo -e "\n5. 测试认证API..."
curl -X POST "${API_BASE}/api/auth/verify" \
  -H "Content-Type: application/json" \
  -d '{
    "user_id": "E001",
    "requested_level": "L1",
    "provided": {}
  }' \
  | python3 -m json.tool

echo -e "\n6. 测试取件API..."
curl -X GET "${API_BASE}/api/pickup/tasks?user_id=E001" \
  -H "Content-Type: application/json" \
  | python3 -m json.tool

echo -e "\n=== 测试完成 ==="
