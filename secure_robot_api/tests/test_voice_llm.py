#!/usr/bin/env python3
"""
测试语音识别和LLM语义分析功能
"""

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from services.voice_service import voice_service

def test_llm_semantic_analysis():
    """测试LLM语义分析功能"""
    
    print("=" * 50)
    print("测试 LLM 语义分析功能")
    print("=" * 50)
    
    # 测试用例
    test_cases = [
        "我要去经理室",
        "帮我送到财务处",
        "请送到前台",
        "我想去大办公区",
        "送到小会议室吧",
        "麻烦送到休息室",
        "我要去等候处等人",
        "送到老板那里",
        "帮我送到会计那边",
        "我要到接待处",
        "请送到茶水间",
        "去办公区",
        "送到会议室",
        "我要吃饭",  # 无效指令
        "今天天气不错",  # 无效指令
    ]
    
    print(f"可用地点: {', '.join(voice_service.get_supported_locations())}")
    print()
    
    for i, text in enumerate(test_cases, 1):
        print(f"测试 {i}: \"{text}\"")
        
        # 提取地点
        location, keywords, analysis_method = voice_service.extract_location(text)
        
        print(f"  📊 分析方法: {analysis_method}")
        if location:
            print(f"  ✓ 识别地点: {location}")
            if keywords:
                print(f"  ✓ 匹配关键词: {', '.join(keywords)}")
        else:
            print(f"  ✗ 未识别到地点")
        
        print()

def test_environment_setup():
    """测试环境配置"""
    
    print("=" * 50)
    print("检查环境配置")
    print("=" * 50)
    
    # 检查Vosk模型
    if voice_service.model and voice_service.rec:
        print("✓ Vosk语音识别模型已加载")
    else:
        print("✗ Vosk语音识别模型未加载")
    
    # 检查OpenAI配置
    if voice_service.openai_client:
        print("✓ OpenAI客户端已初始化")
        print("  ✓ 将使用LLM进行语义分析")
    else:
        print("✗ OpenAI客户端未初始化")
        print("  - 将使用传统关键词匹配")
        
    # 检查环境变量
    api_key = os.getenv('OPENAI_API_KEY')
    if api_key:
        print(f"✓ 检测到API密钥: {api_key[:8]}...")
    else:
        print("✗ 未设置OPENAI_API_KEY环境变量")
    
    base_url = os.getenv('OPENAI_BASE_URL', 'https://api.openai.com/v1')
    print(f"  API Base URL: {base_url}")
    
    print()

if __name__ == "__main__":
    # 测试环境配置
    test_environment_setup()
    
    # 测试语义分析
    test_llm_semantic_analysis()
    
    print("=" * 50)
    print("测试完成!")
    print("=" * 50)
    print()
    print("使用说明:")
    print("1. 确保已设置OPENAI_API_KEY环境变量")
    print("2. 运行 pip install -r requirements.txt")
    print("3. 运行 pip install -r requirements_voice.txt") 
    print("4. 运行 ./setup_vosk.sh 下载语音模型")
    print("5. 启动API服务: uvicorn main:app --reload")
