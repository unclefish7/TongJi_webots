#!/usr/bin/env python3
"""
测试离线LLM语音识别功能
"""

import os
import sys
import requests

def test_ollama_service():
    """测试Ollama服务是否可用"""
    print("=" * 50)
    print("测试Ollama服务")
    print("=" * 50)
    
    try:
        response = requests.get("http://localhost:11434/api/tags", timeout=5)
        if response.status_code == 200:
            models = response.json().get('models', [])
            print("✓ Ollama服务运行正常")
            print(f"✓ 已安装 {len(models)} 个模型:")
            for model in models:
                print(f"  - {model['name']}")
            return True
        else:
            print("✗ Ollama服务响应异常")
            return False
    except Exception as e:
        print(f"✗ 无法连接到Ollama服务: {e}")
        print("\n请先运行以下命令启动Ollama:")
        print("1. ./setup_offline_llm.sh")
        print("2. 或手动启动: ollama serve")
        return False

def test_offline_llm_analysis():
    """测试离线LLM分析功能"""
    print("\n" + "=" * 50)
    print("测试离线LLM语义分析")
    print("=" * 50)
    
    # 添加路径以便导入模块
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    try:
        from services.offline_llm_service import get_offline_llm_service
        
        llm_service = get_offline_llm_service()
        
        if not llm_service.is_available:
            print("✗ 离线LLM服务不可用")
            return False
        
        print(f"✓ 使用模型: {llm_service.model_name}")
        
        # 测试用例
        test_cases = [
            "我要去经理室",
            "帮我送到财务处",
            "请送到前台",
            "我想去大办公区",
            "送到小会议室吧",
            "我要吃饭",  # 无效指令
        ]
        
        available_locations = [
            "经理室", "财务处", "等候处", "前台", "休息室",
            "小办公区", "大办公区", "小会议室", "大会议室"
        ]
        
        print(f"\n可用地点: {', '.join(available_locations)}")
        print()
        
        for i, text in enumerate(test_cases, 1):
            print(f"测试 {i}: \"{text}\"")
            
            try:
                result = llm_service.analyze_location_intent(text, available_locations)
                
                if result:
                    print(f"  ✓ 识别地点: {result}")
                else:
                    print(f"  ✗ 未识别到地点")
            except Exception as e:
                print(f"  ✗ 分析失败: {e}")
            
            print()
        
        return True
        
    except ImportError as e:
        print(f"✗ 导入模块失败: {e}")
        return False
    except Exception as e:
        print(f"✗ 测试失败: {e}")
        return False

def main():
    """主函数"""
    print("离线LLM语音识别系统测试")
    print()
    
    # 测试Ollama服务
    if not test_ollama_service():
        return
    
    # 测试离线LLM分析
    if not test_offline_llm_analysis():
        return
    
    print("=" * 50)
    print("所有测试完成!")
    print("=" * 50)
    print()
    print("现在您可以:")
    print("1. 启动API服务: uvicorn main:app --reload")
    print("2. 使用前端进行语音识别测试")
    print("3. 系统将优先使用离线LLM进行语义分析")

if __name__ == "__main__":
    main()
