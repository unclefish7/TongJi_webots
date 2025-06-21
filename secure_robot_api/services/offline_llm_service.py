"""
离线LLM服务 - 使用Ollama部署本地大语言模型
"""

import json
import requests
import subprocess
import os
from typing import Optional, Dict, Any

class OfflineLLMService:
    """离线LLM服务类"""
    
    def __init__(self):
        self.ollama_url = "http://localhost:11434"
        self.model_name = "qwen2:7b"  # 使用通义千问7B模型
        self.backup_model = "llama3.1:8b"  # 备用模型
        self.is_available = False
        self._check_ollama_status()
    
    def _check_ollama_status(self):
        """检查Ollama服务状态"""
        try:
            response = requests.get(f"{self.ollama_url}/api/tags", timeout=5)
            if response.status_code == 200:
                self.is_available = True
                models = response.json().get('models', [])
                model_names = [model['name'] for model in models]
                
                # 检查主要模型是否可用
                if not any(self.model_name in name for name in model_names):
                    if any(self.backup_model in name for name in model_names):
                        self.model_name = self.backup_model
                        print(f"使用备用模型: {self.backup_model}")
                    else:
                        print(f"警告: 未找到推荐的模型 {self.model_name} 或 {self.backup_model}")
                        if model_names:
                            self.model_name = model_names[0]
                            print(f"使用第一个可用模型: {self.model_name}")
                
                print(f"Ollama服务可用，使用模型: {self.model_name}")
            else:
                self.is_available = False
                print("Ollama服务不可用")
        except Exception as e:
            self.is_available = False
            print(f"检查Ollama服务失败: {e}")
    
    def analyze_location_intent(self, text: str, available_locations: list) -> Optional[str]:
        """
        使用离线LLM分析用户意图并提取地点
        
        Args:
            text: 用户语音识别的文本
            available_locations: 可选择的地点列表
            
        Returns:
            匹配的地点名称，如果没有匹配返回None
        """
        if not self.is_available:
            return None
            
        try:
            # 构建提示词
            prompt = f"""你是一个智能语音助手，需要从用户的语音指令中识别目的地。

可选择的地点列表：
{', '.join(available_locations)}

用户说的话："{text}"

请分析用户想要去哪个地点。如果用户的话中包含明确的地点信息，请返回最匹配的地点名称。如果无法确定或没有提到地点，请返回"无法确定"。

规则：
1. 只返回地点名称，不要其他解释
2. 必须从上述地点列表中选择
3. 如果用户提到了"经理"、"老板"等，对应"经理室"
4. 如果用户提到了"财务"、"会计"等，对应"财务处"
5. 如果用户提到了"前台"、"接待"等，对应"前台"
6. 如果用户提到了"等候"、"等待"等，对应"等候处"
7. 如果用户提到了"休息"、"茶水间"等，对应"休息室"
8. 如果用户提到了"办公"但没有明确大小，对应"大办公区"
9. 如果用户提到了"会议"但没有明确大小，对应"大会议室"

请直接回答地点名称："""

            # 调用Ollama API
            payload = {
                "model": self.model_name,
                "prompt": prompt,
                "stream": False,
                "options": {
                    "temperature": 0.1,
                    "top_p": 0.9,
                    "num_predict": 50
                }
            }
            
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json=payload,
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                answer = result.get('response', '').strip()
                
                # 验证结果是否在可选地点中
                if answer in available_locations:
                    print(f"离线LLM识别地点: {answer}")
                    return answer
                elif answer != "无法确定":
                    print(f"离线LLM返回了无效地点: {answer}")
                    
                return None
            else:
                print(f"离线LLM请求失败: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"离线LLM分析出错: {e}")
            return None
    
    def install_model(self, model_name: str = None) -> bool:
        """安装指定的模型"""
        if model_name is None:
            model_name = self.model_name
            
        try:
            print(f"正在安装模型: {model_name}")
            result = subprocess.run(
                ["ollama", "pull", model_name],
                capture_output=True,
                text=True,
                timeout=300  # 5分钟超时
            )
            
            if result.returncode == 0:
                print(f"模型 {model_name} 安装成功")
                self._check_ollama_status()  # 重新检查状态
                return True
            else:
                print(f"模型安装失败: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print("模型安装超时")
            return False
        except Exception as e:
            print(f"模型安装出错: {e}")
            return False
    
    def get_available_models(self) -> list:
        """获取可用的模型列表"""
        try:
            response = requests.get(f"{self.ollama_url}/api/tags", timeout=5)
            if response.status_code == 200:
                models = response.json().get('models', [])
                return [model['name'] for model in models]
            return []
        except Exception as e:
            print(f"获取模型列表失败: {e}")
            return []

# 全局离线LLM服务实例
offline_llm_service = OfflineLLMService()

def get_offline_llm_service() -> OfflineLLMService:
    """获取离线LLM服务实例"""
    return offline_llm_service
