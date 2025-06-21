import json
import os
import wave
import tempfile
import re
from typing import Dict, List, Optional, Tuple
import vosk
import librosa
import soundfile as sf
import numpy as np
import openai
from openai import OpenAI
from dotenv import load_dotenv
from .offline_llm_service import get_offline_llm_service

# 加载环境变量
load_dotenv()

class VoiceRecognitionService:
    """语音识别服务类"""
    
    def __init__(self):
        # 预设的地点关键词映射
        self.location_keywords = {
            "经理室": ["经理室", "经理", "老板"],
            "财务处": ["财务处", "财务", "会计"],
            "等候处": ["等候处", "等候", "等待"],
            "前台": ["前台", "接待"],
            "休息室": ["休息室", "休息", "茶水间"],
            "小办公区": ["小办公区", "小办公", "办公区"],
            "大办公区": ["大办公区", "大办公", "主办公区"],
            "小会议室": ["小会议室", "小会议", "会议室"],
            "大会议室": ["大会议室", "大会议", "主会议室"]
        }
        
        # 支持的地点列表（用于LLM分析）
        self.available_locations = list(self.location_keywords.keys())
        
        # 初始化Vosk模型
        self.model = None
        self.rec = None
        self._init_model()
        
        # 初始化OpenAI客户端
        self.openai_client = None
        self._init_llm()
        
        # 初始化离线LLM服务
        self.offline_llm = get_offline_llm_service()
    
    def _init_model(self):
        """初始化Vosk语音识别模型"""
        try:
            # 模型路径（优先使用小模型）
            model_paths = [
                "/home/jerry/Documents/webots/secure_robot_api/models/vosk-model-small-cn-0.22",
                "/home/jerry/Documents/webots/secure_robot_api/models/vosk-model-cn-0.22"
            ]
            
            model_path = None
            for path in model_paths:
                if os.path.exists(path):
                    model_path = path
                    break
            
            if not model_path:
                raise FileNotFoundError("未找到Vosk中文模型，请运行setup_vosk.sh下载模型")
            
            # 初始化模型
            vosk.SetLogLevel(-1)  # 减少日志输出
            self.model = vosk.Model(model_path)
            self.rec = vosk.KaldiRecognizer(self.model, 16000)
            
            print(f"Vosk模型初始化成功: {model_path}")
            
        except Exception as e:
            print(f"Vosk模型初始化失败: {e}")
            self.model = None
            self.rec = None
    
    def _init_llm(self):
        """初始化LLM客户端"""
        try:
            # 从环境变量获取API密钥
            api_key = os.getenv('OPENAI_API_KEY')
            base_url = os.getenv('OPENAI_BASE_URL', 'https://api.openai.com/v1')  # 支持自定义端点
            
            if api_key:
                self.openai_client = OpenAI(
                    api_key=api_key,
                    base_url=base_url
                )
                print("LLM客户端初始化成功")
            else:
                print("警告: 未设置OPENAI_API_KEY环境变量，将使用传统关键词匹配")
                
        except Exception as e:
            print(f"LLM客户端初始化失败: {e}")
            self.openai_client = None
    
    def preprocess_audio(self, audio_file_path: str) -> str:
        """
        预处理音频文件，转换为Vosk需要的格式
        
        Args:
            audio_file_path: 输入音频文件路径
            
        Returns:
            处理后的WAV文件路径
        """
        try:
            # 使用librosa加载音频
            audio_data, sample_rate = librosa.load(audio_file_path, sr=16000, mono=True)
            
            # 创建临时WAV文件
            temp_wav = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
            
            # 保存为16kHz单声道WAV
            sf.write(temp_wav.name, audio_data, 16000, subtype='PCM_16')
            
            return temp_wav.name
            
        except Exception as e:
            print(f"音频预处理失败: {e}")
            return audio_file_path
    
    def recognize_speech(self, audio_file_path: str) -> str:
        """
        识别音频文件中的语音内容
        
        Args:
            audio_file_path: 音频文件路径
            
        Returns:
            识别出的文本内容
        """
        if not self.model or not self.rec:
            return ""
        
        try:
            # 预处理音频
            processed_audio = self.preprocess_audio(audio_file_path)
            
            # 读取WAV文件
            with wave.open(processed_audio, 'rb') as wf:
                # 检查音频格式
                if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getframerate() != 16000:
                    print("警告: 音频格式不是16kHz单声道16bit，可能影响识别效果")
                
                # 分块读取音频数据进行识别
                results = []
                while True:
                    data = wf.readframes(4000)
                    if len(data) == 0:
                        break
                    
                    if self.rec.AcceptWaveform(data):
                        result = json.loads(self.rec.Result())
                        if result.get('text'):
                            results.append(result['text'])
                
                # 获取最终结果
                final_result = json.loads(self.rec.FinalResult())
                if final_result.get('text'):
                    results.append(final_result['text'])
            
            # 清理临时文件
            if processed_audio != audio_file_path:
                os.unlink(processed_audio)
            
            # 合并所有识别结果
            full_text = ' '.join(results).strip()
            print(f"语音识别结果: {full_text}")
            
            return full_text
            
        except Exception as e:
            print(f"语音识别失败: {e}")
            return ""
    
    def extract_location(self, text: str) -> Tuple[Optional[str], List[str], str]:
        """
        从识别文本中提取地点信息，优先使用离线LLM，然后在线LLM，最后关键词匹配
        
        Args:
            text: 识别出的文本
            
        Returns:
            (匹配的地点名称, 所有可能的匹配关键词, 分析方法)
        """
        if not text:
            return None, [], "Keywords"
        
        # 1. 优先使用离线LLM进行语义分析
        if self.offline_llm.is_available:
            try:
                offline_result = self.offline_llm.analyze_location_intent(text, self.available_locations)
                if offline_result:
                    return offline_result, [], "Offline-LLM"
            except Exception as e:
                print(f"离线LLM分析失败: {e}")
        
        # 2. 回退到在线LLM
        if self.openai_client:
            try:
                online_result = self._analyze_with_llm(text)
                if online_result:
                    return online_result, [], "Online-LLM"
            except Exception as e:
                print(f"在线LLM分析失败，继续回退到关键词匹配: {e}")
        
        # 3. 最终回退到传统关键词匹配
        location, keywords = self._extract_location_by_keywords(text)
        return location, keywords, "Keywords"
    
    def _analyze_with_llm(self, text: str) -> Optional[str]:
        """
        使用LLM分析用户意图并提取地点
        
        Args:
            text: 用户语音识别的文本
            
        Returns:
            匹配的地点名称，如果没有匹配返回None
        """
        if not self.openai_client:
            return None
            
        try:
            # 构建提示词
            prompt = f"""
你是一个智能语音助手，需要从用户的语音指令中识别目的地。

可选择的地点列表：
{', '.join(self.available_locations)}

用户说的话："{text}"

请分析用户想要去哪个地点。如果用户的话中包含明确的地点信息，请返回最匹配的地点名称。如果无法确定或没有提到地点，请返回"无法确定"。

要求：
1. 只返回地点名称，不要其他解释
2. 必须从上述地点列表中选择
3. 如果用户提到了"经理"、"老板"等，对应"经理室"
4. 如果用户提到了"财务"、"会计"等，对应"财务处"
5. 如果用户提到了"前台"、"接待"等，对应"前台"
6. 如果用户提到了"等候"、"等待"等，对应"等候处"
7. 如果用户提到了"休息"、"茶水间"等，对应"休息室"
8. 如果用户提到了"办公"但没有明确大小，对应"大办公区"
9. 如果用户提到了"会议"但没有明确大小，对应"大会议室"

请直接回答地点名称：
"""

            # 调用LLM
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",  # 或者使用其他模型
                messages=[
                    {"role": "system", "content": "你是一个专业的语音助手，专门识别用户想要去的地点。"},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                max_tokens=50
            )
            
            result = response.choices[0].message.content.strip()
            
            # 验证结果是否在可选地点中
            if result in self.available_locations:
                print(f"LLM识别地点: {result}")
                return result
            elif result != "无法确定":
                print(f"LLM返回了无效地点: {result}")
                
            return None
            
        except Exception as e:
            print(f"LLM分析出错: {e}")
            return None
    
    def _extract_location_by_keywords(self, text: str) -> Tuple[Optional[str], List[str]]:
        """
        使用传统关键词匹配提取地点（作为LLM的回退方案）
        
        Args:
            text: 识别出的文本
            
        Returns:
            (匹配的地点名称, 所有可能的匹配关键词)
        """
        text = text.replace(" ", "").lower()  # 移除空格并转换为小写
        matched_locations = []
        
        # 遍历所有预设地点
        for location, keywords in self.location_keywords.items():
            for keyword in keywords:
                if keyword in text:
                    matched_locations.append((location, keyword))
                    break
        
        # 按关键词长度排序，优先返回最具体的匹配
        if matched_locations:
            matched_locations.sort(key=lambda x: len(x[1]), reverse=True)
            best_match = matched_locations[0]
            return best_match[0], [match[1] for match in matched_locations]
        
        return None, []
    
    def process_voice_command(self, audio_file_path: str) -> Dict:
        """
        处理语音指令，返回完整的识别结果
        
        Args:
            audio_file_path: 音频文件路径
            
        Returns:
            包含识别文本和地点信息的字典
        """
        # 识别语音
        recognized_text = self.recognize_speech(audio_file_path)
        
        # 提取地点
        location, keywords, analysis_method = self.extract_location(recognized_text)
        
        return {
            "success": bool(recognized_text),
            "text": recognized_text,
            "location": location,
            "matched_keywords": keywords,
            "analysis_method": analysis_method,
            "available_locations": list(self.location_keywords.keys())
        }
    
    def get_supported_locations(self) -> list:
        """获取支持的地点列表"""
        return list(self.location_keywords.keys())

# 全局语音识别服务实例
voice_service = VoiceRecognitionService()

def get_voice_service() -> VoiceRecognitionService:
    """获取语音识别服务实例"""
    return voice_service

def get_supported_locations() -> list:
    """获取支持的地点列表"""
    return voice_service.get_supported_locations()
