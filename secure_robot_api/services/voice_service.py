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
        
        # 初始化Vosk模型
        self.model = None
        self.rec = None
        self._init_model()
    
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
    
    def extract_location(self, text: str) -> Tuple[Optional[str], List[str]]:
        """
        从识别文本中提取地点信息
        
        Args:
            text: 识别出的文本
            
        Returns:
            (匹配的地点名称, 所有可能的匹配关键词)
        """
        if not text:
            return None, []
        
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
        location, keywords = self.extract_location(recognized_text)
        
        return {
            "success": bool(recognized_text),
            "text": recognized_text,
            "location": location,
            "matched_keywords": keywords,
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
