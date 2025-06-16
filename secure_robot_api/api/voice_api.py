from fastapi import APIRouter, UploadFile, File, HTTPException
from fastapi.responses import JSONResponse
import tempfile
import os
from services.voice_service import voice_service

router = APIRouter(tags=["voice"])

@router.post("/recognize")
async def recognize_voice_command(audio: UploadFile = File(...)):
    """
    语音识别接口
    
    接受音频文件，返回识别的文本和提取的地点信息
    
    - **audio**: 音频文件 (支持WAV, MP3, M4A等格式)
    
    返回:
    - **success**: 识别是否成功
    - **text**: 识别出的完整文本
    - **location**: 匹配到的地点名称
    - **matched_keywords**: 匹配到的关键词列表
    - **available_locations**: 所有可用的地点列表
    """
    try:
        # 验证文件类型
        if not audio.content_type or not audio.content_type.startswith('audio/'):
            raise HTTPException(status_code=400, detail="请上传音频文件")
        
        # 创建临时文件保存上传的音频
        with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_file:
            # 读取上传的音频数据
            audio_data = await audio.read()
            temp_file.write(audio_data)
            temp_file_path = temp_file.name
        
        try:
            # 处理语音指令
            result = voice_service.process_voice_command(temp_file_path)
            
            # 添加额外信息
            result.update({
                "filename": audio.filename,
                "file_size": len(audio_data),
                "content_type": audio.content_type
            })
            
            return JSONResponse(content=result)
            
        finally:
            # 清理临时文件
            if os.path.exists(temp_file_path):
                os.unlink(temp_file_path)
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"语音识别处理失败: {str(e)}")

@router.get("/locations")
async def get_available_locations():
    """
    获取所有可用的地点列表
    
    返回所有预设的地点名称和对应的关键词
    """
    try:
        return {
            "locations": voice_service.location_keywords,
            "location_names": list(voice_service.location_keywords.keys())
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取地点列表失败: {str(e)}")

@router.post("/test-recognition")
async def test_voice_recognition(text: str):
    """
    测试文本地点提取功能
    
    用于测试地点关键词匹配逻辑，无需上传音频
    
    - **text**: 要测试的文本内容
    """
    try:
        location, keywords = voice_service.extract_location(text)
        
        return {
            "input_text": text,
            "extracted_location": location,
            "matched_keywords": keywords,
            "available_locations": list(voice_service.location_keywords.keys())
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"文本处理失败: {str(e)}")
