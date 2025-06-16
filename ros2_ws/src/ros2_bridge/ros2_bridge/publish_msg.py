#!/usr/bin/env python3

import asyncio
import json
import importlib
import threading
from typing import Dict, Any, Optional
from aiohttp import web, ClientSession
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher


class TopicBridgeNode(Node):
    """
    ROS 2 动态话题发布桥接节点
    提供HTTP接口动态发布ROS 2话题消息
    """
    
    def __init__(self):
        super().__init__('topic_bridge_node')
        
        # 存储动态创建的发布者 - 使用话题名作为键
        self.topic_publishers: Dict[str, Publisher] = {}
        
        # 存储已加载的消息类型
        self.cached_message_types: Dict[str, type] = {}
        
        self.get_logger().info('Topic Bridge Node 已启动')
        
    def get_message_type(self, type_string: str) -> Optional[type]:
        """
        根据类型字符串动态加载ROS 2消息类型
        
        Args:
            type_string: 消息类型字符串，格式如 "std_msgs/String"
            
        Returns:
            消息类型类，如果加载失败返回None
        """
        if type_string in self.cached_message_types:
            return self.cached_message_types[type_string]
        
        try:
            # 解析包名和消息类型
            parts = type_string.split('/')
            if len(parts) != 2:
                self.get_logger().error(f'无效的消息类型格式: {type_string}')
                return None
            
            package_name, message_name = parts
            
            # 动态导入消息模块
            module_name = f'{package_name}.msg'
            module = importlib.import_module(module_name)
            
            # 获取消息类型
            message_type = getattr(module, message_name)
            
            # 缓存消息类型
            self.cached_message_types[type_string] = message_type
            
            self.get_logger().info(f'成功加载消息类型: {type_string}')
            return message_type
            
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f'加载消息类型失败 {type_string}: {str(e)}')
            return None
    
    def get_or_create_publisher(self, topic: str, message_type: type) -> Optional[Publisher]:
        """
        获取或创建发布者
        
        Args:
            topic: 话题名
            message_type: 消息类型
            
        Returns:
            发布者对象，如果创建失败返回None
        """
        # 确保话题名以 / 开头
        if not topic.startswith('/'):
            topic = '/' + topic
        
        # 检查是否已存在该话题的发布者
        if topic in self.topic_publishers:
            self.get_logger().debug(f'使用已存在的发布者: {topic}')
            return self.topic_publishers[topic]
        
        try:
            # 验证话题名格式
            if not self._is_valid_topic_name(topic):
                self.get_logger().error(f'无效的话题名格式: {topic}')
                return None
            
            # 创建新的发布者
            self.get_logger().info(f'正在创建发布者: {topic} ({message_type.__name__})')
            
            # 直接调用父类方法创建发布者
            publisher = self.create_publisher(message_type, topic, 10)
            
            # 存储发布者
            self.topic_publishers[topic] = publisher
            
            self.get_logger().info(f'成功创建发布者: {topic} ({message_type.__name__})')
            return publisher
            
        except Exception as e:
            self.get_logger().error(f'创建发布者失败 {topic}: {str(e)}')
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
            return None
    
    def _is_valid_topic_name(self, topic: str) -> bool:
        """
        验证话题名是否有效
        
        Args:
            topic: 话题名
            
        Returns:
            是否有效
        """
        # 简化验证逻辑，避免复杂的正则表达式
        if not topic.startswith('/'):
            return False
        
        # 移除开头的斜杠
        topic_name = topic[1:]
        
        # 空话题名无效
        if not topic_name:
            return False
        
        # 基本字符检查：允许字母、数字、下划线、斜杠
        allowed_chars = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_/')
        if not all(c in allowed_chars for c in topic_name):
            return False
        
        return True

    def create_message_from_dict(self, message_type: type, data: Dict[str, Any]) -> Optional[Any]:
        """
        根据字典数据创建ROS 2消息对象
        
        Args:
            message_type: 消息类型
            data: 消息数据字典
            
        Returns:
            消息对象，如果创建失败返回None
        """
        try:
            # 创建消息实例
            message = message_type()
            
            # 递归设置消息字段
            self._set_message_fields(message, data)
            
            return message
            
        except Exception as e:
            self.get_logger().error(f'创建消息失败: {str(e)}')
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
            return None
    
    def _set_message_fields(self, message: Any, data: Dict[str, Any]):
        """
        递归设置消息字段
        
        Args:
            message: 消息对象
            data: 数据字典
        """
        for field_name, field_value in data.items():
            if hasattr(message, field_name):
                try:
                    if isinstance(field_value, dict):
                        # 嵌套消息
                        nested_msg = getattr(message, field_name)
                        self._set_message_fields(nested_msg, field_value)
                    else:
                        # 基本类型字段
                        setattr(message, field_name, field_value)
                except Exception as e:
                    self.get_logger().error(f'设置字段 {field_name} 失败: {str(e)}')
            else:
                self.get_logger().warn(f'消息类型中不存在字段: {field_name}')
    
    def publish_message(self, topic: str, type_string: str, data: Dict[str, Any]) -> tuple[bool, str]:
        """
        发布消息
        
        Args:
            topic: 话题名
            type_string: 消息类型字符串
            data: 消息数据
            
        Returns:
            (成功标志, 错误信息)
        """
        try:
            self.get_logger().debug(f'开始发布消息到话题: {topic}, 类型: {type_string}')
            
            # 获取消息类型
            message_type = self.get_message_type(type_string)
            if message_type is None:
                return False, f'无法加载消息类型: {type_string}'
            
            # 获取或创建发布者
            publisher = self.get_or_create_publisher(topic, message_type)
            if publisher is None:
                return False, f'无法创建发布者: {topic}'
            
            # 创建消息
            message = self.create_message_from_dict(message_type, data)
            if message is None:
                return False, '创建消息失败'
            
            # 发布消息
            publisher.publish(message)
            
            self.get_logger().info(f'成功发布消息到话题: {topic}')
            return True, 'success'
            
        except Exception as e:
            error_msg = f'发布消息时发生错误: {str(e)}'
            self.get_logger().error(error_msg)
            import traceback
            self.get_logger().error(f'详细错误信息: {traceback.format_exc()}')
            return False, error_msg


class HTTPServer:
    """
    HTTP服务器类，处理REST API请求
    """
    
    def __init__(self, ros_node: TopicBridgeNode, port: int = 8080):
        self.ros_node = ros_node
        self.port = port
        self.app = web.Application()
        self._setup_routes()
        
    def _setup_routes(self):
        """设置路由"""
        self.app.router.add_post('/publish_topic', self.handle_publish_topic)
        self.app.router.add_get('/health', self.handle_health)
        
    async def handle_publish_topic(self, request):
        """
        处理发布话题请求
        
        Expected JSON format:
        {
            "topic": "/topic_name",
            "type": "std_msgs/String", 
            "data": {"data": "hello world"}
        }
        """
        try:
            # 解析JSON请求
            data = await request.json()
            
            # 验证必需字段
            required_fields = ['topic', 'type', 'data']
            for field in required_fields:
                if field not in data:
                    return web.json_response(
                        {'success': False, 'error': f'缺少必需字段: {field}'}, 
                        status=400
                    )
            
            topic = data['topic']
            msg_type = data['type']
            msg_data = data['data']
            
            # 发布消息
            success, message = self.ros_node.publish_message(topic, msg_type, msg_data)
            
            if success:
                return web.json_response({'success': True})
            else:
                return web.json_response(
                    {'success': False, 'error': message}, 
                    status=400
                )
                
        except json.JSONDecodeError:
            return web.json_response(
                {'success': False, 'error': '无效的JSON格式'}, 
                status=400
            )
        except Exception as e:
            self.ros_node.get_logger().error(f'处理请求时发生错误: {str(e)}')
            return web.json_response(
                {'success': False, 'error': '服务器内部错误'}, 
                status=500
            )
    
    async def handle_health(self, request):
        """健康检查接口"""
        return web.json_response({'status': 'healthy', 'node': 'topic_bridge_node'})
    
    async def start_server(self):
        """启动HTTP服务器"""
        runner = web.AppRunner(self.app)
        await runner.setup()
        
        site = web.TCPSite(runner, '0.0.0.0', self.port)
        await site.start()
        
        self.ros_node.get_logger().info(f'HTTP服务器已启动，端口: {self.port}')
        self.ros_node.get_logger().info(f'API端点: http://localhost:{self.port}/publish_topic')


def ros_spin_thread(node):
    """ROS节点运行线程"""
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'ROS节点运行时发生错误: {str(e)}')


async def main_async():
    """异步主函数"""
    ros_node = None
    
    try:
        # 初始化ROS 2
        rclpy.init()
        
        # 创建ROS节点
        ros_node = TopicBridgeNode()
        
        # 在单独线程中运行ROS节点
        ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
        ros_thread.start()
        
        # 创建并启动HTTP服务器
        http_server = HTTPServer(ros_node, port=8080)
        await http_server.start_server()
        
        ros_node.get_logger().info('Topic Bridge Node 完全启动')
        ros_node.get_logger().info('使用示例:')
        ros_node.get_logger().info('curl -X POST http://localhost:8080/publish_topic \\')
        ros_node.get_logger().info('  -H "Content-Type: application/json" \\')
        ros_node.get_logger().info('  -d \'{"topic":"/test","type":"std_msgs/String","data":{"data":"hello"}}\'')
        
        # 保持服务器运行
        while True:
            await asyncio.sleep(1)
            
    except KeyboardInterrupt:
        if ros_node:
            ros_node.get_logger().info('收到中断信号，正在关闭...')
        else:
            print('收到中断信号，正在关闭...')
    except Exception as e:
        if ros_node:
            ros_node.get_logger().error(f'主函数发生错误: {str(e)}')
        else:
            print(f'主函数发生错误: {str(e)}')
    finally:
        # 清理资源
        if ros_node is not None:
            ros_node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """
    主函数
    """
    try:
        # 运行异步主函数
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print('\n程序被用户中断')
    except Exception as e:
        print(f'程序运行时发生错误: {str(e)}')


if __name__ == '__main__':
    main()
