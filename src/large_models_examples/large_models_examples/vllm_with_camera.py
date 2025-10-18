#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import os
import cv2
import json
import queue
import rclpy
import threading
import numpy as np
import time

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool, Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetModel, SetString, SetInt32
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition, ServoPosition

VLLM_PROMPT = '''
'''

display_size = [int(640*6/4), int(360*6/4)]

class VLLMWithCamera(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        # 初始化变量
        self.image_queue = queue.Queue(maxsize=2)
        self.set_above = False
        self.vllm_result = ''
        self.running = True
        self.action_finish = False
        self.play_audio_finish = False
        self.bridge = CvBridge()
        self.client = speech.OpenAIAPI(api_key, base_url)
        
        # 添加调试标志
        self.image_received = False
        self.first_image_time = None
        
        # 声明参数
        self.declare_parameter('camera_topic', '/ascamera/camera_publisher/rgb0/image')
        camera_topic = self.get_parameter('camera_topic').value
        
        # 打印相机话题信息
        self.get_logger().info(f'Camera topic: {camera_topic}')
        
        # 创建回调组
        timer_cb_group = ReentrantCallbackGroup()
        
        # 创建发布者
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        
        # 创建订阅者
        self.create_subscription(Image, camera_topic, self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        
        # 创建客户端
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_mode_client = self.create_client(SetInt32, 'vocal_detect/set_mode')
        self.set_mode_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        
        # 创建初始化定时器
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        """初始化处理流程"""
        self.timer.cancel()
        
        # 设置模型
        msg = SetModel.Request()
        msg.model_type = 'vllm'
        if os.environ['ASR_LANGUAGE'] == 'Chinese':
            msg.model = stepfun_vllm_model
            msg.api_key = stepfun_api_key
            msg.base_url = stepfun_base_url
        else:
            msg.model = vllm_model
            msg.api_key = vllm_api_key
            msg.base_url = vllm_base_url
        self.send_request(self.set_model_client, msg)

        # 设置提示词
        msg = SetString.Request()
        msg.data = VLLM_PROMPT
        self.send_request(self.set_prompt_client, msg)

        # 设置机械臂初始位置
        set_servo_position(self.joints_pub, 1.0,
                           ((1, 500), (2, 645), (3, 135), (4, 80), (5, 500), (10, 220)))
        
        # 播放启动音频
        speech.play_audio(start_audio_path)
        
        # 等待接收第一帧图像后再启动处理线程
        self.get_logger().info('Waiting for first image...')
        threading.Thread(target=self.wait_and_start_process, daemon=True).start()
        
        # 创建服务
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def wait_and_start_process(self):
        """等待第一帧图像后启动处理线程"""
        # 等待最多10秒钟接收第一帧图像
        timeout = 10.0
        start_time = time.time()
        
        while not self.image_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if self.image_received:
            self.get_logger().info('First image received, starting display process...')
            threading.Thread(target=self.process, daemon=True).start()
        else:
            self.get_logger().error(f'No image received after {timeout} seconds!')
            self.get_logger().error('Please check if the camera topic is correct and publishing data.')

    def send_request(self, client, msg):
        """发送服务请求"""
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def vllm_result_callback(self, msg):
        """VLLM结果回调"""
        self.vllm_result = msg.data

    def play_audio_finish_callback(self, msg):
        """音频播放完成回调"""
        self.play_audio_finish = msg.data

    def process(self):
        """主处理循环 - 显示图像和处理VLLM结果"""
        # 创建OpenCV窗口
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image', display_size[0], display_size[1])
        
        while self.running:
            try:
                # 使用超时获取图像，避免永久阻塞
                image = self.image_queue.get(timeout=0.1)
                
                # 处理VLLM结果
                if self.vllm_result:
                    msg = String()
                    msg.data = self.vllm_result
                    self.tts_text_pub.publish(msg)
                    self.vllm_result = ''
                    self.action_finish = True
                
                # 处理音频播放完成
                if self.play_audio_finish and self.action_finish:
                    self.play_audio_finish = False
                    self.action_finish = False
                    msg = SetBool.Request()
                    msg.data = True
                    self.send_request(self.awake_client, msg)
                
                # 转换颜色空间并调整大小
                bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                display_image = cv2.resize(bgr_image, (display_size[0], display_size[1]))
                
                # 显示图像
                cv2.imshow('image', display_image)
                
                # 设置窗口位置（只执行一次）
                if not self.set_above:
                    cv2.moveWindow('image', 1920 - display_size[0], 0)
                    try:
                        os.system("wmctrl -r image -b add,above")
                    except Exception as e:
                        self.get_logger().warn(f'Failed to set window always on top: {e}')
                    self.set_above = True
                
            except queue.Empty:
                # 队列为空时继续循环
                pass
            except Exception as e:
                self.get_logger().error(f'Error in process loop: {e}')
            
            # 检查键盘输入
            k = cv2.waitKey(1)
            if k == 27 or k == ord('q'):  # ESC或Q键退出
                self.get_logger().info('User requested quit')
                break
        
        # 清理
        cv2.destroyAllWindows()
        self.running = False

    def image_callback(self, ros_image):
        """图像回调函数"""
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            
            # 记录第一次接收图像
            if not self.image_received:
                self.image_received = True
                self.first_image_time = time.time()
                self.get_logger().info(f'First image received! Size: {rgb_image.shape}')
            
            # 如果队列已满，丢弃最旧的图像
            if self.image_queue.full():
                self.image_queue.get()
            
            # 将图像放入队列
            self.image_queue.put(rgb_image)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')

def main():
    node = VLLMWithCamera('vllm_with_camera')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()