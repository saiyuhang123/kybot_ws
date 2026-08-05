#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import base64
import threading
import tempfile
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import requests


class NodeState:
    STANDBY = 0
    CAPTURING = 1
    PROCESSING = 2


class ImageDescriptionNode(Node):
    def __init__(self):
        super().__init__('image_description_node')

        # ROS参数配置
        self.declare_parameter('model', 'qwen3.7-plus')
        self.declare_parameter('prompt', '简短的说')
        self.declare_parameter('image_save_dir', tempfile.gettempdir())
        self.declare_parameter('debug', True)
        self.declare_parameter('api_key', os.getenv('DASHSCOPE_API_KEY', ''))
        self.declare_parameter('base_url', os.getenv('DASHSCOPE_BASE_URL', ''))

        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.prompt = self.get_parameter('prompt').get_parameter_value().string_value
        self.image_save_dir = self.get_parameter('image_save_dir').get_parameter_value().string_value
        self.debug_mode = self.get_parameter('debug').get_parameter_value().bool_value
        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        self.base_url = self.get_parameter('base_url').get_parameter_value().string_value

        if not self.api_key:
            self.get_logger().warn('DASHSCOPE api_key is empty!')
        if not self.base_url:
            self.get_logger().warn('DASHSCOPE base_url is empty!')

        if not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)

        self.bridge = CvBridge()

        # 状态管理
        self.state = NodeState.STANDBY
        self.state_lock = threading.Lock()

        # 图像缓冲区
        self.latest_frame = None
        self.frame_timestamp = None

        # 提示词锁
        self.prompt_lock = threading.Lock()

        # QoS: best effort for camera, reliable for services
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ROS话题和服务
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, sensor_qos)

        self.description_pub = self.create_publisher(
            String, '/vision_description', latch_qos)

        self.prompt_sub = self.create_subscription(
            String, '/vision_prompt_update', self.prompt_callback, 1)

        self.trigger_service = self.create_service(
            Trigger, '/vision_trigger_capture', self.handle_trigger)

        self.get_logger().info('Image Description Node initialized')
        self.get_logger().info(f'Using model: {self.model_name}')
        self.get_logger().info(f'Initial prompt: {self.prompt}')

    def prompt_callback(self, msg):
        with self.prompt_lock:
            try:
                prompt_json = json.loads(msg.data)
                if 'prompt' in prompt_json:
                    self.prompt = prompt_json['prompt']
                else:
                    self.prompt = msg.data
            except ValueError:
                self.prompt = msg.data
            self.get_logger().info(f'Received new prompt: {self.prompt}')

    def image_callback(self, msg):
        with self.state_lock:
            if self.state == NodeState.STANDBY:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                    self.latest_frame = cv_image
                    self.frame_timestamp = self.get_clock().now()
                except Exception as e:
                    self.get_logger().warn(f'Image buffer update failed: {e}')

    def handle_trigger(self, request, response):
        with self.state_lock:
            if self.state != NodeState.STANDBY:
                response.success = False
                response.message = 'Previous request still processing'
                return response
            self.state = NodeState.CAPTURING

        time.sleep(0.05)

        with self.state_lock:
            if self.latest_frame is None:
                self.state = NodeState.STANDBY
                response.success = False
                response.message = 'No image available'
                return response

            capture_frame = self.latest_frame
            self.state = NodeState.PROCESSING

        threading.Thread(
            target=self.process_captured_frame,
            args=(capture_frame,),
            daemon=True
        ).start()

        response.success = True
        response.message = 'Image captured and processing started'
        return response

    def process_captured_frame(self, frame):
        filename = None
        try:
            timestamp = self.get_clock().now().nanoseconds
            filename = os.path.join(self.image_save_dir, f'vision_capture_{timestamp}.png')

            success = cv2.imwrite(filename, frame)
            if not success:
                self.get_logger().error(f'Failed to save image to: {filename}')
                return

            if self.debug_mode:
                self.get_logger().info(f'Image saved to: {filename}')

            with self.prompt_lock:
                current_prompt = self.prompt

            self.get_logger().info(f'Using vision prompt: {current_prompt}')
            description = self.describe_image(filename, current_prompt)

            if description:
                msg = String()
                msg.data = description
                self.description_pub.publish(msg)
                log_msg = description[:80] + '...' if len(description) > 80 else description
                self.get_logger().info(f'Vision description: {log_msg}')

                if self.debug_mode:
                    txt_filename = os.path.splitext(filename)[0] + '.txt'
                    with open(txt_filename, 'w', encoding='utf-8') as f:
                        f.write(description)

            if not self.debug_mode and filename:
                try:
                    os.remove(filename)
                except OSError as e:
                    self.get_logger().warn(f'Failed to remove temp file: {filename} - {e}')
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
        finally:
            with self.state_lock:
                self.state = NodeState.STANDBY

    def encode_image(self, image_path):
        with open(image_path, 'rb') as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def describe_image(self, image_path, prompt_text):
        if not os.path.exists(image_path):
            self.get_logger().error(f'Image file does not exist: {image_path}')
            return None

        if not self.api_key:
            self.get_logger().error('No api_key provided.')
            return None

        try:
            headers = {
                'Content-Type': 'application/json',
                'Authorization': f'Bearer {self.api_key}'
            }

            user_text = f'请用详细描述：{prompt_text}。描述所有你能看到的内容，尽可能全面和具体。只输出描述文本，不要输出多余解释。'

            payload = {
                'model': self.model_name,
                'messages': [
                    {
                        'role': 'system',
                        'content': [{'type': 'text', 'text': 'You are a helpful assistant.'}],
                    },
                    {
                        'role': 'user',
                        'content': [
                            {
                                'type': 'image_url',
                                'image_url': {'url': f'data:image/png;base64,{self.encode_image(image_path)}'},
                            },
                            {'type': 'text', 'text': user_text},
                        ],
                    },
                ]
            }

            response = requests.post(
                f'{self.base_url}/chat/completions',
                headers=headers,
                json=payload,
                timeout=30.0
            )

            response.raise_for_status()
            result = response.json()
            return result['choices'][0]['message']['content']

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'API request failed: {e}')
        except KeyError as e:
            self.get_logger().error(f'Missing expected key in API response: {e}')
        except Exception as e:
            self.get_logger().error(f'Error calling vision model: {e}')

        return None


def main(args=None):
    rclpy.init(args=args)
    node = ImageDescriptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
