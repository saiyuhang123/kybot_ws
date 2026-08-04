#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import base64
import rospy
import threading
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
import cv2  # 先于 cv_bridge 导入
from cv_bridge import CvBridge
import tempfile
import json
import requests

# 使用 OpenCV 4 的兼容设置
if hasattr(cv2, 'cv2'):
    cv2 = cv2.cv2

# 定义状态机
class NodeState:
    STANDBY = 0
    CAPTURING = 1
    PROCESSING = 2

# 图像问答
class ImageDescriptionROSNode:
    def __init__(self):
        rospy.init_node('image_description_node', anonymous=True)

        # ROS参数配置
        self.model_name = rospy.get_param('~model', 'qwen-vl-max-latest')
        self.prompt = rospy.get_param('~prompt', '简短的说')
        self.image_save_dir = rospy.get_param('~image_save_dir', tempfile.gettempdir())
        self.debug_mode = rospy.get_param('~debug', True)

        # ========== [MOD-API] key / base_url 参数化，避免写死 ==========
        # self.api_key = rospy.get_param('~api_key', os.getenv('DASHSCOPE_API_KEY', ''))
        self.api_key = rospy.get_param('~api_key', "sk-d8349756a60e4f728d7a3e5f8fe9145c")
        self.base_url = rospy.get_param('~base_url', "https://dashscope.aliyuncs.com/compatible-mode/v1")
        if not self.api_key:
            rospy.logwarn("DASHSCOPE api_key is empty! Please set rosparam ~api_key or env DASHSCOPE_API_KEY.")

        # 确保保存目录存在
        if not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)

        self.bridge = CvBridge()

        # 状态管理变量
        self.state = NodeState.STANDBY
        self.state_lock = threading.Lock()

        # 图像缓冲区
        self.latest_frame = None
        self.frame_timestamp = None

        # 提示词锁
        self.prompt_lock = threading.Lock()

        # ROS话题和服务
        self.image_sub = rospy.Subscriber(
            '/camera/color/image_raw',
            Image,
            self.image_callback,
            queue_size=1
        )

        # ========== [MOD-TOPIC] 发布独立话题，避免和 qwen_point 混用 ==========
        self.description_pub = rospy.Publisher(
            '/vision_description',
            String,
            queue_size=1,
            latch=True
        )

        # ========== [MOD-TOPIC] 监听独立 prompt 话题，避免覆盖 qwen_point ==========
        self.prompt_sub = rospy.Subscriber(
            '/vision_prompt_update',
            String,
            self.prompt_update_callback,
            queue_size=1
        )

        # ========== [MOD-SRV] 独立触发服务，避免冲突 ==========
        self.trigger_service = rospy.Service(
            '/vision_trigger_capture',
            Trigger,
            self.handle_trigger
        )

        rospy.loginfo("Image Description Node initialized")
        rospy.loginfo("Using model: %s", self.model_name)
        rospy.loginfo("Initial prompt: %s", self.prompt)
        rospy.loginfo("Waiting for new prompts via /vision_prompt_update topic")
        rospy.loginfo("Trigger service: /vision_trigger_capture")
        rospy.loginfo("Publish topic: /vision_description")

    def prompt_update_callback(self, msg):
        """Handle new prompt messages"""
        with self.prompt_lock:
            try:
                prompt_json = json.loads(msg.data)
                if 'prompt' in prompt_json:
                    self.prompt = prompt_json['prompt']
                    rospy.loginfo("Received JSON prompt: %s", self.prompt)
                else:
                    self.prompt = msg.data
                    rospy.loginfo("Received new prompt: %s", self.prompt)
            except ValueError:
                self.prompt = msg.data
                rospy.loginfo("Received new prompt: %s", self.prompt)

    def image_callback(self, msg):
        """Handle image topic callback"""
        with self.state_lock:
            if self.state == NodeState.STANDBY:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    self.latest_frame = cv_image
                    self.frame_timestamp = rospy.Time.now()
                except Exception as e:
                    rospy.logwarn("Image buffer update failed: %s", e)

    # 它的核心职责是安全地获取系统当前最新的相机画面，
    # 并将耗时的视觉处理任务放到后台运行，从而在使用户的触发请求能够被立刻响应而不发生阻塞。
    def handle_trigger(self, req):
        """Service callback: handle capture request"""
        print("============运行到这里了============")
        with self.state_lock:
            if self.state != NodeState.STANDBY:
                return TriggerResponse(success=False, message="Previous request still processing")
            self.state = NodeState.CAPTURING

        rospy.sleep(0.05)  # Wait for next frame

        with self.state_lock:
            if self.latest_frame is None:
                self.state = NodeState.STANDBY
                return TriggerResponse(success=False, message="No image available")

            capture_frame = self.latest_frame
            self.state = NodeState.PROCESSING

        threading.Thread(
            target=self.process_captured_frame,
            args=(capture_frame,),
            daemon=True
        ).start()

        return TriggerResponse(success=True, message="Image captured and processing started")

    def process_captured_frame(self, frame):
        """Process captured frame 处理捕获帧"""
        filename = None
        try:
            timestamp = rospy.Time.now().to_nsec()
            filename = os.path.join(self.image_save_dir, f"vision_capture_{timestamp}.png")

            success = cv2.imwrite(filename, frame)
            if not success:
                rospy.logerr("Failed to save image to: %s", filename)
                return

            if self.debug_mode:
                rospy.loginfo("Image saved to: %s", filename)

            with self.prompt_lock:
                current_prompt = self.prompt

            rospy.loginfo("Using vision prompt: %s", current_prompt)
            description = self.describe_image(filename, current_prompt)

            if description:
                self.description_pub.publish(String(description))
                log_msg = description[:80] + "..." if len(description) > 80 else description
                rospy.loginfo("Vision description: %s", log_msg)

                if self.debug_mode:
                    txt_filename = os.path.splitext(filename)[0] + ".txt"
                    with open(txt_filename, 'w', encoding='utf-8') as f:
                        f.write(description)
                    rospy.loginfo("Description saved to: %s", txt_filename)

            # ========== [MOD-CLEAN] debug_mode=True 保留文件便于排障 ==========
            if not self.debug_mode and filename:
                try:
                    os.remove(filename)
                except OSError as e:
                    rospy.logwarn("Failed to remove temp file: %s - %s", filename, e)
            elif self.debug_mode and filename:
                rospy.loginfo("Debug mode on: keep image at %s", filename)

        except Exception as e:
            rospy.logerr("Processing error: %s", e)
        finally:
            with self.state_lock:
                self.state = NodeState.STANDBY

    def encode_image(self, image_path):
        """Encode image to Base64 string"""
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def describe_image(self, image_path, prompt_text):
        """Use vision-language model to describe image content"""
        if not os.path.exists(image_path):
            rospy.logerr("Image file does not exist: %s", image_path)
            return None

        if not self.api_key:
            rospy.logerr("No api_key provided. Set ~api_key or env DASHSCOPE_API_KEY.")
            return None

        try:
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}"
            }

            # 这里让 prompt 更明确一些：一句话、简洁、聚焦  请用一句话简洁描述只输出描述文本，不要输出多余解释。
            user_text = f"请用详细描述：{prompt_text}。描述所有你能看到的内容，尽可能全面和具体。只输出描述文本，不要输出多余解释。"

            payload = {
                "model": self.model_name,
                "messages": [
                    {
                        "role": "system",
                        "content": [{"type": "text", "text": "You are a helpful assistant."}],
                    },
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/png;base64,{self.encode_image(image_path)}"},
                            },
                            {"type": "text", "text": user_text},
                        ],
                    },
                ]
            }

            response = requests.post(
                f"{self.base_url}/chat/completions",
                headers=headers,
                json=payload,
                timeout=30.0
            )

            response.raise_for_status()
            result = response.json()
            return result['choices'][0]['message']['content']

        except requests.exceptions.RequestException as e:
            rospy.logerr("API request failed: %s", e)
        except KeyError as e:
            rospy.logerr("Missing expected key in API response: %s", e)
        except Exception as e:
            rospy.logerr("Error calling vision model: %s", e)

        return None

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ImageDescriptionROSNode()
        rospy.loginfo("Starting image description node...")
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown by ROS")
    except Exception as e:
        rospy.logfatal("Unhandled exception: %s", e)