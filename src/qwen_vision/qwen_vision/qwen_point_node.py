#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import re
import base64
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import rclpy.duration
import threading
import time
import numpy as np
import cv2
import json
import requests
import tempfile
import tf2_ros
import ast

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point


class NodeState:
    STANDBY = 0
    CAPTURING = 1
    PROCESSING = 2


class ObjectLocatorNode(Node):
    def __init__(self):
        super().__init__('object_locator_node')

        # ROS参数配置
        self.declare_parameter('model', 'qwen3.6-plus')
        self.declare_parameter('prompt', '请只定位目标物体的位置（只返回bbox）')
        self.declare_parameter('image_save_dir', tempfile.gettempdir())
        self.declare_parameter('debug', True)

        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.default_prompt = self.get_parameter('prompt').get_parameter_value().string_value
        self.image_save_dir = self.get_parameter('image_save_dir').get_parameter_value().string_value
        self.debug_mode = self.get_parameter('debug').get_parameter_value().bool_value

        # ===== [MOD] API key 参数化（不要写死在代码里）=====
        # self.declare_parameter('api_key', os.getenv('DASHSCOPE_API_KEY', ''))
        self.declare_parameter('api_key', "sk-87234c4787964295b2bc8687b1814656")
        self.declare_parameter('base_url', "https://dashscope.aliyuncs.com/compatible-mode/v1")
        self.api_key = self.get_parameter('api_key').get_parameter_value().string_value
        self.base_url = self.get_parameter('base_url').get_parameter_value().string_value
        if not self.api_key:
            self.get_logger().warn("DASHSCOPE api_key is empty! Set rosparam ~api_key or env DASHSCOPE_API_KEY.")

        # ===== [MOD] 同步服务等待超时（秒）=====
        self.declare_parameter('locate_timeout', 10.0)
        self.locate_timeout = float(self.get_parameter('locate_timeout').get_parameter_value().double_value)

        # ===== [MOD] 深度有效范围（米）=====
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 3.00)
        self.min_depth_m = float(self.get_parameter('min_depth_m').get_parameter_value().double_value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').get_parameter_value().double_value)

        if not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)

        self.bridge = CvBridge()

        # 状态管理
        self.state = NodeState.STANDBY
        self.state_lock = threading.Lock()

        # 图像缓存
        self.color_frame = None
        self.depth_frame = None
        self.depth_info = None
        self.frame_timestamp = rclpy.time.Time()

        # 源坐标系（从消息header拿）
        self.color_frame_id = None
        self.depth_frame_id = None
        self.camera_info_frame_id = None

        # TF2初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ===== [MOD] 目标物管理 =====
        self.target_lock = threading.Lock()
        self.declare_parameter('target', '')
        self.target_name = self.get_parameter('target').get_parameter_value().string_value  # 例如 "water bottle"

        # 可选：把口语化/同义词统一成你想要的"目标名称"
        self.synonyms = {
            "水": "water bottle",
            "水瓶": "water bottle",
            "矿泉水": "water bottle",
            "瓶子": "bottle",
            "苹果": "apple",
        }
        # 目标名别名（用于模型输出不一致时匹配）
        self.alias_map = {
            "water bottle": {"water bottle", "bottle"},
            "bottle": {"bottle", "water bottle"},
            "apple": {"apple"},
            "orange": {"orange"},
            "pear": {"pear"},
            "peach": {"peach"},
        }


        # ===== [MOD] 同步服务结果缓存 + 事件 =====
        self.done_event = threading.Event()
        self.last_result_lock = threading.Lock()
        self.last_success = False
        self.last_message = ""
        self.last_pose = None   # PoseStamped（map 或 camera frame）
        self.last_name = ""

        # ROS接口
        self.init_ros_interfaces()

        self.prompt_lock = threading.Lock()

        self.get_logger().info("Object Locator Node initialized")
        self.get_logger().info(f"Using model: {self.model_name}")
        self.get_logger().info("Service: /locate_object (async), /locate_object_sync (sync)")
        self.get_logger().info("Topic input: /voice_text, /target_object")
        self.get_logger().info("Topic output: /object_position, /object_name, /image_description")

    def init_ros_interfaces(self):
        """设置所有ROS话题、服务及订阅"""
        # QoS profiles
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # 图像订阅
        # 接收相机的 RGB 彩色图像。
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, sensor_qos)
        # 接收对齐后的深度图像（Depth Image）
        self.depth_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, sensor_qos)

        # 相机内参订阅
        self.info_sub = self.create_subscription(CameraInfo, '/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 1)

        # 语音文本 接收外部语音识别模块发来的文字信息
        self.voice_sub = self.create_subscription(String, '/voice_text', self.voice_text_callback, 1)

        # ===== [MOD] 上位机可直接发布目标物名 =====
        self.target_sub = self.create_subscription(String, '/target_object', self.target_object_callback, 1)

        # 结果发布（latch=True：上层随时能拿到最新一次结果）
        self.visualization_pub = self.create_publisher(Image, '/visualization', 1)
        self.position_pub = self.create_publisher(PoseStamped, '/object_position', latch_qos)
        self.object_name_pub = self.create_publisher(String, '/object_name', latch_qos)
        self.description_pub = self.create_publisher(String, '/image_description', latch_qos)

        # 提示词更新（如果你还需要外部覆盖 prompt，可以保留）
        self.prompt_sub = self.create_subscription(String, '/prompt_update', self.prompt_update_callback, 1)

        # 定位服务（异步）
        self.trigger_service = self.create_service(Trigger, '/locate_object', self.handle_locate_request)
        # ===== [MOD] 定位服务（同步，推荐 robot_aiui.cpp 用这个）=====
        self.sync_service = self.create_service(Trigger, '/locate_object_sync', self.handle_locate_request_sync)

        # 交互式定位服务（保留）
        self.interactive_service = self.create_service(Trigger, '/interactive_locate', self.handle_interactive_request)

    # ===== [MOD] 外部设置目标物 =====
    def target_object_callback(self, msg: String):
        name = (msg.data or "").strip()
        if not name:
            return
        with self.target_lock:
            self.target_name = name
        self.get_logger().info(f"Target object updated: {self.target_name}")

    def voice_text_callback(self, msg):
        """
        /voice_text 示例：'帮我去办公室拿一瓶水' / '帮我抓一个水瓶'
        这里仅负责提取目标物名并更新 target_name + prompt
        """
        text = (msg.data or "").strip()
        if not text:
            return

        target = self.extract_target_object(text)
        if not target:
            self.get_logger().warn(f"Voice text received but target not recognized: {text}")
            return

        target_norm = self.synonyms.get(target, target)

        with self.target_lock:
            self.target_name = target_norm

        # 让 prompt 更明确：只找目标，不要泛化
        new_prompt = f"请只定位 {target_norm} 的位置（只返回它的bbox）"
        with self.prompt_lock:
            self.default_prompt = new_prompt

        self.get_logger().info(f"Target from voice: {self.target_name} | Prompt: {self.default_prompt}")

    def extract_target_object(self, text: str):
        """从中文口语里提取目标物体名"""
        t = re.sub(r"[，。！？,.!?；;:\s]", "", text)

        patterns = [
            r"(?:帮我)?(?:抓|拿|取|找|定位|识别)(?:一下|一个|一只|一瓶|一支|一块|一颗)?(?P<obj>[\u4e00-\u9fa5A-Za-z0-9_]+)",
            r"(?P<obj>[\u4e00-\u9fa5A-Za-z0-9_]+)(?:在哪里|的位置|在哪儿)"
        ]

        for p in patterns:
            m = re.search(p, t)
            if m:
                obj = m.group("obj")
                obj = re.sub(r"(一下|吧|呀|呢|哈)$", "", obj)
                obj = re.sub(r"^(一个|一只|一瓶|一支|一块|一颗)", "", obj)
                return obj
        return None

    def camera_info_callback(self, msg):
        self.depth_info = msg
        self.camera_info_frame_id = msg.header.frame_id

    def color_callback(self, msg):
        with self.state_lock:
            if self.state in (NodeState.STANDBY, NodeState.CAPTURING):
                try:
                    self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    self.frame_timestamp = msg.header.stamp
                    self.color_frame_id = msg.header.frame_id
                except Exception as e:
                    self.get_logger().warn(f"Color image update failed: {e}")

    def depth_callback(self, msg):
        with self.state_lock:
            if self.state in (NodeState.STANDBY, NodeState.CAPTURING):
                try:
                    self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    self.depth_frame_id = msg.header.frame_id
                except Exception as e:
                    self.get_logger().warn(f"Depth image update failed: {e}")


    def prompt_update_callback(self, msg):
        """外部更新 prompt（保留）"""
        try:
            prompt_json = json.loads(msg.data)
            if 'prompt' in prompt_json:
                self.default_prompt = prompt_json['prompt']
                self.get_logger().info(f"Prompt updated: {self.default_prompt}")
            else:
                self.get_logger().warn("Invalid prompt format: missing 'prompt' key")
        except ValueError:
            self.default_prompt = msg.data
            self.get_logger().info(f"Prompt updated (plain text): {self.default_prompt}")

    def fail_publish(self, log_detail: str, msg_cn: str = "目标检测失败"):
        """
        失败统一出口：
        - 终端日志保留详细原因（log_detail）
        - /image_description 只发中文短句（msg_cn）
        - 同步服务 last_message 也只保留中文短句
        """
        self.get_logger().warn(log_detail)
        self.description_pub.publish(String(data=msg_cn))
        with self.last_result_lock:
            self.last_success = False
            self.last_message = msg_cn
            self.last_pose = None
            self.last_name = ""

    def reset_last_result(self):
        with self.last_result_lock:
            self.last_success = False
            self.last_message = ""
            self.last_pose = None
            self.last_name = ""

    def handle_locate_request(self, request, response):
        """异步定位：返回 started；结果通过 topic 输出"""
        with self.state_lock:
            if self.state != NodeState.STANDBY:
                response.success = False
                response.message = "Previous request is still processing"
                return response
            self.state = NodeState.CAPTURING

        self.done_event.clear()
        self.reset_last_result()

        time.sleep(0.1)

        with self.state_lock:
            if self.color_frame is None or self.depth_frame is None:
                self.state = NodeState.STANDBY
                response.success = False
                response.message = "Camera data not available"
                return response

            color_image = self.color_frame
            depth_image = self.depth_frame
            timestamp = self.frame_timestamp

            if self.depth_info:
                camera_info = self.depth_info
            else:
                self.get_logger().warn("Camera info not available, using default intrinsics")
                camera_info = CameraInfo()
                camera_info.k = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
                camera_info.width = 640
                camera_info.height = 480

            with self.prompt_lock:
                prompt = self.default_prompt

            with self.target_lock:
                target = self.target_name

            self.state = NodeState.PROCESSING

        threading.Thread(
            target=self.process_detection,
            args=(color_image, depth_image, camera_info, timestamp, prompt, target),
            daemon=True
        ).start()

        response.success = True
        response.message = "Object localization started (async)"
        return response

    def handle_locate_request_sync(self, request, response):
        """同步定位：阻塞等待结果，返回 JSON（成功）或失败原因（失败）"""
        response = self.handle_locate_request(request, response)
        if not response.success:
            return response

        if not self.done_event.wait(self.locate_timeout):
            self.get_logger().warn(f"Timeout: no result within {self.locate_timeout:.1f}s")
            response.success = False
            response.message = "目标检测失败"
            return response

        with self.last_result_lock:
            if not self.last_success or self.last_pose is None:
                response.success = False
                response.message = self.last_message or "Locate failed"
                return response

            p = self.last_pose.pose.position
            msg = json.dumps({
                "name": self.last_name,
                "frame_id": self.last_pose.header.frame_id,
                "x": float(p.x),
                "y": float(p.y),
                "z": float(p.z)
            }, ensure_ascii=False)

            response.success = True
            response.message = msg
            return response

    def handle_interactive_request(self, request, response):
        self.get_logger().info("Interactive mode: Locating objects with current target/prompt")
        return self.handle_locate_request(request, response)

    def get_source_frame(self):
        if self.camera_info_frame_id:
            return self.camera_info_frame_id
        if self.depth_frame_id:
            return self.depth_frame_id
        if self.color_frame_id:
            return self.color_frame_id
        return "camera_depth_optical_frame"

    def encode_image(self, path):
        with open(path, "rb") as img_file:
            return base64.b64encode(img_file.read()).decode('utf-8')

    # ===== [MOD] 更强的 JSON 提取 =====
    def extract_json_obj(self, content: str):
        if not content:
            return None

        # 1) 优先提取 ```json ... ``` 或 ``` ... ```（兼容数组和对象）
        m = re.search(r"```(?:json)?\s*([\[\{].*?[\]\}])\s*```", content, flags=re.S)
        if m:
            content = m.group(1)
        else:
            # 2) 退化：提取第一个 [ 或 { 到最后一个 ] 或 } 之间的内容
            # 优先按数组提取，避免切掉外层的 []
            arr_start = content.find("[")
            arr_end = content.rfind("]")
            if arr_start != -1 and arr_end != -1 and arr_start < arr_end:
                content = content[arr_start:arr_end + 1]
            elif "{" in content and "}" in content:
                content = content[content.find("{"): content.rfind("}") + 1]

        # 3) json.loads
        try:
            obj = json.loads(content)
            # 模型可能返回顶层数组 [{...}, ...]，自动包装为 {"objects": [...]}
            if isinstance(obj, list):
                return {"objects": obj}
            if isinstance(obj, dict):
                return obj
        except Exception:
            pass

        # 4) ast.literal_eval 兜底（应对单引号 dict / list）
        try:
            obj = ast.literal_eval(content)
            if isinstance(obj, list):
                return {"objects": obj}
            if isinstance(obj, dict):
                return obj
        except Exception:
            return None
        return None

    # ===== [MOD] 只让模型返回"目标物" =====
    def detect_objects_with_vision_model(self, image_path, prompt, target_name: str):
        if not self.api_key:
            self.get_logger().error("No api_key provided. Set ~api_key or env DASHSCOPE_API_KEY.")
            return None

        try:
            base64_image = self.encode_image(image_path)

            target_name = (target_name or "").strip()
            if target_name:
                detailed_prompt = (
                    f"请只定位图像中的目标物：{target_name}。"
                    f"如果有多个 {target_name}，请把每一个实例都返回bbox。"
                    f"如果没有找到 {target_name}，请返回空数组。"
                    "只输出严格JSON，不要输出任何解释/Markdown。"
                    "JSON格式如下："
                    "{\"objects\":[{\"name\":\"TARGET\",\"bbox\":[x_min,y_min,x_max,y_max],\"confidence\":0.0}]} "
                    "或：{\"objects\":[]}"
                )

            else:
                # 没指定目标时才用默认 prompt（不推荐用于抓取）
                detailed_prompt = (
                    f"{prompt}。只输出严格JSON，不要输出任何解释/Markdown。"
                    "{\"objects\":[{\"name\":\"object\",\"bbox\":[x_min,y_min,x_max,y_max],\"confidence\":0.0}]}"
                )

            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}"
            }

            payload = {
                "model": self.model_name,
                "messages": [
                    {
                        "role": "system",
                        "content": [{"type": "text", "text": "你是机器人视觉感知系统，需要严格输出可解析JSON。"}],
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{base64_image}"}} ,
                            {"type": "text", "text": detailed_prompt}
                        ],
                    },
                ]
            }

            response = requests.post(
                f"{self.base_url}/chat/completions",
                headers=headers,
                json=payload,
                timeout=15.0
            )
            response.raise_for_status()
            result = response.json()
            content = result['choices'][0]['message']['content']

            obj_data = self.extract_json_obj(content)
            if not obj_data or 'objects' not in obj_data:
                self.get_logger().error(f"Failed to parse JSON from model. Raw: {content}")
                return None

            return obj_data

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Vision model API request failed: {str(e)}")
            return None
        except Exception as e:
            self.get_logger().error(f"detect_objects_with_vision_model error: {str(e)}")
            return None

    # ===== [MOD] 深度单位兼容 =====
    def depth_to_m(self, depth_value, depth_img):
        if depth_value is None:
            return None
        try:
            v = float(depth_value)
        except Exception:
            return None

        # 常见：uint16(mm)
        if depth_img.dtype == np.uint16 or v > 20.0:
            return v / 1000.0
        # 常见：float32(m)
        return v

    def calculate_3d_positions(self, color_img, depth_img, camera_info, detected_objects):
        object_positions = []
        height, width = color_img.shape[:2]

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        for obj in detected_objects:
            bbox = obj.get('bbox', None)
            if not bbox or len(bbox) != 4:
                continue

            x_min, y_min, x_max, y_max = map(int, bbox)
            x_min = max(0, min(x_min, width - 1))
            y_min = max(0, min(y_min, height - 1))
            x_max = max(0, min(x_max, width - 1))
            y_max = max(0, min(y_max, height - 1))

            center_x = (x_min + x_max) // 2
            center_y = (y_min + y_max) // 2

            depth_value = depth_img[center_y, center_x]

            # 中心深度无效 -> bbox 内取中位数
            if (isinstance(depth_value, (np.integer, int, np.uint16)) and depth_value == 0) or (isinstance(depth_value, float) and (np.isnan(depth_value) or depth_value <= 0)):
                roi = depth_img[max(0, y_min):min(height, y_max + 1), max(0, x_min):min(width, x_max + 1)]
                roi = roi.astype(np.float32)
                roi = roi[np.isfinite(roi)]
                roi = roi[roi > 0]
                if roi.size > 0:
                    depth_value = np.median(roi)
                else:
                    depth_value = 0

            depth_m = self.depth_to_m(depth_value, depth_img)
            if depth_m is None or depth_m <= 0:
                continue

            # 深度范围过滤（避免远处误检）
            if depth_m < self.min_depth_m or depth_m > self.max_depth_m:
                continue

            z = depth_m
            x = (center_x - cx) * z / fx
            y = (center_y - cy) * z / fy

            object_positions.append({
                "name": obj.get("name", "unknown"),
                "confidence": float(obj.get("confidence", 0.0)),
                "bbox": [x_min, y_min, x_max, y_max],
                "camera_position": (float(x), float(y), float(z))
            })

        return object_positions

    # ===== [MOD] 目标选择：只选一个最佳 =====
    def get_target_aliases(self, target: str):
        t = (target or "").strip().lower()
        if not t:
            return set()
        aliases = set([t])
        if t in self.alias_map:
            aliases |= set([a.lower() for a in self.alias_map[t]])
        return aliases

    def select_best_target(self, object_positions, target_name: str):
        if not object_positions:
            return None
        # 排序策略：先置信度，再距离近（z 小）
        def dist(o):
            x, y, z = o["camera_position"]
            return float(np.sqrt(x*x + y*y + z*z))

        target_name = (target_name or "").strip()
        if target_name:
            aliases = self.get_target_aliases(target_name)

            candidates = []
            for o in object_positions:
                n = (o.get("name", "") or "").strip().lower()
                # 宽松匹配：别名命中 or 子串命中
                if (n in aliases) or any(a in n or n in a for a in aliases):
                    candidates.append(o)

            if not candidates:
                return None
            # 就近优先，再按置信度（更符合"就近原则"）
            candidates.sort(key=lambda o: (dist(o), -o.get("confidence", 0.0)))
            return candidates[0]

        # 没指定目标：选最高置信度且最近
        object_positions.sort(key=lambda o: (dist(o), -o.get("confidence", 0.0)))
        return object_positions[0]

    def process_results_publish_one(self, selected_obj, timestamp):
        """只发布一个目标物的 PoseStamped（尽量转到 map）"""
        source_frame = self.get_source_frame()

        # 先做相机系 pose
        x, y, z = selected_obj["camera_position"]
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = source_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # 尝试 TF 到 map
        pose_out = pose_msg
        try:
            tf_time = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform("map", source_frame, tf_time, rclpy.duration.Duration(seconds=1.0))

            point_original = PointStamped()
            point_original.header.stamp = tf_time.to_msg()
            point_original.header.frame_id = source_frame
            point_original.point.x = x
            point_original.point.y = y
            point_original.point.z = z

            point_transformed = do_transform_point(point_original, transform)

            pose_out = PoseStamped()
            pose_out.header.stamp = self.get_clock().now().to_msg()
            pose_out.header.frame_id = "map"
            pose_out.pose.position = point_transformed.point

        except Exception as e:
            self.get_logger().warn(f"TF to map failed, keep camera frame. err={str(e)}")

        # 发布
        self.position_pub.publish(pose_out)
        self.object_name_pub.publish(String(data=selected_obj.get("name", "unknown")))

        # 缓存同步结果
        with self.last_result_lock:
            self.last_success = True
            self.last_name = selected_obj.get("name", "unknown")
            self.last_pose = pose_out
            self.last_message = "OK"

    def visualize_detections(self, color_img, object_positions, selected=None):
        vis_img = color_img.copy()
        for obj in object_positions:
            x_min, y_min, x_max, y_max = obj["bbox"]
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)

            cv2.rectangle(vis_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            label = obj["name"]
            cam_pos = obj.get("camera_position")
            if cam_pos is not None:
                label += f" {cam_pos[2]:.2f}m"

            cv2.putText(vis_img, label, (x_min, max(0, y_min - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if selected is not None:
            x_min, y_min, x_max, y_max = selected["bbox"]
            cv2.rectangle(vis_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 0, 255), 3)

        return vis_img

    def publish_visualization(self, vis_img, timestamp):
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header.stamp = timestamp
            self.visualization_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish visualization: {str(e)}")

    def generate_description(self, selected_obj, target_name: str):
        if not selected_obj:
            if target_name:
                return f"未找到目标：{target_name}，或深度无效/超出范围"
            return "未找到目标物，或深度无效/超出范围"

        x, y, z = selected_obj["camera_position"]
        dist = float(np.sqrt(x*x + y*y + z*z))
        return f"检测到 {selected_obj.get('name','unknown')}，距离约 {dist:.2f} 米"

    def process_detection(self, color_img, depth_img, camera_info, timestamp, prompt, target_name):
        temp_img_path = None
        try:
            # timestamp here is a builtin_interfaces.msg.Time from the frame header
            stamp_ns = timestamp.sec * 10**9 + timestamp.nanosec
            temp_img_path = os.path.join(self.image_save_dir, f"capture_{stamp_ns}.png")
            cv2.imwrite(temp_img_path, color_img)

            # 1) 模型检测（只找目标）
            result = self.detect_objects_with_vision_model(temp_img_path, prompt, target_name)
            if not result or 'objects' not in result:
                self.fail_publish("Vision model returned invalid result")
                return

            objects = result.get("objects", [])
            if not objects:
                self.fail_publish(f"Model found no objects for target={target_name}")
                return

            # 2) 计算3D
            object_positions = self.calculate_3d_positions(color_img, depth_img, camera_info, objects)
            if not object_positions:
                self.fail_publish(f"Objects detected but no valid depth for target={target_name}")
                return

            # 3) 选择最佳目标
            selected = self.select_best_target(object_positions, target_name)
            if not selected:
                self.fail_publish(f"Target not matched after depth filter: target={target_name}")
                return

            # 4) 只发布一个目标物 pose
            self.process_results_publish_one(selected, timestamp)

            # 5) 发布描述
            desc = self.generate_description(selected, target_name)
            self.description_pub.publish(String(data=desc))

            # 6) 可视化（可选）
            vis_img = self.visualize_detections(color_img, object_positions, selected=selected)
            self.publish_visualization(vis_img, timestamp)
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}")
            self.fail_publish(f"Processing error: {str(e)}")  # 对外只说"目标检测失败"
        finally:
            # 清理临时文件（debug=true 保留便于排障）
            if temp_img_path and (not self.debug_mode):
                try:
                    os.remove(temp_img_path)
                except Exception:
                    pass

            with self.state_lock:
                self.state = NodeState.STANDBY

            # ===== [MOD] 通知同步服务：结果已结束（成功或失败）=====
            self.done_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocatorNode()
    try:
        node.get_logger().info("Starting object locator node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node shutdown by keyboard interrupt")
    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
