#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import base64
import rospy
import threading
import numpy as np
import cv2
import json
import requests
import tempfile
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point

class NodeState:
    STANDBY = 0
    CAPTURING = 1
    PROCESSING = 2

class ObjectLocatorNode:
    def __init__(self):

        # 在ROS1中初始化全局节点
        # 在一个 Python 进程中，如果你调用了多次 rospy.init_node()，
        # 后面的调用会直接失败/报错。在单一的 Python 代码（进程）里，rospy 不允许你创建多个不同的 Node。
        rospy.init_node('object_locator_node', anonymous=True)
        
        # ROS参数配置
        #  第二个参数（如 'qwen-vl-max-latest' 或 True）作为强大的“兜底策略”存在。
        #  如果外部（如 roslaunch 文件或命令行）压根没有配置过这些参数，代码就会直接采用这些默认值
        self.model_name = rospy.get_param('~model', 'qwen-vl-max-latest')
        self.default_prompt = rospy.get_param('~prompt', '简洁描述图中的物体并标记位置')
        self.image_save_dir = rospy.get_param('~image_save_dir', tempfile.gettempdir())
        self.debug_mode = rospy.get_param('~debug', True)
        
        if not os.path.exists(self.image_save_dir):
            os.makedirs(self.image_save_dir)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 状态管理
        self.state = NodeState.STANDBY
        self.state_lock = threading.Lock()
        
        # 图像缓存
        self.color_frame = None
        self.depth_frame = None
        self.depth_info = None
        self.frame_timestamp = rospy.Time(0)

        # 源坐标系（从消息header拿）
        self.color_frame_id = None
        self.depth_frame_id = None
        self.camera_info_frame_id = None
        
        # 目标物位置缓存
        self.detected_objects = []
        
        # TF2初始化
        # 实例化了一个空间坐标数据缓冲区。它的核心作用是按时间线“记住”并拼接收集到的坐标变换树（TF Tree）
        # 参数 cache_time=rospy.Duration(10) 指定了系统在内存中保留最近 10 秒钟的历史位姿数据
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))

        # 自动订阅 ROS 内部特殊的 /tf 和 /tf_static 话题，并将网络中其他设备广播过来的、
        # 最新的空间转换矩阵源源不断地泵入（Pump）到前端的 tf_buffer 之中
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # ROS接口
        self.init_ros_interfaces()
        
        rospy.loginfo("Object Locator Node initialized")
        rospy.loginfo(f"Using model: {self.model_name}")
        rospy.loginfo("Waiting for object detection requests...")
        rospy.loginfo("System will automatically trigger when '位置' keyword is detected in prompt")

    def init_ros_interfaces(self):
        """设置所有ROS话题、服务及订阅"""
        # 图像订阅
        self.color_sub = rospy.Subscriber(
            '/camera/color/image_raw', 
            Image, 
            self.color_callback,
            queue_size=1
        )
        
        self.depth_sub = rospy.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', 
            Image, 
            self.depth_callback,
            queue_size=1
        )
        
        # 相机内参订阅
        self.info_sub = rospy.Subscriber(
            '/camera/aligned_depth_to_color/camera_info',
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )
        
        # 结果发布
        self.visualization_pub = rospy.Publisher('/visualization', Image, queue_size=1)
        self.position_pub = rospy.Publisher('/object_position', PoseStamped, queue_size=1)
        self.description_pub = rospy.Publisher('/image_description', String, queue_size=1)
        
        # 提示词更新
        self.prompt_sub = rospy.Subscriber(
            '/prompt_update', 
            String, 
            self.prompt_update_callback,
            queue_size=1
        )
        
        # 定位服务
        self.trigger_service = rospy.Service(
            '/locate_object', 
            Trigger, 
            self.handle_locate_request
        )
        
        # 交互式定位服务
        self.interactive_service = rospy.Service(
            '/interactive_locate',
            Trigger,
            self.handle_interactive_request
        )

    def camera_info_callback(self, msg):
        """处理相机内参信息"""
        self.depth_info = msg
        self.camera_info_frame_id = msg.header.frame_id
        
    def color_callback(self, msg):
        """彩色图像回调"""
        with self.state_lock:
            if self.state == NodeState.STANDBY:
                try:
                    self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    self.frame_timestamp = msg.header.stamp
                    self.color_frame_id = msg.header.frame_id 
                except Exception as e:
                    rospy.logwarn("Color image update failed: %s", e)
    
    def depth_callback(self, msg):
        """深度图像回调"""
        with self.state_lock:
            if self.state == NodeState.STANDBY:
                try:
                    self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    self.depth_frame_id = msg.header.frame_id 
                except Exception as e:
                    rospy.logwarn("Depth image update failed: %s", e)
    
    def prompt_update_callback(self, msg):
        """更新提示词的回调，并检查关键词"""
        try:
            prompt_json = json.loads(msg.data)
            if 'prompt' in prompt_json:
                self.default_prompt = prompt_json['prompt']
                rospy.loginfo("Prompt updated: %s", self.default_prompt)
            else:
                rospy.logwarn("Invalid prompt format: missing 'prompt' key")
                return
        except ValueError:
            rospy.logwarn("Received non-JSON prompt, using as text: %s", msg.data)
            self.default_prompt = msg.data
        
        # 检查提示词是否包含触发关键词"位置"
        self.check_prompt_for_keywords()
    
    # 检查输入的提示词中是否包含“位置”二字，如果有，则自动发起一次物体定位请求，并处理该请求的完整流程。
    def check_prompt_for_keywords(self):
        """检查提示词是否包含触发关键词"""
        if '位置' in self.default_prompt:
            self.auto_trigger_detection()
    
    def auto_trigger_detection(self):
        """自动触发物体定位"""
        rospy.loginfo("Keyword '位置' detected. Triggering object localization automatically.")
        # 创建虚拟请求对象
        # Trigger 是一种标准的 ROS 服务类型，通常不需要传递具体参数，只用于发送“开始执行”的信号
        req = Trigger._request_class()
        return self.handle_locate_request(req)
    
    def handle_locate_request(self, req):
        """处理定位对象的服务请求"""
        with self.state_lock: #获取了线程的互斥锁，确保在检查和修改状态时不会发生竞态条件
            if self.state != NodeState.STANDBY:
                return TriggerResponse(
                    success=False,
                    message="Previous request is still processing"
                )
            self.state = NodeState.CAPTURING
        
        rospy.sleep(0.1)  # 等待下一帧数据
        
        with self.state_lock:
            if self.color_frame is None or self.depth_frame is None:
                self.state = NodeState.STANDBY
                return TriggerResponse(
                    success=False,
                    message="Camera data not available"
                )
            
            color_image = self.color_frame
            depth_image = self.depth_frame
            timestamp = self.frame_timestamp
            
            # 只有在有深度信息时才存储
            if self.depth_info:
                camera_info = self.depth_info
            else:
                rospy.logwarn("Camera info not available, using default intrinsics")
                # 创建默认相机内参 (640x480)
                camera_info = CameraInfo()
                camera_info.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
                camera_info.width = 640
                camera_info.height = 480
            
            self.state = NodeState.PROCESSING
        
        # 启动处理线程
        threading.Thread(
            target=self.process_detection, 
            args=(color_image, depth_image, camera_info, timestamp, self.default_prompt),
            daemon=True
        ).start()
        
        return TriggerResponse(
            success=True,
            message="Object localization started"
        )
    
    def handle_interactive_request(self, req):
        """交互式定位请求"""
        # 在实际应用中，这里可以连接语音识别系统
        # 简化版：使用默认提示词触发定位
        rospy.loginfo("Interactive mode: Locating objects with default prompt")
        return self.handle_locate_request(req)
    
    def process_detection(self, color_img, depth_img, camera_info, timestamp, prompt):
        """处理物体检测和定位"""
        try:
            # 临时文件处理
            temp_img_path = os.path.join(self.image_save_dir, f"capture_{timestamp.to_nsec()}.png")
            cv2.imwrite(temp_img_path, color_img)
            
            # 使用大模型检测物体
            result = self.detect_objects_with_vision_model(temp_img_path, prompt)
            
            if not result or 'objects' not in result or not result['objects']:
                rospy.logwarn("No objects detected by vision model")
                return
            
            self.detected_objects = result['objects']
            
            # 计算3D位置
            object_positions = self.calculate_3d_positions(
                color_img, 
                depth_img, 
                camera_info, 
                self.detected_objects
            )
            
            # 处理结果
            self.process_results(color_img, object_positions, timestamp)
            
            # 发布描述
            # description_str = self.generate_description(result)
            description_str = self.generate_description(object_positions, result)
            self.description_pub.publish(String(description_str))
            
            # 可视化（可选）
            vis_img = self.visualize_detections(color_img, object_positions)
            self.publish_visualization(vis_img, timestamp)
            
        except Exception as e:
            rospy.logerr(f"Processing error: {str(e)}")
        finally:
            # 清理临时文件
            try:
                os.remove(temp_img_path)
            except:
                pass
            
            # 返回待机状态
            with self.state_lock:
                self.state = NodeState.STANDBY

    def detect_objects_with_vision_model(self, image_path, prompt):
        """使用视觉大模型检测物体并获取位置"""
        try:
            # 读取图像并编码为base64
            base64_image = self.encode_image(image_path)
            
            # 构建更详细的提示词
            detailed_prompt = (
                f"{prompt}。Return objects in the following JSON format using English names only:"
                "{"
                "  \"objects\": ["
                "    {\"name\": \"物体名称\","
                "     \"bbox\": [x_min, y_min, x_max, y_max],"
                "     \"confidence\": 0.95,"
                "     \"description\": \"详细描述\""
                "    }"
                "  ]"
                "}"
                "确保JSON可以被解析。"
            )
            
            # 请求配置
            api_key = 'sk-f4a060eef3c747fc906b4a49731beb5c'
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {api_key}"
            }
            
            # 请求体
            payload = {
                "model": self.model_name,
                "messages": [
                    {
                        "role": "system",
                        "content": [{"type": "text", "text": "你是一个机器人视觉感知系统，需要在JSON中精确标记物体位置。"}],
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{base64_image}"}},
                            {"type": "text", "text": detailed_prompt}
                        ],
                    },
                ]
            }
            
            # 发送请求
            response = requests.post(
                "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions",
                headers=headers,
                json=payload,
                timeout=15.0
            )
            
            response.raise_for_status()
            result = response.json()
            content = result['choices'][0]['message']['content']
            
            # 从响应中提取JSON内容
            try:
                # 定位JSON内容的开始和结束位置
                json_start = content.find('{')
                json_end = content.rfind('}') + 1
                json_str = content[json_start:json_end]
                
                # 解析JSON
                obj_data = json.loads(json_str)
                return obj_data
            except (ValueError, KeyError) as e:
                rospy.logerr(f"Failed to parse JSON from vision model: {str(e)}")
                rospy.logerr(f"Model response: {content}")
                return None
                
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Vision model API request failed: {str(e)}")
            return None
        
    def get_source_frame(self):
        """
        优先用 camera_info 的 frame_id，其次 depth 图，其次 color 图。
        都没有才退回旧值（不推荐，但保证不崩）。
        """
        if self.camera_info_frame_id:
            return self.camera_info_frame_id
        if self.depth_frame_id:
            return self.depth_frame_id
        if self.color_frame_id:
            return self.color_frame_id
        return "camera_depth_optical_frame"

    def encode_image(self, path):
        """将图像转换为Base64编码"""
        with open(path, "rb") as img_file:
            return base64.b64encode(img_file.read()).decode('utf-8')
    
    def calculate_3d_positions(self, color_img, depth_img, camera_info, detected_objects):
        """为检测到的物体计算3D位置"""
        object_positions = []
        height, width = color_img.shape[:2]
        
        # 从相机信息中提取内参
        fx = camera_info.K[0]  # 焦距x
        fy = camera_info.K[4]  # 焦距y
        cx = camera_info.K[2]  # 光学中心x
        cy = camera_info.K[5]  # 光学中心y
        
        for obj in detected_objects:
            # 获取边界框坐标
            bbox = obj['bbox']
            x_min, y_min, x_max, y_max = map(int, bbox)
            
            # 确保边界框在图像范围内
            x_min = max(0, min(x_min, width-1))
            y_min = max(0, min(y_min, height-1))
            x_max = max(0, min(x_max, width-1))
            y_max = max(0, min(y_max, height-1))
            
            # 计算中心点坐标（2D）
            center_x = (x_min + x_max) // 2
            center_y = (y_min + y_max) // 2
            
            # 在中心点获取深度值（单位：毫米）
            depth_value = depth_img[center_y, center_x]
            
            if depth_value == 0:
                # 中心点深度无效，尝试邻近点
                roi = depth_img[max(0, y_min):min(height, y_max+1), 
                              max(0, x_min):min(width, x_max+1)]
                nonzero_depth = roi[roi > 0]
                
                if nonzero_depth.size > 0:
                    depth_value = np.median(nonzero_depth)
                else:
                    depth_value = 0  # 保持0，表示深度无效
            
            if depth_value > 0:
                # 将深度值转换为米
                depth_m = depth_value / 1000.0
                
                # 像素坐标转换为3D相机坐标
                z = depth_m
                x = (center_x - cx) * z / fx
                y = (center_y - cy) * z / fy
                
                obj_position = {
                    'name': obj['name'],
                    'conf': obj['confidence'],
                    'position_raw': obj['bbox'],
                    'camera_position': (x, y, z),
                    'confidence': obj['confidence']
                }
                
                object_positions.append(obj_position)
                
                if self.debug_mode:
                    rospy.loginfo(f"{obj['name']} at camera coordinates: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m")
            else:
                rospy.logwarn(f"No valid depth for {obj['name']} at ({center_x}, {center_y})")
                obj['camera_position'] = None
        
        return object_positions

    def process_results(self, color_img, object_positions, timestamp):
        """处理检测结果并发布位置"""
        if not object_positions:
            rospy.loginfo("No valid object positions calculated")
            return
        
        # 尝试获取到目标坐标系的变换
        try:
            source_frame = self.get_source_frame()
            if self.debug_mode:
                rospy.loginfo(f"Using source_frame for 3D points: {source_frame}")

            tf_time = rospy.Time(0) 
            transform = self.tf_buffer.lookup_transform(
                "map",              # 目标坐标系
                source_frame,       # 源坐标系
                tf_time,
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {str(e)}. Publishing in camera frame.")
            transform = None
        
        # 创建并发布位置消息
        for obj in object_positions:
            if obj['camera_position'] is None:
                continue
                
            x, y, z = obj['camera_position']
            
            # 创建PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = source_frame
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            
            # 如果没有变换，直接使用相机坐标系
            if transform:
                # 创建PointStamped用于变换
                point_original = PointStamped()
                point_original.header.stamp = tf_time
                point_original.header.frame_id = source_frame
                point_original.point.x = x
                point_original.point.y = y
                point_original.point.z = z
                
                # 执行坐标变换
                try:
                    point_transformed = do_transform_point(point_original, transform)
                    pose_msg.pose.position = point_transformed.point
                    pose_msg.header.frame_id = "map"
                    
                    if self.debug_mode:
                        rospy.loginfo(f"{obj['name']} map position: "
                                     f"x={pose_msg.pose.position.x:.3f}m, "
                                     f"y={pose_msg.pose.position.y:.3f}m, "
                                     f"z={pose_msg.pose.position.z:.3f}m")
                except Exception as e:
                    rospy.logerr(f"Transform failed: {str(e)}")
            
            # 发布位置
            self.position_pub.publish(pose_msg)

    def visualize_detections(self, color_img, object_positions):
        """可视化检测结果 (在图像上绘制边界框和距离)"""
        vis_img = color_img.copy()
        
        for obj in object_positions:
            # 获取边界框和位置
            x_min, y_min, x_max, y_max = obj['position_raw']
            position = obj['camera_position']
            
            # 转换为整数
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
            
            # 绘制边界框
            cv2.rectangle(vis_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            # 添加标签和距离信息
            label = obj['name']
            if position is not None:
                distance = np.sqrt(position[0]**2 + position[1]**2 + position[2]**2)
                label += f" {distance:.2f}m"
            
            # 使用英文字体绘制标签
            cv2.putText(vis_img, label, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
            # 标记中心点
            center_x = (x_min + x_max) // 2
            center_y = (y_min + y_max) // 2
            cv2.circle(vis_img, (center_x, center_y), 5, (0, 0, 255), -1)
            
        return vis_img
    
    def publish_visualization(self, vis_img, timestamp):
        """发布可视化图像"""
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "bgr8")
            vis_msg.header.stamp = timestamp
            self.visualization_pub.publish(vis_msg)
        except Exception as e:
            rospy.logwarn(f"Failed to publish visualization: {str(e)}")

    def generate_description(self, object_positions, result=None):
        """优先基于 object_positions 生成距离播报；可选补充无深度目标"""
        if not object_positions:
            # 没算出任何3D点（深度都无效）
            if result and result.get('objects'):
                names = [o.get('name', 'unknown') for o in result['objects']]
                return "检测到物体，但无法获取有效深度/距离：" + "、".join(names)
            return "未检测到任何物体或深度无效"

        desc_parts = []
        names_with_depth = set()

        # 1) 有深度的：一定播报距离
        for o in object_positions:
            name = o.get('name', 'unknown')
            cam_pos = o.get('camera_position')
            if cam_pos is None:
                continue
            x, y, z = cam_pos
            dist = float(np.sqrt(x*x + y*y + z*z))
            desc_parts.append(f"- {name} 位于相机前方约 {dist:.2f} 米处")
            names_with_depth.add(name)

        # 2) 可选：把“检测到但无深度”的也说出来（避免你觉得漏报）
        if result and result.get('objects'):
            for obj in result['objects']:
                n = obj.get('name', 'unknown')
                if n not in names_with_depth:
                    desc_parts.append(f"- 检测到 {n}，但无法获取有效深度/距离")

        return "\n".join(desc_parts)

    def spin(self):
        """主循环"""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObjectLocatorNode()
        rospy.loginfo("Starting object locator node...")
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown by ROS")
    except Exception as e:
        rospy.logfatal("Unhandled exception: %s", str(e))