# OCR Node — 基于 PaddleOCR 的 ROS2 文字/数字识别节点

完全解耦的独立 ROS2 节点，订阅海康相机发布的 `/hk_camera/image_raw` 图像话题，使用 PaddleOCR 进行文字和数字识别，发布结构化识别结果。

## 架构

```
hk_camera_node                  ocr_node (本包)              下游消费者
┌──────────────┐               ┌──────────────────┐        ┌──────────────┐
│ Hikvision    │   Image       │  PaddleOCR        │ Result │ 决策/记录     │
│ Camera ──────┼──────────────▶│  ┌────────────┐  │───────▶│ 节点          │
│              │ /hk_camera/   │  │ DB 文本检测  │  │        │              │
└──────────────┘ image_raw     │  │ CRNN 识别   │  │ /ocr/  └──────────────┘
                               │  └────────────┘  │ result
                               │                  │ Image
                               │  预览标注 ◉──────┼────────▶ /ocr/annotated_img
                               └──────────────────┘
```

## 包结构

```
src/ocr_interfaces/          # 纯接口包 (CMake, rosidl)
├── msg/
│   ├── OcrDetection.msg     # 单条检测: corners[8] + text + confidence
│   └── OcrResult.msg        # 一帧结果: header + detections[] + proc_time

src/ocr_node/                # Python 节点包 (ament_python)
├── ocr_node/
│   ├── __init__.py
│   └── ocr_node.py          # 主节点实现
├── config/
│   └── ocr_params.yaml      # 默认参数
├── launch/
│   └── ocr_node.launch.py   # 启动文件
├── package.xml
├── setup.py
└── setup.cfg
```

## 安装

### 1. 安装 PaddleOCR

```bash
# Jetson / Linux
pip install paddlepaddle-gpu paddleocr  # GPU 版
# 或
pip install paddlepaddle paddleocr      # CPU 版

# 如果只需要英文/数字（更轻量）
pip install paddleocr
```

### 2. 编译接口包

```bash
cd ~/kybot_ws
colcon build --packages-select ocr_interfaces
source install/setup.bash
```

### 3. 构建节点包

```bash
colcon build --packages-select ocr_node
source install/setup.bash
```

## 使用

### 独立启动

```bash
# 默认参数 (中文, GPU, 0.5s间隔)
ros2 launch ocr_node ocr_node.launch.py

# 只识别英文/数字
ros2 launch ocr_node ocr_node.launch.py lang:=en

# 提高处理频率 (200ms, 约5FPS)
ros2 launch ocr_node ocr_node.launch.py processing_interval:=0.2

# Jetson CPU 模式
ros2 launch ocr_node ocr_node.launch.py use_gpu:=false
```

### 与 hk_camera 组合启动

可以在 `kybot_bringup` 的 launch 文件中添加：

```python
# 在 bringup.launch.py 中加入
from launch_ros.actions import Node

ocr_node = Node(
    package='ocr_node',
    executable='ocr_node',
    name='ocr_node',
    output='screen',
    parameters=[{
        'lang': 'ch',
        'use_gpu': False,  # Jetson 上建议关 GPU（除非装了 Paddle-GPU）
        'conf_threshold': 0.5,
        'processing_interval': 0.5,
        'enable_annotated_img': True,
    }],
)
```

### 命令行查看结果

```bash
# 查看结构化识别结果
ros2 topic echo /ocr/result

# 查看标注预览图
ros2 run rqt_image_view rqt_image_view /ocr/annotated_img
```

## ROS2 API

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/hk_camera/image_raw` | `sensor_msgs/Image` | 海康相机原始图像 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ocr/result` | `ocr_interfaces/OcrResult` | 结构化识别结果 |
| `/ocr/annotated_img` | `sensor_msgs/Image` | 带标注框的预览图（可选） |

### OcrResult 消息格式

```
std_msgs/Header header              # 原始图像的帧头（时间戳）
OcrDetection[] detections           # 本次检测到的所有文字区域
  float32[8] corners                # 四边形4个角点 (x1,y1,x2,y2,x3,y3,x4,y4)
  string text                       # 识别出的文字
  float32 confidence               # 置信度 (0.0-1.0)
float32 processing_time_ms          # 此帧推理耗时(ms)
```

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lang` | string | `"ch"` | 语言: `ch`(中英), `en`(英文) |
| `use_gpu` | bool | `true` | 是否使用 GPU 加速 |
| `conf_threshold` | float64 | `0.5` | 置信度阈值，低于此值过滤 |
| `processing_interval` | float64 | `0.5` | 两次 OCR 的最小间隔(秒) |
| `enable_annotated_img` | bool | `true` | 是否发布标注预览图 |

## Jetson 性能调优

### 推荐配置 (Jetson Orin/Xavier)

```yaml
ocr_node:
  ros__parameters:
    use_gpu: false           # PaddlePaddle-GPU 在 Jetson 上装较复杂，CPU 亦可
    processing_interval: 0.5 # 2 FPS，平衡精度与负载
    conf_threshold: 0.6      # 提高阈值减少误检
    enable_annotated_img: false  # 减少带宽和内存拷贝
```

### 如果要用 TensorRT 加速

PaddleOCR 支持 TensorRT 部署，但需要额外配置：

```bash
# 安装 PaddlePaddle with TensorRT
pip install paddlepaddle-tensorrt
```

然后在代码中通过 Paddle Inference 配置 TensorRT，或使用 NVIDIA DeepStream 方案（见方案文档）。

## 自定义：添加数字识别专用逻辑

如果你的场景是**仪表盘数字、计数器**等，可以在 `ocr_node.py` 的 `_process_frame()` 中添加后处理：

```python
import re

# 在检测结果中过滤/提取数字
for d in detections:
    # 只保留含数字的结果
    if re.search(r'\d', d.text):
        self.get_logger().info(f'Number found: {d.text}')
```

---

## 依赖

| 包 | 用途 |
|---|---|
| `ocr_interfaces` | 自定义 ROS2 消息 |
| `paddleocr` | OCR 引擎 |
| `opencv-python` | 图像处理 |
| `rclpy` | ROS2 Python 客户端 |
| `cv_bridge` | OpenCV ↔ ROS Image 桥接 |
| `sensor_msgs` | Image 消息类型 |
