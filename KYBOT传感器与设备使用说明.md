# KYBOT ROS2 传感器与设备使用说明

本文档根据 `/home/nvidia/kybot_ws` 当前工程整理，适用于 ROS2 Humble。文档中的命令均使用 ROS2 写法，即 `ros2 topic`、`ros2 service` 和 `ros2 launch`。

当前工程包含的主要设备和功能如下：

| 设备/模块 | 工程包 | 主要作用 | 主要话题 |
|---|---|---|---|
| 二维激光雷达 | `hi_ros2` | 障碍物检测、导航避障 | `/scan_fe` |
| 三维激光雷达 | `rslidar_sdk` | 三维点云、FAST-LIO 定位 | `/rslidar_points`、`/velodyne_points` |
| Livox 三维激光雷达（备用链路） | `livox_ros_driver2` | MID360/HAP 点云 | 以运行时实际列出的点云话题为准 |
| 海康网络相机 | `hk_camera` | 视频流、抓图、云台、报警 | `/hk_camera/image_raw` |
| 深度相机 D435 | `realsense2_camera` + `trash_mission` | 彩色图、深度图、目标测距 | `/front_camera/camera/...` |
| IMU | `wit_ros2_imu` | 加速度、角速度、姿态角 | `/imu/data` |
| YHS 底盘 | `yhs_can_control` | CAN 控制、里程计、底盘反馈 | `/cmd_vel`、`/odom` |
| OCR | `ocr_node` | 海康图像文字识别 | `/ocr/recognize` 服务 |

## 1. 启动前准备

### 1.1 打开终端并加载 ROS2 环境

```bash
cd /home/nvidia/kybot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果工作空间还没有编译，先执行：

```bash
cd /home/nvidia/kybot_ws
colcon build --symlink-install --cmake-args -DDISTRO_ROS=humble
source install/setup.bash
```

### 1.2 查看当前节点和话题

打开新的终端后，都需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/kybot_ws/install/setup.bash
```

查看当前运行的节点：

```bash
ros2 node list
```

查看当前发布的话题：

```bash
ros2 topic list
```

查看某个话题的消息类型和发布/订阅数量：

```bash
ros2 topic info /scan_fe
```

### 1.3 一键启动整车系统

只启动底盘、激光雷达、IMU、定位、Nav2 和海康相机：

```bash
ros2 launch kybot_bringup bringup.launch.py
```

该启动文件会自动启动以下模块，并根据延时分批运行：CAN 底盘、`cmd_vel` 桥接、二维激光雷达、RoboSense 三维激光雷达、点云转换、IMU、EKF、FAST-LIO、海康相机、任务执行器和 Nav2。

常用参数：

```bash
# 不重新配置 CAN，只启动 ROS 节点
ros2 launch kybot_bringup bringup.launch.py setup_can:=false

# 启动海康相机和 OCR
ros2 launch kybot_bringup bringup.launch.py use_ocr:=true
```

如果需要同时启动 D435 深度相机和车前目标检测，使用：

```bash
ros2 launch kybot_bringup trash_pipeline.launch.py use_rviz:=true
```

`trash_pipeline.launch.py` 已经包含 `bringup.launch.py`，不要再同时启动两个整车启动文件，否则会重复启动底盘、雷达和 Nav2。

## 2. 查看二维激光雷达数据

### 2.1 启动二维激光雷达节点

当前整车配置使用的是 Hins FE 型二维激光雷达，IP 地址为 `192.168.1.88`，端口为 `8080`，使用 TCP 通信。

单独启动二维激光雷达：

```bash
source /home/nvidia/kybot_ws/install/setup.bash
ros2 launch hi_ros2 hins_fe_launch.py
```

整车启动时，该节点会由 `kybot_bringup` 自动启动，不需要重复运行。

### 2.2 查看当前发布的话题

```bash
ros2 topic list | grep -E 'scan|laser'
```

当前 FE 雷达的主要话题如下：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/scan_fe` | `sensor_msgs/msg/LaserScan` | 二维激光扫描数据，导航使用该话题 |
| `/scanlaser` | `sensor_msgs/msg/PointCloud2` | 驱动同时生成的点云格式数据 |
| `/laser_info` | `hi_ros2/msg/LaserInfo` | 雷达状态、设备信息和区域报警信息 |

查看 `/scan_fe` 的类型：

```bash
ros2 topic type /scan_fe
```

### 2.3 查看二维激光雷达扫描数据

查看一帧扫描数据：

```bash
ros2 topic echo /scan_fe --once
```

查看雷达发布频率：

```bash
ros2 topic hz /scan_fe
```

查看雷达状态信息：

```bash
ros2 topic echo /laser_info --once
```

正常情况下，`/scan_fe` 的 `header.frame_id` 应为 `laser_fe`，导航参数文件中使用的也是 `/scan_fe`。

### 2.4 二维雷达控制服务

查看服务是否存在：

```bash
ros2 service list | grep laser
```

重启雷达：

```bash
ros2 service call /laser_cmd hi_ros2/srv/CmdSrv "{cmd: 1}"
```

停止点云发送：

```bash
ros2 service call /laser_cmd hi_ros2/srv/CmdSrv "{cmd: 2}"
```

重新开启点云发送：

```bash
ros2 service call /laser_cmd hi_ros2/srv/CmdSrv "{cmd: 3}"
```

## 3. 查看三维激光雷达数据（RoboSense）

### 3.1 启动 RoboSense 三维雷达

当前 `bringup.launch.py` 默认启动 `rslidar_sdk`，配置文件为：

```text
/home/nvidia/kybot_ws/src/rslidar_sdk-v1.5.19/config/config.yaml
```

单独启动驱动：

```bash
ros2 run rslidar_sdk rslidar_sdk_node
```

当前配置使用 RSHELIOS 型号，MSOP 端口为 `6699`，DIFOP 端口为 `7788`。如果修改雷达型号、端口或点云话题，应先修改 `config.yaml`，再重新启动节点。

### 3.2 查看原始点云和转换后的点云

RoboSense 驱动原始点云话题：

```bash
ros2 topic echo /rslidar_points --once
ros2 topic hz /rslidar_points
```

工程中的 `rs_to_velodyne` 节点把 `/rslidar_points` 转换为 FAST-LIO 使用的 Velodyne 格式：

```bash
ros2 run rs_to_velodyne rs_to_velodyne
```

查看转换后的点云：

```bash
ros2 topic echo /velodyne_points --once
ros2 topic hz /velodyne_points
ros2 topic info /velodyne_points
```

当前点云链路为：

```text
rslidar_sdk → /rslidar_points → rs_to_velodyne → /velodyne_points → FAST-LIO
```

`/velodyne_points` 的 `frame_id` 固定为 `velodyne`。FAST-LIO 配置使用 `/velodyne_points` 作为激光雷达输入，时间字段单位为秒。

### 3.3 在 RViz2 中查看三维点云

```bash
ros2 run rviz2 rviz2
```

在 RViz2 中设置：

1. `Global Options → Fixed Frame` 设置为 `base_link` 或 `velodyne`。
2. 添加 `PointCloud2` 显示项。
3. 话题选择 `/velodyne_points`。
4. 如果使用原始 RoboSense 点云，也可以选择 `/rslidar_points`。

### 3.4 Livox MID360 备用启动方式

工程中还保留了 Livox ROS2 驱动。如果实际安装的是 Livox MID360，应使用 Livox 启动文件，不要和 RoboSense 驱动同时启动：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

查看 Livox 实际发布的话题和类型：

```bash
ros2 topic list | grep -i livox
ros2 topic list | grep -E 'point|livox'
```

Livox 启动文件默认使用 `livox_frame` 坐标系，点云格式由 `xfer_format` 参数决定。使用 Livox 进行 FAST-LIO 时，需要同步修改 FAST-LIO 配置中的激光输入话题、线数和坐标系。

## 4. 查看 IMU 数据

### 4.1 检查 IMU 串口

```bash
ls -l /dev/ttyCH341USB0 /dev/imu_usb /dev/ttyUSB0 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

当前整车启动文件使用的 IMU 参数为：

```text
串口：/dev/ttyCH341USB0
波特率：921600
协议：RS485_HIGH
坐标系：imu_link
```

### 4.2 启动 IMU 节点

单独启动 IMU：

```bash
ros2 run wit_ros2_imu wit_ros2_imu \
  --ros-args -p port:=/dev/ttyCH341USB0 -p baudrate:=921600
```

如果系统实际分配的串口名称不同，将 `port` 改成实际设备，例如 `/dev/ttyUSB0` 或 `/dev/imu_usb`。

### 4.3 查看 IMU 话题

```bash
ros2 topic list | grep imu
```

主要话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/imu/data` | `sensor_msgs/msg/Imu` | 加速度、角速度和四元数姿态 |
| `/imu/ImuDataWithRPY` | `imu_msg/msg/ImuData` | IMU 数据和 RPY 欧拉角 |

查看一帧数据和发布频率：

```bash
ros2 topic echo /imu/data --once
ros2 topic hz /imu/data
ros2 topic echo /imu/ImuDataWithRPY --once
```

如果 `/imu/data` 没有数据，优先检查串口权限、设备名称、波特率和 IMU 协议设置。

## 5. 查看海康相机数据

### 5.1 启动海康相机节点

```bash
ros2 launch hk_camera hk_camera.launch.py
```

指定相机 IP 和通道：

```bash
ros2 launch hk_camera hk_camera.launch.py \
  ip:=192.168.1.64 port:=8000 channel:=1
```

整车启动时，海康相机默认由 `bringup.launch.py` 启动。使用 `use_ocr:=true` 时，由 `ocr_camera.launch.py` 统一启动海康相机和 OCR，不能再重复启动相机节点。

### 5.2 查看海康相机话题

```bash
ros2 topic list | grep hk_camera
```

主要话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/hk_camera/image_raw` | `sensor_msgs/msg/Image` | 海康视频帧 |
| `/hk_camera/status` | `std_msgs/msg/String` | 设备和码流状态 |
| `/hk_camera/alarm` | `hk_camera/msg/CameraAlarm` | 报警事件 |

查看图像帧率和状态：

```bash
ros2 topic hz /hk_camera/image_raw
ros2 topic echo /hk_camera/status --once
ros2 topic echo /hk_camera/alarm --once
```

图像消息通常包含大量二进制数据，不建议直接长时间执行 `ros2 topic echo`。查看画面时，建议使用工程中的 GUI 或在 RViz2 中添加 `Image` 显示项，话题选择 `/hk_camera/image_raw`。

### 5.3 海康相机服务

查看服务：

```bash
ros2 service list | grep hk_camera
```

手动登录、开始取流和停止取流：

```bash
ros2 service call /hk_camera/login std_srvs/srv/Trigger "{}"
ros2 service call /hk_camera/start_stream std_srvs/srv/Trigger "{}"
ros2 service call /hk_camera/stop_stream std_srvs/srv/Trigger "{}"
```

抓图并保存到指定文件：

```bash
ros2 service call /hk_camera/capture hk_camera/srv/CapturePicture \
  "{quality: 0, save_path: '/tmp/hk_capture.jpg'}"
```

控制云台预置点：

```bash
ros2 service call /hk_camera/set_ptz_pose hk_camera/srv/SetPTZPose \
  "{pan: 1.0, tilt: 0.0, zoom: 1.0}"
```

当前驱动将 `pan` 字段作为预置点编号使用，因此上面的示例表示转到 1 号预置点；具体预置点应根据相机配置填写。

## 6. 查看深度相机 D435 数据

### 6.1 通过车前感知任务启动 D435

当前工程的车前感知启动文件为：

```bash
ros2 launch trash_mission trash_mission.launch.py
```

该文件会启动：

- D435 彩色图像；
- D435 深度图像；
- 深度对齐到彩色图像；
- `front_perception_node` 目标检测和距离测量。

D435 在该启动文件中使用 `front_camera` 命名空间，因此实际话题不是默认的 `/camera/...`，而是：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/front_camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | 彩色图 |
| `/front_camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | 对齐后的深度图 |
| `/front_camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色相机内参 |
| `/trash/annotated_image` | `sensor_msgs/msg/Image` | 带检测框和距离的图像 |
| `/trash/target` | `trash_mission_interfaces/msg/TrashTarget` | 目标、距离和停车确认结果 |

查看深度相机话题：

```bash
ros2 topic list | grep -E 'front_camera|trash'
```

查看彩色图、深度图的频率：

```bash
ros2 topic hz /front_camera/camera/color/image_raw
ros2 topic hz /front_camera/camera/aligned_depth_to_color/image_raw
```

查看车前检测结果：

```bash
ros2 topic echo /trash/target --once
ros2 topic echo /trash/detection --once
```

### 6.2 单独启动 RealSense 驱动

只检查 D435 是否能正常出图时，可以直接启动 RealSense 驱动：

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true
```

该方式默认使用 `/camera/...` 话题。检查默认话题：

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

注意：`trash_mission` 使用 `/front_camera/...`，而 `qwen_vision` 默认订阅 `/camera/...`。如果启动 Qwen 视觉定位节点，必须保证相机话题名称与节点订阅名称一致，或者增加 remap。

### 6.3 Qwen 深度视觉定位

启动 Qwen 视觉节点：

```bash
ros2 launch qwen_vision qwen_vision.launch.py
```

该 launch 文件只启动 Qwen 视觉处理节点，不负责启动 D435 相机。节点默认订阅：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
```

设置目标物并请求同步定位：

```bash
ros2 topic pub --once /target_object std_msgs/msg/String "{data: 'bottle'}"
ros2 service call /locate_object_sync std_srvs/srv/Trigger "{}"
```

查看输出：

```bash
ros2 topic echo /object_position --once
ros2 topic echo /object_name --once
ros2 topic echo /visualization --once
```

Qwen 节点需要有效的模型 API 配置。API 密钥应通过环境变量或 ROS 参数配置，不要把密钥写入说明文档或提交到代码仓库。

## 7. 查看和测试底盘数据

### 7.1 配置 CAN1

当前底盘使用 `can1`，波特率为 `500000`：

```bash
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up
ip -details link show can1
```

如果使用整车启动文件，默认会自动执行 CAN 配置：

```bash
ros2 launch kybot_bringup bringup.launch.py setup_can:=true
```

### 7.2 启动底盘节点

```bash
ros2 launch yhs_can_control yhs_can_control.launch.py
```

底盘参数文件为：

```text
/home/nvidia/kybot_ws/src/yhs_can_control/params/cfg.yaml
```

### 7.3 查看底盘反馈和里程计

主要话题如下：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 底盘里程计 |
| `/chassis_info_fb` | `yhs_can_interfaces/msg/ChassisInfoFb` | 底盘控制、IO 和超声波反馈 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 输出的速度指令 |
| `/ctrl_cmd` | `yhs_can_interfaces/msg/CtrlCmd` | 底盘运动控制指令 |
| `/io_cmd` | `yhs_can_interfaces/msg/IoCmd` | 底盘 IO 控制指令 |

查看里程计和底盘反馈：

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 topic echo /chassis_info_fb --once
```

### 7.4 速度指令说明

整车启动时，`cmd_vel_bridge.py` 会把 Nav2 的 `/cmd_vel` 转换成底盘的 `ctrl_cmd` 和 `io_cmd`。桥接节点具备速度限幅和超时急停功能，默认超过约 `0.5` 秒没有收到 `/cmd_vel` 会停车。

测试底盘前必须确认车辆处于安全状态。建议先抬起驱动轮或断开动力，并只发布零速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 8. 查看坐标变换 TF

当前主要坐标关系如下：

```text
map → odom → base_link
                 ├── laser_fe
                 └── velodyne
```

查看 TF 树：

```bash
ros2 run tf2_tools view_frames
```

实时查看底盘到二维雷达的变换：

```bash
ros2 run tf2_ros tf2_echo base_link laser_fe
```

实时查看底盘到三维雷达的变换：

```bash
ros2 run tf2_ros tf2_echo base_link velodyne
```

如果 RViz2 中出现点云或激光雷达不显示，优先检查：

1. Fixed Frame 是否设置正确；
2. 点云或 LaserScan 的 `header.frame_id` 是否存在对应 TF；
3. `/scan_fe`、`/velodyne_points` 是否正在发布；
4. 雷达驱动和 TF 是否被重复启动或使用了不同的坐标系名称。

## 9. 常用整体检查命令

### 9.1 一次查看所有设备相关话题

```bash
ros2 topic list | grep -E \
  'scan|laser|rslidar|velodyne|imu|hk_camera|camera|trash|odom|cmd_vel|chassis'
```

### 9.2 查看所有服务

```bash
ros2 service list
```

### 9.3 查看关键话题频率

```bash
ros2 topic hz /scan_fe
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic hz /hk_camera/image_raw
ros2 topic hz /odom
```

### 9.4 录制传感器数据

确认话题正常后，可以使用 rosbag 录制：

```bash
ros2 bag record -o kybot_sensor_bag \
  /scan_fe \
  /velodyne_points \
  /imu/data \
  /odom \
  /hk_camera/image_raw
```

回放：

```bash
ros2 bag play kybot_sensor_bag
```

## 10. 常见问题

### 10.1 `ros2` 找不到包或 launch 文件

重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/kybot_ws/install/setup.bash
```

然后检查包是否存在：

```bash
ros2 pkg list | grep -E 'kybot_bringup|hi_ros2|rslidar_sdk|hk_camera|wit_ros2_imu|yhs_can_control'
```

### 10.2 二维雷达没有 `/scan_fe`

依次检查：

```bash
ping 192.168.1.88
ros2 node list | grep hi_ros2
ros2 topic list | grep scan
ros2 topic hz /scan_fe
```

并确认没有误启动 `hins_he_launch.py`、`hins_se_launch.py` 或其他型号的 launch 文件。

### 10.3 三维雷达有 `/rslidar_points`，但没有 `/velodyne_points`

检查转换节点是否运行：

```bash
ros2 node list | grep rs_to_velodyne
ros2 topic info /rslidar_points
ros2 topic info /velodyne_points
```

如果没有转换节点，手动启动：

```bash
ros2 run rs_to_velodyne rs_to_velodyne
```

### 10.4 D435 没有图像或深度值全为零

确认没有其他程序占用 D435，例如 `realsense-viewer`；然后重新检查：

```bash
ros2 topic list | grep camera
ros2 topic hz /front_camera/camera/color/image_raw
ros2 topic hz /front_camera/camera/aligned_depth_to_color/image_raw
```

同时确认当前节点使用的是 `/front_camera/...` 还是 `/camera/...` 话题。

### 10.5 IMU 没有数据

```bash
ls -l /dev/ttyCH341USB0 /dev/imu_usb /dev/ttyUSB0 2>/dev/null
ros2 topic hz /imu/data
```

如果串口名称发生变化，使用实际串口重新启动节点，并检查串口权限和波特率。

### 10.6 底盘没有 `/odom`

```bash
ip -details link show can1
ros2 node list | grep yhs_can_control
ros2 topic echo /chassis_info_fb --once
ros2 topic hz /odom
```

确认 CAN1 已经启动、底盘控制节点正在运行，并且底盘能够返回反馈帧。

## 11. Elite CS66 机械臂工作空间

第二个工作空间位于 `~/Documents/elite_robot_ws`，主要负责 Elite CS66 六轴机械臂、图漾深度相机、手眼标定、视觉抓取、夹爪和力控打磨。它与本工作空间中的 KYBOT 底盘、2D/3D 激光雷达、海康相机属于两套功能链路，可以按需要组合使用。

主要功能包和脚本对应关系如下：

| 功能 | 包/目录 | 作用 |
|---|---|---|
| Elite 机械臂驱动 | `eli_cs_robot_driver` | 真实机械臂连接、ros2_control、Dashboard、外部脚本 |
| 机械臂工作站封装 | `my_elite_robot_cell_control` | CS66 网络参数、TF 前缀和一键启动 |
| 机械臂描述/MoveIt | `eli_cs_robot_description`、`eli_cs_robot_moveit_config`、`my_elite_robot_cell_moveit_config` | URDF、规划组、轨迹执行和 RViz |
| Gazebo 仿真 | `eli_cs_robot_simulation_gz` | 无真实机械臂时检查控制器和 MoveIt 流程 |
| 图漾相机 | `percipio_camera` | 彩色图、深度图、内参和点云 |
| 手眼标定 | `easy_handeye2`、`biaoding/` | ArUco 识别、标定采样和结果转换 |
| 视觉抓取 | `YOLO/`、`biaoding/yolo_grasp*.py` | YOLO 检测、三维定位和抓取动作 |
| 大模型视觉 | `qwen_vision` | Qwen 定位后端，与 YOLO 共享目标位姿接口 |
| 夹爪 | `inspire_gripper`、`gripper_control`、LinkerHand SDK | 两指、软夹爪和灵巧手控制 |
| 力控打磨 | `elite_polish_app` | 点云定位、力传感器、工具 IO 和自动打磨 |
| 示例/测试 | `elite_robot_example`、`hello_moveit` | 状态监视、基础控制、MoveJ/MoveL 和手动 GUI |

### 11.1 工作空间准备

建议每个终端单独 source 对应工作空间；如果同一终端同时使用两个工作空间，后 source 的工作空间可能覆盖同名 ROS 包的查找路径。

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
```

检查关键包是否已经被 ROS 发现：

```bash
ros2 pkg prefix eli_cs_robot_driver
ros2 pkg prefix my_elite_robot_cell_control
ros2 pkg prefix percipio_camera
ros2 pkg prefix qwen_vision
```

如果包不存在，先在第二个工作空间构建：

```bash
cd ~/Documents/elite_robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 11.2 启动 Elite CS66 机械臂

#### 方式一：使用工作站一键启动文件

该启动文件会调用机械臂驱动，并可选择是否启动 RViz。当前项目常用的网络参数为：机器人 `192.168.1.212`，本机 `192.168.1.102`。

```bash
ros2 launch my_elite_robot_cell_control start_robot.launch.py \
  headless_mode:=true launch_rviz:=false
```

调试 TF 或 MoveIt 时可以打开 RViz：

```bash
ros2 launch my_elite_robot_cell_control start_robot.launch.py \
  headless_mode:=false launch_rviz:=true
```

#### 方式二：直接启动底层驱动

```bash
ros2 launch eli_cs_robot_driver elite_control.launch.py \
  robot_ip:=192.168.1.212 \
  local_ip:=192.168.1.102 \
  cs_type:=cs66 \
  launch_rviz:=false
```

在没有连接真实机械臂、只想检查控制器和 MoveIt 配置时，可以使用仿真硬件：

```bash
ros2 launch eli_cs_robot_driver elite_control.launch.py \
  robot_ip:=192.168.1.212 local_ip:=192.168.1.102 \
  cs_type:=cs66 use_fake_hardware:=true
```

启动真实机械臂前应确认：机械臂处于可控状态、周围没有人员或障碍物、急停可立即操作，并且速度设置较低。`headless_mode:=true` 时不要同时在示教器或其他 SDK 程序中执行运动命令，否则可能覆盖外部控制脚本。

### 11.3 查看机械臂状态和控制器

```bash
ros2 node list | grep -E 'elite|controller|dashboard'
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list | grep -E 'joint|tcp|force|io|script'
```

常用状态话题：

| 功能 | 话题 | 类型 |
|---|---|---|
| 六个关节状态 | `/joint_states` | `sensor_msgs/msg/JointState` |
| TCP 位姿 | `/tcp_pose_broadcaster/pose` | `geometry_msgs/msg/PoseStamped` |
| TCP 位姿（部分驱动版本） | `/tcp_pose_broadcaster/tcp_pose` | 以实际 `ros2 topic list` 为准 |
| 末端力/力矩 | `/force_torque_sensor_broadcaster/wrench` | `geometry_msgs/msg/WrenchStamped` |
| 外部控制脚本状态 | `/io_and_status_controller/robot_task_running` | 以实际驱动版本为准 |
| 外部脚本发送 | `/script_sender/script_command` | `std_msgs/msg/String` |

读取一次状态：

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /tcp_pose_broadcaster/pose --once
ros2 topic echo /force_torque_sensor_broadcaster/wrench --once
```

默认关节名称为：

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

关节位置通常以弧度表示。驱动和控制器异常时，先确认 `scaled_joint_trajectory_controller` 是否 active：

```bash
ros2 control list_controllers | grep scaled
ros2 control set_controller_state \
  scaled_joint_trajectory_controller active
```

常用 Dashboard 服务包括：

```bash
ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger "{}"
ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger "{}"
ros2 service call /dashboard_client/play std_srvs/srv/Trigger "{}"
ros2 service call /dashboard_client/pause std_srvs/srv/Trigger "{}"
ros2 service call /dashboard_client/stop std_srvs/srv/Trigger "{}"
```

不同驱动版本的 Dashboard 服务类型可能不同，执行前先检查：

```bash
ros2 service type /dashboard_client/power_on
ros2 service list | grep dashboard
```

向机械臂发送脚本示例：

```bash
ros2 topic pub --once /script_sender/script_command \
  std_msgs/msg/String "{data: 'popup(\"hello\")'}"
```

运动脚本会改变机械臂当前控制状态。手动运动或脚本运动后，如果控制器不再响应，可以尝试重新发送外部控制脚本：

```bash
ros2 service call /io_and_status_controller/resend_external_script \
  std_srvs/srv/Trigger "{}"
```

### 11.4 机械臂状态监视和示例控制

工作空间提供状态监视和示例控制节点：

```bash
ros2 run elite_robot_example status_monitor
ros2 run elite_robot_example robot_controller
ros2 run elite_robot_example robot_basic_control
```

示例控制程序通常包含回零、准备位、关节运动、上电、刹车释放和 IO 控制等功能。真实机械臂上执行任何运动前，必须先确认关节角度、速度、工作空间和工具状态；建议先使用 `use_fake_hardware:=true` 验证程序流程。

### 11.5 启动 MoveIt

先启动机械臂驱动，再在另一个终端启动 MoveIt：

```bash
# 终端一：机械臂驱动
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
ros2 launch eli_cs_robot_driver elite_control.launch.py \
  robot_ip:=192.168.1.212 local_ip:=192.168.1.102 \
  cs_type:=cs66 launch_rviz:=false

# 终端二：MoveIt 和 RViz
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
ros2 launch eli_cs_robot_moveit_config cs_moveit.launch.py \
  cs_type:=cs66 launch_rviz:=true
```

MoveIt 主要用于规划和执行关节/笛卡尔运动。执行前检查规划组、末端工具坐标系、碰撞模型和当前机械臂姿态；不要在有人或障碍物靠近时直接点击 Execute。

工作站还提供拆分式 MoveIt 启动方式，适合测试自定义 MoveJ/MoveL 服务：

```bash
# 终端一：机械臂驱动
ros2 launch my_elite_robot_cell_control start_robot.launch.py \
  headless_mode:=true launch_rviz:=false

# 终端二：MoveIt move_group
ros2 launch my_elite_robot_cell_moveit_config move_group.launch.py

# 终端三：MoveJ/MoveL 测试服务
ros2 run hello_moveit grasp_move_server

# 终端四：手动输入目标点
ros2 run hello_moveit manual_move_client.py
```

也可以启动外部关节 jog GUI：

```bash
ros2 run hello_moveit joint_jog_gui.py
```

本地远程控制切换后，如果轨迹控制器没有继续执行，应先恢复外部脚本：

```bash
ros2 service call /io_and_status_controller/resend_external_script \
  std_srvs/srv/Trigger "{}"
```

### 11.6 使用 Gazebo 仿真验证

没有连接真实 CS66 时，可以使用 Gazebo 仿真检查 URDF、控制器和 MoveIt 启动流程：

```bash
cd ~/Documents/elite_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 仅启动仿真控制器
ros2 launch eli_cs_robot_simulation_gz cs_sim_control.launch.py \
  cs_type:=cs66 launch_rviz:=true

# 启动 Gazebo + MoveIt + RViz
ros2 launch eli_cs_robot_simulation_gz cs_sim_moveit.launch.py \
  cs_type:=cs66
```

仿真使用 `use_sim_time`，不能把仿真控制器和真实机械臂驱动同时启动到同一个 ROS 图中。仿真验证通过后，再替换为真实驱动并重新检查速度、限位、碰撞模型和 TCP。

### 11.7 Elite 机械臂 TF

工作站启动文件通常给机械臂添加 `cs66_` 前缀，常见 TF 名称为：

```text
cs66_base_link → ... → cs66_tool0
```

底层驱动或手眼标定自定义启动文件可能使用 `base`、`tool0` 等不带前缀的名称，因此不能只凭名称判断 TF 是否正确：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo cs66_base_link cs66_tool0
ros2 topic echo /tf_static --once
```

视觉抓取和手眼标定启动前，必须确认相机坐标系、机械臂基座坐标系和末端坐标系能连通。

## 12. 图漾 Percipio 深度相机

第二个工作空间使用 `percipio_camera`，与 KYBOT 中 RealSense D435 使用的 `/front_camera/...` 话题不是同一套接口。图漾相机通常使用 `/camera/...` 命名空间。

### 12.1 启动图漾相机

默认启动：

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
ros2 launch percipio_camera percipio_camera.launch.py
```

视觉抓取和手眼标定文档使用 1280×960 彩色/深度分辨率时：

```bash
ros2 launch percipio_camera percipio_camera.launch.py \
  color_resolution:=1280x960 \
  depth_resolution:=1280x960
```

如果需要多相机，工作空间还提供 `multi_cam.launch.py`，但应先根据实际设备序列号/IP 修改配置，不要直接假设多相机编号与现场设备一致。

### 12.2 查看彩色、深度和点云话题

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/color/camera_info --once
```

常用话题：

| 功能 | 话题 |
|---|---|
| 彩色图像 | `/camera/color/image_raw` |
| 彩色相机内参 | `/camera/color/camera_info` |
| 深度图像 | `/camera/depth/image_raw` |
| 深度相机内参 | `/camera/depth/camera_info` |
| 深度点云 | `/camera/depth/points` |
| 彩色对齐点云 | `/camera/depth_registered/points` |
| 设备事件 | `/camera/device_event` |
| 软件触发 | `/camera/soft_trigger` |
| 动态配置 | `/camera/dynamic_config` |
| 相机复位 | `/camera/reset` |

点云话题是否存在取决于启动参数。当前主启动文件可能关闭普通点云、开启彩色点云，因此实际使用前以 `ros2 topic list` 为准。

检查图像和深度是否能正常对齐：

```bash
cd ~/Documents/elite_robot_ws
python3 check_camera_info.py
python3 biaoding/verify_color_depth.py
```

如果没有输出，检查相机是否被其他程序占用、分辨率是否匹配、相关话题是否有订阅者，以及相机 SDK 是否正确安装。

## 13. Elite 机械臂手眼标定

手眼标定用于把相机坐标系中的目标位置转换到机械臂基座坐标系。当前项目是 eye-in-hand 方案，常用标定关系为：

```text
相机坐标系 → 机械臂末端 tool0 → 机械臂基座 base/cs66_base_link
```

### 13.1 启动标定所需节点

推荐按以下顺序启动：

```bash
# 终端一：Elite 机械臂
cd ~/Documents/elite_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch eli_cs_robot_driver elite_control.launch.py \
  robot_ip:=192.168.1.212 \
  local_ip:=192.168.1.102 \
  cs_type:=cs66

# 终端二：图漾相机
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
ros2 launch percipio_camera percipio_camera.launch.py \
  color_resolution:=1280x960 depth_resolution:=1280x960

# 终端三：ArUco 识别和标记 TF
cd ~/Documents/elite_robot_ws/biaoding
python3 aruco_single_tf.py

# 终端四：easy_handeye2 标定界面
ros2 launch easy_handeye2 elite_handeye_calibrate.launch.py
```

自定义标定启动文件通常使用以下帧：

```text
tracking_base_frame: camera_color_optical_frame
tracking_marker_frame: aruco_marker_frame
robot_base_frame: base
robot_effector_frame: tool0
```

如果使用工作站启动文件产生了 `cs66_base_link`、`cs66_tool0` 前缀，应以当前 TF 树和启动文件配置为准，确保标定界面中的四个坐标系真实存在。

### 13.2 采集样本、计算和保存

ArUco 默认使用 `DICT_6X6_250`、标记 ID `0`，标记尺寸约为 `0.123 m`。移动机械臂末端从多个角度观察标定板，建议采集至少 15 组、最好 20 组以上姿态，姿态应覆盖不同位置和角度。

标定界面也可以通过服务操作：

```bash
ros2 service list | grep easy_handeye
ros2 service type /easy_handeye2/calibration/take_sample
ros2 service type /easy_handeye2/calibration/compute_calibration
ros2 service type /easy_handeye2/calibration/save_calibration
```

如果使用命令行而不是 RQT 界面，可按以下顺序执行：

```bash
# 每移动一次机械臂、确认标记 TF 稳定后采集一组
ros2 service call /easy_handeye2/calibration/take_sample \
  easy_handeye2_msgs/srv/TakeSample "{}"

# 查看当前样本列表（以当前版本服务定义为准）
ros2 service call /easy_handeye2/calibration/get_sample_list \
  easy_handeye2_msgs/srv/TakeSample "{}"

# 计算并保存
ros2 service call /easy_handeye2/calibration/compute_calibration \
  easy_handeye2_msgs/srv/ComputeCalibration "{}"
ros2 service call /easy_handeye2/calibration/save_calibration \
  easy_handeye2_msgs/srv/SaveCalibration "{}"
```

典型流程是：确认 ArUco TF 稳定 → Take Sample → 改变机械臂姿态 → 重复采样 → Compute Calibration → Save Calibration。

### 13.3 标定结果和视觉程序使用

`easy_handeye2` 的结果通常保存在：

```text
~/.ros2/easy_handeye2/calibrations/elite_cs66_handeye.calib
```

本项目视觉抓取还会使用：

```text
~/Documents/elite_robot_ws/biaoding/hand_eye_result.json
```

重新标定后，需要把最新 `.calib` 转换/更新为 JSON：

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 update_hand_eye_json.py
```

验证结果时检查 TF：

```bash
ros2 run tf2_ros tf2_echo cs66_base_link cs66_tool0
ros2 topic echo /target_object_pose --once
```

如果希望在后续启动中由 `easy_handeye2` 直接发布标定 TF：

```bash
ros2 launch easy_handeye2 publish.launch.py \
  name:=elite_cs66_handeye
```

视觉抓取前应确认 JSON 文件时间和本次标定一致；不要在旧标定结果未确认的情况下直接执行抓取。

## 14. YOLO 目标检测与视觉抓取

工作空间中的 YOLO 链路为：

```text
图漾彩色/深度图 → YOLO 检测 → 深度求三维坐标 → 手眼变换 → /target_object_pose → 机械臂/夹爪抓取
```

### 14.1 YOLO 感知节点

启动感知程序：

```bash
cd ~/Documents/elite_robot_ws/YOLO
python3 yolo_grasp_perception.py
```

常用接口：

| 功能 | 接口 |
|---|---|
| 指定目标类别 | `/yolo/target_class`，`std_msgs/msg/String` |
| 目标位姿 | `/target_object_pose`，`geometry_msgs/msg/PoseStamped` |
| 标注图像 | `/yolo/annotated_image` |
| 启停感知 | `/yolo_perception/set_enabled`，`std_srvs/srv/SetBool` |

指定类别并查看结果：

```bash
ros2 topic pub --once /yolo/target_class \
  std_msgs/msg/String "{data: 'cup'}"
ros2 service call /yolo_perception/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /target_object_pose --once
ros2 topic hz /yolo/annotated_image
```

目标类别可以是 `apple`、`cup`、`apple,cup` 或 `all`，具体类别还取决于当前 YOLO 模型训练标签。模型文件应使用本机当前匹配的 `.pt` 文件；不要把其他机器生成的 TensorRT `.engine` 文件直接当作通用模型使用。

### 14.2 一键启动 LinkerHand O6 抓取

LinkerHand SDK 位于另一个工作空间，使用前需要先启动 CAN2：

```bash
sudo /usr/sbin/ip link set can2 up type can bitrate 1000000
ip link show can2
```

启动一键抓取流程：

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
source ~/Documents/linker_hand_ros2_sdk/install/setup.bash
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp.launch.py
```

该启动文件可以通过参数控制机械臂、相机、LinkerHand、YOLO 感知和抓取主程序：

```bash
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp.launch.py \
  headless_mode:=true launch_rviz:=false \
  run_perception:=true run_linker_hand:=true \
  run_camera:=true run_grasp_main:=true
```

如果 LinkerHand 的 DDS 配置要求固定域 ID 或 CycloneDDS，应只在该套设备的终端中设置对应 `ROS_DOMAIN_ID` 和 `RMW_IMPLEMENTATION`，不要无条件改动所有 KYBOT 终端的环境变量。

### 14.3 一键启动 Inspire 4B4C 两指夹爪

启动两指夹爪版本：

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_two_finger.launch.py
```

默认示例目标为 `bottle`。如需启动后直接执行抓取任务：

```bash
ros2 launch ~/Documents/elite_robot_ws/biaoding/yolo_grasp_two_finger.launch.py \
  grasp_headless:=true
```

该流程底层会启动：Elite 机械臂、图漾相机、`inspire_gripper/Gripper_control_node`、YOLO 感知和 `yolo_grasp.py`。

### 14.4 手动启动抓取主程序

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 yolo_grasp.py
```

不同夹爪可以指定参数：

```bash
python3 yolo_grasp.py --gripper two_finger
python3 yolo_grasp.py --gripper soft_touch
```

抓取主程序常用服务：

```bash
ros2 service call /yolo_grasp/grasp std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/grasp_hold std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/open std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/close std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/home std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/home2 std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/ready std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/place std_srvs/srv/Trigger "{}"
ros2 service call /yolo_grasp/status std_srvs/srv/Trigger "{}"
```

交互式程序中的常用按键：`g` 抓取，`o` 打开，`c` 关闭，`h` 回 Home，`2` 回 Home2，`r` 到 Ready，`j` 示教放置位姿，`l` 放置，`e` 开关连续感知，`t` 切换目标类别，`q` 退出。

抓取流程的基本顺序是：机械臂到 Ready → 启用感知 → 选择目标类别 → 检查 `/target_object_pose` → 抓取或 `grasp_hold` → 底盘移动到放置区域 → Home2 → 执行 `place`。底盘导航时建议机械臂保持 Home2，避免机械臂与周围结构发生碰撞。

## 15. Qwen 视觉感知后端

`qwen_vision` 与 YOLO 共用图漾相机、手眼标定和 `/target_object_pose`，可以通过统一视觉后端切换。启动 Qwen 并同时启动 YOLO：

```bash
cd ~/Documents/elite_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qwen_vision qwen_vision.launch.py \
  backend:=qwen run_yolo:=true
```

只启动 Qwen：

```bash
ros2 launch qwen_vision qwen_vision.launch.py \
  backend:=qwen run_yolo:=false
```

Qwen 视觉链路要求：

```text
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
hand_eye_result.json
cs66_base_link → cs66_tool0 TF
```

常用话题和服务：

| 功能 | 接口 |
|---|---|
| 目标位姿 | `/target_object_pose` |
| 标注图像 | `/qwen/annotated_image` |
| 目标名称 | `/qwen/object_name` |
| 目标描述 | `/qwen/description` |
| 感知完成 | `/qwen/perception_done` |
| 启停 Qwen 感知 | `/qwen_perception/set_enabled` |
| 单次定位 | `/qwen_perception/locate_object_sync` |
| 查询状态 | `/qwen_perception/status` |
| 当前后端 | `/vision_backend` |
| 统一启停 | `/vision_perception/set_enabled` |
| 统一后端查询 | `/vision_perception/backend` |

切换后端并执行一次定位：

```bash
ros2 topic pub --once /vision_backend \
  std_msgs/msg/String "{data: 'qwen'}"
ros2 service call /vision_perception/backend \
  std_srvs/srv/Trigger "{}"
ros2 service call /qwen_perception/locate_object_sync \
  std_srvs/srv/Trigger "{}"
ros2 topic echo /target_object_pose --once
```

切换回 YOLO：

```bash
ros2 topic pub --once /vision_backend \
  std_msgs/msg/String "{data: 'yolo'}"
```

Qwen 配置文件中包含模型、深度比例、距离范围和手眼 JSON 路径等参数。API Key 不应写入说明文档、代码仓库或命令历史，应通过本机私有配置或环境变量提供；使用前检查配置文件中的路径和密钥是否有效，但不要把真实密钥复制到共享文件中。

## 16. 夹爪功能

### 16.1 LinkerHand

LinkerHand 使用独立的 `linker_hand_ros2_sdk` 工作空间和 CAN2。启动后检查节点和控制话题：

```bash
ros2 node list | grep -i hand
ros2 topic list | grep hand
ros2 topic echo /cb_right_hand_control_cmd --once
```

一键 YOLO 启动文件已经包含 LinkerHand 启动选项，通常不需要再手动启动第二个夹爪节点。

### 16.2 Inspire 4B4C

```bash
ros2 run inspire_gripper Gripper_control_node
ros2 service list | grep -E 'Set|Get'
```

底层服务包括位置、速度、开口限制、清错和急停等；实际任务优先使用 `/yolo_grasp/open`、`/yolo_grasp/close` 等经过抓取流程封装的服务。首次使用时先空载、小行程测试，确认夹爪 ID、CAN 通信和方向。

### 16.3 Soft Touch

```bash
ros2 run gripper_control gripper_server
ros2 service list | grep gripper
```

软夹爪由 `gripper_control` 提供服务，具体服务类型和参数以当前安装版本为准：

```bash
ros2 service type <实际的夹爪服务名>
```

## 17. 力控打磨应用

`elite_polish_app` 用于 Elite CS66 的视觉定位、末端力/力矩检测、工具开关和自动打磨。常用启动顺序如下。

### 17.1 启动打磨系统

```bash
# 终端一：机械臂
cd ~/Documents/elite_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_elite_robot_cell_control start_robot.launch.py \
  headless_mode:=true launch_rviz:=false

# 终端二：确认缩放轨迹控制器
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash
ros2 control list_controllers | grep scaled
ros2 control set_controller_state \
  scaled_joint_trajectory_controller active

# 终端三：图漾相机（只有视觉定位需要）
ros2 launch percipio_camera percipio_camera.launch.py

# 终端四：打磨应用
cd ~/Documents/elite_robot_ws
source install/setup.bash
ros2 launch elite_polish_app elite_polish.launch.py
```

打磨启动文件会启动轨迹桥接、力控应用、3D 相机解算和交互命令节点。点云通常通过启动文件从相机原始点云重映射到：

```text
/camera/depth_registered/points
```

检查打磨相关话题：

```bash
ros2 topic list | grep -E 'elite|ys_|force|vision'
ros2 topic hz /camera/depth_registered/points
ros2 topic echo /force_torque_sensor_broadcaster/wrench --once
ros2 topic echo /ys_contact_fts_broadcaster/wrench --once
ros2 run tf2_ros tf2_echo cs66_base_link cs66_tool0
```

### 17.2 打磨控制命令

控制入口为 `/elite_forceapp_cmd`，类型是 `std_msgs/msg/Int32`。常用命令：

```bash
# 回 Home
ros2 topic pub --once /elite_forceapp_cmd \
  std_msgs/msg/Int32 "{data: 0}"

# 视觉定位并进入自动打磨流程（确认安全后再执行）
ros2 topic pub --once /elite_forceapp_cmd \
  std_msgs/msg/Int32 "{data: 3}"

# 调试模式下跳过视觉的流程入口，按项目配置使用
ros2 topic pub --once /elite_forceapp_cmd \
  std_msgs/msg/Int32 "{data: 4}"

# 打磨工具开/关
ros2 topic pub --once /elite_forceapp_cmd \
  std_msgs/msg/Int32 "{data: 52}"
ros2 topic pub --once /elite_forceapp_cmd \
  std_msgs/msg/Int32 "{data: 51}"
```

执行结果和视觉定位信息：

```bash
ros2 topic echo /elite_forceapp_cmd_result --once
ros2 topic echo /elite_vision_pose_broadcaster/pose --once
ros2 topic echo /elite_vision_job_cmd --once
```

力控调试时可以关闭力模式：

```bash
ros2 service call /force_mode_server/set_force_mode \
  eli_common_interface/srv/ForceMode "{enable: false}"
```

打磨参数位于：

```text
~/Documents/elite_robot_ws/src/elite_polish_app/config/polish_params.yaml
```

### 17.3 打磨点云和坐标辅助工具

#### 世界坐标偏移换算

当机械臂基座存在倾斜安装，需要把世界坐标系中的“抬高/侧移”换算为基座坐标系位移时：

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 world_offset_calc.py
```

程序输入目标点的基座坐标 `x y z`，再输入世界坐标偏移 `dx dy dz`；直接回车默认沿世界竖直方向抬高 10 cm。输出结果还会提示目标点到肩关节的距离，用于初步检查是否超出臂展。

#### 单帧点云采集与工件中心点标定

采用“先拍一张点云，再移动打磨头到工件中心点”的方式。

1. 保持工件、治具和相机位置不动，在拍照位采集一帧点云并留底：

```bash
ros2 topic pub --once /elite_vision_job_cmd \
  std_msgs/msg/Int32 "{data: 1}"
cp /tmp/base.pcd /tmp/base_calib.pcd
```

命令 `1` 会让机械臂到拍照位、采集点云并保存到 `/tmp/base.pcd`。每次执行都会覆盖该文件，因此采集后要立即复制保存。

2. 使用示教器低速移动机械臂，使打磨头尖端轻触工件板面中心，保持机械臂和工件不动，另开终端运行：

```bash
cd ~/Documents/elite_robot_ws
python3 biaoding/grinder_tip.py
```

程序输出的打磨头尖端坐标就是工件中心点，也是三测点中的 `O` 点，用于确定工件平面原点。

3. 用中心点附近的小盒检查尖端是否确实落在点云工件表面：

```bash
python3 biaoding/pcd_box_tool.py /tmp/base_calib.pcd \
  <x-0.02> <y-0.02> <z-0.02> \
  <x+0.02> <y+0.02> <z+0.02>
```

盒内应有成片工件点，点云质心与打磨头尖端坐标偏差建议小于 1 cm。若偏差较大，先检查 `grinder_tip.py` 中的工具长度 `TOOL_Z`、手眼标定和当前 TF，不要直接修改裁剪盒坐标。

4. 如果只是获取工件中心点，到此即可；如果要继续标定打磨视觉裁剪盒，再使用同一帧 `/tmp/base_calib.pcd`：

```bash
# 查看点云密集区域，辅助确定工件大盒
python3 biaoding/pcd_box_tool.py /tmp/base_calib.pcd --hist

# 通过 xmin ymin zmin xmax ymax zmax 试框
python3 biaoding/pcd_box_tool.py /tmp/base_calib.pcd \
  <xmin> <ymin> <zmin> <xmax> <ymax> <zmax>
```

然后根据工件表面再确定 `X`、`Y` 两个测点，填写 `polish_params.yaml` 中的 `target_box_*`、`plane_point_o/x/y` 和 `plane_box_z_range`。完整的单帧裁剪盒标定流程见：

```text
~/Documents/elite_robot_ws/docs/裁剪盒标定_手动单帧法.md
```

### 17.4 打磨安全要求

启动后先让机械臂保持静止、末端不接触工件，等待力传感器完成零点稳定；先用低速运行，建议从 10%–20% 速度开始。确认 TCP、工具 IO、力矩阈值、接触方向和急停后，再执行自动流程。工具上电前先验证 IO 状态，发现力值突变、姿态异常、点云丢失或轨迹偏离时立即停止。

## 18. 两个工作空间的相机和功能对照

当前机器上存在两套深度相机使用方式，启动时要区分：

| 功能链路 | 相机 | 主要话题 | 主要用途 |
|---|---|---|---|
| KYBOT 底盘侧 | RealSense D435 | `/front_camera/camera/...` | KYBOT 视觉、导航或原有 Qwen 感知 |
| Elite 机械臂侧 | 图漾 Percipio | `/camera/color/...`、`/camera/depth/...` | 手眼标定、YOLO/Qwen 抓取、打磨视觉 |

同一台物理相机不要同时启动两个驱动。若需要让 KYBOT 底盘移动并由 Elite 机械臂抓取，建议采用分终端、分工作空间的方式：

```text
KYBOT 终端：source ~/kybot_ws/install/setup.bash
              启动底盘、激光雷达、IMU、导航

Elite 终端：source ~/Documents/elite_robot_ws/install/setup.bash
              启动机械臂、图漾相机、视觉抓取或打磨
```

两套工作空间都存在视觉相关包时，必须确认当前终端的包来源：

```bash
ros2 pkg prefix qwen_vision
ros2 pkg prefix percipio_camera
ros2 topic list | grep -E 'front_camera|camera/color|camera/depth'
```

## 19. 推荐的“移动到位—视觉定位—抓取—放置”流程

如果任务同时使用 KYBOT 底盘和 Elite 机械臂，可按以下顺序组织：

1. 在 KYBOT 终端启动底盘、2D/3D 激光雷达、IMU 和导航，确认 `/odom`、`/scan_fe`、`/velodyne_points` 和导航状态正常。
2. 在 Elite 终端启动 CS66 驱动，确认 `/joint_states`、TCP 位姿、控制器和 `cs66_base_link → cs66_tool0` TF 正常。
3. 启动图漾相机，确认彩色图、深度图、相机内参和手眼标定结果可用。
4. 启动 YOLO 或 Qwen，并通过 `/target_object_pose` 检查目标位置是否合理。
5. 让机械臂到 Ready 或 Home2，确认夹爪状态后再调用抓取服务。
6. 抓取后保持安全姿态，再由 KYBOT 底盘导航到放置点。
7. 底盘停止且周围环境安全后，执行放置动作并回到 Home2。

任何一步的坐标系、深度值、控制器状态或目标位姿异常，都应先停止后续运动，只执行话题/TF/服务检查。

## 20. Elite 工作空间常见问题

### 20.1 机械臂控制器未激活

```bash
ros2 control list_controllers
ros2 control set_controller_state \
  scaled_joint_trajectory_controller active
ros2 topic echo /joint_states --once
```

如果反复被切换或无法激活，检查是否同时启动了 MoveIt、示教器程序、其他 SDK 节点或另一个机械臂驱动。

### 20.2 外部脚本被覆盖

现象通常是机械臂停止响应、运动按钮无效或 `/robot_task_running` 状态异常。先停止其他运动程序，再执行：

```bash
ros2 service call /io_and_status_controller/resend_external_script \
  std_srvs/srv/Trigger "{}"
```

仍未恢复时，按项目启动顺序重新启动驱动，并确认示教器没有运行其他外部控制程序。

### 20.3 图漾相机有设备但没有图像

```bash
ros2 node list | grep percipio
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

检查相机分辨率、设备占用、USB/网口连接和是否有订阅者；视觉程序要求的命名空间是 `/camera/...`，不要误用 KYBOT 的 `/front_camera/...`。

### 20.4 目标位姿不对

依次检查：彩色图和深度图是否对齐 → 深度值单位/范围是否正确 → 相机内参 → ArUco/手眼标定 JSON → `cs66_base_link` 与 `cs66_tool0` TF → `/target_object_pose` 的 frame_id 和数值。

```bash
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /target_object_pose --once
ros2 run tf2_tools view_frames
```

不要通过修改抓取偏置来掩盖错误的标定或错误的坐标系。

### 20.5 YOLO/Qwen 请求不到目标

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /yolo/annotated_image --once
ros2 topic echo /qwen/perception_done --once
ros2 service call /vision_perception/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

检查模型文件、目标类别、手眼 JSON 路径、Qwen 私有 API 配置和当前视觉后端是否一致。

### 20.6 打磨点云或力传感器没有数据

```bash
ros2 topic hz /camera/depth_registered/points
ros2 topic echo /force_torque_sensor_broadcaster/wrench --once
ros2 topic echo /ys_contact_fts_broadcaster/wrench --once
```

确认 `elite_polish_app` 的点云重映射与当前相机驱动实际发布的话题一致，并先检查力传感器是否完成零点稳定。
