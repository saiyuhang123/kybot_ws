# RS to Velodyne (ROS2)

基于https://github.com/HViktorTsoi/rs_to_velodyne 改造的ros2版本
将 Robosense（速腾聚创）雷达点云转换为 Velodyne 格式的 ROS2 节点。转换后的点云可直接用于下游算法：FAST-LIO2、LIO-SAM、LOAM、LeGO-LOAM 等。

## 支持格式

| 输入 | 输出 | 支持型号 |
|---|---|---|
| XYZIRT | XYZIRT | RS-Helios / RS-16 / RS-32 / RS-Ruby / RS-BP |
| XYZIRT | XYZIR | 同上 |
| XYZIRT | XYZI | 同上 |
| XYZI | XYZIR | RS-16 / RS-Ruby |

## 编译

```bash
source /opt/ros/humble/setup.bash
cd <你的workspace>
colcon build --packages-select rs_to_velodyne --symlink-install
source install/setup.bash
```

## 使用

默认 XYZIRT → XYZIRT，直接运行即可：

```bash
ros2 run rs_to_velodyne rs_to_velodyne
```

其他格式组合：

```bash
# XYZIRT → XYZIR（不带时间戳）
ros2 run rs_to_velodyne rs_to_velodyne --ros-args -p output_type:=XYZIR

# XYZIRT → XYZI（只要 xyz+intensity）
ros2 run rs_to_velodyne rs_to_velodyne --ros-args -p output_type:=XYZI

# XYZI → XYZIR
ros2 run rs_to_velodyne rs_to_velodyne --ros-args -p input_type:=XYZI -p output_type:=XYZIR
```

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/rslidar_points` | `sensor_msgs/msg/PointCloud2` |
| 发布 | `/velodyne_points` | `sensor_msgs/msg/PointCloud2` |

输出 `frame_id` 固定为 `velodyne`，时间戳使用 ROS2 系统时间（与 IMU 对齐），`time` 字段为帧内相对时间（秒）。

## 配合 FAST-LIO2 使用

### YAML 关键配置

```yaml
/laser_mapping:
  ros__parameters:
    common:
      lid_topic: "/velodyne_points"
      imu_topic: "/imu/data"

    preprocess:
      lidar_type: 2             # 2 = Velodyne 模式
      scan_line: 32             # 按你的雷达线数修改
      timestamp_unit: 0         # 必须为 0 (SEC)
      blind: 0.3
      scan_rate: 10
      feature_extract_enable: true
```

> **`timestamp_unit: 0` 必须配**，因为本节点输出的 `time` 单位是秒。设错会导致 IMU 时间同步失败，报 "No point, skip this scan!"。

### 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `No point, skip this scan!` | `timestamp_unit` ≠ 0 | 改为 `0` |
| `Too few input point cloud!` | 点太少 / 时间窗口太小 | 检查 `scan_line`、`blind` |
| `Failed to find match for field '...'` | PCL 字段匹配问题 | 本节点已内部处理，不会出现 |
| `process has died` / malloc 错误 | FAST-LIO2 内存问题 | 检查 IMU 数据频率、extrinsic 参数 |
| 建图消失 | FAST-LIO2 设计如此（局部地图） | 正常，调大 `cube_len` 可保留更多 |

## 实现说明

与原始 ROS1 版本的核心差异：

- **构建系统**：catkin → ament_cmake
- **序列化**：不再依赖 `pcl::fromROSMsg` / `pcl::toROSMsg`，直接从 PointCloud2 原始 buffer 读写，避免 PCL 结构体对齐问题
- **时间戳**：输出 header.stamp 使用 ROS2 系统时间，解决速腾雷达设备时间与 IMU 系统时间不一致的问题
- **参数**：ROS 命令行参数改为 ROS2 parameters
