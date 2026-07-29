# AGENTS.md — KYBOT ROS2 Workspace

## Quick Reference

```bash
# Build (Humble)
colcon build --symlink-install --cmake-args -DDISTRO_ROS=humble

# Build (Release)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble

# Launch full system
source /home/nvidia/kybot_ws/install/setup.bash
ros2 launch kybot_bringup bringup.launch.py

# Launch individual nodes
ros2 launch ocr_node ocr_node.launch.py
ros2 run hk_camera hk_camera_gui
ros2 launch hk_camera hk_camera.launch.py
```

## Architecture

### Workspace Layout

- **ROS2 Humble** on aarch64 (NVIDIA Jetson)
- Source packages in `src/`
- Build artifacts: `build/`, `install/`, `log/` (gitignored)
- No monorepo tooling — each package is independent

### Key Packages

| Package | Type | Description |
|---------|------|-------------|
| `kybot_bringup` | ament_cmake | Unified launch for all nodes |
| `yhs_can_control` | ament_cmake | Chassis control via CAN bus (can1) |
| `yhs_can_interfaces` | ament_cmake | Custom msg/srv for chassis |
| `hi_driver` | ament_cmake | 2D LiDAR driver (hi_ros2) |
| `rslidar_sdk` | ament_cmake | 3D LiDAR driver (RoboSense) |
| `livox_ros_driver2` | ament_cmake | Livox LiDAR driver |
| `rs_to_velodyne` | ament_cmake | Point cloud format conversion |
| `hk_camera` | ament_cmake | Hikvision camera + PTZ + OCR integration |
| `ocr_node` | ament_python | PaddleOCR text recognition |
| `ocr_interfaces` | ament_cmake | OCR msg/srv definitions |
| `wit_ros2_imu` | ament_python | IMU driver (wit) |
| `my_rviz_panel` | ament_cmake | RViz panel + mission executor |
| `FAST_LIO_LOCALIZATION_ROS2` | ament_cmake | LiDAR localization |
| `MK-mid-description-ros2` | ament_cmake | URDF/robot description |
| `pcd2pgm` | ament_cmake | PCD to PGM map conversion |
| `nav2_params` | — | Nav2 configuration (not a package) |
| `robot_location2_ws` | — | EKF config (not a package) |

### Custom Interfaces

**Messages** (`msg/`):
- `hk_camera`: `CameraAlarm`, `MissionStatus`, `MissionWaypoint`
- `yhs_can_interfaces`: `ChassisInfoFb`, `CtrlCmd`, `CtrlFb`, `IoCmd`, `IoFb`, etc.
- `ocr_interfaces`: `OcrDetection`, `OcrResult`
- `hi_driver`: `AreaCom`, `LaserInfo`
- `imu_msg`: `ImuData`

**Services** (`srv/`):
- `hk_camera`: `CapturePicture`, `RunMission`, `SetPTZPose`
- `ocr_interfaces`: `RecognizeText`
- `hi_driver`: `CmdSrv`

### Launch Flow

`bringup.launch.py` starts nodes with staggered delays (2s intervals):
1. CAN bus setup (sudo ip link set can1)
2. yhs_can_control (chassis)
3. cmd_vel_bridge (Nav2 → chassis)
4. hi_ros2 (2D LiDAR)
5. rslidar_sdk (3D LiDAR)
6. rs_to_velodyne (point cloud conversion)
7. robot_state_publisher (URDF)
8. wit_ros2_imu (IMU, separate terminal)
9. EKF (odom fusion)
10. FAST_LIO localization (separate terminal)
11. OCR + camera (optional, `use_ocr:=true`)

### TF Tree

```
map → odom → base_link → laser_fe (2D LiDAR)
                       → velodyne (3D LiDAR)
```

Static transforms defined in `bringup.launch.py`.

## Build Notes

- Always `source install/setup.bash` before running nodes
- `--symlink-install` is required for Python packages to pick up changes without rebuild
- Livox-SDK2 has its own CMake build in `src/Livox-SDK2/` (not a ROS package)
- `hk_camera` requires Qt5 and Hikvision SDK (libs in `src/hk_camera/lib/`)
- `ocr_node` requires PaddleOCR and GPU (`use_gpu:=true` by default)

## Hardware

- **Chassis**: YHS Ackermann via CAN1 (500kbps)
- **2D LiDAR**: Hins at 192.168.1.88:8080 (TCP)
- **3D LiDAR**: RoboSense (rslidar_sdk) or Livox MID360
- **Camera**: Hikvision at 192.168.1.64:8000 (admin/a1234567)
- **IMU**: WitMotion on /dev/ttyUSB0 (921600 baud)
- **Wheel base**: 0.6m (configurable in `cfg.yaml`)

## Nav2

- Parameters: `src/nav2_params/nav2_params.yaml`
- Behavior tree XML: `/home/nvidia/Documents/nav2_params/*.xml`
- EKF config: `src/robot_location2_ws/param/ekf_config.yaml`
- Waypoint following: `nav2_msgs/action/FollowWaypoints`

## Mission Executor

`my_rviz_panel/mission_executor` provides:
- `/mission/run` service (accepts waypoints)
- `/mission/cancel` service
- `/mission/status` topic
- Actions: navigate → PTZ preset → capture → custom actions

## Testing

No test suite is configured. Packages have `test_depend` entries but no test files.

## Conventions

- C++17 standard
- Python packages use `ament_python` build type
- C++ packages use `ament_cmake` build type
- Custom interfaces use `rosidl_default_generators`
- Launch files are Python (`.launch.py`), not XML
- Parameters loaded from YAML in `share/<pkg>/params/`
