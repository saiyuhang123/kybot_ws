# trash_mission

巡检自动抓取：车前 D435 实时检测与测距。

## M1：实时检测（不训练）

```bash
cd ~/kybot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch trash_mission trash_mission.launch.py
```

会启动：

- 车前 D435（命名空间 `front_camera`，彩色 640x480 + 对齐深度）
- `front_perception_node`：YOLO-World 实时检测，弹出检测画面

启动参数：

```bash
ros2 launch trash_mission trash_mission.launch.py \
    prompts:="iced tea bottle,bottle" show:=true
```

运行时切换提示词：

```bash
ros2 topic pub /trash/target_class std_msgs/msg/String \
    "{data: 'iced tea bottle,bottle'}"
```

画面按键：

- `q` 退出
- `b` 切到 `iced tea bottle,bottle`
- `a` 切到全部类别

话题：

- `/trash/annotated_image`：画框后的图像
- `/trash/target`：结构化结果（`trash_mission_interfaces/msg/TrashTarget`）
- `/trash/detection`：JSON 字符串调试结果

`TrashTarget` 字段：

- `detected`：是否检测到目标
- `cls_name` / `confidence`：类别和置信度
- `distance` / `distance_valid`：距离（米）
- `lateral_offset`：横向偏移（米，正=右）
- `moving`：车是否在运动（订阅 `/odom` 判断）
- `stationary_confirm`：停稳后多帧距离确认是否通过

查看：

```bash
ros2 topic echo /trash/target --once
```

常用参数（通过 `--ros-args -p` 覆盖）：

- `confirm_frames=5`：停车确认所需帧数
- `confirm_tol=0.05`：确认距离波动容忍（米）
- `stationary_speed_threshold=0.05`：静止速度阈值（m/s）
- `min_valid_depth_ratio=0.5`：检测框内有效深度比例下限

注意：D435 是独占设备，运行本 launch 前需要先关闭 `realsense-viewer`。
