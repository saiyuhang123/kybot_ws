# 我的华测 TCP 使用说明

## 当前设备配置

```text
设备 IP：192.168.1.164
TCP 端口：9901
启动文件：src/chcnav/launch/demo_2.xml
```

## 1. 加载 ROS2 环境

每个新终端都执行：

```bash
cd /home/nvidia/kybot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果源码有修改或还没有编译：

```bash
colcon build --symlink-install \
  --packages-up-to msg_interfaces chcnav \
  --cmake-args -DDISTRO_ROS=humble

source install/setup.bash
```

## 2. 检查 TCP 网络

```bash
ping 192.168.1.164
nc -vz 192.168.1.164 9901
ss -tnp | grep 9901
```

## 3. 启动 TCP 驱动

```bash
cd /home/nvidia/kybot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch src/chcnav/launch/demo_2.xml
```

看到以下日志，表示节点已经启动并读取了 TCP 参数：

```text
tcp config host[192.168.1.164] port[9901]
node init successed !
```

使用 `Ctrl+C` 停止驱动。

## 4. 查看 ROS2 话题

另开终端并加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/kybot_ws/install/setup.bash
```

查看话题列表：

```bash
ros2 topic list | grep chcnav
```

查看 NMEA 数据：

```bash
ros2 topic echo /chcnav/nmea_sentence
```

查看定位、姿态和速度数据：

```bash
ros2 topic echo /chcnav/devpvt
```

查看 IMU 数据：

```bash
ros2 topic echo /chcnav/devimu
```

查看原始华测协议：

```bash
ros2 topic echo /chcnav/hc_sentence
```

查看话题频率：

```bash
ros2 topic hz /chcnav/nmea_sentence
ros2 topic hz /chcnav/devpvt
ros2 topic hz /chcnav/devimu
```

## 5. 设备协议配置

如果话题存在但没有数据，在设备网页的 `TCP Server 9901` 中确认已开启相应协议：

```text
GPCHC-10HZ
GPGGA-1HZ
HCINSPVATZCB-10HZ
HCRAWIMUB
```

对应关系：

```text
/chcnav/nmea_sentence  -> NMEA、GPCHC、GPGGA
/chcnav/devpvt         -> HCINSPVATZCB
/chcnav/devimu         -> HCRAWIMUB
```
