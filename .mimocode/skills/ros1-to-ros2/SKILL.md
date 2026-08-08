---
name: ros1-to-ros2
description: "ROS1 catkin 包迁移到 ROS2 Humble ament 的检查清单和转换流程。适用于 C++ 和 Python 包。"
---

# ROS1 → ROS2 包迁移

将 ROS1 catkin 包迁移到 ROS2 Humble (ament_cmake / ament_python)。

## 第一步：分析源包结构

1. 读取 `package.xml` — 记录依赖、消息/服务定义
2. 读取 `CMakeLists.txt` 或 `setup.py` — 记录构建目标
3. 列出所有 `.msg`、`.srv`、`.action` 文件
4. 列出所有节点源文件（`.cpp`、`.py`）
5. 列出 launch 文件（`.launch` XML）

## 第二步：迁移 package.xml

- `format` 2 → 3
- `<buildtool_depend>catkin` → `<buildtool_depend>ament_cmake`（或 `ament_python`）
- `<depend>roscpp` → `<depend>rclcpp`
- `<depend>rospy` → `<depend>rclpy`
- `<depend>std_msgs` → `<depend>std_msgs`（不变，但用 `rosidl` 生成）
- `<depend>message_generation` → `<member_of_group>rosidl_interface_packages`
- `<depend>message_runtime` → `<depend>rosidl_default_runtime`
- 移除 `<export><build_type>catkin</build_type></export>`
- 添加 `<export><build_type>ament_cmake</build_type></export>`

## 第三步：迁移 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# ... 其他依赖

# 如果有 msg/srv
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMsg.msg"
  "srv/MySrv.srv"
)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# 如果节点使用自定义接口
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(my_node ${cpp_typesupport_target})

install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
```

## 第四步：迁移 C++ 源码

| ROS1 | ROS2 |
|------|------|
| `#include "ros/ros.h"` | `#include "rclcpp/rclcpp.hpp"` |
| `ros::NodeHandle nh` | `auto node = rclcpp::Node::make_shared("name")` |
| `ros::Publisher pub = nh.advertise<T>("topic", 10)` | `auto pub = node->create_publisher<T>("topic", 10)` |
| `pub.publish(msg)` | `pub->publish(msg)` |
| `ros::Subscriber sub = nh.subscribe("topic", 10, cb)` | `auto sub = node->create_subscription<T>("topic", 10, cb)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::Rate rate(10)` | `rclcpp::Rate rate(10)` |
| `ROS_INFO("...")` | `RCLCPP_INFO(node->get_logger(), "...")` |
| `ros::Time::now()` | `node->now()` |
| `ros::ServiceServer srv = nh.advertiseService(...)` | `auto srv = node->create_service<T>(...)` |
| `ros::ServiceClient client = nh.serviceClient<T>(...)` | `auto client = node->create_client<T>(...)` |
| `#include <cv_bridge/cv_bridge.h>` | `#include <cv_bridge/cv_bridge.hpp>`（Humble 用 `.h` 也行） |
| `#include <tf2_geometry_msgs/tf2_geometry_msgs.h>` | `#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>` |

## 第五步：迁移 Python 源码

| ROS1 | ROS2 |
|------|------|
| `import rospy` | `import rclpy` |
| `from rospy import init_node` | `rclpy.init()` |
| `rospy.Publisher('topic', T, queue_size=10)` | `node.create_publisher(T, 'topic', 10)` |
| `rospy.Subscriber('topic', T, callback)` | `node.create_subscription(T, 'topic', callback, 10)` |
| `rospy.spin()` | `rclpy.spin(node)` |
| `rospy.loginfo("...")` | `node.get_logger().info("...")` |
| `rospy.Rate(10)` | `node.create_rate(10)` |
| `rospy.Time.now()` | `node.get_clock().now()` |

## 第六步：迁移 Launch 文件

ROS1 XML `.launch` → ROS2 Python `.launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{'param_name': 'value'}],
            remappings=[('/old_topic', '/new_topic')],
        ),
    ])
```

## 第七步：接口命名规则

- `.msg` 文件名：CamelCase（`CamBase.msg` 不是 `cam_base.msg`）
- `.srv` 文件名：CamelCase（`PickAndPlace.srv`）
- 字段名：小写开头（`class_name` 不是 `Class`）
- `.srv` 分隔符：必须用 `---`（不能用注释替代）

## 第八步：常见坑

- `rosidl_generate_interfaces` 和 `add_executable` 不能用同一个 target name → 可执行文件加 `_node` 后缀
- `ament_target_dependencies` 内部用 plain-style `target_link_libraries`，不要混用 keyword (`PRIVATE`) 和 plain style
- `rclcpp::Node` 已继承 `enable_shared_from_this`，子类不要重复继承
- `declare_parameter` 默认值不能是 `std::map`
- NumPy 版本：`cv_bridge` 需要 NumPy <2，pip 安装 ML 框架时注意 pin

## 验证

1. `colcon build --symlink-install --cmake-args -DDISTRO_ROS=humble --packages-select <pkg>`
2. `source install/setup.bash`
3. `ros2 run <pkg> <node>` 或 `ros2 launch <pkg> <launch>.launch.py`
