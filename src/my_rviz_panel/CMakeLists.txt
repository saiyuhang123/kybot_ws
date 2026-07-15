cmake_minimum_required(VERSION 3.8)
project(my_rviz_panel)

# C++ 17 标准支持
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# 开启 Qt 编译支持 (MOC 编译器)
set(CMAKE_AUTOMOC ON)

# 寻找 Qt 库 (这里以 ROS 2 Humble 默认的 Qt5 为例，若您使用的是 Qt6 可以替换为 Qt6)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 头文件搜索路径
include_directories(include)

# 创建共享库 (Shared Library)
add_library(${PROJECT_NAME} SHARED
  src/my_panel.cpp
  include/my_rviz_panel/my_panel.hpp
)

# 链接 Qt 库
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# 链接 ROS 2 依赖项
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rviz_common
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# 安装规则
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_package()