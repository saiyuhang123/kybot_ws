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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hk_camera REQUIRED)
find_package(yaml-cpp REQUIRED)

# 开启 Qt 编译支持 (MOC 编译器)
set(CMAKE_AUTOMOC ON)

# 寻找 Qt 库
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 头文件搜索路径
include_directories(include)

# ---- 1. RViz Panel 插件 (共享库) ----
add_library(${PROJECT_NAME} SHARED
  src/my_panel.cpp
  include/my_rviz_panel/my_panel.hpp
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rviz_common
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_msgs
  std_srvs
  hk_camera
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# ---- 2. Mission Executor 节点 (独立可执行) ----
add_executable(mission_executor
  src/mission_executor.cpp
  include/my_rviz_panel/mission_executor.hpp
)
ament_target_dependencies(mission_executor
  rclcpp
  rclcpp_action
  nav2_msgs
  std_srvs
  hk_camera
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# ---- 安装规则 ----
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME})
ament_package()
