启动命令
ros2 launch kybot_bringup bringup.launch.py

编译
colcon build --symlink-install --cmake-args -DDISTRO_ROS=humble

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble


ros2 launch ocr_node ocr_node.launch.py

ros2 run hk_camera hk_camera_gui


# 1. 启动 Nav2
ros2 launch nav2_bringup navigation_launch.py

# 2. 启动海康相机 (headless)
ros2 run hk_camera hk_camera_node

# 3. 启动任务执行器
ros2 launch my_rviz_panel mission_executor.launch.py

# 4. 打开 RViz, 添加 my_rviz_panel 插件

