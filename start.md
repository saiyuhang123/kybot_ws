启动命令
ros2 launch kybot_bringup bringup.launch.py

编译
colcon build --symlink-install --cmake-args -DDISTRO_ROS=humble

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble


ros2 launch ocr_node ocr_node.launch.py

ros2 run hk_camera hk_camera_gui
