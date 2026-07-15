四个文件的具体行号：

transform_fusion.py — 核心修复
行	改动
26-27	变量名区分 cur_odom_fastlio / cur_map_to_odom_fl
29-31	新增 tf2_ros.Buffer + TransformListener，拿 EKF 的 tf
61-66	新增 tf_to_mat()，把 tf 消息转 4×4 矩阵
75-78	T_map_to_body = T_map_to_odom_fl @ T_odom_fl_to_body 全局位姿
80-87	lookup_transform('odom','base_link') 查 EKF 的局部里程计
89-93	T_map_to_odom = T_map_to_body @ inv(T_odom_ekf_to_body) 坐标系对齐
global_localization.py
行	改动
50	新增 self.cur_scan_odom = None
107	删掉 msg.header.stamp = self.get_clock().now()
114-115	self.cur_scan_odom = copy.deepcopy(self.cur_odom) 原子配对
125	matched_odom = self.cur_scan_odom 用对齐的 odom
ekf_config.yaml
行	改动
23	odom0: x,y 改为 true
24	odom0: yaw 改为 true
47	imu0: yaw 改为 false
55	imu0_relative: false
velodyne_test.yaml
行	改动
5	freq_localization: 1.0 → 5.0
6	localization_th: 0.9 → 0.5
32	time_sync_en: false → true
47	b_gyr_cov: 0.0001 → 0.001
其中 transform_fusion.py 第 89-91 行是整个修复的灵魂：


# [89] 用 EKF 的 odom 定义来计算 map->odom
# [90] if T_odom_ekf_to_body is not None:
# [91]     T_map_to_odom = T_map_to_body @ np.linalg.inv(T_odom_ekf_to_body)
全局位姿真值 ÷ 局部里程计 = 偏移量 → 发布为 map→odom。