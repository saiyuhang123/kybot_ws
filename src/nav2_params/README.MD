启动
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/nvidia/Documents/PCD/pgm_yaml/occupancy_map.yaml params_file:=/home/nvidia/Documents/nav2_params/nav2_params.yaml


wz_max = vx_max / min_turning_r        ← 转弯能力不丢失
       = 0.5 / 0.8 = 0.625 ≈ 0.62

steering = atan(wheel_base × wz / vx)  ← 桥接转换
         = atan(0.6 × 0.62 / 0.5)
         = atan(0.744) ≈ 36.6°         ← 在45°限制内 ✓

预测距离 = time_steps × model_dt × vx_max
         = 56 × 0.1 × 0.5 = 2.8m      ← 代价地图需要覆盖


参数	旧值	新值	公式/原因
vx_max	0.2 (你改的0.5)	0.5	翻倍提速
wz_max	0.25	0.62	= 0.5 ÷ 0.8 (vx_max / min_turning_r)
wz_std	0.10	0.25	wz_max × 40%
vx_std	0.05	0.12	vx_max × 25%
ax_max	0.5	1.0	0.5s 内加速到最大
az_max	0.6	1.2	与 wz_max 配套
obstacle_max_range(local)	2.5	3.5	覆盖预测范围 2.8m + 余量
