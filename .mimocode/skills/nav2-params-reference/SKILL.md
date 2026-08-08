---
name: nav2-params-reference
description: "Nav2 Humble 参数速查卡。覆盖 controller_server、planner_server、bt_navigator、costmap 常用调优参数。用户常说'别改告诉我我自己来'，所以本技能以解释和建议为主。"
---

# Nav2 参数速查卡

本项目 Nav2 参数文件：`src/nav2_params/nav2_params.yaml`

## Controller Server（局部控制器）

### RegulatedPurePursuit / MPPI

| 参数 | 作用 | 典型值 | 调优方向 |
|------|------|--------|----------|
| `controller_frequency` | 控制回路频率 (Hz) | 20 | 增大 → 响应更快但 CPU 更高 |
| `desired_linear_vel` | 期望线速度 (m/s) | 0.3 | 减小 → 更稳但更慢 |
| `max_linear_vel` | 最大线速度 | 0.5 | 安全上限 |
| `max_angular_vel` | 最大角速度 (rad/s) | 1.0 | 阿克曼车型可能无效 |
| `transform_tolerance` | TF 变换容忍时间 (s) | 0.2 | 增大 → 减少 TF 超时警告 |
| `min_lookahead_dist` | 最小前瞻距离 (m) | 0.3 | 增大 → 轨迹更平滑 |
| `max_lookahead_dist` | 最大前瞻距离 (m) | 0.9 | 增大 → 转弯更提前 |
| `lookahead_time` | 前瞻时间 (s) | 1.5 | MPPI 使用 |

### Goal Checker

| 参数 | 作用 | 典型值 |
|------|------|--------|
| `xy_goal_tolerance` | 到达目标的 XY 容差 (m) | 0.15 |
| `yaw_goal_tolerance` | 到达目标的偏航容差 (rad) | 0.25 |

## Planner Server（全局规划器）

| 参数 | 作用 | 典型值 |
|------|------|--------|
| `planner_frequency` | 全局规划频率 (Hz) | 2.0 |
| `expected_planner_frequency` | 期望规划频率 | 2.0 |
| `use_astar` | true=A*，false=Dijkstra | false |

## Costmap（代价地图）

### Global Costmap

| 参数 | 作用 | 典型值 | 注意 |
|------|------|--------|------|
| `inflation_layer.inflation_radius` | 障碍物膨胀半径 (m) | 0.55 | 增大 → 更安全但通道变窄 |
| `inflation_layer.cost_scaling_factor` | 代价衰减因子 | 10.0 | 减小 → 膨胀区域代价更高 |
| `obstacle_layer.obstacle_range` | 检测障碍物最大距离 (m) | 2.5 | |
| `obstacle_layer.raytrace_range` | 射线追踪清除距离 (m) | 3.0 | |
| `update_frequency` | 更新频率 (Hz) | 5.0 | |

### Local Costmap

| 参数 | 作用 | 典型值 | 注意 |
|------|------|--------|------|
| `width` / `height` | 局部代价地图尺寸 (m) | 3.0 | 增大 → 看到更远的障碍 |
| `resolution` | 栅格分辨率 (m/cell) | 0.05 | |
| `inflation_layer.inflation_radius` | 局部膨胀半径 | 0.3 | 比全局小，允许更贴近障碍 |
| `rolling_window` | 是否滚动窗口 | true | |

## BT Navigator（行为树）

| 参数 | 作用 |
|------|------|
| `default_bt_xml_filename` | 行为树 XML 文件路径 |
| `plugin_lib_names` | 加载的 BT 插件列表 |

## 常见问题速查

| 现象 | 可能原因 | 检查参数 |
|------|----------|----------|
| 全局路径穿过障碍 | `inflation_radius` 太小 | global_costmap inflation |
| 到目标点反复前进后退 | `xy_goal_tolerance` 太小，或 controller 震荡 | goal_checker, controller gains |
| 局部代价地图白色（无数据） | obstacle_layer 传感器话题未连接 | `observation_sources` 配置 |
| 规划频率太高/太低 | `planner_frequency` | planner_server |
| 转弯时失控 | 速度或前瞻距离不匹配 | `desired_linear_vel`, `lookahead` |
| TF 超时警告 | `transform_tolerance` 太小 | controller_server |
| 无法通过窄道 | `inflation_radius` 太大 | 两个 costmap 的 inflation |

## 阿克曼车型特别注意

- `relative_move` mode 1（横移）对阿克曼无效，只能用 mode 0（前进/后退）和 mode 2（旋转）
- Nav2 先导航到附近，再用 `relative_move` 服务做最后几十厘米的精确靠近
- 用户倾向保守的膨胀系数（安全优先），代价是无法精准停到目标点
