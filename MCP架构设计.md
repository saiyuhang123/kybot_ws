# KYBOT + Elite 大模型/MCP 架构设计

> 目标:用 MCP(Model Context Protocol)把两台机器人的 ROS2 能力封装成大模型可调用的工具,
> 实现"语音/文字指令 → LLM 规划 → 工具调用 → 机器人执行"的闭环。
>
> 环境:kybot(Jetson,Ubuntu + ROS2 Humble, kybot_ws)、elite 机械臂(工控机,Ubuntu + ROS2 Humble, elite_robot_ws),
> 两台机器同一局域网(192.168.1.x)。本文档在 Windows 侧编写,部署回 Ubuntu 执行。

---

## 1. 总体架构

```
                    ┌─────────────────────────────────────┐
                    │         用户 (语音 / 文字)            │
                    └──────────────┬──────────────────────┘
                                   │
              AIUI (ASR/TTS, 沿用现有 robot_aiui, 只做语音前端)
                                   │ /voice_text ↑TTS
                    ┌──────────────▼──────────────────────┐
                    │   kybot_brain (编排器, Python)        │
                    │   - MCP client                      │
                    │   - LLM 调用 (Qwen, function call)   │
                    │   - 确认流程 / 多轮上限 / 离线降级      │
                    └──────┬───────────────────┬──────────┘
              MCP/HTTP     │                   │  MCP/HTTP (局域网)
        (本机 127.0.0.1)   │                   │   192.168.1.x:8802
                    ┌──────▼──────┐     ┌──────▼───────────┐
                    │ mcp_kybot   │     │ mcp_elite        │
                    │ _server     │     │ _server          │
                    │ (Jetson)    │     │ (工控机)          │
                    └──────┬──────┘     └──────┬───────────┘
                     rclpy │                   │ rclpy
              ┌────────────┼────────┐   ┌──────┼──────────────┐
              ▼            ▼        ▼   ▼      ▼              ▼
           Nav2      hk_camera  qwen_  机械臂驱动  MoveToPose   夹爪
        /mission/*   OCR/PTZ   vision  /script_    srv         service
                                     sender                 (开合度)
```

要点:

- **每台机器人一个 MCP server**,各自只包装本机 ROS2 接口,跨机通信走 MCP 自己的 HTTP 传输,不依赖 DDS 跨机(避免 ROS_DOMAIN_ID/发现机制的坑)。
- **编排器 kybot_brain 部署在 Jetson 上**(挨着 AIUI 语音链路,延迟最低),同时连本机和远端两个 MCP server。
- LLM 只跟 brain 打交道,永远见不到原始 ROS2 接口;所有危险约束在 MCP server 层强制执行。

## 2. 与现有代码的关系(重要)

`robot_aiui/src/agent_bridge.cpp` 里**已经有一套 skill 机制的 agent 循环**(SkillRegistry、LLM 多轮、
`MAX_LLM_ROUNDS=8`、失败上限,skill 含 speak/navigate_to/pick/place/capture/locate_object 等)。

新架构不是推翻它,而是**换层**:

| 现有 | 新架构 | 处置 |
|------|--------|------|
| AIUI 语音识别/合成 (robot_aiui.cpp) | 不变,继续用 | 保留,退化为纯语音前端 |
| agent_bridge.cpp 的 skill 定义 | 迁为 MCP 工具 | 逻辑平移,接口换成 MCP |
| agent_bridge.cpp 的 LLM 循环 (C++ + curl) | kybot_brain (Python) | 替换;C++ 版留作离线降级备选 |
| qwen_vision 的 /locate_object、/image_description | 包成 MCP 工具 | 不动原节点,只加封装 |

brain 用 Python 重写的理由:MCP 官方 SDK 是 Python/TS,C++ 写 MCP client 没有成熟库;
工具编排迭代快,Python + `--symlink-install` 改完即用。

## 3. 软件包结构(解耦原则:brain 零 ROS 依赖)

新建独立仓库 `robot_mcp/`(不进 kybot_ws/elite_robot_ws,两台机器分别按需部署)。
**核心解耦:`robot_brain` 是纯 Python 包,不 import 任何 ROS 库**——LLM 交互、MCP client、
确认流程全在里面,Windows 开发机上 `pip install -e` 就能连着机器人的 MCP server 调试,
ROS 只出现在两个 MCP server(必须)和一个薄薄的语音桥里。三层各自可独立排 bug:

- MCP server 出问题 → 用 `mcp dev` / Cherry Studio 直接点工具,不碰 LLM;
- brain 出问题 → 在开发机用 CLI 模式复现,不碰 ROS;
- 语音链路出问题 → 只看 adapter 和 aiui,不碰前两层。

```
robot_mcp/
├── robot_brain/               # 纯 Python 包 (pip install -e), 无 ROS 依赖, 部署到 Jetson + 开发机
│   ├── pyproject.toml
│   └── robot_brain/
│       ├── orchestrator.py    # LLM 循环 + MCP client (多 server 汇聚)
│       ├── llm.py             # Qwen API (OpenAI 兼容) 封装
│       ├── config.py          # 配置 (yaml + 环境变量)
│       ├── io_base.py         # ChatIO 抽象: say/confirm/log
│       ├── cli_io.py          # 命令行 IO (开发机调试用)
│       ├── confirm.py         # 危险工具确认策略
│       ├── fallback.py        # 断网关键词降级 (急停直通)
│       ├── prompts.py         # system prompt 构建
│       └── __main__.py        # CLI 入口: python -m robot_brain
├── src/                       # colcon 工作空间 (ROS 相关包都在这)
│   ├── mcp_kybot_server/      # ament_python, 部署到 Jetson, 包装底盘侧接口
│   │   └── mcp_kybot_server/
│   │       ├── server.py      # FastMCP 入口, 注册工具
│   │       ├── ros_client.py  # rclpy 节点 + 服务/话题封装
│   │       ├── safety.py      # 参数校验、限流
│   │       └── waypoints.py   # 读取 location.yaml 点位 (含 name)
│   ├── mcp_elite_server/      # ament_python, 部署到工控机, 包装机械臂侧接口 (P3)
│   └── brain_ros_adapter/     # ament_python, 部署到 Jetson, 唯一依赖 robot_brain 的 ROS 包
│       └── brain_ros_adapter/
│           ├── adapter_node.py # /voice_text → brain → /tts_text
│           └── ros_io.py      # ChatIO 的 ROS 实现 (语音确认)
└── README.md                  # 部署 + 分层调试指南
```

技术选型:

- MCP SDK: 官方 Python SDK `mcp`(FastMCP),传输用 **Streamable HTTP**(kybot `127.0.0.1:8801`,elite `0.0.0.0:8802`)。
  调试期可直接把 server 挂到任意 MCP 客户端(Cherry Studio / Claude Desktop / `mcp dev`)手动点工具,这是白送的调试器。
- LLM: Qwen 系列(dashscope,OpenAI 兼容接口 + tool_calls)。qwen_vision 已验证通。
- ROS 集成:每个 server 内部一个全局 rclpy 节点,`MultiThreadedExecutor` 在后台 daemon 线程 spin;
  工具函数是同步包装(`future.result(timeout)`),Action 用 `ActionClient` 带超时阻塞。
- brain 与 ROS 的唯一接触点是 `brain_ros_adapter`(单向依赖 adapter → brain);
  brain 对机器人说话时直接发 `/tts_text`(aiui 新增的订阅),不经过 MCP,减少一跳。

```python
# ros_client.py 核心模式 (两个 server 通用)
import rclpy, threading
from rclpy.executors import MultiThreadedExecutor

_node = None
def init_node(name):
    global _node
    rclpy.init()
    _node = rclpy.create_node(name)
    executor = MultiThreadedExecutor()
    executor.add_node(_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return _node

def call_srv(client, request, timeout=10.0):
    fut = client.call_async(request)
    done = threading.Event()
    fut.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise TimeoutError(f"{client.srv_name} 调用超时")
    return fut.result()
```

## 4. MCP 工具清单

### 4.1 mcp_kybot_server(底盘侧)

**只读工具**(自动执行,无需确认):

| 工具 | 底层接口 | 说明 |
|------|----------|------|
| `get_robot_pose` | TF `map→base_link` | 返回 x/y/yaw |
| `get_chassis_state` | `/chassis_info_fb` (yhs_can_interfaces) | 电量、速度、故障码 |
| `list_waypoints` | 读 `location/location.yaml` | 返回点位名 + 坐标 + 关联动作 |
| `get_mission_status` | 缓存 `/mission/status` (MissionStatus.msg) | 状态/进度/当前点 |
| `capture_image` | `/hk_camera/capture` (CapturePicture.srv) | 存图返回路径,供 VLM 用 |
| `ocr_read` | `/ocr/recognize` (RecognizeText.srv) | 返回文本+置信度列表 |
| `describe_scene` | `/vision_trigger_capture` + 订阅 `/image_description` | Qwen-VL 场景描述 |
| `ask_about_image` | capture → Qwen-VL HTTP | 带问题看图("仪表读数多少") |
| `locate_object` | `/target_object` 发布 + `/locate_object_sync` (Trigger) | 返回 `/object_position` 3D 坐标 |

**运动工具**(需 armed 状态;标注 ⚠ 的需语音确认):

| 工具 | 底层接口 | 说明 |
|------|----------|------|
| `navigate_to_waypoint` ⚠ | 查点位 → Nav2 `NavigateToPose` action | **只接受点位名**,LLM 不许造坐标 |
| `run_mission` ⚠ | `/hk_camera/run_mission` (RunMission.srv) | 点位名列表 → MissionWaypoint[] |
| `cancel_mission` | `/hk_camera/cancel_mission` | 随时允许 |
| `stop_all_motion` | Nav2 cancel + 底盘速度置 0 | 随时允许,最高优先级 |
| `set_ptz` | `/hk_camera/set_ptz_pose` (SetPTZPose.srv) | pan/tilt/zoom 限幅 |
| `speak` | `/string_to_voice` (StringToVoice.srv) | TTS 播报 |

### 4.2 mcp_elite_server(机械臂侧)

**只读工具**:

| 工具 | 底层接口 | 说明 |
|------|----------|------|
| `get_arm_state` | `/joint_states` + TCP 位姿 | 关节角 + 法兰位姿 |
| `arm_camera_capture` | RealSense 最新帧存图 | 供 VLM 定位 |
| `detect_objects` | YOLO 检测(沿用 elite 侧 YOLO) | 返回类别+像素框 |

**运动工具**:

| 工具 | 底层接口 | 说明 |
|------|----------|------|
| `arm_move_named` | 预定义关节配置 → MoveToPose(joint_target) | `home`/`observe`/`grasp_ready`/`place`,**最安全的首选** |
| `arm_move_joints` ⚠ | `/move_to_pose` (MoveToPose.srv, joint_target) | 6 关节角,逐个校验限位 |
| `arm_move_to_pose` ⚠ | `/move_to_pose` (位姿目标, movej/movel) | 工作空间盒校验,速度封顶 |
| `grasp_object_at` ⚠ | 封装 `visual_grasp_test.py` 已验证链路:目标点 → 本地5维IK → FK校验 → 关节角movej | 只接受基座系坐标;FK 偏差 >2cm/5° 拒绝(沿用现有保护) |
| `gripper_set` | Gripper_4Bros2 service_interfaces (status/gripper_id/copen) | 开合度 0~100% |
| `arm_stop` | cancel MoveIt + `/script_sender/script_command` 发 `stopj()` | 随时允许 |

### 4.3 工具 schema 示例

```json
{
  "name": "navigate_to_waypoint",
  "description": "导航到预定义点位。点位名必须来自 list_waypoints 的结果,禁止自行编造坐标。",
  "inputSchema": {
    "type": "object",
    "properties": {
      "name": {"type": "string", "description": "点位名, 如 '3号仪表'、'充电点'"},
      "wait": {"type": "boolean", "default": true, "description": "true=阻塞到到达/失败"}
    },
    "required": ["name"]
  }
}
```

```json
{
  "name": "grasp_object_at",
  "description": "在机械臂基座系指定坐标执行抓取(内置IK+FK校验+夹爪)。坐标必须来自 detect_objects/locate_object 的实测结果。",
  "inputSchema": {
    "type": "object",
    "properties": {
      "x": {"type": "number", "description": "基座系 x (m), 范围 [0.2, 0.8]"},
      "y": {"type": "number", "description": "基座系 y (m), 范围 [-0.5, 0.5]"},
      "z": {"type": "number", "description": "基座系 z (m), 范围 [0.0, 0.9]"},
      "object_name": {"type": "string", "description": "目标名称, 仅用于播报和日志"}
    },
    "required": ["x", "y", "z"]
  }
}
```

```json
{
  "name": "run_mission",
  "description": "执行巡检任务:按点位依次导航→云台预置位→拍照→(可选)OCR/抓取动作。",
  "inputSchema": {
    "type": "object",
    "properties": {
      "waypoints": {"type": "array", "items": {"type": "string"},
                    "description": "点位名列表, 按执行顺序"},
      "report": {"type": "boolean", "default": true,
                 "description": "结束后汇总结果生成巡检报告"}
    },
    "required": ["waypoints"]
  }
}
```

## 5. 安全护栏(全部在 MCP server 层,LLM 无法绕过)

1. **点位白名单**:导航/巡检只接受 `list_waypoints` 里的名字,坐标解析在 server 内完成。
   LLM 永远接触不到原始坐标 → 根除"幻觉坐标"类事故。
2. **机械臂硬约束**(`mcp_elite_server/safety.py`):
   - `velocity_scaling` 服务端封顶 0.3(MCP 入口),无视 LLM 传参;
   - 关节限位逐个校验,位姿目标落在工作空间盒外直接拒绝;
   - `grasp_object_at` 沿用现有 FK 双校验(位置 2cm / 姿态 5°)。
3. **确认分级**:
   - 只读工具:直接执行;
   - 普通运动(导航/PTZ/命名位姿):armed 状态下直接执行;
   - ⚠ 工具(grasp/徒手 movej/movel):brain 先 TTS 询问"我要执行XX,确认吗",用户语音"确认"后执行,10s 无回应自动取消。
4. **限流与互斥**:运动类工具每分钟 ≤ N 次;mission 运行中拒绝新的导航/mission 请求;机械臂运动与底盘运动互斥(抓东西时底盘必须静止)。
5. **急停独立**:物理急停 + `stop_all_motion`/`arm_stop` 不经过 LLM 决策链;brain 收到"停"类关键词走 fallback 直通,不等 LLM。
6. **审计日志**:每次工具调用记录 `时间/工具/参数/结果/确认人`,落盘 JSONL,出事后可回放。

## 6. kybot_brain 编排器

循环(沿用 agent_bridge 的成熟思路,换 Python):

```
/voice_text(或CLI) → 构建 messages(system prompt + 工具schema + 历史)
  → LLM tool_calls → 逐个调 MCP → 结果回填 → 再调 LLM
  → 无 tool_calls 时输出最终文本 → StringToVoice 播报
上限: 8 轮 / 连续 3 次工具失败即中止并播报原因
```

- **System prompt 要点**:机器人能力描述、点位语义(来自 list_waypoints 动态注入)、
  安全规则("禁止编造坐标"、"抓取前必须先 locate/detect")、回答风格(简短、口语化)。
- **离线降级**(`fallback.py`):云端 API 超时/断网时,切关键词匹配——
  "停下/取消/回去充电/巡检" 等核心指令本地直通,保证现场可用。
  有条件后续在 Jetson 上跑 Qwen2.5-7B 量化版做意图解析兜底。
- **巡检报告**:mission 完成后,brain 汇总各点 capture/ocr/describe 结果,
  让 LLM 生成自然语言报告(TTS 播报 + 存 markdown),这是大模型相对纯 OCR 的增量价值。

## 7. 分阶段实施

| 阶段 | 内容 | 验收标准 | 预估 |
|------|------|----------|------|
| P0 | `mcp_kybot_server` 只读工具 + `mcp dev`/CLI 手动调通 | 不碰 LLM,工具全绿 | 1 周 |
| P1 | brain + Qwen + 运动工具 + 语音闭环(L1) | "去3号点"/"停下"/"回充电点" 语音全程 | 1 周 |
| P2 | 视觉工具链 + 巡检报告(L2) | "看一下仪表读数"→ 拍照→VLM/OCR→播报+报告 | 2 周 |
| P3 | `mcp_elite_server`:命名位姿 → movej/movel → grasp(L4 受限版) | 固定工位"抓那个工件"全流程 | 2~3 周 |
| P4 | 任务链 L3 + 护栏加固 + 现场联调 | "巡一遍A区,异常拍照上报" 条件分支可用 | 3~4 周 |

P0~P2 只动 kybot_ws,可在 Jetson 单机上完整闭环;P3 起才需要双机联调。

## 8. 主要风险

| 风险 | 缓解 |
|------|------|
| LLM 幻觉参数(编造坐标/点位) | 点位白名单 + server 端硬校验,从机制上根除 |
| 机械臂误动作伤人/伤设备 | 速度封顶、命名位姿优先、⚠确认、物理急停独立于链路 |
| 现场网络不稳,云端 LLM 掉线 | 关键词 fallback;核心指令不依赖 LLM |
| 抓取成功率低(物理问题) | P3 限定固定工位;标定流程(easy_handeye2)保持不变 |
| 两机时钟/状态不一致 | 互斥规则放 server 端;时钟同步用 chrony |
| C++ agent_bridge 与 brain 双脑并存冲突 | 切换期 aiui 只开 ASR/TTS,skill 调用全部走 brain;验证后移除旧循环 |
