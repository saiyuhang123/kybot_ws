# 车前感知输出的目标结果
builtin_interfaces/Time stamp

bool detected             # 当前帧是否检测到目标
string cls_name           # 类别名
float32 confidence        # 置信度

float32 distance          # 距离（米），无效时为 0
bool   distance_valid     # 距离是否有效
float32 lateral_offset    # 车前方横向偏移（米，正=右）

bool   moving             # 车是否在运动
bool   stationary_confirm # 停车后多帧确认是否通过
float32 blur              # 画面模糊分
