"""点位白名单加载: 解析 location.yaml (与 RViz 面板保存格式一致).

纯 yaml 解析, 不依赖 ROS, 方便离线单测.
"""

import os
from dataclasses import dataclass, field

import yaml


@dataclass
class Waypoint:
    """一个命名点位: name + pose(map系) + ptz_preset + capture + action."""

    name: str
    pose: dict = field(default_factory=dict)  # x,y,z,qx,qy,qz,qw
    ptz_preset: int = 0       # 0 = 不动云台
    capture: bool = False
    action: str = ''          # ""=无动作  grasp=视觉抓取  place=放置

    def brief(self):
        """给 LLM 看的单行描述."""
        parts = [self.name]
        if self.capture:
            parts.append('到达后拍照')
        if self.action:
            parts.append('到达后执行动作:' + self.action)
        return '、'.join(parts)


def load_waypoints(path):
    """加载点位文件, 返回 Waypoint 列表. 文件不存在/格式错时抛异常."""
    path = os.path.expanduser(path)
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    waypoints = []
    for item in (data or {}).get('waypoints', []):
        name = str(item.get('name', '')).strip()
        if not name:
            continue
        waypoints.append(Waypoint(
            name=name,
            pose=dict(item.get('pose', {}) or {}),
            ptz_preset=int(item.get('ptz_preset', 0) or 0),
            capture=bool(item.get('capture', False)),
            action=str(item.get('action', '') or ''),
        ))
    return waypoints


_CN_DIGIT = {'零': 0, '〇': 0, '一': 1, '二': 2, '两': 2, '三': 3,
             '四': 4, '五': 5, '六': 6, '七': 7, '八': 8, '九': 9}


def normalize_name(name):
    """名称归一化: 中文数字→阿拉伯数字 ("点位一"→"点位1", "点位十二"→"点位12").

    用于白名单模糊匹配: 语音场景 ASR 几乎总是输出中文数字,
    而点位名习惯用阿拉伯数字.
    """
    out = []
    run = ''  # 累积中文数字串
    for ch in str(name):
        if ch in _CN_DIGIT or ch == '十':
            run += ch
            continue
        if run:
            out.append(_cn_run_to_int(run))
            run = ''
        out.append(ch)
    if run:
        out.append(_cn_run_to_int(run))
    return ''.join(out)


def _cn_run_to_int(run):
    """一小段中文数字转整数: 无"十"按位拼接 ("一二三"→123), 有"十"按十进制."""
    if '十' not in run:
        return ''.join(str(_CN_DIGIT[c]) for c in run)
    before, _, after = run.partition('十')
    tens = _CN_DIGIT[before] if len(before) == 1 else 1
    ones = _CN_DIGIT[after] if len(after) == 1 else 0
    return str(tens * 10 + ones)


def find_waypoint(waypoints, name):
    """按名找点位: 先精确匹配, 失败后按归一化名称匹配. 返回 Waypoint 或 None."""
    name = str(name).strip()
    for wp in waypoints:
        if wp.name == name:
            return wp
    norm = normalize_name(name)
    for wp in waypoints:
        if normalize_name(wp.name) == norm:
            return wp
    return None
