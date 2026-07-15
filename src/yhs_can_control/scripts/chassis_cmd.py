#!/usr/bin/env python3
"""
底盘控制命令行工具
用法:
    ros2 run yhs_can_control chassis_cmd.py
进入后输入命令控制底盘的行进、转向和灯光。

注意: 控制运动前必须先按 1 使能底盘，否则运动指令无效！

挡位定义(说明书):
    0=Disable  1=P档  2=R档  3=N档  4=D档
"""

import rclpy
from rclpy.node import Node
from yhs_can_interfaces.msg import CtrlCmd, IoCmd


# 说明书挡位定义
GEAR_DISABLE = 0
GEAR_P = 1
GEAR_R = 2
GEAR_N = 3
GEAR_D = 4

GEAR_NAME = {
    GEAR_DISABLE: "Disable",
    GEAR_P:       "P (驻车)",
    GEAR_R:       "R (后退)",
    GEAR_N:       "N (空挡)",
    GEAR_D:       "D (前进)",
}

# g键循环顺序: N → R → D → P → N
GEAR_CYCLE = [GEAR_N, GEAR_R, GEAR_D, GEAR_P]


class ChassisCmdTool(Node):
    def __init__(self):
        super().__init__('chassis_cmd_tool')

        self.ctrl_pub = self.create_publisher(CtrlCmd, 'ctrl_cmd', 1)
        self.io_pub = self.create_publisher(IoCmd, 'io_cmd', 1)

        self.velocity = 0.0
        self.steering = 0.0
        self.gear = GEAR_N  # 安全起见初始 N 挡

        self.io_enable = False
        self.low_beam = False
        self.high_beam = False
        self.turn_lamp = 0  # 0=off, 1=left, 2=right
        self.brake_lamp = False
        self.clearance_lamp = False
        self.fog_lamp = False
        self.speaker = False

        self._warned_enable = False

        self.print_help()
        self.print_status()

        # ~100Hz 持续发布，满足速度控制 30Hz+ 要求
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        self.publish_ctrl()
        self.publish_io()

    def publish_ctrl(self):
        msg = CtrlCmd()
        msg.ctrl_cmd_gear = self.gear
        msg.ctrl_cmd_velocity = self.velocity
        msg.ctrl_cmd_steering = self.steering
        self.ctrl_pub.publish(msg)

    def publish_io(self):
        msg = IoCmd()
        msg.io_cmd_enable = self.io_enable
        msg.io_cmd_lower_beam_headlamp = self.low_beam
        msg.io_cmd_upper_beam_headlamp = self.high_beam
        msg.io_cmd_turn_lamp = self.turn_lamp
        msg.io_cmd_braking_lamp = self.brake_lamp
        msg.io_cmd_clearance_lamp = self.clearance_lamp
        msg.io_cmd_fog_lamp = self.fog_lamp
        msg.io_cmd_speaker = self.speaker
        msg.io_cmd_dis_charge = False
        self.io_pub.publish(msg)

    def _debug_motion(self, action):
        """打印运动调试信息"""
        print(f"  [DEBUG] {action}: gear={self.gear}({GEAR_NAME.get(self.gear, '?')})"
              f" vel={self.velocity:.2f}m/s steering={self.steering:.1f}°"
              f" enable={self.io_enable}")

    def print_status(self):
        turn_map = {0: "关闭", 1: "左转", 2: "右转"}
        print("\n" + "=" * 55)
        print("  当前状态")
        print("-" * 55)
        print(f"  挡位:     {GEAR_NAME.get(self.gear, str(self.gear))}")
        print(f"  速度:     {self.velocity:.2f} m/s")
        print(f"  转向角:   {self.steering:.1f}°")
        print(f"  使能:     {'✓ 已使能' if self.io_enable else '✗ 未使能 (按1使能，否则运动指令无效！)'}")
        print(f"  近光灯:   {'● 开' if self.low_beam else '○ 关'}")
        print(f"  远光灯:   {'● 开' if self.high_beam else '○ 关'}")
        print(f"  转向灯:   {turn_map.get(self.turn_lamp, str(self.turn_lamp))}")
        print(f"  刹车灯:   {'● 开' if self.brake_lamp else '○ 关'}")
        print(f"  示廓灯:   {'● 开' if self.clearance_lamp else '○ 关'}")
        print(f"  雾灯:     {'● 开' if self.fog_lamp else '○ 关'}")
        print(f"  喇叭:     {'● 开' if self.speaker else '○ 关'}")
        print("=" * 55)
        print("  输入命令 (h 查看帮助, q 退出): ", end="", flush=True)

    def _check_enable(self):
        """检查是否已使能"""
        if not self.io_enable and not self._warned_enable:
            print("\n!!! 警告: 底盘未使能！请先按 1 使能，否则运动指令不会执行 !!!")
            self._warned_enable = True
            return False
        if self.io_enable:
            self._warned_enable = False
        return self.io_enable

    def print_help(self):
        print("""
╔══════════════════════════════════════════════════════╗
║            底盘控制命令行工具                        ║
╠══════════════════════════════════════════════════════╣
║  【重要】控制运动前必须先按 1 使能！                 ║
║                                                      ║
║  运动控制:                                          ║
║    w    前进加速 (+0.1 m/s)                          ║
║    s    减速 (-0.1 m/s)                              ║
║    x    急停 (速度归零)                              ║
║    a    左转 (+5°)                                   ║
║    d    右转 (-5°)                                   ║
║    r    转向回正                                      ║
║    g    切换挡位 (N→R→D→P→N)                         ║
║         D挡(4)+w前进 / R挡(2)+w后退                  ║
║                                                      ║
║  灯光及使能:                                         ║
║    1    使能开关  ← 运动前必须先按！                 ║
║    2    近光灯 开/关                                 ║
║    3    远光灯 开/关                                 ║
║    4    转向灯 切换 (关→左→右→关)                   ║
║    5    刹车灯 开/关                                 ║
║    6    示廓灯 开/关                                 ║
║    7    雾灯 开/关                                   ║
║    8    喇叭 开/关                                   ║
║                                                      ║
║  其他:                                               ║
║    h    显示此帮助                                   ║
║    t    显示当前状态                                 ║
║    q    退出                                         ║
╚══════════════════════════════════════════════════════╝
        """)

    def _cycle_gear(self):
        """循环切换挡位"""
        try:
            idx = GEAR_CYCLE.index(self.gear)
        except ValueError:
            idx = 0
        self.gear = GEAR_CYCLE[(idx + 1) % len(GEAR_CYCLE)]

    def run(self):
        try:
            while rclpy.ok():
                self.print_status()
                cmd = input().strip().lower()
                if not cmd:
                    continue

                if cmd == 'q':
                    self.velocity = 0.0
                    self.steering = 0.0
                    self.gear = GEAR_DISABLE
                    self.io_enable = False
                    self.low_beam = False
                    self.high_beam = False
                    self.turn_lamp = 0
                    self.brake_lamp = False
                    self.clearance_lamp = False
                    self.fog_lamp = False
                    self.speaker = False
                    self.publish_ctrl()
                    self.publish_io()
                    print("已停止所有输出，退出。")
                    break

                elif cmd == 'h':
                    self.print_help()

                elif cmd == 't':
                    self.print_status()

                # ---- 运动控制 ----
                elif cmd == 'w':
                    if self.gear == GEAR_N:
                        print("\n!!! 当前为空挡(N)，请先按 g 切换到 D 挡或 R 挡 !!!")
                    elif self.gear == GEAR_P:
                        print("\n!!! 当前为 P 挡(驻车)，请先按 g 切换挡位 !!!")
                    else:
                        self._check_enable()
                        self.velocity += 0.1
                        self._debug_motion("加速")
                elif cmd == 's':
                    self._check_enable()
                    self.velocity -= 0.1
                    if self.velocity < 0.0:
                        self.velocity = 0.0
                    self._debug_motion("减速")
                elif cmd == 'x':
                    self.velocity = 0.0
                    self._debug_motion("急停")
                elif cmd == 'a':
                    self._check_enable()
                    self.steering += 5.0
                    self._debug_motion("左转")
                elif cmd == 'd':
                    self._check_enable()
                    self.steering -= 5.0
                    self._debug_motion("右转")
                elif cmd == 'r':
                    self.steering = 0.0
                    self._debug_motion("转向回正")
                elif cmd == 'g':
                    self._check_enable()
                    self._cycle_gear()
                    self._debug_motion("换挡")
                    print(f"  → 切换到 {GEAR_NAME[self.gear]}")

                # ---- 灯光控制 ----
                elif cmd == '1':
                    self.io_enable = not self.io_enable
                    if self.io_enable:
                        print("  → 底盘已使能 ✓")
                        self._warned_enable = False
                    else:
                        print("  → 底盘已取消使能 ✗")
                elif cmd == '2':
                    self.low_beam = not self.low_beam
                elif cmd == '3':
                    self.high_beam = not self.high_beam
                elif cmd == '4':
                    self.turn_lamp = (self.turn_lamp + 1) % 3
                elif cmd == '5':
                    self.brake_lamp = not self.brake_lamp
                elif cmd == '6':
                    self.clearance_lamp = not self.clearance_lamp
                elif cmd == '7':
                    self.fog_lamp = not self.fog_lamp
                elif cmd == '8':
                    self.speaker = not self.speaker

                else:
                    print(f"未知命令: '{cmd}'，输入 h 查看帮助")

                # 限制范围
                self.velocity = max(0.0, min(5.0, self.velocity))
                self.steering = max(-45.0, min(45.0, self.steering))

        except KeyboardInterrupt:
            print("\n中断退出。")
        except EOFError:
            print("\nEOF 退出。")


def main():
    rclpy.init()
    tool = ChassisCmdTool()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(tool,), daemon=True)
    spin_thread.start()

    tool.run()

    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
