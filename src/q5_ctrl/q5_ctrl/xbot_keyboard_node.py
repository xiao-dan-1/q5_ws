#!/usr/bin/env python3
"""xbot 键盘步进节点：用 RobotEra SDK API（RobotController）手动步进身体 + 手臂关节。

正规 ROS 节点（继承 RobotController，即 rclpy.Node）。对齐 q5_ctrl/keyboard_node.py 的
手臂 7 自由度 + 键位。跑在机器人上（RobotController 依赖 xbot_sdk_api，只在机器人有）。

限位：身体用 MANUAL_BODY_EXTRA_LIMITS（手册第 4 页）；手臂用 MANUAL_ARM_LIMITS（与真机一致）。

用法：ros2 run q5_ctrl xbot_keyboard_node
  身体  q/e 髋 | a/d 膝 | z/c 踝 | w/s 头
  手臂  f/g 选左/右臂，然后：
        肩  y/u 俯仰  i/o 滚  p/[ 偏航
        肘  h/j 俯仰  k/l 偏航
        腕  n/m 俯仰  ,/. 滚
  其它  t 变速 | 0 打印 | x 退出
"""
from __future__ import annotations

import math
import select
import sys
import termios
import time

sys.path.insert(0, "/home/developer/xbot_sdk_api")  # RobotController 只在机器人上

import rclpy  # noqa: E402
from xbot_common_interfaces.msg import HybridJointCommand  # noqa: E402
from robot_controller import RobotController  # noqa: E402
from .q5_joint_limits import (  # noqa: E402
    ARM_JOINT_NAMES,
    BODY_EXTRA_JOINT_NAMES,
    BODY_JOINT_NAMES,
    HAND_JOINT_NAMES,
    MANUAL_ARM_LIMITS,
    MANUAL_BODY_EXTRA_LIMITS,
)

BODY_TOPIC = "/wr1_controller/commands"
HAND_TOPIC = "/hand_controller/commands"
KP, KD, FF = 100.0, 2.0, 0.0
HAND_KP, HAND_KD, HAND_FF = 100.0, 0.0, 350.0
SPEED_DEG_S = 1.5
SPEED_FAST_DEG_S = 6.0
HOLD_MS = 150
EFFORT_LIMIT = 150.0

STEP_JOINTS = ("hip_joint", "knee_joint", "ankle_joint", "neck_pitch_joint")
STEP_LIMITS = {name: MANUAL_BODY_EXTRA_LIMITS[name] for name in STEP_JOINTS}
BODY_KEYS = {
    "q": ("hip_joint", -1), "e": ("hip_joint", +1),
    "a": ("knee_joint", -1), "d": ("knee_joint", +1),
    "z": ("ankle_joint", -1), "c": ("ankle_joint", +1),
    "w": ("neck_pitch_joint", +1), "s": ("neck_pitch_joint", -1),
}

ARM_AXES = {
    "shoulder_pitch": ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint"),
    "shoulder_roll": ("left_shoulder_roll_joint", "right_shoulder_roll_joint"),
    "arm_yaw": ("left_arm_yaw_joint", "right_arm_yaw_joint"),
    "elbow_pitch": ("left_elbow_pitch_joint", "right_elbow_pitch_joint"),
    "elbow_yaw": ("left_elbow_yaw_joint", "right_elbow_yaw_joint"),
    "wrist_pitch": ("left_wrist_pitch_joint", "right_wrist_pitch_joint"),
    "wrist_roll": ("left_wrist_roll_joint", "right_wrist_roll_joint"),
}
ARM_KEYS = {
    # 肩（顶行）: y/u 俯仰, i/o 滚, p/[ 偏航
    "y": ("shoulder_pitch", -1), "u": ("shoulder_pitch", +1),
    "i": ("shoulder_roll", -1),  "o": ("shoulder_roll", +1),
    "p": ("arm_yaw", -1),        "[": ("arm_yaw", +1),
    # 肘（中行）: h/j 俯仰, k/l 偏航
    "h": ("elbow_pitch", -1),    "j": ("elbow_pitch", +1),
    "k": ("elbow_yaw", -1),      "l": ("elbow_yaw", +1),
    # 腕（底行）: n/m 俯仰, ,/. 滚
    "n": ("wrist_pitch", -1),    "m": ("wrist_pitch", +1),
    ",": ("wrist_roll", -1),     ".": ("wrist_roll", +1),
}
ARM_LIMITS = {}
for _ln, _rn in ARM_AXES.values():
    ARM_LIMITS[_ln] = MANUAL_ARM_LIMITS[_ln]
    ARM_LIMITS[_rn] = MANUAL_ARM_LIMITS[_rn]


def getch(timeout=0.01):
    return sys.stdin.read(1) if select.select([sys.stdin], [], [], timeout)[0] else None


class XbotKeyboardNode(RobotController):
    def __init__(self):
        super().__init__("xbot_keyboard_node")
        self._pub = self.create_publisher(HybridJointCommand, BODY_TOPIC, 10)
        self._hand_pub = self.create_publisher(HybridJointCommand, HAND_TOPIC, 10)

        # 等第一批 /joint_states（最多 5s）
        state = None
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            state = self.get_joint_states()
            if state is not None:
                break
        if state is None:
            raise SystemExit("[!] 拿不到 /joint_states，中止。")

        pos = {n: p for n, p in zip(state.name, state.position)}
        # 目标值：身体 4 + 手臂 14，从当前反馈初始化；waist_yaw/neck_yaw 保持反馈
        self._targets = {n: pos.get(n, 0.0) for n in STEP_JOINTS}
        for ln, rn in ARM_AXES.values():
            self._targets[ln] = pos.get(ln, 0.0)
            self._targets[rn] = pos.get(rn, 0.0)
        self._selected_arm = "left"

    def cur(self):
        js = self.get_joint_states()
        if js is None:
            return {}, {}
        return ({n: p for n, p in zip(js.name, js.position)},
                {n: e for n, e in zip(js.name, js.effort)})

    def _limit_of(self, name):
        return STEP_LIMITS.get(name, ARM_LIMITS.get(name, (-1.0, 1.0)))

    def _resolve(self, active):
        kind, key, _d = active
        if kind == "body":
            return key
        ln, rn = ARM_AXES[key]
        return ln if self._selected_arm == "left" else rn

    def publish_body(self):
        s, _ = self.cur()
        if any(n not in s for n in BODY_JOINT_NAMES):
            return  # 反馈不全不发布（缺失关节按 0 发出去会失控）
        pos = [self._targets.get(n, s.get(n, 0.0)) for n in ARM_JOINT_NAMES] + [
            self._targets.get(n, s.get(n, 0.0)) for n in BODY_EXTRA_JOINT_NAMES
        ]
        msg = HybridJointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = list(BODY_JOINT_NAMES)
        msg.position = [float(v) for v in pos]
        msg.velocity = [0.0] * len(BODY_JOINT_NAMES)
        msg.feedforward = [FF] * len(BODY_JOINT_NAMES)
        msg.kp = [KP] * len(BODY_JOINT_NAMES)
        msg.kd = [KD] * len(BODY_JOINT_NAMES)
        self._pub.publish(msg)

    def publish_hand(self):
        s, _ = self.cur()
        if any(n not in s for n in HAND_JOINT_NAMES):
            return  # 手反馈不全不发
        msg = HybridJointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = list(HAND_JOINT_NAMES)
        msg.position = [float(s.get(n, 0.0)) for n in HAND_JOINT_NAMES]
        msg.velocity = [0.0] * len(HAND_JOINT_NAMES)
        msg.feedforward = [HAND_FF] * len(HAND_JOINT_NAMES)
        msg.kp = [HAND_KP] * len(HAND_JOINT_NAMES)
        msg.kd = [HAND_KD] * len(HAND_JOINT_NAMES)
        self._hand_pub.publish(msg)

    def show(self, tag):
        s, _ = self.cur()
        pitch = math.degrees(s.get("ankle_joint", 0) + s.get("knee_joint", 0)
                             + s.get("hip_joint", 0) + s.get("neck_pitch_joint", 0))
        print("  %-3s hip=%6.1f° knee=%6.1f° ankle=%6.1f° neck=%6.1f° -> 头pitch=%6.1f°" % (
            tag, math.degrees(s.get("hip_joint", 0)), math.degrees(s.get("knee_joint", 0)),
            math.degrees(s.get("ankle_joint", 0)), math.degrees(s.get("neck_pitch_joint", 0)), pitch))
        idx = 0 if self._selected_arm == "left" else 1
        print("       %s 肩俯仰=%6.1f° 肘俯仰=%6.1f°" % (
            "左" if self._selected_arm == "left" else "右",
            math.degrees(s.get(ARM_AXES["shoulder_pitch"][idx], 0)),
            math.degrees(s.get(ARM_AXES["elbow_pitch"][idx], 0))))

    def run(self):
        active = None
        last_key_t = 0.0
        moving = False
        speed = SPEED_DEG_S
        last = time.monotonic()
        last_print = time.monotonic()
        last_eff_chk = time.monotonic()

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        new = list(old)
        new[3] = new[3] & ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, new)

        print("=" * 72)
        print("xbot 键盘步进节点  [RobotController]  kp=%.0f kd=%.0f" % (KP, KD))
        print("  身体  q/e 髋  a/d 膝  z/c 踝  w/s 头")
        print("  手臂  f/g 选臂(当前 左)，再：")
        print("        肩: y/u 俯仰   i/o 滚   p/[ 偏航")
        print("        肘: h/j 俯仰   k/l 偏航")
        print("        腕: n/m 俯仰   ,/. 滚")
        print("  t 变速 %.1f↔%.0f°/s   0 打印   x 退出" % (SPEED_DEG_S, SPEED_FAST_DEG_S))
        print("  按住=持续运动，松开=立即停")
        print("=" * 72)
        self.show("IN ")

        try:
            while True:
                now = time.monotonic()
                dt = min(max(now - last, 0.0), 0.05)
                last = now
                rclpy.spin_once(self, timeout_sec=0.001)

                ch = getch(0.0)
                while ch is not None:
                    if ch in ("x", "\x03"):
                        raise KeyboardInterrupt
                    if ch == "0":
                        self.show("P  ")
                    elif ch == "t":
                        speed = SPEED_FAST_DEG_S if speed <= SPEED_DEG_S else SPEED_DEG_S
                        print("  速度: %.1f°/s" % speed)
                    elif ch == "f":
                        self._selected_arm = "left"
                        self.show("L  ")
                    elif ch == "g":
                        self._selected_arm = "right"
                        self.show("R  ")
                    elif ch in BODY_KEYS:
                        active = ("body",) + BODY_KEYS[ch]
                        last_key_t = now
                    elif ch in ARM_KEYS:
                        active = ("arm",) + ARM_KEYS[ch]
                        last_key_t = now
                    ch = getch(0.0)

                if active is not None and now - last_key_t > HOLD_MS / 1000.0:
                    if moving:
                        self.show("ST ")
                    active = None
                    moving = False

                if active is not None:
                    if not moving:
                        self.show(">>+" if active[2] > 0 else "<<-")
                        moving = True
                    name = self._resolve(active)
                    d = active[2]
                    lo, hi = self._limit_of(name)
                    newv = self._targets[name] + d * math.radians(speed) * dt
                    if newv <= lo or newv >= hi:
                        print("  [!] %s 已到限位 [%.1f°, %.1f°]" % (
                            name, math.degrees(lo), math.degrees(hi)))
                        active = None
                        moving = False
                    else:
                        self._targets[name] = newv

                self.publish_body()
                self.publish_hand()

                if now - last_eff_chk > 0.3:
                    _, eff = self.cur()
                    for n in BODY_EXTRA_JOINT_NAMES:
                        if abs(eff.get(n, 0.0)) > EFFORT_LIMIT:
                            print("[!] 力矩看门狗触发：%s=%.1f，停止。" % (n, eff[n]))
                            raise KeyboardInterrupt
                    last_eff_chk = now
                if now - last_print >= 1.0 and moving:
                    self.show("...")
                    last_print = now
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, old)

        self.show("FN ")


def main():
    rclpy.init()
    node = XbotKeyboardNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
