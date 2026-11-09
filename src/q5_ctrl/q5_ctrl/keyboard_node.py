"""q5_ctrl 键盘步进器：手动步进身体 + 手臂关节，发 HybridJointCommand。

对齐 tools_robot/joint_bend_stepper.py 的键盘手感 + 控制方法：
  按住持续运动、松开即停；发 /wr1_controller/commands 全 20 关节
  （身体 + 手臂步进，未步进的保持反馈）+ /hand_controller/commands 24 手关节保持反馈；
  力矩看门狗兜底。

限位：
  - 身体用 MANUAL_BODY_EXTRA_LIMITS（手册第 4 页，与真机站立位姿一致）。
  - 手臂用 MANUAL_ARM_LIMITS（与真机典型位姿一致）。

用法：ros2 run q5_ctrl keyboard_node
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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from xbot_common_interfaces.msg import HybridJointCommand

from .q5_joint_limits import (
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

# ---- 身体 4 关节（手册限位，见 q5_joint_limits）----
STEP_JOINTS = ("hip_joint", "knee_joint", "ankle_joint", "neck_pitch_joint")
STEP_LIMITS = {name: MANUAL_BODY_EXTRA_LIMITS[name] for name in STEP_JOINTS}
BODY_KEYS = {
    "q": ("hip_joint", -1), "e": ("hip_joint", +1),
    "a": ("knee_joint", -1), "d": ("knee_joint", +1),
    "z": ("ankle_joint", -1), "c": ("ankle_joint", +1),
    "w": ("neck_pitch_joint", +1), "s": ("neck_pitch_joint", -1),
}

# ---- 手臂 7 自由度（逻辑轴 -> 左右关节名；限位用 MANUAL_ARM_LIMITS）----
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
    # 肩（顶行）: y/u 俯仰, i/o 滚, p/; 偏航
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


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("q5_keyboard")
        self._state = {}
        self._effort = {}
        self._pub = self.create_publisher(HybridJointCommand, BODY_TOPIC, 10)
        self._hand_pub = self.create_publisher(HybridJointCommand, HAND_TOPIC, 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_state, qos_profile_sensor_data)

    def _on_state(self, msg):
        self._state = {n: float(p) for n, p in zip(msg.name, msg.position)}
        self._effort = {n: float(e) for n, e in zip(msg.name, msg.effort)}

    def missing(self):
        return [n for n in BODY_JOINT_NAMES if n not in self._state]

    def missing_hand(self):
        return [n for n in HAND_JOINT_NAMES if n not in self._state]

    def publish_body(self, targets):
        """全 20 身体关节：14 臂 + 4 步进身体关节给目标，其余保持反馈。"""
        pos = [targets.get(n, self._state.get(n, 0.0)) for n in ARM_JOINT_NAMES] + [
            targets.get(n, self._state.get(n, 0.0)) for n in BODY_EXTRA_JOINT_NAMES
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
        """24 手关节保持当前反馈值（kp=100 kd=0 ff=350，同 posture_node）。"""
        msg = HybridJointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = list(HAND_JOINT_NAMES)
        msg.position = [float(self._state.get(n, 0.0)) for n in HAND_JOINT_NAMES]
        msg.velocity = [0.0] * len(HAND_JOINT_NAMES)
        msg.feedforward = [HAND_FF] * len(HAND_JOINT_NAMES)
        msg.kp = [HAND_KP] * len(HAND_JOINT_NAMES)
        msg.kd = [HAND_KD] * len(HAND_JOINT_NAMES)
        self._hand_pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardNode()

    for _ in range(50):  # 等第一批 /joint_states（最多 5s）
        rclpy.spin_once(node, timeout_sec=0.1)
        if node._state:
            break
    if not node._state:
        raise SystemExit("[!] 拿不到 /joint_states，中止。")

    # 目标值：身体 4 + 手臂 14，从当前反馈初始化；waist_yaw/neck_yaw 不在 targets 里，保持反馈
    targets = {n: node._state.get(n, 0.0) for n in STEP_JOINTS}
    for ln, rn in ARM_AXES.values():
        targets[ln] = node._state.get(ln, 0.0)
        targets[rn] = node._state.get(rn, 0.0)

    selected_arm = "left"  # 当前选中的手臂（left / right）
    active = None          # 当前按住的动作：(kind, key, direction)
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

    def limit_of(name):
        return STEP_LIMITS.get(name, ARM_LIMITS.get(name, (-1.0, 1.0)))

    def resolve(active):
        """把 (kind, key, dir) 解析成实际关节名。"""
        kind, key, d = active
        if kind == "body":
            return key
        ln, rn = ARM_AXES[key]
        return ln if selected_arm == "left" else rn

    def show(tag):
        s = node._state
        pitch = math.degrees(s.get("ankle_joint", 0) + s.get("knee_joint", 0)
                             + s.get("hip_joint", 0) + s.get("neck_pitch_joint", 0))
        print("  %-3s hip=%6.1f° knee=%6.1f° ankle=%6.1f° neck=%6.1f° -> 头pitch=%6.1f°" % (
            tag, math.degrees(s.get("hip_joint", 0)), math.degrees(s.get("knee_joint", 0)),
            math.degrees(s.get("ankle_joint", 0)), math.degrees(s.get("neck_pitch_joint", 0)), pitch))
        arm = selected_arm
        print("       %s 肩俯仰=%6.1f° 肘俯仰=%6.1f°" % (
            "左" if arm == "left" else "右",
            math.degrees(s.get(ARM_AXES["shoulder_pitch"][0 if arm == "left" else 1], 0)),
            math.degrees(s.get(ARM_AXES["elbow_pitch"][0 if arm == "left" else 1], 0))))

    print("=" * 72)
    print("q5_ctrl 键盘步进器  [%s]  kp=%.0f kd=%.0f" % (BODY_TOPIC, KP, KD))
    print("  身体  q/e 髋  a/d 膝  z/c 踝  w/s 头")
    print("  手臂  f/g 选臂(当前 左)，再：")
    print("        肩: y/u 俯仰   i/o 滚   p/[ 偏航")
    print("        肘: h/j 俯仰   k/l 偏航")
    print("        腕: n/m 俯仰   ,/. 滚")
    print("  t 变速 %.1f↔%.0f°/s   0 打印   x 退出" % (SPEED_DEG_S, SPEED_FAST_DEG_S))
    print("  按住=持续运动，松开=立即停")
    print("=" * 72)
    show("IN ")

    try:
        while True:
            now = time.monotonic()
            dt = min(max(now - last, 0.0), 0.05)
            last = now
            rclpy.spin_once(node, timeout_sec=0.001)

            ch = getch(0.0)
            while ch is not None:
                if ch in ("x", "\x03"):
                    raise KeyboardInterrupt
                if ch == "0":
                    show("P  ")
                elif ch == "t":
                    speed = SPEED_FAST_DEG_S if speed <= SPEED_DEG_S else SPEED_DEG_S
                    print("  速度: %.1f°/s" % speed)
                elif ch == "f":
                    selected_arm = "left"
                    show("L  ")
                elif ch == "g":
                    selected_arm = "right"
                    show("R  ")
                elif ch in BODY_KEYS:
                    active = ("body",) + BODY_KEYS[ch]
                    last_key_t = now
                elif ch in ARM_KEYS:
                    active = ("arm",) + ARM_KEYS[ch]
                    last_key_t = now
                ch = getch(0.0)

            if active is not None and now - last_key_t > HOLD_MS / 1000.0:
                if moving:
                    show("ST ")
                active = None
                moving = False

            if active is not None:
                if not moving:
                    show(">>+" if active[2] > 0 else "<<-")
                    moving = True
                name = resolve(active)
                d = active[2]
                lo, hi = limit_of(name)
                newv = targets[name] + d * math.radians(speed) * dt
                if newv <= lo or newv >= hi:
                    print("  [!] %s 已到限位 [%.1f°, %.1f°]" % (
                        name, math.degrees(lo), math.degrees(hi)))
                    active = None
                    moving = False
                else:
                    targets[name] = newv

            if not node.missing():
                node.publish_body(targets)
            if not node.missing_hand():
                node.publish_hand()

            if now - last_eff_chk > 0.3:
                for n in BODY_EXTRA_JOINT_NAMES:
                    if abs(node._effort.get(n, 0.0)) > EFFORT_LIMIT:
                        print("[!] 力矩看门狗触发：%s=%.1f，停止。" % (n, node._effort[n]))
                        raise KeyboardInterrupt
                last_eff_chk = now
            if now - last_print >= 1.0 and moving:
                show("...")
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)

    show("FN ")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
