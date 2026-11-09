#!/usr/bin/env python3
"""自检 TF：等 robot_state_publisher 起来后，检查整棵树是否连到几个末端坐标系。

    ros2 run q5_description check_tf.py
    ros2 run q5_description check_tf.py --expect world:base_link   # 额外检查一对

需要 /robot_state_publisher 在跑（`ros2 launch q5_description description.launch.py`）。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# 全固定关节链：只起 robot_state_publisher 就该查得到。
FIXED_LEAVES = [
    ("base_link", "lidar_link"),
    ("base_link", "left_drv_hang_link"),
]
# 跨可动关节的链：还需要有人发 /joint_states（display.launch.py 里的
# joint_state_publisher 就会发），否则 TF 在踝/膝/髋/腰处断开。
JOINT_LEAVES = [
    ("base_link", "left_hand_index_rota_tip"),
    ("base_link", "right_hand_index_rota_tip"),
    ("base_link", "neck_pitch_link"),
    ("base_link", "left_drv_wheel_link"),
]
LEAVES = FIXED_LEAVES + JOINT_LEAVES


def main():
    extra = []
    for index, value in enumerate(sys.argv):
        if value == "--expect" and index + 1 < len(sys.argv):
            extra = [tuple(pair.split(":")) for pair in sys.argv[index + 1].split(",")]

    rclpy.init()
    node = Node("q5_description_check_tf")
    buffer = Buffer()
    TransformListener(buffer, node)

    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(buffer.can_transform(parent, child, Time())
               for parent, child in LEAVES + extra):
            break

    failed = []
    for parent, child in LEAVES + extra:
        if buffer.can_transform(parent, child, Time()):
            t = buffer.lookup_transform(parent, child, Time())
            p = t.transform.translation
            print(f"OK   {parent} -> {child}  "
                  f"({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")
        else:
            failed.append(f"{parent} -> {child}")
            print(f"FAIL {parent} -> {child}")
    print(f"缓冲区里已知坐标系 {buffer.all_frames_as_yaml().count(chr(10))} 行")

    node.destroy_node()
    rclpy.shutdown()
    if any(f"{parent} -> {child}" in failed for parent, child in JOINT_LEAVES):
        print("提示：跨可动关节的链需要 /joint_states。用 "
              "`ros2 launch q5_description display.launch.py rviz:=false`，"
              "或自己起 joint_state_publisher(_gui)。")
    if failed:
        known = sorted(line.split(":")[0].strip()
                       for line in buffer.all_frames_as_yaml().splitlines()
                       if line and not line.startswith(" ") and ":" in line)
        print(f"缓冲区里的坐标系（{len(known)}）：{known[:20]}{' ...' if len(known) > 20 else ''}")
        print(f"不连通的变换：{failed}")
        return 1
    print("TF OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
