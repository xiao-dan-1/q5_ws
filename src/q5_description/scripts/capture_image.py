#!/usr/bin/env python3
"""存一帧相机图像为 PNG（无头截图用）。

    ros2 run q5_description capture_image.py --topic /cam_view/image_raw --out /tmp/q5.png
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image


def to_array(msg):
    """按常见 encoding 手写转换，不依赖 cv_bridge。"""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if msg.encoding not in channels:
        raise SystemExit(f"暂不支持的 encoding：{msg.encoding}")
    count = channels[msg.encoding]
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step and msg.step != msg.width * count:
        buffer = buffer.reshape(msg.height, msg.step)[:, :msg.width * count]
    image = buffer.reshape(msg.height, msg.width, count)
    if msg.encoding.startswith("bgr") or msg.encoding.startswith("bgra"):
        order = [2, 1, 0, 3] if count == 4 else [2, 1, 0]
        image = image[:, :, order]
    return image[:, :, :3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/cam_view/image_raw")
    parser.add_argument("--out", default="/tmp/q5_cam.png")
    parser.add_argument("--timeout", type=float, default=60.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Node("q5_capture_image")
    received = {}
    # 同时挂两路订阅：真机相机一般发 BEST_EFFORT，Gazebo 的
    # gazebo_ros_camera 发的是 RELIABLE + VOLATILE，哪路先到用哪路。
    callbacks = lambda msg: received.setdefault("msg", msg)          # noqa: E731
    node.create_subscription(Image, args.topic, callbacks, qos_profile_sensor_data)
    node.create_subscription(
        Image, args.topic, callbacks,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and "msg" not in received:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "msg" not in received:
        print(f"{args.timeout:.0f}s 内没收到 {args.topic} 的图像", file=sys.stderr)
        return 1

    msg = received["msg"]
    PilImage.fromarray(to_array(msg)).save(args.out)
    print(f"{args.out}  {msg.width}x{msg.height} {msg.encoding}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
