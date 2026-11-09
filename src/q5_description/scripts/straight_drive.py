#!/usr/bin/env python3
"""平衡 + 直行助手：轮式里程计 + 航向闭环 + 俯仰平衡。

    ros2 run q5_description straight_drive.py --speed 0 --seconds 8      # 只保持平衡站立
    ros2 run q5_description straight_drive.py --speed 0.25 --seconds 10  # 边走边平衡

为什么需要它：只给左右轮同速是开环的，接触/摩擦的左右不对称会让车慢慢转，实测
10 秒能偏 0.27 m、转头 0.34 rad。真机靠编码器/IMU 闭环修正航向，这里用同一套办法：
从 /joint_states 的轮子转角积分出里程计（差速公式），再把航向偏差反馈成左右轮差速。

俯仰平衡（为什么必须有）：这个模型静止时重心落在驱动轮轴后方（踝关节原点
x=-0.085，轮轴 x=0），不加控制会一直后仰、底盘后缘撑地、前脚轮离地 69 mm。
正扭矩让车头上仰、负扭矩低头，所以用"轮速 ∝ -俯仰角"就能把底盘压回水平——这正是
真机 MPC 在做的事。

里程计用的是编码器推断（和真机一致），不是仿真真值——所以它能反映"闭环能压住多少"，
而不是作弊。滑移大时仍会漂，这与真机一样。
"""
import argparse
import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import ApplyJointEffort, GetEntityState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

WHEEL_RADIUS = 0.077          # 与 URDF 的轮胎半径一致
WHEEL_SEPARATION = 2 * 0.2173  # 两轮中心距


class StraightDrive(Node):
    def __init__(self, speed, seconds, heading_kp, max_accel, pitch_kp, pitch_kd,
                 ankle_balance=False, ankle_kp=1800.0, ankle_kd=180.0, ankle_sign=-1.0):
        super().__init__("straight_drive")
        self.ankle_sign = ankle_sign
        self.ankle_kp = ankle_kp
        self.ankle_kd = ankle_kd
        self.speed = speed
        self.max_accel = max_accel          # rad/s^2，限住阶跃带来的翘头
        self.pitch_kp = pitch_kp            # 俯仰平衡：轮速增益
        self.pitch_kd = pitch_kd
        self.seconds = seconds
        self.start = None          # 用消息里的仿真时间戳记时，别和墙钟混用
        self.deadline = None
        self.heading_kp = heading_kp
        self.target_heading = None
        self.true_heading = None   # 仿真真值航向（当 IMU 用）
        self.true_pitch = None
        self.pitch_rate = 0.0
        self.last_pitch = None
        self.last_pitch_time = None
        self.last = None                       # (t, left, right) 上一次轮子读数
        self.x = self.y = self.heading = 0.0
        self.finished = False
        self.command = (0.0, 0.0)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.pub = self.create_publisher(Float64MultiArray,
                                        "/q5_wheel_controller/commands", 10)
        # 仿真里的"IMU"：直接问 Gazebo 要模型位姿的 yaw。轮式里程计看不到打滑，
        # 真机同样得靠 IMU 才能把航向闭住。
        self.imu_client = self.create_client(GetEntityState, "/get_entity_state")
        self.imu_pending = False
        self.create_timer(0.1, self.poll_imu)
        # 踝关节平衡：把车身后仰顶过卡点、再用反向力矩接住（50 Hz PD）
        self.ankle_effort = None
        if ankle_balance:
            # 走 ros2_control 的 effort 接口（踝关节已改成 command_interface=effort）。
            # 之前用 /apply_joint_effort 服务打补丁会被 position 接口的 SetPosition 抵消。
            self.ankle_pub = self.create_publisher(
                Float64MultiArray, "/q5_ankle_effort_controller/commands", 10)
            self.create_timer(0.02, self.apply_ankle_effort)

    def apply_ankle_effort(self):
        if self.true_pitch is None:
            return
        # pitch<0 = 车头上仰。要把它压回来要给正力矩，但必须带阻尼，
        # 否则冲过卡点后会直接翻过去（实测 ±80 顶不动、120 直接掀翻）。
        torque = self.ankle_sign * (self.ankle_kp * self.true_pitch
                                    + self.ankle_kd * self.pitch_rate)
        torque = max(-180.0, min(180.0, torque))
        self.ankle_pub.publish(Float64MultiArray(data=[torque]))
        self.ankle_effort = torque

    def poll_imu(self):
        if not self.imu_client.service_is_ready() or self.imu_pending:
            return
        request = GetEntityState.Request()
        request.name = "q5"
        request.reference_frame = "world"
        self.imu_pending = True
        future = self.imu_client.call_async(request)
        future.add_done_callback(self.on_imu)

    def on_imu(self, future):
        self.imu_pending = False
        try:
            response = future.result()
        except Exception:                                    # 服务偶发失败就用里程计顶着
            return
        if response is None or not response.success:
            return
        q = response.state.pose.orientation
        import math as _math
        self.true_heading = _math.atan2(2 * (q.w * q.z + q.x * q.y),
                                        1 - 2 * (q.y * q.y + q.z * q.z))
        self.true_pitch = _math.asin(max(-1.0, min(1.0,
                                                   2 * (q.w * q.y - q.z * q.x))))
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pitch is not None and self.last_pitch_time is not None:
            dt = now - self.last_pitch_time
            if dt > 1e-3:
                self.pitch_rate = (self.true_pitch - self.last_pitch) / dt
        self.last_pitch, self.last_pitch_time = self.true_pitch, now

    def on_joints(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now <= 0:
            now = self.get_clock().now().nanoseconds * 1e-9
        positions = dict(zip(msg.name, msg.position))
        left = positions.get("left_drv_wheel_joint")
        right = positions.get("right_drv_wheel_joint")
        if left is None or right is None:
            return

        if self.last is not None:
            dt = now - self.last[0]
            if dt > 0:
                dl = (left - self.last[1]) * WHEEL_RADIUS
                dr = (right - self.last[2]) * WHEEL_RADIUS
                self.heading += (dr - dl) / WHEEL_SEPARATION
                distance = (dl + dr) / 2
                self.x += distance * math.cos(self.heading)
                self.y += distance * math.sin(self.heading)
        self.last = (now, left, right)
        if self.start is None:
            self.start = now
            self.deadline = None if self.seconds <= 0 else now + self.seconds
        if self.target_heading is None:
            self.target_heading = (self.true_heading if self.true_heading is not None
                                   else self.heading)

        # 有"IMU"就用真值航向做闭环，否则退回编码器里程计。
        measured = self.true_heading if self.true_heading is not None else self.heading
        error = self.target_heading - measured          # 正偏差=车头左偏
        correction = self.heading_kp * error
        # 俯仰平衡：pitch 为负 = 车头上仰，这时要给正轮速把它压下去
        balance = 0.0
        if self.true_pitch is not None:
            balance = -(self.pitch_kp * self.true_pitch + self.pitch_kd * self.pitch_rate)
        # 差速：想让航向往回拧，左轮慢一点/右轮快一点
        # 加速度限幅：直接给阶跃指令会让车身反力矩把前脚轮顶起来（翘头），
        # 实测 3 rad/s 阶跃 → 前轮离地。这里按 max_accel 斜坡上去。
        dt = 0.05
        limit = self.max_accel * dt
        def ramp(target, current):
            return current + max(-limit, min(limit, target - current))
        left_speed = ramp(self.speed - correction + balance, self.command[0])
        right_speed = ramp(self.speed + correction + balance, self.command[1])
        self.command = (left_speed, right_speed)
        self.pub.publish(Float64MultiArray(data=[left_speed, right_speed]))

        if self.deadline is not None and now > self.deadline:
            self.finished = True
            self.pub.publish(Float64MultiArray(data=[0.0, 0.0]))
            self.report()          # 只打日志；收尾交给 main 的循环判断

    def report(self):
        self.get_logger().info(
            f"终点：x={self.x:+.3f} y={self.y:+.3f} 航向={self.heading:+.4f} rad "
            f"（目标 {self.target_heading:+.4f}）")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.25, help="前进速度 m/s")
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="0 或负数 = 一直跑（当常驻平衡控制器用）")
    parser.add_argument("--heading-kp", type=float, default=1.0)
    parser.add_argument("--max-accel", type=float, default=2.0,
                        help="轮子角加速度上限 rad/s^2，越小越不容易翘头")
    parser.add_argument("--pitch-kp", type=float, default=30.0,
                        help="俯仰平衡比例增益（轮速 rad/s ÷ 俯仰 rad）")
    parser.add_argument("--pitch-kd", type=float, default=3.0)
    parser.add_argument("--ankle-balance", action="store_true",
                        help="用踝关节力矩做俯仰平衡（对付静止后仰/前轮离地）")
    parser.add_argument("--ankle-kp", type=float, default=1800.0)
    parser.add_argument("--ankle-kd", type=float, default=180.0)
    parser.add_argument("--ankle-sign", type=float, default=-1.0,
                        help="把 pitch<0（前轮离地）压回来所需的力矩符号")
    parser.add_argument("--radius", type=float, default=WHEEL_RADIUS)
    args, _ = parser.parse_known_args()

    radius = args.radius
    speed = args.speed / radius                      # m/s -> rad/s
    rclpy.init()
    node = StraightDrive(speed, args.seconds, args.heading_kp, args.max_accel,
                         args.pitch_kp, args.pitch_kd, args.ankle_balance,
                         args.ankle_kp, args.ankle_kd, args.ankle_sign)
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
