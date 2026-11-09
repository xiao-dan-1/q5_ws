"""把 14 个臂关节按**限速直线**移到目标，6 个身体关节保持反馈值。

为什么不用 `start_pose_node`：那条路往 `/servo_poses` 发笛卡尔锚点，由 MPC 做 IK，
7 自由度冗余臂解到哪一支零空间由 MPC 决定——2026-10-20 实测解出的右臂和示范差
45°，正是要避免的东西。这里直接给关节角，不经过 IK。

安全（与 `posture_node` 同一套预检/保持逻辑）：
- 默认 dry-run，只打印逐关节计划；`-p enable_motion:=true` 才发。
- 起始位置取自**运行时的 `/joint_states`**，不是文件里的历史值。
- 每个 tick 都重算 `start + (target-start) * alpha`，所有关节共用同一时长，
  所以关节间的比例保持不变（对肘偏航/腕滚这类会互抵消的轴很重要）。
- 跟踪误差看门狗：实际反馈落后命令超过 `max_tracking_error` 立刻停发并退出
  （说明被别的东西抢了、或机械卡住）。
- 目标先过 `check_target`（手册限位，越界直接拒绝启动，不裁剪）。
- 命令话题必须无其它发布者（MPC/Vive 在跑就拒绝启动）；身体 6 关节必须给反馈值，
  否则 wr1_controller 不让承重关节受力。
"""
import json
import os
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .posture_node import (
    BODY_TOPIC,
    DEFAULT_ALLOWED_PUBLISHERS,
    DISCOVERY_TIMEOUT,
    STATE_TOPIC,
    build_body_command,
    check_body_extra,
    check_target,
    preflight,
    wait_for_discovery,
)
from .q5_joint_limits import ARM_JOINT_NAMES, BODY_EXTRA_JOINT_NAMES

try:
    from xbot_common_interfaces.msg import HybridJointCommand
except ImportError:  # WSL 空环境：只能 dry-run
    from .xbot_msgs import HybridJointCommand


def ramp(start, target, alpha):
    """线性插值；所有关节共用 alpha，所以关节间的位移比例全程不变。

    这一点对肘偏航 + 腕滚这种"反向转、在末端互相抵消"的轴很重要：
    按各自速度独立插值会在中途把抵消关系破坏掉，手会甩出去。
    """
    a = min(1.0, max(0.0, float(alpha)))
    return [s + (t - s) * a for s, t in zip(start, target)]


def plan_duration(start, target, speed, minimum=0.5):
    """最大关节位移 / 限速；下限 minimum 秒，避免极小的移动也算成 0 秒。"""
    if speed <= 0.0:
        raise ValueError("speed 必须 > 0")
    delta = max((abs(t - s) for s, t in zip(start, target)), default=0.0)
    return max(float(minimum), delta / speed)


class ArmMoveNode(Node):
    def __init__(self):
        super().__init__("q5_arm_move")
        self.declare_parameter("target_file", "")
        self.declare_parameter("speed_rad_s", 0.15)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("hold_seconds", 2.0)
        self.declare_parameter("max_tracking_error", 0.25)
        self.declare_parameter("check_environment", True)
        self.declare_parameter("force_publish", False)
        self.declare_parameter("allowed_publishers", list(DEFAULT_ALLOWED_PUBLISHERS))
        self.declare_parameter("state_timeout", 10.0)

        path = str(self.get_parameter("target_file").value)
        if not path:
            raise SystemExit("必须给 -p target_file:=<json>（{'positions': [38]}）")
        raw = json.loads(open(path, encoding="utf-8").read())
        positions = [float(v) for v in raw["positions"]]
        if len(positions) < 14:
            raise SystemExit(f"target_file 至少要 14 个臂关节值，给了 {len(positions)}")
        # 手 24 维**不命令**，用 0 占位过 38 维闸门（0 必在合理范围内）。
        # 不能直接拿示范的手值过闸：示范里 left_hand_thumb_bend=1.836 超出
        # MANUAL_HAND_LIMITS [-1.75, 1.75]——那正是待解决的手部问题，与臂无关。
        try:
            check_target(positions[:14] + [0.0] * 24)
        except ValueError as exc:
            raise SystemExit(f"目标不合法，拒绝启动：{exc}")
        self._target = positions[:14]

        self._enable = bool(self.get_parameter("enable_motion").value)
        self._speed = float(self.get_parameter("speed_rad_s").value)
        self._hold = float(self.get_parameter("hold_seconds").value)
        self._max_err = float(self.get_parameter("max_tracking_error").value)
        self._state_timeout = float(self.get_parameter("state_timeout").value)
        if self._speed <= 0.0:
            raise SystemExit("speed_rad_s 必须 > 0")
        self._check_env = bool(self.get_parameter("check_environment").value)
        self._force = bool(self.get_parameter("force_publish").value)
        self._allowed = tuple(self.get_parameter("allowed_publishers").value)

        self._state = {}
        self._start = None
        self._t0 = None
        self._duration = None
        self._done_at = None
        self._aborted = False

        self._pub = None
        if self._enable:
            if self._check_env and not self._force:
                ready, counts = wait_for_discovery(self, [BODY_TOPIC], DISCOVERY_TIMEOUT)
                self.get_logger().info(
                    f"发现等待完成（{'就绪' if ready else '超时'}）: "
                    + ", ".join(f"{t}={c}" for t, c in counts.items()))
                ok, msgs = preflight(self, [BODY_TOPIC], allowed=self._allowed)
                for line in msgs:
                    self.get_logger().info(f"预检 {line}")
                if not ok:
                    raise SystemExit(
                        "预检不通过，拒绝移动。\n  - " + "\n  - ".join(msgs))
            self._pub = self.create_publisher(HybridJointCommand, BODY_TOPIC, 10)
            self.get_logger().warn(
                f"真机模式：{BODY_TOPIC} 限速 {self._speed:.2f} rad/s 移 14 臂")
        else:
            self.get_logger().warn("dry-run：只打印计划不发布（-p enable_motion:=true 才动）")

        self.create_subscription(JointState, STATE_TOPIC, self._on_state, qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _on_state(self, msg):
        self._state = {n: float(p) for n, p in zip(msg.name, msg.position)}

    def _ready(self):
        return all(n in self._state for n in ARM_JOINT_NAMES + BODY_EXTRA_JOINT_NAMES)

    def _plan(self):
        self._start = [self._state[n] for n in ARM_JOINT_NAMES]
        delta = max(abs(t - s) for t, s in zip(self._target, self._start))
        self._duration = plan_duration(self._start, self._target, self._speed)
        self.get_logger().info(
            f"计划：最大位移 {delta:.4f} rad，{self._duration:.1f} s @ {self._speed:.2f} rad/s")
        for name, s, t in zip(ARM_JOINT_NAMES, self._start, self._target):
            self.get_logger().info(f"  {name:<28} {s:+.4f} -> {t:+.4f}  ({t - s:+.4f})")
        self._t0 = time.monotonic()

    def _tick(self):
        if self._aborted:
            return
        if self._start is None:
            if not self._ready():
                if self._t0 is None:
                    self._t0 = time.monotonic()
                elif time.monotonic() - self._t0 > self._state_timeout:
                    self._abort(f"{self._state_timeout:.0f}s 内没收全 20 个关节反馈，不移动")
                return
            self._t0 = None
            self._plan()
            if not self._enable:
                self.get_logger().info("dry-run：计划已打印，退出")
                self._shutdown()
                return

        elapsed = time.monotonic() - self._t0
        alpha = min(1.0, elapsed / self._duration)
        arm = ramp(self._start, self._target, alpha)

        if alpha >= 1.0:
            if self._done_at is None:
                self._done_at = time.monotonic()
                self.get_logger().info("到达目标，保持中")
            elif time.monotonic() - self._done_at >= self._hold:
                self.get_logger().info("保持完成，退出（控制器保持最后位姿）")
                self._shutdown()
                return

        lag = max(abs(self._state[n] - a) for n, a in zip(ARM_JOINT_NAMES, arm))
        if lag > self._max_err:
            self._abort(f"跟踪误差 {lag:.3f} rad > {self._max_err:.2f}，停发")
            return

        if not self._enable:
            self.get_logger().info(
                f"[dry-run] alpha={alpha:.2f} 最大臂目标位移 "
                f"{max(abs(a - s) for a, s in zip(arm, self._start)):.3f} rad",
                throttle_duration_sec=1.0)
            return

        missing_extra = [n for n in BODY_EXTRA_JOINT_NAMES if n not in self._state]
        if missing_extra:
            self._abort(f"缺未命令关节反馈 {missing_extra}")
            return
        try:
            check_body_extra(self._state)
        except ValueError as exc:
            self._abort(f"保持值非法：{exc}")
            return

        self._pub.publish(build_body_command(self, arm, self._state))

    def _abort(self, reason):
        self._aborted = True
        self.get_logger().error(f"中止：{reason}")
        self._shutdown()

    def _shutdown(self):
        """自己发 SIGINT，与 Ctrl-C 同一条退出路径（rclpy.shutdown 不会让 spin 返回）。"""
        try:
            os.kill(os.getpid(), signal.SIGINT)
        except Exception:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
