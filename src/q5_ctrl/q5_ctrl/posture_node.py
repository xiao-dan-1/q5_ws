"""Q5 姿态命令发布器（WSL/rospc 端）。

按真机实测的接口发命令，两个话题、两条 HybridJointCommand：
  /wr1_controller/commands   14 臂关节   kp=100 kd=2   ff=0
  /hand_controller/commands  24 手关节   kp=100 kd=0   ff=350
关节名用模型契约名（= /joint_states 里的真名，见 q5_joint_limits.py）；
/joint_states 数组顺序是乱的，只按名取值，绝不按下标切。

安全：
- 命令**不驱动关节**，机器人必须已 activate（ACTIVE）才有执行者。
- 默认 dry-run：只构造 + 自检 + 打印，不发布。加 -p enable_motion:=true 才发。
- 手臂越出手册限位直接拒绝（不裁剪）；手部过合理性闸门也拒绝。
- 真机模式发布前做预检（可关）：
    * 命令话题没有订阅者 -> 会静默空转，直接拒绝启动（除非 force_publish）
    * 命令话题上有别的发布者 -> 有 MPC/Vive/hand_server 在抢，拒绝启动（除非 force_publish）
  启动后每 env_check_period 秒复查一次同类条件（默认 5s），因为
  "启动之后才加入的抢话题者"（如 14s 后起来的 hand_tcp_hf_bridge）启动闸门看不见。
"""
import os
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .q5_joint_limits import (
    ARM_JOINT_NAMES,
    BODY_EXTRA_JOINT_NAMES,
    BODY_JOINT_NAMES,
    HAND_JOINT_NAMES,
    MANUAL_ARM_LIMITS,
    MANUAL_BODY_EXTRA_LIMITS,
    MANUAL_HAND_LIMITS,
    MODEL_JOINT_NAMES,
)

try:  # 真机 / 装了 SDK 的环境用真消息
    from xbot_common_interfaces.msg import HybridJointCommand
except ImportError:  # WSL 空环境用替身，dry-run 够用
    from .xbot_msgs import HybridJointCommand

BODY_TOPIC = "/wr1_controller/commands"
HAND_TOPIC = "/hand_controller/commands"
STATE_TOPIC = "/joint_states"
ARM_KP, ARM_KD, ARM_FF = 100.0, 2.0, 0.0
HAND_KP, HAND_KD, HAND_FF = 100.0, 0.0, 350.0

# 启动时等 DDS 发现出订阅者的上限（秒）。别设 0：立刻查会得到"0 个订阅者"的假阴性，
# 2026-09-28 实测就是被这个坑掉一次端到端测试。
DISCOVERY_TIMEOUT = 15.0

# 机器人自带的只读监视节点挂在命令话题上（不是抢控制权），预检时放行。
DEFAULT_ALLOWED_PUBLISHERS = (
    "joint_metric_wr1_command_subscriber",
    "joint_metric_command_subscriber",
)


def build_body_command(node, arm_targets, state):
    """发**全 20 个身体关节**的 HybridJointCommand。

    未命令的 6 个（踝/膝/髋/腰/颈）给**当前反馈值**即"保持"——它们必须出现在
    消息里，否则 wr1_controller 不让它们受力，受载时报驱动错误（2026-09-28 实测）。

    state: {joint_name: position}，来自 /joint_states。缺失的关节按 0.0 处理，
    但那种情况下会打告警（在 _tick 里做）。
    """
    names = list(BODY_JOINT_NAMES)
    positions = [float(v) for v in arm_targets] + [
        float(state.get(n, 0.0)) for n in BODY_EXTRA_JOINT_NAMES
    ]
    return build_command(node, names, positions, ARM_FF, ARM_KP, ARM_KD)


def check_target(positions):
    """38 维目标自检：长度、有限、臂限位（越界抛错，不裁剪）、手部合理性闸门。"""
    if len(positions) != len(MODEL_JOINT_NAMES):
        raise ValueError(f"positions 必须 {len(MODEL_JOINT_NAMES)} 个，给了 {len(positions)}")
    for name, value in zip(MODEL_JOINT_NAMES, positions):
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError(f"{name} 的目标不是有限值: {value}")
    for name, value in zip(ARM_JOINT_NAMES, positions[:14]):
        low, high = MANUAL_ARM_LIMITS[name]
        if not (low <= value <= high):
            raise ValueError(f"{name}={value:.4f} 超出手册限位 [{low}, {high}]")
    for name, value in zip(HAND_JOINT_NAMES, positions[14:]):
        low, high = MANUAL_HAND_LIMITS[name]
        if not (low <= value <= high):
            raise ValueError(
                f"{name}={value:.4f} 超出合理范围 [{low}, {high}]"
                "（手部官方无硬限位，这是合理性闸门，不是手册限位）"
            )
    return positions


def check_body_extra(state):
    """6 个未命令关节的保持值必须落在手册限位内（保持值来自反馈，一般必然合法；
    这条是防止反馈本身异常时把它原样发回去）。"""
    for name in BODY_EXTRA_JOINT_NAMES:
        v = state.get(name)
        if v is None:
            continue
        low, high = MANUAL_BODY_EXTRA_LIMITS[name]
        if not (low <= v <= high):
            raise ValueError(
                f"{name} 当前反馈 {v:.4f} 超出该关节限位 [{low}, {high}]"
                "—— 反馈异常，拒绝把它当保持值发出去"
            )
    return state


def build_command(node, joint_names, positions, feedforward, kp, kd):
    msg = HybridJointCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.joint_name = list(joint_names)
    msg.position = [float(v) for v in positions]
    msg.velocity = [0.0] * len(joint_names)
    msg.feedforward = [feedforward] * len(joint_names)
    msg.kp = [kp] * len(joint_names)
    msg.kd = [kd] * len(joint_names)
    return msg


def command_topic_names(body_only=False):
    """本次会发布的话题列表。"""
    return [BODY_TOPIC] if body_only else [BODY_TOPIC, HAND_TOPIC]


def collect_conflicts(node, topics, allowed=DEFAULT_ALLOWED_PUBLISHERS, own_name=None):
    """返回 {topic: [未授权发布者名]}：命令话题上不是自己在发的那些。"""
    own = own_name or node.get_name()
    conflicts = {}
    for topic in topics:
        others = []
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception:
            continue
        for info in infos:
            name = getattr(info, "node_name", "") or ""
            if name == own or name in allowed:
                continue
            others.append(name)
        if others:
            conflicts[topic] = others
    return conflicts


def preflight(node, topics, allowed=DEFAULT_ALLOWED_PUBLISHERS, own_name=None):
    """真机发布前的环境检查。返回 (ok, 消息列表)。

    没有订阅者 = 命令发出去没人执行（READY 态的 /wr1_controller/commands 就是这样）；
    有别的发布者 = MPC / Vive / hand_server 正在占同一话题。
    两种情况都拒绝，除非调用方显式 force。

    ⚠️ 调用前必须先做 DDS 发现等待（见 wait_for_discovery）：
    本函数只看得到**已经发现自己**的端点，刚 init 的节点立刻查会得到 0 个订阅者，
    那是假阴性，不是环境问题。
    """
    msgs = []
    ok = True
    for topic in topics:
        subs = node.count_subscribers(topic)
        if subs == 0:
            ok = False
            msgs.append(f"{topic}: 没有订阅者 —— 命令不会被任何控制器执行"
                        f"（机器人未启动底层 / 未进 ACTIVE，或 DDS 没连上）")
        else:
            msgs.append(f"{topic}: {subs} 个订阅者")
    conflicts = collect_conflicts(node, topics, allowed=allowed, own_name=own_name)
    for topic, names in conflicts.items():
        ok = False
        msgs.append(f"{topic}: 已有别的发布者 {sorted(names)} —— 会与 MPC/Vive/hand_server 抢同一话题")
    if node.count_publishers(STATE_TOPIC) == 0:
        msgs.append(f"提示：{STATE_TOPIC} 当前无发布者，收不到关节反馈（dry-run 下不影响）")
    return ok, msgs


def wait_for_discovery(node, topics, timeout, sleep=0.5):
    """等 DDS 发现出命令话题的订阅者。

    返回 (是否全部就绪, 每话题最终订阅者数)。
    2026-09-28 实测：不等待时 count_subscribers() 返回 0，节点误判"没有订阅者"而拒绝启动，
    而同一时刻 CLI/rclpy 另起一个节点都看得到 1 个订阅者。纯发现时序问题。
    """
    deadline = time.monotonic() + max(0.0, timeout)
    counts = {}
    while True:
        counts = {t: node.count_subscribers(t) for t in topics}
        if counts and all(c > 0 for c in counts.values()):
            return True, counts
        if time.monotonic() >= deadline:
            return False, counts
        time.sleep(sleep)


class PostureNode(Node):
    def __init__(self):
        super().__init__("q5_posture")
        self.declare_parameter("positions", [0.0] * 38)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("body_only", False)
        self.declare_parameter("check_environment", True)
        self.declare_parameter("force_publish", False)
        self.declare_parameter("allowed_publishers", list(DEFAULT_ALLOWED_PUBLISHERS))
        self.declare_parameter("env_check_period", 5.0)
        # run_seconds: 发满这么多秒后自己优雅退出（0 = 一直发）。
        # 存在的理由：外部用 `timeout` 硬杀正在 30Hz 写命令的发布者，
        # 2026-09-28 把机器人搞进过 ERROR。让节点自己收尾，不要外部 kill。
        self.declare_parameter("run_seconds", 0.0)
        self.declare_parameter("wrap_up_seconds", 1.5)

        target = [float(v) for v in self.get_parameter("positions").value]
        try:
            check_target(target)
        except ValueError as exc:
            raise SystemExit(f"目标不合法，拒绝启动：{exc}")

        self._enable = bool(self.get_parameter("enable_motion").value)
        self._body_only = bool(self.get_parameter("body_only").value)
        self._check_env = bool(self.get_parameter("check_environment").value)
        self._force = bool(self.get_parameter("force_publish").value)
        self._allowed = tuple(self.get_parameter("allowed_publishers").value)
        self._target = target
        self._state = {}
        # 运行期环境复查（见 _check_environment）：启动闸门挡不住"启动后才出现的抢话题者"
        self._env_ok = None
        self._env_sign = None
        self._env_period = max(0.1, float(self.get_parameter("env_check_period").value))
        self._last_env_check = 0.0

        self._topics = command_topic_names(self._body_only)

        if self._enable:
            if self._check_env and not self._force:
                # 先等 DDS 发现（否则 count_subscribers 还是 0，会误判成"没订阅者"）
                ready, counts = wait_for_discovery(self, self._topics, DISCOVERY_TIMEOUT)
                self.get_logger().info(
                    f"发现等待完成（{'就绪' if ready else '超时'}）: "
                    + ", ".join(f"{t}={c}" for t, c in counts.items()))
                ok, msgs = preflight(self, self._topics, allowed=self._allowed)
                for line in msgs:
                    self.get_logger().info(f"预检 {line}")
                if not ok:
                    raise SystemExit(
                        "预检不通过，拒绝发布命令。\n  - " + "\n  - ".join(msgs)
                        + "\n确认环境正确后重试；确实要无视请加 -p force_publish:=true，"
                          "或只做构造加 -p check_environment:=false"
                    )
            elif self._force and self._check_env:
                wait_for_discovery(self, self._topics, DISCOVERY_TIMEOUT)
                ok, msgs = preflight(self, self._topics, allowed=self._allowed)
                for line in msgs:
                    self.get_logger().warn(f"预检(force，不拦截) {line}")
            self.body_pub, self.hand_pub = self._make_publishers()
            self.get_logger().warn(
                f"真机模式：将向 {' + '.join(self._topics)} 发布"
                "（前提：机器人已 ACTIVE，且没有 MPC/Vive 抢同一话题）"
            )
        else:
            self.body_pub = self.hand_pub = None
            self.get_logger().warn("dry-run：只构造消息不发布（发布加 -p enable_motion:=true）")

        self.create_subscription(JointState, STATE_TOPIC, self._on_state, qos_profile_sensor_data)
        hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(f"q5_posture up: 38 维目标 @ {hz} Hz")

        # 自动收尾：到点先平滑减速停发，再优雅退出（不留给外部 kill 的机会）
        self._run_seconds = float(self.get_parameter("run_seconds").value)
        self._wrap_up = max(0.0, float(self.get_parameter("wrap_up_seconds").value))
        self._t0 = None
        self._start_arm = None
        self._stopped = False
        if self._run_seconds > 0.0:
            self.get_logger().info(
                f"run_seconds={self._run_seconds:.1f}s：到点自动收尾退出（wrap_up={self._wrap_up:.1f}s）")

    def _wrap_up_done(self):
        """到点后的收尾：停发 → 优雅退出。

        ⚠️ 只调 rclpy.shutdown() **不会**让 rclpy.spin() 返回（2026-09-28 实测：
        日志打了"已到 run_seconds"但进程继续活着，被外部 timeout 杀掉）。
        这里改成给自己发 SIGINT —— 与 Ctrl-C 同一条路径，main() 里已能干净处理。
        """
        if self._stopped:
            return
        self._stopped = True
        self.get_logger().info("已到 run_seconds，停止发布并优雅退出（机器人由控制器保持最后位姿）")
        try:
            os.kill(os.getpid(), signal.SIGINT)
        except Exception:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _make_publishers(self):
        """建发布器。替身消息没有 _TYPE_SUPPORT，rclpy 建不出来 —— 报人话，不抛栈。"""
        try:
            body = self.create_publisher(HybridJointCommand, BODY_TOPIC, 10)
            hand = None if self._body_only else self.create_publisher(
                HybridJointCommand, HAND_TOPIC, 10)
        except AttributeError as exc:
            raise SystemExit(
                "发布需要真消息类型 xbot_common_interfaces/msg/HybridJointCommand —— "
                "当前用的是替身（只够 dry-run）。\n"
                "装法：Ubuntu-xd 里缺 SDK 包，从机器人拷 src 编，或直接在有 SDK 的机器上跑。\n"
                f"原始错误：{exc}"
            )
        return body, hand

    def _on_state(self, msg):
        """按名收编：/joint_states 顺序是乱的，只做名字到这个字典的映射。"""
        self._state = {n: float(p) for n, p in zip(msg.name, msg.position)}

    def _env_check_due(self):
        """周期性复查节流：每 env_check_period 秒最多查一次。"""
        now = time.monotonic()
        if now - self._last_env_check < self._env_period:
            return False
        self._last_env_check = now
        return True

    def _check_environment(self, context):
        """查环境并在**状态变差或变化时**告警。

        启动闸门只挡得住启动那一刻的冲突；真正的风险是启动后才加入的发布者
        （例：q5_ctrl 先起，14s 后 vive_teleop.launch.py 拉起 hand_tcp_hf_bridge，
        两者同发 /hand_controller/commands）。所以这里周期性复查，
        健康->不健康 或 冲突集合变化 时各告警一次，恢复时也报一次。

        注意：这里调 preflight 前**不**再等发现——节点已经跑起来了，图早就热了；
        再加等待只会让每次复查都卡住。启动时那次等待见 __init__。
        """
        ok, msgs = preflight(self, self._topics, allowed=self._allowed)
        conflicts = collect_conflicts(self, self._topics, allowed=self._allowed)
        sign = tuple(sorted((t, tuple(sorted(n))) for t, n in conflicts.items()))
        if ok:
            if self._env_ok is False:
                self.get_logger().warn(f"{context}环境恢复：命令话题已无冲突")
            self._env_ok = True
            self._env_sign = sign
            return
        if self._env_ok is not False or sign != self._env_sign:
            self.get_logger().warn(
                f"{context}预检发现问题（已启动，不中断发布；下次启动会拒绝）："
                + " | ".join(msgs)
            )
        self._env_ok = False
        self._env_sign = sign

    def _tick(self):
        # ⚠️ run_seconds 的收尾必须放在**最前面**：dry-run 分支下面有 early return，
        # 之前把收尾写在它后面，导致不开 enable_motion 时节点永不自退（2026-09-28 实测踩到）。
        run_started = False
        if self._run_seconds > 0.0:
            now = time.monotonic()
            if self._t0 is None:
                self._t0 = now
                self._start_arm = list(self._target[:14])
            elapsed = now - self._t0
            if elapsed >= self._run_seconds:
                self._wrap_up_done()
                return
            run_started = True

        if not self._enable:
            missing = [n for n in MODEL_JOINT_NAMES if n not in self._state]
            try:
                body = build_body_command(self, self._target[:14], self._state)
                n_joints = len(body.joint_name)
            except Exception:
                n_joints = -1
            self.get_logger().info(
                f"[dry-run] body={n_joints} 关节(全 20 身体关节) kp={ARM_KP} kd={ARM_KD} "
                f"| 收了 {len(self._state)} 个关节反馈, 缺 {len(missing)} 个",
                throttle_duration_sec=5.0)
            return
        if self._check_env and not self._force and self._env_check_due():
            self._check_environment("运行中")

        # 保持值必须来自真实反馈，且要在限位内 —— 否则拒绝这一拍（不发半截命令）
        missing_extra = [n for n in BODY_EXTRA_JOINT_NAMES if n not in self._state]
        if missing_extra:
            self.get_logger().error(
                f"缺未命令关节反馈 {missing_extra}，本轮不发布"
                "（按 0.0 发出去会让踝/膝/髋失控）",
                throttle_duration_sec=5.0)
            return
        try:
            check_body_extra(self._state)
        except ValueError as exc:
            self.get_logger().error(f"保持值非法，本轮不发布：{exc}", throttle_duration_sec=5.0)
            return

        # 自动收尾的"减速段"：到点前 wrap_up 秒内线性回到起始姿态。
        # 目的不是控制精度，而是别让命令流在半截上被外部 kill 掉。
        if run_started and self._wrap_up > 0.0:
            t_start_ramp = self._run_seconds - self._wrap_up
            if elapsed > t_start_ramp:
                alpha = (elapsed - t_start_ramp) / self._wrap_up      # 0 -> 1
                arm_target = [t + (s - t) * alpha
                              for t, s in zip(self._target[:14], self._start_arm)]
                body = build_body_command(self, arm_target, self._state)
            else:
                body = build_body_command(self, self._target[:14], self._state)
        else:
            body = build_body_command(self, self._target[:14], self._state)

        self.body_pub.publish(body)
        if self.hand_pub is not None:
            hand = build_command(self, HAND_JOINT_NAMES, self._target[14:], HAND_FF, HAND_KP, HAND_KD)
            self.hand_pub.publish(hand)


def main(args=None):
    rclpy.init(args=args)
    node = PostureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:  # 被 SIGTERM/SIGINT 杀掉时 rcl 可能已经关了，别喷栈
            rclpy.shutdown()
        except Exception:
            pass
