"""自检：38 维契约、按名收编、限位拒绝、预检逻辑。不碰 ROS 网络，pytest 直接跑。

对齐说明：build_command 用真 HybridJointCommand 时，header.stamp 的 setter 会断言
必须是 builtin_interfaces/Time（旧版假时钟返回字符串 "stamp" 会挂）。这里让假时钟
返回真的 Time()，于是有没有真消息包都能跑。
"""
import types

import pytest

from q5_ctrl.q5_joint_limits import (
    ARM_JOINT_NAMES,
    HAND_JOINT_NAMES,
    MODEL_JOINT_NAMES,
    DATASET_MEAN_POSE,
    MANUAL_HAND_LIMITS,
)
from q5_ctrl.posture_node import (
    BODY_TOPIC,
    DEFAULT_ALLOWED_PUBLISHERS,
    HAND_TOPIC,
    build_body_command,
    check_body_extra,
    check_target,
    build_command,
    collect_conflicts,
    command_topic_names,
    preflight,
    wait_for_discovery,
)
from q5_ctrl.q5_joint_limits import BODY_EXTRA_JOINT_NAMES, BODY_JOINT_NAMES


class _FakeClock:
    """假时钟：必须返回真的 builtin_interfaces/Time。

    真消息的 header.stamp setter 会断言 isinstance(value, Time)，
    自造的 time 类（或旧版返回字符串 "stamp"）都会挂在这里。
    """

    def now(self):
        class _T:
            def to_msg(self):
                from builtin_interfaces.msg import Time
                return Time()
        return _T()


class _FakeNode:
    def get_clock(self):
        return _FakeClock()


def test_contract_shape():
    assert len(MODEL_JOINT_NAMES) == 38
    assert len(ARM_JOINT_NAMES) == 14 and len(HAND_JOINT_NAMES) == 24
    assert len(set(MODEL_JOINT_NAMES)) == 38
    assert ARM_JOINT_NAMES[0] == "left_shoulder_pitch_joint"
    assert HAND_JOINT_NAMES[0] == "left_hand_thumb_bend_joint"


def test_hand_limits_cover_all_hand_joints():
    assert set(MANUAL_HAND_LIMITS) == set(HAND_JOINT_NAMES)


def test_dataset_mean_pose_passes():
    check_target(list(DATASET_MEAN_POSE))


def test_out_of_limit_rejected_not_clamped():
    bad = list(DATASET_MEAN_POSE)
    bad[3] = 0.5  # left_elbow_pitch 手册上限 0.09
    with pytest.raises(ValueError, match="left_elbow_pitch_joint"):
        check_target(bad)


def test_hand_sanity_gate_rejects_absurd_value():
    bad = list(DATASET_MEAN_POSE)
    bad[14] = 99.0  # left_hand_thumb_bend_joint
    with pytest.raises(ValueError, match="left_hand_thumb_bend_joint"):
        check_target(bad)


def test_wrong_length_rejected():
    with pytest.raises(ValueError):
        check_target([0.0] * 37)


def test_non_finite_rejected():
    bad = list(DATASET_MEAN_POSE)
    bad[0] = float("nan")
    with pytest.raises(ValueError, match="有限值"):
        check_target(bad)


def test_build_command_field_counts_and_gains():
    node = _FakeNode()
    msg = build_command(node, ARM_JOINT_NAMES, list(DATASET_MEAN_POSE[:14]), 0.0, 85.0, 20.0)
    assert len(msg.joint_name) == len(msg.position) == len(msg.velocity) == 14
    assert len(msg.feedforward) == len(msg.kp) == len(msg.kd) == 14
    assert set(msg.kp) == {85.0} and set(msg.kd) == {20.0} and set(msg.velocity) == {0.0}


def test_command_topic_names():
    assert command_topic_names(body_only=True) == [BODY_TOPIC]
    assert command_topic_names(body_only=False) == [BODY_TOPIC, HAND_TOPIC]


# --- 全 20 身体关节：只发 14 臂会让未命令的承重关节（踝/膝/髋）报驱动错误 ---
# 2026-09-28 实测：只发 14 臂命令左肘 0.02 rad，ankle_joint + right_elbow_pitch 双双报错。

def test_body_joint_contract_is_20():
    assert len(BODY_JOINT_NAMES) == 20
    assert len(BODY_EXTRA_JOINT_NAMES) == 6
    for n in ("ankle_joint", "knee_joint", "hip_joint",
              "waist_yaw_joint", "neck_yaw_joint", "neck_pitch_joint"):
        assert n in BODY_EXTRA_JOINT_NAMES


def test_build_body_command_carries_all_20_and_holds_uncommanded():
    node = _FakeNode()
    state = {n: 0.25 for n in BODY_EXTRA_JOINT_NAMES}      # 反馈值
    arm = list(DATASET_MEAN_POSE[:14])
    msg = build_body_command(node, arm, state)

    assert len(msg.joint_name) == 20, "必须发全 20 个身体关节"
    assert set(msg.joint_name) == set(BODY_JOINT_NAMES)
    # 臂给目标值
    for i, n in enumerate(ARM_JOINT_NAMES):
        idx = list(msg.joint_name).index(n)
        assert abs(msg.position[idx] - arm[i]) < 1e-9
    # 未命令关节给反馈值（保持），不是 0
    for n in BODY_EXTRA_JOINT_NAMES:
        idx = list(msg.joint_name).index(n)
        assert abs(msg.position[idx] - 0.25) < 1e-9, f"{n} 应为保持值 0.25"
    # 数组等长
    assert len(msg.position) == len(msg.velocity) == len(msg.feedforward) == 20
    assert len(msg.kp) == len(msg.kd) == 20


def test_check_body_extra_rejects_out_of_limit_feedback():
    bad = {n: 0.0 for n in BODY_EXTRA_JOINT_NAMES}
    bad["knee_joint"] = 9.0          # 手册上限 2.09
    with pytest.raises(ValueError, match="knee_joint"):
        check_body_extra(bad)


def test_check_body_extra_accepts_normal_feedback():
    ok = {n: 0.0 for n in BODY_EXTRA_JOINT_NAMES}
    assert check_body_extra(ok) is ok


class _FakeInfo:
    def __init__(self, node_name):
        self.node_name = node_name


class _FakeRosNode:
    """够 preflight 用的最小替身。"""

    def __init__(self, subs=None, pubs=None, publishers_info=None):
        self._subs = subs or {}
        self._pubs = pubs or {}
        self._info = publishers_info or {}

    def get_name(self):
        return "q5_posture"

    def count_subscribers(self, topic):
        return self._subs.get(topic, 0)

    def count_publishers(self, topic):
        return self._pubs.get(topic, 0)

    def get_publishers_info_by_topic(self, topic):
        return [_FakeInfo(n) for n in self._info.get(topic, [])]


def test_collect_conflicts_ignores_own_and_allowed():
    node = _FakeRosNode(publishers_info={
        BODY_TOPIC: ["q5_posture", "joint_metric_wr1_command_subscriber", "mpc_policy_node"],
        HAND_TOPIC: ["hand_tcp_hf_bridge"],
    })
    conflicts = collect_conflicts(node, [BODY_TOPIC, HAND_TOPIC])
    assert conflicts[BODY_TOPIC] == ["mpc_policy_node"]
    assert conflicts[HAND_TOPIC] == ["hand_tcp_hf_bridge"]


def test_preflight_rejects_no_subscriber():
    node = _FakeRosNode(subs={BODY_TOPIC: 0, HAND_TOPIC: 0})
    ok, msgs = preflight(node, [BODY_TOPIC, HAND_TOPIC])
    assert ok is False
    assert any("没有订阅者" in m for m in msgs)


def test_preflight_rejects_conflicting_publisher_even_with_subscriber():
    node = _FakeRosNode(subs={BODY_TOPIC: 1}, publishers_info={BODY_TOPIC: ["vive_dual_arm_teleop_node"]})
    ok, msgs = preflight(node, [BODY_TOPIC])
    assert ok is False
    assert any("别的发布者" in m for m in msgs)


def test_preflight_passes_when_clean():
    node = _FakeRosNode(
        subs={BODY_TOPIC: 1, HAND_TOPIC: 1},
        pubs={"/joint_states": 1},
        publishers_info={BODY_TOPIC: ["q5_posture"], HAND_TOPIC: ["q5_posture"]},
    )
    ok, msgs = preflight(node, [BODY_TOPIC, HAND_TOPIC])
    assert ok is True
    assert any("1 个订阅者" in m for m in msgs)


# --- 运行期复查：启动后才出现的抢话题者必须被发现 ---
# 真实场景：q5_ctrl 先起，14s 后 vive_teleop.launch.py 拉起 hand_tcp_hf_bridge，
# 两者同发 /hand_controller/commands。启动闸门在 t=0 看不见它。

class _FakeEnvNode:
    """可变的假节点：支持在两次检查之间改变环境。"""

    def __init__(self):
        self._subs = {BODY_TOPIC: 1}
        self._pubs = {"/joint_states": 1}
        self._info = {BODY_TOPIC: ["q5_posture"]}
        self._values = {"enable_motion": True, "env_check_period": 0.0}

    def get_name(self):
        return "q5_posture"

    def count_subscribers(self, topic):
        return self._subs.get(topic, 0)

    def count_publishers(self, topic):
        return self._pubs.get(topic, 0)

    def get_publishers_info_by_topic(self, topic):
        return [_FakeInfo(n) for n in self._info.get(topic, [])]

    def get_clock(self):
        return _FakeClock()

    def body_gets_conflict(self):
        self._info[BODY_TOPIC] = ["q5_posture", "mpc_policy_node"]


class _CollectLogger:
    def __init__(self):
        self.warns = []
        self.infos = []

    def warn(self, msg, **kwargs):
        self.warns.append(msg)

    def info(self, msg, **kwargs):
        self.infos.append(msg)

    def error(self, msg, **kwargs):
        pass


class _FakeTargetPub:
    def __init__(self):
        self.published = 0

    def publish(self, msg):
        self.published += 1


def _node_like():
    """造一个够 _tick/_check_environment 跑的对象（不连 ROS）。"""
    from q5_ctrl.posture_node import PostureNode

    n = _FakeEnvNode()
    n.get_logger = lambda: n.logger
    n.logger = _CollectLogger()
    n._check_env = True
    n._force = False
    n._topics = [BODY_TOPIC]
    n._allowed = DEFAULT_ALLOWED_PUBLISHERS
    n._env_ok = None
    n._env_sign = None
    n._env_period = 0.0
    n._last_env_check = 0.0
    n._run_seconds = 0.0          # 默认一直发（自动收尾的测试会自行覆盖）
    n._wrap_up = 0.0
    n._t0 = None
    n._start_arm = None
    n._stopped = False
    n._target = [0.0] * 38
    n._enable = True
    n._body_only = True
    # 6 个未命令关节必须有反馈，否则 _tick 会因"缺保持值"拒绝发布
    n._state = {name: 0.0 for name in BODY_EXTRA_JOINT_NAMES}
    n.body_pub = _FakeTargetPub()
    n.hand_pub = None
    n._env_check_due = types.MethodType(PostureNode._env_check_due, n)
    n._check_environment = types.MethodType(PostureNode._check_environment, n)
    n._wrap_up_done = types.MethodType(PostureNode._wrap_up_done, n)
    n._tick = types.MethodType(PostureNode._tick, n)
    return n


@pytest.fixture(autouse=True)
def _never_signal_the_test_runner(monkeypatch):
    """保险：_wrap_up_done 真实实现会 os.kill(os.getpid(), SIGINT)，
    在 pytest 里等于打断整个测试进程。任何测试都不许真发信号。"""
    from q5_ctrl import posture_node as mod
    killed = []
    monkeypatch.setattr(mod.os, "kill", lambda *a, **k: killed.append(a))
    monkeypatch.setattr(mod.rclpy, "shutdown", lambda *a, **k: None)
    return killed


def test_periodic_check_detects_publisher_that_appears_after_start():
    node = _node_like()
    assert preflight(node, [BODY_TOPIC])[0] is True   # 启动那一刻是干净的

    node.body_gets_conflict()                          # 14s 后 hand_bridge/MPC 起来了
    node._tick()

    assert any("mpc_policy_node" in w or "别的发布者" in w for w in node.logger.warns), \
        "启动后才出现的抢话题者必须告警"
    assert node.body_pub.published == 1, "告警不应中断发布"


def test_periodic_check_warns_on_change_then_stays_quiet():
    node = _node_like()
    node.body_gets_conflict()
    node._tick()
    first = len(node.logger.warns)
    assert first >= 1
    node._tick()   # 同样的冲突，不该重复刷屏
    assert len(node.logger.warns) == first


# --- DDS 发现等待：启动时立刻查会得到假阴性，必须先等 ---
# 2026-09-28 端到端实测：posture_node 在 __init__ 里立刻 count_subscribers() 得到 0，
# 误判"没有订阅者"而拒绝启动；同一时刻另起一个节点查得到 1 个订阅者。

class _LateDiscoveryNode(_FakeRosNode):
    """前 N 次查询返回 0（模拟发现未完成），之后返回真实值。"""

    def __init__(self, appear_after, topic=BODY_TOPIC, real=1):
        super().__init__(subs={topic: 0})
        self._calls = 0
        self._appear_after = appear_after
        self._topic = topic
        self._real = real

    def count_subscribers(self, topic):
        self._calls += 1
        if self._calls > self._appear_after:
            return self._real
        return 0


class _NeverDiscoverNode(_FakeRosNode):
    """永远查不到订阅者（模拟发现一直不成功 / 机器人真的没起）。"""

    def count_subscribers(self, topic):
        return 0


def test_wait_for_discovery_returns_when_subscriber_appears():
    node = _LateDiscoveryNode(appear_after=2)
    ready, counts = wait_for_discovery(node, [BODY_TOPIC], timeout=5.0, sleep=0.0)
    assert ready is True
    assert counts[BODY_TOPIC] == 1


def test_wait_for_discovery_times_out_and_reports_counts():
    node = _NeverDiscoverNode(subs={BODY_TOPIC: 0})
    ready, counts = wait_for_discovery(node, [BODY_TOPIC], timeout=0.05, sleep=0.0)
    assert ready is False
    assert counts[BODY_TOPIC] == 0


def test_discovery_then_preflight_passes_where_immediate_check_would_fail():
    """这就是端到端挂掉的那个场景：不等发现 -> 拒绝；等了 -> 通过。"""
    node = _LateDiscoveryNode(appear_after=2)
    # 立刻查：假阴性
    assert preflight(node, [BODY_TOPIC])[0] is False
    # 等发现后再查：通过
    node2 = _LateDiscoveryNode(appear_after=2)
    wait_for_discovery(node2, [BODY_TOPIC], timeout=5.0, sleep=0.0)
    assert preflight(node2, [BODY_TOPIC])[0] is True


# --- 自动收尾：不许外部 kill（2026-09-28 用 timeout 硬杀把机器人搞进 ERROR）---

def test_run_seconds_wraps_up_and_stops_publishing(monkeypatch, _never_signal_the_test_runner):
    from q5_ctrl import posture_node as mod
    import signal as _signal

    killed = _never_signal_the_test_runner
    node = _node_like()
    node._run_seconds = 10.0
    node._wrap_up = 1.5
    node._t0 = None
    node._start_arm = None
    node._stopped = False
    node._target = [0.1] * 14 + [0.0] * 24      # 目标与起始姿态不同，便于观察回落

    t = {"v": 1000.0}
    monkeypatch.setattr(mod.time, "monotonic", lambda: t["v"])

    # 第一拍：建立 t0 / 起始姿态
    node._tick()
    assert node._t0 == 1000.0
    assert node._start_arm == [0.1] * 14
    assert node.body_pub.published == 1
    assert killed == []

    # 中段：正常发目标
    t["v"] = 1005.0
    node._tick()
    assert node.body_pub.published == 2
    assert killed == []

    # 收尾窗口内：仍在发（回落到起始姿态）
    t["v"] = 1009.0          # elapsed 9.0 > run(10)-wrap(1.5)=8.5
    node._tick()
    assert node.body_pub.published == 3
    assert killed == []

    # 到点：停止发布 + 发 SIGINT 优雅退出
    t["v"] = 1010.5
    node._tick()
    assert node.body_pub.published == 3, "到点后不应再发布"
    assert len(killed) == 1, "到点后应发 SIGINT 优雅退出（而不是靠外部 kill）"
    assert killed[0][1] == _signal.SIGINT
    assert node._stopped is True


def test_run_seconds_zero_means_keep_publishing(monkeypatch):
    from q5_ctrl import posture_node as mod
    node = _node_like()
    node._run_seconds = 0.0                 # 默认：一直发
    shutdown_calls = []
    monkeypatch.setattr(mod.rclpy, "shutdown", lambda *a, **k: shutdown_calls.append(1))
    for _ in range(5):
        node._tick()
    assert node.body_pub.published == 5
    assert shutdown_calls == []


def test_run_seconds_also_exits_in_dry_run(monkeypatch, _never_signal_the_test_runner):
    """回归：收尾逻辑曾写在 dry-run 的 early return 之后 → 不开 enable_motion 时永不自退。"""
    from q5_ctrl import posture_node as mod

    killed = _never_signal_the_test_runner
    node = _node_like()
    node._enable = False            # dry-run
    node._run_seconds = 3.0
    node._wrap_up = 1.0
    node._t0 = None
    node._start_arm = None
    node._stopped = False
    t = {"v": 500.0}
    monkeypatch.setattr(mod.time, "monotonic", lambda: t["v"])

    node._tick()                    # 建立 t0
    assert killed == []
    t["v"] = 502.0
    node._tick()                    # 还没到点
    assert killed == []
    t["v"] = 503.0
    node._tick()                    # 到点 -> 必须退出，即使 dry-run
    assert len(killed) == 1
