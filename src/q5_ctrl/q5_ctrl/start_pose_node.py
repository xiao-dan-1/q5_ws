"""把机器人摆到采集起始位姿（遥操作那条路，一条命令替掉三个终端）。

等价于这三步：
    ros2 run q5_ctrl vive_static_pose_node
    ros2 run vive_dual_arm_teleop vive_teleop_node
    ros2 param set /vive_dual_arm_teleop_node teleop_enabled true
三步合起来的效果只有一个：往 `/servo_poses` 发同一组 ServoPose
（左/右手 + 头 = `locked_{left,right,head}_pose_xyz_quat`），逼近由 MPC 自己完成。
Vive 只负责把增量做成 0、遥操作节点只负责把锁定位姿转发——两个节点都不需要。

前提和 ABC 一样：MPC 在跑（`teleop_base.launch.py` 起的那套），本节点不启停 MPC、也不切状态机。

默认 dry-run：只读 `/get_pose` 打印差值，不发布。真动加 `-p enable_motion:=true`。
"""

import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:  # 真机有真消息；WSL 没有也能 import（dry-run 用不到）
    from xbot_common_interfaces.msg import ServoPose
except ImportError:  # pragma: no cover
    ServoPose = None

SIDES = ("left", "right", "head")

EXIT_OK, EXIT_PREFLIGHT, EXIT_VERIFY = 0, 2, 3


# ---------------------------------------------------------------- 纯函数（可单测）

def load_target(path):
    """读目标位姿，返回 (frame_id, {side: (x,y,z,qx,qy,qz,qw)})。"""
    with open(path) as f:
        data = json.load(f)
    missing = [s for s in SIDES if s not in data]
    if missing:
        raise ValueError("目标文件缺 %s" % missing)
    poses = {}
    for side in SIDES:
        pose = [float(v) for v in data[side]]
        if len(pose) != 7:
            raise ValueError("%s 必须是 7 个数（xyz + 四元数），实际 %d 个" % (side, len(pose)))
        if not all(math.isfinite(v) for v in pose):
            raise ValueError("%s 不是有限数" % side)
        norm = math.sqrt(sum(v * v for v in pose[3:]))
        if abs(norm - 1.0) > 0.01:
            raise ValueError("%s 四元数没归一化（模长 %.4f）" % (side, norm))
        poses[side] = tuple(pose)
    return str(data.get("frame_id", "base_link")), poses


def pose_error(target, stamped):
    """目标 7 元组 vs 实测 PoseStamped，返回 (位置误差 m, 姿态误差 rad)。"""
    dx = target[0] - stamped.pose.position.x
    dy = target[1] - stamped.pose.position.y
    dz = target[2] - stamped.pose.position.z
    q = stamped.pose.orientation
    dot = abs(sum(a * b for a, b in zip(target[3:], (q.x, q.y, q.z, q.w))))
    return math.sqrt(dx * dx + dy * dy + dz * dz), 2.0 * math.acos(min(1.0, dot))


# ---------------------------------------------------------------- 节点

class StartPose(Node):
    def __init__(self):
        super().__init__("start_pose_node")
        if ServoPose is None:
            raise RuntimeError("没有 xbot_common_interfaces，只能在真机上跑")
        self.declare_parameter("target_file", "")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("servo_poses_topic", "/servo_poses")
        self.declare_parameter("get_pose_topic", "/get_pose")
        self.declare_parameter("rate_hz", 90.0)  # 遥操作的 control_rate_hz
        self.declare_parameter("tolerance_m", 0.01)
        self.declare_parameter("tolerance_rad", 0.05)
        self.declare_parameter("timeout_seconds", 30.0)
        self.declare_parameter("hold_seconds", 1.0)

        self.path = self.get_parameter("target_file").value or _default_target()
        self.frame_id, self.target = load_target(self.path)
        self.latest = None
        self.create_subscription(
            ServoPose, self.p("get_pose_topic"), self._on_pose, 10
        )

    def _on_pose(self, msg):
        self.latest = msg

    def p(self, name):
        return self.get_parameter(name).value

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def errors(self):
        """当前三处误差 {side: (m, rad)}；没有 /get_pose 帧就返回 None。"""
        if self.latest is None:
            return None
        return {
            side: pose_error(self.target[side], getattr(self.latest, side + "_pose"))
            for side in SIDES
        }

    def converged(self):
        errors = self.errors()
        if errors is None:
            return False
        return all(
            m <= float(self.p("tolerance_m")) and a <= float(self.p("tolerance_rad"))
            for m, a in errors.values()
        )

    def report(self, errors, header):
        for side in SIDES:
            m, a = errors[side]
            self.get_logger().info(
                "%s %-5s 位置差 %.4f m  姿态差 %.4f rad" % (header, side, m, a)
            )

    # ---------------- dry-run

    def dry_run(self):
        if not self.spin_until(lambda: self.latest is not None, float(self.p("timeout_seconds"))):
            self.get_logger().error(
                "%.1fs 内没等到 %s（MPC 没起？先跑 teleop_base.launch.py）"
                % (float(self.p("timeout_seconds")), self.p("get_pose_topic"))
            )
            return EXIT_PREFLIGHT
        self.report(self.errors(), "dry-run：")
        self.get_logger().info(
            "%s 订阅者=%d；加 -p enable_motion:=true 才真动"
            % (self.p("servo_poses_topic"), self.count_subscribers(self.p("servo_poses_topic")))
        )
        return EXIT_OK

    # ---------------- motion

    def make_message(self, command):
        msg = ServoPose()
        stamp = self.get_clock().now().to_msg()
        for side in SIDES:
            pose = getattr(msg, side + "_pose")
            pose.header.stamp = stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = command[side][:3]
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = command[side][3:]
        return msg

    def move(self):
        topic = self.p("servo_poses_topic")
        if not self.spin_until(lambda: self.latest is not None, 5.0):
            self.get_logger().error("%s 没有帧，不知道当前位姿" % self.p("get_pose_topic"))
            return EXIT_PREFLIGHT
        publisher = self.create_publisher(ServoPose, topic, 10)
        if not self.spin_until(lambda: self.count_subscribers(topic) > 0, 5.0):
            self.get_logger().error("%s 没有订阅者，MPC 没在收位姿" % topic)
            return EXIT_PREFLIGHT

        rate = max(10.0, float(self.p("rate_hz")))
        timeout = float(self.p("timeout_seconds"))
        self.get_logger().warn(
            "MOTION ENABLED：往 %s 发锁定位姿，%.0f Hz，最多 %.0f s"
            % (topic, rate, timeout)
        )
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            publisher.publish(self.make_message(self.target))
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.converged():
                break
            time.sleep(1.0 / rate)

        if not self.converged():
            self.get_logger().error("%.0f s 内没收敛" % timeout)
            self.report(self.errors() or {s: (float("inf"), float("inf")) for s in SIDES}, "超时：")
            return EXIT_VERIFY

        self.get_logger().info("已到位，保持 %.1f s" % float(self.p("hold_seconds")))
        end = time.monotonic() + float(self.p("hold_seconds"))
        while rclpy.ok() and time.monotonic() < end:
            publisher.publish(self.make_message(self.target))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / rate)
        self.report(self.errors(), "到位：")
        return EXIT_OK

    def run(self):
        if self.count_publishers(self.p("get_pose_topic")) == 0:
            self.get_logger().error(
                "%s 没有发布者：MPC 没起。先跑 teleop_base.launch.py（ABC 路线也是这个前提）"
                % self.p("get_pose_topic")
            )
            return EXIT_PREFLIGHT
        return self.move()


def _default_target():
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory("q5_ctrl") + "/config/start_pose.json"


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    code = EXIT_OK
    try:
        node = StartPose()
        code = node.run() if node.p("enable_motion") else node.dry_run()
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    except Exception as exc:  # 目标文件坏、四元数没归一化……
        print("不能用：%s" % exc, file=sys.stderr)
        code = EXIT_PREFLIGHT
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
