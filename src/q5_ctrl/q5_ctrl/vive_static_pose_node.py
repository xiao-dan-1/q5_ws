"""Constant Vive pose, so the collection teleop node runs without trackers.

Why this works: vive_teleop_node only uses the Vive pose *increment*
(current - anchor). With a constant pose the increment is zero, so the commanded
pose ramps to the configured locked_left/right_pose_xyz_quat and stays there --
the same mechanism that positioned the robot for every training episode.

The pose value is therefore irrelevant; it only has to keep arriving, because
input_timeout_sec (default 0.3 s) holds the command when it stops.

Run this *instead of* the Vive ZMQ bridge, then launch the vendor teleop node
unchanged:

    ros2 run q5_ctrl vive_static_pose_node
    ros2 run vive_dual_arm_teleop vive_teleop_node
    ros2 param set /vive_dual_arm_teleop_node teleop_enabled true

别用 `vive_bringup.launch.py`：它会把 `vive_pose_zmq_node` 一起起，和这里抢 /vive/*/pose。
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

LEFT_TOPIC = "/vive/left_elbow/pose"
RIGHT_TOPIC = "/vive/right_elbow/pose"


class ViveStaticPoseNode(Node):
    def __init__(self):
        super().__init__("vive_static_pose_node")
        self.declare_parameter("rate_hz", 30.0)
        rate_hz = float(self.get_parameter("rate_hz").value)
        msg = PoseStamped()
        msg.header.frame_id = "vive_static"
        msg.pose.orientation.w = 1.0
        self._msg = msg
        self._pubs = [
            self.create_publisher(PoseStamped, topic, 10) for topic in (LEFT_TOPIC, RIGHT_TOPIC)
        ]
        self._rate_hz = max(1.0, rate_hz)
        self._reported = False
        self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info("%.0f Hz -> %s %s" % (self._rate_hz, LEFT_TOPIC, RIGHT_TOPIC))

    def _tick(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        for pub in self._pubs:
            pub.publish(self._msg)
        if not self._reported:  # 首帧发出后才报订阅者，用来确认遥操作节点接上了
            self._reported = True
            self.get_logger().info(
                "subs L=%d R=%d" % tuple(p.get_subscription_count() for p in self._pubs)
            )


def main(argv=None):
    rclpy.init(args=argv)
    node = ViveStaticPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM/SIGINT 是正常退出；不拦的话 rclpy 会打一屏栈，看着像挂了
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
