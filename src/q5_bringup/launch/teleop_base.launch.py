#!/usr/bin/env python3
"""Q5 遥操公共编排：只读检查 / activate / MPC / 相机 / MJPEG / 关节中继（**设备无关**）。

这是基础层：**没有开关**，被 include 就全跑。理由（9/18 定）：
    * 基础就该是确定的 —— 谁 include 它，行为都一样，不用先读一堆开关默认值
    * 任何 `DeclareLaunchArgument` 都会变成**全局 launch 配置**，被 launch 当成节点参数
      塞给被 include 的厂商 launch（实测 realsense 会刷一串
      `Parameter 'xxx' is not supported`）。基础层不声明参数，这个泄漏就没了
    * 之前那套开关（check/activate/mpc/camera/relay）在实际使用中从没被关过
开关挪到设备层：vive 专属 = 手套桥/zmq_server/vive；meta 专属 = vr/webxr_camera/hand/mocap。

包含的内容（按延时顺序，对齐 bash 版 `start_guojinbang.sh` 的 sleep）：
    3s   check    只读检查 battery / xbot_state（3s 起是为了等 DDS 发现）
    4s   activate 触发 /activate_service 进 ACTIVE
    6s   mpc      启动 MPC 并回读状态（1=已启动）
    11s  camera   RealSense D455 640x480x30，关深度/红外/IMU
    12s  mjpeg    浏览器图像服务，8111 端口、30fps、JPEG 质量 60
    18s  relay    /joint_states -> /joint_states_relay，限速 60Hz

关于 enable（`/Start_EE_Retarget`）：**刻意不由本 launch 调用**。
实测（9/18）头显未连时它只返回 `False "Please connect to robot first!"`，
因为头显连 :8010 必然发生在 launch 之后。所以由人在确认现场安全后手动执行
—— 这仍是物理安全动作。

（18s 的 teleop_watch 就绪哨兵 10/12 已删：它只认头显话题 /vr_*_hand，
vive 链路永远等不到 → 每次都刷一句错的「等待头显连接 :8010」；而该话题实测
大部分时间为 0，meta 上也会误报。别再加回来。）

用法（一般不用直接跑，设备 launch 会 include；单步调试时可以）：
    ros2 launch q5_bringup teleop_base.launch.py

前提：本 launch 不做 source —— 由调用方负责（ROS_DOMAIN_ID=211、rmw_cyclonedds_cpp、三个 setup.bash）。
冷启动（状态机不在 READY/ACTIVE）：bash /home/developer/xiaodan/debug_head_tune.sh
"""

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch_ros.actions import Node

PY = "/usr/bin/python"
Q5_PUB = "/home/developer/teleop_client/q5_pub_client.py"

CHECK_CMD = """
# timeout 要留够：DDS 首次发现要 1~3 秒，原来给 3 秒会在冷启动时误报"电池无数据"（9/18 实测）
timeout 10 ros2 topic echo /battery_state --once 2>/dev/null \
  | grep -E "voltage|temperature|current|charge|percentage|power_supply_status" \
  || echo "  [WARN] 电池无数据"
# /xbot_state 只在状态切换瞬间发布，读不到属正常，不作为告警
timeout 6 ros2 topic echo /xbot_state --once 2>/dev/null | head -4
"""

# MPC 没起来下游全是空转，所以启动后显式回读一次状态
MPC_CMD = f"""
{PY} {Q5_PUB} --type mpc --cmd start 2>&1 | tail -1
sleep 3
echo "  MPC 状态（1=已启动）:"
{PY} {Q5_PUB} --type mpc --cmd query 2>&1 | tail -1
"""


def _relay():
    """关节中继：/joint_states -> /joint_states_relay，限速 60Hz 供跨机 rospc 订阅。

    60Hz 是写死的：相机 30.5Hz，中继若也 30Hz 会比帧慢，采集时出现 state 陈旧跳帧。
    """
    return Node(
        package="q5_tools", executable="joint_states_relay",
        name="joint_states_relay", output="screen",
        arguments=["--rate", "60.0"],
    )


def generate_launch_description():
    # (延时秒, 动作)。延时对齐 bash 版里的 sleep，保证 MPC 先于下游起来。
    # 注意 period 必须是 float —— 传 int 会被 TimerAction 拒掉。
    plan = [
        # check 排在 3s 而不是 0s：让 DDS 先发现完，否则电池会被读成"无数据"
        (3.0, ExecuteProcess(
            cmd=["bash", "-c", CHECK_CMD], name="check", output="screen")),
        (4.0, ExecuteProcess(
            cmd=["bash", "-c",
                 "ros2 service call /activate_service std_srvs/srv/Trigger {}"],
            name="activate", output="screen")),
        (6.0, ExecuteProcess(
            cmd=["bash", "-c", MPC_CMD], name="mpc", output="screen")),
        (11.0, Node(
            package="q5_tools", executable="camera_guard",
            name="camera_guard", output="screen")),
        # 可先订阅等待相机出帧；与相机守卫独立，兼容相机已在运行的情况。
        (12.0, Node(
            package="q5_tools", executable="mjpeg_server",
            name="mjpeg_server", output="screen",
            arguments=["-t", "/camera/camera/color/image_raw",
                       "-p", "8111", "--fps", "30", "--quality", "60"],
            additional_env={"PYTHONUNBUFFERED": "1"})),
        (18.0, _relay()),
    ]

    return LaunchDescription(
        [LogInfo(msg="teleop_base：check / activate / MPC / 相机 / MJPEG(:8111) / 关节中继"
                     "（无开关，基础层）")]
        + [TimerAction(period=delay, actions=[action]) for delay, action in plan]
    )
