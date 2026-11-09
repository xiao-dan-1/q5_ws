#!/usr/bin/env python3
"""Q5 推理模式 bringup：只起观测链路，不起任何命令发布者。

与 teleop_base 的区别：**不启动 MPC**。MPC 会持续以 ~100Hz 发
/wr1_controller/commands，和策略执行器抢同一个话题；推理时必须让它缺席。

包含（延时对齐 teleop_base）：
    3s   check    只读检查 battery / xbot_state
    4s   activate 触发 /activate_service 进 ACTIVE（/joint_states 才有数据）
    11s  camera   RealSense D455 640x480x30（camera_guard 自检，被占就跳过）
    12s  mjpeg    浏览器图像服务（/camera/camera/color/image_raw，8111 端口）
    18s  relay    /joint_states -> /joint_states_relay @60Hz（跨机推理时才需要）

**不包含**：MPC、hand_server（TCP 2333 手套桥）、zmq_server、vive_bringup。
策略执行器（订阅状态/相机、调策略、发命令）由操作者单独启动；启动前应确认
两条命令话题上没有别的发布者（q5_datacollect preflight 也是这么查的）。

前提：机器人厂商系统已启动；本 launch 不做 source —— 由调用方负责
（ROS_DOMAIN_ID=211、rmw_cyclonedds_cpp、三个 setup.bash）。

用法：
    ros2 launch q5_bringup inference.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node

CHECK_CMD = """
# timeout 要留够：DDS 首次发现要 1~3 秒，给 3 秒会在冷启动时误报"电池无数据"
timeout 10 ros2 topic echo /battery_state --once 2>/dev/null \
  | grep -E "voltage|temperature|current|charge|percentage|power_supply_status" \
  || echo "  [WARN] 电池无数据"
# /xbot_state 只在状态切换瞬间发布，读不到属正常，不作为告警
timeout 6 ros2 topic echo /xbot_state --once 2>/dev/null | head -4
"""

ACTIVATE_CMD = "ros2 service call /activate_service std_srvs/srv/Trigger {}"


def _relay():
    """关节中继：/joint_states -> /joint_states_relay，限速 60Hz 供跨机订阅。"""
    return Node(
        package="q5_tools", executable="joint_states_relay",
        name="joint_states_relay", output="screen",
        arguments=["--rate", "60.0"],
    )


def generate_launch_description():
    plan = [
        (3.0, ExecuteProcess(
            cmd=["bash", "-c", CHECK_CMD], name="check", output="screen")),
        (4.0, ExecuteProcess(
            cmd=["bash", "-c", ACTIVATE_CMD],
            name="activate", output="screen")),
        (11.0, Node(
            package="q5_tools", executable="camera_guard",
            name="camera_guard", output="screen")),
        (12.0, Node(
            package="q5_tools", executable="mjpeg_server",
            name="mjpeg_server", output="screen",
            arguments=["-t", "/camera/camera/color/image_raw",
                       "-p", "8111", "--fps", "30", "--quality", "60"],
            additional_env={"PYTHONUNBUFFERED": "1"})),
        (18.0, _relay()),
    ]

    return LaunchDescription(
        [LogInfo(msg="推理 bringup：check / activate / 相机 / MJPEG(:8111) / 中继"
                     "（不含 MPC、手套桥、zmq、vive）")]
        + [TimerAction(period=delay, actions=[action]) for delay, action in plan]
    )
