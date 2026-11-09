#!/usr/bin/env python3
"""Vive 遥操 = 基础层（teleop_base）+ 手套桥/zmq_server + 设备那一步（vive 厂商 bringup）。

分层（9/18 定）：
    teleop_base   基础层，无开关，被 include 就全跑（check/activate/MPC/相机/中继）
    本文件          平铺基础层 + 起手套桥/zmq_server + 起 vive_bringup
    开关            只留本层专属的：teleop / hand_cmd_topic / vive
    为什么手套桥和 zmq_server 在这里而不在 base：它们是 **vive 链路专属**的
    （meta 用手部跟踪，teleopX_node 直接发 /hand_controller/commands，
    不用手套 TCP 2333，也不用 zmq_server）。

用法：
    ros2 launch q5_bringup vive_teleop.launch.py
    ros2 launch q5_bringup vive_teleop.launch.py teleop:=false    # 跳过手套桥 + zmq_server
    ros2 launch q5_bringup vive_teleop.launch.py vive:=false      # 只起基础层 + 手套桥
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teleop = IfCondition(LaunchConfiguration("teleop"))
    hand_cmd_topic = LaunchConfiguration("hand_cmd_topic")

    # 基础层：不传 launch_arguments —— CLI 参数是全局配置，被 include 的文件自己能看见。
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("q5_bringup"), "/launch/teleop_base.launch.py"]),
    )
    # 设备那一步：厂商 launch 直接 include，别复制节点参数（它们升级时这边自动跟上）
    vive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("vive_dual_arm_teleop"), "/launch/vive_bringup.launch.py"]),
        condition=IfCondition(LaunchConfiguration("vive")),
    )

    # 延时与拆分前保持一致：base 的相机 10s、中继 18s；手套桥 14s、zmq_server 16s、vive 20s。
    # 手套桥要连 TCP 2333、zmq_server 要占 5555，都得起在 vive 之前。
    hand_bridge = Node(
        package="q5_hand_bridge", executable="hand_bridge",
        name="hand_tcp_hf_bridge", output="screen",
        # bind 地址原先是写死在脚本里的，换网段不用改代码了
        parameters=[{"tcp_host": "192.168.8.100", "tcp_port": 2333,
                     "cmd_topic": hand_cmd_topic}],
        condition=teleop,
    )
    zmq_server = Node(
        package="q5_zmq_bridge", executable="zmq_server",
        # name 必须等于脚本内部 super().__init__ 的名字：Node() 会据此注入
        # `-r __node:=<name>`，写错就是把节点改名（bash 时代它叫 q5_zmq_bridge，
        # 改了 ros2 node info 就找不着）。hand_bridge 那条同理。
        name="q5_zmq_bridge", output="screen",
        # 脚本里的 print 走 stdout，Node() 看不到 -u（shebang 之后的位置参数
        # 不进解释器），只能用环境变量关缓冲，否则日志攒满一块才出。
        additional_env={"PYTHONUNBUFFERED": "1"},
        condition=teleop,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "teleop", default_value="true",
            description="手套桥 hand_bridge + zmq_server（vive 链路专属；干跑可关）"),
        DeclareLaunchArgument("hand_cmd_topic", default_value="/hand_controller/commands",
                              description="hand_bridge 的指令话题（干跑时改成一次性话题，手不会动）"),
        DeclareLaunchArgument(
            "vive", default_value="true",
            description="设备那一步：vive_bringup（ZMQ 位姿 -> vive_pose_zmq_node -> vive_teleop_node）"),
        LogInfo(msg="Vive 遥操 = teleop_base（基础层）+ 手套桥/zmq_server + vive_bringup"),
        LogInfo(msg="开始相对控制臂：ros2 param set /vive_dual_arm_teleop_node teleop_enabled true"),
        base,
        TimerAction(period=14.0, actions=[hand_bridge]),
        TimerAction(period=16.0, actions=[zmq_server]),
        TimerAction(period=20.0, actions=[vive]),
    ])
