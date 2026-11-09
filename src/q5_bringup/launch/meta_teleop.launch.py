#!/usr/bin/env python3
"""Meta(Quest) 遥操 = 基础层（teleop_base）+ 设备那一步（XOS 的 webxr / retargetx）。

和 vive 那条链路的区别（依据 9/16 记录）：
  * 走 **XOS 官方栈**：`start_webxr`（8010 WebXR 输入流，头显里看画面 + 手柄/手部跟踪）
    + `start_retargetx`（重定向到臂/手），都由官方客户端 `q5_pub_client.py` 驱动。
  * **不用** PC 侧 ZMQ 位姿链（vive_pose_zmq_node / vive_teleop_node），
    **也不用**手套桥 hand_bridge 与 zmq_server —— 那两个是 vive 链路用的
    （Meta 的手来自头显手部跟踪，`teleopX_node` 直接发 `/hand_controller/commands`）。
    9/18 起那两个节点已从 teleop_base 挪到 vive_teleop.launch.py，本文件不再涉及。
  * 数采口径不变：`/joint_states_relay` + 相机 + 两条命令话题，`record` 照旧能用。

顺序有讲究（9/16 踩过，写进 SOP）：
  ① 头显浏览器连 **8010** → ② 1888 点【开始遥操作】（= start_teleop_dependency，
  要求"VR 端已连上"，否则报 `Please connect to robot first!`）→ ③ 8010 点 START AR

用法：
    ros2 launch q5_bringup meta_teleop.launch.py                    # 基础层 + webxr + retargetx
    ros2 launch q5_bringup meta_teleop.launch.py vr:=false          # 只起基础层
    ros2 launch q5_bringup meta_teleop.launch.py webxr_camera:=realsense   # 头显要真实画面
开关只有 **Meta 专属**的 vr / webxr_camera / hand / mocap —— 基础层（9/18 起）没有开关。

相机类型（手册 §9：无 / 实感深度 / 双目）：
    dummy      不用相机。**默认值** —— D455 留给 ROS realsense 供数采。
               官方手册 §7.3 明确：『realsense-viewer 与 XOS 遥操作同时用摄像头时，
               XOS 启动须选无摄像头』。实测两个进程抢 D455 会 errno=16 EBUSY，
               并可能把相机搞成 WEDGED（见 tools_robot/usb_reset_camera.py）。
    realsense  用 D455。头显能看真实画面，但和 ROS realsense 冲突，
               数采的 image_head(/camera/camera/color/image_raw/compressed) 会拿不到。
    stereo     用双 RGB PayCam。⚠️ 该相机在本机(NUC)看不到 ——
               /dev/video0..5 全部属于 D455，lsusb 里没有 PayCam。
               手册说它在 /dev/video0、video1，与实测不符，估计接在主控侧。未验证。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

PY = "/usr/bin/python"
Q5_PUB = "/home/developer/teleop_client/q5_pub_client.py"


def generate_launch_description():
    vr = IfCondition(LaunchConfiguration("vr"))
    # 基础层：不传 launch_arguments —— CLI 参数是全局配置，被 include 的文件自己能看见。
    # 基础层（9/18 起）**没有开关**，所以本文件不再重复声明 check/activate/mpc/camera/relay。
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("q5_bringup"), "/launch/teleop_base.launch.py"]),
    )

    # 设备那一步：官方客户端的两种子命令（mpc 那步在 teleop_base 里已经起了）。
    # cmd 里要拼 LaunchConfiguration，用嵌套列表（launch 会把它当"替换拼接"处理）
    webxr = ExecuteProcess(
        cmd=["bash", "-c", [f"{PY} {Q5_PUB} --type webxr --cmd start --camera-type ",
                            LaunchConfiguration("webxr_camera"),
                            " 2>&1 | tail -2"]],
        name="webxr", output="screen", condition=vr)
    retargetx = ExecuteProcess(
        cmd=["bash", "-c", [f"{PY} {Q5_PUB} --type retargetx --cmd start --hand ",
                            LaunchConfiguration("hand"),
                            " --mocap ", LaunchConfiguration("mocap"),
                            " 2>&1 | tail -2"]],
        name="retargetx", output="screen", condition=vr)

    return LaunchDescription([
        # 基础层无开关，这里只声明 **Meta 专属**的（vr / webxr_camera / hand / mocap）
        DeclareLaunchArgument("vr", default_value="true",
                              description="设备那一步：起 XOS 的 webxr + retargetx（头显 8010 那套）"),
        DeclareLaunchArgument("webxr_camera", default_value="dummy",
                              description="webxr 视频源：dummy(无) | realsense(实感深度) | stereo(双目)。"
                                          "默认 dummy：D455 留给 ROS realsense 供数采，避免两边抢同一个相机"
                                          "（官方手册 §7.3：realsense-viewer 与 XOS 同用时 XOS 须选『无摄像头』）。"
                                          "需要头显看真实画面再显式传 realsense，代价是数采拿不到 image_head"),
        DeclareLaunchArgument("hand", default_value="xhand", description="retargetx 的手：xhand | lite"),
        DeclareLaunchArgument("mocap", default_value="vr", description="retargetx 的动捕源：vr | gamepad"),
        LogInfo(msg="Meta 遥操 = teleop_base（基础层，无开关）+ XOS 的 webxr / retargetx"),
        LogInfo(msg="SOP：① 头显浏览器连 8010 → ② 1888 点【开始遥操作】→ ③ 8010 点 START AR"
                    "（顺序不能反，否则报 Please connect to robot first!）"),
        LogInfo(msg="提示：Meta 用手部跟踪（teleopX_node 发 /hand_controller/commands），"
                    "手套桥与 zmq_server 是 vive 专属，本链路不涉及"),
        # 使能（开始遥操作）刻意**不自动执行**：必须等头显连上 8010 并点了 Start AR 之后由人触发，
        # 而且它会让机械臂开始跟随 —— 属于物理安全动作。与 vive 那条（teleop_enabled 提示）同款处理。
        LogInfo(msg="使能遥操（头显 Start AR 之后）：ros2 service call /Start_EE_Retarget std_srvs/srv/Trigger"
                    "   或 " + PY + " " + Q5_PUB + " --cmd start_teleop"),
        LogInfo(msg="退出遥操：1888 页面【退出遥操作】，或 " + PY + " " + Q5_PUB + " --type retargetx --cmd stop"),
        base,
        TimerAction(period=20.0, actions=[webxr]),
        TimerAction(period=24.0, actions=[retargetx]),
    ])
