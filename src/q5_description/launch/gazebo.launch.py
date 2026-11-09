"""Gazebo classic 里看模型：世界 + robot_state_publisher + spawn。

    ros2 launch q5_description gazebo.launch.py                 # 带 gzclient 窗口
    ros2 launch q5_description gazebo.launch.py gui:=false      # 无头，配合截图脚本
    ros2 launch q5_description gazebo.launch.py urdf:=q5_reference.urdf

注意两件事：
1. 这套描述目前**没有碰撞几何**，所以默认用零重力世界，模型会停在原地而不是穿过地面。
   要在有重力的世界里站着/走，得先补 <collision>。
2. Gazebo classic 把 URDF 转 SDF 时会把 `package://q5_description/...` 改写成
   `model://q5_description/...`，所以必须把包的 share 目录挂到 GAZEBO_MODEL_PATH，
   否则所有网格都会以 "URI not supported by Fuel / No mesh specified" 静默丢掉
   （只剩 cylinder/box 提示块）。下面那一行就是干这个的。
"""
import os
import re
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction,
                            UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF_CHOICES = ("q5_reference_xhand.urdf", "q5_reference.urdf")


def _share():
    return Path(get_package_share_directory("q5_description"))


def _read_urdf(urdf_path):
    """读 URDF 文本并去掉 XML 声明。

    robot_description 是一个字符串参数；带 `<?xml ...?>` 声明时，lxml 系的工具
    （gazebo_ros 的 spawn_entity.py -topic）会直接抛
    "Unicode strings with encoding declaration are not supported"。
    """
    text = Path(urdf_path).read_text(encoding="utf-8")
    if text.lstrip().startswith("<?xml"):
        text = text.split("?>", 1)[1].lstrip()
    return text


def _static_sdf(urdf_path, cache_dir):
    """URDF -> SDF，并在 <model> 里插入 <static>true</static>。

    为什么要静态：这套 URDF 带官方质量/惯量（底盘 44.9 kg，指节惯量到 1e-5 量级），
    Gazebo classic 的关节求解在这个量级组合下发散——实测模型几帧内被拉塌到原点，
    屏幕上只剩一坨底盘。静态模型完全不进物理，渲染的就是 URDF 的零位姿，
    正是"在仿真里看模型"想要的。代价：关节不会动，别拿它验证动力学。
    """
    out = Path(cache_dir) / f"{Path(urdf_path).stem}_static.sdf"
    text = subprocess.run(["gz", "sdf", "-p", str(urdf_path)],
                          capture_output=True, text=True, check=True).stdout
    out.write_text(re.sub(r"(<model[^>]*>)", r"\1\n    <static>true</static>", text, count=1),
                   encoding="utf-8")
    return out


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    urdf_name = LaunchConfiguration("urdf").perform(context)
    urdf_path = _share() / "urdf" / urdf_name
    if not urdf_path.is_file():
        raise RuntimeError(f"{urdf_path} 不存在")
    robot_description = _read_urdf(urdf_path)
    static = LaunchConfiguration("static").perform(context).strip().lower() in ("1", "true", "yes")
    software_gl = (LaunchConfiguration("software_gl").perform(context).strip().lower()
                   in ("1", "true", "yes"))
    # 这台 WSL 有 WSLg 的 d3d12 GPU 驱动，硬件渲染 gzclient 只要 ~34% CPU，
    # 而 LIBGL_ALWAYS_SOFTWARE=1 会把它按到 llvmpipe，实测 gzclient 吃 400% CPU、
    # 操作明显卡顿。所以默认主动清掉这个变量（哪怕父 shell 或老脚本里 export 过）。
    gl_environment = ([SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")] if software_gl
                      else [UnsetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE")])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"world": world, "verbose": "true", "gui": gui}.items(),
    )
    # install/share —— 装了 q5_description 的那一层，model://q5_description/... 靠它解析。
    share_root = Path(get_package_share_directory("q5_description")).parent
    gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.pathsep.join(filter(None, [str(share_root), os.environ.get("GAZEBO_MODEL_PATH", "")])),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    # static 走 SDF 文件（<static> 只认 SDF）；否则用 -topic 吃 /robot_description。
    spawn_arguments = ["-entity", "q5", "-z", LaunchConfiguration("z")]
    if static:
        spawn_arguments += ["-file", str(_static_sdf(urdf_path, Path("/tmp")))]
    else:
        spawn_arguments += ["-topic", "robot_description"]
    spawn = TimerAction(period=6.0, actions=[Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=spawn_arguments,
        output="screen")])

    return (gl_environment + [gazebo_model_path, gazebo, robot_state_publisher,
                              joint_state_publisher, spawn])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=URDF_CHOICES[0], choices=list(URDF_CHOICES)),
        DeclareLaunchArgument("world", default_value=str(_share() / "worlds" / "q5_description.world")),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("z", default_value="0.08",
                              description="base_link 的 spawn 高度；模型最低点在 -0.077 m，所以 0.08 刚好贴地"),
        DeclareLaunchArgument("static", default_value="true",
                              description="true=生成静态 SDF 后 spawn（不参与物理，模型不会塌；"
                                          "关节不动）。false=按 URDF 动态生成，实测会被求解器拉塌"),
        DeclareLaunchArgument("software_gl", default_value="false",
                              description="true=强制 LIBGL_ALWAYS_SOFTWARE（无 GPU/无 WSLg 时才需要），"
                                          "false=清掉该变量走 GPU，卡顿会明显改善"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
