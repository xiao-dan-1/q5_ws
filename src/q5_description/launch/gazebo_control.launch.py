"""在 Gazebo 里真的驱动 Q5：DART 世界 + ros2_control 控制器。

    ros2 launch q5_description gazebo_control.launch.py

起来之后（另开一个终端，记得先 source）：

    # 看关节角
    ros2 topic echo /joint_states --once

    # 抬左臂：/q5_position_controller 按 yaml 里的关节顺序收一个 Float64MultiArray
    ros2 topic pub --once /q5_position_controller/commands \\
        std_msgs/msg/Float64MultiArray "{data: [38 个数]}"

    # 底盘前后走（rad/s）
    ros2 topic pub --rate 10 /q5_wheel_controller/commands \\
        std_msgs/msg/Float64MultiArray "{data: [2.0, -2.0]}"

为什么必须用 DART 世界：官方质量/惯量在 ODE 的关节求解里会发散、模型几帧内塌成一坨
（见 worlds/q5_control.world 的注释）。
"""
import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction, UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLERS = ("joint_state_broadcaster", "q5_position_controller", "q5_wheel_controller",
               "q5_ankle_effort_controller")


def _share():
    return Path(get_package_share_directory("q5_description"))


def _expand_xacro(xacro_path, cache_dir):
    """xacro 展开成一份没有 XML 声明的 URDF 文件。

    为什么要落文件：spawn_entity.py 把描述当字符串喂给 lxml，带 `<?xml ...?>` 声明
    会直接抛 "Unicode strings with encoding declaration"（-topic 和 -file 都一样）。
    """
    out = Path(cache_dir) / f"{Path(xacro_path).stem}.urdf"
    text = subprocess.run(["xacro", str(xacro_path)], capture_output=True, text=True,
                          check=True).stdout
    if text.lstrip().startswith("<?xml"):
        text = text.split("?>", 1)[1].lstrip()
    out.write_text(text, encoding="utf-8")
    return out


def _setup(context, *args, **kwargs):
    xacro = _share() / "urdf" / "q5_control.urdf.xacro"
    expanded = _expand_xacro(xacro, Path("/tmp"))
    robot_description = expanded.read_text(encoding="utf-8")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world"),
                          "verbose": "true", "gui": LaunchConfiguration("gui"),
                          "pause": LaunchConfiguration("start_paused")}.items(),
    )
    # 同 gazebo.launch.py：URDF→SDF 会把 package:// 改写成 model://，必须把包的
    # share 目录挂进 GAZEBO_MODEL_PATH，否则网格被静默丢弃（gzclient 里一片
    # "No mesh specified"，模型看起来像一坨）。
    share_root = Path(get_package_share_directory("q5_description")).parent
    gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.pathsep.join(filter(None, [str(share_root), os.environ.get("GAZEBO_MODEL_PATH", "")])),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    # 这里不能用 -topic：gazebo_ros2_control 要求模型里带 ros2_control 标签，
    # 走 /robot_description 时 spawn_entity.py 用的是字符串接口，标签能带过去；
    # 但 xacro 展开后的 XML 声明会被 lxml 拒，所以先落一份文件再 -file。
    spawn = TimerAction(period=6.0, actions=[Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", "q5", "-file", str(expanded), "-z", "0.08"],
        output="screen")])
    # controller_manager 由 gazebo_ros2_control 插件起，等它就绪再 spawn 控制器
    spawners = TimerAction(period=16.0, actions=[
        Node(package="controller_manager", executable="spawner", arguments=[name],
             output="screen") for name in CONTROLLERS])
    # 平衡控制器：必须在模型落地之前就在跑。否则底盘后缘一碰到地就把俯仰锁死
    # （后缘+两轮=三点硬接触），之后无论关节挪重心还是驱动轮给扭矩都掰不回来。
    balance = TimerAction(period=12.0, actions=[Node(
        package="q5_description", executable="straight_drive.py",
        arguments=["--speed", "0", "--seconds", "0"], output="screen")])

    return [
        # 软件渲染会把 gzclient 从 ~33% CPU 顶到 ~400%，默认清掉（见 gazebo.launch.py）
        UnsetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE"),
        gazebo_model_path,
        gazebo,
        robot_state_publisher,
        spawn,
        spawners,
        balance,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=str(_share() / "worlds" / "q5_control.world"),
                              description="物理世界；默认 DART 零重力。换 ODE 做对照时传别的文件"),
        DeclareLaunchArgument("balance", default_value="true",
                              description="随仿真启动俯仰平衡控制器"),
        DeclareLaunchArgument("start_paused", default_value="false",
                              description="以暂停状态启动：模型和控制器都就绪后再用 "
                                          "`gz world -p 0` 放开，平衡控制器才能在落地那一刻接住"),
        OpaqueFunction(function=_setup),
    ])
