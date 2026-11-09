"""可视化：robot_state_publisher + 关节滑块 + RViz。

    ros2 launch q5_description display.launch.py
    ros2 launch q5_description display.launch.py urdf:=q5_reference.urdf rviz:=false

没装 joint_state_publisher_gui 时自动退回无 GUI 的 joint_state_publisher，
所以这个 launch 在最小安装下也能跑（只是没有滑块）。
"""
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_URDF = "q5_reference_xhand.urdf"
URDF_CHOICES = ("q5_reference_xhand.urdf", "q5_reference.urdf")


def _share():
    return Path(get_package_share_directory("q5_description"))


def _read_urdf(urdf_path):
    """见 gazebo.launch.py：字符串型的 robot_description 不要带 XML 声明。"""
    text = Path(urdf_path).read_text(encoding="utf-8")
    if text.lstrip().startswith("<?xml"):
        text = text.split("?>", 1)[1].lstrip()
    return text


def _has_joint_gui():
    try:
        get_package_share_directory("joint_state_publisher_gui")
        return True
    except PackageNotFoundError:
        return False


def _truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _setup(context, *args, **kwargs):
    urdf_name = LaunchConfiguration("urdf").perform(context)
    if urdf_name not in URDF_CHOICES:
        raise RuntimeError(f"urdf must be one of {URDF_CHOICES}, got {urdf_name!r}")
    urdf_path = _share() / "urdf" / urdf_name
    if not urdf_path.is_file():
        raise RuntimeError(f"{urdf_path} is missing — run scripts/import_model.py first")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": _read_urdf(urdf_path),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ]
    if _truthy(LaunchConfiguration("use_gui").perform(context)):
        if _has_joint_gui():
            nodes.append(Node(package="joint_state_publisher_gui",
                              executable="joint_state_publisher_gui",
                              name="joint_state_publisher_gui", output="screen"))
        else:
            # 没有 GUI 时仍然发一份全零关节状态，RViz 才不会抱怨 TF 缺失。
            nodes.append(Node(package="joint_state_publisher",
                              executable="joint_state_publisher",
                              name="joint_state_publisher", output="screen"))
    nodes.append(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", str(_share() / "rviz" / "q5_description.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=DEFAULT_URDF, choices=list(URDF_CHOICES)),
        DeclareLaunchArgument("use_gui", default_value="true",
                              description="关节滑块；缺 joint_state_publisher_gui 时自动退回无滑块模式"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
