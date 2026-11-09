"""只起 robot_state_publisher（给别的节点当描述源用）。

    ros2 launch q5_description description.launch.py
    ros2 launch q5_description description.launch.py urdf:=q5_reference.urdf
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_URDF = "q5_reference_xhand.urdf"


def description_path(urdf_name):
    return Path(get_package_share_directory("q5_description")) / "urdf" / urdf_name


def read_urdf(urdf_path):
    """见 gazebo.launch.py：字符串型的 robot_description 不要带 XML 声明。"""
    text = Path(urdf_path).read_text(encoding="utf-8")
    if text.lstrip().startswith("<?xml"):
        text = text.split("?>", 1)[1].lstrip()
    return text


def generate_launch_description():
    urdf = LaunchConfiguration("urdf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_prefix = LaunchConfiguration("frame_prefix")
    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=DEFAULT_URDF,
                              description="urdf/ 下的文件名；q5_reference.urdf 为隐藏手指的图像对照版"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": read_urdf(description_path(DEFAULT_URDF)),
                "use_sim_time": use_sim_time,
                "frame_prefix": frame_prefix,
            }],
        ),
    ])
