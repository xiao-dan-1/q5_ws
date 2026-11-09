from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="q5_policy_runtime",
            executable="policy_node",
            name="q5_policy",
            output="screen",
        ),
    ])
