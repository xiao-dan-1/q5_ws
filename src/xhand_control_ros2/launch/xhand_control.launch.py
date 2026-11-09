import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_file_path = os.path.join(
        get_package_share_directory('xhand_control_ros2'),
        'config',
        'xhand_config.yaml'
    )

    xhand_control_node = Node(
        package='xhand_control_ros2',
        executable='xhand_control_ros2_node',
        name='xhand_control_ros2',
        namespace='xhand_control',
        output='screen',
        parameters=[config_file_path],
    )

    return LaunchDescription([
        xhand_control_node
    ])
