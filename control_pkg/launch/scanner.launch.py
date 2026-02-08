import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('control_pkg'),
        'config',
        'scanner_params.yaml'
    )

    scanner = Node(
            package='control_pkg',
            executable='scanner',
            name='scanner',
            parameters=[config],
            output='screen'
    )

    return LaunchDescription([
        scanner
    ])
