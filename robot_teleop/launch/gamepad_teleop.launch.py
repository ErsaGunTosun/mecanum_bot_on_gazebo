from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_config = os.path.join(
        get_package_share_directory('robot_teleop'),
        'param',
        'joy.yaml'
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_config]
        )

    gamepad_teleop_node = Node(
            package='robot_teleop',
            executable='gamepad_teleop_node',
            name='gamepad_teleop_node',
            parameters=[joy_config]
        )


    return LaunchDescription([
        joy_node,
        gamepad_teleop_node,
    ])

