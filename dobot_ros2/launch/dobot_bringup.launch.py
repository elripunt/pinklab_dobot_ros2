import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    prarm_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
        get_package_share_directory('dobot_ros2'),
        'param',
        'dobot_param.yaml')
        )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
            'param_dir',
            default_value=prarm_dir
            ),
            Node(package = 'dobot_ros2',
                 executable = 'dashboard_server',
                 parameters=[prarm_dir],
                 output = 'screen',
            ),
        ]
    )