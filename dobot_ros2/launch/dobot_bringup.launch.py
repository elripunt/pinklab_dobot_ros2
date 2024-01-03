from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package = 'dobot_ros2',
                 executable = 'dashboard_server',
                 output = 'screen',
            ),
            Node(package = 'dobot_ros2',
                 executable = 'move_server',
                 output = 'screen'
            ),
        ]
    )