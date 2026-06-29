"""Launch the Board1 SocketCAN simulator."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description."""
    return LaunchDescription([
        Node(
            package='board1_simulator',
            executable='board1_simulator_node',
            name='board1_simulator',
            output='screen',
        )
    ])
