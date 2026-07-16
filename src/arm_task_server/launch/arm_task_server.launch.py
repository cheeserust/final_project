"""Launch the semantic arm task server."""

from launch import LaunchDescription
from launch.actions import LogWarning
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the arm task server launch description."""
    return LaunchDescription([
        LogWarning(
            msg=(
                'arm_task_server is deprecated; do not run it with '
                'roscue_arm_pick because /arm/* actions will conflict.'
            )
        ),
        Node(
            package='arm_task_server',
            executable='arm_task_server_node',
            name='arm_task_server',
            output='screen',
        ),
    ])
