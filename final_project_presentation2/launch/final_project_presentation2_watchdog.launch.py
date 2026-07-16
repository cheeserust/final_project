"""Launch only the Pinky-side raw velocity watchdog."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='final_project_presentation2',
            executable='final_project_presentation2_watchdog',
            name='final_project_presentation2_watchdog',
            output='screen',
            respawn=True,
            respawn_delay=0.25,
        ),
    ])
