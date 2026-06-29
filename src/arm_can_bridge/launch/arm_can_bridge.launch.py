"""Launch the Board1 arm CAN bridge node."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create launch description for arm_can_bridge."""
    parameter_file = PathJoinSubstitution([
        FindPackageShare('arm_can_bridge'),
        'config',
        'arm_can_bridge.yaml',
    ])

    return LaunchDescription([
        Node(
            package='arm_can_bridge',
            executable='arm_can_bridge_node',
            name='arm_can_bridge',
            output='screen',
            parameters=[parameter_file],
        )
    ])
