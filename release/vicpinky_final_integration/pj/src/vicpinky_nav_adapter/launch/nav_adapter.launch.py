from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('vicpinky_nav_adapter'),
            'config',
            'nav_adapter.yaml',
        ]
    )

    return LaunchDescription(
        [
            Node(
                package='vicpinky_nav_adapter',
                executable='nav_adapter_node',
                name='vicpinky_nav_adapter',
                output='screen',
                parameters=[config_file],
            ),
        ]
    )
