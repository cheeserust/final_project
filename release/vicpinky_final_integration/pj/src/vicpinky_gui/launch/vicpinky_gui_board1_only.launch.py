"""Launch the VicPinky dashboard for Board1-only base + 3-axis arm testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the Board1-only 4-axis GUI launch description."""
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    auto_port = LaunchConfiguration('auto_port')
    port_search_limit = LaunchConfiguration('port_search_limit')

    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='0.0.0.0',
            description='HTTP bind address',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='HTTP port',
        ),
        DeclareLaunchArgument(
            'auto_port',
            default_value='true',
            description='Use the next free HTTP port if the requested port is busy',
        ),
        DeclareLaunchArgument(
            'port_search_limit',
            default_value='20',
            description='Number of consecutive ports to try when auto_port is true',
        ),
        Node(
            package='vicpinky_gui',
            executable='vicpinky_gui_node',
            name='vicpinky_gui',
            output='screen',
            parameters=[{
                'host': host,
                'port': ParameterValue(port, value_type=int),
                'auto_port': ParameterValue(auto_port, value_type=bool),
                'port_search_limit': ParameterValue(
                    port_search_limit,
                    value_type=int,
                ),
                'manual_arm_mode': 'board1',
                'enable_manual_arm': True,
                'enable_manual_gripper': False,
            }],
        ),
    ])
