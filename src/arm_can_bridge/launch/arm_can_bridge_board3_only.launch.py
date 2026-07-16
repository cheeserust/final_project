"""Launch the arm CAN bridge for Board3-only gripper testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create launch description for Board3-only gripper control."""
    parameter_file = PathJoinSubstitution([
        FindPackageShare('arm_can_bridge'),
        'config',
        'arm_can_bridge.yaml',
    ])
    retry_timeout_file = PathJoinSubstitution([
        FindPackageShare('arm_can_bridge'),
        'config',
        'retry_timeout.yaml',
    ])

    can_interface = LaunchConfiguration('can_interface')
    execution_mode = LaunchConfiguration('execution_mode')
    target_load = LaunchConfiguration('target_load')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='SocketCAN interface name, e.g. can0 or vcan0.',
        ),
        DeclareLaunchArgument(
            'execution_mode',
            default_value='plan_only',
            description='plan_only rejects motion; hardware enables it.',
        ),
        DeclareLaunchArgument(
            'target_load',
            default_value='500',
            description='Default Board3 gripper target load raw value.',
        ),
        Node(
            package='arm_can_bridge',
            executable='arm_can_bridge_node',
            name='arm_can_bridge',
            output='screen',
            parameters=[
                parameter_file,
                retry_timeout_file,
                {
                    'can_interface': can_interface,
                    'execution_mode': execution_mode,
                    'enable_arm': False,
                    'enable_gripper': True,
                    'gripper_target_load_raw': ParameterValue(
                        target_load,
                        value_type=int,
                    ),
                    'board3_inter_frame_delay_ms': 3.0,
                },
            ],
        ),
    ])
