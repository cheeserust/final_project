"""Launch the arm CAN bridge for Board1-only 3-axis arm + base control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create launch description for Board1-only 4-axis arm control."""
    parameter_file = PathJoinSubstitution([
        FindPackageShare('arm_can_bridge'),
        'config',
        'arm_can_bridge.yaml',
    ])

    can_interface = LaunchConfiguration('can_interface')
    execution_mode = LaunchConfiguration('execution_mode')

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
        Node(
            package='arm_can_bridge',
            executable='arm_can_bridge_node',
            name='arm_can_bridge',
            output='screen',
            parameters=[
                parameter_file,
                {
                    'can_interface': can_interface,
                    'execution_mode': execution_mode,
                    'enable_gripper': False,
                    'arm_joint_names': [
                        'arm_joint_1',
                        'arm_joint_2',
                        'arm_joint_3',
                        'base_joint',
                    ],
                    'arm_board_ids': [1, 1, 1, 1],
                    'arm_motor_ids': [0, 1, 2, 3],
                    'arm_min_positions_rad': [
                        -1.50970980,
                        -1.36310215,
                        -1.59697627,
                        -1.57079633,
                    ],
                    'arm_max_positions_rad': [
                        1.57079633,
                        1.39626340,
                        1.57079633,
                        3.14159265,
                    ],
                    'arm_home_positions_rad': [
                        -1.50970980,
                        -1.36310215,
                        -1.59697627,
                        -1.57079633,
                    ],
                    'arm_raw_position_signs': [1, 1, 1, 1],
                    'arm_raw_position_offsets_rad': [
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    ],
                    'arm_command_min_angle_raw': [
                        -8500, -7810, -9150, -9000,
                    ],
                    'arm_command_max_angle_raw': [
                        9000, 8000, 9000, 18000,
                    ],
                    'fixed_joint_state_names': [
                        'arm_joint_4',
                    ],
                    'fixed_joint_state_positions_rad': [
                        0.0,
                    ],
                    'required_homing_mask': 0x0F,
                    'queue_capacity': 28,
                    'board1_queue_capacity': 124,
                    'arm_trajectory_point_duration_ticks': 8,
                    'arm_trajectory_min_duration_ticks': 8,
                    'arm_max_ahead_points': 4,
                    'packed_position_feedback_board_ids': [1],
                    'axis_status_flags_board_ids': [1],
                },
            ],
        ),
    ])
