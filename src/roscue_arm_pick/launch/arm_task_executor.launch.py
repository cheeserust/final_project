"""Launch the PC-side MoveIt task executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create a plan-only or hardware task executor."""
    package_share = FindPackageShare('roscue_arm_pick')
    execution_mode = LaunchConfiguration('execution_mode')

    def config(name):
        return PathJoinSubstitution([
            package_share,
            'config',
            name,
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'execution_mode',
            default_value='plan_only',
            description='plan_only rejects RunTask goals; hardware executes them.',
        ),
        Node(
            package='roscue_arm_pick',
            executable='task_executor_node',
            name='task_executor_node',
            output='screen',
            parameters=[{
                'aruco_targets_yaml': config('aruco_targets.yaml'),
                'fixed_poses_yaml': config('fixed_poses.yaml'),
                'task_sequence_yaml': config('task_sequence.yaml'),
                'gripper_profiles_yaml': config('gripper_profiles.yaml'),
                'task_waypoints_yaml': config('task_waypoints.yaml'),
                'execution_mode': execution_mode,
                'target_frame': 'base_link',
                'planning_group': 'arm',
                'grasp_planning_group': 'arm_grasp',
                'button_planning_group': 'arm_button',
                'pose_link': 'gripper_base_link',
                'grasp_pose_link': 'grasp_tcp_link',
                'button_pose_link': 'button_contact_link',
                'move_action_name': '/move_action',
            }],
        ),
    ])
