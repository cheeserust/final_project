"""Backward-compatible single-machine ArUco task launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Include perception and task execution for local debugging."""
    package_share = FindPackageShare('roscue_arm_pick')
    return LaunchDescription([
        DeclareLaunchArgument('launch_camera', default_value='false'),
        DeclareLaunchArgument('execution_mode', default_value='plan_only'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    package_share,
                    'launch',
                    'aruco_perception.launch.py',
                ])
            ),
            launch_arguments={
                'launch_camera': LaunchConfiguration('launch_camera'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    package_share,
                    'launch',
                    'arm_task_executor.launch.py',
                ])
            ),
            launch_arguments={
                'execution_mode': LaunchConfiguration('execution_mode'),
            }.items(),
        ),
    ])
