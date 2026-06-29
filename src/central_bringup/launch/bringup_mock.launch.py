from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock_servers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mock_task_servers'),
                'launch',
                'mock_servers.launch.py'
            ])
        )
    )

    mission_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mission_manager'),
                'launch',
                'mission_manager.launch.py'
            ])
        )
    )

    return LaunchDescription([
        mock_servers_launch,

        # mock action server들이 먼저 올라오도록 mission_manager를 약간 늦게 실행한다.
        TimerAction(
            period=1.0,
            actions=[
                mission_manager_launch
            ]
        ),
    ])