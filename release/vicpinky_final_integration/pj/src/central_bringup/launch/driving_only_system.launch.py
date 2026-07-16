"""Launch the central-PC stack for the arm-free 4F/5F driving test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, **arguments):
    """Include a package launch file with forwarded arguments."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package),
                'launch',
                filename,
            ])
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    """Start Nav2 adaptation, driving-only mission control, and the GUI."""
    gui_host = LaunchConfiguration('gui_host')
    gui_port = LaunchConfiguration('gui_port')
    driving_flow = PathJoinSubstitution([
        FindPackageShare('mission_manager'),
        'config',
        'mission_flow_driving_only.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('gui_port', default_value='8080'),

        include('vicpinky_nav_adapter', 'nav_adapter.launch.py'),
        include(
            'mission_manager',
            'mission_manager.launch.py',
            mission_flow_file=driving_flow,
            launch_ready_and_approach='false',
        ),
        include(
            'vicpinky_gui',
            'vicpinky_gui.launch.py',
            host=gui_host,
            port=gui_port,
            auto_port='false',
            enable_manual_arm='false',
            enable_manual_gripper='false',
            mission_flow_file=driving_flow,
        ),
    ])
