"""Launch the complete central-PC stack for the final VicPinky mission."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, **arguments):
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
    can_interface = LaunchConfiguration('can_interface')
    execution_mode = LaunchConfiguration('execution_mode')
    launch_depth_camera = LaunchConfiguration('launch_depth_camera')
    gui_host = LaunchConfiguration('gui_host')
    gui_port = LaunchConfiguration('gui_port')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='SocketCAN interface connected to the three STM boards.',
        ),
        DeclareLaunchArgument(
            'execution_mode',
            default_value='hardware',
            description='hardware for the robot; plan_only for safe inspection.',
        ),
        DeclareLaunchArgument(
            'launch_depth_camera',
            default_value='true',
            description='Start the PC-connected wrist RealSense.',
        ),
        DeclareLaunchArgument('gui_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('gui_port', default_value='8080'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        include(
            'central_bringup',
            'arm_hardware_bringup.launch.py',
            can_interface=can_interface,
            execution_mode=execution_mode,
            use_rviz=use_rviz,
        ),
        include(
            'roscue_arm_pick',
            'aruco_perception.launch.py',
            launch_camera=launch_depth_camera,
        ),
        # This is the only /nav/go_to server in the final deployment.
        include('vicpinky_nav_adapter', 'nav_adapter.launch.py'),
        include('mission_manager', 'mission_manager.launch.py'),
        include(
            'vicpinky_gui',
            'vicpinky_gui.launch.py',
            host=gui_host,
            port=gui_port,
            auto_port='false',
        ),
    ])
