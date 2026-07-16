"""Launch the VicPinky browser control dashboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the launch description."""
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    auto_port = LaunchConfiguration('auto_port')
    port_search_limit = LaunchConfiguration('port_search_limit')
    manual_arm_mode = LaunchConfiguration('manual_arm_mode')
    enable_manual_arm = LaunchConfiguration('enable_manual_arm')
    enable_manual_gripper = LaunchConfiguration('enable_manual_gripper')
    mission_flow_file = LaunchConfiguration('mission_flow_file')
    map_topic = LaunchConfiguration('map_topic')
    amcl_pose_topic = LaunchConfiguration('amcl_pose_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    global_path_topic = LaunchConfiguration('global_path_topic')
    local_path_topic = LaunchConfiguration('local_path_topic')

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
        DeclareLaunchArgument(
            'manual_arm_mode',
            default_value='full',
            description='Manual arm joint set: full, board1, or board2.',
        ),
        DeclareLaunchArgument(
            'enable_manual_arm',
            default_value='true',
            description='Show and enable manual arm controls.',
        ),
        DeclareLaunchArgument(
            'enable_manual_gripper',
            default_value='true',
            description='Show and enable manual gripper controls.',
        ),
        DeclareLaunchArgument(
            'mission_flow_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission_manager'),
                'config',
                'mission_flow.yaml',
            ]),
            description='Mission flow YAML displayed by the dashboard.',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='OccupancyGrid topic used by the driving map panel.',
        ),
        DeclareLaunchArgument(
            'amcl_pose_topic',
            default_value='/amcl_pose',
            description='AMCL pose topic used by the driving map panel.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic used by the driving map panel.',
        ),
        DeclareLaunchArgument(
            'global_path_topic',
            default_value='/plan',
            description='Global path topic used by the driving map panel.',
        ),
        DeclareLaunchArgument(
            'local_path_topic',
            default_value='/local_plan',
            description='Local path topic used by the driving map panel.',
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
                'manual_arm_mode': manual_arm_mode,
                'enable_manual_arm': ParameterValue(
                    enable_manual_arm,
                    value_type=bool,
                ),
                'enable_manual_gripper': ParameterValue(
                    enable_manual_gripper,
                    value_type=bool,
                ),
                'mission_flow_file': mission_flow_file,
                'map_topic': map_topic,
                'amcl_pose_topic': amcl_pose_topic,
                'odom_topic': odom_topic,
                'global_path_topic': global_path_topic,
                'local_path_topic': local_path_topic,
            }],
        ),
    ])
