from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('mission_manager')
    mission_flow_file = LaunchConfiguration('mission_flow_file')
    locations_file = LaunchConfiguration('locations_file')
    action_servers_file = LaunchConfiguration('action_servers_file')
    launch_ready_and_approach = LaunchConfiguration(
        'launch_ready_and_approach'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_flow_file',
            default_value=PathJoinSubstitution([
                package_share,
                'config',
                'mission_flow.yaml',
            ]),
            description='Mission flow YAML loaded by mission_manager.',
        ),
        DeclareLaunchArgument(
            'locations_file',
            default_value=PathJoinSubstitution([
                package_share,
                'config',
                'locations.yaml',
            ]),
            description='Mission locations YAML.',
        ),
        DeclareLaunchArgument(
            'action_servers_file',
            default_value=PathJoinSubstitution([
                package_share,
                'config',
                'action_servers.yaml',
            ]),
            description='Action server profiles YAML.',
        ),
        DeclareLaunchArgument(
            'launch_ready_and_approach',
            default_value='true',
            description='Launch the arm/base coordination action server.',
        ),
        Node(
            package='mission_manager',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen',
            parameters=[{
                'mission_flow_file': mission_flow_file,
                'locations_file': locations_file,
                'action_servers_file': action_servers_file,
            }],
        ),
        Node(
            package='mission_manager',
            executable='ready_and_approach_coordinator',
            name='ready_and_approach_coordinator',
            output='screen',
            condition=IfCondition(launch_ready_and_approach),
        ),
    ])
