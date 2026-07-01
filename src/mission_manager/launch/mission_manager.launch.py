from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('mission_manager')

    return LaunchDescription([
        Node(
            package='mission_manager',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen',
            parameters=[{
                'mission_flow_file': PathJoinSubstitution([
                    package_share,
                    'config',
                    'mission_flow.yaml',
                ]),
                'locations_file': PathJoinSubstitution([
                    package_share,
                    'config',
                    'locations.yaml',
                ]),
                'action_servers_file': PathJoinSubstitution([
                    package_share,
                    'config',
                    'action_servers.yaml',
                ]),
            }],
        )
    ])
