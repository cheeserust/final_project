"""Launch the standalone central-PC presentation node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    start_web = LaunchConfiguration('start_web')
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_web',
            default_value='true',
            description='Start the built-in web UI and REST API',
        ),
        Node(
            package='final_project_presentation2',
            executable='final_project_presentation2',
            name='final_project_presentation2',
            output='screen',
            parameters=[{
                'start_web': ParameterValue(start_web, value_type=bool),
            }],
        ),
    ])
