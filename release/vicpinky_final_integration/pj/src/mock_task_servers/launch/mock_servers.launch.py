from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mock_task_servers',
            executable='mock_task_servers_node',
            name='mock_task_servers',
            output='screen'
        )
    ])
