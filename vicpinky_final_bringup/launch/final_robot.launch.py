#!/usr/bin/env python3

"""Start final Pinky services while treating the team base launch as opaque."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose untouched Pinky bringup with final mission support nodes."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_base = LaunchConfiguration('launch_base')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_task_servers = LaunchConfiguration('launch_task_servers')
    launch_front_camera = LaunchConfiguration('launch_front_camera')
    launch_rear_camera = LaunchConfiguration('launch_rear_camera')
    front_video_device = LaunchConfiguration('front_video_device')
    rear_video_device = LaunchConfiguration('rear_video_device')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')

    base_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vicpinky_bringup'),
            'launch',
            'bringup.launch.xml',
        ])),
        condition=IfCondition(launch_base),
    )

    nav2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vicpinky_navigation'),
            'launch',
            'bringup_launch.xml',
        ])),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    task_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vicpinky_task_servers'),
            'launch',
            'task_servers.launch.py',
        ])),
        condition=IfCondition(launch_task_servers),
    )

    front_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='front_camera',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': front_video_device,
            'camera_frame_id': 'front_camera_optical_frame',
        }],
        condition=IfCondition(launch_front_camera),
    )

    rear_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='rear_camera',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': rear_video_device,
            'camera_frame_id': 'rear_camera_optical_frame',
        }],
        condition=IfCondition(launch_rear_camera),
    )

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_base', default_value='true'),
        DeclareLaunchArgument('launch_nav2', default_value='true'),
        DeclareLaunchArgument('launch_task_servers', default_value='true'),
        DeclareLaunchArgument('launch_front_camera', default_value='true'),
        DeclareLaunchArgument('launch_rear_camera', default_value='true'),
        DeclareLaunchArgument('front_video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('rear_video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('vicpinky_task_servers'),
                'maps',
                '4f.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('vicpinky_navigation'),
                'params',
                'nav2_params.yaml',
            ]),
        ),
    ]

    return LaunchDescription(arguments + [
        base_launch,
        front_camera,
        rear_camera,
        nav2_launch,
        task_launch,
    ])
