#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_dir = get_package_share_directory("roscue_arm_description")

    xacro_file = os.path.join(pkg_dir, "urdf", "Untitled.urdf.xacro")
    rviz_file = os.path.join(pkg_dir, "rviz", "display.rviz")

    prefix = LaunchConfiguration("prefix")

    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            xacro_file,
            " ",
            "prefix:=",
            prefix,
        ]),
        value_type=str,
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Joint prefix",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                rviz_file,
            ],
            output="screen",
        ),
    ])
