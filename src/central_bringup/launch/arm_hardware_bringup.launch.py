"""Launch the PC-side hardware MoveIt and CAN arm stack without fake control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from roscue_arm_pick.config_validation import (
    load_urdf_limits,
    load_yaml,
    validate_configuration,
)


def _include(package, launch_file, condition=None, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package),
                'launch',
                launch_file,
            ])
        ),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def _hardware_readiness_errors():
    pick_share = get_package_share_directory('roscue_arm_pick')
    bridge_share = get_package_share_directory('arm_can_bridge')
    moveit_share = get_package_share_directory('roscue_arm_moveit_config')
    description_share = get_package_share_directory(
        'roscue_arm_description'
    )
    fixed_config = load_yaml(os.path.join(
        pick_share,
        'config',
        'fixed_poses.yaml',
    ))
    report = validate_configuration(
        fixed_config=fixed_config,
        gripper_config=load_yaml(os.path.join(
            pick_share,
            'config',
            'gripper_profiles.yaml',
        )),
        bridge_config=load_yaml(os.path.join(
            bridge_share,
            'config',
            'arm_can_bridge.yaml',
        )),
        moveit_limits=load_yaml(os.path.join(
            moveit_share,
            'config',
            'joint_limits.yaml',
        )),
        urdf_limits=load_urdf_limits([
            os.path.join(
                description_share,
                'urdf',
                'roscue_arm.urdf.xacro',
            ),
            os.path.join(
                description_share,
                'urdf',
                'assemblies',
                'roscue_arm.urdf.xacro',
            ),
        ]),
    )
    errors = list(report.errors)
    if not bool(fixed_config.get('calibration', {}).get('complete', False)):
        errors.append('fixed_poses.yaml calibration.complete is false')
    return errors


def _launch_setup(context):
    execution_mode = LaunchConfiguration('execution_mode')
    mode = execution_mode.perform(context).strip().lower()
    if mode not in {'plan_only', 'hardware'}:
        raise RuntimeError(
            'execution_mode must be plan_only or hardware'
        )
    if mode == 'hardware':
        errors = _hardware_readiness_errors()
        if errors:
            raise RuntimeError(
                'Hardware bringup blocked before node startup: '
                + '; '.join(errors)
            )

    can_interface = LaunchConfiguration('can_interface')
    use_rviz = LaunchConfiguration('use_rviz')
    return [
        _include('roscue_arm_moveit_config', 'rsp.launch.py'),
        _include('roscue_arm_moveit_config', 'move_group.launch.py'),
        _include(
            'arm_can_bridge',
            'arm_can_bridge.launch.py',
            can_interface=can_interface,
            execution_mode=execution_mode,
        ),
        _include(
            'roscue_arm_pick',
            'arm_task_executor.launch.py',
            execution_mode=execution_mode,
        ),
        _include(
            'roscue_arm_moveit_config',
            'moveit_rviz.launch.py',
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    """Create the real-controller graph; never spawn ros2_control mocks."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='SocketCAN interface connected to STM32 boards.',
        ),
        DeclareLaunchArgument(
            'execution_mode',
            default_value='plan_only',
            description='Use hardware only after calibration validation passes.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz for optional visualization.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
