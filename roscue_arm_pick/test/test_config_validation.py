"""Tests for cross-package arm safety configuration validation."""

from pathlib import Path

from roscue_arm_pick.config_validation import (
    load_urdf_limits,
    load_yaml,
    validate_configuration,
)


WORKSPACE_SRC = Path(__file__).resolve().parents[2]


def test_project_configuration_detects_uncalibrated_home_values():
    package = WORKSPACE_SRC / 'roscue_arm_pick'
    bridge = WORKSPACE_SRC / 'arm_can_bridge'
    moveit = WORKSPACE_SRC / 'roscue_arm_moveit_config'
    description = WORKSPACE_SRC / 'roscue_arm_description'

    report = validate_configuration(
        fixed_config=load_yaml(str(package / 'config/fixed_poses.yaml')),
        gripper_config=load_yaml(
            str(package / 'config/gripper_profiles.yaml')
        ),
        bridge_config=load_yaml(
            str(bridge / 'config/arm_can_bridge.yaml')
        ),
        moveit_limits=load_yaml(
            str(moveit / 'config/joint_limits.yaml')
        ),
        urdf_limits=load_urdf_limits([
            str(description / 'urdf/roscue_arm.urdf.xacro'),
            str(
                description
                / 'urdf/assemblies/roscue_arm.urdf.xacro'
            ),
        ]),
    )

    assert report.ok is False
    assert any(
        'bridge_home.arm_joint_1' in error
        for error in report.errors
    )
    assert any(
        'calibration.complete is false' in warning
        for warning in report.warnings
    )


def test_gripper_profiles_stay_inside_bridge_limits():
    package = WORKSPACE_SRC / 'roscue_arm_pick'
    bridge = load_yaml(
        str(WORKSPACE_SRC / 'arm_can_bridge/config/arm_can_bridge.yaml')
    )['arm_can_bridge']['ros__parameters']
    profiles = load_yaml(str(package / 'config/gripper_profiles.yaml'))
    names = bridge['gripper_joint_names']
    minimums = dict(zip(names, bridge['gripper_min_positions_rad']))
    maximums = dict(zip(names, bridge['gripper_max_positions_rad']))

    for profile in profiles['objects'].values():
        for joint, position in zip(names, profile['gripper_close_rad']):
            assert minimums[joint] <= position <= maximums[joint]
