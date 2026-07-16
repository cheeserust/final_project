"""Tests for cross-package arm safety configuration validation."""

from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree

from roscue_arm_pick.config_validation import (
    load_urdf_limits,
    load_yaml,
    validate_configuration,
)


WORKSPACE_SRC = Path(__file__).resolve().parents[2]


def test_project_configuration_is_hardware_ready():
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

    assert report.ok is True
    assert report.errors == ()
    assert report.warnings == ()


def test_project_uses_direct_arm_action_without_v2_stream_settings():
    params = load_yaml(
        str(WORKSPACE_SRC / 'arm_can_bridge/config/arm_can_bridge.yaml')
    )['arm_can_bridge']['ros__parameters']

    assert params['arm_action_name'] == '/arm_controller/execute_joint_goal'
    assert 'arm_inter_frame_delay_ms' not in params
    assert 'arm_trajectory_point_duration_ticks' not in params
    assert 'arm_max_ahead_points' not in params
    assert 'board1_queue_capacity' not in params
    assert 'board2_queue_capacity' not in params


def test_tcp_planning_groups_match_solver_and_ompl_configuration():
    moveit = WORKSPACE_SRC / 'roscue_arm_moveit_config/config'
    root = ElementTree.parse(moveit / 'roscue_arm.srdf').getroot()
    chains = {}
    for group in root.findall('group'):
        chain = group.find('chain')
        if chain is not None:
            chains[group.attrib['name']] = (
                chain.attrib['base_link'],
                chain.attrib['tip_link'],
            )

    expected = {
        'arm': ('arm_base_link', 'gripper_base_link'),
        'arm_grasp': ('arm_base_link', 'grasp_tcp_link'),
        'arm_button': ('arm_base_link', 'button_contact_link'),
    }
    assert chains == expected

    kinematics = load_yaml(str(moveit / 'kinematics.yaml'))
    ompl = load_yaml(str(moveit / 'ompl_planning.yaml'))
    for group_name in expected:
        assert kinematics[group_name]['kinematics_solver'] == (
            'kdl_kinematics_plugin/KDLKinematicsPlugin'
        )
        assert kinematics[group_name]['position_only_ik'] is True
        assert ompl[group_name]['default_planner_config'] == (
            'RRTConnectkConfigDefault'
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


def _project_validation_inputs():
    package = WORKSPACE_SRC / 'roscue_arm_pick'
    moveit = WORKSPACE_SRC / 'roscue_arm_moveit_config'
    description = WORKSPACE_SRC / 'roscue_arm_description'
    return {
        'fixed_config': load_yaml(str(package / 'config/fixed_poses.yaml')),
        'gripper_config': load_yaml(
            str(package / 'config/gripper_profiles.yaml')
        ),
        'bridge_config': load_yaml(
            str(WORKSPACE_SRC / 'arm_can_bridge/config/arm_can_bridge.yaml')
        ),
        'moveit_limits': load_yaml(
            str(moveit / 'config/joint_limits.yaml')
        ),
        'urdf_limits': load_urdf_limits([
            str(description / 'urdf/roscue_arm.urdf.xacro'),
            str(
                description
                / 'urdf/assemblies/roscue_arm.urdf.xacro'
            ),
        ]),
    }


def test_hardware_validation_fails_when_raw_command_limits_are_missing():
    inputs = _project_validation_inputs()
    broken = deepcopy(inputs['bridge_config'])
    del broken['arm_can_bridge']['ros__parameters'][
        'arm_command_min_angle_raw'
    ]
    inputs['bridge_config'] = broken

    report = validate_configuration(**inputs)

    assert report.ok is False
    assert 'arm CAN command raw limits are missing' in report.errors


def test_hardware_validation_rejects_direct_duration_over_uint16_ms():
    inputs = _project_validation_inputs()
    broken = deepcopy(inputs['fixed_config'])
    broken['arm_named_poses']['home']['duration_sec'] = 66.0
    inputs['fixed_config'] = broken

    report = validate_configuration(**inputs)

    assert report.ok is False
    assert any('duration must be 1..65535 ms' in error for error in report.errors)
