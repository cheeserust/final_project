"""Unit tests for arm task profile helpers."""

from pathlib import Path

from arm_task_server.arm_task_server_node import (
    ArmTaskServer,
    ControllerConfig,
    parse_extra_json,
    task_key_for_goal,
)
import yaml


def test_parse_extra_json_returns_mapping():
    assert parse_extra_json('{"button_role": "floor_select"}') == {
        'button_role': 'floor_select',
    }


def test_parse_extra_json_ignores_invalid_payloads():
    assert parse_extra_json('not json') == {}
    assert parse_extra_json('[1, 2, 3]') == {}


def test_task_key_dispatches_button_roles():
    server_config = {
        'task': 'press_elevator_call_button',
        'dispatch_by_button_role': {
            'elevator_call': 'press_elevator_call_button',
            'floor_select': 'press_floor_button',
        },
    }

    assert task_key_for_goal(
        'press_button',
        server_config,
        {'button_role': 'floor_select'},
    ) == 'press_floor_button'

    assert task_key_for_goal(
        'press_button',
        server_config,
        {'button_role': 'elevator_call'},
    ) == 'press_elevator_call_button'


def test_task_key_uses_default_without_dispatch_match():
    assert task_key_for_goal(
        'pick',
        {'task': 'pick'},
        {},
    ) == 'pick'


def test_default_config_steps_reference_existing_poses():
    config_path = Path(__file__).parents[1] / 'config' / 'arm_tasks.yaml'
    config = yaml.safe_load(config_path.read_text())

    for task_config in config['tasks'].values():
        for step in task_config['steps']:
            if 'wait_sec' in step:
                continue

            controller_name = step['controller']
            pose_name = step['pose']
            assert controller_name in config['controllers']
            assert pose_name in config['poses'][controller_name]

            joint_count = len(config['controllers'][controller_name]['joints'])
            pose_count = len(
                config['poses'][controller_name][pose_name]['degrees']
            )
            assert pose_count == joint_count


def test_make_controller_goal_sets_gripper_effort_target_load():
    controller = ControllerConfig(
        name='gripper',
        action_name='/gripper_controller/follow_joint_trajectory',
        joint_names=('finger_a', 'finger_b'),
        default_duration_sec=1.0,
    )

    goal = ArmTaskServer._make_controller_goal(
        controller,
        positions_rad=(0.1, 0.2),
        duration_sec=1.25,
        target_load_raw=500,
    )

    point = goal.trajectory.points[0]
    assert goal.trajectory.joint_names == ['finger_a', 'finger_b']
    assert list(point.positions) == [0.1, 0.2]
    assert list(point.effort) == [500.0, 500.0]
    assert point.time_from_start.sec == 1
    assert point.time_from_start.nanosec == 250_000_000
