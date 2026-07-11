"""Unit tests for semantic RunTask dispatch."""

import json
from types import SimpleNamespace

import pytest

from roscue_arm_pick.task_dispatch import (
    parse_extra_json,
    resolve_run_task,
    TaskDispatchError,
)


TASKS = {
    'pick_object_2': {'type': 'pick_to_fixed_place'},
    'place_to_robot': {'type': 'place_fixed'},
    'press_elevator_up': {'type': 'press_button'},
    'press_elevator_down': {'type': 'press_button'},
    'press_floor_4': {'type': 'press_button'},
    'press_floor_5': {'type': 'press_button'},
    'press_4f': {'type': 'alias', 'target': 'press_floor_4'},
}
ALLOWED = set(TASKS) - {'press_4f'}


def goal(*, floor=4, extra='{}'):
    """Build the small goal surface needed by the dispatcher."""
    return SimpleNamespace(target_floor=floor, extra_json=extra)


def test_execute_uses_explicit_concrete_task():
    concrete = resolve_run_task(
        'execute',
        goal(extra='{"arm_task_name": "pick_object_2"}'),
        TASKS,
        ALLOWED,
    )
    assert concrete == 'pick_object_2'


@pytest.mark.parametrize(
    ('floor', 'role', 'button_floor', 'expected'),
    [
        (4, 'elevator_call', None, 'press_elevator_up'),
        (5, 'elevator_call', None, 'press_elevator_down'),
        (4, 'floor_select', 4, 'press_floor_4'),
        (4, 'floor_select', 5, 'press_floor_5'),
    ],
)
def test_button_dispatch(floor, role, button_floor, expected):
    extra = {'button_role': role}
    if button_floor is not None:
        extra['button_floor'] = button_floor
    assert resolve_run_task(
        'press_button',
        goal(floor=floor, extra=json.dumps(extra)),
        TASKS,
        ALLOWED,
    ) == expected


def test_alias_is_resolved_before_type_validation():
    allowed = set(ALLOWED)
    allowed.add('press_floor_4')
    assert resolve_run_task(
        'press_button',
        goal(extra='{"arm_task_name": "press_4f"}'),
        TASKS,
        allowed,
    ) == 'press_floor_4'


def test_pick_endpoint_rejects_place_task():
    with pytest.raises(TaskDispatchError, match='cannot execute'):
        resolve_run_task(
            'pick',
            goal(extra='{"arm_task_name": "place_to_robot"}'),
            TASKS,
            ALLOWED,
        )


def test_invalid_extra_json_is_rejected():
    with pytest.raises(TaskDispatchError, match='Invalid extra_json'):
        parse_extra_json('{bad json')
