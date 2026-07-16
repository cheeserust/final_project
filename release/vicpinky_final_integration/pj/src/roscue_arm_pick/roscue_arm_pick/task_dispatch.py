"""Resolve central RunTask goals to concrete arm task profiles."""

from __future__ import annotations

import json
from typing import Any, Mapping


class TaskDispatchError(ValueError):
    """Raised when a semantic arm goal cannot be dispatched safely."""


def parse_extra_json(raw: str) -> dict[str, Any]:
    """Parse a RunTask extra_json value into a mapping."""
    if not raw:
        return {}
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise TaskDispatchError(f'Invalid extra_json: {exc}') from exc
    if not isinstance(value, dict):
        raise TaskDispatchError('extra_json must contain a JSON object')
    return value


def resolve_alias(tasks: Mapping[str, Any], task_name: str) -> str:
    """Resolve task aliases while rejecting missing targets and loops."""
    visited: set[str] = set()
    current = str(task_name)

    while current in tasks:
        if current in visited:
            raise TaskDispatchError(f'Task alias loop detected at {current}')
        visited.add(current)
        task = tasks[current]
        if not isinstance(task, Mapping):
            raise TaskDispatchError(f'Task {current} must be a mapping')
        if task.get('type') != 'alias':
            return current
        current = str(task.get('target', '')).strip()

    raise TaskDispatchError(f'Unknown task: {current or task_name}')


def _explicit_task(extra: Mapping[str, Any]) -> str:
    return str(extra.get('arm_task_name', '')).strip()


def _button_task(goal: Any, extra: Mapping[str, Any]) -> str:
    role = str(extra.get('button_role', '')).strip()
    if role == 'elevator_call':
        floor = int(goal.target_floor)
        if floor == 4:
            return 'press_elevator_up'
        if floor == 5:
            return 'press_elevator_down'
        raise TaskDispatchError(
            f'Unsupported elevator-call floor: {floor}'
        )

    if role == 'floor_select':
        try:
            floor = int(extra.get('button_floor'))
        except (TypeError, ValueError) as exc:
            raise TaskDispatchError(
                'floor_select requires integer extra_json.button_floor'
            ) from exc
        if floor == 4:
            return 'press_floor_4'
        if floor == 5:
            return 'press_floor_5'
        raise TaskDispatchError(f'Unsupported floor button: {floor}')

    raise TaskDispatchError(
        'press_button requires button_role=elevator_call or floor_select'
    )


def resolve_run_task(
    endpoint: str,
    goal: Any,
    tasks: Mapping[str, Any],
    mission_allowed_tasks: set[str],
) -> str:
    """Resolve one RunTask endpoint and goal to a concrete YAML task key."""
    if endpoint == 'homing':
        return '__homing__'

    extra = parse_extra_json(goal.extra_json)
    concrete = _explicit_task(extra)

    if endpoint == 'press_button' and not concrete:
        concrete = _button_task(goal, extra)

    if not concrete:
        raise TaskDispatchError(
            f'{endpoint} requires extra_json.arm_task_name'
        )

    concrete = resolve_alias(tasks, concrete)
    if concrete not in mission_allowed_tasks:
        raise TaskDispatchError(
            f'Task is not allowed from mission actions: {concrete}'
        )

    task_type = str(tasks[concrete].get('type', ''))
    if endpoint == 'pick' and task_type != 'pick_to_fixed_place':
        raise TaskDispatchError(
            f'/arm/pick cannot execute task type {task_type}: {concrete}'
        )
    if endpoint == 'place' and task_type not in {
        'place_fixed',
        'pick_previous_to_fixed_place',
    }:
        raise TaskDispatchError(
            f'/arm/place cannot execute task type {task_type}: {concrete}'
        )
    if endpoint == 'press_button' and task_type != 'press_button':
        raise TaskDispatchError(
            '/arm/press_button requires a press_button task, '
            f'got {task_type}: {concrete}'
        )

    return concrete
