import json

from mission_manager.operator_confirm_console import (
    OperatorGateState,
    prompt_from_goal,
)
from vicpinky_interfaces.action import RunTask


def test_gate_rejects_concurrent_reservation():
    gate = OperatorGateState()

    assert gate.reserve() is True
    assert gate.reserve() is False


def test_stale_enter_cannot_confirm_the_next_gate():
    gate = OperatorGateState()
    assert gate.reserve()
    first_token = gate.activate('first')
    gate.finish(first_token)

    assert gate.reserve()
    second_token = gate.activate('second')

    assert gate.confirm(first_token) is False
    assert gate.is_confirmed(second_token) is False
    assert gate.confirm(second_token) is True
    assert gate.is_confirmed(second_token) is True


def test_finishing_gate_clears_confirmation():
    gate = OperatorGateState()
    assert gate.reserve()
    token = gate.activate('prompt')
    assert gate.confirm(token)

    gate.finish(token)

    assert gate.snapshot() is None
    assert gate.is_confirmed(token) is False


def test_prompt_prefers_extra_json_and_has_safe_fallback():
    goal = RunTask.Goal()
    goal.target_name = 'target fallback'
    goal.extra_json = json.dumps({'prompt': 'press Enter now'})
    assert prompt_from_goal(goal) == 'press Enter now'

    goal.extra_json = '{invalid'
    assert prompt_from_goal(goal) == 'target fallback'

    goal.target_name = ''
    assert 'Enter' in prompt_from_goal(goal)
