"""Unit tests for ready-and-approach validation and result joining."""

import json

from mission_manager.ready_and_approach_logic import (
    child_extra_json,
    ChildOutcome,
    combine_child_outcomes,
    parse_ready_and_approach_request,
    ReadyAndApproachConfigError,
)
import pytest


VALID_EXTRA = json.dumps({
    'arm_task_name': 'arm_ready',
    'arm_start_to_drive_delay_sec': 2.0,
    'distance_m': 0.27,
    'speed_mps': 0.15,
})


def test_parse_valid_request_preserves_scenario_values():
    request = parse_ready_and_approach_request(VALID_EXTRA)

    assert request.arm_task_name == 'arm_ready'
    assert request.arm_start_to_drive_delay_sec == 2.0
    assert request.distance_m == 0.27
    assert request.speed_mps == 0.15


@pytest.mark.parametrize(
    ('extra', 'message'),
    [
        ('{broken', 'Invalid extra_json'),
        ('[]', 'JSON object'),
        (
            '{"distance_m": 0.27, "speed_mps": 0.15}',
            'arm_start_to_drive_delay_sec is required',
        ),
        (
            '{"arm_start_to_drive_delay_sec": -1, '
            '"distance_m": 0.27, "speed_mps": 0.15}',
            'cannot be negative',
        ),
        (
            '{"arm_start_to_drive_delay_sec": 2, '
            '"distance_m": 0, "speed_mps": 0.15}',
            'distance_m cannot be zero',
        ),
        (
            '{"arm_start_to_drive_delay_sec": 2, '
            '"distance_m": -1.01, "speed_mps": 0.15}',
            'absolute value cannot exceed 1.0 m',
        ),
        (
            '{"arm_start_to_drive_delay_sec": 2, '
            '"distance_m": 0.27, "speed_mps": false}',
            'speed_mps must be a finite number',
        ),
        (
            '{"arm_start_to_drive_delay_sec": 2, '
            '"distance_m": 0.27, "speed_mps": 0.31}',
            'speed_mps cannot exceed 0.3 m/s',
        ),
        (
            '{"arm_task_name": "observe_call_4f", '
            '"arm_start_to_drive_delay_sec": 2, '
            '"distance_m": 0.27, "speed_mps": 0.15}',
            'arm_task_name must be arm_ready',
        ),
    ],
)
def test_invalid_request_is_rejected(extra, message):
    with pytest.raises(ReadyAndApproachConfigError, match=message):
        parse_ready_and_approach_request(extra)


def test_child_payloads_do_not_leak_coordinator_timing_fields():
    request = parse_ready_and_approach_request(VALID_EXTRA)

    arm_raw, base_raw = child_extra_json(request)

    assert json.loads(arm_raw) == {'arm_task_name': 'arm_ready'}
    assert json.loads(base_raw) == {
        'distance_m': 0.27,
        'speed_mps': 0.15,
    }


def test_distance_and_speed_safety_boundaries_are_allowed():
    request = parse_ready_and_approach_request(json.dumps({
        'arm_start_to_drive_delay_sec': 2.0,
        'distance_m': -1.0,
        'speed_mps': 0.3,
    }))

    assert request.distance_m == -1.0
    assert request.speed_mps == 0.3


def test_two_successful_children_join_to_success():
    decision = combine_child_outcomes([
        ChildOutcome('arm', True, 'ready'),
        ChildOutcome('base', True, 'arrived'),
    ])

    assert decision.success is True
    assert decision.canceled is False
    assert decision.message == 'Ready-and-approach succeeded: arm, base'


def test_empty_child_outcome_set_is_not_successful():
    decision = combine_child_outcomes([])

    assert decision.success is False
    assert decision.canceled is False
    assert 'no child outcomes' in decision.message


def test_child_failure_reason_is_preserved():
    decision = combine_child_outcomes([
        ChildOutcome('arm', True, 'ready'),
        ChildOutcome('base', False, 'motor timeout'),
    ])

    assert decision.success is False
    assert decision.canceled is False
    assert 'base: motor timeout' in decision.message


def test_child_cancellation_is_visible_in_combined_decision():
    decision = combine_child_outcomes([
        ChildOutcome('arm', False, 'operator stop', canceled=True),
    ])

    assert decision.success is False
    assert decision.canceled is True
    assert 'operator stop' in decision.message
