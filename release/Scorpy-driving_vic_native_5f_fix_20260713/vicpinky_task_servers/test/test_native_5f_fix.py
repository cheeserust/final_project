from types import SimpleNamespace

import pytest

from vicpinky_task_servers.elevator_board_off import BOARDING_STOP_CM
from vicpinky_task_servers.elevator_board_off import BOARD_MARKER_ID
from vicpinky_task_servers.elevator_board_off import ElevatorServers
from vicpinky_task_servers.nav_go_to_server import NavGoToServer
from vicpinky_task_servers.nav_go_to_server import TARGET_ALIASES


def _goal(extra_json=''):
    return SimpleNamespace(extra_json=extra_json)


def test_nav_start_delay_defaults_to_zero():
    assert NavGoToServer.parse_start_delay_sec('') == 0.0
    assert NavGoToServer.parse_start_delay_sec('{}') == 0.0


def test_nav_start_delay_accepts_three_seconds():
    value = NavGoToServer.parse_start_delay_sec(
        '{"start_delay_sec": 3.0}'
    )
    assert value == 3.0


@pytest.mark.parametrize(
    'payload',
    [
        '{"start_delay_sec": -1.0}',
        '{"start_delay_sec": NaN}',
        '[]',
        '{invalid json}',
    ],
)
def test_nav_start_delay_rejects_invalid_values(payload):
    with pytest.raises((TypeError, ValueError)):
        NavGoToServer.parse_start_delay_sec(payload)


def test_exit_distance_defaults_to_original_sixty_cm():
    assert ElevatorServers.exit_target_cm(_goal()) == 60.0


def test_exit_distance_accepts_landing_marker_override():
    target = ElevatorServers.exit_target_cm(
        _goal('{"exit_target_distance_cm": 70.0}')
    )
    assert target == 70.0


def test_boarding_marker_and_distance_are_unchanged():
    assert BOARD_MARKER_ID == 10
    assert BOARDING_STOP_CM == 50.0


def test_legacy_mission_targets_resolve_to_native_keys():
    assert TARGET_ALIASES['4']['elevator_front_4f'] == 'elevator_front'
    assert TARGET_ALIASES['5']['elevator_front_5f'] == 'elevator_front'
    assert TARGET_ALIASES['4']['object'] == '402'
