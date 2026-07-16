import json
from types import SimpleNamespace

from vicpinky_task_servers.base_drive_straight_server import (
    BaseDriveStraightServer,
    parse_drive_options,
)
from vicpinky_task_servers.base_rotate_server import BaseRotateServer
from vicpinky_task_servers.dock_align_server import (
    DockAlignServer,
    update_alignment_hold,
)
from vicpinky_task_servers.elevator_board_off import ElevatorServers
from vicpinky_task_servers.nav_go_to_server import NavGoToServer


def test_drive_rejects_stale_odom(monkeypatch):
    server = object.__new__(BaseDriveStraightServer)
    server.latest_xy = (1.0, 2.0)
    server.last_odom_monotonic = 10.0
    server.odom_stale_timeout_sec = 0.5

    monkeypatch.setattr(
        'vicpinky_task_servers.base_drive_straight_server.time.monotonic',
        lambda: 10.6,
    )
    assert not server.odom_is_fresh()


def test_drive_accepts_fresh_odom(monkeypatch):
    server = object.__new__(BaseDriveStraightServer)
    server.latest_xy = (1.0, 2.0)
    server.last_odom_monotonic = 10.0
    server.odom_stale_timeout_sec = 0.5

    monkeypatch.setattr(
        'vicpinky_task_servers.base_drive_straight_server.time.monotonic',
        lambda: 10.4,
    )
    assert server.odom_is_fresh()


def test_drive_options_accept_driving_only_start_delay():
    options = parse_drive_options(
        json.dumps({
            'distance_m': 0.27,
            'speed_mps': 0.15,
            'start_delay_sec': 2.0,
        }),
        default_speed=0.10,
    )

    assert options == (0.27, 0.15, 2.0)


def test_drive_options_clamp_negative_start_delay():
    options = parse_drive_options(
        json.dumps({'start_delay_sec': -1.0}),
        default_speed=0.15,
    )

    assert options == (0.60, 0.15, 0.0)


def test_exit_distance_uses_landing_marker_goal_override_only():
    goal = SimpleNamespace(extra_json=json.dumps({
        'exit_target_distance_cm': 70.0,
    }))

    assert ElevatorServers.exit_target_cm(goal) == 70.0


def test_exit_distance_keeps_sixty_centimetre_default():
    goal = SimpleNamespace(extra_json='')

    assert ElevatorServers.exit_target_cm(goal) == 60.0


def test_nav_goal_accepts_three_second_map_settle_delay():
    extra_json = json.dumps({
        'pose': {'x': 2.84, 'y': 1.17},
        'start_delay_sec': 3.0,
    })

    assert NavGoToServer.parse_start_delay_sec(extra_json) == 3.0


def test_nav_goal_without_delay_keeps_immediate_behavior():
    assert NavGoToServer.parse_start_delay_sec('{}') == 0.0


def test_alignment_must_remain_valid_for_three_seconds():
    aligned_since, ready, held_sec = update_alignment_hold(
        None,
        now=10.0,
        hold_sec=3.0,
        is_aligned=True,
    )
    assert aligned_since == 10.0
    assert not ready
    assert held_sec == 0.0

    aligned_since, ready, held_sec = update_alignment_hold(
        aligned_since,
        now=12.5,
        hold_sec=3.0,
        is_aligned=True,
    )
    assert not ready
    assert held_sec == 2.5

    aligned_since, ready, held_sec = update_alignment_hold(
        aligned_since,
        now=13.0,
        hold_sec=3.0,
        is_aligned=True,
    )
    assert ready
    assert held_sec == 3.0


def test_alignment_hold_resets_when_alignment_is_lost():
    aligned_since, ready, held_sec = update_alignment_hold(
        10.0,
        now=12.0,
        hold_sec=3.0,
        is_aligned=False,
    )

    assert aligned_since is None
    assert not ready
    assert held_sec == 0.0


def test_alignment_requires_fresh_offset_and_distance(monkeypatch):
    server = object.__new__(DockAlignServer)
    server.latest_offset_x = 0.0
    server.latest_distance = 1.27
    server.latest_offset_time = 10.0
    server.latest_distance_time = 9.0
    server.marker_timeout_sec = 0.5

    monkeypatch.setattr(
        'vicpinky_task_servers.dock_align_server.time.monotonic',
        lambda: 10.1,
    )
    assert not server.has_fresh_marker()


def test_alignment_requires_fresh_expected_marker_id(monkeypatch):
    server = object.__new__(DockAlignServer)
    server.latest_marker_id = 20
    server.latest_marker_id_time = 10.0
    server.marker_timeout_sec = 0.5

    monkeypatch.setattr(
        'vicpinky_task_servers.dock_align_server.time.monotonic',
        lambda: 10.6,
    )
    assert not server.has_fresh_marker_id(20)


def test_rotation_rejects_stale_odom(monkeypatch):
    server = object.__new__(BaseRotateServer)
    server.latest_yaw = 0.25
    server.last_odom_monotonic = 20.0
    server.odom_stale_timeout_sec = 0.5

    monkeypatch.setattr(
        'vicpinky_task_servers.base_rotate_server.time.monotonic',
        lambda: 20.6,
    )
    assert not server.odom_is_fresh()


def test_rotation_requires_an_odom_sample(monkeypatch):
    server = object.__new__(BaseRotateServer)
    server.latest_yaw = None
    server.last_odom_monotonic = 0.0
    server.odom_stale_timeout_sec = 0.5

    monkeypatch.setattr(
        'vicpinky_task_servers.base_rotate_server.time.monotonic',
        lambda: 0.1,
    )
    assert not server.odom_is_fresh()
