from vicpinky_task_servers.base_drive_straight_server import (
    BaseDriveStraightServer,
)
from vicpinky_task_servers.base_rotate_server import BaseRotateServer


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
