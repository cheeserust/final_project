"""Unit tests for trajectory streaming completion timing."""

from arm_can_bridge import trajectory_streamer as streamer_module
from arm_can_bridge.trajectory_streamer import TrajectoryStreamer


class _AlwaysCompleteBoardState:
    def __init__(self) -> None:
        self.completion_check_times: list[float] = []

    def is_trajectory_complete(self) -> bool:
        self.completion_check_times.append(streamer_module.time.monotonic())
        return True

    def has_error(self) -> bool:
        return False


class _UnusedTransport:
    pass


def test_completion_wait_ignores_complete_status_until_min_wait(monkeypatch):
    board_state = _AlwaysCompleteBoardState()
    now = {'value': 0.0}

    monkeypatch.setattr(streamer_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        streamer_module.time,
        'monotonic',
        lambda: now['value'],
    )

    def sleep(duration_s):
        now['value'] += duration_s

    monkeypatch.setattr(streamer_module.time, 'sleep', sleep)

    streamer = TrajectoryStreamer(
        board_state=board_state,
        transport=_UnusedTransport(),
        queue_wait_timeout_ms=100,
        completion_grace_ms=100,
    )

    streamer._wait_for_completion(
        timeout_s=1.0,
        min_wait_s=0.06,
        cancel_requested=None,
    )

    assert board_state.completion_check_times
    assert board_state.completion_check_times[0] >= 0.06
