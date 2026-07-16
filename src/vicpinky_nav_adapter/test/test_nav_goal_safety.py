import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from vicpinky_nav_adapter.nav_adapter_node import VicPinkyNavAdapter


class PendingFuture:

    def __init__(self, result):
        self._result = result
        self.callback = None

    @staticmethod
    def done():
        return False

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callback = callback


def test_late_nav_goal_is_canceled_after_send_timeout():
    adapter = VicPinkyNavAdapter.__new__(VicPinkyNavAdapter)
    adapter._cancel_nav_goal = Mock()
    adapter.get_logger = Mock(return_value=Mock())
    goal_handle = Mock()
    goal_handle.accepted = True
    future = PendingFuture(goal_handle)

    adapter._cancel_late_nav_goal(future)
    future.callback(future)

    adapter._cancel_nav_goal.assert_called_once_with(goal_handle)


def test_late_rejected_nav_goal_needs_no_cancel():
    adapter = VicPinkyNavAdapter.__new__(VicPinkyNavAdapter)
    adapter._cancel_nav_goal = Mock()
    adapter.get_logger = Mock(return_value=Mock())
    goal_handle = Mock()
    goal_handle.accepted = False
    future = PendingFuture(goal_handle)

    adapter._cancel_late_nav_goal(future)
    future.callback(future)

    adapter._cancel_nav_goal.assert_not_called()


def test_navigation_target_accepts_configured_start_delay():
    adapter = VicPinkyNavAdapter.__new__(VicPinkyNavAdapter)
    adapter._default_frame_id = 'map'
    goal = SimpleNamespace(extra_json=json.dumps({
        'pose': {'x': 2.84, 'y': 1.17, 'yaw': 0.109},
        'start_delay_sec': 3.0,
    }))

    target = adapter._target_from_goal(goal)

    assert target.start_delay_sec == 3.0


@pytest.mark.parametrize('delay', [-1.0, float('nan'), float('inf')])
def test_navigation_target_rejects_invalid_start_delay(delay):
    adapter = VicPinkyNavAdapter.__new__(VicPinkyNavAdapter)
    adapter._default_frame_id = 'map'
    goal = SimpleNamespace(extra_json=json.dumps({
        'pose': {'x': 2.84, 'y': 1.17, 'yaw': 0.109},
        'start_delay_sec': delay,
    }))

    with pytest.raises(ValueError, match='start_delay_sec'):
        adapter._target_from_goal(goal)
