from unittest.mock import Mock

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
