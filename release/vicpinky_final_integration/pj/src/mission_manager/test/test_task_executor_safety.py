"""Test pending child-goal cleanup in the mission task executor."""

from unittest.mock import Mock

from mission_manager import task_executor as task_executor_module
from mission_manager.mission_state import MissionStep
from mission_manager.task_executor import TaskExecutor


class PendingFuture:
    """Controllable future that can deliver a late goal response."""

    def __init__(self):
        self._callback = None

    def done(self):
        """Report that no response has arrived yet."""
        return False

    def add_done_callback(self, callback):
        """Store a cleanup callback for the eventual response."""
        self._callback = callback

    def deliver(self, goal_handle):
        """Deliver a completed goal response to the cleanup callback."""
        completed = Mock()
        completed.result.return_value = goal_handle
        self._callback(completed)


def mission_step(timeout_sec=10.0):
    """Build the minimal child step used by these tests."""
    return MissionStep(
        state='TEST',
        task_id='test',
        server='/child/test',
        target_name='target',
        target_floor=4,
        marker_id=-1,
        timeout_sec=timeout_sec,
        retry=0,
    )


def bare_executor(send_future):
    """Construct a TaskExecutor shell around a mocked action client."""
    node = Mock()
    client = Mock()
    client.wait_for_server.return_value = True
    client.send_goal_async.return_value = send_future
    executor = TaskExecutor.__new__(TaskExecutor)
    executor._node = node
    executor._get_client = Mock(return_value=client)
    return executor


def test_cancel_while_goal_response_pending_cancels_late_acceptance(
    monkeypatch,
):
    pending = PendingFuture()
    executor = bare_executor(pending)
    mission_goal = Mock()
    mission_goal.is_cancel_requested = True
    monkeypatch.setattr(task_executor_module.rclpy, 'ok', lambda: True)

    result = executor.execute(mission_step(), mission_goal)

    assert result.canceled is True
    assert result.success is False

    late_goal = Mock()
    late_goal.accepted = True
    pending.deliver(late_goal)
    late_goal.cancel_goal_async.assert_called_once()


def test_timeout_while_goal_response_pending_cancels_late_acceptance(
    monkeypatch,
):
    pending = PendingFuture()
    executor = bare_executor(pending)
    mission_goal = Mock()
    mission_goal.is_cancel_requested = False
    monkeypatch.setattr(task_executor_module.rclpy, 'ok', lambda: True)

    result = executor.execute(mission_step(timeout_sec=0.0), mission_goal)

    assert result.timed_out is True
    assert result.success is False

    late_goal = Mock()
    late_goal.accepted = True
    pending.deliver(late_goal)
    late_goal.cancel_goal_async.assert_called_once()
