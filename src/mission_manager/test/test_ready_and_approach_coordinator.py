"""Test coordinator failure and cancellation propagation."""

import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from mission_manager import ready_and_approach_coordinator as coordinator_module
from mission_manager.ready_and_approach_coordinator import (
    ActiveChild,
    CoordinationCanceled,
    CoordinationError,
    ReadyAndApproachCoordinator,
)
import pytest
from rclpy.action import GoalResponse
from vicpinky_interfaces.action import RunTask


VALID_EXTRA = (
    '{"arm_task_name":"arm_ready",'
    '"arm_start_to_drive_delay_sec":2.0,'
    '"distance_m":0.27,"speed_mps":0.15}'
)


def completed_future(value):
    """Return the minimal already-complete future surface used by tests."""
    return SimpleNamespace(done=lambda: True, result=lambda: value)


def parent_goal_handle(*, cancel_requested=False):
    """Build a mock parent goal handle with a valid RunTask request."""
    request = RunTask.Goal()
    request.task_id = 'ready_and_approach'
    request.target_name = 'elevator_door_4f'
    request.target_floor = 4
    request.marker_id = 50
    request.extra_json = VALID_EXTRA
    return SimpleNamespace(
        request=request,
        is_cancel_requested=cancel_requested,
        publish_feedback=Mock(),
        succeed=Mock(),
        abort=Mock(),
        canceled=Mock(),
    )


def bare_coordinator():
    """Construct a coordinator shell without initializing a ROS node."""
    coordinator = ReadyAndApproachCoordinator.__new__(
        ReadyAndApproachCoordinator
    )
    coordinator._reservation_lock = threading.Lock()
    coordinator._execution_reserved = True
    coordinator._execution_timeout_sec = 80.0
    coordinator._server_wait_timeout_sec = 0.001
    coordinator._goal_response_timeout_sec = 1.0
    coordinator._cancel_timeout_sec = 0.01
    coordinator.get_logger = Mock(return_value=Mock())
    return coordinator


def test_goal_callback_rejects_unsupported_task_id():
    coordinator = bare_coordinator()
    coordinator._execution_reserved = False
    request = parent_goal_handle().request
    request.task_id = 'rotate'

    response = coordinator._goal_callback(request)

    assert response == GoalResponse.REJECT
    assert coordinator._execution_reserved is False


def test_goal_callback_rejects_second_motion_reservation():
    coordinator = bare_coordinator()

    response = coordinator._goal_callback(parent_goal_handle().request)

    assert response == GoalResponse.REJECT


def test_child_goal_rejection_is_reported():
    coordinator = bare_coordinator()
    client = Mock()
    client.send_goal_async.return_value = completed_future(
        SimpleNamespace(accepted=False)
    )

    with pytest.raises(CoordinationError, match='arm action rejected'):
        coordinator._send_child_goal(
            name='arm',
            client=client,
            child_goal=RunTask.Goal(),
            parent_goal_handle=parent_goal_handle(),
            overall_deadline=time.monotonic() + 1.0,
            feedback_enabled=threading.Event(),
        )


def test_parent_cancel_is_detected_while_waiting_for_server(monkeypatch):
    coordinator = bare_coordinator()
    client = Mock()
    monkeypatch.setattr(coordinator_module.rclpy, 'ok', lambda: True)

    with pytest.raises(CoordinationCanceled, match='canceled'):
        coordinator._wait_for_server(
            client,
            'arm',
            parent_goal_handle(cancel_requested=True),
            time.monotonic() + 1.0,
        )

    client.wait_for_server.assert_not_called()


def test_server_wait_timeout_is_reported(monkeypatch):
    coordinator = bare_coordinator()
    client = Mock()
    client.wait_for_server.return_value = False
    monkeypatch.setattr(coordinator_module.rclpy, 'ok', lambda: True)

    with pytest.raises(CoordinationError, match='Timed out waiting for base'):
        coordinator._wait_for_server(
            client,
            'base',
            parent_goal_handle(),
            time.monotonic() + 1.0,
        )


def test_aborted_child_status_and_message_are_preserved():
    coordinator = bare_coordinator()
    response = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(success=False, message='motor jam'),
    )
    child = ActiveChild(
        name='base',
        goal_handle=Mock(),
        result_future=completed_future(response),
    )

    outcome = coordinator._child_outcome(child)

    assert outcome.success is False
    assert outcome.canceled is False
    assert outcome.message == 'ABORTED: motor jam'


def test_base_rejection_aborts_parent_and_cancels_accepted_arm():
    coordinator = bare_coordinator()
    coordinator._arm_client = Mock()
    coordinator._base_client = Mock()
    coordinator._wait_for_server = Mock()
    coordinator._publish_feedback = Mock()
    coordinator._wait_for_drive_start = Mock(return_value=None)
    coordinator._cancel_children = Mock()
    arm_child = ActiveChild(
        name='arm',
        goal_handle=Mock(),
        result_future=SimpleNamespace(done=lambda: False),
    )
    coordinator._send_child_goal = Mock(side_effect=[
        arm_child,
        CoordinationError('base action rejected its goal'),
    ])
    parent = parent_goal_handle()

    result = coordinator._execute_callback(parent)

    assert result.success is False
    assert result.message == 'base action rejected its goal'
    parent.abort.assert_called_once()
    parent.canceled.assert_not_called()
    canceled_children = coordinator._cancel_children.call_args.args[0]
    assert canceled_children == {'arm': arm_child}
    assert coordinator._execution_reserved is False


def test_parent_cancel_cancels_accepted_arm_and_marks_parent_canceled():
    coordinator = bare_coordinator()
    coordinator._arm_client = Mock()
    coordinator._base_client = Mock()
    coordinator._wait_for_server = Mock()
    coordinator._publish_feedback = Mock()
    coordinator._cancel_children = Mock()
    arm_child = ActiveChild(
        name='arm',
        goal_handle=Mock(),
        result_future=SimpleNamespace(done=lambda: False),
    )
    coordinator._send_child_goal = Mock(return_value=arm_child)
    coordinator._wait_for_drive_start = Mock(
        side_effect=CoordinationCanceled('operator canceled')
    )
    parent = parent_goal_handle(cancel_requested=True)

    result = coordinator._execute_callback(parent)

    assert result.success is False
    assert result.message == 'operator canceled'
    parent.canceled.assert_called_once()
    parent.abort.assert_not_called()
    canceled_children = coordinator._cancel_children.call_args.args[0]
    assert canceled_children == {'arm': arm_child}


def test_join_stops_on_child_failure(monkeypatch):
    coordinator = bare_coordinator()
    monkeypatch.setattr(coordinator_module.rclpy, 'ok', lambda: True)
    successful_result = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(success=True, message='ready'),
    )
    failed_result = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(success=False, message='drive fault'),
    )
    children = {
        'arm': ActiveChild('arm', Mock(), completed_future(successful_result)),
        'base': ActiveChild('base', Mock(), completed_future(failed_result)),
    }

    with pytest.raises(CoordinationError, match='base: ABORTED: drive fault'):
        coordinator._join_children(
            children,
            parent_goal_handle(),
            time.monotonic() + 1.0,
            {},
        )


class PendingFuture:
    """Small controllable future used to exercise late-goal cleanup."""

    def __init__(self):
        self._callback = None

    def done(self):
        """Report that the simulated response is still pending."""
        return False

    def add_done_callback(self, callback):
        """Store the callback registered by the coordinator."""
        self._callback = callback

    def deliver(self, goal_handle):
        """Deliver an accepted goal handle to the stored callback."""
        completed = completed_future(goal_handle)
        self._callback(completed)


def test_late_goal_response_is_canceled_without_becoming_orphaned():
    coordinator = bare_coordinator()
    pending = PendingFuture()
    late_goal_handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=Mock(),
    )

    coordinator._cancel_late_goal(pending, 'base')
    pending.deliver(late_goal_handle)

    late_goal_handle.cancel_goal_async.assert_called_once()


class NewlyFailedResultFuture:
    """Become failed after the coordinator starts sending the base goal."""

    def __init__(self, response):
        self._response = response
        self._done_checks = 0

    def done(self):
        """Return false for pre-send validation and true during the wait."""
        self._done_checks += 1
        return self._done_checks >= 2

    def result(self):
        """Return the configured failed arm result."""
        return self._response


def test_arm_failure_while_base_response_pending_cancels_late_base(
    monkeypatch,
):
    coordinator = bare_coordinator()
    monkeypatch.setattr(coordinator_module.rclpy, 'ok', lambda: True)
    arm_response = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(success=False, message='arm fault'),
    )
    arm_child = ActiveChild(
        name='arm',
        goal_handle=Mock(),
        result_future=NewlyFailedResultFuture(arm_response),
    )
    base_pending = PendingFuture()
    base_client = Mock()
    base_client.send_goal_async.return_value = base_pending

    with pytest.raises(CoordinationError, match='arm: ABORTED: arm fault'):
        coordinator._send_child_goal(
            name='base',
            client=base_client,
            child_goal=RunTask.Goal(),
            parent_goal_handle=parent_goal_handle(),
            overall_deadline=time.monotonic() + 1.0,
            feedback_enabled=threading.Event(),
            watch_child=arm_child,
        )

    late_base = SimpleNamespace(
        accepted=True,
        cancel_goal_async=Mock(),
    )
    base_pending.deliver(late_base)
    late_base.cancel_goal_async.assert_called_once()
