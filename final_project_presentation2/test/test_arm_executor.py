"""Safety regressions for concurrent arm/gripper action submission."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from final_project_presentation2.arm_executor import (
    ArmPoseExecutor,
    PoseCancellationUnconfirmed,
    PoseExecutionError,
)

import pytest


class FakeFuture:
    """Minimal controllable future used by the action safety tests."""

    def __init__(
        self,
        result: Any = None,
        *,
        done: bool = False,
        error: BaseException | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._done = done
        self._result = result
        self._error = error
        self._callbacks: list[Callable[[FakeFuture], None]] = []

    def done(self) -> bool:
        with self._lock:
            return self._done

    def result(self) -> Any:
        with self._lock:
            if not self._done:
                raise RuntimeError('future is not complete')
            if self._error is not None:
                raise self._error
            return self._result

    def add_done_callback(
        self,
        callback: Callable[[FakeFuture], None],
    ) -> None:
        call_now = False
        with self._lock:
            if self._done:
                call_now = True
            else:
                self._callbacks.append(callback)
        if call_now:
            callback(self)

    def set_result(self, result: Any) -> None:
        with self._lock:
            if self._done:
                raise RuntimeError('future is already complete')
            self._result = result
            self._done = True
            callbacks = list(self._callbacks)
            self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeGoalHandle:
    """Fake accepted goal that records cancellation attempts."""

    def __init__(self, *, cancellation_completes: bool = True) -> None:
        self.accepted = True
        self.result_future = FakeFuture()
        self.cancel_count = 0
        self._cancellation_completes = cancellation_completes

    def get_result_async(self) -> FakeFuture:
        return self.result_future

    def cancel_goal_async(self) -> FakeFuture:
        self.cancel_count += 1
        if (
            self._cancellation_completes
            and not self.result_future.done()
        ):
            self.result_future.set_result(SimpleNamespace(
                status=GoalStatus.STATUS_CANCELED,
                result=object(),
            ))
        return FakeFuture(object(), done=self._cancellation_completes)


class FakeClient:
    """Action-client double returning one configured response future."""

    def __init__(self, response_future: FakeFuture) -> None:
        self.response_future = response_future

    def wait_for_server(self, *, timeout_sec: float) -> bool:
        del timeout_sec
        return True

    def send_goal_async(self, _goal: Any) -> FakeFuture:
        return self.response_future


class FailingClient:
    """Action client that loses submission state while sending a goal."""

    def wait_for_server(self, *, timeout_sec: float) -> bool:
        del timeout_sec
        return True

    def send_goal_async(self, _goal: Any) -> FakeFuture:
        raise RuntimeError('send transport failed')


def _pose() -> dict[str, Any]:
    return {
        'arm_enabled': True,
        'arm_positions_deg': [0.0] * 5,
        'arm_duration_sec': 5.0,
        'gripper_enabled': True,
        'gripper_positions_deg': [0.0] * 9,
        'gripper_duration_sec': 5.0,
        'target_load_raw': 500,
        'dwell_sec': 0.0,
    }


def _executor(
    arm_response: FakeFuture,
    gripper_response: FakeFuture,
) -> ArmPoseExecutor:
    executor = object.__new__(ArmPoseExecutor)
    executor._arm_client = FakeClient(arm_response)
    executor._gripper_client = FakeClient(gripper_response)
    executor._handles_lock = threading.Lock()
    executor._active_handles = []
    executor._late_response_futures = set()
    executor._unconfirmed_submissions = 0
    executor._event_callback = None
    return executor


def _wait_until(predicate: Callable[[], bool], timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('condition did not become true')


def test_stop_tracks_ready_peer_and_cancels_late_accepted_goal():
    """A response stuck ahead of an accepted peer cannot create a ghost goal."""
    arm_response = FakeFuture()
    gripper_handle = FakeGoalHandle()
    gripper_response = FakeFuture(gripper_handle, done=True)
    executor = _executor(arm_response, gripper_response)
    stop_event = threading.Event()
    failures: list[BaseException] = []

    def run() -> None:
        try:
            executor.execute(
                _pose(),
                stop_event=stop_event,
                server_timeout_sec=1.0,
                result_timeout_margin_sec=0.5,
            )
        except BaseException as exc:
            failures.append(exc)

    thread = threading.Thread(target=run)
    thread.start()

    # The gripper is the second submission. It must be tracked even though the
    # first (arm) response has not completed yet.
    _wait_until(lambda: gripper_handle in executor._active_handles)
    stop_event.set()
    # A goal response can arrive after STOP. execute() must retain ownership,
    # cancel it, and wait for its terminal result before returning.
    arm_handle = FakeGoalHandle()
    arm_response.set_result(arm_handle)
    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert len(failures) == 1
    assert isinstance(failures[0], PoseExecutionError)
    assert not isinstance(failures[0], PoseCancellationUnconfirmed)
    assert 'stopped' in str(failures[0])
    assert gripper_handle.cancel_count == 1
    assert arm_handle.cancel_count == 1
    assert gripper_handle not in executor._active_handles
    assert arm_handle not in executor._active_handles
    assert executor.shutdown_drained


def test_response_timeout_reports_unconfirmed_late_submission():
    arm_response = FakeFuture()
    gripper_handle = FakeGoalHandle()
    executor = _executor(
        arm_response,
        FakeFuture(gripper_handle, done=True),
    )

    with pytest.raises(
        PoseCancellationUnconfirmed,
        match='could not be confirmed terminal',
    ):
        executor.execute(
            _pose(),
            stop_event=threading.Event(),
            server_timeout_sec=0.04,
            result_timeout_margin_sec=0.5,
        )

    assert gripper_handle.cancel_count == 1
    late_arm_handle = FakeGoalHandle()
    arm_response.set_result(late_arm_handle)
    assert late_arm_handle.cancel_count == 1
    assert executor.shutdown_drained


def test_cancel_active_wait_is_bounded_when_cancel_response_never_arrives():
    executor = _executor(FakeFuture(), FakeFuture())
    handle = FakeGoalHandle(cancellation_completes=False)
    executor._active_handles.append(handle)

    started = time.monotonic()
    executor.cancel_active()
    elapsed = time.monotonic() - started

    assert handle.cancel_count == 1
    assert elapsed < 0.75
    assert handle in executor._active_handles


def test_arm_waits_for_actual_result_past_requested_duration():
    arm_handle = FakeGoalHandle(cancellation_completes=False)
    executor = _executor(
        FakeFuture(arm_handle, done=True),
        FakeFuture(),
    )
    pose = _pose()
    pose['arm_duration_sec'] = 0.001
    pose['gripper_enabled'] = False
    outcomes: list[Any] = []

    def run() -> None:
        outcomes.append(executor.execute(
            pose,
            stop_event=threading.Event(),
            server_timeout_sec=0.04,
            result_timeout_margin_sec=0.0,
        ))

    thread = threading.Thread(target=run)
    thread.start()
    _wait_until(lambda: arm_handle in executor._active_handles)

    # Passing the requested duration must not advance the sequence while the
    # STM status-backed action result still reports the arm as active.
    time.sleep(0.05)
    assert thread.is_alive()
    assert arm_handle.cancel_count == 0

    arm_handle.result_future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(success=True, message='completed'),
    ))
    thread.join(timeout=1.0)

    assert not thread.is_alive()
    assert len(outcomes) == 1
    assert outcomes[0].success
    assert arm_handle.cancel_count == 0
    assert arm_handle not in executor._active_handles
    assert executor.shutdown_drained


def test_gripper_result_timeout_still_fails_pose():
    gripper_handle = FakeGoalHandle()
    executor = _executor(
        FakeFuture(),
        FakeFuture(gripper_handle, done=True),
    )
    pose = _pose()
    pose['arm_enabled'] = False
    pose['gripper_duration_sec'] = 0.001

    with pytest.raises(
        PoseExecutionError,
        match='gripper controller result timed out',
    ):
        executor.execute(
            pose,
            stop_event=threading.Event(),
            server_timeout_sec=0.04,
            result_timeout_margin_sec=0.0,
        )

    assert gripper_handle.cancel_count == 1
    assert gripper_handle not in executor._active_handles
    assert executor.shutdown_drained


def test_goal_response_transport_error_is_permanently_unconfirmed():
    gripper_handle = FakeGoalHandle()
    executor = _executor(
        FakeFuture(done=True, error=RuntimeError('transport failed')),
        FakeFuture(gripper_handle, done=True),
    )

    with pytest.raises(
        PoseCancellationUnconfirmed,
        match='goal response failed',
    ):
        executor.execute(
            _pose(),
            stop_event=threading.Event(),
            server_timeout_sec=0.04,
            result_timeout_margin_sec=0.5,
        )

    assert gripper_handle.cancel_count == 1
    assert not executor.shutdown_drained


def test_result_transport_error_keeps_goal_tracked():
    arm_handle = FakeGoalHandle()
    arm_handle.result_future = FakeFuture(
        done=True,
        error=RuntimeError('result transport failed'),
    )
    gripper_handle = FakeGoalHandle()
    executor = _executor(
        FakeFuture(arm_handle, done=True),
        FakeFuture(gripper_handle, done=True),
    )

    with pytest.raises(
        PoseCancellationUnconfirmed,
        match='unconfirmed terminal state',
    ):
        executor.execute(
            _pose(),
            stop_event=threading.Event(),
            server_timeout_sec=0.04,
            result_timeout_margin_sec=0.5,
        )

    assert arm_handle.cancel_count == 1
    assert gripper_handle.cancel_count == 1
    assert arm_handle in executor._active_handles
    assert not executor.shutdown_drained


def test_second_submission_failure_still_cancels_first_late_acceptance():
    arm_response = FakeFuture()
    executor = _executor(arm_response, FakeFuture())
    executor._gripper_client = FailingClient()

    with pytest.raises(
        PoseCancellationUnconfirmed,
        match='submission state is unknown',
    ):
        executor.execute(
            _pose(),
            stop_event=threading.Event(),
            server_timeout_sec=0.04,
            result_timeout_margin_sec=0.5,
        )

    late_arm_handle = FakeGoalHandle()
    arm_response.set_result(late_arm_handle)
    assert late_arm_handle.cancel_count == 1
    assert not executor.shutdown_drained
