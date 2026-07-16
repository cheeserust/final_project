"""Standalone arm and gripper action execution for presentation poses."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Callable, Iterable
import uuid

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from vicpinky_interfaces.action import ExecuteArmGoal


ARM_JOINT_NAMES = [
    'base_joint',
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
]

GRIPPER_JOINT_NAMES = [
    'finger_1_base_joint',
    'finger_1_middle_joint',
    'finger_1_tip_joint',
    'finger_2_base_joint',
    'finger_2_middle_joint',
    'finger_2_tip_joint',
    'finger_3_base_joint',
    'finger_3_middle_joint',
    'finger_3_tip_joint',
]


@dataclass(frozen=True)
class PoseExecutionResult:
    success: bool
    message: str


class PoseExecutionError(RuntimeError):
    """Raised when a controller rejects or fails a saved pose."""


class PoseCancellationUnconfirmed(PoseExecutionError):
    """Raised when STOP cannot prove every submitted goal is terminal."""


class ArmPoseExecutor:
    """
    Send the two hardware actions without depending on the legacy GUI.

    The ROS executor must keep spinning while :meth:`execute` runs in a worker
    thread. When a pose contains both controllers, both goals are submitted
    before either result is awaited. A failure cancels the peer controller.
    """

    def __init__(
        self,
        node: Any,
        arm_action_name: str,
        gripper_action_name: str,
        event_callback: Callable[[str, str], None] | None = None,
        callback_group: Any | None = None,
    ) -> None:
        self._node = node
        self._arm_client = ActionClient(
            node,
            ExecuteArmGoal,
            str(arm_action_name),
            callback_group=callback_group,
        )
        self._gripper_client = ActionClient(
            node,
            FollowJointTrajectory,
            str(gripper_action_name),
            callback_group=callback_group,
        )
        self._event_callback = event_callback
        self._handles_lock = threading.Lock()
        self._active_handles: list[Any] = []
        self._late_response_futures: set[Any] = set()
        self._unconfirmed_submissions = 0

    @property
    def arm_ready(self) -> bool:
        """Return whether discovery has found the arm action server."""
        return bool(self._arm_client.server_is_ready())

    @property
    def gripper_ready(self) -> bool:
        """Return whether discovery has found the gripper action server."""
        return bool(self._gripper_client.server_is_ready())

    @property
    def shutdown_drained(self) -> bool:
        """Return whether no accepted or late-response goal remains."""
        with self._handles_lock:
            return (
                not self._active_handles
                and not self._late_response_futures
                and self._unconfirmed_submissions == 0
            )

    def execute(
        self,
        pose: dict[str, Any],
        *,
        stop_event: threading.Event,
        server_timeout_sec: float,
        result_timeout_margin_sec: float,
    ) -> PoseExecutionResult:
        if not self.shutdown_drained:
            raise PoseCancellationUnconfirmed(
                'A previous action submission is not confirmed terminal; '
                'restart the presentation node before executing another pose'
            )
        arm_enabled = bool(pose.get('arm_enabled', False))
        gripper_enabled = bool(pose.get('gripper_enabled', False))
        if not arm_enabled and not gripper_enabled:
            raise PoseExecutionError('Pose has no enabled controller')

        submissions: list[tuple[str, Any, float]] = []
        request_id = str(uuid.uuid4())
        created_ms = int(time.time() * 1000)

        if arm_enabled:
            if not self._wait_server(
                self._arm_client,
                server_timeout_sec,
                stop_event,
            ):
                raise PoseExecutionError('Arm action server is unavailable')
            arm_goal, arm_duration = self._make_arm_goal(
                pose,
                request_id,
                created_ms,
            )

        if gripper_enabled:
            if not self._wait_server(
                self._gripper_client,
                server_timeout_sec,
                stop_event,
            ):
                raise PoseExecutionError('Gripper action server is unavailable')
            gripper_goal, gripper_duration = self._make_gripper_goal(pose)

        # Submit only after every required server is known to be available.
        # This prevents the arm starting alone while waiting for a missing
        # gripper server.
        try:
            if arm_enabled:
                future = self._arm_client.send_goal_async(arm_goal)
                if future is None:
                    raise RuntimeError('arm submission returned no future')
                submissions.append(('arm', future, arm_duration))
            if gripper_enabled:
                future = self._gripper_client.send_goal_async(gripper_goal)
                if future is None:
                    raise RuntimeError(
                        'gripper submission returned no future'
                    )
                submissions.append(('gripper', future, gripper_duration))
        except Exception as exc:
            # An exception at this boundary cannot prove whether the action
            # request reached its server. Retain a permanent unsafe sentinel
            # and cancel every earlier response if it later gets accepted.
            self._mark_unconfirmed_submission()
            self._cancel_pending_goal_responses(submissions)
            raise PoseCancellationUnconfirmed(
                f'Action goal submission state is unknown: {exc}'
            ) from exc

        # Collect all response futures under one deadline. Waiting for them in
        # submission order is unsafe: a later controller can already have an
        # accepted (and moving) goal while an earlier response is still stuck.
        # The result future is requested and the handle is tracked immediately
        # after acceptance, before any other response is awaited.
        accepted: list[tuple[str, Any, float, Any | None]] = []
        pending_responses = list(submissions)
        response_deadline = (
            time.monotonic() + max(0.0, float(server_timeout_sec))
        )
        response_error: str | None = None
        while pending_responses:
            if stop_event.is_set():
                if not self._cancel_submitted_and_wait(
                    pending_responses,
                    accepted,
                    server_timeout_sec,
                ):
                    raise PoseCancellationUnconfirmed(
                        'Pose stop requested, but a submitted goal did not '
                        'reach a confirmed terminal state'
                    )
                first_name = pending_responses[0][0]
                raise PoseExecutionError(
                    f'{first_name} goal response was stopped'
                )

            completed_responses = [
                item for item in pending_responses if item[1].done()
            ]
            for name, future, duration in completed_responses:
                pending_responses.remove((name, future, duration))
                try:
                    handle = future.result()
                except Exception as exc:
                    self._mark_unconfirmed_submission()
                    response_error = response_error or (
                        f'{name} goal response failed: {exc}'
                    )
                    continue
                try:
                    accepted_by_server = (
                        handle is not None and bool(handle.accepted)
                    )
                except Exception as exc:
                    self._mark_unconfirmed_submission()
                    response_error = response_error or (
                        f'{name} goal acceptance state failed: {exc}'
                    )
                    continue
                if not accepted_by_server:
                    response_error = response_error or (
                        f'{name} goal was rejected'
                    )
                    continue

                self._track_handle(handle)
                try:
                    result_future = handle.get_result_async()
                except Exception as exc:
                    result_future = None
                    response_error = response_error or (
                        f'{name} result request failed: {exc}'
                    )
                if result_future is None and response_error is None:
                    response_error = (
                        f'{name} result request returned no future'
                    )
                accepted.append((name, handle, duration, result_future))

            if response_error is not None:
                if not self._cancel_submitted_and_wait(
                    pending_responses,
                    accepted,
                    server_timeout_sec,
                ):
                    raise PoseCancellationUnconfirmed(
                        f'{response_error}; peer cancellation was not '
                        'confirmed terminal'
                    )
                raise PoseExecutionError(response_error)

            if not pending_responses:
                break
            if time.monotonic() >= response_deadline:
                first_name = pending_responses[0][0]
                if not self._cancel_submitted_and_wait(
                    pending_responses,
                    accepted,
                    server_timeout_sec,
                ):
                    raise PoseCancellationUnconfirmed(
                        f'{first_name} goal response timed out and all '
                        'submitted goals could not be confirmed terminal'
                    )
                raise PoseExecutionError(
                    f'{first_name} goal response timed out'
                )
            time.sleep(0.02)

        pending = [
            (
                name,
                handle,
                result_future,
                (
                    None
                    if name == 'arm'
                    else time.monotonic()
                    + duration
                    + result_timeout_margin_sec
                ),
            )
            for name, handle, duration, result_future in accepted
            if result_future is not None
        ]
        while pending:
            if stop_event.is_set():
                if not self._cancel_and_confirm_results(
                    pending, timeout_sec=server_timeout_sec
                ):
                    raise PoseCancellationUnconfirmed(
                        'Pose stop requested, but controller cancellation '
                        'was not confirmed terminal'
                    )
                raise PoseExecutionError('Pose execution was stopped')

            completed = None
            for item in pending:
                if item[2].done():
                    completed = item
                    break
                if item[3] is not None and time.monotonic() >= item[3]:
                    canceled = self._cancel_and_confirm_results(
                        pending,
                        timeout_sec=server_timeout_sec,
                    )
                    if not canceled:
                        raise PoseCancellationUnconfirmed(
                            f'{item[0]} controller result timed out and '
                            'cancellation was not confirmed terminal'
                        )
                    raise PoseExecutionError(
                        f'{item[0]} controller result timed out'
                    )

            if completed is None:
                time.sleep(0.02)
                continue

            name, handle, future, _deadline = completed
            if not self._future_has_terminal_result(future):
                canceled = self._cancel_and_confirm_results(
                    pending,
                    timeout_sec=server_timeout_sec,
                )
                if not canceled:
                    raise PoseCancellationUnconfirmed(
                        f'{name} controller returned an unconfirmed '
                        'terminal state'
                    )
            try:
                wrapped = future.result()
            except Exception as exc:
                canceled = self._cancel_and_confirm_results(
                    pending,
                    timeout_sec=server_timeout_sec,
                )
                if not canceled:
                    raise PoseCancellationUnconfirmed(
                        f'{name} controller result failed and its terminal '
                        'state could not be confirmed'
                    ) from exc
                raise PoseExecutionError(
                    f'{name} controller result failed: {exc}'
                ) from exc

            self._untrack_handle(handle)
            if not self._result_succeeded(name, wrapped):
                peers = [
                    value for value in pending if value is not completed
                ]
                canceled = self._cancel_and_confirm_results(
                    peers,
                    timeout_sec=server_timeout_sec,
                )
                if not canceled:
                    raise PoseCancellationUnconfirmed(
                        f'{name} controller failed and peer cancellation '
                        'was not confirmed terminal'
                    )
                message = self._result_message(name, wrapped)
                raise PoseExecutionError(
                    f'{name} controller failed: {message}'
                )
            pending.remove(completed)

        dwell = float(pose.get('dwell_sec', 0.0))
        self._interruptible_wait(dwell, stop_event)
        return PoseExecutionResult(True, 'Pose completed')

    def cancel_active(self, *, wait_timeout_sec: float = 0.25) -> bool:
        """Request cancellation without forgetting nonterminal handles."""
        with self._handles_lock:
            handles = list(self._active_handles)
        return self._cancel_handles(
            handles,
            wait_timeout_sec=wait_timeout_sec,
        )

    def destroy(self) -> None:
        self.cancel_active(wait_timeout_sec=0.0)
        self._arm_client.destroy()
        self._gripper_client.destroy()

    def _wait_server(
        self,
        client: Any,
        timeout_sec: float,
        stop_event: threading.Event,
    ) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while not stop_event.is_set() and time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=0.1):
                return True
        return False

    @staticmethod
    def _make_arm_goal(
        pose: dict[str, Any],
        request_id: str,
        created_ms: int,
    ) -> tuple[Any, float]:
        positions = pose.get('arm_positions_deg', [])
        if len(positions) != len(ARM_JOINT_NAMES):
            raise PoseExecutionError('Arm pose must contain exactly 5 angles')
        duration = float(pose.get('arm_duration_sec', 2.0))
        duration_ms = round(duration * 1000.0)
        if not 1 <= duration_ms <= 65535:
            raise PoseExecutionError('Arm duration must be 0.001..65.535 sec')

        goal = ExecuteArmGoal.Goal()
        goal.joint_names = list(ARM_JOINT_NAMES)
        goal.positions = [math.radians(float(value)) for value in positions]
        goal.duration_ms = int(duration_ms)
        goal.request_id = request_id
        goal.web_created_unix_ms = int(created_ms)
        goal.gui_received_unix_ms = int(time.time() * 1000)
        return goal, duration

    @staticmethod
    def _make_gripper_goal(
        pose: dict[str, Any],
    ) -> tuple[Any, float]:
        positions = pose.get('gripper_positions_deg', [])
        if len(positions) != len(GRIPPER_JOINT_NAMES):
            raise PoseExecutionError(
                'Gripper pose must contain exactly 9 angles'
            )
        duration = float(pose.get('gripper_duration_sec', 1.0))
        if not 0.001 <= duration <= 65.535:
            raise PoseExecutionError(
                'Gripper duration must be 0.001..65.535 sec'
            )
        load = int(pose.get('target_load_raw', 500))
        if not 0 <= load <= 1023:
            raise PoseExecutionError('Gripper load must be in range 0..1023')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(GRIPPER_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [
            math.radians(float(value)) for value in positions
        ]
        point.effort = [float(load)] * len(GRIPPER_JOINT_NAMES)
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(
            round((duration - seconds) * 1_000_000_000)
        )
        if point.time_from_start.nanosec >= 1_000_000_000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec -= 1_000_000_000
        goal.trajectory.points = [point]
        return goal, duration

    @staticmethod
    def _result_succeeded(name: str, wrapped: Any) -> bool:
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False
        if name == 'arm':
            return bool(getattr(wrapped.result, 'success', False))
        success_code = FollowJointTrajectory.Result.SUCCESSFUL
        return int(getattr(wrapped.result, 'error_code', -1)) == success_code

    @staticmethod
    def _result_message(name: str, wrapped: Any) -> str:
        if wrapped is None:
            return 'result was not returned'
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return f'action status {wrapped.status}'
        result = wrapped.result
        if name == 'arm':
            return str(getattr(result, 'message', 'hardware reported failure'))
        return str(getattr(result, 'error_string', 'trajectory failed'))

    @staticmethod
    def _await_future(
        future: Any,
        stop_event: threading.Event,
        timeout_sec: float,
        label: str,
    ) -> Any:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while not future.done():
            if stop_event.is_set():
                raise PoseExecutionError(f'{label} was stopped')
            if time.monotonic() >= deadline:
                raise PoseExecutionError(f'{label} timed out')
            time.sleep(0.02)
        return future.result()

    def _track_handle(self, handle: Any) -> None:
        with self._handles_lock:
            if handle not in self._active_handles:
                self._active_handles.append(handle)

    def _untrack_handle(self, handle: Any) -> None:
        with self._handles_lock:
            if handle in self._active_handles:
                self._active_handles.remove(handle)

    def _mark_unconfirmed_submission(self) -> None:
        with self._handles_lock:
            self._unconfirmed_submissions += 1

    @staticmethod
    def _future_has_terminal_result(future: Any) -> bool:
        if not future.done():
            return False
        try:
            wrapped = future.result()
        except Exception:
            return False
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        return getattr(wrapped, 'status', None) in terminal_statuses

    def _defer_untrack_on_result(self, handle: Any, future: Any) -> None:
        def untrack(done_future: Any) -> None:
            if self._future_has_terminal_result(done_future):
                self._untrack_handle(handle)

        try:
            future.add_done_callback(untrack)
        except Exception:
            # Keeping the handle active is safer than forgetting a goal whose
            # terminal state cannot be observed. A later STOP/destroy retries
            # cancellation through cancel_active().
            return

    def _defer_untrack_for_pending_results(
        self,
        pending: Iterable[tuple[str, Any, Any, float | None]],
    ) -> None:
        for _name, handle, future, _deadline in list(pending):
            self._defer_untrack_on_result(handle, future)

    def _abort_accepted(
        self,
        accepted: Iterable[tuple[str, Any, float, Any | None]],
        *,
        timeout_sec: float,
    ) -> bool:
        entries = list(accepted)
        observable: list[tuple[str, Any, Any, float]] = []
        all_observable = True
        for _name, handle, _duration, result_future in entries:
            if result_future is not None:
                self._defer_untrack_on_result(handle, result_future)
                observable.append((
                    _name,
                    handle,
                    result_future,
                    time.monotonic() + max(0.0, timeout_sec),
                ))
            else:
                all_observable = False
        self._cancel_handles(
            (handle for _, handle, _, _ in entries),
            wait_timeout_sec=0.0,
        )
        terminal = self._wait_for_terminal_results(
            observable,
            timeout_sec,
        )
        return all_observable and terminal

    def _cancel_submitted_and_wait(
        self,
        pending_responses: Iterable[tuple[str, Any, float]],
        accepted: Iterable[tuple[str, Any, float, Any | None]],
        timeout_sec: float,
    ) -> bool:
        """Cancel every submission under one shared terminal deadline."""
        started = time.monotonic()
        timeout = max(0.0, float(timeout_sec))
        self._cancel_pending_goal_responses(pending_responses)
        accepted_terminal = self._abort_accepted(
            accepted,
            timeout_sec=timeout,
        )
        remaining = max(0.0, timeout - (time.monotonic() - started))
        return accepted_terminal and self._wait_for_shutdown_drain(remaining)

    def _cancel_pending_goal_responses(
        self,
        pending: Iterable[tuple[str, Any, float]],
    ) -> None:
        for _name, future, _duration in list(pending):
            self._cancel_late_goal_response(future)

    def _cancel_late_goal_response(self, future: Any) -> None:
        """Cancel a goal that gets accepted after its caller has aborted."""
        with self._handles_lock:
            self._late_response_futures.add(future)

        def cancel_if_accepted(done_future: Any) -> None:
            try:
                try:
                    handle = done_future.result()
                except Exception:
                    self._mark_unconfirmed_submission()
                    return
                if handle is None or not handle.accepted:
                    return
                self._track_handle(handle)
                try:
                    result_future = handle.get_result_async()
                except Exception:
                    result_future = None
                if result_future is not None:
                    self._defer_untrack_on_result(handle, result_future)
                # This callback can run in a ROS executor thread. Do not block
                # it waiting for the cancellation service response.
                self._cancel_handles([handle], wait_timeout_sec=0.0)
            finally:
                with self._handles_lock:
                    self._late_response_futures.discard(done_future)

        try:
            future.add_done_callback(cancel_if_accepted)
        except Exception:
            # Some future implementations reject callbacks after completion.
            # Handle that race synchronously when possible.
            try:
                if future.done():
                    cancel_if_accepted(future)
            except Exception:
                pass

    def _cancel_and_confirm_results(
        self,
        pending: Iterable[tuple[str, Any, Any, float | None]],
        *,
        timeout_sec: float,
    ) -> bool:
        """Request cancellation and require every result to be terminal."""
        entries = list(pending)
        self._defer_untrack_for_pending_results(entries)
        self._cancel_handles(
            (handle for _name, handle, _future, _deadline in entries),
            wait_timeout_sec=0.0,
        )
        return self._wait_for_terminal_results(entries, timeout_sec)

    def _wait_for_terminal_results(
        self,
        pending: Iterable[tuple[str, Any, Any, float | None]],
        timeout_sec: float,
    ) -> bool:
        entries = list(pending)
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while True:
            if all(
                self._future_has_terminal_result(future)
                for _, _, future, _ in entries
            ):
                for _name, handle, _future, _item_deadline in entries:
                    self._untrack_handle(handle)
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

    def _wait_for_shutdown_drain(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while not self.shutdown_drained:
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)
        return True

    def _cancel_handles(
        self,
        handles: Iterable[Any],
        *,
        wait_timeout_sec: float = 0.25,
    ) -> bool:
        cancel_futures: list[Any] = []
        submitted = True
        seen: set[int] = set()
        for handle in list(handles):
            if handle is None:
                continue
            identity = id(handle)
            if identity in seen:
                continue
            seen.add(identity)
            try:
                cancel_future = handle.cancel_goal_async()
            except Exception:
                submitted = False
                continue
            if cancel_future is not None:
                cancel_futures.append(cancel_future)
            else:
                submitted = False

        deadline = time.monotonic() + max(0.0, float(wait_timeout_sec))
        while cancel_futures and time.monotonic() < deadline:
            cancel_futures = [
                future for future in cancel_futures if not future.done()
            ]
            if cancel_futures:
                time.sleep(0.01)
        return submitted and all(future.done() for future in cancel_futures)

    @staticmethod
    def _interruptible_wait(
        duration_sec: float,
        stop_event: threading.Event,
    ) -> None:
        if duration_sec <= 0.0:
            return
        if stop_event.wait(duration_sec):
            raise PoseExecutionError('Pose dwell was stopped')
