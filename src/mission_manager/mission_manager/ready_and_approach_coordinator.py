"""Coordinate arm-ready and delayed straight-drive RunTask actions."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Optional

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vicpinky_interfaces.action import RunTask

from .ready_and_approach_logic import (
    child_extra_json,
    ChildOutcome,
    combine_child_outcomes,
    parse_ready_and_approach_request,
    ReadyAndApproachConfigError,
)


POLL_PERIOD_SEC = 0.02


class CoordinationError(RuntimeError):
    """Raised when the coordinator must abort its parent action."""


class CoordinationCanceled(CoordinationError):
    """Raised when cancellation of the parent action is requested."""


@dataclass
class ActiveChild:
    """Accepted child goal and its pending result."""

    name: str
    goal_handle: Any
    result_future: Any


class ReadyAndApproachCoordinator(Node):
    """Expose /mission/ready_and_approach as one atomic RunTask action."""

    def __init__(self) -> None:
        super().__init__('ready_and_approach_coordinator')

        self.declare_parameter(
            'action_name',
            '/mission/ready_and_approach',
        )
        self.declare_parameter('arm_action_name', '/arm/execute')
        self.declare_parameter(
            'base_action_name',
            '/base/drive_straight',
        )
        self.declare_parameter('server_wait_timeout_sec', 5.0)
        self.declare_parameter('goal_response_timeout_sec', 5.0)
        self.declare_parameter('execution_timeout_sec', 80.0)
        self.declare_parameter('cancel_timeout_sec', 2.0)

        self._action_name = str(self.get_parameter('action_name').value)
        self._arm_action_name = str(
            self.get_parameter('arm_action_name').value
        )
        self._base_action_name = str(
            self.get_parameter('base_action_name').value
        )
        self._server_wait_timeout_sec = self._positive_parameter(
            'server_wait_timeout_sec'
        )
        self._goal_response_timeout_sec = self._positive_parameter(
            'goal_response_timeout_sec'
        )
        self._execution_timeout_sec = self._positive_parameter(
            'execution_timeout_sec'
        )
        self._cancel_timeout_sec = self._positive_parameter(
            'cancel_timeout_sec'
        )

        self._callback_group = ReentrantCallbackGroup()
        self._reservation_lock = threading.Lock()
        self._execution_reserved = False

        self._arm_client = ActionClient(
            self,
            RunTask,
            self._arm_action_name,
            callback_group=self._callback_group,
        )
        self._base_client = ActionClient(
            self,
            RunTask,
            self._base_action_name,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            RunTask,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'Ready-and-approach coordinator ready: '
            f'{self._action_name} -> '
            f'{self._arm_action_name} + {self._base_action_name}'
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def destroy_node(self) -> bool:
        """Release action entities before destroying the node."""
        self._action_server.destroy()
        self._arm_client.destroy()
        self._base_client.destroy()
        return super().destroy_node()

    def _goal_callback(self, goal_request: RunTask.Goal) -> GoalResponse:
        if (
            goal_request.task_id
            and goal_request.task_id != 'ready_and_approach'
        ):
            self.get_logger().warning(
                'Rejecting unsupported coordinator task_id: '
                f'{goal_request.task_id}'
            )
            return GoalResponse.REJECT

        try:
            parse_ready_and_approach_request(goal_request.extra_json)
        except ReadyAndApproachConfigError as exc:
            self.get_logger().warning(f'Rejecting unsafe goal: {exc}')
            return GoalResponse.REJECT

        with self._reservation_lock:
            if self._execution_reserved:
                self.get_logger().warning(
                    'Rejecting ready-and-approach goal: already running'
                )
                return GoalResponse.REJECT
            self._execution_reserved = True

        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().warning(
            'Ready-and-approach cancellation requested'
        )
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> RunTask.Result:
        active_children: dict[str, ActiveChild] = {}
        feedback_enabled = threading.Event()
        feedback_enabled.set()

        try:
            request = parse_ready_and_approach_request(
                goal_handle.request.extra_json
            )
            deadline = time.monotonic() + self._execution_timeout_sec

            self._publish_feedback(
                goal_handle,
                phase='preflight',
                progress=0.02,
                detail='Checking arm and base action servers',
            )
            self._wait_for_server(
                self._arm_client,
                'arm',
                goal_handle,
                deadline,
            )
            self._wait_for_server(
                self._base_client,
                'base',
                goal_handle,
                deadline,
            )

            arm_extra, base_extra = child_extra_json(request)
            arm_goal = self._make_child_goal(
                goal_handle.request,
                task_id='arm_execute',
                target_name='robot_arm',
                extra_json=arm_extra,
            )
            arm_child = self._send_child_goal(
                name='arm',
                client=self._arm_client,
                child_goal=arm_goal,
                parent_goal_handle=goal_handle,
                overall_deadline=deadline,
                feedback_enabled=feedback_enabled,
            )
            active_children['arm'] = arm_child

            arm_started_at = time.monotonic()
            drive_start_at = (
                arm_started_at
                + request.arm_start_to_drive_delay_sec
            )
            self._publish_feedback(
                goal_handle,
                phase='arm_started',
                progress=0.15,
                detail=(
                    'arm_ready accepted; drive starts after '
                    f'{request.arm_start_to_drive_delay_sec:.3f} s'
                ),
            )

            early_arm_outcome = self._wait_for_drive_start(
                arm_child,
                goal_handle,
                drive_start_at,
                deadline,
            )
            if (
                early_arm_outcome is not None
                and not early_arm_outcome.success
            ):
                decision = combine_child_outcomes([early_arm_outcome])
                raise CoordinationError(decision.message)

            base_goal = self._make_child_goal(
                goal_handle.request,
                task_id='drive_straight',
                target_name=goal_handle.request.target_name,
                extra_json=base_extra,
            )
            base_child = self._send_child_goal(
                name='base',
                client=self._base_client,
                child_goal=base_goal,
                parent_goal_handle=goal_handle,
                overall_deadline=deadline,
                feedback_enabled=feedback_enabled,
                watch_child=arm_child,
            )
            active_children['base'] = base_child

            self._publish_feedback(
                goal_handle,
                phase='children_running',
                progress=0.45,
                detail=(
                    f'Driving {request.distance_m:.3f} m at '
                    f'{request.speed_mps:.3f} m/s; joining child results'
                ),
            )

            initial_outcomes = {}
            if early_arm_outcome is not None:
                initial_outcomes['arm'] = early_arm_outcome

            decision = self._join_children(
                active_children,
                goal_handle,
                deadline,
                initial_outcomes,
            )
            if not decision.success:
                raise CoordinationError(decision.message)

            self._publish_feedback(
                goal_handle,
                phase='completed',
                progress=1.0,
                detail=decision.message,
            )
            goal_handle.succeed()
            return self._result(True, decision.message)

        except CoordinationCanceled as exc:
            feedback_enabled.clear()
            self._cancel_children(active_children)
            goal_handle.canceled()
            return self._result(False, str(exc))
        except (CoordinationError, ReadyAndApproachConfigError) as exc:
            feedback_enabled.clear()
            self._cancel_children(active_children)
            goal_handle.abort()
            return self._result(False, str(exc))
        except Exception as exc:
            feedback_enabled.clear()
            self._cancel_children(active_children)
            self.get_logger().error(
                f'Unexpected ready-and-approach failure: {exc}'
            )
            goal_handle.abort()
            return self._result(
                False,
                f'Unexpected ready-and-approach failure: {exc}',
            )
        finally:
            feedback_enabled.clear()
            with self._reservation_lock:
                self._execution_reserved = False

    def _wait_for_server(
        self,
        client: ActionClient,
        name: str,
        parent_goal_handle,
        overall_deadline: float,
    ) -> None:
        server_deadline = min(
            overall_deadline,
            time.monotonic() + self._server_wait_timeout_sec,
        )

        while rclpy.ok():
            self._raise_if_canceled(parent_goal_handle)
            remaining = server_deadline - time.monotonic()
            if remaining <= 0.0:
                raise CoordinationError(
                    f'Timed out waiting for {name} action server'
                )

            if client.wait_for_server(
                timeout_sec=min(0.1, remaining)
            ):
                return

        raise CoordinationError(
            f'ROS shutdown while waiting for {name} action server'
        )

    def _send_child_goal(
        self,
        *,
        name: str,
        client: ActionClient,
        child_goal: RunTask.Goal,
        parent_goal_handle,
        overall_deadline: float,
        feedback_enabled: threading.Event,
        watch_child: Optional[ActiveChild] = None,
    ) -> ActiveChild:
        watch_failure = self._child_failure_if_done(watch_child)
        if watch_failure is not None:
            raise CoordinationError(watch_failure)

        send_future = client.send_goal_async(
            child_goal,
            feedback_callback=lambda message: self._child_feedback(
                parent_goal_handle,
                name,
                message,
                feedback_enabled,
            ),
        )
        response_deadline = min(
            overall_deadline,
            time.monotonic() + self._goal_response_timeout_sec,
        )

        while rclpy.ok() and not send_future.done():
            if parent_goal_handle.is_cancel_requested:
                self._cancel_late_goal(send_future, name)
                raise CoordinationCanceled(
                    'Ready-and-approach canceled while sending '
                    f'{name} goal'
                )
            watch_failure = self._child_failure_if_done(watch_child)
            if watch_failure is not None:
                self._cancel_late_goal(send_future, name)
                raise CoordinationError(watch_failure)
            if time.monotonic() >= response_deadline:
                self._cancel_late_goal(send_future, name)
                raise CoordinationError(
                    f'Timed out waiting for {name} goal response'
                )
            time.sleep(POLL_PERIOD_SEC)

        if not send_future.done():
            raise CoordinationError(
                f'ROS shutdown while sending {name} goal'
            )

        watch_failure = self._child_failure_if_done(watch_child)
        if watch_failure is not None:
            self._cancel_late_goal(send_future, name)
            raise CoordinationError(watch_failure)

        try:
            child_goal_handle = send_future.result()
        except Exception as exc:
            raise CoordinationError(
                f'{name} goal request failed: {exc}'
            ) from exc

        if child_goal_handle is None:
            raise CoordinationError(
                f'{name} action returned no goal handle'
            )
        if not child_goal_handle.accepted:
            raise CoordinationError(f'{name} action rejected its goal')

        return ActiveChild(
            name=name,
            goal_handle=child_goal_handle,
            result_future=child_goal_handle.get_result_async(),
        )

    def _child_failure_if_done(
        self,
        child: Optional[ActiveChild],
    ) -> Optional[str]:
        if child is None or not child.result_future.done():
            return None

        outcome = self._child_outcome(child)
        if outcome.success:
            return None

        return combine_child_outcomes([outcome]).message

    def _wait_for_drive_start(
        self,
        arm_child: ActiveChild,
        parent_goal_handle,
        drive_start_at: float,
        overall_deadline: float,
    ) -> Optional[ChildOutcome]:
        arm_outcome = None

        while rclpy.ok() and time.monotonic() < drive_start_at:
            self._raise_if_canceled(parent_goal_handle)
            self._raise_if_timed_out(overall_deadline)

            if arm_outcome is None and arm_child.result_future.done():
                arm_outcome = self._child_outcome(arm_child)
                if not arm_outcome.success:
                    return arm_outcome

            remaining = max(0.0, drive_start_at - time.monotonic())
            time.sleep(min(POLL_PERIOD_SEC, remaining))

        if not rclpy.ok():
            raise CoordinationError(
                'ROS shutdown before delayed base start'
            )
        self._raise_if_canceled(parent_goal_handle)
        self._raise_if_timed_out(overall_deadline)
        return arm_outcome

    def _join_children(
        self,
        active_children: dict[str, ActiveChild],
        parent_goal_handle,
        overall_deadline: float,
        initial_outcomes: dict[str, ChildOutcome],
    ):
        outcomes = dict(initial_outcomes)

        while rclpy.ok():
            self._raise_if_canceled(parent_goal_handle)
            self._raise_if_timed_out(overall_deadline)

            for name, child in active_children.items():
                if name in outcomes or not child.result_future.done():
                    continue

                outcome = self._child_outcome(child)
                outcomes[name] = outcome
                if not outcome.success:
                    decision = combine_child_outcomes(outcomes.values())
                    raise CoordinationError(decision.message)

            if len(outcomes) == len(active_children):
                return combine_child_outcomes(outcomes.values())

            time.sleep(POLL_PERIOD_SEC)

        raise CoordinationError('ROS shutdown while joining child results')

    def _child_outcome(self, child: ActiveChild) -> ChildOutcome:
        try:
            response = child.result_future.result()
        except Exception as exc:
            return ChildOutcome(
                name=child.name,
                success=False,
                message=f'result request failed: {exc}',
            )

        if response is None:
            return ChildOutcome(
                name=child.name,
                success=False,
                message='action returned no result',
            )

        status = int(response.status)
        result = response.result
        child_message = str(
            getattr(result, 'message', '') or 'no child result message'
        )
        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and bool(getattr(result, 'success', False))
        )
        canceled = status == GoalStatus.STATUS_CANCELED

        if succeeded:
            return ChildOutcome(
                name=child.name,
                success=True,
                message=child_message,
            )

        status_name = self._status_name(status)
        return ChildOutcome(
            name=child.name,
            success=False,
            canceled=canceled,
            message=f'{status_name}: {child_message}',
        )

    def _cancel_children(
        self,
        active_children: dict[str, ActiveChild],
    ) -> None:
        cancel_futures = []

        for child in active_children.values():
            if child.result_future.done():
                continue
            try:
                cancel_futures.append(
                    (child.name, child.goal_handle.cancel_goal_async())
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to request {child.name} cancellation: {exc}'
                )

        deadline = time.monotonic() + self._cancel_timeout_sec
        while rclpy.ok() and cancel_futures:
            cancel_futures = [
                item for item in cancel_futures if not item[1].done()
            ]
            if not cancel_futures or time.monotonic() >= deadline:
                break
            time.sleep(POLL_PERIOD_SEC)

        for name, future in cancel_futures:
            if not future.done():
                self.get_logger().warning(
                    f'Timed out waiting for {name} cancellation response'
                )

    def _cancel_late_goal(self, send_future, name: str) -> None:
        def cancel_when_available(completed_future) -> None:
            try:
                late_goal_handle = completed_future.result()
                if late_goal_handle is not None and late_goal_handle.accepted:
                    late_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to cancel late {name} goal: {exc}'
                )

        if send_future.done():
            cancel_when_available(send_future)
            return

        # A goal cannot be canceled by ID until its response contains a goal
        # handle. Registering this callback prevents an orphaned motion while
        # allowing cancellation of already accepted siblings without delay.
        send_future.add_done_callback(cancel_when_available)

    @staticmethod
    def _make_child_goal(
        parent_goal: RunTask.Goal,
        *,
        task_id: str,
        target_name: str,
        extra_json: str,
    ) -> RunTask.Goal:
        child_goal = RunTask.Goal()
        child_goal.task_id = task_id
        child_goal.target_name = target_name
        child_goal.target_floor = parent_goal.target_floor
        child_goal.marker_id = parent_goal.marker_id
        child_goal.extra_json = extra_json
        return child_goal

    def _child_feedback(
        self,
        parent_goal_handle,
        child_name: str,
        feedback_message,
        feedback_enabled: threading.Event,
    ) -> None:
        if not feedback_enabled.is_set():
            return

        child_feedback = feedback_message.feedback
        child_progress = float(
            max(0.0, min(1.0, child_feedback.progress))
        )
        if child_name == 'arm':
            progress = 0.15 + 0.3 * child_progress
        else:
            progress = 0.45 + 0.5 * child_progress

        self._publish_feedback(
            parent_goal_handle,
            phase=f'{child_name}:{child_feedback.phase}',
            progress=progress,
            detail=child_feedback.detail,
        )

    @staticmethod
    def _publish_feedback(
        goal_handle,
        *,
        phase: str,
        progress: float,
        detail: str,
    ) -> None:
        feedback = RunTask.Feedback()
        feedback.phase = str(phase)
        feedback.progress = float(max(0.0, min(1.0, progress)))
        feedback.detail = str(detail)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _raise_if_canceled(parent_goal_handle) -> None:
        if parent_goal_handle.is_cancel_requested:
            raise CoordinationCanceled('Ready-and-approach canceled')

    @staticmethod
    def _raise_if_timed_out(overall_deadline: float) -> None:
        if time.monotonic() >= overall_deadline:
            raise CoordinationError(
                'Ready-and-approach execution timed out'
            )

    @staticmethod
    def _result(success: bool, message: str) -> RunTask.Result:
        result = RunTask.Result()
        result.success = bool(success)
        result.message = str(message)
        return result

    @staticmethod
    def _status_name(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, str(status))


def main(args: Optional[list[str]] = None) -> None:
    """Run the ready-and-approach coordinator with concurrent callbacks."""
    rclpy.init(args=args)
    node = ReadyAndApproachCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
