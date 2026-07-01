import time
from typing import Callable, Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from vicpinky_interfaces.action import RunTask

from .mission_state import MissionStep, TaskExecutionResult


TaskFeedbackCallback = Callable[
    [MissionStep, RunTask.Feedback],
    None,
]


class TaskExecutor:
    """RunTask Action Server 하나를 안정적으로 호출하는 공통 실행기."""

    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
    ):
        self._node = node
        self._callback_group = callback_group

        # 같은 Action Server를 여러 단계에서 호출할 수 있으므로
        # Client를 한 번 만든 뒤 재사용한다.
        self._clients: Dict[str, ActionClient] = {}

    def _get_client(self, server_name: str) -> ActionClient:
        if server_name not in self._clients:
            self._clients[server_name] = ActionClient(
                self._node,
                RunTask,
                server_name,
                callback_group=self._callback_group,
            )

        return self._clients[server_name]

    def _cancel_child_goal(
        self,
        child_goal_handle,
        timeout_sec: float = 2.0,
    ) -> None:
        try:
            cancel_future = child_goal_handle.cancel_goal_async()
            deadline = time.monotonic() + timeout_sec

            while (
                rclpy.ok()
                and not cancel_future.done()
                and time.monotonic() < deadline
            ):
                time.sleep(0.05)

        except Exception as exc:
            self._node.get_logger().warning(
                f'Failed to cancel child goal: {exc}'
            )

    def execute(
        self,
        step: MissionStep,
        mission_goal_handle,
        feedback_callback: Optional[TaskFeedbackCallback] = None,
    ) -> TaskExecutionResult:
        """
        Execute one MissionStep on a child RunTask Action Server.

        이 함수는 동기적으로 결과를 기다리지만,
        MissionManager는 MultiThreadedExecutor를 사용하므로
        다른 스레드에서 Action feedback/result callback을 처리할 수 있다.
        """
        client = self._get_client(step.server)
        deadline = time.monotonic() + step.timeout_sec

        self._node.get_logger().info(
            f'Waiting for Action Server: {step.server}'
        )

        while not client.wait_for_server(timeout_sec=0.25):
            if mission_goal_handle.is_cancel_requested:
                return TaskExecutionResult(
                    success=False,
                    message=(
                        f'Mission canceled while waiting for '
                        f'{step.server}'
                    ),
                    canceled=True,
                )

            if time.monotonic() >= deadline:
                return TaskExecutionResult(
                    success=False,
                    message=(
                        f'Action Server not available before timeout: '
                        f'{step.server}'
                    ),
                    timed_out=True,
                )

        task_goal = RunTask.Goal()
        task_goal.task_id = step.task_id
        task_goal.target_name = step.target_name
        task_goal.target_floor = step.target_floor
        task_goal.marker_id = step.marker_id
        task_goal.extra_json = step.extra_json

        def child_feedback_handler(feedback_message):
            if feedback_callback is not None:
                feedback_callback(
                    step,
                    feedback_message.feedback,
                )

        self._node.get_logger().info(
            f'Sending child goal: '
            f'state={step.state}, '
            f'server={step.server}, '
            f'task_id={step.task_id}, '
            f'target={step.target_name}, '
            f'floor={step.target_floor}, '
            f'marker={step.marker_id}'
        )

        send_goal_future = client.send_goal_async(
            task_goal,
            feedback_callback=child_feedback_handler,
        )

        while rclpy.ok() and not send_goal_future.done():
            if time.monotonic() >= deadline:
                return TaskExecutionResult(
                    success=False,
                    message=(
                        f'Timeout while sending goal to '
                        f'{step.server}'
                    ),
                    timed_out=True,
                )

            time.sleep(0.05)

        child_goal_handle = send_goal_future.result()

        if child_goal_handle is None:
            return TaskExecutionResult(
                success=False,
                message=(
                    f'No goal handle returned from {step.server}'
                ),
            )

        if not child_goal_handle.accepted:
            return TaskExecutionResult(
                success=False,
                message=f'Goal rejected by {step.server}',
            )

        if mission_goal_handle.is_cancel_requested:
            self._cancel_child_goal(child_goal_handle)

            return TaskExecutionResult(
                success=False,
                message=f'Mission canceled during {step.state}',
                canceled=True,
            )

        result_future = child_goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            if mission_goal_handle.is_cancel_requested:
                self._cancel_child_goal(child_goal_handle)

                return TaskExecutionResult(
                    success=False,
                    message=f'Mission canceled during {step.state}',
                    canceled=True,
                )

            if time.monotonic() >= deadline:
                self._cancel_child_goal(child_goal_handle)

                return TaskExecutionResult(
                    success=False,
                    message=(
                        f'Timeout during {step.state} '
                        f'on {step.server}'
                    ),
                    timed_out=True,
                )

            time.sleep(0.05)

        result_response = result_future.result()

        if result_response is None:
            return TaskExecutionResult(
                success=False,
                message=f'No result returned from {step.server}',
            )

        child_result = result_response.result

        return TaskExecutionResult(
            success=bool(child_result.success),
            message=str(child_result.message),
        )
