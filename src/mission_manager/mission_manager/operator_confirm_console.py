"""Provide an operator-confirmation action controlled by terminal Enter."""

import json
import select
import sys
import termios
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vicpinky_interfaces.action import RunTask


class OperatorGateState:
    """Thread-safe state shared by ROS callbacks and the console loop."""

    def __init__(self):
        self._condition = threading.Condition()
        self._reserved = False
        self._active_token = 0
        self._active_prompt: Optional[str] = None
        self._confirmed_token: Optional[int] = None

    def reserve(self) -> bool:
        """Reserve the only gate slot for an accepted action goal."""
        with self._condition:
            if self._reserved:
                return False
            self._reserved = True
            return True

    def activate(self, prompt: str) -> int:
        """Make a reserved goal visible to the console and return its token."""
        with self._condition:
            if not self._reserved:
                raise RuntimeError('operator gate was not reserved')
            self._active_token += 1
            self._active_prompt = prompt
            self._confirmed_token = None
            self._condition.notify_all()
            return self._active_token

    def snapshot(self) -> Optional[Tuple[int, str]]:
        """Return the active token and prompt, if a gate is waiting."""
        with self._condition:
            if self._active_prompt is None:
                return None
            return self._active_token, self._active_prompt

    def confirm(self, token: int) -> bool:
        """Confirm only the exact gate currently displayed to the operator."""
        with self._condition:
            if (
                self._active_prompt is None
                or token != self._active_token
            ):
                return False
            self._confirmed_token = token
            self._condition.notify_all()
            return True

    def is_confirmed(self, token: int) -> bool:
        """Return whether Enter confirmed the supplied active token."""
        with self._condition:
            return self._confirmed_token == token

    def wait(self, timeout_sec: float) -> None:
        """Wait briefly for confirmation or goal-state changes."""
        with self._condition:
            self._condition.wait(timeout=timeout_sec)

    def finish(self, token: int) -> None:
        """Release the slot without allowing confirmation to leak forward."""
        with self._condition:
            if token == self._active_token:
                self._active_prompt = None
                self._confirmed_token = None
            self._reserved = False
            self._condition.notify_all()


def prompt_from_goal(goal: RunTask.Goal) -> str:
    """Resolve a human-readable prompt from one RunTask goal."""
    payload = {}
    try:
        decoded = json.loads(goal.extra_json or '{}')
        if isinstance(decoded, dict):
            payload = decoded
    except (TypeError, ValueError, json.JSONDecodeError):
        pass

    prompt = str(payload.get('prompt') or '').strip()
    if prompt:
        return prompt

    target = str(goal.target_name or '').strip()
    if target:
        return target

    return '안전 상태를 확인한 뒤 Enter를 누르세요.'


class OperatorConfirmConsole(Node):
    """Expose ``/operator/confirm`` and complete it on terminal Enter."""

    def __init__(self):
        super().__init__('operator_confirm_console')
        self._gate = OperatorGateState()
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            RunTask,
            '/operator/confirm',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            'Operator confirmation console ready: /operator/confirm'
        )

    def _goal_callback(self, goal_request: RunTask.Goal) -> GoalResponse:
        task_id = str(goal_request.task_id or '').strip()
        if task_id not in {'', 'operator_confirm'}:
            self.get_logger().warning(
                f'Rejecting unsupported operator task_id: {task_id}'
            )
            return GoalResponse.REJECT
        if not self._gate.reserve():
            self.get_logger().warning(
                'Rejecting operator confirmation: another gate is active'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> RunTask.Result:
        result = RunTask.Result()
        prompt = prompt_from_goal(goal_handle.request)
        token = self._gate.activate(prompt)
        self.get_logger().warning(f'OPERATOR WAIT: {prompt}')

        next_feedback_at = 0.0
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = 'Operator confirmation canceled'
                    goal_handle.canceled()
                    return result

                if self._gate.is_confirmed(token):
                    result.success = True
                    result.message = 'Operator confirmed with Enter'
                    goal_handle.succeed()
                    self.get_logger().info(result.message)
                    return result

                now = time.monotonic()
                if now >= next_feedback_at:
                    feedback = RunTask.Feedback()
                    feedback.phase = 'WAIT_OPERATOR'
                    feedback.progress = 0.0
                    feedback.detail = prompt
                    goal_handle.publish_feedback(feedback)
                    next_feedback_at = now + 1.0

                self._gate.wait(0.1)

            result.success = False
            result.message = 'ROS shutdown while waiting for operator'
            return result
        finally:
            self._gate.finish(token)

    @staticmethod
    def _flush_pending_input() -> None:
        """Discard Enter presses queued before the current prompt appeared."""
        try:
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
        except (AttributeError, OSError, termios.error):
            pass

    @staticmethod
    def _readline(timeout_sec: float) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return None
        line = sys.stdin.readline()
        if line == '':
            raise EOFError('operator console stdin closed')
        return line

    def run_console(self) -> None:
        """Wait for active gates and consume exactly one fresh Enter each."""
        displayed_token: Optional[int] = None

        while rclpy.ok():
            active = self._gate.snapshot()
            if active is None:
                displayed_token = None
                # Drain keys entered while no gate is active.
                self._readline(0.1)
                continue

            token, prompt = active
            if displayed_token != token:
                self._flush_pending_input()
                print(
                    f'\n[주행 수동 확인] {prompt}\n'
                    '계속 진행하려면 Enter를 누르세요: ',
                    end='',
                    flush=True,
                )
                displayed_token = token

            line = self._readline(0.1)
            if line is None:
                continue
            if self._gate.confirm(token):
                print('[확인됨] 다음 단계로 진행합니다.', flush=True)

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OperatorConfirmConsole()

    if not sys.stdin.isatty():
        node.get_logger().error(
            'A terminal is required. Run this executable with ros2 run in '
            'a separate interactive terminal.'
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_console()
    except (EOFError, KeyboardInterrupt) as exc:
        node.get_logger().warning(f'Operator console stopped: {exc}')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
