"""Convert mission RunTask navigation goals into Nav2 NavigateToPose goals."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
from typing import Any, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vicpinky_interfaces.action import RunTask


@dataclass(frozen=True)
class NavigationTarget:
    """Navigation target resolved from RunTask.extra_json."""

    x: float
    y: float
    yaw: float
    frame_id: str


class VicPinkyNavAdapter(Node):
    """Expose /nav/go_to and forward accepted goals to Nav2."""

    def __init__(self) -> None:
        super().__init__('vicpinky_nav_adapter')

        self.declare_parameter('run_task_action_name', '/nav/go_to')
        self.declare_parameter('navigate_action_name', '/navigate_to_pose')
        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('nav_server_wait_timeout_sec', 5.0)
        self.declare_parameter('goal_response_timeout_sec', 10.0)
        self.declare_parameter('feedback_log_period_sec', 1.0)

        self._run_task_action_name = str(
            self.get_parameter('run_task_action_name').value
        )
        self._navigate_action_name = str(
            self.get_parameter('navigate_action_name').value
        )
        self._default_frame_id = str(
            self.get_parameter('default_frame_id').value
        )
        self._nav_server_wait_timeout_sec = float(
            self.get_parameter('nav_server_wait_timeout_sec').value
        )
        self._goal_response_timeout_sec = float(
            self.get_parameter('goal_response_timeout_sec').value
        )
        self._feedback_log_period_sec = float(
            self.get_parameter('feedback_log_period_sec').value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            self._navigate_action_name,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            RunTask,
            self._run_task_action_name,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self._last_feedback_log_time = 0.0

        self.get_logger().info(
            'vicpinky_nav_adapter ready: '
            f'{self._run_task_action_name} -> {self._navigate_action_name}'
        )

    def destroy_node(self) -> bool:
        """Release action entities before node shutdown."""
        self._action_server.destroy()
        self._nav_client.destroy()
        return super().destroy_node()

    def _goal_callback(self, goal_request: RunTask.Goal) -> GoalResponse:
        if goal_request.task_id and goal_request.task_id != 'go_to':
            self.get_logger().warning(
                f'Reject navigation task_id={goal_request.task_id!r}'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> RunTask.Result:
        goal = goal_handle.request
        result = RunTask.Result()

        try:
            target = self._target_from_goal(goal)
        except ValueError as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

        self._publish_feedback(
            goal_handle,
            phase='sending_goal',
            progress=0.05,
            detail=(
                f'Navigate to {goal.target_name}: '
                f'x={target.x:.3f}, y={target.y:.3f}, '
                f'yaw={target.yaw:.3f}, frame={target.frame_id}'
            ),
        )

        if not self._wait_for_nav_server(goal_handle):
            return self._finish_canceled_or_aborted(
                goal_handle,
                result,
                'Nav2 NavigateToPose action server is not available',
            )

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._build_pose(target)

        send_future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda feedback: self._nav_feedback(
                goal_handle,
                feedback,
            ),
        )

        send_response = self._wait_for_future(
            send_future,
            goal_handle,
            timeout_sec=self._goal_response_timeout_sec,
        )

        if goal_handle.is_cancel_requested:
            return self._finish_canceled(goal_handle, result)

        if send_response is None:
            return self._finish_aborted(
                goal_handle,
                result,
                'Timed out while sending Nav2 goal',
            )

        nav_goal_handle = send_response

        if not nav_goal_handle.accepted:
            return self._finish_aborted(
                goal_handle,
                result,
                'Nav2 rejected NavigateToPose goal',
            )

        self._publish_feedback(
            goal_handle,
            phase='navigating',
            progress=0.2,
            detail='Nav2 accepted goal',
        )

        result_future = nav_goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_nav_goal(nav_goal_handle)
                return self._finish_canceled(goal_handle, result)

            time.sleep(0.05)

        if goal_handle.is_cancel_requested:
            self._cancel_nav_goal(nav_goal_handle)
            return self._finish_canceled(goal_handle, result)

        nav_result_response = result_future.result()

        if nav_result_response is None:
            return self._finish_aborted(
                goal_handle,
                result,
                'No result returned from Nav2',
            )

        if nav_result_response.status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_feedback(
                goal_handle,
                phase='arrived',
                progress=1.0,
                detail='Navigation completed',
            )
            goal_handle.succeed()
            result.success = True
            result.message = (
                f'Arrived at {goal.target_name or "navigation target"}'
            )
            return result

        status_name = self._status_name(nav_result_response.status)
        return self._finish_aborted(
            goal_handle,
            result,
            f'Nav2 finished with status {status_name}',
        )

    def _target_from_goal(self, goal: RunTask.Goal) -> NavigationTarget:
        payload = self._parse_extra_json(goal.extra_json)
        pose = payload.get('pose')

        if pose is None:
            raise ValueError(
                'Navigation goal has no pose. Add pose to '
                'mission_manager/config/locations.yaml for this location.'
            )

        if not isinstance(pose, dict):
            raise ValueError('extra_json.pose must be a mapping')

        try:
            x = float(pose['x'])
            y = float(pose['y'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                'extra_json.pose must contain numeric x and y'
            ) from exc

        yaw = self._extract_yaw(pose)
        frame_id = str(pose.get('frame_id', self._default_frame_id))

        if not frame_id:
            raise ValueError('extra_json.pose.frame_id cannot be empty')

        return NavigationTarget(x=x, y=y, yaw=yaw, frame_id=frame_id)

    def _parse_extra_json(self, extra_json: str) -> dict[str, Any]:
        if not extra_json:
            return {}

        try:
            payload = json.loads(extra_json)
        except json.JSONDecodeError as exc:
            raise ValueError(f'Invalid extra_json: {exc}') from exc

        if not isinstance(payload, dict):
            raise ValueError('extra_json root must be a mapping')

        return payload

    def _extract_yaw(self, pose: dict[str, Any]) -> float:
        if 'yaw' in pose:
            return float(pose['yaw'])

        if 'theta' in pose:
            return float(pose['theta'])

        orientation = pose.get('orientation')

        if isinstance(orientation, dict):
            z = float(orientation.get('z', 0.0))
            w = float(orientation.get('w', 1.0))
            return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

        return 0.0

    def _build_pose(self, target: NavigationTarget) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = target.frame_id
        pose.pose.position.x = target.x
        pose.pose.position.y = target.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(target.yaw * 0.5)
        pose.pose.orientation.w = math.cos(target.yaw * 0.5)
        return pose

    def _wait_for_nav_server(self, goal_handle) -> bool:
        deadline = time.monotonic() + self._nav_server_wait_timeout_sec

        while rclpy.ok():
            if self._nav_client.wait_for_server(timeout_sec=0.25):
                return True

            if goal_handle.is_cancel_requested:
                return False

            if time.monotonic() >= deadline:
                return False

        return False

    def _wait_for_future(
        self,
        future,
        goal_handle,
        *,
        timeout_sec: Optional[float],
    ):
        deadline = None

        if timeout_sec is not None:
            deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                return None

            if deadline is not None and time.monotonic() >= deadline:
                return None

            time.sleep(0.05)

        if not future.done():
            return None

        return future.result()

    def _nav_feedback(self, goal_handle, feedback_message) -> None:
        feedback = feedback_message.feedback
        now = time.monotonic()

        if (
            now - self._last_feedback_log_time
            < self._feedback_log_period_sec
        ):
            return

        self._last_feedback_log_time = now

        distance = getattr(feedback, 'distance_remaining', None)
        detail = 'Nav2 feedback received'
        progress = 0.5

        if distance is not None:
            try:
                distance_value = float(distance)
                detail = f'distance_remaining={distance_value:.3f} m'
                progress = max(0.2, min(0.95, 1.0 / (1.0 + distance_value)))
            except (TypeError, ValueError):
                pass

        self._publish_feedback(
            goal_handle,
            phase='navigating',
            progress=progress,
            detail=detail,
        )

    def _publish_feedback(
        self,
        goal_handle,
        *,
        phase: str,
        progress: float,
        detail: str,
    ) -> None:
        feedback = RunTask.Feedback()
        feedback.phase = phase
        feedback.progress = float(max(0.0, min(1.0, progress)))
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _cancel_nav_goal(self, nav_goal_handle) -> None:
        try:
            cancel_future = nav_goal_handle.cancel_goal_async()
            self._wait_plain_future(cancel_future, timeout_sec=2.0)
        except Exception as exc:
            self.get_logger().warning(f'Failed to cancel Nav2 goal: {exc}')

    def _wait_plain_future(self, future, *, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)

    def _finish_canceled_or_aborted(
        self,
        goal_handle,
        result: RunTask.Result,
        message: str,
    ) -> RunTask.Result:
        if goal_handle.is_cancel_requested:
            return self._finish_canceled(goal_handle, result)

        return self._finish_aborted(goal_handle, result, message)

    def _finish_canceled(
        self,
        goal_handle,
        result: RunTask.Result,
    ) -> RunTask.Result:
        goal_handle.canceled()
        result.success = False
        result.message = 'Navigation canceled'
        return result

    def _finish_aborted(
        self,
        goal_handle,
        result: RunTask.Result,
        message: str,
    ) -> RunTask.Result:
        goal_handle.abort()
        result.success = False
        result.message = message
        return result

    def _status_name(self, status: int) -> str:
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
    """Run the VicPinky navigation adapter node."""
    rclpy.init(args=args)
    node = VicPinkyNavAdapter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
