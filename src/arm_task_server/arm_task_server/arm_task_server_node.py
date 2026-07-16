"""Run semantic arm tasks by sending controller trajectories."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import json
import math
import os
import time
from typing import Any, Mapping, Sequence

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from vicpinky_interfaces.action import ExecuteArmGoal, RunTask
import yaml


DEFAULT_CONTROLLER_WAIT_TIMEOUT_SEC = 5.0
FUTURE_POLL_PERIOD_SEC = 0.02


@dataclass(frozen=True)
class ControllerConfig:
    """Configuration for one direct-arm or Board3 trajectory target."""

    name: str
    action_name: str
    joint_names: tuple[str, ...]
    default_duration_sec: float
    interface: str = 'follow_joint_trajectory'


def parse_extra_json(raw: str) -> dict[str, Any]:
    """Parse a RunTask extra_json payload into a dictionary."""
    if not raw:
        return {}

    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        return {}

    if isinstance(parsed, dict):
        return parsed

    return {}


def task_key_for_goal(
    server_key: str,
    server_config: Mapping[str, Any],
    extra: Mapping[str, Any],
) -> str:
    """Return the concrete task profile key for one incoming goal."""
    default_task = str(server_config.get('task', server_key))
    dispatch = server_config.get('dispatch_by_button_role', {})

    if not isinstance(dispatch, Mapping):
        return default_task

    button_role = str(extra.get('button_role', ''))
    return str(dispatch.get(button_role, default_task))


class ArmTaskServer(Node):
    """Expose pick, place, and button-press RunTask action servers."""

    def __init__(self) -> None:
        super().__init__('arm_task_server')

        self._callback_group = ReentrantCallbackGroup()
        self._action_servers: list[ActionServer] = []

        default_config_path = os.path.join(
            get_package_share_directory('arm_task_server'),
            'config',
            'arm_tasks.yaml',
        )

        self.declare_parameter('config_file', default_config_path)
        self.declare_parameter(
            'controller_wait_timeout_sec',
            DEFAULT_CONTROLLER_WAIT_TIMEOUT_SEC,
        )
        config_file = (
            self.get_parameter('config_file')
            .get_parameter_value()
            .string_value
        )
        self._config = self._load_config(config_file)
        self._controllers = self._load_controller_configs(self._config)
        self._controller_clients = {
            name: ActionClient(
                self,
                (
                    ExecuteArmGoal
                    if controller.interface == 'direct_arm_v3'
                    else FollowJointTrajectory
                ),
                controller.action_name,
                callback_group=self._callback_group,
            )
            for name, controller in self._controllers.items()
        }

        self._create_action_servers()

        server_names = [
            str(config.get('action_name'))
            for config in self._config['servers'].values()
        ]
        self.get_logger().info(
            f'Arm task servers ready: {server_names}'
        )

    def _load_config(self, path: str) -> dict[str, Any]:
        if not os.path.exists(path):
            raise FileNotFoundError(f'Arm task config not found: {path}')

        with open(path, 'r') as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise ValueError('arm_tasks.yaml must contain a mapping')

        for key in ('controllers', 'servers', 'poses', 'tasks'):
            if key not in data:
                raise ValueError(f'arm_tasks.yaml missing key: {key}')

        self.get_logger().info(f'Loaded arm task config: {path}')
        return data

    @staticmethod
    def _load_controller_configs(
        config: Mapping[str, Any],
    ) -> dict[str, ControllerConfig]:
        controllers: dict[str, ControllerConfig] = {}

        for name, raw_config in config['controllers'].items():
            joint_names = tuple(str(value) for value in raw_config['joints'])
            if not joint_names:
                raise ValueError(f'Controller {name} has no joints')

            controllers[str(name)] = ControllerConfig(
                name=str(name),
                action_name=str(raw_config['action_name']),
                joint_names=joint_names,
                default_duration_sec=float(
                    raw_config.get('default_duration_sec', 1.0)
                ),
                interface=str(
                    raw_config.get('interface', 'follow_joint_trajectory')
                ),
            )

        return controllers

    def _create_action_servers(self) -> None:
        for server_key, server_config in self._config['servers'].items():
            action_name = str(server_config['action_name'])
            action_server = ActionServer(
                self,
                RunTask,
                action_name,
                execute_callback=partial(
                    self.execute_callback,
                    server_key=str(server_key),
                ),
                goal_callback=partial(
                    self.goal_callback,
                    server_key=str(server_key),
                ),
                cancel_callback=partial(
                    self.cancel_callback,
                    server_key=str(server_key),
                ),
                callback_group=self._callback_group,
            )
            self._action_servers.append(action_server)

    def goal_callback(self, goal_request, server_key: str):
        """Accept a goal only when its task profile exists."""
        server_config = self._config['servers'][server_key]
        extra = parse_extra_json(goal_request.extra_json)
        task_key = task_key_for_goal(server_key, server_config, extra)

        if task_key not in self._config['tasks']:
            self.get_logger().error(
                f'Rejecting {server_key}: task profile not found: '
                f'{task_key}'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted goal on {server_config["action_name"]}: '
            f'task_id={goal_request.task_id}, '
            f'target={goal_request.target_name}, '
            f'profile={task_key}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle, server_key: str):
        """Accept cancellation for a running semantic arm task."""
        self.get_logger().warning(
            f'Cancel requested on arm task server: {server_key}'
        )
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle, server_key: str):
        """Execute one semantic arm task as configured trajectory steps."""
        goal = goal_handle.request
        server_config = self._config['servers'][server_key]
        extra = parse_extra_json(goal.extra_json)
        task_key = task_key_for_goal(server_key, server_config, extra)
        task_config = self._config['tasks'][task_key]
        steps = list(task_config.get('steps', []))

        self.get_logger().info(
            f'Start arm task: profile={task_key}, '
            f'target={goal.target_name}, floor={goal.target_floor}'
        )

        if not steps:
            goal_handle.abort()
            return self._result(False, f'Task has no steps: {task_key}')

        total_steps = len(steps)
        for index, step in enumerate(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(False, f'Canceled: {task_key}')

            phase = str(step.get('phase', f'step_{index + 1}'))
            self._publish_feedback(
                goal_handle,
                phase=phase,
                progress=float(index) / float(total_steps),
                detail=f'Start {task_key}.{phase}',
            )

            try:
                success, message = self._execute_step(
                    goal_handle,
                    task_key,
                    step,
                )
            except Exception as exc:
                success = False
                message = f'Failed {task_key}.{phase}: {exc}'
                self.get_logger().error(message)

            if not success:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(False, f'Canceled: {task_key}')

                goal_handle.abort()
                return self._result(False, message)

            self._publish_feedback(
                goal_handle,
                phase=phase,
                progress=float(index + 1) / float(total_steps),
                detail=f'Finished {task_key}.{phase}',
            )

        goal_handle.succeed()
        return self._result(True, f'Arm task succeeded: {task_key}')

    def _execute_step(
        self,
        goal_handle,
        task_key: str,
        step: Mapping[str, Any],
    ) -> tuple[bool, str]:
        wait_sec = step.get('wait_sec')
        if wait_sec is not None:
            return self._wait_step(goal_handle, task_key, float(wait_sec))

        controller_name = str(step['controller'])
        pose_name = str(step['pose'])

        if controller_name not in self._controllers:
            return False, f'Unknown controller: {controller_name}'

        pose_config = self._pose_config(controller_name, pose_name)
        controller = self._controllers[controller_name]
        duration_sec = float(
            step.get(
                'duration_sec',
                pose_config.get(
                    'duration_sec',
                    controller.default_duration_sec,
                ),
            )
        )

        positions = self._pose_positions_rad(controller, pose_config)
        target_load = step.get(
            'target_load_raw',
            pose_config.get('target_load_raw'),
        )

        return self._send_controller_goal(
            goal_handle,
            controller,
            positions,
            duration_sec,
            target_load,
        )

    @staticmethod
    def _result(success: bool, message: str) -> RunTask.Result:
        result = RunTask.Result()
        result.success = bool(success)
        result.message = str(message)
        return result

    @staticmethod
    def _publish_feedback(
        goal_handle,
        *,
        phase: str,
        progress: float,
        detail: str,
    ) -> None:
        feedback = RunTask.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _wait_step(
        self,
        goal_handle,
        task_key: str,
        wait_sec: float,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.0, wait_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, f'Canceled while waiting in {task_key}'
            time.sleep(FUTURE_POLL_PERIOD_SEC)

        return True, 'wait complete'

    def _pose_config(
        self,
        controller_name: str,
        pose_name: str,
    ) -> Mapping[str, Any]:
        poses = self._config['poses'].get(controller_name, {})
        if pose_name not in poses:
            raise ValueError(
                f'Pose {controller_name}.{pose_name} not found'
            )

        return poses[pose_name]

    @staticmethod
    def _pose_positions_rad(
        controller: ControllerConfig,
        pose_config: Mapping[str, Any],
    ) -> tuple[float, ...]:
        degrees = pose_config['degrees']
        if len(degrees) != len(controller.joint_names):
            raise ValueError(
                f'Pose for {controller.name} must contain '
                f'{len(controller.joint_names)} values'
            )

        return tuple(math.radians(float(value)) for value in degrees)

    def _send_controller_goal(
        self,
        goal_handle,
        controller: ControllerConfig,
        positions_rad: Sequence[float],
        duration_sec: float,
        target_load_raw,
    ) -> tuple[bool, str]:
        client = self._controller_clients[controller.name]
        wait_timeout = self._controller_wait_timeout_sec()
        deadline = time.monotonic() + wait_timeout

        while not client.wait_for_server(timeout_sec=0.25):
            if goal_handle.is_cancel_requested:
                return False, f'Canceled waiting for {controller.action_name}'
            if time.monotonic() >= deadline:
                return False, (
                    f'Controller action server not available: '
                    f'{controller.action_name}'
                )

        controller_goal = self._make_controller_goal(
            controller,
            positions_rad,
            duration_sec,
            target_load_raw,
        )
        send_future = client.send_goal_async(controller_goal)
        ok, message = self._wait_for_future(
            send_future,
            goal_handle,
            wait_timeout,
        )
        if not ok:
            return False, message

        controller_goal_handle = send_future.result()
        if (
            controller_goal_handle is None
            or not controller_goal_handle.accepted
        ):
            return False, f'Goal rejected by {controller.action_name}'

        result_future = controller_goal_handle.get_result_async()
        ok, message = self._wait_for_future(
            result_future,
            goal_handle,
            None,
        )
        if not ok:
            self._cancel_controller_goal(controller_goal_handle)
            return False, message

        result_response = result_future.result()
        if result_response is None:
            return False, f'No result from {controller.action_name}'

        result = result_response.result
        if controller.interface == 'direct_arm_v3':
            if not result.success:
                return False, (
                    f'{controller.action_name} failed: {result.message}'
                )
        elif result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, (
                f'{controller.action_name} failed: '
                f'error_code={result.error_code}, '
                f'error_string={result.error_string}'
            )

        return True, f'{controller.name} trajectory complete'

    @staticmethod
    def _make_controller_goal(
        controller: ControllerConfig,
        positions_rad: Sequence[float],
        duration_sec: float,
        target_load_raw,
    ):
        if controller.interface == 'direct_arm_v3':
            if target_load_raw is not None:
                raise ValueError('Direct arm goal does not accept target load')
            duration_ms = round(float(duration_sec) * 1000.0)
            if not 1 <= duration_ms <= 0xFFFF:
                raise ValueError('Arm duration must be 1..65535 ms')
            goal = ExecuteArmGoal.Goal()
            goal.joint_names = list(controller.joint_names)
            goal.positions = [float(value) for value in positions_rad]
            goal.duration_ms = int(duration_ms)
            return goal

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(controller.joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions_rad]
        if target_load_raw is not None:
            point.effort = [
                float(target_load_raw)
                for _ in positions_rad
            ]

        whole_seconds = int(duration_sec)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (duration_sec - whole_seconds) * 1_000_000_000
        )
        goal.trajectory.points = [point]
        return goal

    def _wait_for_future(
        self,
        future,
        goal_handle,
        timeout_sec: float | None,
    ) -> tuple[bool, str]:
        deadline = (
            None
            if timeout_sec is None
            else time.monotonic() + max(0.0, timeout_sec)
        )

        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                return False, 'Canceled while waiting for controller result'
            if deadline is not None and time.monotonic() >= deadline:
                return False, 'Timeout waiting for controller result'
            time.sleep(FUTURE_POLL_PERIOD_SEC)

        if not future.done():
            return False, 'Controller future did not complete'

        return True, 'future complete'

    def _cancel_controller_goal(self, controller_goal_handle) -> None:
        try:
            cancel_future = controller_goal_handle.cancel_goal_async()
            self._wait_for_future_without_goal(cancel_future, 1.0)
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to cancel controller goal: {exc}'
            )

    @staticmethod
    def _wait_for_future_without_goal(future, timeout_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(FUTURE_POLL_PERIOD_SEC)

    def _controller_wait_timeout_sec(self) -> float:
        return float(
            self.get_parameter('controller_wait_timeout_sec').value
        )


def main(args=None) -> None:
    """Run the arm task server node."""
    rclpy.init(args=args)

    node = ArmTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
