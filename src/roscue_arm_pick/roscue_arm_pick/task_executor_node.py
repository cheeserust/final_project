#!/usr/bin/env python3
"""Execute calibrated ArUco-guided arm tasks through MoveIt and CAN actions."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import math
import os
import threading
import time
from typing import Any, Mapping, Sequence

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from roscue_arm_pick.config_validation import (
    load_urdf_limits,
    load_yaml,
    validate_configuration,
)
from roscue_arm_pick.task_dispatch import (
    resolve_alias,
    resolve_run_task,
    TaskDispatchError,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vicpinky_interfaces.action import ExecuteArmGoal, RunTask
from vicpinky_interfaces.msg import DetectedMarker


ACTION_ENDPOINTS = {
    'execute': '/arm/execute',
    'pick': '/arm/pick',
    'place': '/arm/place',
    'press_button': '/arm/press_button',
    'homing': '/arm/homing',
}
FUTURE_POLL_SEC = 0.02


class TaskFailure(RuntimeError):
    """Raised when a task cannot complete safely."""


class TaskCanceled(TaskFailure):
    """Raised when the parent RunTask goal is canceled."""


@dataclass(frozen=True)
class MarkerObservation:
    """One atomic marker ID/pose observation and its local receive time."""

    message: DetectedMarker
    received_monotonic: float


class TaskExecutorNode(Node):
    """Expose central RunTask actions and a plan-only manual task topic."""

    def __init__(self) -> None:
        super().__init__('task_executor_node')
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.RLock()
        self._marker_lock = threading.Lock()
        self._execution_reserved = False
        self._cancel_event = threading.Event()
        self._active_goal_handle = None
        self._active_downstream_goal = None
        self._homing_active = False
        self._marker_observations: dict[int, MarkerObservation] = {}
        self._plan_only_arm_positions: list[float] | None = None
        self.last_object_task: dict[str, Any] | None = None

        self._declare_parameters()
        self._load_configuration()
        self._configure_runtime()
        self._validate_runtime_configuration()

        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/planned_arm_trajectory',
            10,
        )
        self.gripper_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/planned_gripper_trajectory',
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            '/arm_task_status',
            10,
        )

        self.arm_client = ActionClient(
            self,
            ExecuteArmGoal,
            '/arm_controller/execute_joint_goal',
            callback_group=self._callback_group,
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self._callback_group,
        )
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            self.move_action_name,
            callback_group=self._callback_group,
        )
        self.homing_client = self.create_client(
            Trigger,
            self.homing_service_name,
            callback_group=self._callback_group,
        )
        self.enable_client = self.create_client(
            Trigger,
            self.enable_service_name,
            callback_group=self._callback_group,
        )
        self.estop_client = self.create_client(
            Trigger,
            self.estop_service_name,
            callback_group=self._callback_group,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            DetectedMarker,
            self.detection_topic,
            self.marker_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            self.arm_task_topic,
            self.manual_task_callback,
            10,
            callback_group=self._callback_group,
        )
        if (
            self.legacy_task_topic
            and self.legacy_task_topic != self.arm_task_topic
        ):
            self.create_subscription(
                String,
                self.legacy_task_topic,
                self.manual_task_callback,
                10,
                callback_group=self._callback_group,
            )

        self._action_servers = []
        for endpoint, default_name in ACTION_ENDPOINTS.items():
            action_name = str(
                self.get_parameter(f'{endpoint}_action_name').value
                or default_name
            )
            self._action_servers.append(
                ActionServer(
                    self,
                    RunTask,
                    action_name,
                    execute_callback=partial(
                        self.execute_action_callback,
                        endpoint=endpoint,
                    ),
                    goal_callback=partial(
                        self.goal_callback,
                        endpoint=endpoint,
                    ),
                    cancel_callback=self.cancel_callback,
                    callback_group=self._callback_group,
                )
            )

        self.get_logger().info(
            f'Task executor ready: execution_mode={self.execution_mode}, '
            f'manual_topic={self.arm_task_topic}'
        )
        if self.execution_mode == 'plan_only':
            self.get_logger().warning(
                'Plan-only mode: /arm/* RunTask goals are rejected; '
                'manual /arm_task commands never reach hardware'
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter('aruco_targets_yaml', '')
        self.declare_parameter('fixed_poses_yaml', '')
        self.declare_parameter('task_sequence_yaml', '')
        self.declare_parameter('gripper_profiles_yaml', '')
        self.declare_parameter('task_waypoints_yaml', '')
        self.declare_parameter('bridge_config_yaml', '')
        self.declare_parameter('moveit_limits_yaml', '')
        self.declare_parameter('arm_task_topic', '/arm_task')
        self.declare_parameter('legacy_task_topic', '/roscue_arm_task')
        self.declare_parameter('detection_topic', '/detected_marker')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('grasp_planning_group', 'arm_grasp')
        self.declare_parameter('button_planning_group', 'arm_button')
        self.declare_parameter('pose_link', 'gripper_base_link')
        self.declare_parameter('grasp_pose_link', 'grasp_tcp_link')
        self.declare_parameter('button_pose_link', 'button_contact_link')
        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter('homing_service_name', '/arm_board/home_all')
        self.declare_parameter('enable_service_name', '/arm_board/enable')
        self.declare_parameter('estop_service_name', '/arm_board/estop')
        self.declare_parameter('auto_enable_before_homing', True)
        self.declare_parameter('execution_mode', 'plan_only')
        self.declare_parameter('downstream_result_timeout_sec', 120.0)
        self.declare_parameter('use_orientation_constraint', False)
        self.declare_parameter('position_tolerance_m', 0.02)
        self.declare_parameter('orientation_tolerance_rad', 0.5)
        self.declare_parameter('marker_wait_sec', 2.0)
        self.declare_parameter('marker_ttl_sec', 0.5)
        for endpoint, action_name in ACTION_ENDPOINTS.items():
            self.declare_parameter(f'{endpoint}_action_name', action_name)

    def _config_path(self, parameter: str, package: str, filename: str) -> str:
        configured = str(self.get_parameter(parameter).value)
        if configured:
            return configured
        return os.path.join(
            get_package_share_directory(package),
            'config',
            filename,
        )

    def _load_configuration(self) -> None:
        self.aruco_cfg = load_yaml(self._config_path(
            'aruco_targets_yaml',
            'roscue_arm_pick',
            'aruco_targets.yaml',
        ))
        self.fixed_cfg = load_yaml(self._config_path(
            'fixed_poses_yaml',
            'roscue_arm_pick',
            'fixed_poses.yaml',
        ))
        self.task_cfg = load_yaml(self._config_path(
            'task_sequence_yaml',
            'roscue_arm_pick',
            'task_sequence.yaml',
        ))
        self.gripper_cfg = load_yaml(self._config_path(
            'gripper_profiles_yaml',
            'roscue_arm_pick',
            'gripper_profiles.yaml',
        ))
        self.waypoint_cfg = load_yaml(self._config_path(
            'task_waypoints_yaml',
            'roscue_arm_pick',
            'task_waypoints.yaml',
        ))
        self.bridge_config_path = self._config_path(
            'bridge_config_yaml',
            'arm_can_bridge',
            'arm_can_bridge.yaml',
        )
        self.moveit_limits_path = self._config_path(
            'moveit_limits_yaml',
            'roscue_arm_moveit_config',
            'joint_limits.yaml',
        )

    def _configure_runtime(self) -> None:
        self.arm_joint_names = list(self.fixed_cfg['joint_order']['arm'])
        self.gripper_joint_names = list(
            self.fixed_cfg['joint_order']['gripper']
        )
        self.tasks: Mapping[str, Any] = self.task_cfg['tasks']
        self.mission_allowed_tasks = {
            str(value)
            for value in self.task_cfg.get('mission_allowed_tasks', [])
        }
        self.arm_task_topic = str(self.get_parameter('arm_task_topic').value)
        self.legacy_task_topic = str(
            self.get_parameter('legacy_task_topic').value
        )
        self.detection_topic = str(
            self.get_parameter('detection_topic').value
        )
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.planning_group = str(
            self.get_parameter('planning_group').value
        )
        self.grasp_planning_group = str(
            self.get_parameter('grasp_planning_group').value
        )
        self.button_planning_group = str(
            self.get_parameter('button_planning_group').value
        )
        self.pose_link = str(self.get_parameter('pose_link').value)
        self.grasp_pose_link = str(
            self.get_parameter('grasp_pose_link').value
        )
        self.button_pose_link = str(
            self.get_parameter('button_pose_link').value
        )
        self.move_action_name = str(
            self.get_parameter('move_action_name').value
        )
        self.homing_service_name = str(
            self.get_parameter('homing_service_name').value
        )
        self.enable_service_name = str(
            self.get_parameter('enable_service_name').value
        )
        self.estop_service_name = str(
            self.get_parameter('estop_service_name').value
        )
        self.auto_enable_before_homing = bool(
            self.get_parameter('auto_enable_before_homing').value
        )
        self.execution_mode = str(
            self.get_parameter('execution_mode').value
        ).strip().lower()
        if self.execution_mode not in {'plan_only', 'hardware'}:
            raise ValueError('execution_mode must be plan_only or hardware')
        self.execute_plans = self.execution_mode == 'hardware'
        self.downstream_result_timeout_sec = float(
            self.get_parameter('downstream_result_timeout_sec').value
        )

        planning = self.waypoint_cfg.get('planning', {})
        self.marker_wait_sec = float(planning.get(
            'marker_wait_sec',
            self.get_parameter('marker_wait_sec').value,
        ))
        self.marker_ttl_sec = float(planning.get(
            'marker_ttl_sec',
            self.get_parameter('marker_ttl_sec').value,
        ))
        self.position_tolerance_m = float(planning.get(
            'position_tolerance_m',
            self.get_parameter('position_tolerance_m').value,
        ))
        self.orientation_tolerance_rad = float(planning.get(
            'orientation_tolerance_rad',
            self.get_parameter('orientation_tolerance_rad').value,
        ))
        self.use_orientation_constraint = bool(planning.get(
            'use_orientation_constraint',
            self.get_parameter('use_orientation_constraint').value,
        ))
        self.velocity_scaling = float(planning.get('velocity_scaling', 0.1))
        self.acceleration_scaling = float(
            planning.get('acceleration_scaling', 0.1)
        )
        self.allowed_planning_time_sec = float(
            planning.get('allowed_planning_time_sec', 10.0)
        )
        self.planning_attempts = int(planning.get('planning_attempts', 30))
        self.move_action_wait_sec = float(
            planning.get('move_action_wait_sec', 30.0)
        )
        self.min_goal_z_m = float(planning.get('min_goal_z_m', 0.02))
        self.max_goal_z_m = float(planning.get('max_goal_z_m', 1.20))

    def _validate_runtime_configuration(self) -> None:
        description_share = get_package_share_directory(
            'roscue_arm_description'
        )
        urdf_limits = load_urdf_limits([
            os.path.join(
                description_share,
                'urdf',
                'roscue_arm.urdf.xacro',
            ),
            os.path.join(
                description_share,
                'urdf',
                'assemblies',
                'roscue_arm.urdf.xacro',
            ),
        ])
        report = validate_configuration(
            fixed_config=self.fixed_cfg,
            gripper_config=self.gripper_cfg,
            bridge_config=load_yaml(self.bridge_config_path),
            moveit_limits=load_yaml(self.moveit_limits_path),
            urdf_limits=urdf_limits,
        )
        for warning in report.warnings:
            self.get_logger().warning(f'Configuration warning: {warning}')
        for error in report.errors:
            self.get_logger().warning(f'Configuration mismatch: {error}')

        calibration_complete = bool(
            self.fixed_cfg.get('calibration', {}).get('complete', False)
        )
        if self.execution_mode == 'hardware' and (
            not calibration_complete or not report.ok
        ):
            reasons = list(report.errors)
            if not calibration_complete:
                reasons.append('calibration.complete is false')
            raise RuntimeError(
                'Hardware execution blocked by configuration: '
                + '; '.join(reasons)
            )

    def marker_callback(self, msg: DetectedMarker) -> None:
        observation = MarkerObservation(msg, time.monotonic())
        with self._marker_lock:
            self._marker_observations[int(msg.marker_id)] = observation

    def goal_callback(self, goal_request: Any, endpoint: str):
        """Validate and reserve one mission-triggered task."""
        if self.execution_mode != 'hardware':
            self.get_logger().warning(
                f'Reject /arm/{endpoint}: execution_mode=plan_only'
            )
            return GoalResponse.REJECT
        try:
            resolve_run_task(
                endpoint,
                goal_request,
                self.tasks,
                self.mission_allowed_tasks,
            )
        except TaskDispatchError as exc:
            self.get_logger().warning(f'Reject /arm/{endpoint}: {exc}')
            return GoalResponse.REJECT

        with self._state_lock:
            if self._execution_reserved:
                self.get_logger().warning(
                    f'Reject /arm/{endpoint}: another arm task is active'
                )
                return GoalResponse.REJECT
            self._cancel_event.clear()
            self._execution_reserved = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: Any):
        """Forward mission cancellation to the active downstream action."""
        del goal_handle
        self._cancel_event.set()
        with self._state_lock:
            downstream = self._active_downstream_goal
            homing_active = self._homing_active
        if downstream is not None:
            downstream.cancel_goal_async()
        if homing_active:
            if self.estop_client.service_is_ready():
                self.estop_client.call_async(Trigger.Request())
                self.get_logger().error(
                    'Homing cancel requested arm-board emergency stop'
                )
            else:
                self.get_logger().error(
                    'Homing canceled but /arm_board/estop is unavailable'
                )
        self.get_logger().warning('Arm task cancellation requested')
        return CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle: Any, endpoint: str):
        """Run one concrete task and return all failures to mission_manager."""
        with self._state_lock:
            self._active_goal_handle = goal_handle

        try:
            task_name = resolve_run_task(
                endpoint,
                goal_handle.request,
                self.tasks,
                self.mission_allowed_tasks,
            )
            if task_name == '__homing__':
                self._run_homing()
            else:
                self.run_task(task_name)
            self._check_canceled()
            goal_handle.succeed()
            return self._result(True, f'Completed arm task: {task_name}')
        except TaskCanceled as exc:
            goal_handle.canceled()
            self.publish_status(f'CANCELED: {exc}')
            return self._result(False, str(exc))
        except Exception as exc:
            goal_handle.abort()
            self.publish_status(f'FAILED task: {exc}')
            self.get_logger().error(f'Arm task failed: {exc}')
            return self._result(False, str(exc))
        finally:
            with self._state_lock:
                self._active_goal_handle = None
                self._active_downstream_goal = None
                self._execution_reserved = False
            self._cancel_event.clear()

    def manual_task_callback(self, msg: String) -> None:
        """Execute a concrete debug task without reporting mission success."""
        task_name = msg.data.strip()
        with self._state_lock:
            if self._execution_reserved:
                self.publish_status(
                    f'FAILED busy: cannot start manual task {task_name}'
                )
                return
            self._execution_reserved = True
            self._active_goal_handle = None
        self._cancel_event.clear()
        self._plan_only_arm_positions = None
        previous_execute_plans = self.execute_plans
        self.execute_plans = False

        try:
            self.run_task(task_name)
            self.publish_status(
                f'Completed manual plan-only task: {task_name}'
            )
        except Exception as exc:
            self.publish_status(f'FAILED manual task: {exc}')
            self.get_logger().error(f'Manual arm task failed: {exc}')
        finally:
            self.execute_plans = previous_execute_plans
            with self._state_lock:
                self._execution_reserved = False
                self._active_downstream_goal = None

    @staticmethod
    def _result(success: bool, message: str) -> RunTask.Result:
        result = RunTask.Result()
        result.success = bool(success)
        result.message = str(message)
        return result

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_phase(
        self,
        phase: str,
        detail: str,
        progress: float,
    ) -> None:
        self.publish_status(detail)
        with self._state_lock:
            goal_handle = self._active_goal_handle
        if goal_handle is None:
            return
        feedback = RunTask.Feedback()
        feedback.phase = phase
        feedback.progress = float(max(0.0, min(1.0, progress)))
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def _check_canceled(self) -> None:
        if self._cancel_event.is_set():
            raise TaskCanceled('Arm task canceled by mission manager')

    def _wait_for_future(
        self,
        future: Any,
        timeout_sec: float,
        label: str,
    ) -> Any:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if self._cancel_event.is_set():
                future.add_done_callback(self._cancel_late_goal)
                raise TaskCanceled(f'Canceled while waiting for {label}')
            if time.monotonic() >= deadline:
                future.add_done_callback(self._cancel_late_goal)
                raise TaskFailure(f'Timeout waiting for {label}')
            time.sleep(FUTURE_POLL_SEC)
        if not future.done():
            future.add_done_callback(self._cancel_late_goal)
            raise TaskFailure(f'ROS shutdown while waiting for {label}')
        try:
            return future.result()
        except Exception as exc:
            raise TaskFailure(f'{label} failed: {exc}') from exc

    def _cancel_late_goal(self, future: Any) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
        except Exception:
            return

    def _wait_for_action_server(
        self,
        client: ActionClient,
        timeout_sec: float,
        label: str,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while not client.wait_for_server(timeout_sec=0.2):
            self._check_canceled()
            if time.monotonic() >= deadline:
                raise TaskFailure(f'Action server unavailable: {label}')

    def _send_action_goal(
        self,
        client: ActionClient,
        goal: Any,
        label: str,
        wait_server_sec: float = 5.0,
    ) -> Any:
        self._wait_for_action_server(client, wait_server_sec, label)
        goal_handle = self._wait_for_future(
            client.send_goal_async(goal),
            wait_server_sec,
            f'{label} goal acceptance',
        )
        if goal_handle is None or not goal_handle.accepted:
            raise TaskFailure(f'{label} goal rejected')
        with self._state_lock:
            self._active_downstream_goal = goal_handle
        try:
            try:
                response = self._wait_for_future(
                    goal_handle.get_result_async(),
                    self.downstream_result_timeout_sec,
                    f'{label} result',
                )
            except Exception:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                raise
        finally:
            with self._state_lock:
                if self._active_downstream_goal is goal_handle:
                    self._active_downstream_goal = None
        return response

    def _run_homing(self) -> None:
        with self._state_lock:
            self._homing_active = True
        try:
            if self.auto_enable_before_homing:
                self.publish_phase('enable', 'Enabling all arm boards', 0.05)
                self._call_trigger(
                    self.enable_client,
                    self.enable_service_name,
                    'arm board enable service',
                )
            self.publish_phase('homing', 'Calling arm board homing', 0.1)
            self._call_trigger(
                self.homing_client,
                self.homing_service_name,
                'arm board homing service',
            )
            self.publish_phase('homing', 'Arm homing confirmed', 1.0)
        finally:
            with self._state_lock:
                self._homing_active = False

    def _call_trigger(
        self,
        client: Any,
        service_name: str,
        label: str,
    ) -> None:
        deadline = time.monotonic() + 5.0
        while not client.wait_for_service(timeout_sec=0.2):
            self._check_canceled()
            if time.monotonic() >= deadline:
                raise TaskFailure(
                    f'Service unavailable: {service_name}'
                )
        response = self._wait_for_future(
            client.call_async(Trigger.Request()),
            self.downstream_result_timeout_sec,
            label,
        )
        if response is None or not response.success:
            message = response.message if response is not None else 'no response'
            raise TaskFailure(f'{label} failed: {message}')

    def run_task(self, task_name: str) -> None:
        """Run one concrete task, returning home only after normal success."""
        concrete = resolve_alias(self.tasks, task_name)
        task = self.tasks[concrete]
        task_type = str(task.get('type', ''))
        self.publish_phase(
            'start',
            f'Executing task: {concrete}',
            0.0,
        )

        start_pose = str(task.get('start_pose', 'home')).strip()
        return_pose = str(task.get('return_pose', 'home')).strip()
        if start_pose and start_pose.lower() != 'none':
            self.go_named_arm_pose(start_pose)

        if task_type in {'home', 'named_arm_pose'}:
            self.go_named_arm_pose(str(task.get('pose_name', 'home')))
        elif task_type == 'named_arm_gripper_pose':
            self.go_named_arm_pose(str(task['pose_name']))
            self.send_named_gripper_pose(str(task['gripper_pose_name']))
        elif task_type == 'press_button':
            self.execute_press_button(task)
        elif task_type == 'pick_to_fixed_place':
            self.execute_pick_to_fixed_place(task)
        elif task_type == 'pick_to_named_place':
            self.execute_pick_to_named_place(task)
        elif task_type == 'transfer_named_pose':
            self.execute_transfer_named_pose(task)
        elif task_type == 'pick_previous_to_fixed_place':
            self.execute_pick_previous_to_fixed_place(task)
        elif task_type == 'place_fixed':
            self.execute_place_fixed(task)
        else:
            raise TaskFailure(f'Unsupported task type: {task_type}')

        self._check_canceled()
        if return_pose and return_pose.lower() != 'none':
            self.publish_phase(
                'return_pose',
                f'Returning arm to {return_pose}',
                0.95,
            )
            self.go_named_arm_pose(return_pose)
        self.publish_phase('complete', f'Completed task: {concrete}', 1.0)

    def go_named_arm_pose(self, name: str) -> None:
        poses = self.fixed_cfg.get('arm_named_poses', {})
        if name not in poses:
            raise TaskFailure(f'Unknown named arm pose: {name}')
        pose = poses[name]
        trajectory = self.make_single_point_trajectory(
            self.arm_joint_names,
            pose['positions_rad'],
            float(pose.get('duration_sec', 3.0)),
        )
        self.arm_trajectory_pub.publish(trajectory)
        self.publish_status(f'Named arm trajectory: {name}')
        if not self.execute_plans:
            self._plan_only_arm_positions = [
                float(value) for value in pose['positions_rad']
            ]
            return
        self._execute_direct_arm_goal(
            pose['positions_rad'],
            float(pose.get('duration_sec', 3.0)),
            f'arm named pose {name}',
        )

    def go_observe_pose_if_configured(self, task: Mapping[str, Any]) -> None:
        pose_name = str(task.get('observe_pose', '')).strip()
        if pose_name:
            self.publish_phase(
                'observe_pose',
                f'Moving to observe pose: {pose_name}',
                0.15,
            )
            self.go_named_arm_pose(pose_name)

    def send_named_gripper_pose(self, name: str) -> None:
        poses = self.fixed_cfg.get('gripper_named_poses', {})
        if name not in poses:
            raise TaskFailure(f'Unknown named gripper pose: {name}')
        pose = poses[name]
        self.send_gripper_positions(
            pose['positions_rad'],
            duration_sec=float(pose.get('duration_sec', 2.0)),
            effort=float(pose.get('effort', 500)),
            label=name,
        )

    def _clear_marker(self, marker_id: int) -> None:
        with self._marker_lock:
            self._marker_observations.pop(marker_id, None)

    def _marker_age_sec(self, observation: MarkerObservation) -> float:
        receive_age = time.monotonic() - observation.received_monotonic
        stamp = observation.message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return receive_age
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        source_age = (
            self.get_clock().now().nanoseconds - stamp_ns
        ) / 1_000_000_000.0
        return max(receive_age, source_age)

    def check_marker(self, expected_marker_id: int) -> DetectedMarker:
        deadline = time.monotonic() + self.marker_wait_sec
        while time.monotonic() <= deadline:
            self._check_canceled()
            with self._marker_lock:
                observation = self._marker_observations.get(
                    expected_marker_id
                )
            if observation is not None:
                age = self._marker_age_sec(observation)
                if age <= self.marker_ttl_sec:
                    pose = observation.message.pose.position
                    self.publish_phase(
                        'marker',
                        f'Marker OK id={expected_marker_id}, '
                        f'x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}, '
                        f'frame={observation.message.header.frame_id}',
                        0.25,
                    )
                    return observation.message
            time.sleep(0.05)

        with self._marker_lock:
            visible = sorted(self._marker_observations)
        raise TaskFailure(
            f'Target marker {expected_marker_id} not detected with age <= '
            f'{self.marker_ttl_sec:.2f}s within {self.marker_wait_sec:.1f}s; '
            f'visible_ids={visible}'
        )

    def transform_marker_to_base(self, detection: DetectedMarker):
        pose_stamped = PoseStamped()
        pose_stamped.header = detection.header
        pose_stamped.pose = detection.pose
        stamp = detection.header.stamp
        query_time = (
            Time.from_msg(stamp)
            if stamp.sec != 0 or stamp.nanosec != 0
            else Time()
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                detection.header.frame_id,
                query_time,
                timeout=Duration(seconds=1.0),
            )
            pose = do_transform_pose(pose_stamped.pose, transform)
        except Exception as exc:
            raise TaskFailure(f'TF transform failed: {exc}') from exc
        p = pose.position
        self.publish_phase(
            'marker_tf',
            f'Marker in {self.target_frame}: '
            f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}',
            0.3,
        )
        return pose

    def execute_press_button(self, task: Mapping[str, Any]) -> None:
        marker_id = int(task['marker_id'])
        self.go_observe_pose_if_configured(task)
        self._clear_marker(marker_id)
        marker_base = self.transform_marker_to_base(
            self.check_marker(marker_id)
        )
        target_cfg = self.aruco_cfg['aruco_targets'][marker_id]
        button_cfg = self.get_button_waypoint_cfg(
            marker_id,
            target_cfg['name'],
        )
        self.go_named_arm_pose('ready')
        corrected_marker = self.apply_pose_correction(marker_base)
        button_center = self.make_offset_pose(
            corrected_marker,
            button_cfg['center_offset'],
        )
        pre_pose = self.make_offset_pose(
            button_center.pose,
            button_cfg['pre_press_offset'],
        )
        press_pose = self.make_offset_pose(
            button_center.pose,
            button_cfg['press_offset'],
        )
        retreat_pose = self.make_offset_pose(
            button_center.pose,
            button_cfg['retreat_offset'],
        )
        self.plan_and_optionally_execute(
            pre_pose,
            'button pre-press',
            pose_link=self.button_pose_link,
        )
        self.send_button_gripper_pose()
        self.plan_and_optionally_execute(
            press_pose,
            'button press',
            pose_link=self.button_pose_link,
        )
        self._wait_interruptibly(
            float(button_cfg.get('press_hold_sec', 2.0)),
            'button hold',
        )
        self.plan_and_optionally_execute(
            retreat_pose,
            'button retreat',
            pose_link=self.button_pose_link,
        )

    def execute_pick_to_fixed_place(self, task: Mapping[str, Any]) -> None:
        marker_id = int(task['marker_id'])
        self.go_observe_pose_if_configured(task)
        self._clear_marker(marker_id)
        marker_base = self.transform_marker_to_base(
            self.check_marker(marker_id)
        )
        target_cfg = self.aruco_cfg['aruco_targets'][marker_id]
        object_key = str(task.get('object_key', target_cfg['name']))
        object_profile = self.gripper_cfg['objects'][object_key]
        object_waypoints = self.waypoint_cfg['objects'][object_key]
        place_name = str(task['place_name'])

        self.publish_status(
            f'Pick target: {target_cfg["name"]}; place={place_name}'
        )
        self.go_named_arm_pose('ready')
        self.send_named_gripper_pose('open')

        corrected_marker = self.apply_pose_correction(marker_base)
        approach_pose = self.make_offset_pose(
            corrected_marker,
            object_waypoints['approach_offset'],
        )
        grasp_pose = self.make_offset_pose(
            corrected_marker,
            object_waypoints['grasp_offset'],
        )
        lift_pose = self.make_offset_pose(
            grasp_pose.pose,
            object_waypoints['lift_offset'],
        )
        place_approach, place_pose, place_retreat = (
            self.make_fixed_place_sequence(place_name)
        )

        self.plan_and_optionally_execute(
            approach_pose,
            'pick approach',
            pose_link=self.grasp_pose_link,
        )
        self.plan_and_optionally_execute(
            grasp_pose,
            'pick grasp',
            pose_link=self.grasp_pose_link,
        )
        self.send_object_grip(object_key, object_profile)
        self.plan_and_optionally_execute(
            lift_pose,
            'pick lift',
            pose_link=self.grasp_pose_link,
        )
        self.plan_and_optionally_execute(
            place_approach,
            f'place approach {place_name}',
        )
        self.plan_and_optionally_execute(place_pose, f'place {place_name}')
        self.send_named_gripper_pose('open')
        self.plan_and_optionally_execute(
            place_retreat,
            f'place retreat {place_name}',
        )
        self.last_object_task = {
            'type': 'pick_to_fixed_place',
            'marker_id': marker_id,
            'object_key': object_key,
            'place_name': place_name,
            'observe_pose': task.get('observe_pose'),
        }

    def execute_pick_to_named_place(self, task: Mapping[str, Any]) -> None:
        marker_id = int(task['marker_id'])
        self.go_observe_pose_if_configured(task)
        self._clear_marker(marker_id)
        marker_base = self.transform_marker_to_base(
            self.check_marker(marker_id)
        )
        target_cfg = self.aruco_cfg['aruco_targets'][marker_id]
        object_key = str(task.get('object_key', target_cfg['name']))
        object_profile = self.gripper_cfg['objects'][object_key]
        object_waypoints = self.waypoint_cfg['objects'][object_key]

        self.go_named_arm_pose('ready')
        self.send_named_gripper_pose('open')
        corrected_marker = self.apply_pose_correction(marker_base)
        approach_pose = self.make_offset_pose(
            corrected_marker,
            object_waypoints['approach_offset'],
        )
        grasp_pose = self.make_offset_pose(
            corrected_marker,
            object_waypoints['grasp_offset'],
        )
        lift_pose = self.make_offset_pose(
            grasp_pose.pose,
            object_waypoints['lift_offset'],
        )
        self.plan_and_optionally_execute(
            approach_pose,
            'pick approach',
            pose_link=self.grasp_pose_link,
        )
        self.plan_and_optionally_execute(
            grasp_pose,
            'pick grasp',
            pose_link=self.grasp_pose_link,
        )
        self.send_object_grip(object_key, object_profile)
        self.plan_and_optionally_execute(
            lift_pose,
            'pick lift',
            pose_link=self.grasp_pose_link,
        )
        self.go_named_arm_pose(str(task['place_pose_name']))
        self.send_named_gripper_pose('open')

    def execute_transfer_named_pose(self, task: Mapping[str, Any]) -> None:
        object_key = str(task['object_key'])
        object_profile = self.gripper_cfg['objects'][object_key]
        self.send_named_gripper_pose('open')
        self.go_named_arm_pose(str(task['pickup_pose_name']))
        self.send_object_grip(object_key, object_profile)
        self.go_named_arm_pose(str(task['delivery_pose_name']))
        self.send_named_gripper_pose('open')

    def send_object_grip(
        self,
        object_key: str,
        object_profile: Mapping[str, Any],
    ) -> None:
        stages = object_profile.get('close_stages_rad')
        if not stages:
            stages = [object_profile['gripper_close_rad']]
        effort = float(object_profile.get('gripper_effort', 700))
        duration = float(object_profile.get('stage_duration_sec', 1.2))
        for index, positions in enumerate(stages, start=1):
            self.send_gripper_positions(
                positions,
                duration_sec=duration,
                effort=effort,
                label=f'{object_key}_stage_{index}',
            )

    def _wait_interruptibly(self, duration_sec: float, label: str) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while True:
            self._check_canceled()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(FUTURE_POLL_SEC, remaining))
        self.publish_status(f'Completed wait: {label}')

    def execute_pick_previous_to_fixed_place(
        self,
        task: Mapping[str, Any],
    ) -> None:
        if self.last_object_task is not None:
            pick_task = dict(self.last_object_task)
        else:
            pick_task = {
                'type': 'pick_to_fixed_place',
                'marker_id': int(task['fallback_marker_id']),
                'object_key': task['fallback_object_key'],
            }
        pick_task['place_name'] = task['place_name']
        if task.get('observe_pose'):
            pick_task['observe_pose'] = task['observe_pose']
        self.execute_pick_to_fixed_place(pick_task)

    def execute_place_fixed(self, task: Mapping[str, Any]) -> None:
        place_name = str(task['place_name'])
        self.go_named_arm_pose('ready')
        approach, place_pose, retreat = self.make_fixed_place_sequence(
            place_name
        )
        self.plan_and_optionally_execute(
            approach,
            f'place approach {place_name}',
        )
        self.plan_and_optionally_execute(place_pose, f'place {place_name}')
        self.send_named_gripper_pose('open')
        self.plan_and_optionally_execute(
            retreat,
            f'place retreat {place_name}',
        )

    def send_button_gripper_pose(self) -> None:
        button = self.gripper_cfg.get('buttons', {})
        if 'press_pose_rad' not in button:
            self.send_named_gripper_pose('press')
            return
        self.send_gripper_positions(
            button['press_pose_rad'],
            duration_sec=float(button.get('duration_sec', 2.0)),
            effort=float(button.get('gripper_effort', 500)),
            label='button_press',
        )

    def send_gripper_positions(
        self,
        positions: Sequence[float],
        duration_sec: float,
        effort: float,
        label: str,
    ) -> None:
        trajectory = self.make_single_point_trajectory(
            self.gripper_joint_names,
            positions,
            duration_sec,
            effort=effort,
        )
        self.gripper_trajectory_pub.publish(trajectory)
        self.publish_status(f'Gripper trajectory: {label}')
        if not self.execute_plans:
            return
        self._execute_follow_trajectory(
            self.gripper_client,
            trajectory,
            f'gripper {label}',
        )

    def _execute_follow_trajectory(
        self,
        client: ActionClient,
        trajectory: JointTrajectory,
        label: str,
    ) -> None:
        if not trajectory.points:
            raise TaskFailure(f'{label} trajectory has no points')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        response = self._send_action_goal(client, goal, label)
        result = response.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise TaskFailure(
                f'{label} failed: code={result.error_code}, '
                f'message={result.error_string}'
            )

    def _execute_direct_arm_goal(
        self,
        positions: Sequence[float],
        duration_sec: float,
        label: str,
    ) -> None:
        if len(positions) != len(self.arm_joint_names):
            raise TaskFailure(f'{label} position count does not match joints')
        duration_ms = round(float(duration_sec) * 1000.0)
        if not 1 <= duration_ms <= 0xFFFF:
            raise TaskFailure(f'{label} duration must be 1..65535 ms')
        goal = ExecuteArmGoal.Goal()
        goal.joint_names = list(self.arm_joint_names)
        goal.positions = [float(value) for value in positions]
        goal.duration_ms = int(duration_ms)
        response = self._send_action_goal(self.arm_client, goal, label)
        if not response.result.success:
            raise TaskFailure(
                f'{label} failed: {response.result.message}'
            )

    def apply_pose_correction(self, pose: Any):
        correction = self.waypoint_cfg.get('global', {}).get(
            'marker_pose_correction_m'
        )
        if correction is None:
            return pose
        return self.make_offset_pose(pose, correction).pose

    def make_offset_pose(
        self,
        reference_pose: Any,
        offset_cfg: Any,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.orientation = reference_pose.orientation
        frame, xyz = self.parse_offset(offset_cfg)
        if frame == 'marker':
            dx, dy, dz = self.rotate_vector_by_quaternion(
                xyz,
                reference_pose.orientation,
            )
        else:
            dx, dy, dz = [float(value) for value in xyz]
        pose.pose.position.x = reference_pose.position.x + dx
        pose.pose.position.y = reference_pose.position.y + dy
        pose.pose.position.z = reference_pose.position.z + dz
        return pose

    @staticmethod
    def parse_offset(offset_cfg: Any) -> tuple[str, Sequence[float]]:
        if isinstance(offset_cfg, dict):
            return str(offset_cfg.get('frame', 'marker')), offset_cfg['xyz']
        return 'marker', offset_cfg

    def get_button_waypoint_cfg(
        self,
        marker_id: int,
        target_name: str,
    ) -> dict[str, Any]:
        button_root = self.waypoint_cfg['buttons']
        default_cfg = dict(button_root['default'])
        for name, config in button_root.items():
            if name == 'default':
                continue
            if (
                int(config.get('marker_id', -1)) == marker_id
                or name == target_name
            ):
                merged = dict(default_cfg)
                merged.update(config)
                merged.setdefault(
                    'center_offset',
                    {'frame': 'marker', 'xyz': [0.0, 0.0, 0.0]},
                )
                return merged
        default_cfg['center_offset'] = {
            'frame': 'marker',
            'xyz': [0.0, 0.0, 0.0],
        }
        return default_cfg

    def make_fixed_place_pose(self, place_name: str) -> PoseStamped:
        config = self.get_fixed_place_cfg(place_name)
        pose = PoseStamped()
        pose.header.frame_id = str(config.get('frame_id', self.target_frame))
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(config['position'][0])
        pose.pose.position.y = float(config['position'][1])
        pose.pose.position.z = float(config['position'][2])
        pose.pose.orientation.x = float(config['orientation_xyzw'][0])
        pose.pose.orientation.y = float(config['orientation_xyzw'][1])
        pose.pose.orientation.z = float(config['orientation_xyzw'][2])
        pose.pose.orientation.w = float(config['orientation_xyzw'][3])
        return pose

    def make_fixed_place_sequence(
        self,
        place_name: str,
    ) -> tuple[PoseStamped, PoseStamped, PoseStamped]:
        config = self.get_fixed_place_cfg(place_name)
        place_pose = self.make_fixed_place_pose(place_name)
        approach = self.make_offset_pose(
            place_pose.pose,
            {
                'frame': 'base',
                'xyz': config.get(
                    'approach_offset_m',
                    [0.0, 0.0, 0.12],
                ),
            },
        )
        retreat = self.make_offset_pose(
            place_pose.pose,
            {
                'frame': 'base',
                'xyz': config.get(
                    'retreat_offset_m',
                    [0.0, 0.0, 0.14],
                ),
            },
        )
        return approach, place_pose, retreat

    def get_fixed_place_cfg(self, place_name: str) -> Mapping[str, Any]:
        places = self.waypoint_cfg.get('fixed_places', {})
        if place_name in places:
            return places[place_name]
        fixed = self.fixed_cfg.get('fixed_place_poses', {})
        if place_name in fixed:
            return fixed[place_name]
        raise TaskFailure(f'Unknown fixed place: {place_name}')

    def plan_and_optionally_execute(
        self,
        target_pose: PoseStamped,
        label: str,
        pose_link: str | None = None,
    ) -> None:
        trajectory = self.plan_pose(target_pose, label, pose_link=pose_link)
        if not self.execute_plans:
            return
        del trajectory
        raise TaskFailure(
            'MoveIt waypoint execution is disabled by the Board1/2 V3 '
            'contract. Configure a validated direct joint pose for '
            f'{label}; a planned trajectory endpoint is not reused.'
        )

    def plan_pose(
        self,
        target_pose: PoseStamped,
        label: str,
        pose_link: str | None = None,
    ) -> JointTrajectory:
        self.validate_target_pose(target_pose, label)
        active_pose_link = pose_link or self.pose_link
        planning_group = self._planning_group_for_pose_link(active_pose_link)
        self.publish_phase(
            'planning',
            f'Planning {label}: '
            f'x={target_pose.pose.position.x:.3f}, '
            f'y={target_pose.pose.position.y:.3f}, '
            f'z={target_pose.pose.position.z:.3f}, '
            f'group={planning_group}, link={active_pose_link}',
            0.45,
        )
        goal = MoveGroup.Goal()
        goal.request = self.make_motion_plan_request(
            target_pose,
            pose_link=active_pose_link,
            planning_group=planning_group,
        )
        goal.planning_options = self.make_planning_options()
        response = self._send_action_goal(
            self.move_group_client,
            goal,
            f'MoveGroup plan {label}',
            wait_server_sec=self.move_action_wait_sec,
        )
        result = response.result
        error_code = result.error_code
        if error_code.val != 1:
            details = []
            message = str(getattr(error_code, 'message', '')).strip()
            source = str(getattr(error_code, 'source', '')).strip()
            if message:
                details.append(f'message={message}')
            if source:
                details.append(f'source={source}')
            detail_text = f', {", ".join(details)}' if details else ''
            raise TaskFailure(
                f'MoveGroup plan failed for {label}: '
                f'error_code={error_code.val}{detail_text}'
            )
        trajectory = result.planned_trajectory.joint_trajectory
        if not trajectory.points:
            raise TaskFailure(f'MoveGroup returned an empty plan for {label}')
        endpoint_positions = self._arm_trajectory_endpoint(trajectory, label)
        if not self.execute_plans:
            self._plan_only_arm_positions = endpoint_positions
        self.arm_trajectory_pub.publish(trajectory)
        self.publish_status(
            f'OK plan: {label}, points={len(trajectory.points)}'
        )
        return trajectory

    def validate_target_pose(
        self,
        target_pose: PoseStamped,
        label: str,
    ) -> None:
        p = target_pose.pose.position
        values = (p.x, p.y, p.z)
        if not all(math.isfinite(value) for value in values):
            raise TaskFailure(f'Target pose contains non-finite values: {label}')
        if p.z < self.min_goal_z_m or p.z > self.max_goal_z_m:
            raise TaskFailure(
                f'Target pose out of workspace for {label}: '
                f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, '
                f'allowed_z=[{self.min_goal_z_m:.3f}, '
                f'{self.max_goal_z_m:.3f}]'
            )

    def _planning_group_for_pose_link(self, pose_link: str) -> str:
        if pose_link == self.grasp_pose_link:
            return self.grasp_planning_group
        if pose_link == self.button_pose_link:
            return self.button_planning_group
        return self.planning_group

    def make_motion_plan_request(
        self,
        target_pose: PoseStamped,
        pose_link: str | None = None,
        planning_group: str | None = None,
    ) -> MotionPlanRequest:
        active_pose_link = pose_link or self.pose_link
        request = MotionPlanRequest()
        request.group_name = (
            planning_group
            or self._planning_group_for_pose_link(active_pose_link)
        )
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.allowed_planning_time_sec
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.start_state.is_diff = True
        if self._plan_only_arm_positions is not None:
            request.start_state.joint_state.name = list(self.arm_joint_names)
            request.start_state.joint_state.position = list(
                self._plan_only_arm_positions
            )

        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = self.target_frame
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -0.2
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.5

        constraints = Constraints()
        constraints.name = 'arm_task_pose_goal'
        position = PositionConstraint()
        position.header.frame_id = target_pose.header.frame_id
        position.link_name = active_pose_link
        position.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.position_tolerance_m] * 3
        position.constraint_region.primitives.append(box)
        position.constraint_region.primitive_poses.append(target_pose.pose)
        constraints.position_constraints.append(position)

        if self.use_orientation_constraint:
            orientation = OrientationConstraint()
            orientation.header.frame_id = target_pose.header.frame_id
            orientation.link_name = active_pose_link
            orientation.orientation = target_pose.pose.orientation
            orientation.absolute_x_axis_tolerance = (
                self.orientation_tolerance_rad
            )
            orientation.absolute_y_axis_tolerance = (
                self.orientation_tolerance_rad
            )
            orientation.absolute_z_axis_tolerance = (
                self.orientation_tolerance_rad
            )
            orientation.weight = 1.0
            constraints.orientation_constraints.append(orientation)
        request.goal_constraints.append(constraints)
        return request

    def _arm_trajectory_endpoint(
        self,
        trajectory: JointTrajectory,
        label: str,
    ) -> list[float]:
        names = list(trajectory.joint_names)
        if set(names) != set(self.arm_joint_names):
            raise TaskFailure(
                f'MoveGroup trajectory joint mismatch for {label}: '
                f'expected={self.arm_joint_names}, received={names}'
            )
        endpoint = trajectory.points[-1].positions
        if len(endpoint) != len(names):
            raise TaskFailure(
                f'MoveGroup trajectory endpoint is malformed for {label}'
            )
        by_name = {
            name: float(endpoint[index])
            for index, name in enumerate(names)
        }
        return [by_name[name] for name in self.arm_joint_names]

    @staticmethod
    def make_planning_options() -> PlanningOptions:
        options = PlanningOptions()
        options.plan_only = True
        options.look_around = False
        options.replan = False
        return options

    @staticmethod
    def make_single_point_trajectory(
        joint_names: Sequence[str],
        positions: Sequence[float],
        duration_sec: float,
        effort: float | None = None,
    ) -> JointTrajectory:
        if len(joint_names) != len(positions):
            raise TaskFailure('Trajectory position count does not match joints')
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        if effort is not None:
            point.effort = [float(effort)] * len(joint_names)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1e9
        )
        trajectory.points.append(point)
        return trajectory

    @staticmethod
    def rotate_vector_by_quaternion(
        vector: Sequence[float],
        quaternion: Any,
    ) -> tuple[float, float, float]:
        x, y, z = [float(value) for value in vector]
        qx = float(quaternion.x)
        qy = float(quaternion.y)
        qz = float(quaternion.z)
        qw = float(quaternion.w)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return x, y, z
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )


def main() -> None:
    rclpy.init()
    node = TaskExecutorNode()
    executor = MultiThreadedExecutor(num_threads=6)
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
