"""Focused behavior tests for the task execution safety rules."""

import threading
import time
from types import SimpleNamespace
from unittest.mock import call, Mock

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
import pytest
from rclpy.action import CancelResponse, GoalResponse
from roscue_arm_pick.task_executor_node import (
    MarkerObservation,
    TaskCanceled,
    TaskExecutorNode,
    TaskFailure,
)
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vicpinky_interfaces.msg import DetectedMarker


def bare_executor():
    """Create an executor shell without initializing an rclpy Node."""
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor.tasks = {
        'place': {'type': 'place_fixed', 'place_name': 'robot_front_drop'},
    }
    executor.publish_phase = Mock()
    executor.go_named_arm_pose = Mock()
    executor.execute_place_fixed = Mock()
    executor._cancel_event = threading.Event()
    return executor


def test_successful_task_returns_home_after_body():
    executor = bare_executor()

    executor.run_task('place')

    assert executor.go_named_arm_pose.call_args_list == [
        call('home'),
        call('home'),
    ]


def test_failed_task_does_not_issue_recovery_home():
    executor = bare_executor()
    executor.execute_place_fixed.side_effect = TaskFailure('CAN failed')

    with pytest.raises(TaskFailure, match='CAN failed'):
        executor.run_task('place')

    executor.go_named_arm_pose.assert_called_once_with('home')


def test_fresh_atomic_marker_is_accepted():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor._marker_lock = threading.Lock()
    executor._marker_observations = {}
    executor._cancel_event = threading.Event()
    executor.marker_wait_sec = 0.1
    executor.marker_ttl_sec = 0.5
    executor.publish_phase = Mock()
    marker = DetectedMarker()
    marker.marker_id = 55
    executor._marker_observations[55] = MarkerObservation(
        marker,
        time.monotonic(),
    )

    assert executor.check_marker(55).marker_id == 55


def test_stale_marker_is_rejected():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor._marker_lock = threading.Lock()
    executor._marker_observations = {}
    executor._cancel_event = threading.Event()
    executor.marker_wait_sec = 0.02
    executor.marker_ttl_sec = 0.01
    marker = DetectedMarker()
    marker.marker_id = 55
    executor._marker_observations[55] = MarkerObservation(
        marker,
        time.monotonic() - 1.0,
    )

    with pytest.raises(TaskFailure, match='not detected'):
        executor.check_marker(55)


def test_old_source_timestamp_is_rejected_after_network_delay():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor._marker_lock = threading.Lock()
    executor._marker_observations = {}
    executor._cancel_event = threading.Event()
    executor.marker_wait_sec = 0.02
    executor.marker_ttl_sec = 0.5
    executor.get_clock = Mock(return_value=SimpleNamespace(
        now=Mock(return_value=SimpleNamespace(nanoseconds=3_000_000_000)),
    ))
    marker = DetectedMarker()
    marker.marker_id = 55
    marker.header.stamp.sec = 1
    executor._marker_observations[55] = MarkerObservation(
        marker,
        time.monotonic(),
    )

    with pytest.raises(TaskFailure, match='not detected'):
        executor.check_marker(55)


def test_move_group_failure_reason_is_preserved():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor.move_group_client = Mock()
    executor.move_action_wait_sec = 1.0
    executor.validate_target_pose = Mock()
    executor.publish_phase = Mock()
    executor.make_motion_plan_request = Mock()
    executor.make_planning_options = Mock()
    executor._send_action_goal = Mock(return_value=SimpleNamespace(
        result=SimpleNamespace(error_code=SimpleNamespace(val=-31)),
    ))

    with pytest.raises(TaskFailure, match='error_code=-31'):
        executor.plan_pose(PoseStamped(), 'pick approach')


def test_follow_trajectory_failure_reason_is_preserved():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    result = FollowJointTrajectory.Result()
    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
    result.error_string = 'CAN execution failed'
    executor._send_action_goal = Mock(return_value=SimpleNamespace(
        result=result,
    ))
    trajectory = JointTrajectory()
    trajectory.points.append(JointTrajectoryPoint())

    with pytest.raises(TaskFailure, match='CAN execution failed'):
        executor._execute_follow_trajectory(
            Mock(),
            trajectory,
            'planned arm grasp',
        )


def test_cancel_is_forwarded_to_active_downstream_action():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor._cancel_event = threading.Event()
    executor._state_lock = threading.RLock()
    executor._active_downstream_goal = Mock()
    executor.get_logger = Mock(return_value=Mock())

    response = executor.cancel_callback(Mock())

    assert response == CancelResponse.ACCEPT
    assert executor._cancel_event.is_set()
    executor._active_downstream_goal.cancel_goal_async.assert_called_once()


def test_second_goal_is_rejected_while_task_is_reserved():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor.execution_mode = 'hardware'
    executor.tasks = {}
    executor.mission_allowed_tasks = set()
    executor._state_lock = threading.RLock()
    executor._execution_reserved = True
    executor.get_logger = Mock(return_value=Mock())

    response = executor.goal_callback(Mock(), endpoint='homing')

    assert response == GoalResponse.REJECT


def test_run_task_result_contains_downstream_failure():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor.tasks = {
        'pick_object_2': {'type': 'pick_to_fixed_place'},
    }
    executor.mission_allowed_tasks = {'pick_object_2'}
    executor._cancel_event = threading.Event()
    executor._state_lock = threading.RLock()
    executor._active_goal_handle = None
    executor._active_downstream_goal = None
    executor._execution_reserved = True
    executor.publish_status = Mock()
    executor.get_logger = Mock(return_value=Mock())
    executor.run_task = Mock(side_effect=TaskFailure('CAN board timeout'))
    request = SimpleNamespace(
        target_floor=5,
        extra_json='{"arm_task_name": "pick_object_2"}',
    )
    goal_handle = SimpleNamespace(
        request=request,
        abort=Mock(),
        canceled=Mock(),
        succeed=Mock(),
    )

    result = executor.execute_action_callback(
        goal_handle,
        endpoint='execute',
    )

    assert result.success is False
    assert result.message == 'CAN board timeout'
    goal_handle.abort.assert_called_once()
    goal_handle.succeed.assert_not_called()


def test_canceled_task_uses_canceled_result_state():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor.tasks = {
        'pick_object_2': {'type': 'pick_to_fixed_place'},
    }
    executor.mission_allowed_tasks = {'pick_object_2'}
    executor._cancel_event = threading.Event()
    executor._state_lock = threading.RLock()
    executor._active_goal_handle = None
    executor._active_downstream_goal = None
    executor._execution_reserved = True
    executor.publish_status = Mock()
    executor.get_logger = Mock(return_value=Mock())
    executor.run_task = Mock(side_effect=TaskCanceled('operator canceled'))
    request = SimpleNamespace(
        target_floor=5,
        extra_json='{"arm_task_name": "pick_object_2"}',
    )
    goal_handle = SimpleNamespace(
        request=request,
        abort=Mock(),
        canceled=Mock(),
        succeed=Mock(),
    )

    result = executor.execute_action_callback(
        goal_handle,
        endpoint='execute',
    )

    assert result.success is False
    assert result.message == 'operator canceled'
    goal_handle.canceled.assert_called_once()
    goal_handle.abort.assert_not_called()


def test_cancel_between_goal_acceptance_and_execution_is_not_lost():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor.tasks = {
        'pick_object_2': {'type': 'pick_to_fixed_place'},
    }
    executor.mission_allowed_tasks = {'pick_object_2'}
    executor._cancel_event = threading.Event()
    executor._cancel_event.set()
    executor._state_lock = threading.RLock()
    executor._active_goal_handle = None
    executor._active_downstream_goal = None
    executor._execution_reserved = True
    executor.publish_status = Mock()
    executor.get_logger = Mock(return_value=Mock())
    executor.run_task = Mock()
    request = SimpleNamespace(
        target_floor=5,
        extra_json='{"arm_task_name": "pick_object_2"}',
    )
    goal_handle = SimpleNamespace(
        request=request,
        abort=Mock(),
        canceled=Mock(),
        succeed=Mock(),
    )

    result = executor.execute_action_callback(
        goal_handle,
        endpoint='execute',
    )

    assert result.success is False
    goal_handle.canceled.assert_called_once()
    goal_handle.succeed.assert_not_called()


def test_manual_topic_never_enables_hardware_execution():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor._state_lock = threading.RLock()
    executor._execution_reserved = False
    executor._active_goal_handle = None
    executor._active_downstream_goal = None
    executor._cancel_event = threading.Event()
    executor._plan_only_arm_positions = None
    executor.execute_plans = True
    executor.publish_status = Mock()
    executor.get_logger = Mock(return_value=Mock())
    observed_execution_flags = []
    executor.run_task = Mock(side_effect=lambda _task: (
        observed_execution_flags.append(executor.execute_plans)
    ))
    message = String()
    message.data = 'pick_object_2'

    executor.manual_task_callback(message)

    assert observed_execution_flags == [False]
    assert executor.execute_plans is True


def test_plan_only_endpoint_is_reordered_to_configured_joint_order():
    executor = TaskExecutorNode.__new__(TaskExecutorNode)
    executor.arm_joint_names = ['joint_a', 'joint_b']
    trajectory = JointTrajectory()
    trajectory.joint_names = ['joint_b', 'joint_a']
    point = JointTrajectoryPoint()
    point.positions = [2.0, 1.0]
    trajectory.points.append(point)

    endpoint = executor._arm_trajectory_endpoint(trajectory, 'test plan')

    assert endpoint == [1.0, 2.0]
