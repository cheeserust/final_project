"""Unit tests for STM status normalization in the ROS bridge node."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

from arm_can_bridge.arm_can_bridge_node import ArmCanBridgeNode
from arm_can_bridge.can_protocol import (
    ALL_MOTORS,
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD2,
    BoardState,
    BoardStatus,
)
from rclpy.action import GoalResponse
from rclpy.exceptions import ParameterUninitializedException


class _NodeWithUninitializedParameter:

    def get_parameter(self, name):
        raise ParameterUninitializedException(name)


def test_normalizes_board1_axis_status_flags_from_current_firmware():
    status = BoardStatus(
        board_id=BOARD_ID_BOARD1,
        state=BoardState.IDLE,
        error_code=0,
        homing_done_bits=0xBB,
        moving_motor_id=0xBB,
        limit_status_bits=0x0F,
        queue_free=28,
        enabled=True,
        reserved=0x42,
    )

    normalized = ArmCanBridgeNode._normalize_axis_status_flags(status)

    assert normalized.homing_done_bits == 0x0F
    assert normalized.moving_motor_id == ALL_MOTORS
    assert normalized.limit_status_bits == 0
    assert normalized.queue_free == 28
    assert normalized.enabled is True


def test_normalizes_board2_single_axis_status_flags():
    status = BoardStatus(
        board_id=BOARD_ID_BOARD2,
        state=BoardState.IDLE,
        error_code=0,
        homing_done_bits=0x0B,
        moving_motor_id=0x00,
        limit_status_bits=0x01,
        queue_free=28,
        enabled=True,
        reserved=0x24,
    )

    normalized = ArmCanBridgeNode._normalize_axis_status_flags(status)

    assert normalized.homing_done_bits == 0x01
    assert normalized.moving_motor_id == ALL_MOTORS
    assert normalized.limit_status_bits == 0
    assert normalized.queue_free == 28


def test_normalizes_moving_axis_from_axis_status_flags():
    status = BoardStatus(
        board_id=BOARD_ID_BOARD1,
        state=BoardState.MOVING,
        error_code=0,
        homing_done_bits=0xBF,
        moving_motor_id=0xBB,
        limit_status_bits=0x00,
        queue_free=24,
        enabled=True,
        reserved=0,
    )

    normalized = ArmCanBridgeNode._normalize_axis_status_flags(status)

    assert normalized.homing_done_bits == 0x0F
    assert normalized.moving_motor_id == 0


def test_optional_list_parameter_accepts_uninitialized_empty_list():
    values = ArmCanBridgeNode._optional_list_parameter(
        _NodeWithUninitializedParameter(),
        'fixed_joint_state_names',
        str,
    )

    assert values == []


def test_plan_only_bridge_rejects_direct_trajectory_goal():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    node._execution_mode = 'plan_only'
    node.get_logger = Mock(return_value=Mock())
    controller = SimpleNamespace(label='arm')

    response = node._goal_callback_follow_joint_trajectory(
        Mock(),
        controller,
    )

    assert response == GoalResponse.REJECT


def test_post_home_escape_streams_one_direct_batch_and_commits_target():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    batch = SimpleNamespace(target_positions_rad=(-1.48, -1.36))
    commanded_state = SimpleNamespace(
        positions=Mock(return_value=(-1.51, -1.36)),
        start_trajectory=Mock(),
        mark_positions_valid=Mock(),
    )
    converter = SimpleNamespace(
        build_command_limit_entry_batch=Mock(return_value=batch),
    )
    streamer = SimpleNamespace(stream=Mock())
    board_state = SimpleNamespace(mark_commanded_position_valid=Mock())
    node._arm_controller = SimpleNamespace(
        commanded_state=commanded_state,
        trajectory_converter=converter,
        trajectory_streamer=streamer,
        board_state=board_state,
    )
    node.get_parameter = Mock(return_value=SimpleNamespace(value=60))
    node._mark_arm_feedback_positions = Mock()
    node.get_logger = Mock(return_value=Mock())

    assert node._execute_post_home_escape() is True
    converter.build_command_limit_entry_batch.assert_called_once_with(
        (-1.51, -1.36),
        60,
    )
    commanded_state.start_trajectory.assert_called_once_with(
        initial_positions=(-1.51, -1.36),
        batches=(batch,),
    )
    streamer.stream.assert_called_once_with((batch,))
    commanded_state.mark_positions_valid.assert_called_once_with(
        batch.target_positions_rad
    )
    board_state.mark_commanded_position_valid.assert_called_once_with()


def test_control_reservation_rejects_when_a_controller_is_active():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    arm = SimpleNamespace(
        label='arm',
        active=False,
        lock=threading.Lock(),
    )
    gripper = SimpleNamespace(
        label='gripper',
        active=True,
        lock=threading.Lock(),
    )
    node._controllers = (arm, gripper)
    node.get_logger = Mock(return_value=Mock())
    response = SimpleNamespace(success=True, message='')

    reserved = node._reserve_all_controllers(response, 'Homing')

    assert reserved is None
    assert arm.active is False
    assert gripper.active is True
    assert response.success is False
    assert 'gripper trajectory is active' in response.message
