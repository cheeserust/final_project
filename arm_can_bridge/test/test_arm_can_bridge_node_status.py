"""Unit tests for STM status normalization in the ROS bridge node."""

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
