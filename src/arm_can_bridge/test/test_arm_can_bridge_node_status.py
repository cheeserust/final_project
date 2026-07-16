"""Unit tests for STM status normalization in the ROS bridge node."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

from arm_can_bridge.arm_can_bridge_node import ArmCanBridgeNode
from arm_can_bridge.can_protocol import (
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD2,
    BoardState,
    BoardStatus,
    CAN_ID_BOARD1_STATUS,
    CanFrame,
)
from rclpy.action import GoalResponse
from rclpy.exceptions import ParameterUninitializedException


class _NodeWithUninitializedParameter:

    def get_parameter(self, name):
        raise ParameterUninitializedException(name)


def test_board1_v3_status_snapshot_is_not_legacy_normalized():
    status = BoardStatus(
        board_id=BOARD_ID_BOARD1,
        state=BoardState.IDLE,
        error_code=0,
        homing_done_bits=0xBB,
        moving_motor_id=0xBB,
        limit_status_bits=0x0F,
        queue_free=1,
        enabled=True,
        reserved=0x42,
    )

    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    normalized = node._normalize_board_status(status)

    assert normalized is status
    assert normalized.ready_mask == 0x0F
    assert normalized.moving_mask == 0
    assert normalized.target_reached_mask == 0x0F


def test_board2_legacy_status_snapshot_uses_single_axis_mask():
    status = BoardStatus(
        board_id=BOARD_ID_BOARD2,
        state=BoardState.IDLE,
        error_code=0,
        homing_done_bits=0x0B,
        moving_motor_id=0x00,
        limit_status_bits=0x01,
        queue_free=32,
        enabled=True,
        reserved=0x24,
    )

    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    normalized = node._normalize_board_status(status)

    assert normalized is status
    assert normalized.ready_mask == 0x01
    assert normalized.target_reached_mask == 0x01


def test_board1_v3_moving_mask_is_derived_from_same_frame():
    status = BoardStatus(
        board_id=BOARD_ID_BOARD1,
        state=BoardState.MOVING,
        error_code=0,
        homing_done_bits=0xBF,
        moving_motor_id=0xBB,
        limit_status_bits=0x00,
        queue_free=0,
        enabled=True,
        reserved=0,
    )

    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    normalized = node._normalize_board_status(status)

    assert normalized.moving_mask == 0x01
    assert normalized.status_sequence == 0


def test_impossible_board1_status_is_ignored_before_state_updates():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    node._arm_v3 = Mock()
    board_state = Mock()
    node._controllers = (SimpleNamespace(board_state=board_state),)
    logger = Mock()
    node.get_logger = Mock(return_value=logger)

    node._handle_can_frame(CanFrame(
        CAN_ID_BOARD1_STATUS,
        bytes([
            BoardState.MOVING,
            235,
            0x44,
            0x44,
            0x2E,
            0,
            1,
            7,
        ]),
    ))

    node._arm_v3.update_status.assert_not_called()
    board_state.update_status.assert_not_called()
    logger.warning.assert_called_once()


def test_valid_board1_status_is_still_forwarded_after_invalid_status():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    node._arm_v3 = Mock()
    board_state = Mock()
    node._controllers = (SimpleNamespace(board_state=board_state),)
    node.get_logger = Mock(return_value=Mock())

    node._handle_can_frame(CanFrame(
        CAN_ID_BOARD1_STATUS,
        bytes([
            BoardState.MOVING,
            234,
            0x44,
            0x44,
            0xC3,
            0,
            1,
            7,
        ]),
    ))
    node._handle_can_frame(CanFrame(
        CAN_ID_BOARD1_STATUS,
        bytes([
            BoardState.MOVING,
            0,
            0x44,
            0x44,
            0,
            0,
            1,
            8,
        ]),
    ))

    node._arm_v3.update_status.assert_called_once()
    board_state.update_status.assert_called_once_with(
        node._arm_v3.update_status.call_args.args[0]
    )


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
    node._arm_controller = SimpleNamespace(label='arm')

    response = node._goal_callback_arm_goal_v3(Mock())

    assert response == GoalResponse.REJECT


def test_post_home_escape_is_skipped_without_arm_controller():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    node._arm_controller = None

    assert node._execute_post_home_escape() is False


def test_post_home_escape_executes_five_degrees_away():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    joint_names = (
        'arm_joint_1',
        'arm_joint_2',
        'arm_joint_3',
        'base_joint',
        'arm_joint_4',
    )
    home_positions = (-1.5, -1.3, -1.5, -1.5, -1.5)
    commanded_state = Mock()
    board_state = Mock()
    node._arm_controller = SimpleNamespace(
        joint_names=joint_names,
        home_positions_rad=home_positions,
        commanded_state=commanded_state,
        board_state=board_state,
    )
    node._arm_v3 = Mock()
    node._mark_arm_feedback_positions = Mock()

    def execute(**kwargs):
        positions = dict(zip(kwargs['joint_names'], kwargs['positions_rad']))
        return SimpleNamespace(positions_by_name=positions)

    node._arm_v3.execute.side_effect = execute

    assert node._execute_post_home_escape() is True
    call = node._arm_v3.execute.call_args.kwargs
    assert call['duration_ms'] == 1000
    assert all(
        abs(target - home - 0.0872664626) < 1e-9
        for target, home in zip(call['positions_rad'], home_positions)
    )
    commanded_state.mark_positions_valid.assert_called_once()
    board_state.mark_commanded_position_valid.assert_called_once()


def test_clear_interrupt_waits_for_cancel_before_clear_without_disable():
    node = ArmCanBridgeNode.__new__(ArmCanBridgeNode)
    node._arm_controller = SimpleNamespace(active=True)
    node._arm_v3 = SimpleNamespace(
        active_goal_id=7,
        request_active_cancel=Mock(return_value=True),
    )
    node._send_or_fail = Mock(return_value=True)
    node._wait_until = Mock(return_value=True)
    node._clear_active_goal_timeout_ms = 7000
    node.get_logger = Mock(return_value=Mock())
    response = SimpleNamespace(success=False, message='')

    assert node._finish_active_arm_goal_for_clear(response) is True

    node._send_or_fail.assert_not_called()
    node._arm_v3.request_active_cancel.assert_called_once_with()
    assert node._wait_until.call_args.kwargs['timeout_s'] == 7.0


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
