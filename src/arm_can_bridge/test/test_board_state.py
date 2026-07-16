"""Tests for Board1 V3, Board2 legacy, and Board3 queue state."""

from arm_can_bridge.board_state import BoardStateTracker, MultiBoardStateTracker
from arm_can_bridge.can_protocol import (
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD2,
    BOARD_ID_BOARD3,
    BoardState,
    BoardStatus,
)
import pytest


def arm_status(board_id, *, state=BoardState.IDLE, axis2=0xBB,
               axis3=0xBB, slot=1, error=0, limit_bits=0):
    """Make Board1/2 status with valid+ready+reached axis nibbles."""
    return BoardStatus(
        board_id=board_id, state=state, error_code=error,
        homing_done_bits=axis2, moving_motor_id=axis3,
        limit_status_bits=limit_bits, queue_free=slot,
        enabled=True, reserved=9,
    )


def test_board1_accepts_only_fresh_idle_free_goal_slot():
    tracker = BoardStateTracker(
        board_id=BOARD_ID_BOARD1, queue_capacity=1, status_timeout_ms=500,
    )
    tracker.update_status(arm_status(BOARD_ID_BOARD1), received_at=10.0)
    tracker.mark_commanded_position_valid()
    assert tracker.all_axes_homed()
    assert tracker.can_accept_new_trajectory(now=10.1)
    assert not tracker.can_accept_new_trajectory(now=10.6)


@pytest.mark.parametrize(
    ('board_id', 'axis2', 'axis3', 'slot', 'limit_bits'),
    [
        (BOARD_ID_BOARD1, 0xBB, 0xBB, 1, 0x04),
        (BOARD_ID_BOARD2, 0x0B, 0x00, 32, 0x01),
    ],
)
def test_arm_limit_bit_alone_does_not_reject_away_motion(
    board_id, axis2, axis3, slot, limit_bits,
):
    """STM, not the server, decides whether motion approaches the limit."""
    tracker = BoardStateTracker(
        board_id=board_id,
        queue_capacity=slot,
        required_homing_mask=(0x01 if board_id == BOARD_ID_BOARD2 else 0x0F),
        status_timeout_ms=500,
    )
    tracker.update_status(
        arm_status(
            board_id,
            axis2=axis2,
            axis3=axis3,
            slot=slot,
            limit_bits=limit_bits,
        ),
        received_at=15.0,
    )
    tracker.mark_commanded_position_valid()

    assert not tracker.has_error()
    assert tracker.can_accept_new_trajectory(now=15.1)


def test_arm_error_state_rejects_motion_even_with_no_error_code():
    tracker = BoardStateTracker(
        board_id=BOARD_ID_BOARD1,
        queue_capacity=1,
        status_timeout_ms=500,
    )
    tracker.update_status(
        arm_status(
            BOARD_ID_BOARD1,
            state=BoardState.ERROR,
            limit_bits=0x04,
        ),
        received_at=16.0,
    )
    tracker.mark_commanded_position_valid()

    assert tracker.has_error()
    assert not tracker.can_accept_new_trajectory(now=16.1)


def test_busy_slot_and_moving_state_reject_a_second_goal():
    tracker = BoardStateTracker(board_id=BOARD_ID_BOARD1, queue_capacity=1)
    tracker.update_status(
        arm_status(
            BOARD_ID_BOARD1, state=BoardState.MOVING,
            axis2=0x77, axis3=0x77, slot=0,
        ),
        received_at=20.0,
    )
    tracker.mark_commanded_position_valid()
    assert not tracker.can_accept_new_trajectory(now=20.1)


def test_arm_completion_requires_all_target_masks_and_free_slot():
    tracker = BoardStateTracker(board_id=BOARD_ID_BOARD1, queue_capacity=1)
    tracker.update_status(arm_status(BOARD_ID_BOARD1), received_at=30.0)
    tracker.mark_commanded_position_valid()
    assert tracker.is_trajectory_complete(now=30.1)
    tracker.update_status(
        arm_status(BOARD_ID_BOARD1, axis3=0x3B), received_at=30.2,
    )
    assert not tracker.is_trajectory_complete(now=30.3)


def test_goal_slot_values_above_one_fail_closed():
    tracker = BoardStateTracker(board_id=BOARD_ID_BOARD1, queue_capacity=1)
    with pytest.raises(ValueError):
        tracker.update_status(
            arm_status(BOARD_ID_BOARD1, slot=124), received_at=40.0,
        )


def test_multi_board_requires_board1_and_board2_ready():
    tracker = MultiBoardStateTracker(
        board_ids=[BOARD_ID_BOARD1, BOARD_ID_BOARD2], status_timeout_ms=500,
    )
    tracker.update_status(arm_status(BOARD_ID_BOARD1), received_at=50.0)
    tracker.update_status(
        arm_status(BOARD_ID_BOARD2, axis2=0x0B, axis3=0, slot=32),
        received_at=50.0,
    )
    tracker.mark_commanded_position_valid()
    assert tracker.can_accept_new_trajectory(now=50.1)


def test_board3_keeps_legacy_nine_frame_buffer_semantics():
    tracker = BoardStateTracker(
        board_id=BOARD_ID_BOARD3, queue_capacity=9,
        requires_homing=False, requires_ready=True, requires_fault_clear=True,
    )
    tracker.update_status(
        BoardStatus(
            board_id=BOARD_ID_BOARD3, state=BoardState.IDLE,
            error_code=0, homing_done_bits=1, moving_motor_id=0,
            limit_status_bits=0, queue_free=9, enabled=True, reserved=0xFF,
        ),
        received_at=60.0,
    )
    tracker.mark_commanded_position_valid()
    assert tracker.can_accept_new_trajectory(now=60.1)
    assert tracker.reserve_queue_slots(9, now=60.1)
