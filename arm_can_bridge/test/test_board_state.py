"""Unit tests for board runtime status tracking."""

from arm_can_bridge.board_state import BoardStateTracker
from arm_can_bridge.board_state import MultiBoardStateTracker
from arm_can_bridge.can_protocol import (
    ALL_MOTORS,
    BOARD3_SERVO_COUNT,
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD2,
    BOARD_ID_BOARD3,
    BoardError,
    BoardState,
    BoardStatus,
)
import pytest


def compact_status_bytes(axis_flags):
    """Pack up to four compact axis flag nibbles into status bytes 2 and 3."""
    padded = list(axis_flags) + [0, 0, 0, 0]
    return (
        (padded[0] & 0x0F) | ((padded[1] & 0x0F) << 4),
        (padded[2] & 0x0F) | ((padded[3] & 0x0F) << 4),
    )


def make_status(
    *,
    state=BoardState.IDLE,
    error_code=BoardError.NONE,
    homing_done_bits=None,
    moving_motor_id=None,
    axis_flags=None,
    limit_status_bits=0,
    queue_free=32,
    enabled=True,
    board_id=BOARD_ID_BOARD1,
    reserved=0,
) -> BoardStatus:
    """Create one BoardStatus with safe defaults."""
    if board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2):
        if axis_flags is None:
            axis_count = 4 if board_id == BOARD_ID_BOARD1 else 1
            axis_flags = [0x0B] * axis_count
        compact_byte2, compact_byte3 = compact_status_bytes(axis_flags)
        if homing_done_bits is None:
            homing_done_bits = compact_byte2
        if moving_motor_id is None:
            moving_motor_id = compact_byte3
    else:
        if homing_done_bits is None:
            homing_done_bits = 1
        if moving_motor_id is None:
            moving_motor_id = 0

    return BoardStatus(
        state=int(state),
        error_code=int(error_code),
        homing_done_bits=homing_done_bits,
        moving_motor_id=moving_motor_id,
        limit_status_bits=limit_status_bits,
        queue_free=queue_free,
        enabled=enabled,
        reserved=reserved,
        board_id=board_id,
    )


def make_ready_tracker(received_at=10.0) -> BoardStateTracker:
    """Create a tracker with fresh, homed, enabled status."""
    tracker = BoardStateTracker(status_timeout_ms=500)
    tracker.update_status(
        make_status(),
        received_at=received_at,
    )
    tracker.mark_commanded_position_valid()
    return tracker


def test_initial_state_has_no_usable_status():
    tracker = BoardStateTracker(status_timeout_ms=500)

    assert tracker.has_status() is False
    assert tracker.is_status_stale(now=0.0) is True
    assert tracker.can_accept_new_trajectory(now=0.0) is False
    assert tracker.available_queue_slots() == 0


def test_fresh_ready_status_accepts_new_trajectory():
    tracker = make_ready_tracker()

    assert tracker.is_status_stale(now=10.4) is False
    assert tracker.is_enabled() is True
    assert tracker.all_axes_homed() is True
    assert tracker.has_error() is False
    assert tracker.can_accept_new_trajectory(now=10.4) is True
    assert tracker.is_trajectory_complete(now=10.4) is True


def test_status_becomes_stale_after_timeout():
    tracker = make_ready_tracker()

    assert tracker.is_status_stale(now=10.5) is False
    assert tracker.is_status_stale(now=10.500001) is True
    assert tracker.can_accept_new_trajectory(now=10.6) is False
    assert tracker.is_trajectory_complete(now=10.6) is False


def test_moving_board_can_stream_but_cannot_accept_new_goal():
    tracker = BoardStateTracker(status_timeout_ms=500)
    tracker.update_status(
        make_status(
            state=BoardState.MOVING,
            axis_flags=[0x07, 0x07, 0x07, 0x07],
            queue_free=12,
        ),
        received_at=20.0,
    )
    tracker.mark_commanded_position_valid()

    assert tracker.can_accept_new_trajectory(now=20.1) is False
    assert tracker.can_stream_slots(4, now=20.1) is True
    assert tracker.is_trajectory_complete(now=20.1) is False


def test_error_estop_disabled_or_unhomed_blocks_motion():
    cases = [
        make_status(
            state=BoardState.ERROR,
            error_code=BoardError.QUEUE_FULL,
        ),
        make_status(state=BoardState.ESTOP),
        make_status(enabled=False),
        make_status(axis_flags=[0x0B, 0x0B, 0x0B, 0x00]),
    ]

    for status in cases:
        tracker = BoardStateTracker(status_timeout_ms=500)
        tracker.update_status(status, received_at=30.0)
        tracker.mark_commanded_position_valid()

        assert tracker.can_accept_new_trajectory(now=30.1) is False
        assert tracker.can_stream_slots(4, now=30.1) is False


def test_reserve_queue_slots_decrements_local_credit():
    tracker = BoardStateTracker(status_timeout_ms=500)
    tracker.update_status(
        make_status(queue_free=8),
        received_at=40.0,
    )
    tracker.mark_commanded_position_valid()

    assert tracker.reserve_queue_slots(4, now=40.1) is True
    assert tracker.available_queue_slots() == 4

    assert tracker.reserve_queue_slots(4, now=40.1) is True
    assert tracker.available_queue_slots() == 0

    assert tracker.reserve_queue_slots(4, now=40.1) is False
    assert tracker.available_queue_slots() == 0


def test_new_status_refreshes_local_queue_credit():
    tracker = make_ready_tracker(received_at=50.0)

    assert tracker.reserve_queue_slots(4, now=50.1) is True
    assert tracker.available_queue_slots() == 28

    tracker.update_status(
        make_status(queue_free=30),
        received_at=50.2,
    )

    assert tracker.available_queue_slots() == 30


def test_refund_does_not_exceed_last_reported_credit():
    tracker = BoardStateTracker(status_timeout_ms=500)
    tracker.update_status(
        make_status(queue_free=8),
        received_at=60.0,
    )
    tracker.mark_commanded_position_valid()

    assert tracker.reserve_queue_slots(4, now=60.1) is True
    tracker.refund_queue_slots(4)
    assert tracker.available_queue_slots() == 8

    tracker.refund_queue_slots(4)
    assert tracker.available_queue_slots() == 8


def test_invalid_queue_free_is_rejected():
    tracker = BoardStateTracker(queue_capacity=32)

    with pytest.raises(ValueError, match='outside the configured range'):
        tracker.update_status(make_status(queue_free=33))


def test_error_or_disable_invalidates_commanded_position():
    tracker = make_ready_tracker(received_at=70.0)
    assert tracker.commanded_position_valid() is True

    tracker.update_status(
        make_status(enabled=False),
        received_at=70.1,
    )
    assert tracker.commanded_position_valid() is False


def test_completion_requires_idle_empty_enabled_homed_status():
    tracker = make_ready_tracker(received_at=80.0)
    assert tracker.is_trajectory_complete(now=80.1) is True

    tracker.update_status(
        make_status(queue_free=31),
        received_at=80.2,
    )
    assert tracker.is_trajectory_complete(now=80.3) is False

    tracker.update_status(
        make_status(axis_flags=[0x0B, 0x0B, 0x07, 0x0B]),
        received_at=80.4,
    )
    assert tracker.is_trajectory_complete(now=80.5) is False


def test_board3_completion_uses_staging_status_fields():
    tracker = BoardStateTracker(
        board_id=BOARD_ID_BOARD3,
        queue_capacity=BOARD3_SERVO_COUNT,
        requires_homing=False,
        requires_ready=True,
        requires_fault_clear=True,
    )
    tracker.update_status(
        make_status(
            board_id=BOARD_ID_BOARD3,
            homing_done_bits=1,
            moving_motor_id=0,
            queue_free=BOARD3_SERVO_COUNT,
            reserved=ALL_MOTORS,
        ),
        received_at=85.0,
    )
    tracker.mark_commanded_position_valid()

    assert tracker.is_trajectory_complete(now=85.1) is True

    tracker.update_status(
        make_status(
            board_id=BOARD_ID_BOARD3,
            state=BoardState.MOVING,
            homing_done_bits=1,
            moving_motor_id=0,
            queue_free=BOARD3_SERVO_COUNT,
            reserved=ALL_MOTORS,
        ),
        received_at=85.15,
    )

    assert tracker.is_trajectory_complete(now=85.16) is False
    assert tracker.can_stream_slots(BOARD3_SERVO_COUNT, now=85.16) is False

    tracker.update_status(
        make_status(
            board_id=BOARD_ID_BOARD3,
            state=BoardState.STAGING,
            homing_done_bits=1,
            moving_motor_id=3,
            queue_free=BOARD3_SERVO_COUNT - 3,
            reserved=ALL_MOTORS,
        ),
        received_at=85.2,
    )

    assert tracker.is_trajectory_complete(now=85.3) is False


def test_snapshot_is_consistent():
    tracker = make_ready_tracker(received_at=90.0)
    assert tracker.reserve_queue_slots(4, now=90.1) is True

    snapshot = tracker.snapshot(now=90.2)

    assert snapshot.status is not None
    assert snapshot.status_age_ms == pytest.approx(200.0)
    assert snapshot.status_stale is False
    assert snapshot.local_queue_free == 28
    assert snapshot.commanded_position_valid is True


def test_multi_board_tracker_requires_all_boards_ready():
    tracker = MultiBoardStateTracker(
        board_ids=[BOARD_ID_BOARD1, BOARD_ID_BOARD2],
        status_timeout_ms=500,
    )

    tracker.update_status(
        make_status(board_id=BOARD_ID_BOARD1),
        received_at=100.0,
    )
    tracker.update_status(
        make_status(board_id=BOARD_ID_BOARD2),
        received_at=100.0,
    )
    tracker.mark_commanded_position_valid()

    assert tracker.can_accept_new_trajectory(now=100.1) is True
    assert tracker.reserve_queue_slots(
        {BOARD_ID_BOARD1: 4, BOARD_ID_BOARD2: 1},
        now=100.1,
    )
    assert tracker.snapshot(now=100.1).boards[
        BOARD_ID_BOARD1
    ].local_queue_free == 28
    assert tracker.snapshot(now=100.1).boards[
        BOARD_ID_BOARD2
    ].local_queue_free == 31

    tracker.update_status(
        make_status(
            board_id=BOARD_ID_BOARD2,
            axis_flags=[0x00],
        ),
        received_at=100.2,
    )

    assert tracker.can_accept_new_trajectory(now=100.3) is False
