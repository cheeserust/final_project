"""Unit tests for Board3 compressed position feedback assembly."""

import math
import struct

from arm_can_bridge.board3_feedback import Board3PositionFeedbackAssembler
from arm_can_bridge.can_protocol import (
    angle_raw_to_rad,
    unpack_board3_position_feedback,
)


def make_group(
    group_index,
    first_raw,
    second_raw,
    third_raw,
    *,
    flags=0x40,
):
    """Create one decoded 0x303 feedback group."""
    data = struct.pack(
        '<BhhhB',
        group_index,
        first_raw,
        second_raw,
        third_raw,
        flags,
    )
    return unpack_board3_position_feedback(data)


def test_assembler_returns_snapshot_after_three_valid_groups():
    assembler = Board3PositionFeedbackAssembler()

    assert assembler.update(make_group(1, 100, 200, 300)) is None
    assert assembler.update(make_group(2, 400, 500, 600)) is None

    snapshot = assembler.update(make_group(3, 700, 800, 900))

    assert snapshot is not None
    assert len(snapshot.positions_rad) == 9
    assert math.isclose(snapshot.positions_rad[0], angle_raw_to_rad(100))
    assert math.isclose(snapshot.positions_rad[8], angle_raw_to_rad(900))
    assert snapshot.status_codes == (0,) * 9
    assert snapshot.raw_flags == (0x40, 0x40, 0x40)
    assert snapshot.fault_groups == ()


def test_assembler_ignores_invalid_groups():
    assembler = Board3PositionFeedbackAssembler()

    assert assembler.update(make_group(1, 100, 200, 300, flags=0x01)) is None
    assert assembler.update(make_group(2, 400, 500, 600)) is None
    assert assembler.update(make_group(3, 700, 800, 900)) is None


def test_assembler_accepts_v1_1_zero_reserved_flags():
    assembler = Board3PositionFeedbackAssembler()

    assert assembler.update(make_group(1, 100, 200, 300, flags=0x00)) is None
    assert assembler.update(make_group(2, 400, 500, 600, flags=0x00)) is None
    snapshot = assembler.update(make_group(3, 700, 800, 900, flags=0x00))

    assert snapshot is not None
    assert snapshot.raw_flags == (0x00, 0x00, 0x00)
    assert snapshot.status_codes == (0,) * 9


def test_assembler_discards_stale_partial_cycle():
    assembler = Board3PositionFeedbackAssembler(max_cycle_age_s=0.05)

    assert (
        assembler.update(
            make_group(1, 100, 200, 300),
            received_at=1.0,
        )
        is None
    )
    assert (
        assembler.update(
            make_group(2, 400, 500, 600),
            received_at=1.2,
        )
        is None
    )
    assert (
        assembler.update(
            make_group(3, 700, 800, 900),
            received_at=1.21,
        )
        is None
    )
