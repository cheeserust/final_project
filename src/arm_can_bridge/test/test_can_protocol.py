"""Unit tests for the finalized Board1 CAN protocol."""

import math

from arm_can_bridge.can_protocol import (
    ALL_MOTORS,
    angle_raw_to_rad,
    BOARD3_SERVO_COUNT,
    BOARD3_TARGET_LOAD_MAX,
    Board3FeedbackMotorStatus,
    BOARD_ID_BOARD2,
    BOARD_ID_BOARD3,
    BoardError,
    BoardState,
    build_control_byte,
    decode_control_byte,
    duration_ns_to_ticks,
    error_name_for_board,
    pack_board3_servo_command,
    pack_clear_error,
    pack_enable,
    pack_estop,
    pack_gripper_home,
    pack_homing,
    pack_position_command,
    rad_to_angle_raw,
    unpack_board3_position_feedback,
    unpack_motor_position_feedback,
    unpack_status,
)
import pytest


def test_angle_conversion_uses_point_zero_one_degree_units():
    assert rad_to_angle_raw(0.0) == 0
    assert rad_to_angle_raw(math.pi / 6.0) == 3000
    assert rad_to_angle_raw(math.pi / 2.0) == 9000
    assert rad_to_angle_raw(-math.pi / 4.0) == -4500
    assert math.isclose(
        angle_raw_to_rad(9000),
        math.pi / 2.0,
        rel_tol=1e-9,
    )


def test_board3_position_feedback_unpack_uses_int16_groups():
    data = bytes([
        0x02,
        0xB8,
        0x0B,
        0xF2,
        0xF9,
        0x00,
        0x00,
        0x64,
    ])

    feedback = unpack_board3_position_feedback(data)

    assert feedback.group_index == 2
    assert feedback.motor_ids == (3, 4, 5)
    assert feedback.positions_raw == (3000, -1550, 0)
    assert math.isclose(feedback.positions_rad[0], math.pi / 6.0)
    assert math.isclose(feedback.positions_rad[1], angle_raw_to_rad(-1550))
    assert feedback.status_codes == (
        Board3FeedbackMotorStatus.OK,
        Board3FeedbackMotorStatus.MOVING,
        Board3FeedbackMotorStatus.CONTACT_HOLD,
    )
    assert feedback.valid is True
    assert feedback.fault is False
    assert feedback.raw_flags == 0x64


def test_board3_position_feedback_rejects_bad_payload():
    with pytest.raises(ValueError, match='8 bytes'):
        unpack_board3_position_feedback(bytes(7))

    with pytest.raises(ValueError, match='1..3'):
        unpack_board3_position_feedback(bytes([4, 0, 0, 0, 0, 0, 0, 0]))


def test_motor_position_feedback_unpack_uses_int32_angle():
    data = bytes([
        0x02,
        0x0F,
        0xF2,
        0xF9,
        0xFF,
        0xFF,
        0x00,
        0x2A,
    ])

    feedback = unpack_motor_position_feedback(data, board_id=1)

    assert feedback.board_id == 1
    assert feedback.motor_id == 2
    assert feedback.flags == 0x0F
    assert feedback.position_raw == -1550
    assert math.isclose(feedback.position_rad, angle_raw_to_rad(-1550))
    assert feedback.error_code == 0
    assert feedback.sequence == 0x2A
    assert feedback.position_valid is True
    assert feedback.homed is True
    assert feedback.moving is True
    assert feedback.target_reached is True


def test_motor_position_feedback_rejects_bad_payload():
    with pytest.raises(ValueError, match='8 bytes'):
        unpack_motor_position_feedback(bytes(7), board_id=1)

    with pytest.raises(ValueError, match='invalid for board'):
        unpack_motor_position_feedback(
            bytes([4, 1, 0, 0, 0, 0, 0, 0]),
            board_id=1,
        )

    with pytest.raises(ValueError, match='reserved flag'):
        unpack_motor_position_feedback(
            bytes([0, 0x10, 0, 0, 0, 0, 0, 0]),
            board_id=2,
        )


def test_duration_rounds_up_to_five_millisecond_ticks():
    assert duration_ns_to_ticks(1_000_000) == 1
    assert duration_ns_to_ticks(5_000_000) == 1
    assert duration_ns_to_ticks(7_000_000) == 2
    assert duration_ns_to_ticks(50_000_000) == 10


def test_duration_rejects_zero_and_values_over_uint8_limit():
    with pytest.raises(ValueError):
        duration_ns_to_ticks(0)

    with pytest.raises(OverflowError):
        duration_ns_to_ticks(1_275_000_001)


def test_default_control_byte_matches_protocol_example():
    assert build_control_byte(0) == 0x80

    decoded = decode_control_byte(0x80)
    assert decoded.execute is True
    assert decoded.relative is False
    assert decoded.step_mode is False
    assert decoded.motor_id == 0


def test_control_byte_combines_flags_and_motor_id():
    value = build_control_byte(
        3,
        execute=True,
        relative=True,
        step_mode=True,
    )

    assert value == 0xE3

    decoded = decode_control_byte(value)
    assert decoded.execute is True
    assert decoded.relative is True
    assert decoded.step_mode is True
    assert decoded.motor_id == 3


def test_position_command_matches_30_degree_example():
    frame = pack_position_command(
        motor_id=0,
        target_pos=3000,
        speed=0,
        duration_ticks=10,
    )

    assert frame.can_id == 0x101
    assert frame.data == bytes([
        0x80,
        0xB8,
        0x0B,
        0x00,
        0x00,
        0x00,
        0x00,
        0x0A,
    ])


def test_position_command_supports_negative_target():
    frame = pack_position_command(
        motor_id=2,
        target_pos=-4500,
        speed=123,
        duration_ticks=20,
    )

    assert frame.data[0] == 0x82
    assert int.from_bytes(
        frame.data[1:5],
        byteorder='little',
        signed=True,
    ) == -4500
    assert int.from_bytes(
        frame.data[5:7],
        byteorder='little',
        signed=False,
    ) == 123
    assert frame.data[7] == 20


def test_board3_servo_command_uses_target_load_field():
    frame = pack_board3_servo_command(
        motor_id=0,
        target_pos=0,
        target_load=500,
        duration_ticks=100,
    )

    assert frame.can_id == 0x103
    assert frame.data == bytes([
        0x80,
        0x00,
        0x00,
        0x00,
        0x00,
        0xF4,
        0x01,
        0x64,
    ])


def test_board3_servo_command_rejects_invalid_target_load():
    with pytest.raises(ValueError, match='target_load'):
        pack_board3_servo_command(
            motor_id=0,
            target_pos=0,
            target_load=BOARD3_TARGET_LOAD_MAX + 1,
            duration_ticks=100,
        )


def test_control_commands_use_final_can_ids_and_payload_lengths():
    assert pack_estop().can_id == 0x001
    assert pack_estop().data == bytes.fromhex('0100000000000000')

    assert pack_enable(True).can_id == 0x010
    assert pack_enable(True).data == bytes.fromhex('0100000000000000')
    assert pack_enable(True, board_id=3).data == bytes.fromhex(
        '0100000000000000'
    )
    assert pack_enable(False).data == bytes.fromhex('0000000000000000')

    assert pack_homing().can_id == 0x020
    assert pack_homing().data == bytes.fromhex('FF00000000000000')

    assert pack_clear_error().can_id == 0x030
    assert pack_clear_error().data == bytes.fromhex('FF00000000000000')
    assert pack_clear_error(board_id=3).data == bytes.fromhex(
        'FF00000000000000'
    )

    assert pack_gripper_home().can_id == 0x023
    assert pack_gripper_home().data == bytes.fromhex('FF00000000000000')
    assert pack_gripper_home(duration_ticks=100).data == bytes.fromhex(
        'FF00640000000000'
    )

    with pytest.raises(ValueError, match='Board3 home'):
        pack_homing(board_id=3)
    with pytest.raises(ValueError, match='0xFF'):
        pack_homing(0, board_id=2)
    with pytest.raises(ValueError, match='0xFF'):
        pack_clear_error(0, board_id=2)


def test_board_specific_error_names_match_integrated_protocol():
    assert error_name_for_board(5, 1) == 'QUEUE_FULL'
    assert error_name_for_board(5, 2) == 'QUEUE_FULL'
    assert error_name_for_board(5, 3) == 'ERR_DURATION_MISMATCH'
    assert error_name_for_board(9, 3) == 'ERR_ESTOP'
    assert error_name_for_board(99, 3) == 'UNKNOWN(99)'


def test_status_unpack_and_ready_properties():
    raw = bytes([
        BoardState.IDLE,
        BoardError.NONE,
        0xBB,
        0xBB,
        0x00,
        1,
        1,
        0,
    ])

    status = unpack_status(raw)

    assert status.state == BoardState.IDLE
    assert status.error_code == BoardError.NONE
    assert status.all_required_axes_homed is True
    assert status.healthy is True
    assert status.prepared_for_trajectory is True
    assert status.trajectory_complete is True
    assert status.goal_slot_free == 1
    assert status.moving_mask == 0
    assert status.target_reached_mask == 0x0F


def test_board3_contact_hold_is_ready_and_complete():
    raw = bytes([
        BoardState.CONTACT_HOLD,
        BoardError.NONE,
        0x01,
        0,
        0x00,
        BOARD3_SERVO_COUNT,
        1,
        ALL_MOTORS,
    ])

    status = unpack_status(raw, board_id=BOARD_ID_BOARD3)

    assert status.state == BoardState.CONTACT_HOLD
    assert status.healthy is True
    assert status.prepared_for_trajectory is True
    assert status.trajectory_complete is True


def test_board2_legacy_status_accepts_full_queue_credit():
    raw = bytes([
        BoardState.IDLE,
        BoardError.NONE,
        0x0B,
        0,
        0,
        32,
        1,
        7,
    ])

    status = unpack_status(
        raw,
        board_id=BOARD_ID_BOARD2,
        board2_legacy=True,
    )

    assert status.goal_slot_free == 32
    assert status.target_reached_mask == 0x01
    assert status.trajectory_complete is True


def test_board2_legacy_status_rejects_queue_credit_above_capacity():
    raw = bytes([
        BoardState.IDLE,
        BoardError.NONE,
        0x0B,
        0,
        0,
        33,
        1,
        7,
    ])

    with pytest.raises(ValueError, match='0..32'):
        unpack_status(
            raw,
            board_id=BOARD_ID_BOARD2,
            board2_legacy=True,
        )


def test_board2_legacy_status_rejects_error_above_protocol_range():
    raw = bytes([
        BoardState.MOVING,
        7,
        0x07,
        0,
        0,
        31,
        1,
        7,
    ])

    with pytest.raises(ValueError, match='error code'):
        unpack_status(
            raw,
            board_id=BOARD_ID_BOARD2,
            board2_legacy=True,
        )


@pytest.mark.parametrize('error_code', [7, 234, 235, 255])
def test_board1_status_rejects_error_above_protocol_range(error_code):
    raw = bytes([
        BoardState.MOVING,
        error_code,
        0x44,
        0x44,
        0,
        0,
        1,
        7,
    ])

    with pytest.raises(ValueError, match='error code'):
        unpack_status(raw)


@pytest.mark.parametrize('limit_bits', [0x10, 0x2E, 0xC3, 0xF0])
def test_board1_status_rejects_limit_bits_above_four_axis_mask(limit_bits):
    raw = bytes([
        BoardState.MOVING,
        BoardError.NONE,
        0x44,
        0x44,
        limit_bits,
        0,
        1,
        7,
    ])

    with pytest.raises(ValueError, match='limit status bits'):
        unpack_status(raw)


def test_board1_status_accepts_error_and_limit_boundaries():
    status = unpack_status(bytes([
        BoardState.MOVING,
        BoardError.RESERVED,
        0x44,
        0x44,
        0x0F,
        0,
        1,
        7,
    ]))

    assert status.error_code == BoardError.RESERVED
    assert status.limit_status_bits == 0x0F


def test_board3_status_keeps_board_specific_error_codes_above_six():
    status = unpack_status(
        bytes([
            BoardState.ERROR,
            10,
            0,
            0,
            0,
            BOARD3_SERVO_COUNT,
            1,
            0,
        ]),
        board_id=BOARD_ID_BOARD3,
    )

    assert status.error_code == 10


def test_error_status_is_not_ready():
    raw = bytes([
        BoardState.ERROR,
        BoardError.QUEUE_FULL,
        0x0F,
        ALL_MOTORS,
        0x00,
        0,
        1,
        0,
    ])

    status = unpack_status(raw)

    assert status.healthy is False
    assert status.prepared_for_trajectory is False
    assert status.trajectory_complete is False
