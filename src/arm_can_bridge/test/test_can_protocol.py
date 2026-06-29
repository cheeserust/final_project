"""Unit tests for the finalized Board1 CAN protocol."""

import math

from arm_can_bridge.can_protocol import (
    ALL_MOTORS,
    angle_raw_to_rad,
    BoardError,
    BoardState,
    build_control_byte,
    decode_control_byte,
    duration_ns_to_ticks,
    pack_clear_error,
    pack_enable,
    pack_estop,
    pack_homing,
    pack_position_command,
    rad_to_angle_raw,
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


def test_control_commands_use_final_can_ids_and_payload_lengths():
    assert pack_estop().can_id == 0x001
    assert pack_estop().data == bytes([ALL_MOTORS])

    assert pack_enable(True).can_id == 0x010
    assert pack_enable(True).data == bytes([ALL_MOTORS, 1])
    assert pack_enable(False).data == bytes([ALL_MOTORS, 0])

    assert pack_homing().can_id == 0x020
    assert pack_homing().data == bytes([ALL_MOTORS, ALL_MOTORS, 0])

    assert pack_clear_error().can_id == 0x030
    assert pack_clear_error().data == bytes([ALL_MOTORS, ALL_MOTORS])


def test_status_unpack_and_ready_properties():
    raw = bytes([
        BoardState.IDLE,
        BoardError.NONE,
        0x0F,
        ALL_MOTORS,
        0x00,
        32,
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
