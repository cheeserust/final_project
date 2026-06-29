"""Unit tests for Linux SocketCAN frame conversion."""

import struct

from arm_can_bridge.can_protocol import (
    CAN_ID_STATUS,
    CanFrame,
    pack_estop,
    pack_position_command,
)
from arm_can_bridge.socketcan_transport import (
    build_socketcan_filter_data,
    CAN_EFF_FLAG,
    CAN_FRAME_FORMAT,
    CAN_FRAME_SIZE,
    CAN_RTR_FLAG,
    CAN_SFF_MASK,
    decode_socketcan_frame,
    encode_socketcan_frame,
    UnsupportedSocketCanFrame,
)
import pytest


def test_estop_encodes_target_board_payload():
    """Verify that ESTOP carries the target board id payload."""
    encoded = encode_socketcan_frame(pack_estop())
    raw_can_id, data_length, payload = struct.unpack(
        CAN_FRAME_FORMAT,
        encoded,
    )

    assert len(encoded) == CAN_FRAME_SIZE
    assert raw_can_id == 0x001
    assert data_length == 1
    assert payload[:1] == b'\xFF'
    assert payload[1:] == bytes(7)


def test_position_command_round_trip_preserves_all_bytes():
    """Verify an eight-byte 0x101 command survives encode/decode."""
    original = pack_position_command(
        motor_id=2,
        target_pos=-4500,
        speed=0,
        duration_ticks=10,
    )

    decoded = decode_socketcan_frame(
        encode_socketcan_frame(original)
    )

    assert decoded == original
    assert decoded.can_id == 0x101
    assert len(decoded.data) == 8


def test_status_frame_decodes_with_reported_dlc():
    """Verify that one status frame is decoded as an eight-byte payload."""
    status_payload = bytes.fromhex('01000FFF00200100')
    encoded = struct.pack(
        CAN_FRAME_FORMAT,
        CAN_ID_STATUS,
        len(status_payload),
        status_payload,
    )

    decoded = decode_socketcan_frame(encoded)

    assert decoded.can_id == CAN_ID_STATUS
    assert decoded.data == status_payload


def test_decode_rejects_wrong_linux_frame_size():
    """Reject buffers that are not Linux ``struct can_frame`` sized."""
    with pytest.raises(ValueError, match='must contain'):
        decode_socketcan_frame(bytes(15))


def test_decode_rejects_invalid_classic_can_dlc():
    """Reject a Classic CAN frame reporting more than eight bytes."""
    encoded = struct.pack(
        CAN_FRAME_FORMAT,
        CAN_ID_STATUS,
        9,
        bytes(8),
    )

    with pytest.raises(ValueError, match='0..8'):
        decode_socketcan_frame(encoded)


@pytest.mark.parametrize('flag', [CAN_EFF_FLAG, CAN_RTR_FLAG])
def test_decode_rejects_unsupported_frame_flags(flag):
    """Reject extended and remote frames from the Board1 protocol."""
    encoded = struct.pack(
        CAN_FRAME_FORMAT,
        CAN_ID_STATUS | flag,
        0,
        bytes(8),
    )

    with pytest.raises(UnsupportedSocketCanFrame):
        decode_socketcan_frame(encoded)


def test_filter_data_contains_exact_standard_id_mask():
    """Verify the kernel filter matches only standard Board1 status data."""
    data = build_socketcan_filter_data([CAN_ID_STATUS])
    can_id, mask = struct.unpack('=II', data)

    assert can_id == CAN_ID_STATUS
    assert mask & CAN_SFF_MASK == CAN_SFF_MASK
    assert mask & CAN_EFF_FLAG == 0
    assert mask & CAN_RTR_FLAG == 0


def test_filter_builder_removes_duplicate_ids():
    """Avoid installing duplicate kernel receive filters."""
    data = build_socketcan_filter_data([
        CAN_ID_STATUS,
        CAN_ID_STATUS,
    ])

    assert len(data) == struct.calcsize('=II')


def test_filter_builder_rejects_non_standard_id():
    """Reject receive filters outside the 11-bit identifier range."""
    with pytest.raises(ValueError, match='11-bit'):
        build_socketcan_filter_data([0x800])


def test_can_frame_encoding_keeps_short_payload_length():
    """Verify variable-length Board1 control payloads keep their DLC."""
    original = CanFrame(can_id=0x010, data=b'\x01')
    encoded = encode_socketcan_frame(original)
    decoded = decode_socketcan_frame(encoded)

    assert decoded == original
    assert decoded.data == b'\x01'
