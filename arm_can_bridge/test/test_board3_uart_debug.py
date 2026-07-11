"""Tests for the Board3 UART debug protocol helpers."""

from arm_can_bridge.board3_uart_debug import (
    crc16_ccitt_false,
    encode_uart_frame,
    FLAG_ACK_REQ,
    TYPE_ACK,
    TYPE_COMMAND,
    UartFrame,
    UartFrameParser,
)


def test_crc_matches_uart_protocol_enable_example():
    frame = UartFrame(
        frame_type=TYPE_COMMAND,
        flags=FLAG_ACK_REQ,
        seq=0x10,
        board_id=0xFF,
        msg_id=0x010,
        payload=bytes.fromhex('0100000000000000'),
    )

    assert encode_uart_frame(frame) == bytes.fromhex(
        'AA 55 01 01 01 10 FF 10 00 08 '
        '01 00 00 00 00 00 00 00 E4 07'
    )


def test_crc_matches_uart_protocol_board2_move_example():
    frame = UartFrame(
        frame_type=TYPE_COMMAND,
        flags=FLAG_ACK_REQ,
        seq=0x11,
        board_id=0x02,
        msg_id=0x102,
        payload=bytes.fromhex('80B80B0000E8030A'),
    )

    assert encode_uart_frame(frame) == bytes.fromhex(
        'AA 55 01 01 01 11 02 02 01 08 '
        '80 B8 0B 00 00 E8 03 0A F4 06'
    )


def test_parser_decodes_chunked_ack_example():
    raw = bytes.fromhex('AA 55 01 03 00 11 02 02 01 02 00 20 57 DF')
    parser = UartFrameParser()

    assert parser.feed(raw[:3]) == []
    frames = parser.feed(raw[3:])

    assert len(frames) == 1
    assert frames[0] == UartFrame(
        frame_type=TYPE_ACK,
        flags=0x00,
        seq=0x11,
        board_id=0x02,
        msg_id=0x102,
        payload=bytes.fromhex('0020'),
    )


def test_parser_skips_bad_crc_without_nack():
    raw = bytearray(
        encode_uart_frame(
            UartFrame(
                frame_type=TYPE_COMMAND,
                flags=FLAG_ACK_REQ,
                seq=0x01,
                board_id=0x03,
                msg_id=0x103,
                payload=bytes.fromhex('8000000000F40164'),
            )
        )
    )
    raw[-1] ^= 0xFF
    parser = UartFrameParser()

    assert parser.feed(bytes(raw)) == []
    assert parser.crc_error_count == 1


def test_crc16_ccitt_false_known_check_value():
    assert crc16_ccitt_false(b'123456789') == 0x29B1
