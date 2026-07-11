"""Board3-only UART protocol debug tool for hardware bring-up."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import sys
import time
from typing import Any, Optional

from .board3_feedback import Board3PositionFeedbackAssembler
from .can_protocol import (
    BOARD3_SERVO_COUNT,
    BOARD_ID_BOARD3,
    BoardError,
    BoardState,
    BoardStatus,
    CAN_ID_BOARD3_POSITION_FEEDBACK,
    CAN_ID_BOARD3_STATUS,
    CAN_ID_ENABLE,
    CanFrame,
    error_name_for_board,
    pack_board3_servo_command,
    pack_clear_error,
    pack_enable,
    pack_estop,
    pack_gripper_home,
    unpack_board3_position_feedback,
    unpack_status,
)


SOF = b'\xAA\x55'
PROTOCOL_VERSION = 0x01
MAX_PAYLOAD_LENGTH = 64

TYPE_COMMAND = 0x01
TYPE_FEEDBACK = 0x02
TYPE_ACK = 0x03
TYPE_NACK = 0x04
TYPE_HEARTBEAT = 0x05

FLAG_ACK_REQ = 0x01
FLAG_RETRY = 0x02

BOARD_ID_BROADCAST = 0xFF

ACK_RESULT_OK = 0x00

NACK_REASON_NAMES = {
    0x01: 'UNSUPPORTED_VERSION',
    0x02: 'INVALID_LENGTH',
    0x03: 'UNSUPPORTED_MSG_ID',
    0x04: 'BOARD_MISMATCH',
    0x05: 'INVALID_PAYLOAD',
    0x06: 'NOT_ENABLED',
    0x07: 'ESTOP_ACTIVE',
    0x08: 'QUEUE_FULL',
    0x09: 'STAGING_ERROR',
    0x0A: 'BUSY',
}


@dataclass(frozen=True)
class UartFrame:
    """Decoded VicPinky UART wrapper frame."""

    frame_type: int
    flags: int
    seq: int
    board_id: int
    msg_id: int
    payload: bytes

    def to_can_frame(self) -> CanFrame:
        """Return this UART frame as the equivalent CAN payload frame."""
        return CanFrame(self.msg_id, self.payload)


class UartFrameParser:
    """Incrementally parse binary VicPinky UART frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_error_count = 0
        self.length_error_count = 0

    def feed(self, data: bytes) -> list[UartFrame]:
        """Append raw bytes and return all complete valid frames."""
        if data:
            self._buffer.extend(data)

        frames: list[UartFrame] = []

        while True:
            sof_index = self._buffer.find(SOF)

            if sof_index < 0:
                self._keep_possible_sof_prefix()
                break

            if sof_index > 0:
                del self._buffer[:sof_index]

            if len(self._buffer) < 10:
                break

            payload_length = self._buffer[9]

            if payload_length > MAX_PAYLOAD_LENGTH:
                self.length_error_count += 1
                del self._buffer[0]
                continue

            total_length = 12 + payload_length

            if len(self._buffer) < total_length:
                break

            raw = bytes(self._buffer[:total_length])
            del self._buffer[:total_length]

            expected_crc = int.from_bytes(raw[-2:], 'little')
            actual_crc = crc16_ccitt_false(raw[2:-2])

            if expected_crc != actual_crc:
                self.crc_error_count += 1
                continue

            frames.append(
                UartFrame(
                    frame_type=raw[3],
                    flags=raw[4],
                    seq=raw[5],
                    board_id=raw[6],
                    msg_id=int.from_bytes(raw[7:9], 'little'),
                    payload=raw[10:-2],
                )
            )

        return frames

    def _keep_possible_sof_prefix(self) -> None:
        """Keep a trailing 0xAA byte that could start the next frame."""
        if self._buffer[-1:] == SOF[:1]:
            del self._buffer[:-1]
        else:
            self._buffer.clear()


class Board3UartMonitor:
    """Decode and print Board3 feedback frames from UART."""

    def __init__(self, *, verbose: bool = True) -> None:
        self._verbose = bool(verbose)
        self._feedback = Board3PositionFeedbackAssembler()
        self._latest_status: Optional[BoardStatus] = None
        self._latest_positions_rad: Optional[tuple[float, ...]] = None

    def handle_frame(self, frame: UartFrame) -> None:
        """Decode one incoming UART frame."""
        if frame.frame_type == TYPE_FEEDBACK:
            self._handle_feedback(frame)
            return

        if frame.frame_type == TYPE_HEARTBEAT:
            print(
                'RX HEARTBEAT '
                f'seq=0x{frame.seq:02X} board=0x{frame.board_id:02X} '
                f'payload={frame.payload.hex().upper()}'
            )
            return

        if frame.frame_type in (TYPE_ACK, TYPE_NACK):
            print(format_ack_or_nack(frame))
            return

        print(
            f'RX type=0x{frame.frame_type:02X} seq=0x{frame.seq:02X} '
            f'board=0x{frame.board_id:02X} msg=0x{frame.msg_id:03X} '
            f'payload={frame.payload.hex().upper()}'
        )

    def latest_status(self) -> Optional[BoardStatus]:
        """Return the latest decoded Board3 status."""
        return self._latest_status

    def _handle_feedback(self, frame: UartFrame) -> None:
        """Decode a Board3 status or compressed-position feedback frame."""
        if frame.msg_id == CAN_ID_BOARD3_STATUS:
            status = unpack_status(frame.payload, board_id=BOARD_ID_BOARD3)
            self._latest_status = status

            if self._verbose:
                print(f'RX 0x203 {format_board3_status(status)}')

            return

        if frame.msg_id == CAN_ID_BOARD3_POSITION_FEEDBACK:
            group = unpack_board3_position_feedback(frame.payload)
            snapshot = self._feedback.update(group)

            if snapshot is None:
                return

            self._latest_positions_rad = snapshot.positions_rad

            if self._verbose:
                print(
                    'RX 0x303 '
                    f'{format_board3_positions(snapshot.positions_rad)} '
                    f'flags={snapshot.raw_flags}'
                )

            return

        print(
            f'RX FEEDBACK board=0x{frame.board_id:02X} '
            f'msg=0x{frame.msg_id:03X} '
            f'payload={frame.payload.hex().upper()}'
        )


class UartDebugSession:
    """Small blocking UART session for command/ACK debugging."""

    def __init__(
        self,
        serial_port: Any,
        *,
        monitor: Board3UartMonitor,
        ack_timeout_s: float,
        retry_count: int,
        inter_frame_gap_s: float,
        verbose_raw: bool,
    ) -> None:
        self._serial = serial_port
        self._monitor = monitor
        self._ack_timeout_s = float(ack_timeout_s)
        self._retry_count = int(retry_count)
        self._inter_frame_gap_s = float(inter_frame_gap_s)
        self._verbose_raw = bool(verbose_raw)
        self._parser = UartFrameParser()
        self._seq = 0

    def listen(self, duration_s: float) -> None:
        """Print incoming frames for a fixed duration."""
        deadline = time.monotonic() + float(duration_s)

        while time.monotonic() < deadline:
            self._pump_once(deadline)

        self._print_parser_errors()

    def send_command(
        self,
        frame: CanFrame,
        *,
        board_id: int = BOARD_ID_BOARD3,
        ack_required: bool = True,
    ) -> bool:
        """Send one CAN-compatible command through the UART wrapper."""
        seq = self._next_seq()

        for attempt in range(self._retry_count + 1):
            flags = FLAG_ACK_REQ if ack_required else 0

            if attempt:
                flags |= FLAG_RETRY

            uart_frame = UartFrame(
                frame_type=TYPE_COMMAND,
                flags=flags,
                seq=seq,
                board_id=int(board_id),
                msg_id=frame.can_id,
                payload=frame.data,
            )
            raw = encode_uart_frame(uart_frame)

            self._serial.write(raw)
            self._serial.flush()
            print(
                format_tx_frame(
                    uart_frame,
                    raw if self._verbose_raw else b'',
                    attempt=attempt,
                )
            )
            time.sleep(self._inter_frame_gap_s)

            if not ack_required:
                return True

            ack = self._wait_for_ack(seq, frame.can_id)

            if ack is None:
                print(
                    f'ACK timeout seq=0x{seq:02X} '
                    f'msg=0x{frame.can_id:03X} attempt={attempt + 1}'
                )
                continue

            print(format_ack_or_nack(ack))

            if ack.frame_type == TYPE_ACK:
                return True

            return False

        return False

    def send_bad_crc_probe(self, frame: CanFrame) -> None:
        """Send one intentionally corrupted frame and listen for a moment."""
        seq = self._next_seq()
        uart_frame = UartFrame(
            frame_type=TYPE_COMMAND,
            flags=FLAG_ACK_REQ,
            seq=seq,
            board_id=BOARD_ID_BOARD3,
            msg_id=frame.can_id,
            payload=frame.data,
        )
        raw = bytearray(encode_uart_frame(uart_frame))
        raw[-1] ^= 0xFF
        self._serial.write(bytes(raw))
        self._serial.flush()
        print(
            'TX BAD_CRC '
            f'seq=0x{seq:02X} msg=0x{frame.can_id:03X} '
            f'raw={bytes(raw).hex(" ").upper()}'
        )
        self.listen(self._ack_timeout_s)

    def _wait_for_ack(self, seq: int, msg_id: int) -> Optional[UartFrame]:
        """Wait for a matching ACK or NACK while still printing feedback."""
        deadline = time.monotonic() + self._ack_timeout_s

        while time.monotonic() < deadline:
            frames = self._pump_once(deadline)

            for frame in frames:
                if (
                    frame.frame_type in (TYPE_ACK, TYPE_NACK)
                    and frame.seq == seq
                    and frame.msg_id == msg_id
                ):
                    return frame

        return None

    def _pump_once(self, deadline: float) -> list[UartFrame]:
        """Read serial bytes once, parse frames, and dispatch non-ACK frames."""
        timeout_s = max(0.0, min(0.05, deadline - time.monotonic()))
        waiting = getattr(self._serial, 'in_waiting', 0)
        read_size = max(1, min(512, int(waiting) if waiting else 1))
        data = self._serial.read(read_size)

        if not data and timeout_s > 0.0:
            time.sleep(timeout_s)
            return []

        frames = self._parser.feed(data)

        for frame in frames:
            if frame.frame_type not in (TYPE_ACK, TYPE_NACK):
                self._monitor.handle_frame(frame)

        return frames

    def _next_seq(self) -> int:
        """Return the next sequence number."""
        value = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return value

    def _print_parser_errors(self) -> None:
        """Print parser error counters that matter during bring-up."""
        if self._parser.crc_error_count:
            print(f'Parser CRC errors: {self._parser.crc_error_count}')

        if self._parser.length_error_count:
            print(f'Parser length errors: {self._parser.length_error_count}')


def crc16_ccitt_false(data: bytes) -> int:
    """Compute CRC-16/CCITT-FALSE."""
    crc = 0xFFFF

    for value in data:
        crc ^= int(value) << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def encode_uart_frame(frame: UartFrame) -> bytes:
    """Encode one UART wrapper frame."""
    if not 0 <= int(frame.frame_type) <= 0xFF:
        raise ValueError('frame_type must fit uint8')
    if not 0 <= int(frame.flags) <= 0xFF:
        raise ValueError('flags must fit uint8')
    if frame.flags & 0xF8:
        raise ValueError('reserved UART flag bits must be zero')
    if not 0 <= int(frame.seq) <= 0xFF:
        raise ValueError('seq must fit uint8')
    if not 0 <= int(frame.board_id) <= 0xFF:
        raise ValueError('board_id must fit uint8')
    if not 0 <= int(frame.msg_id) <= 0xFFFF:
        raise ValueError('msg_id must fit uint16')
    if len(frame.payload) > MAX_PAYLOAD_LENGTH:
        raise ValueError('UART payload length exceeds 64 bytes')

    body = bytes([
        PROTOCOL_VERSION,
        int(frame.frame_type),
        int(frame.flags),
        int(frame.seq),
        int(frame.board_id),
    ])
    body += int(frame.msg_id).to_bytes(2, 'little')
    body += bytes([len(frame.payload)])
    body += bytes(frame.payload)
    crc = crc16_ccitt_false(body)

    return SOF + body + crc.to_bytes(2, 'little')


def format_tx_frame(
    frame: UartFrame,
    raw: bytes,
    *,
    attempt: int,
) -> str:
    """Return a concise TX log line."""
    retry = ' retry=1' if attempt else ''
    raw_text = f' raw={raw.hex(" ").upper()}' if raw else ''
    return (
        f'TX COMMAND seq=0x{frame.seq:02X} board=0x{frame.board_id:02X} '
        f'msg=0x{frame.msg_id:03X} flags=0x{frame.flags:02X}{retry} '
        f'payload={frame.payload.hex().upper()}{raw_text}'
    )


def format_ack_or_nack(frame: UartFrame) -> str:
    """Return a compact ACK/NACK string."""
    if frame.frame_type == TYPE_ACK:
        result = frame.payload[0] if len(frame.payload) >= 1 else None
        detail = frame.payload[1] if len(frame.payload) >= 2 else None
        result_text = 'OK' if result == ACK_RESULT_OK else str(result)
        detail_text = 'NA' if detail is None else str(detail)
        return (
            f'RX ACK seq=0x{frame.seq:02X} board=0x{frame.board_id:02X} '
            f'msg=0x{frame.msg_id:03X} result={result_text} '
            f'queue_free/detail={detail_text}'
        )

    reason = frame.payload[0] if len(frame.payload) >= 1 else None
    detail = frame.payload[1] if len(frame.payload) >= 2 else None
    reason_text = NACK_REASON_NAMES.get(reason, str(reason))
    detail_text = 'NA' if detail is None else str(detail)
    return (
        f'RX NACK seq=0x{frame.seq:02X} board=0x{frame.board_id:02X} '
        f'msg=0x{frame.msg_id:03X} reason={reason_text} '
        f'detail={detail_text}'
    )


def format_board3_status(status: BoardStatus) -> str:
    """Return a compact Board3 status string."""
    return (
        f'state={board3_state_name(status.state)} '
        f'err={error_name_for_board(status.error_code, BOARD_ID_BOARD3)} '
        f'ready={status.homing_done_bits} '
        f'staging={status.board3_staging_count} '
        f'fault={status.limit_status_bits} '
        f'free={status.board3_buffer_free} '
        f'enabled={int(status.enabled)} '
        f'fault_id={status.board3_fault_motor_id}'
    )


def format_board3_positions(positions_rad: tuple[float, ...]) -> str:
    """Return a compact Board3 position string in degrees."""
    degrees = [f'{math.degrees(value):.2f}' for value in positions_rad]
    return 'deg=[' + ','.join(degrees) + ']'


def board3_state_name(value: int) -> str:
    """Return Board3-specific state names."""
    names = {
        0: 'INIT',
        1: 'IDLE',
        2: 'STAGING',
        3: 'MOVING',
        4: 'ERROR',
        5: 'ESTOP',
        6: 'DISABLED',
        7: 'CONTACT_HOLD',
    }
    return names.get(int(value), f'UNKNOWN({value})')


def is_board3_ready(status: BoardStatus) -> bool:
    """Return whether Board3 looks ready for servo commands."""
    return (
        status.state in (BoardState.IDLE, BoardState.CONTACT_HOLD)
        and status.error_code == BoardError.NONE
        and status.homing_done_bits == 1
        and status.limit_status_bits == 0
        and status.enabled
    )


def make_board3_command_frames(args: argparse.Namespace) -> list[CanFrame]:
    """Build the command sequence requested by CLI arguments."""
    frames: list[CanFrame] = []

    if args.estop:
        return [pack_estop()]

    if args.clear_error:
        frames.append(pack_clear_error(board_id=BOARD_ID_BOARD3))

    if not args.skip_enable:
        frames.append(pack_enable(True, board_id=BOARD_ID_BOARD3))

    if args.home:
        frames.append(pack_gripper_home(duration_ticks=args.duration_ticks))

    if not args.no_move:
        target_001deg = int(round(args.degrees * 100.0))

        for motor_id in range(BOARD3_SERVO_COUNT):
            frames.append(
                pack_board3_servo_command(
                    motor_id=motor_id,
                    target_pos=target_001deg,
                    target_load=args.target_load,
                    duration_ticks=args.duration_ticks,
                )
            )

    return frames


def board_id_for_command(frame: CanFrame, *, broadcast_common: bool) -> int:
    """Return the UART board id to use for one command frame."""
    common_ids = {
        CAN_ID_ENABLE,
        0x001,
        0x023,
        0x030,
    }

    if broadcast_common and frame.can_id in common_ids:
        return BOARD_ID_BROADCAST

    return BOARD_ID_BOARD3


def import_serial_module() -> Any:
    """Import pyserial with a helpful error if it is missing."""
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(
            'pyserial is required. Install python3-serial, then retry.'
        ) from exc

    return serial


def parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Board3 UART protocol debug/smoke test.',
    )
    parser.add_argument(
        '--port',
        default='/dev/ttyUSB0',
        help='Serial device path, default: /dev/ttyUSB0',
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=921600,
        help='UART baudrate, default: 921600',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=2.0,
        help='ACK/listen timeout in seconds, default: 2.0',
    )
    parser.add_argument(
        '--ack-timeout',
        type=float,
        default=0.02,
        help='ACK timeout per command in seconds, default: 0.02',
    )
    parser.add_argument(
        '--retry-count',
        type=int,
        default=3,
        help='ACK retry count, default: 3',
    )
    parser.add_argument(
        '--inter-frame-gap',
        type=float,
        default=0.001,
        help='Gap after each TX frame in seconds, default: 0.001',
    )
    parser.add_argument(
        '--listen-only',
        action='store_true',
        help='Only decode incoming UART frames.',
    )
    parser.add_argument(
        '--status-only',
        action='store_true',
        help='Alias for --listen-only during bring-up.',
    )
    parser.add_argument(
        '--bad-crc',
        action='store_true',
        help='Send one corrupted enable frame and check that it is ignored.',
    )
    parser.add_argument(
        '--clear-error',
        action='store_true',
        help='Send Board3 clear-error before enable/move.',
    )
    parser.add_argument(
        '--skip-enable',
        action='store_true',
        help='Do not send enable before other commands.',
    )
    parser.add_argument(
        '--home',
        action='store_true',
        help='Send Board3 gripper home posture command.',
    )
    parser.add_argument(
        '--no-move',
        action='store_true',
        help='Do not send the 9-servo move frames.',
    )
    parser.add_argument(
        '--estop',
        action='store_true',
        help='Send ESTOP and exit.',
    )
    parser.add_argument(
        '--degrees',
        type=float,
        default=0.0,
        help='Target angle for all 9 servos in degrees, default: 0.0',
    )
    parser.add_argument(
        '--duration-ticks',
        type=int,
        default=100,
        help='Duration in 5 ms ticks, default: 100 = 500 ms',
    )
    parser.add_argument(
        '--target-load',
        type=int,
        default=500,
        help='Board3 target load raw value 0..1023, default: 500',
    )
    parser.add_argument(
        '--broadcast-common',
        action='store_true',
        help='Use UART board_id 0xFF for common commands.',
    )
    parser.add_argument(
        '--no-ack',
        action='store_true',
        help='Do not request ACK/NACK from the board.',
    )
    parser.add_argument(
        '--quiet-raw',
        action='store_true',
        help='Do not print raw UART bytes on TX.',
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    """Run the Board3 UART debug tool."""
    args = parse_args(argv)
    serial_module = import_serial_module()
    monitor = Board3UartMonitor(verbose=True)

    print(f'Opening {args.port} at {args.baud} baud')

    with serial_module.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=0.005,
        write_timeout=0.1,
    ) as serial_port:
        serial_port.reset_input_buffer()
        session = UartDebugSession(
            serial_port,
            monitor=monitor,
            ack_timeout_s=args.ack_timeout,
            retry_count=args.retry_count,
            inter_frame_gap_s=args.inter_frame_gap,
            verbose_raw=not args.quiet_raw,
        )

        if args.listen_only or args.status_only:
            session.listen(args.timeout)
            return 0

        if args.bad_crc:
            session.send_bad_crc_probe(
                pack_enable(True, board_id=BOARD_ID_BOARD3)
            )
            return 0

        frames = make_board3_command_frames(args)

        if not frames:
            print('No command frames selected')
            return 0

        for frame in frames:
            board_id = board_id_for_command(
                frame,
                broadcast_common=args.broadcast_common,
            )
            ok = session.send_command(
                frame,
                board_id=board_id,
                ack_required=not args.no_ack,
            )

            if not ok:
                print(f'Command failed: msg=0x{frame.can_id:03X}')
                return 2

        session.listen(args.timeout)
        latest = monitor.latest_status()

        if latest is not None:
            print(f'Latest Board3 status: {format_board3_status(latest)}')

            if not args.estop and not is_board3_ready(latest):
                return 3

        return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
