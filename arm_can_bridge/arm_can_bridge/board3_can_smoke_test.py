"""Board3-only SocketCAN smoke test for hardware bring-up."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from typing import Callable, Optional

from .board3_feedback import Board3PositionFeedbackAssembler
from .can_protocol import (
    BOARD3_SERVO_COUNT,
    BOARD_ID_BOARD3,
    BoardError,
    BoardState,
    BoardStatus,
    CAN_ID_BOARD3_POSITION_FEEDBACK,
    CAN_ID_BOARD3_STATUS,
    CanFrame,
    error_name_for_board,
    pack_board3_servo_command,
    pack_clear_error,
    pack_enable,
    unpack_board3_position_feedback,
    unpack_status,
)
from .socketcan_transport import SocketCanTransport


StatusPredicate = Callable[[BoardStatus], bool]


class Board3StatusMonitor:
    """Collect and print Board3 status frames from SocketCAN."""

    def __init__(self, *, verbose: bool = True) -> None:
        self._verbose = bool(verbose)
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._latest: Optional[BoardStatus] = None
        self._feedback = Board3PositionFeedbackAssembler()
        self._latest_positions_rad: Optional[tuple[float, ...]] = None

    def handle_frame(self, frame: CanFrame) -> None:
        """Decode one Board3 status or position feedback CAN frame."""
        if frame.can_id == CAN_ID_BOARD3_POSITION_FEEDBACK:
            group = unpack_board3_position_feedback(frame.data)
            snapshot = self._feedback.update(group)
            if snapshot is None:
                return

            with self._lock:
                self._latest_positions_rad = snapshot.positions_rad

            if self._verbose:
                print(
                    'RX 0x303 '
                    f'{format_board3_positions(snapshot.positions_rad)} '
                    f'flags={snapshot.raw_flags}'
                )

            return

        if frame.can_id != CAN_ID_BOARD3_STATUS:
            return

        status = unpack_status(frame.data, board_id=BOARD_ID_BOARD3)

        with self._lock:
            self._latest = status
            self._event.set()

        if self._verbose:
            print(f'RX 0x203 {format_board3_status(status)}')

    def latest(self) -> Optional[BoardStatus]:
        """Return the latest status snapshot."""
        with self._lock:
            return self._latest

    def latest_positions_rad(self) -> Optional[tuple[float, ...]]:
        """Return the latest complete Board3 position feedback snapshot."""
        with self._lock:
            return self._latest_positions_rad

    def wait_for_status(self, timeout_s: float) -> Optional[BoardStatus]:
        """Wait until at least one status frame arrives."""
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            status = self.latest()

            if status is not None:
                return status

            remaining_s = max(0.0, deadline - time.monotonic())
            self._event.wait(timeout=min(0.1, remaining_s))

        return None

    def wait_until(
        self,
        predicate: StatusPredicate,
        *,
        timeout_s: float,
    ) -> Optional[BoardStatus]:
        """Wait until a status satisfies ``predicate``."""
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            status = self.latest()

            if status is not None and predicate(status):
                return status

            self._event.clear()
            remaining_s = max(0.0, deadline - time.monotonic())
            self._event.wait(timeout=min(0.1, remaining_s))

        return None


def format_board3_status(status: BoardStatus) -> str:
    """Return a compact Board3 status string."""
    return (
        f'state={board3_state_name(status.state)} '
        f'err={error_name(status.error_code)} '
        f'ready={status.homing_done_bits} '
        f'staging={status.board3_staging_count} '
        f'fault={status.limit_status_bits} '
        f'free={status.board3_buffer_free} '
        f'enabled={int(status.enabled)} '
        f'fault_id={status.board3_fault_motor_id}'
    )


def format_board3_positions(positions_rad: tuple[float, ...]) -> str:
    """Return a compact Board3 position string in degrees."""
    degrees = [
        f'{math.degrees(value):.2f}'
        for value in positions_rad
    ]
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
    }
    return names.get(int(value), f'UNKNOWN({value})')


def error_name(value: int) -> str:
    """Return known error name, or the raw value."""
    return error_name_for_board(value, BOARD_ID_BOARD3)


def is_board3_ready(status: BoardStatus) -> bool:
    """Return whether Board3 is ready to accept gripper command frames."""
    return (
        status.state == BoardState.IDLE
        and status.error_code == BoardError.NONE
        and status.homing_done_bits == 1
        and status.limit_status_bits == 0
        and status.enabled
    )


def is_board3_command_complete(status: BoardStatus) -> bool:
    """Return whether the Board3 staging buffer is idle and empty."""
    return (
        is_board3_ready(status)
        and status.board3_staging_count == 0
        and status.board3_buffer_free == BOARD3_SERVO_COUNT
    )


def send_frame(transport: SocketCanTransport, frame: CanFrame) -> None:
    """Send a frame and print it in candump-like form."""
    transport.send_frame(frame)
    print(f'TX {frame.can_id:03X}#{frame.data.hex().upper()}')


def send_gripper_set(
    transport: SocketCanTransport,
    *,
    target_001deg: int,
    duration_ticks: int,
    target_load: int,
) -> None:
    """Send one complete nine-servo Board3 command set."""
    for motor_id in range(BOARD3_SERVO_COUNT):
        frame = pack_board3_servo_command(
            motor_id=motor_id,
            target_pos=target_001deg,
            target_load=target_load,
            duration_ticks=duration_ticks,
        )
        send_frame(transport, frame)


def parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Board3-only CAN smoke test using central protocol code.',
    )
    parser.add_argument(
        '--interface',
        default='can0',
        help='SocketCAN interface name, default: can0',
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
        '--speed',
        dest='target_load',
        type=int,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Wait timeout in seconds, default: 5.0',
    )
    parser.add_argument(
        '--status-only',
        action='store_true',
        help='Only listen for Board3 status; do not send commands',
    )
    parser.add_argument(
        '--clear-error',
        action='store_true',
        help='Send Board3 clear-error before enable',
    )
    parser.add_argument(
        '--skip-enable',
        action='store_true',
        help='Do not send enable before the command set',
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    """Run the Board3-only CAN smoke test."""
    args = parse_args(argv)
    target_001deg = int(round(args.degrees * 100.0))

    monitor = Board3StatusMonitor(verbose=True)
    transport = SocketCanTransport(
        args.interface,
        receive_ids=(
            CAN_ID_BOARD3_STATUS,
            CAN_ID_BOARD3_POSITION_FEEDBACK,
        ),
        frame_callback=monitor.handle_frame,
        error_callback=lambda exc: print(f'RX error: {exc}', file=sys.stderr),
    )

    print(f'Opening {args.interface} for Board3-only test')

    try:
        transport.open()

        initial_status = monitor.wait_for_status(args.timeout)

        if initial_status is None:
            print('No 0x203 status received before timeout')
            return 2

        if args.status_only:
            print('Status-only check passed')
            return 0

        if args.clear_error:
            send_frame(
                transport,
                pack_clear_error(board_id=BOARD_ID_BOARD3),
            )
            time.sleep(0.05)

        if not args.skip_enable:
            send_frame(
                transport,
                pack_enable(True, board_id=BOARD_ID_BOARD3),
            )

        ready_status = monitor.wait_until(
            is_board3_ready,
            timeout_s=args.timeout,
        )

        if ready_status is None:
            latest = monitor.latest()
            print('Board3 did not become ready before timeout')

            if latest is not None:
                print(f'Latest: {format_board3_status(latest)}')

            return 3

        print(f'Board3 ready: {format_board3_status(ready_status)}')
        print(
            'Sending 9 servo frames: '
            f'target={target_001deg} x0.01deg, '
            f'target_load={int(args.target_load)}, '
            f'duration={args.duration_ticks} ticks'
        )

        send_gripper_set(
            transport,
            target_001deg=target_001deg,
            duration_ticks=int(args.duration_ticks),
            target_load=int(args.target_load),
        )

        complete_status = monitor.wait_until(
            is_board3_command_complete,
            timeout_s=args.timeout,
        )

        if complete_status is None:
            latest = monitor.latest()
            print('Board3 command did not complete before timeout')

            if latest is not None:
                print(f'Latest: {format_board3_status(latest)}')

            return 4

        print(f'Board3 command complete: {format_board3_status(complete_status)}')
        return 0

    finally:
        transport.close()


if __name__ == '__main__':
    raise SystemExit(main())
