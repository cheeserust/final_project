"""Send and receive Board1 frames through Linux SocketCAN."""

from __future__ import annotations

from pathlib import Path
import socket
import struct
import threading
from typing import Callable, Iterable, Optional

from .can_protocol import CanFrame


CAN_FRAME_FORMAT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF

SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
CAN_RAW_FILTER = getattr(socket, 'CAN_RAW_FILTER', 1)

FrameCallback = Callable[[CanFrame], None]
ErrorCallback = Callable[[Exception], None]


class SocketCanTransportError(RuntimeError):
    """Raised when a Linux SocketCAN operation fails."""


class UnsupportedSocketCanFrame(ValueError):
    """Raised for extended, RTR, or error frames not used by Board1."""


def encode_socketcan_frame(frame: CanFrame) -> bytes:
    """Encode one protocol frame into Linux ``struct can_frame`` bytes."""
    payload = frame.data.ljust(8, b'\x00')

    return struct.pack(
        CAN_FRAME_FORMAT,
        int(frame.can_id),
        len(frame.data),
        payload,
    )


def decode_socketcan_frame(raw_frame: bytes) -> CanFrame:
    """Decode Linux ``struct can_frame`` bytes into one protocol frame."""
    if len(raw_frame) != CAN_FRAME_SIZE:
        raise ValueError(
            f'Linux CAN frame must contain {CAN_FRAME_SIZE} bytes, '
            f'got {len(raw_frame)}'
        )

    raw_can_id, data_length, payload = struct.unpack(
        CAN_FRAME_FORMAT,
        raw_frame,
    )

    if data_length > 8:
        raise ValueError(
            f'Classic CAN data length must be 0..8, got {data_length}'
        )

    unsupported_flags = raw_can_id & (
        CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG
    )

    if unsupported_flags:
        raise UnsupportedSocketCanFrame(
            f'Unsupported SocketCAN flags: {unsupported_flags:#x}'
        )

    can_id = raw_can_id & CAN_SFF_MASK
    return CanFrame(can_id=can_id, data=payload[:data_length])


def build_socketcan_filter_data(
    receive_ids: Iterable[int],
) -> bytes:
    """Build exact-match filters for standard 11-bit CAN IDs."""
    normalized_ids = tuple(dict.fromkeys(int(value) for value in receive_ids))
    mask = CAN_SFF_MASK
    filter_data = bytearray()

    for can_id in normalized_ids:
        if not 0 <= can_id <= CAN_SFF_MASK:
            raise ValueError(
                f'Receive filter ID must be an 11-bit ID: {can_id:#x}'
            )

        filter_data.extend(struct.pack('=II', can_id, mask))

    return bytes(filter_data)


class SocketCanTransport:
    """Threaded transport for standard 11-bit Classic CAN frames."""

    def __init__(
        self,
        interface_name: str,
        *,
        receive_ids: Iterable[int] = (),
        receive_timeout_s: float = 0.1,
        frame_callback: Optional[FrameCallback] = None,
        error_callback: Optional[ErrorCallback] = None,
    ) -> None:
        """Store transport configuration without opening the socket."""
        if not interface_name:
            raise ValueError('interface_name cannot be empty')
        if receive_timeout_s <= 0.0:
            raise ValueError('receive_timeout_s must be greater than zero')

        self._interface_name = str(interface_name)
        self._receive_ids = tuple(
            dict.fromkeys(int(value) for value in receive_ids)
        )
        self._receive_timeout_s = float(receive_timeout_s)
        self._frame_callback = frame_callback
        self._error_callback = error_callback

        self._socket: Optional[socket.socket] = None
        self._receive_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._state_lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._sent_frames = 0
        self._received_frames = 0
        self._send_errors = 0
        self._receive_errors = 0
        self._short_writes = 0

    @property
    def interface_name(self) -> str:
        """Return the configured Linux CAN network interface name."""
        return self._interface_name

    def set_frame_callback(
        self,
        callback: Optional[FrameCallback],
    ) -> None:
        """Replace the callback invoked for every accepted CAN frame."""
        with self._state_lock:
            self._frame_callback = callback

    def set_error_callback(
        self,
        callback: Optional[ErrorCallback],
    ) -> None:
        """Replace the asynchronous transport-error callback."""
        with self._state_lock:
            self._error_callback = callback

    def is_open(self) -> bool:
        """Return whether the SocketCAN socket is currently open."""
        with self._state_lock:
            return self._socket is not None

    def diagnostics(self) -> dict[str, object]:
        """Return application counters plus Linux interface statistics."""
        with self._state_lock:
            result: dict[str, object] = {
                'interface': self._interface_name,
                'open': self._socket is not None,
                'sent_frames': self._sent_frames,
                'received_frames': self._received_frames,
                'send_errors': self._send_errors,
                'receive_errors': self._receive_errors,
                'short_writes': self._short_writes,
            }
        interface_path = Path('/sys/class/net') / self._interface_name
        try:
            result['operstate'] = (
                interface_path / 'operstate'
            ).read_text(encoding='utf-8').strip()
        except OSError:
            result['operstate'] = 'unavailable'
        for name in (
            'tx_errors',
            'rx_errors',
            'tx_dropped',
            'rx_dropped',
        ):
            try:
                result[name] = int(
                    (interface_path / 'statistics' / name)
                    .read_text(encoding='utf-8')
                    .strip()
                )
            except (OSError, ValueError):
                result[name] = None
        return result

    def open(self) -> None:  # noqa: A003
        """Open the CAN RAW socket and start its receiver thread."""
        with self._state_lock:
            if self._socket is not None:
                return

            if not hasattr(socket, 'AF_CAN') or not hasattr(socket, 'CAN_RAW'):
                raise SocketCanTransportError(
                    'This Python/Linux environment does not support AF_CAN'
                )

            can_socket: Optional[socket.socket] = None

            try:
                can_socket = socket.socket(
                    socket.AF_CAN,
                    socket.SOCK_RAW,
                    socket.CAN_RAW,
                )
                if self._receive_ids:
                    filter_data = build_socketcan_filter_data(
                        self._receive_ids
                    )
                    can_socket.setsockopt(
                        SOL_CAN_RAW,
                        CAN_RAW_FILTER,
                        filter_data,
                    )
                can_socket.settimeout(self._receive_timeout_s)
                can_socket.bind((self._interface_name,))
            except OSError as exc:
                if can_socket is not None:
                    can_socket.close()

                raise SocketCanTransportError(
                    f'Failed to open SocketCAN interface '
                    f'"{self._interface_name}": {exc}'
                ) from exc

            self._socket = can_socket
            self._stop_event.clear()
            self._receive_thread = threading.Thread(
                target=self._receive_loop,
                name=f'socketcan-rx-{self._interface_name}',
                daemon=True,
            )
            self._receive_thread.start()

    def close(self) -> None:
        """Stop receiving and close the CAN RAW socket safely."""
        with self._state_lock:
            self._stop_event.set()
            can_socket = self._socket
            receive_thread = self._receive_thread
            self._socket = None
            self._receive_thread = None

        if can_socket is not None:
            try:
                can_socket.close()
            except OSError:
                pass

        if (
            receive_thread is not None
            and receive_thread is not threading.current_thread()
        ):
            receive_thread.join(timeout=1.0)

    def send_frame(self, frame: CanFrame) -> None:
        """Send one protocol frame through the opened CAN interface."""
        encoded = encode_socketcan_frame(frame)

        with self._state_lock:
            can_socket = self._socket

        if can_socket is None:
            raise SocketCanTransportError(
                'SocketCAN transport is not open'
            )

        try:
            with self._send_lock:
                sent_bytes = can_socket.send(encoded)
        except OSError as exc:
            with self._state_lock:
                self._send_errors += 1
            raise SocketCanTransportError(
                f'Failed to send CAN ID {frame.can_id:#x}: {exc}'
            ) from exc

        if sent_bytes != CAN_FRAME_SIZE:
            with self._state_lock:
                self._short_writes += 1
            raise SocketCanTransportError(
                f'Partial SocketCAN write: '
                f'{sent_bytes}/{CAN_FRAME_SIZE} bytes'
            )
        with self._state_lock:
            self._sent_frames += 1

    def __enter__(self) -> 'SocketCanTransport':
        """Open the transport for use in a context manager."""
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """Close the context-managed transport."""
        self.close()

    def _receive_loop(self) -> None:
        """Receive Linux CAN frames until ``close`` requests shutdown."""
        while not self._stop_event.is_set():
            with self._state_lock:
                can_socket = self._socket

            if can_socket is None:
                return

            try:
                raw_frame = can_socket.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop_event.is_set():
                    with self._state_lock:
                        self._receive_errors += 1
                    self._report_error(
                        SocketCanTransportError(
                            f'SocketCAN receive failed: {exc}'
                        )
                    )
                return

            try:
                frame = decode_socketcan_frame(raw_frame)
            except UnsupportedSocketCanFrame:
                continue
            except Exception as exc:
                with self._state_lock:
                    self._receive_errors += 1
                self._report_error(exc)
                continue

            if self._receive_ids and frame.can_id not in self._receive_ids:
                continue

            with self._state_lock:
                self._received_frames += 1
            self._dispatch_frame(frame)

    def _dispatch_frame(self, frame: CanFrame) -> None:
        """Invoke the current receive callback without killing the thread."""
        with self._state_lock:
            callback = self._frame_callback

        if callback is None:
            return

        try:
            callback(frame)
        except Exception as exc:
            self._report_error(exc)

    def _report_error(self, error: Exception) -> None:
        """Report asynchronous receive or callback failures."""
        with self._state_lock:
            callback = self._error_callback

        if callback is not None:
            try:
                callback(error)
            except Exception:
                pass
