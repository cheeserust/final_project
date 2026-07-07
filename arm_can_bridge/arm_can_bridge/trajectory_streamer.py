"""Stream converted trajectory batches to STM32 boards over SocketCAN."""

from __future__ import annotations

import time
from typing import Callable, Iterable, Optional

import rclpy

from .board_state import MultiBoardStateTracker
from .can_protocol import BOARD_ID_ALL
from .can_protocol import DURATION_TICK_NS
from .can_protocol import pack_enable
from .socketcan_transport import SocketCanTransport
from .trajectory_converter import TrajectoryBatch


ProgressCallback = Callable[[int, int, TrajectoryBatch], None]
CancelPredicate = Callable[[], bool]


class TrajectoryStreamingError(RuntimeError):
    """Raised when streaming cannot continue safely."""


class TrajectoryCanceled(RuntimeError):
    """Raised when FollowJointTrajectory goal is canceled."""


class TrajectoryStreamer:
    """Queue-free based CAN streaming for board trajectory batches."""

    def __init__(
        self,
        *,
        board_state: MultiBoardStateTracker,
        transport: SocketCanTransport,
        queue_wait_timeout_ms: int,
        completion_grace_ms: int,
    ) -> None:
        if queue_wait_timeout_ms <= 0:
            raise ValueError('queue_wait_timeout_ms must be positive')
        if completion_grace_ms <= 0:
            raise ValueError('completion_grace_ms must be positive')

        self._board_state = board_state
        self._transport = transport
        self._queue_wait_timeout_s = queue_wait_timeout_ms / 1000.0
        self._completion_grace_s = completion_grace_ms / 1000.0

    def stream(
        self,
        batches: Iterable[TrajectoryBatch],
        *,
        cancel_requested: Optional[CancelPredicate] = None,
        progress_callback: Optional[ProgressCallback] = None,
    ) -> None:
        """Send all batches to the configured boards and wait for completion."""
        batch_list = tuple(batches)

        if not batch_list:
            raise TrajectoryStreamingError('No trajectory batches to stream')

        if not self._board_state.can_accept_new_trajectory():
            raise TrajectoryStreamingError(
                'Arm boards are not ready to accept a new trajectory'
            )

        total = len(batch_list)
        expected_motion_time_s = 0.0
        motion_started_at_s: float | None = None

        for index, batch in enumerate(batch_list):
            self._raise_if_cancelled(cancel_requested)

            self._wait_for_queue_slots(
                required_slots_by_board=batch.queue_slots_by_board,
                cancel_requested=cancel_requested,
            )

            # Reserve before sending, because 0x201 status only arrives
            # periodically and would otherwise overestimate free queue slots.
            if not self._board_state.reserve_queue_slots(
                batch.queue_slots_by_board
            ):
                raise TrajectoryStreamingError(
                    'Queue slots disappeared before sending a batch'
                )

            sent_any = False

            try:
                duration_s = (
                    batch.duration_ticks
                    * DURATION_TICK_NS
                    / 1_000_000_000.0
                )

                for frame in batch.frames:
                    self._raise_if_cancelled(cancel_requested)
                    if motion_started_at_s is None:
                        motion_started_at_s = time.monotonic()
                    self._transport.send_frame(frame)
                    sent_any = True

                expected_motion_time_s += duration_s

            except Exception:
                # Only refund if no frame was actually sent.
                # If partial CAN frames went out, the STM32 state is unknown.
                if not sent_any:
                    self._board_state.refund_queue_slots(
                        batch.queue_slots_by_board
                    )
                raise

            if progress_callback is not None:
                progress_callback(index + 1, total, batch)

        if motion_started_at_s is None:
            raise TrajectoryStreamingError('No CAN frames were sent')

        earliest_completion_s = motion_started_at_s + expected_motion_time_s
        min_wait_s = max(0.0, earliest_completion_s - time.monotonic())

        self._wait_for_completion(
            timeout_s=expected_motion_time_s + self._completion_grace_s,
            min_wait_s=min_wait_s,
            cancel_requested=cancel_requested,
        )

    def stop_by_disable(self) -> None:
        """Stop motion by broadcasting the protocol disable command."""
        self._transport.send_frame(
            pack_enable(False, board_id=BOARD_ID_ALL)
        )

    def _wait_for_queue_slots(
        self,
        *,
        required_slots_by_board: dict[int, int],
        cancel_requested: Optional[CancelPredicate],
    ) -> None:
        deadline = time.monotonic() + self._queue_wait_timeout_s

        while rclpy.ok() and time.monotonic() < deadline:
            self._raise_if_cancelled(cancel_requested)

            if self._board_state.can_stream_slots(required_slots_by_board):
                return

            if self._board_state.has_error():
                raise TrajectoryStreamingError(
                    'A board reported ERROR/ESTOP while waiting for queue'
                )

            time.sleep(0.01)

        raise TrajectoryStreamingError(
            f'Timeout waiting for queue slots {required_slots_by_board}'
        )

    def _wait_for_completion(
        self,
        *,
        timeout_s: float,
        min_wait_s: float,
        cancel_requested: Optional[CancelPredicate],
    ) -> None:
        now = time.monotonic()
        deadline = now + timeout_s
        completion_allowed_at = now + max(0.0, min_wait_s)

        while rclpy.ok() and time.monotonic() < deadline:
            self._raise_if_cancelled(cancel_requested)

            now = time.monotonic()
            if (
                now >= completion_allowed_at
                and self._board_state.is_trajectory_complete()
            ):
                return

            if self._board_state.has_error():
                raise TrajectoryStreamingError(
                    'A board reported ERROR/ESTOP during completion wait'
                )

            time.sleep(0.02)

        raise TrajectoryStreamingError(
            'Timeout waiting for arm board trajectory completion'
        )

    @staticmethod
    def _raise_if_cancelled(
        cancel_requested: Optional[CancelPredicate],
    ) -> None:
        if cancel_requested is not None and cancel_requested():
            raise TrajectoryCanceled('trajectory canceled')
