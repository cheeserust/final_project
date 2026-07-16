"""Stream converted trajectory batches to STM32 boards over SocketCAN."""

from __future__ import annotations

import time
from typing import Callable, Iterable, Optional

import rclpy

from .board_state import MultiBoardStateTracker
from .can_protocol import CAN_ID_BOARD1_POSITION_COMMAND
from .can_protocol import CAN_ID_BOARD2_POSITION_COMMAND
from .can_protocol import CAN_ID_BOARD3_SERVO_COMMAND
from .can_protocol import DURATION_TICK_NS
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
        arm_inter_frame_delay_ms: float = 0.0,
        board3_inter_frame_delay_ms: float = 0.0,
        max_in_flight_batches: int = 0,
    ) -> None:
        if queue_wait_timeout_ms <= 0:
            raise ValueError('queue_wait_timeout_ms must be positive')
        if completion_grace_ms <= 0:
            raise ValueError('completion_grace_ms must be positive')
        if arm_inter_frame_delay_ms < 0.0:
            raise ValueError('arm_inter_frame_delay_ms cannot be negative')
        if board3_inter_frame_delay_ms < 0.0:
            raise ValueError('board3_inter_frame_delay_ms cannot be negative')
        if max_in_flight_batches < 0:
            raise ValueError('max_in_flight_batches cannot be negative')

        self._board_state = board_state
        self._transport = transport
        self._queue_wait_timeout_s = queue_wait_timeout_ms / 1000.0
        self._completion_grace_s = completion_grace_ms / 1000.0
        self._arm_inter_frame_delay_s = (
            float(arm_inter_frame_delay_ms) / 1000.0
        )
        self._board3_inter_frame_delay_s = (
            float(board3_inter_frame_delay_ms) / 1000.0
        )
        self._max_in_flight_batches = int(max_in_flight_batches)

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
            max_in_flight_slots_by_board = (
                self._max_in_flight_slots_by_board(batch)
            )

            self._wait_for_queue_slots(
                required_slots_by_board=batch.queue_slots_by_board,
                max_in_flight_slots_by_board=max_in_flight_slots_by_board,
                cancel_requested=cancel_requested,
            )

            # Reserve before sending, because 0x201 status only arrives
            # periodically and would otherwise overestimate free queue slots.
            if not self._board_state.reserve_queue_slots(
                batch.queue_slots_by_board,
                max_in_flight_slots_by_board=max_in_flight_slots_by_board,
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

                for frame_index, frame in enumerate(batch.frames):
                    self._raise_if_cancelled(cancel_requested)
                    if motion_started_at_s is None:
                        motion_started_at_s = time.monotonic()
                    self._transport.send_frame(frame)
                    sent_any = True
                    self._sleep_after_frame(batch, frame_index)

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

            if index < total - 1:
                self._sleep_between_batches(batch, batch_list[index + 1])

        if motion_started_at_s is None:
            raise TrajectoryStreamingError('No CAN frames were sent')

        earliest_completion_s = motion_started_at_s + expected_motion_time_s
        min_wait_s = max(0.0, earliest_completion_s - time.monotonic())

        self._wait_for_completion(
            timeout_s=expected_motion_time_s + self._completion_grace_s,
            min_wait_s=min_wait_s,
            cancel_requested=cancel_requested,
        )

    def _sleep_after_frame(
        self,
        batch: TrajectoryBatch,
        frame_index: int,
    ) -> None:
        if frame_index >= len(batch.frames) - 1:
            return

        current_can_id = batch.frames[frame_index].can_id
        next_can_id = batch.frames[frame_index + 1].can_id

        arm_command_ids = {
            CAN_ID_BOARD1_POSITION_COMMAND,
            CAN_ID_BOARD2_POSITION_COMMAND,
        }

        if (
            self._arm_inter_frame_delay_s > 0.0
            and current_can_id in arm_command_ids
            and next_can_id in arm_command_ids
        ):
            time.sleep(self._arm_inter_frame_delay_s)
            return

        if (
            self._board3_inter_frame_delay_s > 0.0
            and current_can_id == CAN_ID_BOARD3_SERVO_COMMAND
            and next_can_id == CAN_ID_BOARD3_SERVO_COMMAND
        ):
            time.sleep(self._board3_inter_frame_delay_s)
            return

    def _sleep_between_batches(
        self,
        current_batch: TrajectoryBatch,
        next_batch: TrajectoryBatch,
    ) -> None:
        if not current_batch.frames or not next_batch.frames:
            return

        current_can_id = current_batch.frames[-1].can_id
        next_can_id = next_batch.frames[0].can_id

        arm_command_ids = {
            CAN_ID_BOARD1_POSITION_COMMAND,
            CAN_ID_BOARD2_POSITION_COMMAND,
        }

        if (
            self._arm_inter_frame_delay_s > 0.0
            and current_can_id in arm_command_ids
            and next_can_id in arm_command_ids
        ):
            time.sleep(self._arm_inter_frame_delay_s)
            return

        if (
            self._board3_inter_frame_delay_s > 0.0
            and current_can_id == CAN_ID_BOARD3_SERVO_COMMAND
            and next_can_id == CAN_ID_BOARD3_SERVO_COMMAND
        ):
            time.sleep(self._board3_inter_frame_delay_s)
            return

    def _max_in_flight_slots_by_board(
        self,
        batch: TrajectoryBatch,
    ) -> dict[int, int] | None:
        if self._max_in_flight_batches <= 0:
            return None

        return {
            board_id: slots * self._max_in_flight_batches
            for board_id, slots in batch.queue_slots_by_board.items()
        }

    def _wait_for_queue_slots(
        self,
        *,
        required_slots_by_board: dict[int, int],
        max_in_flight_slots_by_board: Optional[dict[int, int]],
        cancel_requested: Optional[CancelPredicate],
    ) -> None:
        deadline = time.monotonic() + self._queue_wait_timeout_s

        while rclpy.ok() and time.monotonic() < deadline:
            self._raise_if_cancelled(cancel_requested)

            if self._board_state.can_stream_slots(
                required_slots_by_board,
                max_in_flight_slots_by_board=max_in_flight_slots_by_board,
            ):
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
