"""Track Board1/2/3 status and queue capacity safely."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Mapping, Optional

from .can_protocol import (
    BOARD2_REQUIRED_HOMING_MASK,
    BOARD3_SERVO_COUNT,
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD2,
    BOARD_ID_BOARD3,
    BoardError,
    BoardState as BoardStateCode,
    BoardStatus,
    QUEUE_CAPACITY,
    REQUIRED_HOMING_MASK,
    validate_board_id,
)


@dataclass(frozen=True)
class BoardRuntimeConfig:
    """Runtime readiness settings for one STM32 board."""

    board_id: int
    queue_capacity: int = QUEUE_CAPACITY
    required_homing_mask: int = REQUIRED_HOMING_MASK
    requires_homing: bool = True
    requires_ready: bool = False
    requires_fault_clear: bool = False


@dataclass(frozen=True)
class BoardStateSnapshot:
    """Immutable view of one board runtime state."""

    board_id: int
    status: Optional[BoardStatus]
    status_age_ms: Optional[float]
    status_stale: bool
    local_queue_free: int
    commanded_position_valid: bool


@dataclass(frozen=True)
class MultiBoardStateSnapshot:
    """Immutable view of all configured boards."""

    boards: Mapping[int, BoardStateSnapshot]


def default_board_config(board_id: int) -> BoardRuntimeConfig:
    """Return the default runtime config for a known board id."""
    normalized = validate_board_id(board_id)

    if normalized == BOARD_ID_BOARD1:
        return BoardRuntimeConfig(
            board_id=normalized,
            queue_capacity=1,
            required_homing_mask=REQUIRED_HOMING_MASK,
            requires_homing=True,
            requires_ready=False,
            requires_fault_clear=False,
        )
    if normalized == BOARD_ID_BOARD2:
        return BoardRuntimeConfig(
            board_id=normalized,
            queue_capacity=QUEUE_CAPACITY,
            required_homing_mask=BOARD2_REQUIRED_HOMING_MASK,
            requires_homing=True,
            requires_ready=False,
            requires_fault_clear=False,
        )
    return BoardRuntimeConfig(
        board_id=normalized,
        queue_capacity=BOARD3_SERVO_COUNT,
        required_homing_mask=0x01,
        requires_homing=False,
        requires_ready=True,
        requires_fault_clear=True,
    )


class BoardStateTracker:
    """
    Maintain status for one STM32 board.

    The STM32 reports status every 100 ms. This class stores the most recent
    report, detects stale communication, and maintains a conservative local
    queue credit. The local credit is reduced immediately when the host sends
    trajectory frames, rather than waiting for the next status report.
    """

    def __init__(
        self,
        *,
        board_id: int = BOARD_ID_BOARD1,
        status_timeout_ms: int = 500,
        queue_capacity: int = QUEUE_CAPACITY,
        required_homing_mask: int = REQUIRED_HOMING_MASK,
        requires_homing: bool = True,
        requires_ready: bool = False,
        requires_fault_clear: bool = False,
    ) -> None:
        """Initialize the tracker configuration and empty state."""
        if status_timeout_ms <= 0:
            raise ValueError('status_timeout_ms must be greater than zero')
        if not 1 <= int(queue_capacity) <= 0xFF:
            raise ValueError('queue_capacity must be in range 1..255')
        if not 0 <= int(required_homing_mask) <= 0xFF:
            raise ValueError('required_homing_mask must fit uint8')

        self._board_id = validate_board_id(board_id)
        self._status_timeout_s = status_timeout_ms / 1000.0
        self._queue_capacity = int(queue_capacity)
        self._required_homing_mask = int(required_homing_mask)
        self._requires_homing = bool(requires_homing)
        self._requires_ready = bool(requires_ready)
        self._requires_fault_clear = bool(requires_fault_clear)

        self._status: Optional[BoardStatus] = None
        self._status_received_at: Optional[float] = None
        self._local_queue_free = 0
        self._commanded_position_valid = False
        self._lock = threading.RLock()

    @property
    def board_id(self) -> int:
        """Return this tracker board id."""
        return self._board_id

    @property
    def queue_capacity(self) -> int:
        """Return the configured total queue slots."""
        return self._queue_capacity

    @property
    def required_homing_mask(self) -> int:
        """Return the bit mask required before trajectory execution."""
        return self._required_homing_mask

    def update_status(
        self,
        status: BoardStatus,
        *,
        received_at: Optional[float] = None,
    ) -> None:
        """Store a newly decoded board status report."""
        if status.board_id != self._board_id:
            raise ValueError(
                f'Status board_id {status.board_id} does not match '
                f'tracker board_id {self._board_id}'
            )
        if not 0 <= status.queue_free <= self._queue_capacity:
            raise ValueError(
                'Board queue_free is outside the configured range: '
                f'{status.queue_free} not in 0..{self._queue_capacity}'
            )

        timestamp = time.monotonic() if received_at is None else received_at

        with self._lock:
            self._status = status
            self._status_received_at = float(timestamp)
            self._local_queue_free = int(status.queue_free)

            if (
                status.state in (BoardStateCode.ERROR, BoardStateCode.ESTOP)
                or status.error_code != BoardError.NONE
                or not status.enabled
                or not self._readiness_ok(status)
            ):
                self._commanded_position_valid = False

    def reset(self) -> None:
        """Forget all status after transport shutdown or reconnection."""
        with self._lock:
            self._status = None
            self._status_received_at = None
            self._local_queue_free = 0
            self._commanded_position_valid = False

    def mark_commanded_position_valid(self) -> None:
        """Mark the host-side position estimate valid after homing."""
        with self._lock:
            self._commanded_position_valid = True

    def invalidate_commanded_position(self) -> None:
        """Invalidate the host estimate after disable, ESTOP, or reset."""
        with self._lock:
            self._commanded_position_valid = False

    def has_status(self) -> bool:
        """Return whether at least one valid status report was received."""
        with self._lock:
            return self._status is not None

    def latest_status(self) -> Optional[BoardStatus]:
        """Return the latest immutable BoardStatus, or ``None``."""
        with self._lock:
            return self._status

    def status_age_ms(self, *, now: Optional[float] = None) -> Optional[float]:
        """Return milliseconds since the latest report, or ``None``."""
        current_time = time.monotonic() if now is None else now

        with self._lock:
            if self._status_received_at is None:
                return None

            return max(
                0.0,
                (float(current_time) - self._status_received_at) * 1000.0,
            )

    def is_status_stale(self, *, now: Optional[float] = None) -> bool:
        """Return true if no recent status report is available."""
        current_time = time.monotonic() if now is None else now

        with self._lock:
            if self._status_received_at is None:
                return True

            return (
                float(current_time) - self._status_received_at
            ) > self._status_timeout_s

    def commanded_position_valid(self) -> bool:
        """Return whether the host open-loop position estimate is usable."""
        with self._lock:
            return self._commanded_position_valid

    def has_error(self) -> bool:
        """Return whether this board reports ERROR, ESTOP, or nonzero error."""
        with self._lock:
            if self._status is None:
                return True

            return (
                self._status.error_code != BoardError.NONE
                or self._status.state in (
                    BoardStateCode.ERROR,
                    BoardStateCode.ESTOP,
                )
                or (
                    self._requires_fault_clear
                    and self._status.limit_status_bits != 0
                )
            )

    def is_estop(self) -> bool:
        """Return whether the latest status reports ESTOP."""
        with self._lock:
            return (
                self._status is not None
                and self._status.state == BoardStateCode.ESTOP
            )

    def is_enabled(self) -> bool:
        """Return whether the motor or servo enable is active."""
        with self._lock:
            return bool(self._status and self._status.enabled)

    def all_axes_homed(self) -> bool:
        """Return whether this board's readiness bits are present."""
        with self._lock:
            return bool(
                self._status and self._readiness_ok(self._status)
            )

    def available_queue_slots(self) -> int:
        """Return Board1 slot state or Board2/3 legacy queue credit."""
        with self._lock:
            return self._local_queue_free

    def can_accept_new_trajectory(
        self,
        *,
        now: Optional[float] = None,
    ) -> bool:
        """Return whether a new FollowJointTrajectory goal may start."""
        with self._lock:
            status = self._status

            if status is None or self.is_status_stale(now=now):
                return False

            allowed_states = (
                (BoardStateCode.IDLE, BoardStateCode.CONTACT_HOLD)
                if self._board_id == BOARD_ID_BOARD3
                else (BoardStateCode.IDLE,)
            )

            return (
                status.state in allowed_states
                and status.error_code == BoardError.NONE
                and status.enabled
                and self._readiness_ok(status)
                and self._commanded_position_valid
                and (
                    status.goal_slot_free == self._queue_capacity
                    if self._board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2)
                    else True
                )
            )

    def can_stream_slots(
        self,
        required_slots: int,
        *,
        max_in_flight_slots: Optional[int] = None,
        now: Optional[float] = None,
    ) -> bool:
        """Return whether trajectory frames can be streamed now."""
        if required_slots <= 0:
            raise ValueError('required_slots must be greater than zero')
        if max_in_flight_slots is not None:
            if max_in_flight_slots < required_slots:
                raise ValueError(
                    'max_in_flight_slots must be >= required_slots'
                )

        with self._lock:
            status = self._status

            if status is None or self.is_status_stale(now=now):
                return False

            allowed_states = (
                (BoardStateCode.IDLE, BoardStateCode.CONTACT_HOLD)
                if self._board_id == BOARD_ID_BOARD3
                else (BoardStateCode.IDLE, BoardStateCode.MOVING)
            )

            board_can_move = (
                status.state in allowed_states
                and status.error_code == BoardError.NONE
                and status.enabled
                and self._readiness_ok(status)
                and self._commanded_position_valid
            )

            return (
                board_can_move
                and self._local_queue_free >= required_slots
                and (
                    max_in_flight_slots is None
                    or (
                        self._queue_capacity
                        - self._local_queue_free
                        + required_slots
                    )
                    <= max_in_flight_slots
                )
            )

    def reserve_queue_slots(
        self,
        required_slots: int,
        *,
        max_in_flight_slots: Optional[int] = None,
        now: Optional[float] = None,
    ) -> bool:
        """Atomically reserve local queue credit before sending frames."""
        if required_slots <= 0:
            raise ValueError('required_slots must be greater than zero')

        with self._lock:
            if not self.can_stream_slots(
                required_slots,
                max_in_flight_slots=max_in_flight_slots,
                now=now,
            ):
                return False

            self._local_queue_free -= required_slots
            return True

    def refund_queue_slots(self, slots: int) -> None:
        """Return locally reserved credit if no CAN frame was transmitted."""
        if slots <= 0:
            raise ValueError('slots must be greater than zero')

        with self._lock:
            reported_free = (
                self._status.queue_free
                if self._status is not None
                else self._queue_capacity
            )
            limit = min(self._queue_capacity, reported_free)
            self._local_queue_free = min(
                limit,
                self._local_queue_free + slots,
            )

    def is_trajectory_complete(
        self,
        *,
        now: Optional[float] = None,
    ) -> bool:
        """Return whether STM32 reports an empty, idle trajectory queue."""
        with self._lock:
            status = self._status

            if status is None or self.is_status_stale(now=now):
                return False

            done_states = (
                (BoardStateCode.IDLE, BoardStateCode.CONTACT_HOLD)
                if self._board_id == BOARD_ID_BOARD3
                else (BoardStateCode.IDLE,)
            )

            return (
                status.state in done_states
                and status.error_code == BoardError.NONE
                and (
                    status.board3_staging_count == 0
                    if self._board_id == BOARD_ID_BOARD3
                    else (
                        status.moving_mask == 0
                        and status.target_reached_mask
                        == self._required_homing_mask
                    )
                )
                and status.queue_free == self._queue_capacity
                and status.enabled
                and self._readiness_ok(status)
                and not (
                    self._requires_fault_clear
                    and status.limit_status_bits != 0
                )
            )

    def snapshot(
        self,
        *,
        now: Optional[float] = None,
    ) -> BoardStateSnapshot:
        """Return one consistent immutable state snapshot."""
        current_time = time.monotonic() if now is None else float(now)

        with self._lock:
            if self._status_received_at is None:
                age_ms = None
                stale = True
            else:
                age_s = max(
                    0.0,
                    current_time - self._status_received_at,
                )
                age_ms = age_s * 1000.0
                stale = age_s > self._status_timeout_s

            return BoardStateSnapshot(
                board_id=self._board_id,
                status=self._status,
                status_age_ms=age_ms,
                status_stale=stale,
                local_queue_free=self._local_queue_free,
                commanded_position_valid=(
                    self._commanded_position_valid
                ),
            )

    def _readiness_ok(self, status: BoardStatus) -> bool:
        if self._requires_homing:
            return (
                status.position_valid_mask & self._required_homing_mask
            ) == self._required_homing_mask

        if self._requires_ready:
            return status.homing_done_bits == 1

        return True


class MultiBoardStateTracker:
    """Coordinate readiness and queue credits across multiple boards."""

    def __init__(
        self,
        *,
        board_ids: list[int] | tuple[int, ...],
        status_timeout_ms: int = 500,
        queue_capacities: Optional[Mapping[int, int]] = None,
        required_homing_masks: Optional[Mapping[int, int]] = None,
    ) -> None:
        if not board_ids:
            raise ValueError('board_ids cannot be empty')

        unique_board_ids = tuple(
            dict.fromkeys(validate_board_id(board_id) for board_id in board_ids)
        )

        self._trackers: dict[int, BoardStateTracker] = {}

        for board_id in unique_board_ids:
            config = default_board_config(board_id)
            queue_capacity = (
                int(queue_capacities[board_id])
                if queue_capacities and board_id in queue_capacities
                else config.queue_capacity
            )
            required_homing_mask = (
                int(required_homing_masks[board_id])
                if (
                    required_homing_masks
                    and board_id in required_homing_masks
                )
                else config.required_homing_mask
            )

            self._trackers[board_id] = BoardStateTracker(
                board_id=board_id,
                status_timeout_ms=status_timeout_ms,
                queue_capacity=queue_capacity,
                required_homing_mask=required_homing_mask,
                requires_homing=config.requires_homing,
                requires_ready=config.requires_ready,
                requires_fault_clear=config.requires_fault_clear,
            )

    @property
    def board_ids(self) -> tuple[int, ...]:
        """Return configured board ids."""
        return tuple(self._trackers.keys())

    def tracker_for(self, board_id: int) -> BoardStateTracker:
        """Return one configured board tracker."""
        normalized = validate_board_id(board_id)
        return self._trackers[normalized]

    def update_status(
        self,
        status: BoardStatus,
        *,
        received_at: Optional[float] = None,
    ) -> None:
        """Store a board status if that board is configured."""
        tracker = self._trackers.get(status.board_id)
        if tracker is None:
            return
        tracker.update_status(status, received_at=received_at)

    def reset(self) -> None:
        """Forget all board statuses."""
        for tracker in self._trackers.values():
            tracker.reset()

    def mark_commanded_position_valid(self) -> None:
        """Mark all required board estimates valid."""
        for tracker in self._trackers.values():
            tracker.mark_commanded_position_valid()

    def invalidate_commanded_position(self) -> None:
        """Invalidate all required board estimates."""
        for tracker in self._trackers.values():
            tracker.invalidate_commanded_position()

    def has_status(self) -> bool:
        """Return whether every configured board has reported status."""
        return all(tracker.has_status() for tracker in self._trackers.values())

    def is_status_stale(self, *, now: Optional[float] = None) -> bool:
        """Return true if any configured board status is stale."""
        return any(
            tracker.is_status_stale(now=now)
            for tracker in self._trackers.values()
        )

    def latest_statuses(self) -> dict[int, Optional[BoardStatus]]:
        """Return latest statuses by board id."""
        return {
            board_id: tracker.latest_status()
            for board_id, tracker in self._trackers.items()
        }

    def is_enabled(self) -> bool:
        """Return whether every configured board is enabled."""
        return all(tracker.is_enabled() for tracker in self._trackers.values())

    def is_estop(self) -> bool:
        """Return whether any configured board reports ESTOP."""
        return any(tracker.is_estop() for tracker in self._trackers.values())

    def all_axes_homed(self) -> bool:
        """Return whether every configured board is ready for motion."""
        return all(
            tracker.all_axes_homed()
            for tracker in self._trackers.values()
        )

    def has_error(self) -> bool:
        """Return whether any configured board reports an error."""
        return any(tracker.has_error() for tracker in self._trackers.values())

    def can_accept_new_trajectory(
        self,
        *,
        now: Optional[float] = None,
    ) -> bool:
        """Return whether every board can accept a new trajectory."""
        return all(
            tracker.can_accept_new_trajectory(now=now)
            for tracker in self._trackers.values()
        )

    def can_stream_slots(
        self,
        required_slots_by_board: Mapping[int, int],
        *,
        max_in_flight_slots_by_board: Optional[Mapping[int, int]] = None,
        now: Optional[float] = None,
    ) -> bool:
        """Return whether every required board has enough queue credit."""
        for board_id, slots in required_slots_by_board.items():
            if slots <= 0:
                continue
            tracker = self._trackers.get(validate_board_id(board_id))
            if tracker is None:
                return False
            max_in_flight_slots = (
                int(max_in_flight_slots_by_board[board_id])
                if (
                    max_in_flight_slots_by_board
                    and board_id in max_in_flight_slots_by_board
                )
                else None
            )
            if not tracker.can_stream_slots(
                slots,
                max_in_flight_slots=max_in_flight_slots,
                now=now,
            ):
                return False

        return True

    def reserve_queue_slots(
        self,
        required_slots_by_board: Mapping[int, int],
        *,
        max_in_flight_slots_by_board: Optional[Mapping[int, int]] = None,
        now: Optional[float] = None,
    ) -> bool:
        """Reserve queue slots on all boards atomically enough for streaming."""
        reserved: list[tuple[BoardStateTracker, int]] = []

        for board_id, slots in required_slots_by_board.items():
            if slots <= 0:
                continue
            tracker = self._trackers.get(validate_board_id(board_id))
            if tracker is None:
                for previous_tracker, previous_slots in reserved:
                    previous_tracker.refund_queue_slots(previous_slots)
                return False
            max_in_flight_slots = (
                int(max_in_flight_slots_by_board[board_id])
                if (
                    max_in_flight_slots_by_board
                    and board_id in max_in_flight_slots_by_board
                )
                else None
            )
            if not tracker.reserve_queue_slots(
                slots,
                max_in_flight_slots=max_in_flight_slots,
                now=now,
            ):
                for previous_tracker, previous_slots in reserved:
                    previous_tracker.refund_queue_slots(previous_slots)
                return False
            reserved.append((tracker, slots))

        return True

    def refund_queue_slots(
        self,
        slots_by_board: Mapping[int, int],
    ) -> None:
        """Return locally reserved queue credits."""
        for board_id, slots in slots_by_board.items():
            if slots <= 0:
                continue
            tracker = self._trackers.get(validate_board_id(board_id))
            if tracker is not None:
                tracker.refund_queue_slots(slots)

    def is_trajectory_complete(
        self,
        *,
        now: Optional[float] = None,
    ) -> bool:
        """Return whether every configured board is idle and empty."""
        return all(
            tracker.is_trajectory_complete(now=now)
            for tracker in self._trackers.values()
        )

    def snapshot(
        self,
        *,
        now: Optional[float] = None,
    ) -> MultiBoardStateSnapshot:
        """Return one immutable snapshot of all configured boards."""
        return MultiBoardStateSnapshot(
            boards={
                board_id: tracker.snapshot(now=now)
                for board_id, tracker in self._trackers.items()
            }
        )
