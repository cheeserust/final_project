"""Assemble Board3 compressed actual-position feedback groups."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from .can_protocol import (
    BOARD3_SERVO_COUNT,
    Board3PositionFeedbackGroup,
)


@dataclass(frozen=True)
class Board3PositionFeedbackSnapshot:
    """One complete nine-servo Board3 actual-position snapshot."""

    positions_rad: tuple[float, ...]
    status_codes: tuple[int, ...]
    raw_flags: tuple[int, ...]
    fault_groups: tuple[int, ...]
    received_at: float


class Board3PositionFeedbackAssembler:
    """Collect three 0x303 groups into one coherent Board3 snapshot."""

    def __init__(self, *, max_cycle_age_s: float = 0.05) -> None:
        if max_cycle_age_s <= 0.0:
            raise ValueError('max_cycle_age_s must be greater than zero')

        self._max_cycle_age_s = float(max_cycle_age_s)
        self._lock = threading.RLock()
        self._clear_locked()

    def reset(self) -> None:
        """Forget any partially collected feedback groups."""
        with self._lock:
            self._clear_locked()

    def update(
        self,
        group: Board3PositionFeedbackGroup,
        *,
        received_at: float | None = None,
    ) -> Board3PositionFeedbackSnapshot | None:
        """Store one group and return a snapshot once all groups are present."""
        if not group.valid:
            return None

        timestamp = (
            time.monotonic()
            if received_at is None
            else float(received_at)
        )

        with self._lock:
            if self._should_start_new_cycle(group, timestamp):
                self._clear_locked()
                self._cycle_start_s = timestamp

            for motor_id, position, status in zip(
                group.motor_ids,
                group.positions_rad,
                group.status_codes,
            ):
                self._positions_rad[motor_id] = position
                self._status_codes[motor_id] = status

            self._groups_seen.add(group.group_index)
            self._raw_flags[group.group_index - 1] = group.raw_flags

            if group.fault:
                self._fault_groups.add(group.group_index)

            if self._groups_seen != {1, 2, 3}:
                return None

            if any(value is None for value in self._positions_rad):
                return None
            if any(value is None for value in self._status_codes):
                return None
            if any(value is None for value in self._raw_flags):
                return None

            snapshot = Board3PositionFeedbackSnapshot(
                positions_rad=tuple(
                    float(value)
                    for value in self._positions_rad
                    if value is not None
                ),
                status_codes=tuple(
                    int(value)
                    for value in self._status_codes
                    if value is not None
                ),
                raw_flags=tuple(
                    int(value)
                    for value in self._raw_flags
                    if value is not None
                ),
                fault_groups=tuple(sorted(self._fault_groups)),
                received_at=timestamp,
            )
            self._clear_locked()
            return snapshot

    def _should_start_new_cycle(
        self,
        group: Board3PositionFeedbackGroup,
        timestamp: float,
    ) -> bool:
        if group.group_index == 1:
            return True
        if self._cycle_start_s is None:
            return True
        return (
            timestamp - self._cycle_start_s
        ) > self._max_cycle_age_s

    def _clear_locked(self) -> None:
        self._positions_rad: list[float | None] = [
            None
            for _ in range(BOARD3_SERVO_COUNT)
        ]
        self._status_codes: list[int | None] = [
            None
            for _ in range(BOARD3_SERVO_COUNT)
        ]
        self._raw_flags: list[int | None] = [None, None, None]
        self._groups_seen: set[int] = set()
        self._fault_groups: set[int] = set()
        self._cycle_start_s: float | None = None
