"""Host-side commanded joint position estimate for open-loop steppers."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Sequence

from sensor_msgs.msg import JointState

from .can_protocol import DURATION_TICK_NS


@dataclass(frozen=True)
class ScheduledSegment:
    """One time-bounded commanded trajectory segment."""

    start_s: float
    end_s: float
    from_positions: tuple[float, ...]
    to_positions: tuple[float, ...]


class CommandedStateEstimator:
    """
    Keep the best available host-side joint position for one controller.

    For boards without actual feedback, it mirrors scheduled commands and
    keeps the last commanded target after the trajectory time elapses.
    Actual feedback handlers may overwrite positions when hardware reports
    measured positions.
    """

    def __init__(
        self,
        joint_names: Sequence[str],
    ) -> None:
        if not joint_names:
            raise ValueError('joint_names cannot be empty')

        if len(set(joint_names)) != len(joint_names):
            raise ValueError('joint_names must be unique')

        self._joint_names = tuple(str(name) for name in joint_names)
        self._positions = [0.0 for _ in self._joint_names]
        self._valid = False
        self._lock = threading.RLock()

        self._trajectory_start_time = 0.0
        self._segments: tuple[ScheduledSegment, ...] = ()

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return configured joint names."""
        return self._joint_names

    def reset_invalid(self) -> None:
        """Invalidate position estimate after disable, ESTOP, or reconnect."""
        with self._lock:
            self._valid = False
            self._segments = ()

    def mark_positions_valid(
        self,
        positions: Sequence[float],
    ) -> None:
        """Set commanded positions after successful homing."""
        if len(positions) != len(self._joint_names):
            raise ValueError('positions length must match joint_names')

        with self._lock:
            self._positions = [float(value) for value in positions]
            self._valid = True
            self._segments = ()
            self._trajectory_start_time = time.monotonic()

    def mark_homed_zero(self) -> None:
        """Set all commanded positions to zero after successful homing."""
        self.mark_positions_valid([0.0 for _ in self._joint_names])

    def is_valid(self) -> bool:
        """Return whether this estimate can be trusted."""
        with self._lock:
            return self._valid

    def positions(self) -> tuple[float, ...]:
        """Return current estimated positions."""
        with self._lock:
            self._update_locked(time.monotonic())
            return tuple(self._positions)

    def start_trajectory(
        self,
        *,
        initial_positions: Sequence[float],
        batches,
    ) -> None:
        """Schedule a full trajectory for time-based position estimation."""
        if len(initial_positions) != len(self._joint_names):
            raise ValueError(
                'initial_positions length must match joint_names'
            )

        initial = tuple(float(value) for value in initial_positions)
        segments = []
        elapsed_s = 0.0
        previous_positions = initial

        for batch in batches:
            duration_s = (
                batch.duration_ticks
                * DURATION_TICK_NS
                / 1_000_000_000.0
            )

            if duration_s <= 0.0:
                raise ValueError('batch duration must be positive')

            target_positions = tuple(
                float(value)
                for value in batch.target_positions_rad
            )

            if len(target_positions) != len(self._joint_names):
                raise ValueError(
                    'batch target length must match joint_names'
                )

            segment = ScheduledSegment(
                start_s=elapsed_s,
                end_s=elapsed_s + duration_s,
                from_positions=previous_positions,
                to_positions=target_positions,
            )
            segments.append(segment)

            elapsed_s += duration_s
            previous_positions = target_positions

        if not segments:
            raise ValueError('trajectory must contain at least one segment')

        with self._lock:
            if not self._valid:
                raise RuntimeError(
                    'Cannot schedule trajectory before homing/valid position'
                )

            self._positions = list(initial)
            self._trajectory_start_time = time.monotonic()
            self._segments = tuple(segments)

    def to_joint_state_msg(self, stamp) -> JointState:
        """Create a sensor_msgs/JointState message."""
        with self._lock:
            self._update_locked(time.monotonic())

            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(self._joint_names)
            msg.position = list(self._positions)
            msg.velocity = []
            msg.effort = []

            return msg

    def _update_locked(self, now: float) -> None:
        if not self._segments:
            return

        elapsed_s = max(0.0, now - self._trajectory_start_time)
        last_segment = self._segments[-1]

        if elapsed_s >= last_segment.end_s:
            self._positions = list(last_segment.to_positions)
            return

        for segment in self._segments:
            if segment.start_s <= elapsed_s <= segment.end_s:
                duration_s = segment.end_s - segment.start_s

                if duration_s <= 0.0:
                    self._positions = list(segment.to_positions)
                    return

                ratio = (elapsed_s - segment.start_s) / duration_s
                ratio = max(0.0, min(1.0, ratio))

                self._positions = [
                    start + (target - start) * ratio
                    for start, target in zip(
                        segment.from_positions,
                        segment.to_positions,
                    )
                ]
                return
