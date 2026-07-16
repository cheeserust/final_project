"""Board1 Goal V3 + Board2 legacy direct-goal execution state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
import threading
import time
from typing import Callable, Mapping, Sequence

from .can_protocol import (
    ArmGoalAck,
    ArmGoalAckResult,
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD2,
    BoardState,
    BoardStatus,
    CanFrame,
    MAX_DURATION_TICKS,
    pack_arm_goal_control_v3,
    pack_arm_goal_v3,
    pack_position_command,
)
from .can_writer import SerializedCanWriter


ARM_JOINT_NAMES = (
    'base_joint',
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
)
JOINT_TO_BOARD_MOTOR = {
    'base_joint': (BOARD_ID_BOARD1, 3),
    'arm_joint_1': (BOARD_ID_BOARD1, 0),
    'arm_joint_2': (BOARD_ID_BOARD1, 1),
    'arm_joint_3': (BOARD_ID_BOARD1, 2),
    'arm_joint_4': (BOARD_ID_BOARD2, 0),
}
JOINT_RAW_LIMITS = {
    'base_joint': (-9000, 18000),
    'arm_joint_1': (-8650, 9000),
    'arm_joint_2': (-7810, 8000),
    'arm_joint_3': (-9150, 9000),
    'arm_joint_4': (-9000, 9000),
}
FULL_MASK_BY_BOARD = {
    BOARD_ID_BOARD1: 0x0F,
    BOARD_ID_BOARD2: 0x01,
}
COMPLETION_FREE_BY_BOARD = {
    BOARD_ID_BOARD1: 1,
    BOARD_ID_BOARD2: 32,
}


class ArmGoalPhase(IntEnum):
    """Feedback phases for the direct arm action."""

    STAGING = 1
    READY = 2
    MOVING = 3
    CANCELING = 4
    COMPLETE = 5


class ArmGoalRuntimeState(str, Enum):
    """Host-side lifecycle for the single active hybrid arm goal."""

    IDLE = 'IDLE'
    SENDING = 'SENDING'
    WAITING_READY = 'WAITING_READY'
    STARTING = 'STARTING'
    MOVING = 'MOVING'
    CANCELLING = 'CANCELLING'
    COMPLETED = 'COMPLETED'
    ABORTED_BY_ESTOP = 'ABORTED_BY_ESTOP'


class ArmGoalV3Error(RuntimeError):
    """Raised when a V3 goal cannot safely complete."""


class ArmGoalV3Canceled(ArmGoalV3Error):
    """Raised after Board1 acknowledges cancellation."""


class ArmGoalV3AbortedByEstop(ArmGoalV3Error):
    """Raised when an E-stop preempts the active goal without disabling."""


class _TerminalAckError(ArmGoalV3Error):
    """Carry the exact non-retryable ACK that failed a goal."""

    def __init__(self, ack: ArmGoalAck) -> None:
        self.ack = ack
        super().__init__(
            f'Board{ack.board_id} {ack.result.name}: '
            f'goal={ack.goal_id}, mask=0x{ack.received_axis_mask:02X}, '
            f'duration_ms={ack.duration_ms}, state={ack.state_snapshot}'
        )


@dataclass(frozen=True)
class ArmGoalFrames:
    """One direct goal split into Board1 V3 and Board2 legacy batches."""

    goal_id: int
    duration_ms: int
    positions_by_name: Mapping[str, float]
    board1: tuple[CanFrame, ...]
    board2: tuple[CanFrame, ...]


def build_arm_goal_frames_v3(
    *,
    joint_names: Sequence[str],
    positions_rad: Sequence[float],
    duration_ms: int,
    goal_id: int,
) -> ArmGoalFrames:
    """Build ordered Board1 V3 frames and one Board2 legacy frame."""
    if len(joint_names) != len(positions_rad):
        raise ValueError('joint_names and positions must have equal lengths')
    if len(joint_names) != len(ARM_JOINT_NAMES):
        raise ValueError('Direct arm goal must contain exactly five joints')
    if len(set(joint_names)) != len(joint_names):
        raise ValueError('Direct arm goal contains duplicate joint names')
    if set(joint_names) != set(ARM_JOINT_NAMES):
        raise ValueError(
            f'Direct arm goal joints must be {list(ARM_JOINT_NAMES)}'
        )
    if not 1 <= int(duration_ms) <= 0xFFFF:
        raise ValueError('duration_ms must be in range 1..65535')
    if not 0 <= int(goal_id) <= 0xFF:
        raise ValueError('goal_id must fit uint8')

    positions = {
        str(name): float(position)
        for name, position in zip(joint_names, positions_rad)
    }
    frames_by_board_motor: dict[tuple[int, int], CanFrame] = {}
    for name in ARM_JOINT_NAMES:
        position = positions[name]
        if not math.isfinite(position):
            raise ValueError(f'{name} position must be finite')
        target_raw = round(position * 18_000.0 / math.pi)
        minimum, maximum = JOINT_RAW_LIMITS[name]
        if not minimum <= target_raw <= maximum:
            raise ValueError(
                f'{name} target raw {target_raw} is outside '
                f'[{minimum}, {maximum}]'
            )
        board_id, motor_id = JOINT_TO_BOARD_MOTOR[name]
        if board_id == BOARD_ID_BOARD1:
            frame = pack_arm_goal_v3(
                board_id=board_id,
                motor_id=motor_id,
                target_pos=int(target_raw),
                goal_id=int(goal_id),
                duration_ms=int(duration_ms),
            )
        else:
            duration_ticks = min(
                MAX_DURATION_TICKS,
                max(1, math.ceil(int(duration_ms) / 5.0)),
            )
            frame = pack_position_command(
                board_id=BOARD_ID_BOARD2,
                motor_id=motor_id,
                target_pos=int(target_raw),
                speed=0,
                duration_ticks=duration_ticks,
                execute=True,
                relative=False,
                step_mode=False,
            )
        frames_by_board_motor[(board_id, motor_id)] = frame

    return ArmGoalFrames(
        goal_id=int(goal_id),
        duration_ms=int(duration_ms),
        positions_by_name=positions,
        board1=tuple(
            frames_by_board_motor[(BOARD_ID_BOARD1, motor_id)]
            for motor_id in range(4)
        ),
        board2=(frames_by_board_motor[(BOARD_ID_BOARD2, 0)],),
    )


class ArmGoalV3Coordinator:
    """Coordinate Board1 V3 with a directly executing legacy Board2."""

    def __init__(
        self,
        writer: SerializedCanWriter,
        *,
        ack_timeout_s: float = 0.25,
        communication_timeout_s: float = 1.0,
        max_stage_attempts: int = 200,
        event_callback: Callable[[Mapping[str, object]], None] | None = None,
    ) -> None:
        if ack_timeout_s <= 0.0:
            raise ValueError('ack_timeout_s must be positive')
        if communication_timeout_s <= 0.0:
            raise ValueError('communication_timeout_s must be positive')
        if max_stage_attempts <= 0:
            raise ValueError('max_stage_attempts must be positive')
        self._writer = writer
        self._ack_timeout_s = float(ack_timeout_s)
        self._communication_timeout_s = float(communication_timeout_s)
        self._max_stage_attempts = int(max_stage_attempts)
        self._event_callback = event_callback
        self._condition = threading.Condition()
        self._acks: dict[int, ArmGoalAck] = {}
        self._last_acks: dict[int, ArmGoalAck] = {}
        self._statuses: dict[int, tuple[BoardStatus, float]] = {}
        self._ready_boards: set[int] = set()
        self._started_boards: set[int] = set()
        self._cancelled_boards: set[int] = set()
        self._staging_timeout_boards: set[int] = set()
        self._terminal_ack: ArmGoalAck | None = None
        self._next_goal_id = 0
        self._capability_confirmed = False
        self._active_goal_id: int | None = None
        self._active_duration_ms: int | None = None
        self._board2_goal_sent = False
        self._external_cancel_requested = False
        self._estop_abort_requested = False
        self._state = ArmGoalRuntimeState.IDLE

    @property
    def capability_confirmed(self) -> bool:
        with self._condition:
            return self._capability_confirmed

    @property
    def active_goal_id(self) -> int | None:
        with self._condition:
            return self._active_goal_id

    @property
    def state(self) -> ArmGoalRuntimeState:
        with self._condition:
            return self._state

    def request_active_cancel(self) -> bool:
        """Request a coordinated cancellation from another control callback."""
        with self._condition:
            if self._active_goal_id is None:
                return False
            self._external_cancel_requested = True
            self._condition.notify_all()
            return True

    def abort_by_estop(self, frame: CanFrame) -> None:
        """Latch E-stop, purge queued motion, and priority-send the E-stop."""
        with self._condition:
            aborted_goal_id = self._active_goal_id
            self._estop_abort_requested = True
            self._external_cancel_requested = False
            self._state = ArmGoalRuntimeState.ABORTED_BY_ESTOP
            self._condition.notify_all()
        receipt = self._writer.send_emergency(frame)
        self._emit(
            'estop_sent',
            goal_id=aborted_goal_id,
            tx_started=receipt.started_at,
            tx_completed=receipt.completed_at,
        )

    def clear_estop_latch(self) -> None:
        """Allow only a future user goal after Enable or Clear Error."""
        with self._condition:
            self._estop_abort_requested = False
            if self._active_goal_id is None:
                self._state = ArmGoalRuntimeState.IDLE
            self._condition.notify_all()

    def update_ack(self, ack: ArmGoalAck) -> None:
        """Store validated evidence without letting stale ACKs overwrite it."""
        if ack.board_id != BOARD_ID_BOARD1:
            self._emit(
                'ack_ignored',
                board_id=ack.board_id,
                goal_id=ack.goal_id,
                reason='board2_legacy',
            )
            return
        promoted_duplicate = False
        ready_accepted = False
        stale_reason: str | None = None
        with self._condition:
            self._last_acks[ack.board_id] = ack
            if self._estop_abort_requested:
                stale_reason = 'estop_epoch'
            elif self._active_goal_id is None:
                stale_reason = 'no_active_goal'
            elif ack.goal_id != self._active_goal_id:
                stale_reason = 'goal_id_mismatch'
            elif ack.duration_ms != self._active_duration_ms:
                stale_reason = 'duration_mismatch'
            else:
                self._acks[ack.board_id] = ack
                full_mask = FULL_MASK_BY_BOARD[ack.board_id]
                if (
                    ack.result
                    in (ArmGoalAckResult.READY, ArmGoalAckResult.DUPLICATE)
                    and ack.received_axis_mask == full_mask
                ):
                    self._ready_boards.add(ack.board_id)
                    ready_accepted = True
                    promoted_duplicate = (
                        ack.result == ArmGoalAckResult.DUPLICATE
                    )
                elif (
                    ack.result == ArmGoalAckResult.STARTED
                    and ack.received_axis_mask == full_mask
                ):
                    self._started_boards.add(ack.board_id)
                elif ack.result == ArmGoalAckResult.CANCELLED:
                    self._cancelled_boards.add(ack.board_id)
                elif ack.result == ArmGoalAckResult.STAGING_TIMEOUT:
                    self._staging_timeout_boards.add(ack.board_id)
                elif ack.result in (
                    ArmGoalAckResult.BUSY,
                    ArmGoalAckResult.CONFLICT,
                    ArmGoalAckResult.INVALID,
                ):
                    self._terminal_ack = ack
                self._condition.notify_all()

        self._emit(
            'ack_received',
            board_id=ack.board_id,
            version=ack.protocol_version,
            result=ack.result.name,
            goal_id=ack.goal_id,
            mask=f'0x{ack.received_axis_mask:02X}',
            state=ack.state_snapshot,
            duration_ms=ack.duration_ms,
            stale=stale_reason is not None,
            stale_reason=stale_reason,
            duplicate_promoted_to_ready=promoted_duplicate,
        )
        if ready_accepted:
            timestamp = time.monotonic()
            self._emit(
                'board_ready',
                board_id=ack.board_id,
                goal_id=ack.goal_id,
                result=ack.result.name,
                **({'t4': timestamp} if ack.board_id == 1 else {'t5': timestamp}),
            )

    def update_status(self, status: BoardStatus) -> None:
        """Store one complete Board1 V3 or Board2 legacy status snapshot."""
        if status.board_id not in (BOARD_ID_BOARD1, BOARD_ID_BOARD2):
            return
        received_at = time.monotonic()
        with self._condition:
            self._statuses[status.board_id] = (status, received_at)
            if all(
                board_id in self._statuses
                for board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2)
            ):
                self._capability_confirmed = True
            self._condition.notify_all()
        self._emit(
            'status_received',
            board_id=status.board_id,
            state=int(status.state),
            error=int(status.error_code),
            goal_slot_free=status.goal_slot_free,
            enabled=status.enabled,
            sequence=status.status_sequence,
            limit_bits=f'0x{status.limit_status_bits:02X}',
        )

    def probe_capability(self) -> bool:
        """Passively confirm both status protocols without sending CANCEL."""
        now = time.monotonic()
        with self._condition:
            success = all(
                board_id in self._statuses
                and now - self._statuses[board_id][1]
                <= self._communication_timeout_s
                for board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2)
            )
            self._capability_confirmed = success
        self._emit('capability_probe', success=success, destructive=False)
        return success

    def reset_capability(self) -> None:
        """Block motion until fresh statuses arrive after transport loss."""
        with self._condition:
            self._capability_confirmed = False
            self._external_cancel_requested = False
            self._estop_abort_requested = False
            self._clear_goal_evidence_locked()
            self._statuses.clear()
            self._condition.notify_all()

    def execute(
        self,
        *,
        joint_names: Sequence[str],
        positions_rad: Sequence[float],
        duration_ms: int,
        cancel_requested: Callable[[], bool] | None = None,
        feedback: Callable[[ArmGoalPhase, int, str], None] | None = None,
        request_id: str | None = None,
        web_created_unix_ms: int = 0,
        gui_received_unix_ms: int = 0,
    ) -> ArmGoalFrames:
        """Run one hybrid goal; fresh MOVING heartbeats have no deadline."""
        with self._condition:
            if not self._capability_confirmed:
                raise ArmGoalV3Error(
                    'Board1 V3 + Board2 legacy status is not confirmed'
                )
            self._require_board2_ready_locked(time.monotonic())
            if self._estop_abort_requested:
                raise ArmGoalV3Error('E-stop is latched; Enable or Clear first')
            if self._active_goal_id is not None:
                raise ArmGoalV3Error('Another direct arm goal is active')
            goal_id = self._allocate_goal_id_locked()

        goal = build_arm_goal_frames_v3(
            joint_names=joint_names,
            positions_rad=positions_rad,
            duration_ms=duration_ms,
            goal_id=goal_id,
        )
        with self._condition:
            self._active_goal_id = goal_id
            self._active_duration_ms = goal.duration_ms
            self._board2_goal_sent = False
            self._external_cancel_requested = False
            self._clear_goal_evidence_locked()
            self._state = ArmGoalRuntimeState.SENDING

        self._emit(
            'goal_begin',
            request_id=request_id,
            t0_web_created_unix_ms=int(web_created_unix_ms),
            t1_gui_received_unix_ms=int(gui_received_unix_ms),
            goal_id=goal_id,
            duration_ms=goal.duration_ms,
            targets={
                name: goal.positions_by_name[name]
                for name in ARM_JOINT_NAMES
            },
            t1_server_received=time.monotonic(),
        )
        if goal.duration_ms > MAX_DURATION_TICKS * 5:
            self._emit(
                'board2_legacy_duration_clamped',
                goal_id=goal_id,
                requested_duration_ms=goal.duration_ms,
                board2_duration_ms=MAX_DURATION_TICKS * 5,
            )
        try:
            self._publish(feedback, ArmGoalPhase.STAGING, goal_id, 'staging')
            self._stage_until_ready(goal, cancel_requested)
            self._publish(feedback, ArmGoalPhase.READY, goal_id, 'ready')
            self._raise_or_cancel(goal_id, cancel_requested)
            with self._condition:
                self._require_board2_ready_locked(time.monotonic())
                self._statuses.clear()
                self._state = ArmGoalRuntimeState.STARTING
                # Mark the legacy axis conservatively before writing: a TX
                # failure can still be ambiguous after the frame is queued.
                self._board2_goal_sent = True
            board2_receipt = self._writer.send_batch(
                goal.board2,
                goal_id=goal_id,
                category='goal',
            )
            self._emit(
                'board2_legacy_goal_sent',
                board_id=BOARD_ID_BOARD2,
                goal_id=goal_id,
                tx_completed=getattr(
                    board2_receipt,
                    'completed_at',
                    time.monotonic(),
                ),
                payloads=[
                    frame.data.hex().upper() for frame in goal.board2
                ],
            )
            start_receipt = self._writer.send_batch(
                (pack_arm_goal_control_v3(1, goal_id),),
                goal_id=goal_id,
                category='start',
            )
            self._emit(
                'start_sent',
                goal_id=goal_id,
                t6=getattr(start_receipt, 'completed_at', time.monotonic()),
            )
            self._wait_started(goal_id, cancel_requested)
            with self._condition:
                self._state = ArmGoalRuntimeState.MOVING
            self._publish(feedback, ArmGoalPhase.MOVING, goal_id, 'moving')
            self._wait_complete(goal_id, cancel_requested)
            with self._condition:
                self._state = ArmGoalRuntimeState.COMPLETED
            self._publish(feedback, ArmGoalPhase.COMPLETE, goal_id, 'complete')
            self._emit('goal_complete', goal_id=goal_id)
            return goal
        except (ArmGoalV3Canceled, ArmGoalV3AbortedByEstop):
            raise
        except ArmGoalV3Error as exc:
            if self._should_cancel_after_failure(exc):
                try:
                    self._cancel(goal_id)
                except ArmGoalV3Error as cancel_exc:
                    raise ArmGoalV3Error(
                        f'{exc}; coordinated cancel failed: {cancel_exc}'
                    ) from exc
            raise
        finally:
            with self._condition:
                self._active_goal_id = None
                self._active_duration_ms = None
                self._board2_goal_sent = False
                self._external_cancel_requested = False
                if not self._estop_abort_requested:
                    self._state = ArmGoalRuntimeState.IDLE
                self._condition.notify_all()

    def _stage_until_ready(
        self,
        goal: ArmGoalFrames,
        cancel_requested: Callable[[], bool] | None,
    ) -> None:
        for attempt in range(1, self._max_stage_attempts + 1):
            self._raise_or_cancel(goal.goal_id, cancel_requested)
            with self._condition:
                self._state = ArmGoalRuntimeState.SENDING
                board1_missing = BOARD_ID_BOARD1 not in self._ready_boards
                self._staging_timeout_boards.discard(BOARD_ID_BOARD1)
            self._emit(
                'staging_attempt',
                goal_id=goal.goal_id,
                attempt=attempt,
                max_attempts=self._max_stage_attempts,
                boards=[BOARD_ID_BOARD1] if board1_missing else [],
            )
            if board1_missing:
                receipt = self._writer.send_batch(
                    goal.board1,
                    goal_id=goal.goal_id,
                    category='goal',
                )
                completed_at = getattr(
                    receipt, 'completed_at', time.monotonic()
                )
                deadline = completed_at + self._ack_timeout_s
                self._emit(
                    'board_batch_sent',
                    board_id=BOARD_ID_BOARD1,
                    goal_id=goal.goal_id,
                    attempt=attempt,
                    t2=getattr(receipt, 'started_at', completed_at),
                    t3=completed_at,
                    payloads=[frame.data.hex().upper() for frame in goal.board1],
                )
            self._raise_or_cancel(goal.goal_id, cancel_requested)
            with self._condition:
                self._raise_for_board_status_error_locked()
                self._raise_for_terminal_ack_locked()

            with self._condition:
                self._state = ArmGoalRuntimeState.WAITING_READY
            while True:
                self._raise_or_cancel(goal.goal_id, cancel_requested)
                with self._condition:
                    self._raise_for_board_status_error_locked()
                    self._raise_for_terminal_ack_locked()
                    if BOARD_ID_BOARD1 in self._ready_boards:
                        now = time.monotonic()
                        self._emit(
                            'board1_ready',
                            goal_id=goal.goal_id,
                            t5=now,
                        )
                        return
                    if (
                        BOARD_ID_BOARD1 in self._staging_timeout_boards
                    ):
                        ack = self._acks.get(BOARD_ID_BOARD1)
                        received_mask = (
                            0 if ack is None else ack.received_axis_mask
                        )
                        missing_mask = (
                            FULL_MASK_BY_BOARD[BOARD_ID_BOARD1]
                            & ~received_mask
                        )
                        self._emit(
                            'staging_timeout',
                            board_id=BOARD_ID_BOARD1,
                            goal_id=goal.goal_id,
                            attempt=attempt,
                            received_mask=f'0x{received_mask:02X}',
                            missing_mask=f'0x{missing_mask:02X}',
                            missing_motor_ids=[
                                motor_id
                                for motor_id in range(4)
                                if missing_mask & (1 << motor_id)
                            ],
                        )
                        break
                    now = time.monotonic()
                    if now >= deadline:
                        break
                    timeout = max(0.0, deadline - now)
                    self._condition.wait(timeout=min(0.02, timeout))

        raise ArmGoalV3Error(self._ready_timeout_detail(goal.goal_id))

    def _wait_started(
        self,
        goal_id: int,
        cancel_requested: Callable[[], bool] | None,
    ) -> None:
        deadline = time.monotonic() + self._communication_timeout_s
        while time.monotonic() < deadline:
            self._raise_or_cancel(goal_id, cancel_requested)
            with self._condition:
                self._raise_for_board_status_error_locked()
                self._raise_for_terminal_ack_locked()
                if BOARD_ID_BOARD1 in self._started_boards:
                    self._emit(
                        'board1_started', goal_id=goal_id, t7=time.monotonic()
                    )
                    return
                self._condition.wait(timeout=0.02)
        raise ArmGoalV3Error('STARTED timeout; missing boards=[1]')

    def _wait_complete(
        self,
        goal_id: int,
        cancel_requested: Callable[[], bool] | None,
    ) -> None:
        wait_started_at = time.monotonic()
        while True:
            self._raise_or_cancel(goal_id, cancel_requested)
            with self._condition:
                now = time.monotonic()
                self._raise_for_board_status_error_locked()
                if self._statuses_complete(now):
                    return
                missing = {
                    BOARD_ID_BOARD1,
                    BOARD_ID_BOARD2,
                } - set(self._statuses)
                stale = [
                    board_id
                    for board_id, (_, received_at) in self._statuses.items()
                    if now - received_at > self._communication_timeout_s
                ]
                if stale or (
                    missing
                    and now - wait_started_at > self._communication_timeout_s
                ):
                    raise ArmGoalV3Error(
                        'Board status heartbeat timeout; '
                        f'missing={sorted(missing)}, stale={sorted(stale)}'
                    )
                self._condition.wait(timeout=0.02)

    def _cancel(self, goal_id: int) -> None:
        with self._condition:
            if self._estop_abort_requested:
                raise ArmGoalV3AbortedByEstop(
                    f'Goal {goal_id} aborted by E-stop'
                )
            self._state = ArmGoalRuntimeState.CANCELLING
            self._cancelled_boards.clear()
        self._writer.discard_goal(goal_id)
        if not self._writer.wait_goal_idle(goal_id):
            raise ArmGoalV3Error(
                f'Goal {goal_id} TX batch did not become idle before CANCEL'
            )
        self._publish(None, ArmGoalPhase.CANCELING, goal_id, 'canceling')
        cancel_receipt = self._writer.send_batch(
            (pack_arm_goal_control_v3(2, goal_id),),
            goal_id=goal_id,
            category='cancel',
        )
        cancel_sent_at = getattr(
            cancel_receipt,
            'completed_at',
            time.monotonic(),
        )
        deadline = time.monotonic() + self._communication_timeout_s
        board1_cancelled = False
        with self._condition:
            while time.monotonic() < deadline:
                if BOARD_ID_BOARD1 in self._cancelled_boards:
                    self._emit(
                        'board1_cancelled',
                        goal_id=goal_id,
                        board2_legacy_unconfirmed=True,
                    )
                    board1_cancelled = True
                    break
                if self._estop_abort_requested:
                    raise ArmGoalV3AbortedByEstop(
                        f'Goal {goal_id} aborted by E-stop during CANCEL'
                    )
                self._condition.wait(timeout=0.02)
            if not board1_cancelled:
                raise ArmGoalV3Error(
                    'CANCELLED timeout; '
                    f'missing boards=[1]; {self._status_detail_locked()}'
                )
            board2_goal_sent = self._board2_goal_sent

        if board2_goal_sent:
            self._wait_board2_legacy_idle(
                goal_id,
                not_before=cancel_sent_at,
            )

    def _raise_or_cancel(
        self,
        goal_id: int,
        predicate: Callable[[], bool] | None,
    ) -> None:
        with self._condition:
            estop_abort = self._estop_abort_requested
            external_cancel = self._external_cancel_requested
        if estop_abort:
            raise ArmGoalV3AbortedByEstop(
                f'Goal {goal_id} aborted by E-stop; motor enable unchanged'
            )
        if external_cancel or (predicate is not None and predicate()):
            self._cancel(goal_id)
            raise ArmGoalV3Canceled(
                f'Goal {goal_id} canceled on Board1; '
                'Board2 legacy execution cannot be confirmed canceled'
            )

    def _raise_for_terminal_ack_locked(self) -> None:
        if self._terminal_ack is not None:
            raise _TerminalAckError(self._terminal_ack)

    def _raise_for_board_status_error_locked(self) -> None:
        for board_id, (status, _) in self._statuses.items():
            if (
                status.error_code != 0
                or not status.enabled
                or status.state in (
                    BoardState.ERROR,
                    BoardState.ESTOP,
                    BoardState.DISABLED,
                )
            ):
                raise ArmGoalV3Error(
                    f'Board{board_id} status error: '
                    f'state={int(status.state)}, '
                    f'error={int(status.error_code)}, '
                    f'enabled={status.enabled}, '
                    f'limit_bits=0x{status.limit_status_bits:02X}'
                )

    def _require_board2_ready_locked(self, now: float) -> None:
        """Reject a goal until the non-cancellable legacy axis is idle."""
        entry = self._statuses.get(BOARD_ID_BOARD2)
        if entry is None:
            raise ArmGoalV3Error('Board2 legacy has no status')
        status, received_at = entry
        age = now - received_at
        if age > self._communication_timeout_s:
            raise ArmGoalV3Error(
                f'Board2 legacy status is stale ({age * 1000.0:.1f} ms)'
            )
        if not (
            status.state == BoardState.IDLE
            and status.error_code == 0
            and status.enabled
            and status.ready_mask == FULL_MASK_BY_BOARD[BOARD_ID_BOARD2]
            and status.moving_mask == 0
            and status.target_reached_mask
            == FULL_MASK_BY_BOARD[BOARD_ID_BOARD2]
            and status.queue_free
            == COMPLETION_FREE_BY_BOARD[BOARD_ID_BOARD2]
        ):
            raise ArmGoalV3Error(
                'Board2 legacy is not ready for a new goal; '
                + self._status_detail_locked()
            )

    def _wait_board2_legacy_idle(
        self,
        goal_id: int,
        *,
        not_before: float,
    ) -> None:
        """Quarantine replacement goals until Board2 finishes its target."""
        wait_started_at = time.monotonic()
        while True:
            with self._condition:
                if self._estop_abort_requested:
                    raise ArmGoalV3AbortedByEstop(
                        f'Goal {goal_id} aborted by E-stop during CANCEL'
                    )
                now = time.monotonic()
                entry = self._statuses.get(BOARD_ID_BOARD2)
                if entry is not None:
                    status, received_at = entry
                    if (
                        received_at >= not_before
                        and
                        now - received_at
                        <= self._communication_timeout_s
                        and status.state == BoardState.IDLE
                        and status.error_code == 0
                        and status.enabled
                        and status.ready_mask
                        == FULL_MASK_BY_BOARD[BOARD_ID_BOARD2]
                        and status.moving_mask == 0
                        and status.target_reached_mask
                        == FULL_MASK_BY_BOARD[BOARD_ID_BOARD2]
                        and status.queue_free
                        == COMPLETION_FREE_BY_BOARD[BOARD_ID_BOARD2]
                    ):
                        self._emit(
                            'board2_legacy_idle_after_cancel',
                            goal_id=goal_id,
                        )
                        return
                    if (
                        now - received_at
                        > self._communication_timeout_s
                    ):
                        raise ArmGoalV3Error(
                            'Board2 legacy status heartbeat timeout during '
                            f'CANCEL; {self._status_detail_locked()}'
                        )
                    if (
                        status.error_code != 0
                        or not status.enabled
                        or status.state in (
                            BoardState.ERROR,
                            BoardState.ESTOP,
                            BoardState.DISABLED,
                        )
                    ):
                        self._raise_for_board_status_error_locked()
                elif (
                    now - wait_started_at
                    > self._communication_timeout_s
                ):
                    raise ArmGoalV3Error(
                        'Board2 legacy status missing during CANCEL'
                    )
                self._condition.wait(timeout=0.02)

    def _statuses_complete(self, now: float) -> bool:
        if BOARD_ID_BOARD1 not in self._started_boards:
            return False
        for board_id, expected_mask in FULL_MASK_BY_BOARD.items():
            entry = self._statuses.get(board_id)
            if entry is None:
                return False
            status, received_at = entry
            if now - received_at > self._communication_timeout_s:
                return False
            if not (
                status.state == BoardState.IDLE
                and status.error_code == 0
                and status.moving_mask == 0
                and status.target_reached_mask == expected_mask
                and status.goal_slot_free
                == COMPLETION_FREE_BY_BOARD[board_id]
            ):
                return False
        return True

    def _ready_timeout_detail(self, goal_id: int) -> str:
        with self._condition:
            parts = []
            ack = self._acks.get(BOARD_ID_BOARD1)
            if ack is None:
                parts.append('Board1=no matching ACK')
            else:
                parts.append(
                    f'Board1={ack.result.name}/'
                    f'mask=0x{ack.received_axis_mask:02X}/'
                    f'duration={ack.duration_ms}'
                )
            parts.append('Board2=legacy/no READY ACK')
            return (
                f'READY timeout for goal {goal_id} after '
                f'{self._max_stage_attempts} staging attempts; '
                + ', '.join(parts)
                + '; '
                + self._status_detail_locked()
            )

    def _status_detail_locked(self) -> str:
        parts = []
        now = time.monotonic()
        for board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2):
            entry = self._statuses.get(board_id)
            if entry is None:
                parts.append(f'Board{board_id}=no status')
                continue
            status, received_at = entry
            parts.append(
                f'Board{board_id}=state:{int(status.state)}/'
                f'error:{status.error_code}/slot:{status.goal_slot_free}/'
                f'enabled:{status.enabled}/seq:{status.status_sequence}/'
                f'age_ms:{(now - received_at) * 1000.0:.1f}'
            )
        return ', '.join(parts)

    def _should_cancel_after_failure(self, error: ArmGoalV3Error) -> bool:
        with self._condition:
            if self._estop_abort_requested:
                return False
        return not (
            isinstance(error, _TerminalAckError)
            and error.ack.result == ArmGoalAckResult.BUSY
        )

    def _clear_goal_evidence_locked(self) -> None:
        self._acks.clear()
        self._ready_boards.clear()
        self._started_boards.clear()
        self._cancelled_boards.clear()
        self._staging_timeout_boards.clear()
        self._terminal_ack = None

    def _allocate_goal_id_locked(self) -> int:
        goal_id = self._next_goal_id
        self._next_goal_id = (self._next_goal_id + 1) & 0xFF
        return goal_id

    def _emit(self, event: str, **fields: object) -> None:
        callback = self._event_callback
        if callback is None:
            return
        payload: dict[str, object] = {
            'component': 'arm_goal_v3',
            'event': event,
            'monotonic_s': time.monotonic(),
            'active_state': self._state.value,
            **fields,
        }
        try:
            callback(payload)
        except Exception:
            pass

    @staticmethod
    def _publish(callback, phase, goal_id, detail) -> None:
        if callback is not None:
            callback(phase, goal_id, detail)
