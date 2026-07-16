"""Pure-Python simulation model for the STM32 Board1 CAN protocol."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import struct
from typing import Deque, Optional

from arm_can_bridge.can_protocol import (
    ALL_MOTORS,
    BOARD_ID_BOARD1,
    BOARD1_MOTOR_COUNT,
    BOARD2_REQUIRED_HOMING_MASK,
    BOARD_ID_BOARD2,
    BOARD_ID_BOARD3,
    BOARD3_READY_VALUE,
    BOARD3_SERVO_COUNT,
    CAN_ID_BOARD1_POSITION_FEEDBACK,
    CAN_ID_BOARD1_STATUS,
    CAN_ID_BOARD2_POSITION_FEEDBACK,
    CAN_ID_BOARD2_POSITION_COMMAND,
    CAN_ID_BOARD2_STATUS,
    CAN_ID_BOARD3_SERVO_COMMAND,
    CAN_ID_BOARD3_POSITION_FEEDBACK,
    CAN_ID_BOARD3_GRIPPER_HOME,
    CAN_ID_BOARD3_STATUS,
    CAN_ID_CLEAR_ERROR,
    CAN_ID_ENABLE,
    CAN_ID_ESTOP,
    CAN_ID_HOMING,
    CAN_ID_POSITION_COMMAND,
    QUEUE_CAPACITY,
    REQUIRED_HOMING_MASK,
    BoardError,
    BoardState,
    CanFrame,
    decode_control_byte,
)


@dataclass
class QueuedCommand:
    """One accepted 0x101 command in the simulated queue."""

    motor_id: int
    target_pos: int
    speed: int
    duration_s: float


@dataclass
class ActiveCommand:
    """One motor command currently being executed by the simulator."""

    motor_id: int
    target_pos: int
    speed: int
    remaining_s: float
    start_pos: int
    duration_s: float


class Board1SimulatorModel:
    """State machine that mimics the implemented STM32 Board1 behavior."""

    def __init__(
        self,
        *,
        board_id: int = BOARD_ID_BOARD1,
        motor_count: int = BOARD1_MOTOR_COUNT,
        position_can_id: int = CAN_ID_POSITION_COMMAND,
        position_feedback_can_id: int = CAN_ID_BOARD1_POSITION_FEEDBACK,
        status_can_id: int = CAN_ID_BOARD1_STATUS,
        required_homing_mask: int = REQUIRED_HOMING_MASK,
        supports_homing: bool = True,
        ready_when_enabled: bool = False,
        queue_capacity: int = QUEUE_CAPACITY,
        homing_duration_s: float = 0.5,
    ) -> None:
        """Initialize the simulated Board1 state."""
        if not 1 <= motor_count <= 15:
            raise ValueError('motor_count must be in range 1..15')
        if not 1 <= queue_capacity <= 0xFF:
            raise ValueError('queue_capacity must be in range 1..255')
        if homing_duration_s < 0.0:
            raise ValueError('homing_duration_s cannot be negative')

        self.board_id = int(board_id)
        self.motor_count = int(motor_count)
        self.position_can_id = int(position_can_id)
        self.position_feedback_can_id = int(position_feedback_can_id)
        self.status_can_id = int(status_can_id)
        self.required_homing_mask = int(required_homing_mask)
        self.supports_homing = bool(supports_homing)
        self.ready_when_enabled = bool(ready_when_enabled)
        self.queue_capacity = int(queue_capacity)
        self.homing_duration_s = float(homing_duration_s)

        self.state = BoardState.INIT
        self.error_code = BoardError.NONE
        self.homing_done_bits = 0
        self.limit_status_bits = 0
        self.enabled = False

        self._queue: Deque[tuple[QueuedCommand, ...]] = deque()
        self._staged_commands: list[QueuedCommand] = []
        self._active_by_motor: list[Optional[ActiveCommand]] = [
            None
            for _ in range(self.motor_count)
        ]
        self._homing_remaining_s = 0.0
        self._homing_mask = 0

        # This is not reported by 0x201, but is useful for future tests.
        self.commanded_angle_raw = [0] * self.motor_count
        self._feedback_motor_index = 0

        self.state = BoardState.IDLE

    @property
    def queue_free(self) -> int:
        """Return remaining queue command slots."""
        queued_slots = len(self._queue) * self.motor_count
        staged_slots = len(self._staged_commands)
        return self.queue_capacity - queued_slots - staged_slots

    @property
    def moving_motor_id(self) -> int:
        """Return active moving or homing motor, or 255 if none."""
        if self.state == BoardState.HOMING:
            for motor_id in range(self.motor_count):
                if self._homing_mask & (1 << motor_id):
                    return motor_id

        for motor_id, command in enumerate(self._active_by_motor):
            if command is not None:
                return motor_id

        return ALL_MOTORS

    def _set_error(self, error_code: BoardError) -> None:
        self.error_code = BoardError(error_code)
        self.state = BoardState.ERROR

    def _clear_motion(self) -> None:
        self._queue.clear()
        self._staged_commands.clear()
        self._active_by_motor = [
            None
            for _ in range(self.motor_count)
        ]
        self._homing_remaining_s = 0.0
        self._homing_mask = 0

    def handle_frame(self, frame: CanFrame) -> bool:
        """Apply one RPi-to-STM32 CAN frame.

        Returns True when the event should cause an immediate 0x201 status
        response in addition to the periodic status timer.
        """
        if frame.can_id == CAN_ID_ESTOP:
            if not self._is_valid_estop_payload(frame.data):
                return True
            self._handle_estop()
            return True

        if frame.can_id == CAN_ID_ENABLE:
            self._handle_enable(frame.data)
            return True

        if frame.can_id == CAN_ID_HOMING:
            if not self.supports_homing or self.board_id == BOARD_ID_BOARD3:
                return False
            self._handle_homing(frame.data)
            return True

        if frame.can_id == CAN_ID_BOARD3_GRIPPER_HOME:
            if self.board_id != BOARD_ID_BOARD3:
                return False
            self._handle_gripper_home(frame.data)
            return True

        if frame.can_id == CAN_ID_CLEAR_ERROR:
            self._handle_clear_error(frame.data)
            return True

        if frame.can_id == self.position_can_id:
            self._handle_position_command(frame.data)
            return True

        return False

    def _is_valid_estop_payload(self, data: bytes) -> bool:
        if len(data) < 1:
            self._set_error(BoardError.INVALID_CMD)
            return False

        if data[0] != 1:
            self._set_error(BoardError.INVALID_CMD)
            return False

        return True

    def _handle_estop(self) -> None:
        self._clear_motion()
        self.enabled = False
        self.state = BoardState.ESTOP

    def _handle_enable(self, data: bytes) -> None:
        if len(data) < 1:
            self._set_error(BoardError.INVALID_CMD)
            return

        enabled = bool(data[0])

        if enabled:
            self.enabled = True
            self.error_code = BoardError.NONE
            if self.ready_when_enabled:
                self.homing_done_bits = BOARD3_READY_VALUE
            self.state = BoardState.IDLE
        else:
            self.enabled = False
            if self.ready_when_enabled:
                self.homing_done_bits = 0
            self._clear_motion()
            self.state = BoardState.IDLE

    def _handle_homing(self, data: bytes) -> None:
        if len(data) < 2:
            self._set_error(BoardError.INVALID_CMD)
            return

        motor_id = data[0]
        mode = data[1]

        if not self.enabled or self.state == BoardState.ESTOP:
            return

        if mode != 0:
            self._set_error(BoardError.INVALID_CMD)
            return

        if motor_id == ALL_MOTORS:
            self._homing_mask = self.required_homing_mask
        elif 0 <= motor_id < self.motor_count:
            self._homing_mask = 1 << motor_id
        else:
            self._set_error(BoardError.INVALID_CMD)
            return

        self._queue.clear()
        self._active_by_motor = [
            None
            for _ in range(self.motor_count)
        ]
        self._homing_remaining_s = self.homing_duration_s
        self.state = BoardState.HOMING

        if self.homing_duration_s == 0.0:
            self._finish_homing()

    def _handle_gripper_home(self, data: bytes) -> None:
        if len(data) < 3:
            self._set_error(BoardError.INVALID_CMD)
            return
        motor_id = data[0]
        mode = data[1]
        if mode != 0:
            self._set_error(BoardError.INVALID_CMD)
            return
        if not self.enabled or self.state == BoardState.ESTOP:
            return
        self._handle_board3_home_posture(motor_id)

    def _handle_board3_home_posture(self, motor_id: int) -> None:
        if motor_id != ALL_MOTORS:
            self._set_error(BoardError.INVALID_CMD)
            return

        self._clear_motion()
        self.homing_done_bits = BOARD3_READY_VALUE

        if self.homing_duration_s == 0.0:
            self.commanded_angle_raw = [0] * self.motor_count
            self.state = BoardState.IDLE
            return

        for local_motor_id in range(self.motor_count):
            self._active_by_motor[local_motor_id] = ActiveCommand(
                motor_id=local_motor_id,
                target_pos=0,
                speed=0,
                remaining_s=self.homing_duration_s,
                start_pos=self.commanded_angle_raw[local_motor_id],
                duration_s=self.homing_duration_s,
            )

        self.state = BoardState.MOVING

    def _handle_clear_error(self, data: bytes) -> None:
        if len(data) < 1:
            self._set_error(BoardError.INVALID_CMD)
            return

        motor_id = data[0]

        if motor_id not in (*range(self.motor_count), ALL_MOTORS):
            self._set_error(BoardError.INVALID_CMD)
            return

        self.error_code = BoardError.NONE

        if self.state != BoardState.ESTOP:
            self.state = BoardState.IDLE

    def _handle_position_command(self, data: bytes) -> None:
        if len(data) != 8:
            self._set_error(BoardError.INVALID_CMD)
            return

        control = decode_control_byte(data[0])

        if not control.execute:
            return

        if control.relative or control.step_mode or control.reserved:
            self._staged_commands.clear()
            self._set_error(BoardError.INVALID_CMD)
            return

        if control.motor_id >= self.motor_count:
            self._staged_commands.clear()
            self._set_error(BoardError.INVALID_CMD)
            return

        if self.state in (BoardState.ERROR, BoardState.ESTOP):
            return

        if not self.enabled or not self._all_axes_homed():
            self._set_error(BoardError.INVALID_CMD)
            return

        if self.queue_free <= 0:
            self._staged_commands.clear()
            self._set_error(BoardError.QUEUE_FULL)
            return

        if self.board_id == BOARD_ID_BOARD3:
            if any(
                command.motor_id == control.motor_id
                for command in self._staged_commands
            ):
                self._staged_commands.clear()
                self._set_error(BoardError.INVALID_CMD)
                return
        else:
            expected_motor_id = len(self._staged_commands)
            if control.motor_id != expected_motor_id:
                self._staged_commands.clear()
                self._set_error(BoardError.INVALID_CMD)
                return

        target_pos = int.from_bytes(
            data[1:5],
            byteorder='little',
            signed=True,
        )
        speed = int.from_bytes(
            data[5:7],
            byteorder='little',
            signed=False,
        )
        duration_ticks = data[7]
        duration_s = 0.001 if duration_ticks == 0 else duration_ticks * 0.005

        if (
            self._staged_commands
            and duration_s != self._staged_commands[0].duration_s
        ):
            self._staged_commands.clear()
            self._set_error(BoardError.INVALID_CMD)
            return

        self._staged_commands.append(
            QueuedCommand(
                motor_id=control.motor_id,
                target_pos=target_pos,
                speed=speed,
                duration_s=duration_s,
            )
        )

        if len(self._staged_commands) == self.motor_count:
            self._queue.append(
                tuple(
                    sorted(
                        self._staged_commands,
                        key=lambda command: command.motor_id,
                    )
                )
            )
            self._staged_commands.clear()

        self._start_ready_commands()

    def _all_axes_homed(self) -> bool:
        return (
            self.homing_done_bits & self.required_homing_mask
        ) == self.required_homing_mask

    def _finish_homing(self) -> None:
        self.homing_done_bits |= self._homing_mask

        for motor_id in range(self.motor_count):
            if self._homing_mask & (1 << motor_id):
                self.commanded_angle_raw[motor_id] = 0

        self._homing_mask = 0
        self._homing_remaining_s = 0.0
        self.state = BoardState.IDLE

    def _start_ready_commands(self) -> None:
        if not self._queue:
            if not self._has_active_motion() and self.state == BoardState.MOVING:
                self.state = BoardState.IDLE
            return

        if self._has_active_motion():
            return

        point = self._queue.popleft()

        for command in point:
            self._active_by_motor[command.motor_id] = ActiveCommand(
                motor_id=command.motor_id,
                target_pos=command.target_pos,
                speed=command.speed,
                remaining_s=command.duration_s,
                start_pos=self.commanded_angle_raw[command.motor_id],
                duration_s=command.duration_s,
            )

        if self._has_active_motion():
            self.state = BoardState.MOVING
        elif self.state == BoardState.MOVING:
            self.state = BoardState.IDLE

    def _has_active_motion(self) -> bool:
        return any(command is not None for command in self._active_by_motor)

    def tick(self, delta_s: float) -> None:
        """Advance simulated time by ``delta_s`` seconds."""
        if delta_s <= 0.0:
            return

        if self.state == BoardState.HOMING:
            self._homing_remaining_s -= delta_s

            if self._homing_remaining_s <= 0.0:
                self._finish_homing()
            return

        self._start_ready_commands()

        if not self._has_active_motion():
            return

        for motor_id, command in enumerate(self._active_by_motor):
            if command is None:
                continue

            command.remaining_s -= delta_s

            if command.remaining_s <= 0.0:
                self.commanded_angle_raw[motor_id] = command.target_pos
                self._active_by_motor[motor_id] = None
                continue

            elapsed_s = max(0.0, command.duration_s - command.remaining_s)
            if command.duration_s <= 0.0:
                ratio = 1.0
            else:
                ratio = max(0.0, min(1.0, elapsed_s / command.duration_s))

            self.commanded_angle_raw[motor_id] = round(
                command.start_pos
                + (command.target_pos - command.start_pos) * ratio
            )

        self._start_ready_commands()

    def build_status_frame(self) -> CanFrame:
        """Build the current 0x201 status response frame."""
        if self.board_id == BOARD_ID_BOARD3:
            staging_count = len(self._staged_commands)
            buffer_free = max(0, BOARD3_SERVO_COUNT - staging_count)
            data = bytes([
                int(self.state),
                int(self.error_code),
                self.homing_done_bits & 0xFF,
                staging_count & 0xFF,
                self.limit_status_bits & 0xFF,
                buffer_free & 0xFF,
                1 if self.enabled else 0,
                ALL_MOTORS,
            ])
            return CanFrame(self.status_can_id, data)

        data = bytes([
            int(self.state),
            int(self.error_code),
            self.homing_done_bits & 0xFF,
            self.moving_motor_id & 0xFF,
            self.limit_status_bits & 0xFF,
            self.queue_free & 0xFF,
            1 if self.enabled else 0,
            0,
        ])
        return CanFrame(self.status_can_id, data)

    def build_position_feedback_frames(
        self,
        *,
        max_frames: Optional[int] = None,
    ) -> tuple[CanFrame, ...]:
        """Build actual-position feedback frames for this simulated board."""
        if self.board_id != BOARD_ID_BOARD3:
            frame_count = (
                self.motor_count
                if max_frames is None
                else min(int(max_frames), self.motor_count)
            )
            motor_ids = [
                (self._feedback_motor_index + offset) % self.motor_count
                for offset in range(frame_count)
            ]
            self._feedback_motor_index = (
                self._feedback_motor_index + frame_count
            ) % self.motor_count
            return tuple(
                self._build_motor_position_feedback_frame(motor_id)
                for motor_id in motor_ids
            )

        frames = []
        for group_index in range(1, 4):
            start = (group_index - 1) * 3
            motor_ids = range(start, start + 3)
            raw_positions = [
                self._feedback_position_raw(motor_id)
                for motor_id in motor_ids
            ]
            status_codes = [
                self._feedback_status_code(motor_id)
                for motor_id in motor_ids
            ]
            flags = 0x40
            flags |= status_codes[0] & 0x03
            flags |= (status_codes[1] & 0x03) << 2
            flags |= (status_codes[2] & 0x03) << 4

            if self.limit_status_bits or self.error_code != BoardError.NONE:
                flags |= 0x80

            frames.append(
                CanFrame(
                    CAN_ID_BOARD3_POSITION_FEEDBACK,
                    struct.pack(
                        '<BhhhB',
                        group_index,
                        raw_positions[0],
                        raw_positions[1],
                        raw_positions[2],
                        flags,
                    ),
                )
            )

        return tuple(frames)

    def _build_motor_position_feedback_frame(
        self,
        motor_id: int,
    ) -> CanFrame:
        flags = 0x01
        if self._all_axes_homed():
            flags |= 0x02
        if self._active_by_motor[motor_id] is not None:
            flags |= 0x04
        else:
            flags |= 0x08

        error_code = int(self.error_code)
        return CanFrame(
            self.position_feedback_can_id,
            struct.pack(
                '<BBiBB',
                motor_id,
                flags,
                int(self.commanded_angle_raw[motor_id]),
                error_code,
                0,
            ),
        )

    def _feedback_position_raw(self, motor_id: int) -> int:
        raw = int(self.commanded_angle_raw[motor_id])
        return max(-32768, min(32767, raw))

    def _feedback_status_code(self, motor_id: int) -> int:
        if self.state == BoardState.ERROR:
            return 3
        if self._active_by_motor[motor_id] is not None:
            return 1
        return 0


def make_board2_simulator_model(
    *,
    queue_capacity: int = QUEUE_CAPACITY,
    homing_duration_s: float = 0.5,
) -> Board1SimulatorModel:
    """Create a one-axis Board2 simulator model."""
    return Board1SimulatorModel(
        board_id=BOARD_ID_BOARD2,
        motor_count=1,
        position_can_id=CAN_ID_BOARD2_POSITION_COMMAND,
        position_feedback_can_id=CAN_ID_BOARD2_POSITION_FEEDBACK,
        status_can_id=CAN_ID_BOARD2_STATUS,
        required_homing_mask=BOARD2_REQUIRED_HOMING_MASK,
        queue_capacity=queue_capacity,
        homing_duration_s=homing_duration_s,
    )


def make_board3_simulator_model(
    *,
    queue_capacity: int = QUEUE_CAPACITY,
    homing_duration_s: float = 0.5,
) -> Board1SimulatorModel:
    """Create a nine-servo Board3 simulator model."""
    return Board1SimulatorModel(
        board_id=BOARD_ID_BOARD3,
        motor_count=BOARD3_SERVO_COUNT,
        position_can_id=CAN_ID_BOARD3_SERVO_COMMAND,
        position_feedback_can_id=CAN_ID_BOARD3_POSITION_FEEDBACK,
        status_can_id=CAN_ID_BOARD3_STATUS,
        required_homing_mask=BOARD3_READY_VALUE,
        supports_homing=True,
        ready_when_enabled=True,
        queue_capacity=queue_capacity,
        homing_duration_s=homing_duration_s,
    )
