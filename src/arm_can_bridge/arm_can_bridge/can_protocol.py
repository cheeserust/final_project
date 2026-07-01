"""Pack and unpack the VicPinky Board1/2/3 Classic CAN protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct


CAN_ID_ESTOP = 0x001
CAN_ID_ENABLE = 0x010
CAN_ID_HOMING = 0x020
CAN_ID_CLEAR_ERROR = 0x030

CAN_ID_BOARD1_POSITION_COMMAND = 0x101
CAN_ID_BOARD2_POSITION_COMMAND = 0x102
CAN_ID_BOARD3_SERVO_COMMAND = 0x103

CAN_ID_BOARD1_STATUS = 0x201
CAN_ID_BOARD2_STATUS = 0x202
CAN_ID_BOARD3_STATUS = 0x203

CAN_ID_BOARD1_POSITION_FEEDBACK = 0x301
CAN_ID_BOARD2_POSITION_FEEDBACK = 0x302
CAN_ID_BOARD3_POSITION_FEEDBACK = 0x303

# Backward-compatible aliases for Board1-only callers and tests.
CAN_ID_POSITION_COMMAND = CAN_ID_BOARD1_POSITION_COMMAND
CAN_ID_STATUS = CAN_ID_BOARD1_STATUS

BOARD_ID_BOARD1 = 1
BOARD_ID_BOARD2 = 2
BOARD_ID_BOARD3 = 3
BOARD_ID_ALL_LEGACY = 0x00
BOARD_ID_ALL = 0xFF

BOARD1_MOTOR_COUNT = 4
BOARD2_MOTOR_COUNT = 1
BOARD3_SERVO_COUNT = 9

ALL_MOTORS = 0xFF
REQUIRED_HOMING_MASK = 0x0F
BOARD2_REQUIRED_HOMING_MASK = 0x01
BOARD3_READY_VALUE = 0x01
QUEUE_CAPACITY = 32

ANGLE_RAW_PER_DEGREE = 100.0
DURATION_TICK_NS = 5_000_000
MAX_DURATION_TICKS = 0xFF

FLAG_EXECUTE = 0x80
FLAG_RELATIVE = 0x40
FLAG_STEP_MODE = 0x20
FLAG_RESERVED = 0x10
MOTOR_ID_MASK = 0x0F

MOVE_CAN_ID_BY_BOARD_ID = {
    BOARD_ID_BOARD1: CAN_ID_BOARD1_POSITION_COMMAND,
    BOARD_ID_BOARD2: CAN_ID_BOARD2_POSITION_COMMAND,
    BOARD_ID_BOARD3: CAN_ID_BOARD3_SERVO_COMMAND,
}

BOARD_ID_BY_MOVE_CAN_ID = {
    can_id: board_id
    for board_id, can_id in MOVE_CAN_ID_BY_BOARD_ID.items()
}

STATUS_CAN_ID_BY_BOARD_ID = {
    BOARD_ID_BOARD1: CAN_ID_BOARD1_STATUS,
    BOARD_ID_BOARD2: CAN_ID_BOARD2_STATUS,
    BOARD_ID_BOARD3: CAN_ID_BOARD3_STATUS,
}

BOARD_ID_BY_STATUS_CAN_ID = {
    can_id: board_id
    for board_id, can_id in STATUS_CAN_ID_BY_BOARD_ID.items()
}

STATUS_CAN_IDS = tuple(STATUS_CAN_ID_BY_BOARD_ID.values())
POSITION_FEEDBACK_CAN_ID_BY_BOARD_ID = {
    BOARD_ID_BOARD1: CAN_ID_BOARD1_POSITION_FEEDBACK,
    BOARD_ID_BOARD2: CAN_ID_BOARD2_POSITION_FEEDBACK,
    BOARD_ID_BOARD3: CAN_ID_BOARD3_POSITION_FEEDBACK,
}
BOARD_ID_BY_POSITION_FEEDBACK_CAN_ID = {
    can_id: board_id
    for board_id, can_id in POSITION_FEEDBACK_CAN_ID_BY_BOARD_ID.items()
}
POSITION_FEEDBACK_CAN_IDS = tuple(
    POSITION_FEEDBACK_CAN_ID_BY_BOARD_ID.values()
)
RECEIVE_CAN_IDS = STATUS_CAN_IDS + POSITION_FEEDBACK_CAN_IDS


class BoardState(IntEnum):
    """State values reported in status byte 0."""

    INIT = 0
    IDLE = 1
    HOMING = 2
    STAGING = 2
    MOVING = 3
    ERROR = 4
    ESTOP = 5
    DISABLED = 6


class BoardError(IntEnum):
    """Error values reported in status byte 1."""

    NONE = 0
    INVALID_CMD = 1
    LIMIT_DETECTED = 2
    DRIVER_FAULT = 3
    HOMING_FAIL = 4
    QUEUE_FULL = 5
    RESERVED = 6


class Board3FeedbackMotorStatus(IntEnum):
    """Per-motor status codes packed into Board3 0x303 Byte7."""

    OK = 0
    MOVING = 1
    CONTACT_HOLD = 2
    ERROR = 3


BOARD3_ERROR_NAMES = {
    0: 'ERR_NONE',
    1: 'ERR_INVALID_CMD',
    2: 'ERR_INVALID_MOTOR_ID',
    3: 'ERR_DUPLICATE_MOTOR_ID',
    4: 'ERR_STAGING_TIMEOUT',
    5: 'ERR_DURATION_MISMATCH',
    6: 'ERR_ANGLE_RANGE',
    7: 'ERR_SERVO_COMM',
    8: 'ERR_SERVO_FAULT',
    9: 'ERR_ESTOP',
    10: 'ERR_DISABLED',
}


def error_name_for_board(value: int, board_id: int) -> str:
    """Return a protocol-aware error name for one board status value."""
    if validate_board_id(board_id) == BOARD_ID_BOARD3:
        return BOARD3_ERROR_NAMES.get(int(value), f'UNKNOWN({value})')

    try:
        return BoardError(int(value)).name
    except ValueError:
        return f'UNKNOWN({value})'


@dataclass(frozen=True)
class CanFrame:
    """A standard 11-bit Classic CAN frame with up to eight data bytes."""

    can_id: int
    data: bytes

    def __post_init__(self) -> None:
        """Validate the CAN identifier and payload length."""
        if not 0 <= self.can_id <= 0x7FF:
            raise ValueError(
                f'CAN ID must be an 11-bit standard ID: {self.can_id:#x}'
            )

        if not 0 <= len(self.data) <= 8:
            raise ValueError(
                f'Classic CAN data length must be 0..8, got {len(self.data)}'
            )


@dataclass(frozen=True)
class PositionControl:
    """Decoded control flags and motor identifier from command byte 0."""

    execute: bool
    relative: bool
    step_mode: bool
    reserved: bool
    motor_id: int


@dataclass(frozen=True)
class BoardStatus:
    """Decoded payload of CAN ID 0x201, 0x202, or 0x203."""

    state: int
    error_code: int
    homing_done_bits: int
    moving_motor_id: int
    limit_status_bits: int
    queue_free: int
    enabled: bool
    reserved: int
    board_id: int = BOARD_ID_BOARD1

    @property
    def board3_staging_count(self) -> int:
        """Return Board3 staging count carried in status byte 3."""
        return self.moving_motor_id

    @property
    def board3_buffer_free(self) -> int:
        """Return Board3 staging-buffer free count carried in byte 5."""
        return self.queue_free

    @property
    def board3_fault_motor_id(self) -> int:
        """Return Board3 fault motor ID carried in status byte 7."""
        return self.reserved

    @property
    def all_required_axes_homed(self) -> bool:
        """Return whether this board's homing or ready requirement is met."""
        if self.board_id == BOARD_ID_BOARD1:
            return (
                self.homing_done_bits & REQUIRED_HOMING_MASK
            ) == REQUIRED_HOMING_MASK
        if self.board_id == BOARD_ID_BOARD2:
            return (
                self.homing_done_bits & BOARD2_REQUIRED_HOMING_MASK
            ) == BOARD2_REQUIRED_HOMING_MASK
        if self.board_id == BOARD_ID_BOARD3:
            return self.homing_done_bits == BOARD3_READY_VALUE
        return False

    @property
    def has_fault(self) -> bool:
        """Return whether this status reports a limit or servo fault."""
        if self.board_id == BOARD_ID_BOARD3:
            return self.limit_status_bits != 0
        return False

    @property
    def healthy(self) -> bool:
        """Return whether the board is not reporting an error or ESTOP."""
        return (
            self.error_code == BoardError.NONE
            and self.state not in (
                BoardState.ERROR,
                BoardState.ESTOP,
                BoardState.DISABLED,
            )
            and not self.has_fault
        )

    @property
    def prepared_for_trajectory(self) -> bool:
        """Return whether trajectory commands may be streamed safely."""
        return (
            self.healthy
            and self.enabled
            and self.all_required_axes_homed
            and self.state in (BoardState.IDLE, BoardState.MOVING)
        )

    @property
    def trajectory_complete(self) -> bool:
        """Infer that all queued commands have finished executing."""
        if self.board_id == BOARD_ID_BOARD3:
            return (
                self.healthy
                and self.state == BoardState.IDLE
                and self.board3_staging_count == 0
            )

        return (
            self.healthy
            and self.state == BoardState.IDLE
            and self.moving_motor_id == ALL_MOTORS
        )


@dataclass(frozen=True)
class Board3PositionFeedbackGroup:
    """Decoded one-third of Board3 actual position feedback CAN ID 0x303."""

    group_index: int
    motor_ids: tuple[int, ...]
    positions_raw: tuple[int, ...]
    positions_rad: tuple[float, ...]
    status_codes: tuple[int, ...]
    valid: bool
    fault: bool
    raw_flags: int


@dataclass(frozen=True)
class MotorPositionFeedback:
    """Decoded actual-position feedback for Board1/2 CAN ID 0x301/0x302."""

    board_id: int
    motor_id: int
    flags: int
    position_raw: int
    position_rad: float
    error_code: int
    sequence: int

    @property
    def position_valid(self) -> bool:
        """Return whether the position field is valid."""
        return bool(self.flags & 0x01)

    @property
    def homed(self) -> bool:
        """Return whether this motor is homed or ready."""
        return bool(self.flags & 0x02)

    @property
    def moving(self) -> bool:
        """Return whether this motor is currently moving."""
        return bool(self.flags & 0x04)

    @property
    def target_reached(self) -> bool:
        """Return whether this motor reports target reached."""
        return bool(self.flags & 0x08)


def rad_to_angle_raw(radian: float) -> int:
    """Convert radians to signed 0.01-degree units."""
    if not math.isfinite(radian):
        raise ValueError(f'Radian must be finite: {radian}')

    value = round(radian * 18_000.0 / math.pi)

    if not -(2**31) <= value <= (2**31 - 1):
        raise OverflowError(f'Angle does not fit int32: {value}')

    return int(value)


def angle_raw_to_rad(angle_raw: int) -> float:
    """Convert signed 0.01-degree units to radians."""
    return float(angle_raw) * math.pi / 18_000.0


def duration_ns_to_ticks(duration_ns: int) -> int:
    """Convert nanoseconds to rounded-up 5 ms uint8 duration ticks."""
    if duration_ns <= 0:
        raise ValueError('Segment duration must be greater than zero')

    ticks = math.ceil(duration_ns / DURATION_TICK_NS)

    if ticks > MAX_DURATION_TICKS:
        raise OverflowError(
            'Segment duration exceeds the 1275 ms protocol limit: '
            f'{duration_ns} ns'
        )

    return int(max(1, ticks))


def validate_board_id(board_id: int, *, allow_all: bool = False) -> int:
    """Return a normalized board id or raise ValueError."""
    normalized = int(board_id)
    valid_ids = {BOARD_ID_BOARD1, BOARD_ID_BOARD2, BOARD_ID_BOARD3}

    if allow_all:
        valid_ids.add(BOARD_ID_ALL_LEGACY)
        valid_ids.add(BOARD_ID_ALL)

    if normalized not in valid_ids:
        allowed = ', '.join(str(value) for value in sorted(valid_ids))
        raise ValueError(
            f'board_id must be one of {allowed}, got {normalized}'
        )

    return normalized


def move_can_id_for_board(board_id: int) -> int:
    """Return the position/servo command CAN ID for a board."""
    return MOVE_CAN_ID_BY_BOARD_ID[validate_board_id(board_id)]


def status_can_id_for_board(board_id: int) -> int:
    """Return the status CAN ID for a board."""
    return STATUS_CAN_ID_BY_BOARD_ID[validate_board_id(board_id)]


def board_id_from_status_can_id(can_id: int) -> int:
    """Return the board id that owns one status CAN ID."""
    try:
        return BOARD_ID_BY_STATUS_CAN_ID[int(can_id)]
    except KeyError as exc:
        raise ValueError(f'Unsupported status CAN ID: {can_id:#x}') from exc


def board_id_from_move_can_id(can_id: int) -> int:
    """Return the board id that owns one move CAN ID."""
    try:
        return BOARD_ID_BY_MOVE_CAN_ID[int(can_id)]
    except KeyError as exc:
        raise ValueError(f'Unsupported move CAN ID: {can_id:#x}') from exc


def board_id_from_position_feedback_can_id(can_id: int) -> int:
    """Return the board id that owns one position feedback CAN ID."""
    try:
        return BOARD_ID_BY_POSITION_FEEDBACK_CAN_ID[int(can_id)]
    except KeyError as exc:
        raise ValueError(
            f'Unsupported position feedback CAN ID: {can_id:#x}'
        ) from exc


def motor_count_for_board(board_id: int) -> int:
    """Return the valid local motor/servo count for a board."""
    normalized = validate_board_id(board_id)

    if normalized == BOARD_ID_BOARD1:
        return BOARD1_MOTOR_COUNT
    if normalized == BOARD_ID_BOARD2:
        return BOARD2_MOTOR_COUNT
    return BOARD3_SERVO_COUNT


def build_control_byte(
    motor_id: int,
    *,
    execute: bool = True,
    relative: bool = False,
    step_mode: bool = False,
    reserved: bool = False,
) -> int:
    """Build command byte 0 from flags and a local motor identifier."""
    if not 0 <= int(motor_id) <= MOTOR_ID_MASK:
        raise ValueError(f'motor_id must fit 0..15, got {motor_id}')

    value = int(motor_id) & MOTOR_ID_MASK

    if execute:
        value |= FLAG_EXECUTE
    if relative:
        value |= FLAG_RELATIVE
    if step_mode:
        value |= FLAG_STEP_MODE
    if reserved:
        value |= FLAG_RESERVED

    return value


def decode_control_byte(value: int) -> PositionControl:
    """Decode command byte 0 into named flags and the motor identifier."""
    if not 0 <= int(value) <= 0xFF:
        raise ValueError('Control byte must fit uint8')

    return PositionControl(
        execute=bool(value & FLAG_EXECUTE),
        relative=bool(value & FLAG_RELATIVE),
        step_mode=bool(value & FLAG_STEP_MODE),
        reserved=bool(value & FLAG_RESERVED),
        motor_id=int(value) & MOTOR_ID_MASK,
    )


def pack_position_command(
    motor_id: int,
    target_pos: int,
    speed: int,
    duration_ticks: int,
    *,
    board_id: int = BOARD_ID_BOARD1,
    execute: bool = True,
    relative: bool = False,
    step_mode: bool = False,
) -> CanFrame:
    """Pack one motor or servo trajectory point."""
    normalized_board_id = validate_board_id(board_id)

    if not 0 <= int(motor_id) < motor_count_for_board(normalized_board_id):
        raise ValueError(
            f'motor_id {motor_id} is invalid for board {normalized_board_id}'
        )
    if not -(2**31) <= int(target_pos) <= (2**31 - 1):
        raise OverflowError('target_pos must fit signed int32')
    if not 0 <= int(speed) <= 0xFFFF:
        raise ValueError('speed must fit uint16')
    if not 0 <= int(duration_ticks) <= MAX_DURATION_TICKS:
        raise ValueError('duration_ticks must fit uint8')

    control = build_control_byte(
        int(motor_id),
        execute=execute,
        relative=relative,
        step_mode=step_mode,
        reserved=False,
    )

    data = struct.pack(
        '<BiHB',
        control,
        int(target_pos),
        int(speed),
        int(duration_ticks),
    )
    return CanFrame(move_can_id_for_board(normalized_board_id), data)


def _reserved_payload(*values: int) -> bytes:
    """Return an eight-byte command payload padded with reserved zeros."""
    if len(values) > 8:
        raise ValueError('Classic CAN command payload cannot exceed 8 bytes')
    return bytes(int(value) & 0xFF for value in values) + bytes(
        8 - len(values)
    )


def pack_estop(board_id: int = BOARD_ID_ALL) -> CanFrame:
    """Pack the broadcast emergency-stop command."""
    # Byte 0 is an ESTOP request flag, not a target board id.  The argument is
    # retained for compatibility with older call sites but is not encoded.
    validate_board_id(board_id, allow_all=True)
    return CanFrame(CAN_ID_ESTOP, _reserved_payload(1))


def pack_enable(
    enabled: bool,
    board_id: int = BOARD_ID_ALL,
) -> CanFrame:
    """Pack the common motor/servo enable or disable command."""
    normalized_board_id = validate_board_id(board_id, allow_all=True)
    return CanFrame(
        CAN_ID_ENABLE,
        _reserved_payload(1 if enabled else 0, normalized_board_id),
    )


def pack_homing(
    motor_id: int = ALL_MOTORS,
    mode: int = 0,
    *,
    board_id: int = BOARD_ID_ALL,
) -> CanFrame:
    """Pack a homing command for Board1, Board2, Board3, or broadcast."""
    normalized_board_id = validate_board_id(board_id, allow_all=True)

    if mode != 0:
        raise ValueError('Only homing mode 0 is supported')
    if motor_id != ALL_MOTORS:
        if normalized_board_id in (BOARD_ID_ALL_LEGACY, BOARD_ID_ALL):
            raise ValueError(
                'Broadcast homing must target all local motors'
            )
        if normalized_board_id == BOARD_ID_BOARD3:
            raise ValueError('Board3 homing supports only all gripper motors')
        else:
            valid_motor_count = motor_count_for_board(normalized_board_id)
            if not 0 <= int(motor_id) < valid_motor_count:
                raise ValueError('Homing motor_id is invalid for board')

    return CanFrame(
        CAN_ID_HOMING,
        _reserved_payload(normalized_board_id, int(motor_id), int(mode)),
    )


def pack_clear_error(
    motor_id: int = ALL_MOTORS,
    *,
    board_id: int = BOARD_ID_ALL,
) -> CanFrame:
    """Pack a clear-error command with a target board id."""
    normalized_board_id = validate_board_id(board_id, allow_all=True)

    if motor_id != ALL_MOTORS:
        if normalized_board_id in (BOARD_ID_ALL_LEGACY, BOARD_ID_ALL):
            raise ValueError(
                'Broadcast clear-error must target all local motors'
            )
        if normalized_board_id == BOARD_ID_BOARD3:
            raise ValueError(
                'Board3 clear-error supports only all gripper motors'
            )
        else:
            valid_motor_count = motor_count_for_board(normalized_board_id)
            if not 0 <= int(motor_id) < valid_motor_count:
                raise ValueError('Clear-error motor_id is invalid for board')

    return CanFrame(
        CAN_ID_CLEAR_ERROR,
        _reserved_payload(normalized_board_id, int(motor_id)),
    )


def unpack_status(
    data: bytes,
    *,
    board_id: int = BOARD_ID_BOARD1,
) -> BoardStatus:
    """Decode the eight-byte board status payload."""
    normalized_board_id = validate_board_id(board_id)

    if len(data) != 8:
        raise ValueError('Board status payload must contain 8 bytes')

    values = struct.unpack('<BBBBBBBB', data)

    return BoardStatus(
        board_id=normalized_board_id,
        state=values[0],
        error_code=values[1],
        homing_done_bits=values[2],
        moving_motor_id=values[3],
        limit_status_bits=values[4],
        queue_free=values[5],
        enabled=bool(values[6]),
        reserved=values[7],
    )


def unpack_motor_position_feedback(
    data: bytes,
    *,
    board_id: int,
) -> MotorPositionFeedback:
    """Decode one Board1/2 actual-position feedback payload."""
    normalized_board_id = validate_board_id(board_id)
    if normalized_board_id == BOARD_ID_BOARD3:
        raise ValueError('Board3 position feedback uses compressed groups')

    if len(data) != 8:
        raise ValueError(
            'Motor position feedback payload must contain 8 bytes'
        )

    motor_id = int(data[0])
    if not 0 <= motor_id < motor_count_for_board(normalized_board_id):
        raise ValueError(
            f'motor_id {motor_id} is invalid for board {normalized_board_id}'
        )

    flags = int(data[1])
    if flags & 0xF0:
        raise ValueError(
            f'Motor position feedback reserved flag bits set: {flags:#04x}'
        )

    position_raw = struct.unpack_from('<i', data, 2)[0]
    return MotorPositionFeedback(
        board_id=normalized_board_id,
        motor_id=motor_id,
        flags=flags,
        position_raw=int(position_raw),
        position_rad=angle_raw_to_rad(position_raw),
        error_code=int(data[6]),
        sequence=int(data[7]),
    )


def unpack_board3_position_feedback(
    data: bytes,
) -> Board3PositionFeedbackGroup:
    """Decode one Board3 compressed actual-position feedback group."""
    if len(data) != 8:
        raise ValueError(
            'Board3 position feedback payload must contain 8 bytes'
        )

    group_index = int(data[0])
    if group_index not in (1, 2, 3):
        raise ValueError(
            f'Board3 position feedback group must be 1..3, '
            f'got {group_index}'
        )

    raw_positions = tuple(
        int(value)
        for value in struct.unpack_from('<hhh', data, 1)
    )
    positions_rad = tuple(
        angle_raw_to_rad(value)
        for value in raw_positions
    )

    raw_flags = int(data[7])
    status_codes = (
        raw_flags & 0x03,
        (raw_flags >> 2) & 0x03,
        (raw_flags >> 4) & 0x03,
    )
    first_motor_id = (group_index - 1) * 3
    motor_ids = tuple(range(first_motor_id, first_motor_id + 3))

    return Board3PositionFeedbackGroup(
        group_index=group_index,
        motor_ids=motor_ids,
        positions_raw=raw_positions,
        positions_rad=positions_rad,
        status_codes=status_codes,
        valid=bool(raw_flags & 0x40),
        fault=bool(raw_flags & 0x80),
        raw_flags=raw_flags,
    )
