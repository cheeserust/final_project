"""Convert ROS 2 joint trajectories into board CAN command batches."""

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from trajectory_msgs.msg import JointTrajectory

from .can_protocol import (
    BOARD3_TARGET_LOAD_MAX,
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD3,
    board_id_from_move_can_id,
    CanFrame,
    DURATION_TICK_NS,
    MAX_DURATION_TICKS,
    motor_count_for_board,
    pack_board3_servo_command,
    pack_position_command,
    rad_to_angle_raw,
    validate_board_id,
)


JOINT_LIMIT_EPSILON_RAD = 1e-8


class TrajectoryConversionError(ValueError):
    """Raised when a trajectory cannot be converted safely."""


@dataclass(frozen=True)
class TrajectoryBatch:
    """Synchronized board commands for one trajectory segment."""

    source_point_index: int
    duration_ticks: int
    target_positions_rad: tuple[float, ...]
    frames: tuple[CanFrame, ...]

    @property
    def queue_slots(self) -> int:
        """Return the number of STM32 queue slots required."""
        return len(self.frames)

    @property
    def queue_slots_by_board(self) -> dict[int, int]:
        """Return the number of command slots required per board."""
        slots: dict[int, int] = {}

        for frame in self.frames:
            board_id = board_id_from_move_can_id(frame.can_id)
            slots[board_id] = slots.get(board_id, 0) + 1

        return slots


class ArmTrajectoryConverter:
    """Convert JointTrajectory messages into board CAN frame batches."""

    def __init__(
        self,
        joint_names: Sequence[str],
        motor_ids: Sequence[int],
        min_positions_rad: Sequence[float],
        max_positions_rad: Sequence[float],
        board_ids: Sequence[int] | None = None,
        speed_raw: int = 0,
        aux_raw_by_board: Mapping[int, int] | None = None,
        start_position_tolerance_rad: float = 0.02,
    ):
        self._joint_names = tuple(joint_names)
        self._board_ids = tuple(
            validate_board_id(value)
            for value in (
                board_ids
                if board_ids is not None
                else [BOARD_ID_BOARD1] * len(joint_names)
            )
        )
        self._motor_ids = tuple(int(value) for value in motor_ids)
        self._min_positions = tuple(
            float(value) for value in min_positions_rad
        )
        self._max_positions = tuple(
            float(value) for value in max_positions_rad
        )
        self._speed_raw = int(speed_raw)
        self._aux_raw_by_board = {
            validate_board_id(board_id): int(value)
            for board_id, value in (
                aux_raw_by_board.items()
                if aux_raw_by_board is not None
                else ()
            )
        }
        self._start_tolerance = float(
            start_position_tolerance_rad
        )

        self._validate_configuration()

    def _validate_configuration(self) -> None:
        count = len(self._joint_names)

        if count <= 0:
            raise ValueError('joint_names cannot be empty')

        if len(set(self._joint_names)) != count:
            raise ValueError('Configured joint names must be unique')

        if len(self._board_ids) != count:
            raise ValueError(
                'board_ids length must match joint_names length'
            )

        if len(self._motor_ids) != count:
            raise ValueError(
                'motor_ids length must match joint_names length'
            )

        mapping_pairs = tuple(zip(self._board_ids, self._motor_ids))
        if len(set(mapping_pairs)) != count:
            raise ValueError(
                'Configured board_id/motor_id pairs must be unique'
            )

        for board_id, motor_id in mapping_pairs:
            if not 0 <= motor_id < motor_count_for_board(board_id):
                raise ValueError(
                    f'motor_id {motor_id} is invalid for board {board_id}'
                )

        if len(self._min_positions) != count:
            raise ValueError(
                'min_positions_rad length must match joint_names'
            )

        if len(self._max_positions) != count:
            raise ValueError(
                'max_positions_rad length must match joint_names'
            )

        if not 0 <= self._speed_raw <= 0xFFFF:
            raise ValueError('speed_raw must fit uint16')

        for board_id, value in self._aux_raw_by_board.items():
            if board_id == BOARD_ID_BOARD3:
                if not 0 <= value <= BOARD3_TARGET_LOAD_MAX:
                    raise ValueError(
                        'Board3 target load must be in range '
                        f'0..{BOARD3_TARGET_LOAD_MAX}'
                    )
            elif not 0 <= value <= 0xFFFF:
                raise ValueError(
                    f'aux_raw_by_board[{board_id}] must fit uint16'
                )

        if self._start_tolerance < 0.0:
            raise ValueError(
                'start_position_tolerance_rad cannot be negative'
            )

        for index, minimum in enumerate(self._min_positions):
            maximum = self._max_positions[index]

            if minimum >= maximum:
                raise ValueError(
                    f'Invalid joint limit for '
                    f'{self._joint_names[index]}'
                )

        board1_motor_ids = {
            motor_id
            for board_id, motor_id in mapping_pairs
            if board_id == BOARD_ID_BOARD1
        }

        if board1_motor_ids and board1_motor_ids != {0, 1, 2, 3}:
            raise ValueError(
                'Board1 joints must map to motor IDs 0, 1, 2, 3'
            )

    @staticmethod
    def _duration_to_ns(duration) -> int:
        if duration.sec < 0:
            raise TrajectoryConversionError(
                'time_from_start cannot be negative'
            )

        if not 0 <= duration.nanosec < 1_000_000_000:
            raise TrajectoryConversionError(
                'time_from_start.nanosec is invalid'
            )

        return (
            int(duration.sec) * 1_000_000_000
            + int(duration.nanosec)
        )

    def _make_joint_index_map(
        self,
        received_joint_names: Sequence[str],
    ) -> tuple[int, ...]:
        names = tuple(received_joint_names)

        if len(set(names)) != len(names):
            raise TrajectoryConversionError(
                'Trajectory joint names contain duplicates'
            )

        expected = set(self._joint_names)
        received = set(names)

        if received != expected:
            missing = sorted(expected - received)
            unexpected = sorted(received - expected)

            raise TrajectoryConversionError(
                f'Joint name mismatch: '
                f'missing={missing}, unexpected={unexpected}'
            )

        index_by_name = {
            name: index
            for index, name in enumerate(names)
        }

        return tuple(
            index_by_name[name]
            for name in self._joint_names
        )

    def _reorder_positions(
        self,
        positions: Sequence[float],
        joint_indices: Sequence[int],
        point_index: int,
    ) -> tuple[float, ...]:
        if len(positions) != len(joint_indices):
            raise TrajectoryConversionError(
                f'Point {point_index} positions length does not '
                'match joint_names length'
            )

        ordered_values = [
            float(positions[index])
            for index in joint_indices
        ]

        for joint_index, position in enumerate(ordered_values):
            if not math.isfinite(position):
                raise TrajectoryConversionError(
                    f'Point {point_index} contains a non-finite '
                    f'position for {self._joint_names[joint_index]}'
                )

            minimum = self._min_positions[joint_index]
            maximum = self._max_positions[joint_index]

            if (
                position < minimum - JOINT_LIMIT_EPSILON_RAD
                or position > maximum + JOINT_LIMIT_EPSILON_RAD
            ):
                raise TrajectoryConversionError(
                    f'Point {point_index} exceeds the limit for '
                    f'{self._joint_names[joint_index]}: '
                    f'{position} not in [{minimum}, {maximum}]'
                )

            ordered_values[joint_index] = min(
                max(position, minimum),
                maximum,
            )

        return tuple(ordered_values)

    def _reorder_effort_target_loads(
        self,
        efforts: Sequence[float],
        joint_indices: Sequence[int],
        point_index: int,
    ) -> tuple[int | None, ...] | None:
        if not efforts:
            return None

        if len(efforts) != len(joint_indices):
            raise TrajectoryConversionError(
                f'Point {point_index} effort length does not '
                'match joint_names length'
            )

        target_loads: list[int | None] = []

        for joint_index, received_index in enumerate(joint_indices):
            if self._board_ids[joint_index] != BOARD_ID_BOARD3:
                target_loads.append(None)
                continue

            effort_value = float(efforts[received_index])
            if not math.isfinite(effort_value):
                raise TrajectoryConversionError(
                    f'Point {point_index} contains a non-finite '
                    f'target load for {self._joint_names[joint_index]}'
                )

            target_load = int(round(effort_value))
            if not 0 <= target_load <= BOARD3_TARGET_LOAD_MAX:
                raise TrajectoryConversionError(
                    f'Point {point_index} target load for '
                    f'{self._joint_names[joint_index]} must be in '
                    f'[0, {BOARD3_TARGET_LOAD_MAX}], got {target_load}'
                )

            target_loads.append(target_load)

        return tuple(target_loads)

    @staticmethod
    def _split_duration_ticks(
        duration_ns: int,
    ) -> tuple[int, ...]:
        if duration_ns <= 0:
            raise TrajectoryConversionError(
                'Segment duration must be greater than zero'
            )

        total_ticks = math.ceil(
            duration_ns / DURATION_TICK_NS
        )

        chunks = []

        while total_ticks > 0:
            chunk = min(total_ticks, MAX_DURATION_TICKS)
            chunks.append(int(chunk))
            total_ticks -= chunk

        return tuple(chunks)

    def _aux_raw_for_joint(self, joint_index: int) -> int:
        board_id = self._board_ids[joint_index]
        return self._aux_raw_by_board.get(board_id, self._speed_raw)

    def _build_frames(
        self,
        target_positions: Sequence[float],
        duration_ticks: int,
        target_loads_raw: Sequence[int | None] | None = None,
    ) -> tuple[CanFrame, ...]:
        frames = []

        if (
            target_loads_raw is not None
            and len(target_loads_raw) != len(self._joint_names)
        ):
            raise ValueError('target_loads_raw length must match joint_names')

        for index, target_position in enumerate(target_positions):
            board_id = self._board_ids[index]
            target_pos_raw = rad_to_angle_raw(target_position)

            if board_id == BOARD_ID_BOARD3:
                target_load = self._aux_raw_for_joint(index)
                if (
                    target_loads_raw is not None
                    and target_loads_raw[index] is not None
                ):
                    target_load = int(target_loads_raw[index])

                frame = pack_board3_servo_command(
                    motor_id=self._motor_ids[index],
                    target_pos=target_pos_raw,
                    target_load=target_load,
                    duration_ticks=duration_ticks,
                    execute=True,
                )
            else:
                frame = pack_position_command(
                    board_id=board_id,
                    motor_id=self._motor_ids[index],
                    target_pos=target_pos_raw,
                    speed=self._aux_raw_for_joint(index),
                    duration_ticks=duration_ticks,
                    execute=True,
                    relative=False,
                    step_mode=False,
                )
            frames.append(frame)

        return tuple(frames)

    def convert(
        self,
        trajectory: JointTrajectory,
        current_positions_rad: Sequence[float],
    ) -> tuple[TrajectoryBatch, ...]:
        """Convert one JointTrajectory into ordered CAN command batches."""
        if len(current_positions_rad) != len(self._joint_names):
            raise TrajectoryConversionError(
                'Current position length must match configured joints'
            )

        current_positions = tuple(
            float(value)
            for value in current_positions_rad
        )

        if not all(math.isfinite(value) for value in current_positions):
            raise TrajectoryConversionError(
                'Current positions must contain only finite values'
            )

        if not trajectory.points:
            raise TrajectoryConversionError(
                'Trajectory must contain at least one point'
            )

        joint_indices = self._make_joint_index_map(
            trajectory.joint_names
        )

        batches = []
        previous_positions = current_positions
        previous_time_ns = 0

        for point_index, point in enumerate(trajectory.points):
            target_positions = self._reorder_positions(
                point.positions,
                joint_indices,
                point_index,
            )
            target_loads_raw = self._reorder_effort_target_loads(
                point.effort,
                joint_indices,
                point_index,
            )

            point_time_ns = self._duration_to_ns(
                point.time_from_start
            )

            if point_index == 0 and point_time_ns == 0:
                max_difference = max(
                    abs(target - current)
                    for target, current in zip(
                        target_positions,
                        current_positions,
                    )
                )

                if max_difference > self._start_tolerance:
                    raise TrajectoryConversionError(
                        'The zero-time start point differs from the '
                        'current commanded position'
                    )

                previous_positions = target_positions
                continue

            if point_time_ns <= previous_time_ns:
                raise TrajectoryConversionError(
                    f'Point {point_index} time_from_start must '
                    'strictly increase'
                )

            segment_duration_ns = (
                point_time_ns - previous_time_ns
            )

            tick_chunks = self._split_duration_ticks(
                segment_duration_ns
            )

            total_ticks = sum(tick_chunks)
            completed_ticks = 0

            for chunk_index, duration_ticks in enumerate(
                tick_chunks
            ):
                completed_ticks += duration_ticks
                fraction = completed_ticks / total_ticks

                if chunk_index == len(tick_chunks) - 1:
                    intermediate_target = target_positions
                else:
                    intermediate_target = tuple(
                        start + (target - start) * fraction
                        for start, target in zip(
                            previous_positions,
                            target_positions,
                        )
                    )

                frames = self._build_frames(
                    intermediate_target,
                    duration_ticks,
                    target_loads_raw,
                )

                batches.append(
                    TrajectoryBatch(
                        source_point_index=point_index,
                        duration_ticks=duration_ticks,
                        target_positions_rad=tuple(
                            intermediate_target
                        ),
                        frames=frames,
                    )
                )

            previous_positions = target_positions
            previous_time_ns = point_time_ns

        if not batches:
            raise TrajectoryConversionError(
                'Trajectory contains no executable segment'
            )

        return tuple(batches)
