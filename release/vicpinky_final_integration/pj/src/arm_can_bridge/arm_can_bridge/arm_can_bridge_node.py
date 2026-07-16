"""ROS 2 control-services node for the arm CAN bridge."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import struct
import threading
import time
from typing import Callable, Sequence

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .board3_feedback import Board3PositionFeedbackAssembler
from .board_state import MultiBoardStateTracker
from .can_protocol import (
    ALL_MOTORS,
    angle_raw_to_rad,
    BOARD3_SERVO_COUNT,
    Board3FeedbackMotorStatus,
    BOARD_ID_ALL,
    BOARD_ID_BOARD1,
    BOARD_ID_BOARD2,
    BOARD_ID_BOARD3,
    BOARD_ID_BY_POSITION_FEEDBACK_CAN_ID,
    BOARD_ID_BY_STATUS_CAN_ID,
    BoardState,
    CAN_ID_BOARD3_POSITION_FEEDBACK,
    CanFrame,
    error_name_for_board,
    MAX_DURATION_TICKS,
    motor_count_for_board,
    MotorPositionFeedback,
    pack_clear_error,
    pack_enable,
    pack_estop,
    pack_gripper_home,
    pack_homing,
    RECEIVE_CAN_IDS,
    unpack_board3_position_feedback,
    unpack_motor_position_feedback,
    unpack_status,
)
from .commanded_state import CommandedStateEstimator
from .socketcan_transport import SocketCanTransport, SocketCanTransportError
from .trajectory_converter import (
    ArmTrajectoryConverter,
    TrajectoryConversionError,
)
from .trajectory_streamer import (
    TrajectoryCanceled,
    TrajectoryStreamer,
    TrajectoryStreamingError,
)


@dataclass
class TrajectoryControllerContext:
    """Runtime objects for one FollowJointTrajectory controller."""

    label: str
    action_name: str
    joint_names: list[str]
    board_ids: list[int]
    motor_ids: list[int]
    home_positions_rad: list[float]
    raw_position_signs: list[float]
    raw_position_offsets_rad: list[float]
    board_state: MultiBoardStateTracker
    commanded_state: CommandedStateEstimator
    trajectory_converter: ArmTrajectoryConverter
    trajectory_streamer: TrajectoryStreamer
    active: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)
    action_server: object | None = None


class ArmCanBridgeNode(Node):
    """Bridge MoveIt FollowJointTrajectory goals to STM32 CAN boards."""

    def __init__(self) -> None:
        """Create transport, board trackers, services, and action servers."""
        super().__init__('arm_can_bridge')
        self._callback_group = ReentrantCallbackGroup()

        self._declare_parameters()

        self._can_interface = str(
            self.get_parameter('can_interface').value
        )
        self._execution_mode = str(
            self.get_parameter('execution_mode').value
        ).strip().lower()
        if self._execution_mode not in {'plan_only', 'hardware'}:
            raise ValueError(
                'execution_mode must be plan_only or hardware'
            )
        self._control_wait_timeout_s = (
            int(self.get_parameter('control_wait_timeout_ms').value)
            / 1000.0
        )
        self._homing_wait_timeout_s = (
            int(self.get_parameter('homing_wait_timeout_ms').value)
            / 1000.0
        )
        self._arm_enabled = bool(
            self.get_parameter('enable_arm').value
        )
        self._gripper_enabled = bool(
            self.get_parameter('enable_gripper').value
        )
        self._packed_position_feedback_board_ids = (
            self._configured_board_id_set(
                'packed_position_feedback_board_ids'
            )
        )
        if bool(self.get_parameter('board1_packed_position_feedback').value):
            self._packed_position_feedback_board_ids.add(BOARD_ID_BOARD1)

        self._axis_status_flags_board_ids = self._configured_board_id_set(
            'axis_status_flags_board_ids'
        )
        self._ready_bits_from_fault_board_ids = self._configured_board_id_set(
            'ready_bits_from_fault_board_ids'
        )
        if bool(self.get_parameter('board1_ready_bits_from_fault_bits').value):
            self._ready_bits_from_fault_board_ids.add(BOARD_ID_BOARD1)

        self._idle_moving_motor_id_board_ids = self._configured_board_id_set(
            'idle_moving_motor_id_board_ids'
        )
        board1_idle_moving_motor_id = int(
            self.get_parameter('board1_idle_moving_motor_id').value
        )
        if board1_idle_moving_motor_id != ALL_MOTORS:
            self._idle_moving_motor_id_board_ids.add(BOARD_ID_BOARD1)
        self._idle_moving_motor_id = int(
            self.get_parameter('idle_moving_motor_id').value
        )
        if board1_idle_moving_motor_id != ALL_MOTORS:
            self._idle_moving_motor_id = board1_idle_moving_motor_id
        self._board3_feedback = Board3PositionFeedbackAssembler()
        self._arm_feedback_lock = threading.RLock()
        self._arm_feedback_positions_rad: list[float | None] = []
        self._arm_feedback_joint_by_board_motor: dict[
            tuple[int, int],
            int,
        ] = {}
        self._arm_raw_position_sign_by_board_motor: dict[
            tuple[int, int],
            float,
        ] = {}
        self._arm_raw_position_offset_by_board_motor: dict[
            tuple[int, int],
            float,
        ] = {}

        self._status_publisher = self.create_publisher(
            String,
            '/arm_board/status_log',
            10,
        )

        self._transport = SocketCanTransport(
            interface_name=self._can_interface,
            receive_ids=RECEIVE_CAN_IDS,
            frame_callback=self._handle_can_frame,
            error_callback=self._handle_transport_error,
        )

        self._arm_controller = None
        controllers = []
        if self._arm_enabled:
            self._arm_controller = self._create_controller_context(
                label='arm',
                action_name_param='arm_action_name',
                joint_names_param='arm_joint_names',
                board_ids_param='arm_board_ids',
                motor_ids_param='arm_motor_ids',
                min_positions_param='arm_min_positions_rad',
                max_positions_param='arm_max_positions_rad',
                home_positions_param='arm_home_positions_rad',
            )
            controllers.append(self._arm_controller)

        self._gripper_controller = None
        if self._gripper_enabled:
            self._gripper_controller = self._create_controller_context(
                label='gripper',
                action_name_param='gripper_action_name',
                joint_names_param='gripper_joint_names',
                board_ids_param='gripper_board_ids',
                motor_ids_param='gripper_motor_ids',
                min_positions_param='gripper_min_positions_rad',
                max_positions_param='gripper_max_positions_rad',
                home_positions_param='gripper_home_positions_rad',
            )
            controllers.append(self._gripper_controller)
        if not controllers:
            raise ValueError('At least one of enable_arm or enable_gripper must be true')
        self._controllers = tuple(controllers)
        self._configure_fixed_joint_states()
        self._validate_combined_joint_names()
        self._configure_arm_position_feedback()
        self._transport.open()

        self._services = [
            self.create_service(
                Trigger,
                '/arm_board/enable',
                self._handle_enable,
                callback_group=self._callback_group,
            ),
            self.create_service(
                Trigger,
                '/arm_board/disable',
                self._handle_disable,
                callback_group=self._callback_group,
            ),
            self.create_service(
                Trigger,
                '/arm_board/home_all',
                self._handle_home_all,
                callback_group=self._callback_group,
            ),
            self.create_service(
                Trigger,
                '/arm_board/clear_error',
                self._handle_clear_error,
                callback_group=self._callback_group,
            ),
            self.create_service(
                Trigger,
                '/arm_board/estop',
                self._handle_estop,
                callback_group=self._callback_group,
            ),
            self.create_service(
                Trigger,
                '/arm_board/status',
                self._handle_status_request,
                callback_group=self._callback_group,
            ),
        ]

        status_publish_period_s = (
            int(self.get_parameter('status_publish_period_ms').value)
            / 1000.0
        )
        self._status_timer = self.create_timer(
            status_publish_period_s,
            self._publish_status_log,
            callback_group=self._callback_group,
        )

        joint_states_topic = str(
            self.get_parameter('joint_states_topic').value
        )
        self._joint_state_publisher = self.create_publisher(
            JointState,
            joint_states_topic,
            10,
        )

        joint_state_rate_hz = float(
            self.get_parameter('joint_state_rate_hz').value
        )
        self._joint_state_timer = self.create_timer(
            1.0 / joint_state_rate_hz,
            self._publish_joint_states,
            callback_group=self._callback_group,
        )

        self._action_servers = []
        for controller in self._controllers:
            controller.action_server = ActionServer(
                self,
                FollowJointTrajectory,
                controller.action_name,
                execute_callback=(
                    lambda goal_handle, active_controller=controller:
                    self._execute_follow_joint_trajectory(
                        goal_handle,
                        active_controller,
                    )
                ),
                goal_callback=(
                    lambda goal_request, active_controller=controller:
                    self._goal_callback_follow_joint_trajectory(
                        goal_request,
                        active_controller,
                    )
                ),
                cancel_callback=(
                    lambda goal_handle, active_controller=controller:
                    self._cancel_callback_follow_joint_trajectory(
                        goal_handle,
                        active_controller,
                    )
                ),
                callback_group=self._callback_group,
            )
            self._action_servers.append(controller.action_server)

        self.get_logger().info(
            f'arm_can_bridge started on {self._can_interface}; '
            f'execution_mode={self._execution_mode}'
        )
        self.get_logger().info(
            'Action servers ready: '
            + ', '.join(
                controller.action_name
                for controller in self._controllers
            )
        )
        self.get_logger().info(
            'Services ready: /arm_board/enable, /arm_board/disable, '
            '/arm_board/home_all, /arm_board/clear_error, '
            '/arm_board/estop, /arm_board/status'
        )
        if self._execution_mode == 'plan_only':
            self.get_logger().warning(
                'Plan-only mode: trajectory goals and state-changing arm '
                'services are rejected; disable and ESTOP remain available'
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('execution_mode', 'plan_only')
        self.declare_parameter('status_timeout_ms', 500)
        self.declare_parameter('queue_capacity', 32)
        self.declare_parameter('board1_queue_capacity', 124)
        self.declare_parameter('board2_queue_capacity', 127)
        self.declare_parameter('required_homing_mask', 0x0F)
        self.declare_parameter('control_wait_timeout_ms', 3000)
        self.declare_parameter('homing_wait_timeout_ms', 120000)
        self.declare_parameter('status_publish_period_ms', 500)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_state_rate_hz', 50.0)
        self.declare_parameter('enable_arm', True)
        self.declare_parameter('enable_gripper', True)
        self.declare_parameter('arm_speed_raw', 0)
        self.declare_parameter('gripper_target_load_raw', 500)
        # Backward-compatible alias for older config files.
        self.declare_parameter('speed_raw', 0)
        self.declare_parameter('queue_wait_timeout_ms', 3000)
        self.declare_parameter('completion_grace_ms', 3000)
        self.declare_parameter('arm_inter_frame_delay_ms', 7.0)
        self.declare_parameter('board3_inter_frame_delay_ms', 3.0)
        self.declare_parameter('arm_trajectory_point_duration_ticks', 8)
        self.declare_parameter('arm_trajectory_min_duration_ticks', 8)
        self.declare_parameter('arm_post_home_escape_duration_ticks', 60)
        self.declare_parameter('arm_max_ahead_points', 4)
        self.declare_parameter('disable_on_trajectory_error', False)
        self.declare_parameter('start_position_tolerance_rad', 0.02)
        self.declare_parameter('packed_position_feedback_board_ids', [
            BOARD_ID_BOARD1,
            BOARD_ID_BOARD2,
        ])
        self.declare_parameter('axis_status_flags_board_ids', [
            BOARD_ID_BOARD1,
            BOARD_ID_BOARD2,
        ])
        self.declare_parameter('ready_bits_from_fault_board_ids', [0])
        self.declare_parameter('idle_moving_motor_id_board_ids', [0])
        self.declare_parameter('idle_moving_motor_id', ALL_MOTORS)
        self.declare_parameter('board1_packed_position_feedback', False)
        self.declare_parameter('board1_ready_bits_from_fault_bits', False)
        self.declare_parameter('board1_idle_moving_motor_id', ALL_MOTORS)

        self.declare_parameter(
            'arm_action_name',
            '/arm_controller/follow_joint_trajectory',
        )
        self.declare_parameter('arm_joint_names', [
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'base_joint',
            'arm_joint_4',
        ])
        self.declare_parameter(
            'arm_board_ids',
            [
                BOARD_ID_BOARD1,
                BOARD_ID_BOARD1,
                BOARD_ID_BOARD1,
                BOARD_ID_BOARD1,
                BOARD_ID_BOARD2,
            ],
        )
        self.declare_parameter('arm_motor_ids', [0, 1, 2, 3, 0])
        self.declare_parameter('arm_min_positions_rad', [
            -1.50970980,
            -1.36310215,
            -1.59697627,
            -1.57079633,
            -1.57079633,
        ])
        self.declare_parameter('arm_max_positions_rad', [
            1.57079633,
            1.39626340,
            1.57079633,
            3.14159265,
            1.57079633,
        ])
        self.declare_parameter('arm_home_positions_rad', [
            -1.50970980,
            -1.36310215,
            -1.59697627,
            -1.57079633,
            -1.57079633,
        ])
        self.declare_parameter('arm_raw_position_signs', [
            1,
            1,
            1,
            1,
            1,
        ])
        self.declare_parameter('arm_raw_position_offsets_rad', [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ])
        self.declare_parameter('arm_command_min_angle_raw', [
            -8500,
            -7810,
            -9150,
            -9000,
            -9000,
        ])
        self.declare_parameter('arm_command_max_angle_raw', [
            9000,
            8000,
            9000,
            18000,
            9000,
        ])
        self.declare_parameter(
            'fixed_joint_state_names',
            Parameter.Type.STRING_ARRAY,
        )
        self.declare_parameter(
            'fixed_joint_state_positions_rad',
            Parameter.Type.DOUBLE_ARRAY,
        )

        self.declare_parameter(
            'gripper_action_name',
            '/gripper_controller/follow_joint_trajectory',
        )
        self.declare_parameter('gripper_joint_names', [
            'finger_1_base_joint',
            'finger_1_middle_joint',
            'finger_1_tip_joint',
            'finger_2_base_joint',
            'finger_2_middle_joint',
            'finger_2_tip_joint',
            'finger_3_base_joint',
            'finger_3_middle_joint',
            'finger_3_tip_joint',
        ])
        self.declare_parameter(
            'gripper_board_ids',
            [
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
                BOARD_ID_BOARD3,
            ],
        )
        self.declare_parameter(
            'gripper_motor_ids',
            [0, 1, 2, 3, 4, 5, 6, 7, 8],
        )
        self.declare_parameter('gripper_min_positions_rad', [
            -1.22696646,
            -2.40331838,
            -1.94255146,
            -1.22696646,
            -2.40331838,
            -1.94255146,
            -1.22696646,
            -2.40331838,
            -1.94255146,
        ])
        self.declare_parameter('gripper_max_positions_rad', [
            1.22696646,
            0.91978852,
            1.94255146,
            1.22696646,
            0.91978852,
            1.94255146,
            1.22696646,
            0.91978852,
            1.94255146,
        ])
        self.declare_parameter('gripper_home_positions_rad', [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ])
        self.declare_parameter('gripper_raw_position_signs', [
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
        ])
        self.declare_parameter('gripper_raw_position_offsets_rad', [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ])

    def _create_controller_context(
        self,
        *,
        label: str,
        action_name_param: str,
        joint_names_param: str,
        board_ids_param: str,
        motor_ids_param: str,
        min_positions_param: str,
        max_positions_param: str,
        home_positions_param: str,
    ) -> TrajectoryControllerContext:
        joint_names = self._list_parameter(joint_names_param, str)
        board_ids = self._list_parameter(board_ids_param, int)
        motor_ids = self._list_parameter(motor_ids_param, int)
        min_positions_rad = self._list_parameter(
            min_positions_param,
            float,
        )
        max_positions_rad = self._list_parameter(
            max_positions_param,
            float,
        )
        home_positions_rad = self._list_parameter(
            home_positions_param,
            float,
        )
        raw_position_signs = self._raw_position_signs_for_controller(
            label,
            len(joint_names),
        )
        raw_position_offsets_rad = self._raw_position_offsets_for_controller(
            label,
            len(joint_names),
        )

        self._validate_controller_lengths(
            label=label,
            joint_names=joint_names,
            board_ids=board_ids,
            motor_ids=motor_ids,
            min_positions_rad=min_positions_rad,
            max_positions_rad=max_positions_rad,
            home_positions_rad=home_positions_rad,
            raw_position_signs=raw_position_signs,
            raw_position_offsets_rad=raw_position_offsets_rad,
        )

        board_state = self._create_board_state(board_ids)
        commanded_state = CommandedStateEstimator(
            joint_names=joint_names,
        )
        trajectory_converter = ArmTrajectoryConverter(
            joint_names=joint_names,
            board_ids=board_ids,
            motor_ids=motor_ids,
            min_positions_rad=min_positions_rad,
            max_positions_rad=max_positions_rad,
            speed_raw=self._speed_raw_for_controller(label),
            aux_raw_by_board=self._aux_raw_by_board_for_controller(label),
            start_position_tolerance_rad=float(
                self.get_parameter('start_position_tolerance_rad').value
            ),
            raw_position_signs=raw_position_signs,
            raw_position_offsets_rad=raw_position_offsets_rad,
            max_segment_duration_ticks=(
                int(
                    self.get_parameter(
                        'arm_trajectory_point_duration_ticks'
                    ).value
                )
                if label == 'arm'
                else MAX_DURATION_TICKS
            ),
            min_segment_duration_ticks=(
                int(
                    self.get_parameter(
                        'arm_trajectory_min_duration_ticks'
                    ).value
                )
                if label == 'arm'
                else 1
            ),
            command_min_angle_raw=(
                self._list_parameter('arm_command_min_angle_raw', int)
                if label == 'arm'
                else None
            ),
            command_max_angle_raw=(
                self._list_parameter('arm_command_max_angle_raw', int)
                if label == 'arm'
                else None
            ),
        )
        trajectory_streamer = TrajectoryStreamer(
            board_state=board_state,
            transport=self._transport,
            queue_wait_timeout_ms=int(
                self.get_parameter('queue_wait_timeout_ms').value
            ),
            completion_grace_ms=int(
                self.get_parameter('completion_grace_ms').value
            ),
            arm_inter_frame_delay_ms=float(
                self.get_parameter('arm_inter_frame_delay_ms').value
            ),
            board3_inter_frame_delay_ms=float(
                self.get_parameter('board3_inter_frame_delay_ms').value
            ),
            max_in_flight_batches=(
                int(self.get_parameter('arm_max_ahead_points').value)
                if label == 'arm'
                else 0
            ),
        )

        return TrajectoryControllerContext(
            label=label,
            action_name=str(self.get_parameter(action_name_param).value),
            joint_names=joint_names,
            board_ids=board_ids,
            motor_ids=motor_ids,
            home_positions_rad=home_positions_rad,
            raw_position_signs=raw_position_signs,
            raw_position_offsets_rad=raw_position_offsets_rad,
            board_state=board_state,
            commanded_state=commanded_state,
            trajectory_converter=trajectory_converter,
            trajectory_streamer=trajectory_streamer,
        )

    def _speed_raw_for_controller(self, label: str) -> int:
        if label != 'arm':
            return 0

        arm_speed_raw = int(self.get_parameter('arm_speed_raw').value)
        legacy_speed_raw = int(self.get_parameter('speed_raw').value)

        if arm_speed_raw == 0 and legacy_speed_raw != 0:
            return legacy_speed_raw

        return arm_speed_raw

    def _aux_raw_by_board_for_controller(self, label: str) -> dict[int, int]:
        if label != 'gripper':
            return {}

        return {
            BOARD_ID_BOARD3: int(
                self.get_parameter('gripper_target_load_raw').value
            ),
        }

    def _raw_position_signs_for_controller(
        self,
        label: str,
        count: int,
    ) -> list[float]:
        del count
        return self._list_parameter(
            f'{label}_raw_position_signs',
            float,
        )

    def _raw_position_offsets_for_controller(
        self,
        label: str,
        count: int,
    ) -> list[float]:
        del count
        return self._list_parameter(
            f'{label}_raw_position_offsets_rad',
            float,
        )

    def _create_board_state(
        self,
        board_ids: Sequence[int],
    ) -> MultiBoardStateTracker:
        unique_board_ids = list(
            dict.fromkeys(int(value) for value in board_ids)
        )
        queue_capacity = int(self.get_parameter('queue_capacity').value)
        board1_queue_capacity = int(
            self.get_parameter('board1_queue_capacity').value
        )
        board2_queue_capacity = int(
            self.get_parameter('board2_queue_capacity').value
        )

        return MultiBoardStateTracker(
            board_ids=unique_board_ids,
            status_timeout_ms=int(
                self.get_parameter('status_timeout_ms').value
            ),
            queue_capacities={
                board_id: (
                    BOARD3_SERVO_COUNT
                    if board_id == BOARD_ID_BOARD3
                    else board1_queue_capacity
                    if board_id == BOARD_ID_BOARD1
                    else board2_queue_capacity
                    if board_id == BOARD_ID_BOARD2
                    else queue_capacity
                )
                for board_id in unique_board_ids
            },
            required_homing_masks={
                BOARD_ID_BOARD1: int(
                    self.get_parameter('required_homing_mask').value
                ),
                BOARD_ID_BOARD2: 0x01,
            },
        )

    @staticmethod
    def _validate_controller_lengths(
        *,
        label: str,
        joint_names: Sequence[str],
        board_ids: Sequence[int],
        motor_ids: Sequence[int],
        min_positions_rad: Sequence[float],
        max_positions_rad: Sequence[float],
        home_positions_rad: Sequence[float],
        raw_position_signs: Sequence[float],
        raw_position_offsets_rad: Sequence[float],
    ) -> None:
        count = len(joint_names)

        for field_name, values in (
            ('board_ids', board_ids),
            ('motor_ids', motor_ids),
            ('min_positions_rad', min_positions_rad),
            ('max_positions_rad', max_positions_rad),
            ('home_positions_rad', home_positions_rad),
            ('raw_position_signs', raw_position_signs),
            ('raw_position_offsets_rad', raw_position_offsets_rad),
        ):
            if len(values) != count:
                raise ValueError(
                    f'{label}_{field_name} length must match '
                    f'{label}_joint_names length'
                )

        for index, sign in enumerate(raw_position_signs):
            if float(sign) not in (-1.0, 1.0):
                raise ValueError(
                    f'{label}_raw_position_signs[{index}] must be -1 or 1'
                )

    def _validate_combined_joint_names(self) -> None:
        all_joint_names = [
            name
            for controller in self._controllers
            for name in controller.joint_names
        ]
        all_joint_names.extend(self._fixed_joint_state_names)

        if len(set(all_joint_names)) != len(all_joint_names):
            raise ValueError(
                'Controller and fixed joint state names must not overlap'
            )

    def _configure_fixed_joint_states(self) -> None:
        names = self._optional_list_parameter(
            'fixed_joint_state_names',
            str,
        )
        positions = self._optional_list_parameter(
            'fixed_joint_state_positions_rad',
            float,
        )

        if len(names) != len(positions):
            raise ValueError(
                'fixed_joint_state_positions_rad length must match '
                'fixed_joint_state_names length'
            )

        self._fixed_joint_state_names = names
        self._fixed_joint_state_positions_rad = positions

    def _configure_arm_position_feedback(self) -> None:
        if self._arm_controller is None:
            with self._arm_feedback_lock:
                self._arm_feedback_positions_rad = []
                self._arm_feedback_joint_by_board_motor = {}
                self._arm_raw_position_sign_by_board_motor = {}
                self._arm_raw_position_offset_by_board_motor = {}
            return

        mapping = {}
        signs = {}
        offsets = {}
        for joint_index, (board_id, motor_id) in enumerate(
            zip(
                self._arm_controller.board_ids,
                self._arm_controller.motor_ids,
            )
        ):
            if board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2):
                key = (int(board_id), int(motor_id))
                mapping[key] = joint_index
                signs[key] = float(
                    self._arm_controller.raw_position_signs[joint_index]
                )
                offsets[key] = float(
                    self._arm_controller.raw_position_offsets_rad[joint_index]
                )

        with self._arm_feedback_lock:
            self._arm_feedback_positions_rad = [
                None
                for _ in self._arm_controller.joint_names
            ]
            self._arm_feedback_joint_by_board_motor = mapping
            self._arm_raw_position_sign_by_board_motor = signs
            self._arm_raw_position_offset_by_board_motor = offsets

    def _list_parameter(
        self,
        name: str,
        cast: Callable,
    ) -> list:
        value = self.get_parameter(name).value
        return [cast(item) for item in value]

    def _optional_list_parameter(
        self,
        name: str,
        cast: Callable,
    ) -> list:
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []

        if value is None:
            return []

        return [cast(item) for item in value]

    def _configured_board_id_set(self, name: str) -> set[int]:
        return {
            int(board_id)
            for board_id in self._list_parameter(name, int)
            if int(board_id) > 0
        }

    def _handle_can_frame(self, frame: CanFrame) -> None:
        position_board_id = BOARD_ID_BY_POSITION_FEEDBACK_CAN_ID.get(
            frame.can_id
        )
        if position_board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2):
            if self._arm_controller is None:
                return
            self._handle_motor_position_feedback(frame, position_board_id)
            return

        if frame.can_id == CAN_ID_BOARD3_POSITION_FEEDBACK:
            if not self._gripper_enabled:
                return
            self._handle_board3_position_feedback(frame)
            return

        board_id = BOARD_ID_BY_STATUS_CAN_ID.get(frame.can_id)
        if board_id is None:
            return
        if board_id == BOARD_ID_BOARD3 and not self._gripper_enabled:
            return

        try:
            status = unpack_status(frame.data, board_id=board_id)
            status = self._normalize_board_status(status)
            for controller in self._controllers:
                controller.board_state.update_status(status)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(
                f'Failed to decode board {board_id} status: {exc}'
            )

    def _normalize_board_status(self, status):
        if status.board_id not in (
            BOARD_ID_BOARD1,
            BOARD_ID_BOARD2,
        ):
            return status

        homing_done_bits = status.homing_done_bits
        limit_status_bits = status.limit_status_bits
        moving_motor_id = status.moving_motor_id

        if status.board_id in self._axis_status_flags_board_ids:
            return self._normalize_axis_status_flags(status)

        if status.board_id in self._ready_bits_from_fault_board_ids:
            homing_done_bits = status.limit_status_bits
            limit_status_bits = 0

        if (
            status.board_id in self._idle_moving_motor_id_board_ids
            and moving_motor_id == self._idle_moving_motor_id
        ):
            moving_motor_id = ALL_MOTORS

        if (
            homing_done_bits == status.homing_done_bits
            and limit_status_bits == status.limit_status_bits
            and moving_motor_id == status.moving_motor_id
        ):
            return status

        return replace(
            status,
            homing_done_bits=homing_done_bits,
            limit_status_bits=limit_status_bits,
            moving_motor_id=moving_motor_id,
        )

    @staticmethod
    def _normalize_axis_status_flags(status):
        axis_flags = (
            status.homing_done_bits & 0x0F,
            (status.homing_done_bits >> 4) & 0x0F,
            status.moving_motor_id & 0x0F,
            (status.moving_motor_id >> 4) & 0x0F,
        )
        motor_count = motor_count_for_board(status.board_id)

        homing_done_bits = 0
        moving_motor_id = ALL_MOTORS
        for motor_id, flags in enumerate(axis_flags[:motor_count]):
            if flags & 0x01:
                homing_done_bits |= 1 << motor_id
            if moving_motor_id == ALL_MOTORS and flags & 0x04:
                moving_motor_id = motor_id

        return replace(
            status,
            homing_done_bits=homing_done_bits,
            moving_motor_id=moving_motor_id,
            limit_status_bits=0,
        )

    def _handle_motor_position_feedback(
        self,
        frame: CanFrame,
        board_id: int,
    ) -> None:
        try:
            if (
                board_id in self._packed_position_feedback_board_ids
            ):
                self._handle_packed_motor_position_feedback(frame, board_id)
                return

            feedback = unpack_motor_position_feedback(
                frame.data,
                board_id=board_id,
            )
            feedback = self._transform_arm_position_feedback(feedback)

            if not feedback.position_valid:
                return

            self._apply_arm_position_feedback(feedback)

            if feedback.error_code:
                self.get_logger().warning(
                    'Motor position feedback reports error: '
                    f'board={feedback.board_id}, '
                    f'motor={feedback.motor_id}, '
                    f'error={feedback.error_code}, '
                    f'flags=0x{feedback.flags:02X}, '
                    f'seq={feedback.sequence}'
                )
        except (ValueError, TypeError, IndexError) as exc:
            self.get_logger().error(
                f'Failed to decode board {board_id} position feedback: {exc}'
            )

    def _handle_packed_motor_position_feedback(
        self,
        frame: CanFrame,
        board_id: int,
    ) -> None:
        if len(frame.data) != 8:
            raise ValueError(
                'Packed motor position feedback must contain 8 bytes'
            )

        positions_raw = struct.unpack('<hhhh', frame.data)
        for motor_id in range(motor_count_for_board(board_id)):
            position_raw = positions_raw[motor_id]
            position_rad = self._arm_position_rad_from_raw(
                board_id=board_id,
                motor_id=motor_id,
                position_raw=position_raw,
            )
            self._apply_arm_position_feedback(
                MotorPositionFeedback(
                    board_id=board_id,
                    motor_id=motor_id,
                    flags=0x03,
                    position_raw=int(position_raw),
                    position_rad=position_rad,
                    error_code=0,
                    sequence=0,
                )
            )

    def _transform_arm_position_feedback(
        self,
        feedback: MotorPositionFeedback,
    ) -> MotorPositionFeedback:
        return replace(
            feedback,
            position_rad=self._arm_position_rad_from_raw(
                board_id=feedback.board_id,
                motor_id=feedback.motor_id,
                position_raw=feedback.position_raw,
            ),
        )

    def _arm_position_rad_from_raw(
        self,
        *,
        board_id: int,
        motor_id: int,
        position_raw: int,
    ) -> float:
        key = (int(board_id), int(motor_id))
        raw_position_rad = angle_raw_to_rad(position_raw)

        with self._arm_feedback_lock:
            sign = self._arm_raw_position_sign_by_board_motor.get(key, 1.0)
            offset = self._arm_raw_position_offset_by_board_motor.get(
                key,
                0.0,
            )

        return (raw_position_rad - offset) / sign

    def _apply_arm_position_feedback(
        self,
        feedback: MotorPositionFeedback,
    ) -> None:
        if self._arm_controller is None:
            return

        key = (feedback.board_id, feedback.motor_id)

        with self._arm_feedback_lock:
            joint_index = self._arm_feedback_joint_by_board_motor.get(key)
            if joint_index is None:
                return

            positions = list(self._arm_feedback_positions_rad)
            if any(value is None for value in positions):
                if self._arm_controller.commanded_state.is_valid():
                    positions = list(
                        self._arm_controller.commanded_state.positions()
                    )

            positions[joint_index] = feedback.position_rad
            self._arm_feedback_positions_rad = positions

            if any(value is None for value in positions):
                return

            complete_positions = [
                float(value)
                for value in positions
                if value is not None
            ]

            self._arm_controller.commanded_state.mark_positions_valid(
                complete_positions
            )
            self._arm_controller.board_state.mark_commanded_position_valid()

    def _handle_board3_position_feedback(self, frame: CanFrame) -> None:
        try:
            group = unpack_board3_position_feedback(frame.data)
            snapshot = self._board3_feedback.update(group)
            if snapshot is None:
                return

            self._apply_board3_position_feedback(snapshot.positions_rad)

            if snapshot.fault_groups or any(
                status == Board3FeedbackMotorStatus.ERROR
                for status in snapshot.status_codes
            ):
                self.get_logger().warning(
                    'Board3 position feedback reports fault: '
                    f'groups={snapshot.fault_groups}, '
                    f'status={snapshot.status_codes}, '
                    f'flags={snapshot.raw_flags}'
                )
        except (ValueError, TypeError, IndexError) as exc:
            self.get_logger().error(
                f'Failed to decode Board3 position feedback: {exc}'
            )

    def _apply_board3_position_feedback(
        self,
        positions_by_motor_rad: Sequence[float],
    ) -> None:
        if self._gripper_controller is None:
            return

        joint_positions = []

        for joint_index, (board_id, motor_id) in enumerate(zip(
            self._gripper_controller.board_ids,
            self._gripper_controller.motor_ids,
        )):
            if board_id != BOARD_ID_BOARD3:
                return

            raw_position_rad = positions_by_motor_rad[int(motor_id)]
            sign = self._gripper_controller.raw_position_signs[joint_index]
            offset = self._gripper_controller.raw_position_offsets_rad[
                joint_index
            ]
            joint_positions.append((raw_position_rad - offset) / sign)

        self._gripper_controller.commanded_state.mark_positions_valid(
            joint_positions
        )
        self._gripper_controller.board_state.mark_commanded_position_valid()

    def _handle_transport_error(self, error: Exception) -> None:
        self.get_logger().error(f'SocketCAN transport error: {error}')
        self._reset_all_board_states()
        self._invalidate_all_commanded_positions()
        self._board3_feedback.reset()

    def _send_frame(self, frame: CanFrame) -> None:
        self._transport.send_frame(frame)
        self.get_logger().info(
            f'Sent CAN {frame.can_id:#04x}#{frame.data.hex().upper()}'
        )

    def _wait_until(
        self,
        predicate,
        *,
        timeout_s: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_s

        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)

        return bool(predicate())

    def _send_or_fail(
        self,
        frame: CanFrame,
        response: Trigger.Response,
    ) -> bool:
        try:
            self._send_frame(frame)
        except SocketCanTransportError as exc:
            response.success = False
            response.message = f'CAN send failed: {exc}'
            return False
        return True

    def _reject_unless_hardware(
        self,
        response: Trigger.Response,
        operation: str,
    ) -> bool:
        if self._execution_mode == 'hardware':
            return False
        response.success = False
        response.message = (
            f'{operation} rejected: execution_mode=plan_only'
        )
        self.get_logger().warning(response.message)
        return True

    def _reserve_all_controllers(
        self,
        response: Trigger.Response,
        operation: str,
    ) -> tuple[TrajectoryControllerContext, ...] | None:
        """Reserve every motion controller for one non-emergency operation."""
        reserved: list[TrajectoryControllerContext] = []
        for controller in self._controllers:
            with controller.lock:
                if controller.active:
                    for previous in reserved:
                        with previous.lock:
                            previous.active = False
                    response.success = False
                    response.message = (
                        f'{operation} rejected: '
                        f'{controller.label} trajectory is active'
                    )
                    self.get_logger().warning(response.message)
                    return None
                controller.active = True
                reserved.append(controller)
        return tuple(reserved)

    @staticmethod
    def _release_controller_reservations(
        reserved: Sequence[TrajectoryControllerContext],
    ) -> None:
        for controller in reserved:
            with controller.lock:
                controller.active = False

    def _handle_enable(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self._reject_unless_hardware(response, 'Enable'):
            return response

        reserved = self._reserve_all_controllers(response, 'Enable')
        if reserved is None:
            return response

        try:
            return self._handle_enable_reserved(response)
        finally:
            self._release_controller_reservations(reserved)

    def _handle_enable_reserved(
        self,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Enable boards while all controllers are reserved."""
        if not self._send_or_fail(
            pack_enable(True, board_id=BOARD_ID_ALL),
            response,
        ):
            return response

        success = self._wait_until(
            lambda: (
                self._all_board_states_have_status()
                and not self._any_board_state_stale()
                and self._all_board_states_enabled()
                and not self._any_board_state_estop()
            ),
            timeout_s=self._control_wait_timeout_s,
        )

        response.success = success
        response.message = self._format_status(
            'Enable confirmed' if success else 'Enable timeout'
        )
        return response

    def _handle_disable(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if not self._send_or_fail(
            pack_enable(False, board_id=BOARD_ID_ALL),
            response,
        ):
            return response

        self._invalidate_all_commanded_positions()

        success = self._wait_until(
            lambda: (
                self._all_board_states_have_status()
                and not self._any_board_state_stale()
                and not self._any_board_state_enabled()
            ),
            timeout_s=self._control_wait_timeout_s,
        )

        response.success = success
        response.message = self._format_status(
            'Disable confirmed' if success else 'Disable timeout'
        )
        return response

    def _handle_home_all(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self._reject_unless_hardware(response, 'Homing'):
            return response

        reserved = self._reserve_all_controllers(response, 'Homing')
        if reserved is None:
            return response

        try:
            return self._handle_home_all_reserved(response)
        finally:
            self._release_controller_reservations(reserved)

    def _handle_home_all_reserved(
        self,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Home every board and move the arm into firmware command limits."""
        if not self._all_board_states_enabled():
            response.success = False
            response.message = self._format_status(
                'Cannot home: required boards are not enabled'
            )
            return response

        if self._arm_controller is not None:
            if not self._send_or_fail(
                pack_homing(
                    ALL_MOTORS,
                    0,
                    board_id=BOARD_ID_ALL,
                ),
                response,
            ):
                return response

        if self._gripper_enabled:
            if not self._send_or_fail(
                pack_gripper_home(),
                response,
            ):
                return response

        success = self._wait_until(
            lambda: (
                self._all_board_states_have_status()
                and not self._any_board_state_stale()
                and self._all_board_states_enabled()
                and self._all_board_states_ready()
                and self._all_board_trajectories_complete()
                and not self._any_board_state_error()
            ),
            timeout_s=self._homing_wait_timeout_s,
        )

        if success:
            self._mark_all_commanded_positions_valid()
            try:
                escaped = self._execute_post_home_escape()
            except (
                TrajectoryConversionError,
                TrajectoryStreamingError,
                SocketCanTransportError,
                ValueError,
            ) as exc:
                self._invalidate_all_commanded_positions()
                response.success = False
                response.message = self._format_status(
                    'Homing reached but post-home escape failed: '
                    f'{exc}'
                )
                self.get_logger().error(response.message)
                return response

            response.success = True
            response.message = self._format_status(
                'Homing and post-home escape confirmed'
                if escaped else 'Homing confirmed; escape not required'
            )
            return response

        response.success = success
        response.message = self._format_status(
            'Homing confirmed' if success else 'Homing timeout'
        )
        return response

    def _handle_clear_error(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self._reject_unless_hardware(response, 'Clear error'):
            return response

        reserved = self._reserve_all_controllers(response, 'Clear error')
        if reserved is None:
            return response

        try:
            return self._handle_clear_error_reserved(response)
        finally:
            self._release_controller_reservations(reserved)

    def _handle_clear_error_reserved(
        self,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Clear board errors while all controllers are reserved."""
        if not self._send_or_fail(
            pack_clear_error(
                ALL_MOTORS,
                board_id=BOARD_ID_ALL,
            ),
            response,
        ):
            return response

        success = self._wait_until(
            lambda: self._all_required_errors_clear(),
            timeout_s=self._control_wait_timeout_s,
        )

        response.success = success
        response.message = self._format_status(
            'Clear error confirmed'
            if success else 'Clear error timeout'
        )
        return response

    def _execute_post_home_escape(self) -> bool:
        """Run the single unsplit arm batch needed after mechanical homing."""
        controller = self._arm_controller
        if controller is None:
            return False

        initial_positions = controller.commanded_state.positions()
        duration_ticks = int(
            self.get_parameter('arm_post_home_escape_duration_ticks').value
        )
        batch = controller.trajectory_converter.build_command_limit_entry_batch(
            initial_positions,
            duration_ticks,
        )
        if batch is None:
            return False

        controller.commanded_state.start_trajectory(
            initial_positions=initial_positions,
            batches=(batch,),
        )
        controller.trajectory_streamer.stream((batch,))
        controller.commanded_state.mark_positions_valid(
            batch.target_positions_rad
        )
        controller.board_state.mark_commanded_position_valid()
        self._mark_arm_feedback_positions(batch.target_positions_rad)
        self.get_logger().info(
            'Post-home command-limit escape completed: '
            f'targets={list(batch.target_positions_rad)}'
        )
        return True

    def _handle_estop(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if not self._send_or_fail(
            pack_estop(board_id=BOARD_ID_ALL),
            response,
        ):
            return response

        self._invalidate_all_commanded_positions()

        success = self._wait_until(
            lambda: (
                self._any_board_state_estop()
                or (
                    self._all_board_states_have_status()
                    and not self._any_board_state_enabled()
                )
            ),
            timeout_s=self._control_wait_timeout_s,
        )

        response.success = success
        response.message = self._format_status(
            'ESTOP confirmed' if success else 'ESTOP timeout'
        )
        return response

    def _handle_status_request(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        response.success = self._all_board_states_have_status()
        response.message = self._format_status('Status snapshot')
        return response

    def _goal_callback_follow_joint_trajectory(
        self,
        goal_request,
        controller: TrajectoryControllerContext,
    ):
        if self._execution_mode != 'hardware':
            self.get_logger().warning(
                f'Reject {controller.label} trajectory: '
                'execution_mode=plan_only'
            )
            return GoalResponse.REJECT

        with controller.lock:
            if controller.active:
                self.get_logger().warning(
                    f'Reject {controller.label} trajectory: '
                    'another trajectory is active'
                )
                return GoalResponse.REJECT

            if not controller.board_state.can_accept_new_trajectory():
                self.get_logger().warning(
                    f'Reject {controller.label} trajectory: '
                    'required boards are not ready; '
                    f'{self._format_status("goal check")}'
                )
                return GoalResponse.REJECT

            if not controller.commanded_state.is_valid():
                self.get_logger().warning(
                    f'Reject {controller.label} trajectory: '
                    'commanded position estimate is not valid'
                )
                return GoalResponse.REJECT

            try:
                current_positions = controller.commanded_state.positions()
                controller.trajectory_converter.convert(
                    goal_request.trajectory,
                    current_positions,
                )
            except TrajectoryConversionError as exc:
                self.get_logger().warning(
                    f'Reject {controller.label} trajectory: {exc}; '
                    f'goal_joints={list(goal_request.trajectory.joint_names)}, '
                    f'current_positions={list(current_positions)}'
                )
                return GoalResponse.REJECT

            return GoalResponse.ACCEPT

    def _cancel_callback_follow_joint_trajectory(
        self,
        goal_handle,
        controller: TrajectoryControllerContext,
    ):
        del goal_handle
        self.get_logger().warning(
            f'{controller.label} FollowJointTrajectory cancel requested'
        )
        return CancelResponse.ACCEPT

    def _execute_follow_joint_trajectory(
        self,
        goal_handle,
        controller: TrajectoryControllerContext,
    ):
        result = FollowJointTrajectory.Result()

        with controller.lock:
            if controller.active:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = (
                    f'{controller.label} trajectory is already active'
                )
                return result
            controller.active = True

        try:
            trajectory = goal_handle.request.trajectory
            initial_positions = controller.commanded_state.positions()

            batches = controller.trajectory_converter.convert(
                trajectory,
                initial_positions,
            )

            controller.commanded_state.start_trajectory(
                initial_positions=initial_positions,
                batches=batches,
            )

            def publish_progress(
                completed: int,
                total: int,
                batch,
            ) -> None:
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = list(controller.joint_names)
                feedback.actual.positions = list(
                    controller.commanded_state.positions()
                )
                feedback.desired.positions = list(
                    batch.target_positions_rad
                )
                feedback.error.positions = [
                    desired - actual
                    for desired, actual in zip(
                        feedback.desired.positions,
                        feedback.actual.positions,
                    )
                ]

                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f'{controller.label} trajectory streaming progress: '
                    f'{completed}/{total}'
                )

            controller.trajectory_streamer.stream(
                batches,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                progress_callback=publish_progress,
            )

            if goal_handle.is_cancel_requested:
                self._invalidate_all_commanded_positions()

                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Trajectory canceled'
                return result

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'Trajectory completed'
            return result

        except TrajectoryCanceled as exc:
            self.get_logger().warning(
                f'{controller.label} trajectory execution canceled: {exc}'
            )

            self._invalidate_all_commanded_positions()

            goal_handle.canceled()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        except TrajectoryStreamingError as exc:
            status_snapshot = self._format_status('failure')
            self.get_logger().error(
                f'{controller.label} trajectory execution failed: {exc}; '
                f'{status_snapshot}'
            )

            self.get_logger().warning(
                'Leaving motor enable unchanged after trajectory error; '
                'use /arm_board/disable or ESTOP only if needed'
            )

            self._invalidate_all_commanded_positions()

            goal_handle.abort()
            result.error_code = (
                FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            )
            result.error_string = f'{exc}; {status_snapshot}'
            return result

        except Exception as exc:
            self.get_logger().exception(
                f'Unexpected {controller.label} trajectory execution '
                f'error: {exc}'
            )

            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        finally:
            with controller.lock:
                controller.active = False

    def _reset_all_board_states(self) -> None:
        for controller in self._controllers:
            controller.board_state.reset()

    def _invalidate_all_commanded_positions(self) -> None:
        for controller in self._controllers:
            controller.board_state.invalidate_commanded_position()
            controller.commanded_state.reset_invalid()
        self._reset_arm_position_feedback()

    def _mark_all_commanded_positions_valid(self) -> None:
        for controller in self._controllers:
            controller.board_state.mark_commanded_position_valid()
            controller.commanded_state.mark_positions_valid(
                controller.home_positions_rad
            )
        if self._arm_controller is not None:
            self._mark_arm_feedback_positions(
                self._arm_controller.home_positions_rad
            )

    def _reset_arm_position_feedback(self) -> None:
        if self._arm_controller is None:
            return

        with self._arm_feedback_lock:
            self._arm_feedback_positions_rad = [
                None
                for _ in self._arm_controller.joint_names
            ]

    def _mark_arm_feedback_positions(
        self,
        positions_rad: Sequence[float],
    ) -> None:
        with self._arm_feedback_lock:
            self._arm_feedback_positions_rad = [
                float(value)
                for value in positions_rad
            ]

    def _all_board_states_have_status(self) -> bool:
        return all(
            controller.board_state.has_status()
            for controller in self._controllers
        )

    def _any_board_state_stale(self) -> bool:
        return any(
            controller.board_state.is_status_stale()
            for controller in self._controllers
        )

    def _all_board_states_enabled(self) -> bool:
        return all(
            controller.board_state.is_enabled()
            for controller in self._controllers
        )

    def _any_board_state_enabled(self) -> bool:
        return any(
            controller.board_state.is_enabled()
            for controller in self._controllers
        )

    def _any_board_state_estop(self) -> bool:
        return any(
            controller.board_state.is_estop()
            for controller in self._controllers
        )

    def _all_board_states_ready(self) -> bool:
        return all(
            controller.board_state.all_axes_homed()
            for controller in self._controllers
        )

    def _all_board_trajectories_complete(self) -> bool:
        return all(
            controller.board_state.is_trajectory_complete()
            for controller in self._controllers
        )

    def _any_board_state_error(self) -> bool:
        return any(
            controller.board_state.has_error()
            for controller in self._controllers
        )

    def _all_required_errors_clear(self) -> bool:
        return (
            self._all_board_states_have_status()
            and not self._any_board_state_stale()
            and not self._any_board_state_error()
        )

    def _publish_status_log(self) -> None:
        message = String()
        message.data = self._format_status('periodic')
        self._status_publisher.publish(message)

    def _publish_joint_states(self) -> None:
        if not all(
            controller.commanded_state.is_valid()
            for controller in self._controllers
        ):
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for controller in self._controllers:
            msg.name.extend(controller.joint_names)
            msg.position.extend(controller.commanded_state.positions())

        msg.name.extend(self._fixed_joint_state_names)
        msg.position.extend(self._fixed_joint_state_positions_rad)

        self._joint_state_publisher.publish(msg)

    def _format_status(self, prefix: str) -> str:
        controller_parts = []

        for controller in self._controllers:
            snapshot = controller.board_state.snapshot()
            board_parts = []

            for board_id, board_snapshot in snapshot.boards.items():
                status = board_snapshot.status

                if status is None:
                    board_parts.append(
                        f'board{board_id}=no status, '
                        f'stale={board_snapshot.status_stale}'
                    )
                    continue

                board_parts.append(
                    f'board{board_id}: '
                    f'state={self._state_name(status.state)}, '
                    f'error='
                    f'{self._error_name(status.error_code, status.board_id)}, '
                    f'ready=0x{status.homing_done_bits:02X}, '
                    f'moving={status.moving_motor_id}, '
                    f'fault=0x{status.limit_status_bits:02X}, '
                    f'queue_free={status.queue_free}, '
                    f'local_queue_free={board_snapshot.local_queue_free}, '
                    f'enabled={status.enabled}, '
                    f'stale={board_snapshot.status_stale}, '
                    f'age_ms='
                    f'{self._format_age(board_snapshot.status_age_ms)}, '
                    f'position_valid='
                    f'{board_snapshot.commanded_position_valid}'
                )

            can_accept = (
                self._execution_mode == 'hardware'
                and controller.board_state.can_accept_new_trajectory()
            )
            controller_parts.append(
                f'{controller.label}['
                + '; '.join(board_parts)
                + f'; accept_traj='
                f'{can_accept}'
                + ']'
            )

        return f'{prefix}: ' + ' || '.join(controller_parts)

    @staticmethod
    def _format_age(age_ms: float | None) -> str:
        if age_ms is None:
            return 'None'
        return f'{age_ms:.1f}'

    @staticmethod
    def _state_name(value: int) -> str:
        try:
            return BoardState(value).name
        except ValueError:
            return f'UNKNOWN({value})'

    @staticmethod
    def _error_name(value: int, board_id: int) -> str:
        return error_name_for_board(value, board_id)

    def destroy_node(self) -> bool:
        """Close SocketCAN transport before destroying the ROS node."""
        self._transport.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the arm CAN bridge service node."""
    rclpy.init(args=args)
    node: ArmCanBridgeNode | None = None
    executor = MultiThreadedExecutor(num_threads=4)
    node_added = False

    try:
        node = ArmCanBridgeNode()
        executor.add_node(node)
        node_added = True
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if node_added:
                executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
