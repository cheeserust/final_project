"""ROS 2 control-services node for the arm CAN bridge."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import json
import math
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
from vicpinky_interfaces.action import ExecuteArmGoal

from .arm_goal_v3 import (
    ArmGoalV3AbortedByEstop,
    ArmGoalV3Canceled,
    ArmGoalV3Coordinator,
    ArmGoalV3Error,
    build_arm_goal_frames_v3,
)
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
    BOARD_ID_BY_ACK_CAN_ID,
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
    QUEUE_CAPACITY,
    RECEIVE_CAN_IDS,
    unpack_arm_goal_ack_v3,
    unpack_board3_position_feedback,
    unpack_motor_position_feedback,
    unpack_status,
)
from .can_writer import SerializedCanWriter
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
    trajectory_converter: ArmTrajectoryConverter | None
    trajectory_streamer: TrajectoryStreamer | None
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
        self._arm_v3_ready_timeout_ms = int(
            self.get_parameter('arm_v3_ready_timeout_ms').value
        )
        self._arm_v3_max_stage_attempts = int(
            self.get_parameter('arm_v3_max_stage_attempts').value
        )
        self._arm_v3_communication_timeout_ms = int(
            self.get_parameter('arm_v3_communication_timeout_ms').value
        )
        self._can_tx_retry_count = int(
            self.get_parameter('can_tx_retry_count').value
        )
        self._can_tx_retry_delay_ms = float(
            self.get_parameter('can_tx_retry_delay_ms').value
        )
        self._can_batch_inter_frame_delay_ms = float(
            self.get_parameter('can_batch_inter_frame_delay_ms').value
        )
        self._can_writer_batch_timeout_ms = int(
            self.get_parameter('can_writer_batch_timeout_ms').value
        )
        self._clear_active_goal_timeout_ms = int(
            self.get_parameter('clear_active_goal_timeout_ms').value
        )
        if self._clear_active_goal_timeout_ms <= 0:
            raise ValueError('clear_active_goal_timeout_ms must be positive')
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
        self._can_writer = SerializedCanWriter(
            self._transport,
            retry_count=self._can_tx_retry_count,
            retry_delay_s=self._can_tx_retry_delay_ms / 1000.0,
            batch_inter_frame_delay_s=(
                self._can_batch_inter_frame_delay_ms / 1000.0
            ),
            request_timeout_s=self._can_writer_batch_timeout_ms / 1000.0,
            event_callback=self._log_protocol_event,
        )
        self._arm_v3 = ArmGoalV3Coordinator(
            self._can_writer,
            ack_timeout_s=self._arm_v3_ready_timeout_ms / 1000.0,
            communication_timeout_s=(
                self._arm_v3_communication_timeout_ms / 1000.0
            ),
            max_stage_attempts=self._arm_v3_max_stage_attempts,
            event_callback=self._log_protocol_event,
        )
        self.get_logger().info(
            'Arm direct transport: Board1 Goal V3 + Board2 legacy; '
            'retry profile: '
            f'READY={self._arm_v3_ready_timeout_ms}ms x '
            f'{self._arm_v3_max_stage_attempts}, '
            f'TX retry={self._can_tx_retry_count} x '
            f'{self._can_tx_retry_delay_ms:g}ms, '
            f'batch gap={self._can_batch_inter_frame_delay_ms:g}ms, '
            f'writer timeout={self._can_writer_batch_timeout_ms}ms, '
            f'heartbeat={self._arm_v3_communication_timeout_ms}ms'
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
            raise ValueError(
                'At least one of enable_arm or enable_gripper must be true'
            )
        self._controllers = tuple(controllers)
        self._configure_fixed_joint_states()
        self._validate_combined_joint_names()
        self._configure_arm_position_feedback()
        self._transport.open()
        if self._execution_mode == 'hardware' and self._arm_enabled:
            if not self._arm_v3.probe_capability():
                self.get_logger().error(
                    'Board1 V3 capability probe failed; direct arm goals '
                    'remain blocked'
                )
        self._v3_probe_timer = self.create_timer(
            2.0,
            self._retry_v3_capability_probe,
            callback_group=self._callback_group,
        )

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
            if controller.label == 'arm':
                controller.action_server = ActionServer(
                    self,
                    ExecuteArmGoal,
                    controller.action_name,
                    execute_callback=self._execute_arm_goal_v3,
                    goal_callback=self._goal_callback_arm_goal_v3,
                    cancel_callback=self._cancel_callback_arm_goal_v3,
                    callback_group=self._callback_group,
                )
                self._action_servers.append(controller.action_server)
                continue
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
                'Plan-only mode: motion goals and state-changing arm '
                'services are rejected; disable and ESTOP remain available'
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('execution_mode', 'plan_only')
        # Required external profile: config/retry_timeout.yaml.
        self.declare_parameter('arm_v3_ready_timeout_ms')
        self.declare_parameter('arm_v3_max_stage_attempts')
        self.declare_parameter('arm_v3_communication_timeout_ms')
        self.declare_parameter('can_tx_retry_count')
        self.declare_parameter('can_tx_retry_delay_ms')
        self.declare_parameter('can_batch_inter_frame_delay_ms')
        self.declare_parameter('can_writer_batch_timeout_ms')
        self.declare_parameter('clear_active_goal_timeout_ms')
        self.declare_parameter('status_timeout_ms')
        self.declare_parameter('queue_capacity', BOARD3_SERVO_COUNT)
        self.declare_parameter('required_homing_mask', 0x0F)
        self.declare_parameter('control_wait_timeout_ms')
        self.declare_parameter('homing_wait_timeout_ms')
        self.declare_parameter('status_publish_period_ms', 500)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_state_rate_hz', 50.0)
        self.declare_parameter('enable_arm', True)
        self.declare_parameter('enable_gripper', True)
        self.declare_parameter('gripper_target_load_raw', 500)
        self.declare_parameter('queue_wait_timeout_ms')
        self.declare_parameter('completion_grace_ms')
        self.declare_parameter('board3_inter_frame_delay_ms', 3.0)
        self.declare_parameter('start_position_tolerance_rad', 0.02)
        self.declare_parameter('packed_position_feedback_board_ids', [
            BOARD_ID_BOARD1,
            BOARD_ID_BOARD2,
        ])
        self.declare_parameter('board1_packed_position_feedback', False)

        self.declare_parameter(
            'arm_action_name',
            '/arm_controller/execute_joint_goal',
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
            -8650,
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
        trajectory_converter = None
        trajectory_streamer = None
        if label == 'gripper':
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
                max_segment_duration_ticks=MAX_DURATION_TICKS,
                min_segment_duration_ticks=1,
            )
            trajectory_streamer = TrajectoryStreamer(
                board_state=board_state,
                transport=self._can_writer,
                queue_wait_timeout_ms=int(
                    self.get_parameter('queue_wait_timeout_ms').value
                ),
                completion_grace_ms=int(
                    self.get_parameter('completion_grace_ms').value
                ),
                board3_inter_frame_delay_ms=float(
                    self.get_parameter('board3_inter_frame_delay_ms').value
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
        del label
        return 0

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
        return MultiBoardStateTracker(
            board_ids=unique_board_ids,
            status_timeout_ms=int(
                self.get_parameter('status_timeout_ms').value
            ),
            queue_capacities={
                board_id: (
                    BOARD3_SERVO_COUNT
                    if board_id == BOARD_ID_BOARD3
                    else 1
                    if board_id == BOARD_ID_BOARD1
                    else QUEUE_CAPACITY
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
        ack_board_id = BOARD_ID_BY_ACK_CAN_ID.get(frame.can_id)
        if ack_board_id is not None:
            try:
                self._arm_v3.update_ack(
                    unpack_arm_goal_ack_v3(
                        frame.data,
                        board_id=ack_board_id,
                    )
                )
            except (ValueError, TypeError) as exc:
                self.get_logger().error(
                    f'Failed to decode Board{ack_board_id} V3 ACK: {exc}'
                )
            return

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
            status = unpack_status(
                frame.data,
                board_id=board_id,
                board2_legacy=(board_id == BOARD_ID_BOARD2),
            )
            status = self._normalize_board_status(status)
            self._arm_v3.update_status(status)
            for controller in self._controllers:
                controller.board_state.update_status(status)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(
                f'Ignoring invalid Board{board_id} status '
                f'{frame.data.hex().upper()}; keeping last valid status: '
                f'{exc}',
                throttle_duration_sec=1.0,
            )

    def _normalize_board_status(self, status):
        # Board2 legacy keeps the same single-axis flag nibble layout.
        return status

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
        self._arm_v3.reset_capability()
        self._reset_all_board_states()
        self._invalidate_all_commanded_positions()
        self._board3_feedback.reset()

    def _retry_v3_capability_probe(self) -> None:
        """Passively confirm fresh Board1 V3 and Board2 legacy status."""
        if (
            self._execution_mode != 'hardware'
            or not self._arm_enabled
            or self._arm_v3.capability_confirmed
            or self._arm_v3.active_goal_id is not None
        ):
            return
        try:
            if self._arm_v3.probe_capability():
                self.get_logger().info(
                    'Board1 V3 + Board2 legacy status capability confirmed'
                )
            else:
                self.get_logger().warning(
                    'Board1 V3 + Board2 legacy status still waiting'
                )
        except (SocketCanTransportError, RuntimeError) as exc:
            self.get_logger().warning(f'V3 capability probe retry failed: {exc}')

    def _log_protocol_event(self, event) -> None:
        """Write one machine-readable Goal V3/TX diagnostic record."""
        self.get_logger().info(
            'ARM_V3 ' + json.dumps(
                dict(event),
                ensure_ascii=False,
                sort_keys=True,
                default=str,
            )
        )

    def _send_frame(self, frame: CanFrame) -> None:
        self._can_writer.send_batch((frame,))
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
        if success:
            self._arm_v3.clear_estop_latch()
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
            except Exception as exc:
                # A service callback must report failure without terminating
                # the entire bridge process.
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

        if not self._finish_active_arm_goal_for_clear(response):
            return response

        reserved = self._reserve_all_controllers(response, 'Clear error')
        if reserved is None:
            return response

        try:
            return self._handle_clear_error_reserved(response)
        finally:
            self._release_controller_reservations(reserved)

    def _finish_active_arm_goal_for_clear(
        self,
        response: Trigger.Response,
    ) -> bool:
        """Cancel an active direct arm goal before sending Clear Error."""
        controller = self._arm_controller
        if controller is None or self._arm_v3.active_goal_id is None:
            return True

        self.get_logger().warning(
            'Clear error requested during arm motion; canceling the active '
            'goal without disabling motor power'
        )
        self._arm_v3.request_active_cancel()
        finished = self._wait_until(
            lambda: (
                self._arm_v3.active_goal_id is None
                and not controller.active
            ),
            timeout_s=self._clear_active_goal_timeout_ms / 1000.0,
        )
        if finished:
            return True

        response.success = False
        response.message = (
            'Clear error could not finish the active arm goal within '
            f'{self._clear_active_goal_timeout_ms}ms; '
            'motor enable was left unchanged'
        )
        self.get_logger().error(response.message)
        return False

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
        if success:
            self._arm_v3.clear_estop_latch()
        response.message = self._format_status(
            'Clear error confirmed'
            if success else 'Clear error timeout'
        )
        return response

    def _execute_post_home_escape(self) -> bool:
        """Move every arm axis away from its asserted home limit switch."""
        controller = self._arm_controller
        if controller is None:
            return False

        escape_rad = math.radians(5.0)
        targets = [
            home + escape_rad
            for home in controller.home_positions_rad
        ]
        if all(
            abs(target - home) < 1e-9
            for target, home in zip(targets, controller.home_positions_rad)
        ):
            return False

        completed = self._arm_v3.execute(
            joint_names=controller.joint_names,
            positions_rad=targets,
            duration_ms=1000,
        )
        ordered_positions = [
            completed.positions_by_name[name]
            for name in controller.joint_names
        ]
        controller.commanded_state.mark_positions_valid(ordered_positions)
        controller.board_state.mark_commanded_position_valid()
        self._mark_arm_feedback_positions(ordered_positions)
        return True

    def _handle_estop(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        try:
            self._arm_v3.abort_by_estop(
                pack_estop(board_id=BOARD_ID_ALL)
            )
        except (SocketCanTransportError, RuntimeError) as exc:
            response.success = False
            response.message = f'E-stop CAN send failed: {exc}'
            return response

        self._invalidate_all_commanded_positions()

        success = self._wait_until(
            self._both_arm_boards_estop,
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

    def _goal_callback_arm_goal_v3(self, goal_request):
        """Validate one direct final-angle goal without reading waypoints."""
        controller = self._arm_controller
        if controller is None or self._execution_mode != 'hardware':
            return GoalResponse.REJECT
        with controller.lock:
            if controller.active or self._arm_v3.active_goal_id is not None:
                self.get_logger().warning('Reject direct arm goal: BUSY')
                return GoalResponse.REJECT
            if not self._arm_v3.capability_confirmed:
                self.get_logger().warning(
                    'Reject direct arm goal: Board1 V3 + Board2 legacy '
                    'status not confirmed'
                )
                return GoalResponse.REJECT
            if not controller.board_state.can_accept_new_trajectory():
                self.get_logger().warning(
                    'Reject direct arm goal: Board1/Board2 status is not '
                    'fresh, idle, enabled, homed, and fully available'
                )
                return GoalResponse.REJECT
            try:
                build_arm_goal_frames_v3(
                    joint_names=goal_request.joint_names,
                    positions_rad=goal_request.positions,
                    duration_ms=goal_request.duration_ms,
                    goal_id=0,
                )
            except (ValueError, OverflowError) as exc:
                self.get_logger().warning(f'Reject direct arm goal: {exc}')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback_arm_goal_v3(self, goal_handle):
        """Accept cancel; Board1 confirms it while Board2 remains legacy."""
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute_arm_goal_v3(self, goal_handle):
        """Execute one Board1 V3 + Board2 legacy direct goal."""
        result = ExecuteArmGoal.Result()
        controller = self._arm_controller
        if controller is None:
            goal_handle.abort()
            result.message = 'Arm controller is disabled'
            return result
        with controller.lock:
            if controller.active:
                goal_handle.abort()
                result.message = 'Another arm goal is active'
                return result
            controller.active = True

        try:
            def publish_feedback(phase, goal_id, detail):
                message = ExecuteArmGoal.Feedback()
                message.goal_id = int(goal_id)
                message.phase = int(phase)
                message.detail = str(detail)
                goal_handle.publish_feedback(message)

            completed = self._arm_v3.execute(
                joint_names=goal_handle.request.joint_names,
                positions_rad=goal_handle.request.positions,
                duration_ms=goal_handle.request.duration_ms,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback=publish_feedback,
                request_id=(
                    str(getattr(goal_handle.request, 'request_id', ''))
                    or None
                ),
                web_created_unix_ms=int(
                    getattr(goal_handle.request, 'web_created_unix_ms', 0)
                ),
                gui_received_unix_ms=int(
                    getattr(goal_handle.request, 'gui_received_unix_ms', 0)
                ),
            )
            ordered_positions = [
                completed.positions_by_name[name]
                for name in controller.joint_names
            ]
            controller.commanded_state.mark_positions_valid(ordered_positions)
            controller.board_state.mark_commanded_position_valid()
            self._mark_arm_feedback_positions(ordered_positions)
            goal_handle.succeed()
            result.success = True
            result.goal_id = completed.goal_id
            result.message = 'Direct arm goal completed'
            return result
        except ArmGoalV3Canceled as exc:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = str(exc)
            else:
                goal_handle.abort()
                result.message = 'Direct arm goal interrupted by Clear Error'
            return result
        except ArmGoalV3AbortedByEstop as exc:
            self.get_logger().warning(str(exc))
            goal_handle.abort()
            result.message = str(exc)
            return result
        except (ArmGoalV3Error, ValueError, OverflowError) as exc:
            self.get_logger().error(f'Direct arm goal failed: {exc}')
            goal_handle.abort()
            result.message = str(exc)
            return result
        finally:
            with controller.lock:
                controller.active = False

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
                    'goal_joints='
                    f'{list(goal_request.trajectory.joint_names)}, '
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

    def _both_arm_boards_estop(self) -> bool:
        """Require fresh powered-hold E-stop status from Board1 and Board2."""
        controller = self._arm_controller
        if controller is None or controller.board_state.is_status_stale():
            return False
        statuses = controller.board_state.latest_statuses()
        return all(
            statuses.get(board_id) is not None
            and statuses[board_id].state == BoardState.ESTOP
            for board_id in (BOARD_ID_BOARD1, BOARD_ID_BOARD2)
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
        self._log_protocol_event({
            'component': 'socketcan',
            'event': 'diagnostics',
            **self._transport.diagnostics(),
            'active_goal_id': self._arm_v3.active_goal_id,
            'active_state': self._arm_v3.state.value,
        })

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

                common = (
                    f'board{board_id}: '
                    f'state={self._state_name(status.state)}, '
                    f'error='
                    f'{self._error_name(status.error_code, status.board_id)}, '
                )
                if board_id == BOARD_ID_BOARD1:
                    protocol_fields = (
                        f'ready_mask=0x{status.ready_mask:02X}, '
                        f'moving_mask=0x{status.moving_mask:02X}, '
                        f'reached_mask=0x{status.target_reached_mask:02X}, '
                        f'goal_slot_free={status.goal_slot_free}, '
                        f'seq={status.status_sequence}, '
                    )
                elif board_id == BOARD_ID_BOARD2:
                    protocol_fields = (
                        f'ready_mask=0x{status.ready_mask:02X}, '
                        f'moving_mask=0x{status.moving_mask:02X}, '
                        f'reached_mask=0x{status.target_reached_mask:02X}, '
                        f'queue_free={status.queue_free}, '
                        f'seq={status.status_sequence}, '
                    )
                else:
                    protocol_fields = (
                        f'ready=0x{status.homing_done_bits:02X}, '
                        f'moving={status.moving_motor_id}, '
                        f'buffer_free={status.queue_free}, '
                    )
                board_parts.append(
                    common
                    + protocol_fields
                    + f'fault=0x{status.limit_status_bits:02X}, '
                    f'enabled={status.enabled}, '
                    f'stale={board_snapshot.status_stale}, '
                    f'age_ms={self._format_age(board_snapshot.status_age_ms)}, '
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

        return (
            f'{prefix}: arm_v3_state={self._arm_v3.state.value}, '
            f'active_goal={self._arm_v3.active_goal_id} || '
            + ' || '.join(controller_parts)
        )

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
        self._can_writer.close()
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
