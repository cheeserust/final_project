"""
Central-PC node for the standalone final project presentation.

This package intentionally does not import the legacy GUI, mission, Nav2, or
demo packages.  It talks only to the robot's documented ROS hardware topics
and actions.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
import signal
import threading
import time
import traceback
from typing import Any, Callable

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from werkzeug.serving import make_server

from .arm_executor import (
    ARM_JOINT_NAMES,
    ArmPoseExecutor,
    GRIPPER_JOINT_NAMES,
    PoseCancellationUnconfirmed,
    PoseExecutionError,
)
from .aruco_tracker import ArucoTracker
from .board_status import parse_arm_board_status
from .config_store import (
    ConfigConflictError,
    ConfigStore,
    ConfigStoreError,
    validate_document,
)
from .control_core import (
    ExactAngleTurnController,
    TurnControlConfig,
    VelocityCommand,
    VisualApproachConfig,
    VisualApproachController,
)
from .web_app import ApiError, create_app
from .workflow_core import (
    add_category,
    canonical_entity_id,
    delete_category as core_delete_category,
    delete_pose as core_delete_pose,
    rename_category,
    WorkflowCoreError,
)


ZERO_COMMAND = VelocityCommand()


class OperationStopped(RuntimeError):
    """Internal non-error termination raised by STOP or process shutdown."""


class RecoverableOperationError(RuntimeError):
    """A stopped demo step that may be retried without clearing an error."""


def _now() -> float:
    return time.monotonic()


def _yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _json_safe(value: Any) -> Any:
    """Convert enums/dataclasses and non-finite floats for the HTTP snapshot."""
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [_json_safe(item) for item in value]
    enum_value = getattr(value, 'value', None)
    if enum_value is not None:
        return _json_safe(enum_value)
    if hasattr(value, '__dict__'):
        return _json_safe(vars(value))
    return str(value)


class PresentationNode(Node):
    """Own the presentation route, action execution, and HTTP API."""

    def __init__(self) -> None:
        super().__init__('final_project_presentation2')
        self.declare_parameter('start_web', True)

        self._lock = threading.RLock()
        self._bridge = CvBridge()
        self._events: deque[dict[str, Any]] = deque(maxlen=120)
        self._state = 'STARTING'
        self._active_kind: str | None = None
        self._active_name: str | None = None
        self._route_direction: str | None = None
        self._phase = 'startup'
        self._error: str | None = None
        self._config_error: str | None = None
        self._restart_required: list[str] = []
        self._checkpoint: dict[str, Any] | None = None
        self._workflow_progress: dict[str, Any] | None = None
        self._workflow_recovery: dict[str, Any] | None = None
        self._worker: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._return_confirm_event = threading.Event()
        self._shutdown_event = threading.Event()
        self._operator_lease_at = _now()
        self._lease_expiry_triggered = False
        self._safety_abort_reason: str | None = None
        self._desired_command = ZERO_COMMAND
        self._desired_command_at = _now()
        self._arm_status_poll_future = None
        self._arm_board_status: dict[str, Any] = {
            'available': False,
            'service_success': False,
            'updated_at_sec': None,
            'message': 'Waiting for /arm_board/status',
            'controllers': [],
            'boards': [],
        }

        self._yaw_rad: float | None = None
        self._odom_at: float | None = None
        self._last_odom_header_stamp: float | None = None
        self._joint_positions: dict[str, float] = {}
        self._joint_update_at: dict[str, float] = {}
        self._frames: dict[str, Any] = {'front': None, 'rear': None}
        self._frame_at: dict[str, float | None] = {
            'front': None,
            'rear': None,
        }
        self._camera_info_at: dict[str, float | None] = {
            'front': None,
            'rear': None,
        }
        self._camera_jpegs: dict[str, bytes | None] = {
            'front': None,
            'rear': None,
        }
        self._jpeg_at: dict[str, float | None] = {
            'front': None,
            'rear': None,
        }
        self._observations: dict[tuple[str, int], Any] = {}
        self._visible_markers: dict[str, list[dict[str, Any]]] = {
            'front': [],
            'rear': [],
        }
        self._metrics: dict[str, Any] = {
            'active_camera': None,
            'target_marker_id': None,
            'detected_marker_id': None,
            'distance_m': None,
            'lateral_m': None,
            'yaw_error_deg': None,
            'turn_progress_deg': None,
            'turn_target_deg': None,
            'control_reason': None,
        }

        self._config_path = self._resolve_config_path()
        self._store = ConfigStore(self._config_path)
        self._config: dict[str, Any] = {}
        self._load_initial_config()

        self._tracker = ArucoTracker.from_config(self._config)
        self._vision_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_callback_group = ReentrantCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_callback_group = ReentrantCallbackGroup()
        topics = self._config['topics']
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        raw_command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # A queued odometry history must never be replayed as fresh control
        # feedback after a network pause.  JointState keeps a deeper history
        # because it is used only for pose readiness, never turn integration.
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        joint_state_qos = QoSProfile(depth=1)
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._raw_cmd_publisher = self.create_publisher(
            TwistStamped,
            topics['cmd_vel_raw'],
            raw_command_qos,
        )
        self._watchdog_config_publisher = self.create_publisher(
            String,
            topics['watchdog_config'],
            transient_qos,
        )
        self._subscriptions = [
            self.create_subscription(
                Image,
                topics['front_image'],
                lambda message: self._image_callback('front', message),
                sensor_qos,
                callback_group=self._vision_callback_group,
            ),
            self.create_subscription(
                Image,
                topics['rear_image'],
                lambda message: self._image_callback('rear', message),
                sensor_qos,
                callback_group=self._vision_callback_group,
            ),
            self.create_subscription(
                CameraInfo,
                topics['front_camera_info'],
                lambda message: self._camera_info_callback('front', message),
                sensor_qos,
                callback_group=self._vision_callback_group,
            ),
            self.create_subscription(
                CameraInfo,
                topics['rear_camera_info'],
                lambda message: self._camera_info_callback('rear', message),
                sensor_qos,
                callback_group=self._vision_callback_group,
            ),
            self.create_subscription(
                Odometry,
                topics['odom'],
                self._odom_callback,
                odom_qos,
                callback_group=self._state_callback_group,
            ),
            self.create_subscription(
                JointState,
                topics['joint_states'],
                self._joint_state_callback,
                joint_state_qos,
                callback_group=self._state_callback_group,
            ),
        ]
        self._pose_executor = ArmPoseExecutor(
            self,
            topics['arm_action'],
            topics['gripper_action'],
            self._record_event,
            callback_group=self._action_callback_group,
        )
        self._arm_status_client = self.create_client(
            Trigger,
            topics['arm_status_service'],
            callback_group=self._action_callback_group,
        )
        self._arm_command_clients = {
            'status': self._arm_status_client,
            'enable': self.create_client(
                Trigger,
                topics.get('arm_enable_service', '/arm_board/enable'),
                callback_group=self._action_callback_group,
            ),
            'disable': self.create_client(
                Trigger,
                topics.get('arm_disable_service', '/arm_board/disable'),
                callback_group=self._action_callback_group,
            ),
            'home': self.create_client(
                Trigger,
                topics.get('arm_home_service', '/arm_board/home_all'),
                callback_group=self._action_callback_group,
            ),
            'clear': self.create_client(
                Trigger,
                topics.get(
                    'arm_clear_error_service', '/arm_board/clear_error'
                ),
                callback_group=self._action_callback_group,
            ),
            'estop': self.create_client(
                Trigger,
                topics.get('arm_estop_service', '/arm_board/estop'),
                callback_group=self._action_callback_group,
            ),
        }

        watchdog_rate = float(self._config['safety']['watchdog_rate_hz'])
        control_rate = float(self._config['motion_control']['control_rate_hz'])
        self._command_timer = self.create_timer(
            1.0 / max(watchdog_rate, control_rate),
            self._publish_command,
            callback_group=self._control_callback_group,
        )
        self._watchdog_config_timer = self.create_timer(
            1.0,
            self._publish_watchdog_config,
            callback_group=self._control_callback_group,
        )
        self._arm_status_timer = self.create_timer(
            1.0,
            self._poll_arm_board_status,
            callback_group=self._state_callback_group,
        )

        self._web_server = None
        self._web_thread: threading.Thread | None = None
        if bool(self.get_parameter('start_web').value):
            self._start_web_server()

        with self._lock:
            self._state = 'IDLE' if self._config_error is None else 'ERROR_LATCHED'
            self._phase = 'ready' if self._config_error is None else 'config_error'
        self._publish_watchdog_config()
        self._record_event('info', f'Configuration: {self._config_path}')
        self.get_logger().info(
            f'final_project_presentation2 ready; config={self._config_path}'
        )

    # ------------------------------------------------------------------
    # Configuration and ROS callbacks

    def _resolve_config_path(self) -> Path:
        """Return the single source JSON used by the running node."""
        share = Path(get_package_share_directory('final_project_presentation2'))
        installed_config = (
            share / 'config' / 'final_project_presentation2.json'
        )
        source_config = (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'final_project_presentation2.json'
        ).resolve()
        if (
            installed_config.resolve() != source_config
            or not source_config.is_file()
        ):
            raise RuntimeError(
                'The default JSON config is not linked to the package source '
                'folder. Rebuild with `colcon build --symlink-install '
                '--packages-select final_project_presentation2`.'
            )
        return source_config

    def _load_initial_config(self) -> None:
        document = self._store.snapshot()
        self._config = document
        self._config_error = None

    def _image_callback(self, camera_name: str, message: Image) -> None:
        arrival = _now()
        try:
            # Some camera drivers used in the working elevator demo publish a
            # zero, stale, or occasionally irregular source stamp.  Camera
            # freshness is therefore based on local arrival time.  A bad
            # frame is isolated to this callback and the next frame is still
            # eligible immediately.
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            observations = self._tracker.detect(frame, camera_name, arrival)
            with self._lock:
                last_jpeg = self._jpeg_at[camera_name]
            jpeg_period = max(
                0.1, float(self._config['web']['poll_interval_sec'])
            )
            jpeg_ok = False
            encoded = None
            if last_jpeg is None or arrival - last_jpeg >= jpeg_period:
                jpeg_ok, encoded = cv2.imencode(
                    '.jpg',
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 78],
                )
            visible = []
            with self._lock:
                self._frames[camera_name] = frame
                self._frame_at[camera_name] = arrival
                if jpeg_ok:
                    self._camera_jpegs[camera_name] = encoded.tobytes()
                    self._jpeg_at[camera_name] = arrival
                valid_keys: set[tuple[str, int]] = set()
                for observation in observations:
                    marker_id = int(getattr(observation, 'marker_id'))
                    if bool(getattr(observation, 'pose_valid', False)):
                        key = (camera_name, marker_id)
                        valid_keys.add(key)
                        self._observations[key] = observation
                    visible.append(self._observation_payload(observation))
                observation_stale_sec = float(
                    self._config['timeouts']['marker_observation_stale_sec']
                )
                for key in list(self._observations):
                    if key[0] == camera_name and key not in valid_keys:
                        previous = self._observations[key]
                        if (
                            arrival - float(previous.timestamp_sec)
                            > observation_stale_sec
                        ):
                            del self._observations[key]
                self._visible_markers[camera_name] = visible
                active_camera = self._metrics.get('active_camera')
                target_id = self._metrics.get('target_marker_id')
                target_observation = (
                    None
                    if active_camera != camera_name or target_id is None
                    else self._observations.get(
                        (camera_name, int(target_id))
                    )
                )
                if (
                    active_camera == camera_name
                    and target_id is not None
                    and (
                        target_observation is None
                        or arrival - float(target_observation.timestamp_sec)
                        > observation_stale_sec
                    )
                ):
                    self._clear_detection_metrics_locked()
        except Exception as exc:
            self._record_event(
                'warning',
                f'{camera_name} frame processing failed: {exc}',
            )

    def _camera_info_callback(
        self,
        camera_name: str,
        message: CameraInfo,
    ) -> None:
        now = _now()
        if self._config['cameras'][camera_name].get(
            'use_camera_info', False
        ):
            try:
                camera_cfg = self._config['cameras'][camera_name]
                self._tracker.set_camera_calibration(camera_name, {
                    'camera_matrix': list(message.k),
                    'distortion_coefficients': list(message.d),
                    'flip_horizontal': bool(
                        camera_cfg.get('flip_horizontal', False)
                    ),
                    'flip_vertical': bool(
                        camera_cfg.get('flip_vertical', False)
                    ),
                })
                with self._lock:
                    self._camera_info_at[camera_name] = now
            except Exception as exc:
                self._record_event(
                    'warning',
                    f'{camera_name} CameraInfo rejected: {exc}',
                )
        else:
            with self._lock:
                self._camera_info_at[camera_name] = now

    def _odom_callback(self, message: Odometry) -> None:
        try:
            yaw = _yaw_from_odom(message)
        except Exception as exc:
            self._record_event('warning', f'Invalid odometry: {exc}')
            return
        with self._lock:
            stale_limit = float(self._config['timeouts']['odom_stale_sec'])
        if not self._source_stamp_is_fresh(message.header.stamp, stale_limit):
            return
        stamp = self._stamp_seconds(message.header.stamp)
        with self._lock:
            if (
                self._last_odom_header_stamp is not None
                and stamp <= self._last_odom_header_stamp
            ):
                return
            self._last_odom_header_stamp = stamp
            self._yaw_rad = yaw
            self._odom_at = _now()

    def _joint_state_callback(self, message: JointState) -> None:
        arrival = _now()
        if not self._source_stamp_is_fresh(
            message.header.stamp,
            float(self._config['timeouts']['joint_state_stale_sec']),
        ):
            return
        with self._lock:
            for index, name in enumerate(message.name):
                if index >= len(message.position):
                    break
                value = float(message.position[index])
                if math.isfinite(value):
                    self._joint_positions[str(name)] = value
                    self._joint_update_at[str(name)] = arrival

    @staticmethod
    def _stamp_seconds(stamp: Any) -> float:
        return (
            float(stamp.sec)
            + float(stamp.nanosec) / 1_000_000_000.0
        )

    def _source_stamp_is_fresh(self, stamp: Any, max_age_sec: float) -> bool:
        source = self._stamp_seconds(stamp)
        if source <= 0.0 or not math.isfinite(source):
            return False
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        age = now - source
        return -0.1 <= age <= float(max_age_sec)

    @staticmethod
    def _observation_payload(observation: Any) -> dict[str, Any]:
        def optional_float(name: str) -> float | None:
            value = getattr(observation, name, None)
            return None if value is None else float(value)

        return {
            'marker_id': int(getattr(observation, 'marker_id')),
            'camera_name': str(getattr(observation, 'camera_name')),
            'timestamp_sec': float(getattr(observation, 'timestamp_sec')),
            'pose_valid': bool(getattr(observation, 'pose_valid', False)),
            'distance_m': optional_float('distance_m'),
            'lateral_m': optional_float('lateral_m'),
            'yaw_rad': optional_float('yaw_rad'),
        }

    # ------------------------------------------------------------------
    # Safety output and public status

    def _velocity_graph_status(self) -> dict[str, Any]:
        with self._lock:
            raw_topic = str(self._config['topics']['cmd_vel_raw'])
            hardware_topic = str(self._config['topics']['cmd_vel'])
        try:
            raw_subscriptions = self.get_subscriptions_info_by_topic(
                raw_topic
            )
            raw_publishers = self.get_publishers_info_by_topic(raw_topic)
            hardware_publishers = self.get_publishers_info_by_topic(
                hardware_topic
            )
        except Exception:
            raw_subscriptions = []
            raw_publishers = []
            hardware_publishers = []

        watchdog_raw_subscribers = [
            info
            for info in raw_subscriptions
            if (
                info.node_name == 'final_project_presentation2_watchdog'
                and info.topic_type == 'geometry_msgs/msg/TwistStamped'
            )
        ]
        central_raw_publishers = [
            info
            for info in raw_publishers
            if (
                info.node_name == 'final_project_presentation2'
                and info.topic_type == 'geometry_msgs/msg/TwistStamped'
            )
        ]
        watchdog_cmd_publishers = [
            info
            for info in hardware_publishers
            if (
                info.node_name == 'final_project_presentation2_watchdog'
                and info.topic_type == 'geometry_msgs/msg/Twist'
            )
        ]
        ready = (
            len(watchdog_raw_subscribers) == 1
            and len(raw_publishers) == 1
            and len(central_raw_publishers) == 1
            and len(hardware_publishers) == 1
            and len(watchdog_cmd_publishers) == 1
        )
        return {
            'ready': ready,
            'raw_subscription_count': len(raw_subscriptions),
            'raw_publisher_count': len(raw_publishers),
            'cmd_vel_publisher_count': len(hardware_publishers),
        }

    def _publish_command(self) -> None:
        now = _now()
        cancel_for_lease = False
        cancel_for_graph = False
        graph_error: str | None = None
        with self._lock:
            motion_active = (
                self._worker is not None
                and self._worker.is_alive()
                and self._active_kind in {
                    'route',
                    'manual_turn',
                    'marker_test',
                    'route_continue',
                    'workflow',
                }
            )
        graph = self._velocity_graph_status() if motion_active else None
        with self._lock:
            command = self._desired_command
            if graph is not None and not graph['ready']:
                command = ZERO_COMMAND
                if self._safety_abort_reason is None:
                    graph_error = (
                        'Velocity graph safety contract changed during '
                        'motion: expected the presentation node as the sole '
                        'raw publisher and the watchdog as the sole '
                        '/cmd_vel publisher'
                    )
                    self._safety_abort_reason = graph_error
                    self._stop_event.set()
                    self._return_confirm_event.set()
                    cancel_for_graph = True
            control_rate = float(
                self._config['motion_control']['control_rate_hz']
            )
            if now - self._desired_command_at > max(
                0.15, 2.5 / control_rate
            ):
                command = ZERO_COMMAND
            lease_timeout = float(
                self._config['safety'].get(
                    'operator_lease_timeout_sec', 2.5
                )
            )
            if now - self._operator_lease_at > lease_timeout:
                command = ZERO_COMMAND
                if (
                    not self._lease_expiry_triggered
                    and self._worker is not None
                    and self._worker.is_alive()
                ):
                    self._lease_expiry_triggered = True
                    self._safety_abort_reason = (
                        'Operator heartbeat expired during active operation'
                    )
                    self._stop_event.set()
                    self._return_confirm_event.set()
                    cancel_for_lease = True

            if (
                self._shutdown_event.is_set()
                or self._stop_event.is_set()
                or self._state in {'STOPPING', 'ERROR_LATCHED'}
            ):
                # This final gate closes the race where a worker passes its
                # stop check and writes one last nonzero desired command after
                # STOP or shutdown has already forced zero.
                command = ZERO_COMMAND

            maximum_linear = float(
                self._config['safety']['max_linear_mps']
            )
            maximum_angular = float(
                self._config['safety']['max_angular_rps']
            )
            linear = max(
                -maximum_linear,
                min(maximum_linear, float(command.linear_x)),
            )
            angular = max(
                -maximum_angular,
                min(maximum_angular, float(command.angular_z)),
            )
            if not math.isfinite(linear) or not math.isfinite(angular):
                linear = 0.0
                angular = 0.0

        message = self._raw_command_message(linear, angular)
        self._raw_cmd_publisher.publish(message)
        if cancel_for_lease or cancel_for_graph:
            self._pose_executor.cancel_active(wait_timeout_sec=0.0)
        if cancel_for_lease:
            self._record_event(
                'error',
                'Operator heartbeat expired; active operation was stopped',
            )
        if graph_error is not None:
            self._record_event('error', graph_error)

    def _set_command(self, command: VelocityCommand) -> None:
        with self._lock:
            self._desired_command = command
            self._desired_command_at = _now()

    def _raw_command_message(
        self,
        linear_x: float = 0.0,
        angular_z: float = 0.0,
    ) -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        return message

    def _force_zero(self) -> None:
        self._set_command(ZERO_COMMAND)
        for _ in range(3):
            self._raw_cmd_publisher.publish(self._raw_command_message())

    def _publish_watchdog_config(self) -> None:
        with self._lock:
            safety = deepcopy(self._config['safety'])
            payload = {
                'schema_version': 1,
                'revision': int(self._config.get('revision', 0)),
                'valid': self._config_error is None,
                'cmd_timeout_sec': float(
                    safety['watchdog_cmd_timeout_sec']
                ),
                'publish_rate_hz': float(safety['watchdog_rate_hz']),
                'max_linear_mps': float(safety['max_linear_mps']),
                'max_angular_rps': float(safety['max_angular_rps']),
                'reject_nonfinite_commands': bool(
                    safety.get('reject_nonfinite_commands', True)
                ),
            }
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self._watchdog_config_publisher.publish(message)

    def _readiness(self) -> dict[str, Any]:
        now = _now()
        graph = self._velocity_graph_status()
        with self._lock:
            timeout = self._config['timeouts']
            enforce_arm_status = bool(
                self._config['safety'].get(
                    'require_arm_fault_clear', False
                )
            )
            arm_status_available = (
                self._arm_status_client.service_is_ready()
            )

            def fresh(stamp: float | None, limit: float) -> dict[str, Any]:
                age = None if stamp is None else max(0.0, now - stamp)
                return {
                    'ready': age is not None and age <= float(limit),
                    'age_sec': age,
                }

            def camera_ready(camera_name: str) -> dict[str, Any]:
                image = fresh(
                    self._frame_at[camera_name], timeout['camera_stale_sec']
                )
                use_info = bool(
                    self._config['cameras'][camera_name]['use_camera_info']
                )
                info = fresh(
                    self._camera_info_at[camera_name],
                    timeout['camera_info_sec'],
                )
                image['camera_info_ready'] = (
                    info['ready'] if use_info else True
                )
                image['camera_info_age_sec'] = info['age_sec']
                image['ready'] = (
                    image['ready'] and image['camera_info_ready']
                )
                return image

            readiness = {
                'config': {
                    'ready': (
                        self._config_error is None
                        and 'topics' not in self._restart_required
                    ),
                    'message': (
                        self._config_error
                        or (
                            'Node restart required for topic changes'
                            if 'topics' in self._restart_required
                            else None
                        )
                    ),
                },
                'front_camera': camera_ready('front'),
                'rear_camera': camera_ready('rear'),
                'odom': fresh(
                    self._odom_at, timeout['odom_stale_sec']
                ),
                'watchdog': {
                    'ready': self._config_error is None and graph['ready'],
                    'raw_subscription_count': graph[
                        'raw_subscription_count'
                    ],
                    'raw_publisher_count': graph['raw_publisher_count'],
                    'cmd_vel_publisher_count': graph[
                        'cmd_vel_publisher_count'
                    ],
                    'message': (
                        None
                        if graph['ready']
                        else 'Expected the presentation node as the sole '
                        'raw publisher and the watchdog as the sole '
                        '/cmd_vel publisher'
                    ),
                },
                'arm': {
                    'ready': self._pose_executor.arm_ready,
                },
                'gripper': {
                    'ready': self._pose_executor.gripper_ready,
                },
                'arm_status_service': {
                    'ready': (
                        arm_status_available
                        if enforce_arm_status
                        else True
                    ),
                    'available': arm_status_available,
                    'enforced': enforce_arm_status,
                    'message': (
                        None
                        if enforce_arm_status
                        else 'Demo mode: arm status is advisory; the action '
                        'server makes the final admission decision'
                    ),
                },
                'action_state': {
                    'ready': self._pose_executor.shutdown_drained,
                    'message': (
                        None
                        if self._pose_executor.shutdown_drained
                        else 'An arm or gripper action is not confirmed '
                        'terminal'
                    ),
                },
            }
            drive_required = (
                'config',
                'front_camera',
                'rear_camera',
                'odom',
                'watchdog',
                'arm_status_service',
                'action_state',
            )
            turn_required = (
                'config',
                'odom',
                'watchdog',
                'arm_status_service',
                'action_state',
            )
            pose_required = (
                'config',
                'arm',
                'gripper',
                'arm_status_service',
                'action_state',
            )
            readiness['drive_ready'] = all(
                readiness[name]['ready'] for name in drive_required
            )
            readiness['turn_ready'] = all(
                readiness[name]['ready'] for name in turn_required
            )
            readiness['pose_ready'] = all(
                readiness[name]['ready'] for name in pose_required
            )
            readiness['workflow_ready'] = (
                readiness['drive_ready'] and readiness['pose_ready']
            )
            # Backward-compatible alias used by older dashboard snapshots.
            readiness['motion_ready'] = readiness['drive_ready']
            readiness['motion_block_reasons'] = [
                name
                for name in drive_required
                if not readiness[name]['ready']
            ]
            readiness['turn_block_reasons'] = [
                name
                for name in turn_required
                if not readiness[name]['ready']
            ]
            readiness['pose_block_reasons'] = [
                name
                for name in pose_required
                if not readiness[name]['ready']
            ]
            return readiness

    def snapshot(self) -> dict[str, Any]:
        readiness = self._readiness()
        now = _now()
        with self._lock:
            document = deepcopy(self._config)
            categories = [
                {'id': int(key), **value}
                for key, value in sorted(
                    document.get('categories', {}).items(),
                    key=lambda item: int(item[0]),
                )
            ]
            poses = [
                self._pose_to_api(int(key), value)
                for key, value in sorted(
                    document.get('poses', {}).items(),
                    key=lambda item: int(item[0]),
                )
            ]
            workflows = [
                {'id': int(key), **value}
                for key, value in sorted(
                    document.get('workflows', {}).items(),
                    key=lambda item: int(item[0]),
                )
            ]
            joint_feedback = {
                name: {
                    'position_deg': math.degrees(position),
                    'age_sec': (
                        None
                        if name not in self._joint_update_at
                        else now - self._joint_update_at[name]
                    ),
                }
                for name, position in self._joint_positions.items()
                if name in set(ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES)
            }
            arm_feedback_deg = [
                (
                    None
                    if name not in self._joint_positions
                    else math.degrees(self._joint_positions[name])
                )
                for name in ARM_JOINT_NAMES
            ]
            gripper_feedback_deg = [
                (
                    None
                    if name not in self._joint_positions
                    else math.degrees(self._joint_positions[name])
                )
                for name in GRIPPER_JOINT_NAMES
            ]
            if any(value is None for value in arm_feedback_deg):
                arm_feedback_deg = []
            if any(value is None for value in gripper_feedback_deg):
                gripper_feedback_deg = []
            worker_active = (
                self._worker is not None and self._worker.is_alive()
            )
            display_metrics = deepcopy(self._metrics)
            active_camera = display_metrics.get('active_camera')
            target_id = display_metrics.get('target_marker_id')
            observation = None
            if active_camera in {'front', 'rear'} and target_id is not None:
                observation = self._observations.get(
                    (str(active_camera), int(target_id))
                )
            observation_fresh = (
                observation is not None
                and now - float(observation.timestamp_sec)
                <= float(document['timeouts']['marker_observation_stale_sec'])
            )
            if not observation_fresh:
                for key in (
                    'detected_marker_id',
                    'distance_m',
                    'lateral_m',
                    'yaw_error_deg',
                    'marker_yaw_deg',
                    'distance_error_m',
                    'lateral_error_m',
                ):
                    display_metrics[key] = None
            vision = {
                'active_camera': display_metrics['active_camera'],
                'target_id': display_metrics['target_marker_id'],
                'detected_id': display_metrics['detected_marker_id'],
                'distance_m': display_metrics['distance_m'],
                'lateral_m': display_metrics['lateral_m'],
                'yaw_error_deg': display_metrics.get('yaw_error_deg'),
                'observation_age_sec': (
                    None
                    if observation is None
                    else max(0.0, now - float(observation.timestamp_sec))
                ),
                'detected': display_metrics['detected_marker_id'] is not None,
                'detection': {
                    'id': display_metrics['detected_marker_id'],
                    'visible': display_metrics['detected_marker_id'] is not None,
                    'distance_m': display_metrics['distance_m'],
                    'lateral_m': display_metrics['lateral_m'],
                    'yaw_error_deg': display_metrics.get('yaw_error_deg'),
                },
            }
            all_visible_markers = deepcopy(self._visible_markers)
            visible_markers = deepcopy(all_visible_markers)
            target_status = 'idle'
            target_status_message = None
            # During an active approach, expose only the configured target
            # from the configured camera.  A marker mounted for the opposite
            # travel direction may still be visible, but it must not become
            # the dashboard's active detection or influence control.
            if active_camera in {'front', 'rear'} and target_id is not None:
                target_camera = str(active_camera)
                target_marker_id = int(target_id)
                visible_markers = {
                    camera: [
                        marker
                        for marker in markers
                        if camera == target_camera
                        and int(marker.get('marker_id', -1))
                        == target_marker_id
                    ]
                    for camera, markers in visible_markers.items()
                }
                expected_candidates = [
                    marker
                    for marker in all_visible_markers[target_camera]
                    if int(marker.get('marker_id', -1)) == target_marker_id
                ]
                wrong_camera_candidates = [
                    marker
                    for camera, markers in all_visible_markers.items()
                    if camera != target_camera
                    for marker in markers
                    if int(marker.get('marker_id', -1)) == target_marker_id
                ]
                other_ids = sorted({
                    int(marker.get('marker_id', -1))
                    for marker in all_visible_markers[target_camera]
                    if int(marker.get('marker_id', -1)) >= 0
                    and int(marker.get('marker_id', -1)) != target_marker_id
                })
                if observation_fresh:
                    target_status = 'tracking'
                    target_status_message = (
                        f'{target_camera} camera is tracking ID '
                        f'{target_marker_id}'
                    )
                elif expected_candidates:
                    target_status = 'pose_invalid'
                    target_status_message = (
                        f'ID {target_marker_id} is visible on {target_camera} '
                        'but metric pose is invalid'
                    )
                elif wrong_camera_candidates:
                    cameras = sorted({
                        str(marker.get('camera_name', 'unknown'))
                        for marker in wrong_camera_candidates
                    })
                    target_status = 'wrong_camera'
                    target_status_message = (
                        f'ID {target_marker_id} is visible on '
                        f'{", ".join(cameras)}, expected {target_camera}'
                    )
                elif other_ids:
                    target_status = 'wrong_id'
                    target_status_message = (
                        f'{target_camera} sees IDs {other_ids}, expected '
                        f'ID {target_marker_id}'
                    )
                else:
                    target_status = 'not_visible'
                    target_status_message = (
                        f'Waiting for ID {target_marker_id} on '
                        f'{target_camera}'
                    )
            vision['target_status'] = target_status
            vision['target_status_message'] = target_status_message
            arm_board_status = deepcopy(self._arm_board_status)
            status_updated_at = arm_board_status.get('updated_at_sec')
            arm_board_status['age_sec'] = (
                None
                if status_updated_at is None
                else max(0.0, time.time() - float(status_updated_at))
            )
            result = {
                'timestamp': int(time.time() * 1000),
                'poll_interval_sec': float(
                    document['web']['poll_interval_sec']
                ),
                'revision': int(document.get('revision', 0)),
                'schema_version': int(document.get('schema_version', 1)),
                'active': worker_active,
                'error_latched': self._state == 'ERROR_LATCHED',
                'error': self._error,
                'workflow_progress': deepcopy(self._workflow_progress),
                'workflow_recovery': deepcopy(self._workflow_recovery),
                'waiting_return_confirm': (
                    self._state == 'WAITING_RETURN_CONFIRM'
                ),
                'motion_ready': readiness['motion_ready'],
                'run': {
                    'active': worker_active,
                    'name': self._active_name,
                    'kind': self._active_kind,
                    'route': self._route_direction,
                    'step_name': self._phase,
                    'waiting_return_confirm': (
                        self._state == 'WAITING_RETURN_CONFIRM'
                    ),
                },
                'state': {
                    'name': self._state,
                    'active_kind': self._active_kind,
                    'active_name': self._active_name,
                    'route_direction': self._route_direction,
                    'phase': self._phase,
                    'error': self._error,
                    'checkpoint': deepcopy(self._checkpoint),
                    'workflow_recovery': deepcopy(
                        self._workflow_recovery
                    ),
                },
                'readiness': readiness,
                'metrics': display_metrics,
                'vision': vision,
                'turn': {
                    'actual_deg': self._metrics['turn_progress_deg'],
                    'target_deg': self._metrics['turn_target_deg'],
                },
                'visible_markers': visible_markers,
                'all_visible_markers': all_visible_markers,
                'watchdog': deepcopy(readiness['watchdog']),
                'arm_board_status': arm_board_status,
                'joint_feedback': joint_feedback,
                'categories': categories,
                'poses': poses,
                'workflows': workflows,
                'route': {
                    'name': self._route_direction,
                    **deepcopy(document['route']),
                },
                'route_config': deepcopy(document['route']),
                'markers': deepcopy(document['markers']),
                'turn_control': deepcopy(document['turn_control']),
                'arm': {
                    **deepcopy(document['arm']),
                    'feedback_deg': arm_feedback_deg,
                },
                'gripper': {
                    **deepcopy(document['gripper']),
                    'feedback_deg': gripper_feedback_deg,
                },
                'config_info': {
                    'path': str(self._config_path),
                    'schema_version': int(document['schema_version']),
                    'revision': int(document['revision']),
                    'valid': self._config_error is None,
                    'restart_required': list(self._restart_required),
                },
                'events': list(self._events),
            }
        return _json_safe(result)

    def renew_operator_lease(self) -> dict[str, Any]:
        with self._lock:
            self._operator_lease_at = _now()
        return {'lease': 'renewed'}

    def camera_jpeg(self, camera_name: str) -> bytes | None:
        with self._lock:
            # Frame freshness proves that the camera is live.  JPEG encoding
            # is intentionally throttled to the UI poll rate, so its own
            # timestamp can be older even while new source frames arrive.
            frame_stamp = self._frame_at.get(camera_name)
            jpeg_stamp = self._jpeg_at.get(camera_name)
            now = _now()
            frame_limit = float(
                self._config['timeouts']['camera_stale_sec']
            )
            jpeg_limit = max(
                frame_limit,
                2.0 * float(self._config['web']['poll_interval_sec']),
            )
            if (
                frame_stamp is None
                or jpeg_stamp is None
                or now - frame_stamp > frame_limit
                or now - jpeg_stamp > jpeg_limit
            ):
                return None
            return self._camera_jpegs.get(camera_name)

    def _record_event(self, level: str, message: str) -> None:
        entry = {
            'time_unix_ms': int(time.time() * 1000),
            'timestamp': int(time.time() * 1000),
            'level': str(level),
            'message': str(message),
        }
        with self._lock:
            self._events.appendleft(entry)
        logger = self.get_logger()
        if level == 'error':
            logger.error(str(message))
        elif level == 'warning':
            logger.warning(str(message))
        else:
            logger.info(str(message))

    def report_http_exception(self, error: Exception) -> None:
        self._record_event(
            'error',
            f'HTTP handler error: {error}\n{traceback.format_exc()}',
        )

    def _start_web_server(self) -> None:
        share = Path(get_package_share_directory('final_project_presentation2'))
        app = create_app(self, share / 'static')
        web = self._config['web']
        self._web_server = make_server(
            str(web['host']),
            int(web['port']),
            app,
            threaded=True,
        )
        self._web_thread = threading.Thread(
            target=self._web_server.serve_forever,
            name='final-project-web',
            daemon=True,
        )
        self._web_thread.start()
        self.get_logger().info(
            f"Web UI: http://{web['host']}:{int(web['port'])}"
        )

    # ------------------------------------------------------------------
    # Worker lifecycle and safety gates

    def _start_worker(
        self,
        *,
        kind: str,
        name: str,
        target: Callable[[threading.Event], None],
        allow_checkpoint: bool = False,
        allow_workflow_recovery: bool = False,
    ) -> dict[str, Any]:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError(
                    f'Another operation is active: {self._active_name}',
                    409,
                )
            if self._config_error is not None:
                raise ApiError(
                    f'Configuration is invalid: {self._config_error}',
                    409,
                )
            if 'topics' in self._restart_required:
                raise ApiError(
                    'ROS topic configuration changed; restart the node '
                    'before starting an operation',
                    409,
                )
            if self._state == 'ERROR_LATCHED':
                raise ApiError(
                    'Clear the latched error before starting another operation',
                    409,
                )
            if self._checkpoint is not None and not allow_checkpoint:
                raise ApiError(
                    'A route checkpoint is pending; continue or discard it first',
                    409,
                    checkpoint=deepcopy(self._checkpoint),
                )
            if (
                self._workflow_recovery is not None
                and not allow_workflow_recovery
            ):
                raise ApiError(
                    'A workflow step is paused; retry, skip, or end the '
                    'paused workflow first',
                    409,
                    workflow_recovery=deepcopy(self._workflow_recovery),
                )
            self._stop_event = threading.Event()
            self._return_confirm_event.clear()
            self._operator_lease_at = _now()
            self._lease_expiry_triggered = False
            self._safety_abort_reason = None
            self._state = 'RUNNING'
            self._active_kind = str(kind)
            self._active_name = str(name)
            self._phase = 'starting'
            self._error = None
            self._metrics['active_camera'] = None
            self._metrics['target_marker_id'] = None
            self._metrics['control_reason'] = None
            self._clear_detection_metrics_locked()
            self._clear_turn_metrics_locked()
            stop_event = self._stop_event
            worker = threading.Thread(
                target=self._worker_entry,
                args=(target, stop_event),
                name=f'final-project-{kind}',
                daemon=True,
            )
            self._worker = worker
            worker.start()
        self._record_event('info', f'Started {kind}: {name}')
        return {'started': True, 'kind': kind, 'name': name}

    def _worker_entry(
        self,
        target: Callable[[threading.Event], None],
        stop_event: threading.Event,
    ) -> None:
        stopped = False
        recoverable_message: str | None = None
        try:
            target(stop_event)
            with self._lock:
                safety_reason = self._safety_abort_reason
            if safety_reason is not None:
                raise RuntimeError(safety_reason)
            stopped = stop_event.is_set()
        except OperationStopped as exc:
            with self._lock:
                safety_reason = self._safety_abort_reason
            if safety_reason is None:
                stopped = True
                self._record_event('warning', str(exc))
            else:
                self._latch_worker_error(safety_reason)
                return
        except RecoverableOperationError as exc:
            with self._lock:
                safety_reason = self._safety_abort_reason
            if safety_reason is not None:
                self._latch_worker_error(safety_reason)
                return
            if self._shutdown_event.is_set() or stop_event.is_set():
                stopped = True
                self._record_event('warning', 'Operation stopped by user')
            else:
                recoverable_message = str(exc)
                self._record_event(
                    'warning',
                    f'{recoverable_message}; robot stopped and the step may '
                    'be retried',
                )
        except Exception as exc:
            self._latch_worker_error(str(exc))
            return
        finally:
            self._force_zero()

        with self._lock:
            self._state = 'IDLE'
            if recoverable_message is not None:
                if getattr(self, '_workflow_recovery', None) is None:
                    self._phase = 'incomplete'
            else:
                self._phase = 'stopped' if stopped else 'complete'
            self._active_kind = None
            self._active_name = None
            self._route_direction = None
            self._metrics['active_camera'] = None
            self._metrics['target_marker_id'] = None
            self._metrics['control_reason'] = None
            self._clear_detection_metrics_locked()
            if stopped or recoverable_message is not None:
                self._error = None
        if recoverable_message is not None:
            return
        self._record_event(
            'warning' if stopped else 'info',
            'Operation stopped' if stopped else 'Operation completed',
        )

    def _latch_worker_error(self, message: str) -> None:
        self._force_zero()
        self._pose_executor.cancel_active(wait_timeout_sec=0.0)
        with self._lock:
            self._state = 'ERROR_LATCHED'
            self._phase = 'failed'
            self._error = str(message)
            self._metrics['active_camera'] = None
            self._metrics['target_marker_id'] = None
            self._metrics['control_reason'] = None
            self._clear_detection_metrics_locked()
            progress = getattr(self, '_workflow_progress', None)
            if progress is not None and progress.get('status') == 'running':
                progress['status'] = 'failed_safety'
                progress['error'] = str(message)
        self._record_event('error', str(message))

    def _check_stop(
        self,
        stop_event: threading.Event,
        *,
        require_lease: bool = False,
    ) -> None:
        with self._lock:
            safety_reason = self._safety_abort_reason
        if safety_reason is not None:
            raise RuntimeError(safety_reason)
        if self._shutdown_event.is_set() or stop_event.is_set():
            raise OperationStopped('Operation stopped by user')
        if require_lease:
            with self._lock:
                age = _now() - self._operator_lease_at
                timeout = float(
                    self._config['safety'].get(
                        'operator_lease_timeout_sec', 2.5
                    )
                )
            if age > timeout:
                raise RuntimeError(
                    f'Operator heartbeat expired ({age:.2f}s > {timeout:.2f}s)'
                )

    def _set_phase(self, phase: str, **metrics: Any) -> None:
        with self._lock:
            self._phase = str(phase)
            self._metrics.update(metrics)

    def stop(self) -> dict[str, Any]:
        with self._lock:
            self._stop_event.set()
            self._return_confirm_event.set()
            active = self._active_name
            if self._worker is not None and self._worker.is_alive():
                self._state = 'STOPPING'
                self._phase = 'emergency_stop'
        self._pose_executor.cancel_active(wait_timeout_sec=0.0)
        self._force_zero()
        self._record_event('warning', f'STOP requested (active={active})')
        return {'stopping': bool(active), 'active': active}

    def clear_error(self) -> dict[str, Any]:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError('Cannot clear an error while a worker is active', 409)
            if not self._pose_executor.shutdown_drained:
                raise ApiError(
                    'Cannot clear the error because an arm or gripper action '
                    'is not confirmed terminal; restart the node',
                    409,
                )
            if self._config_error is not None:
                raise ApiError(
                    f'Reload a valid config first: {self._config_error}',
                    409,
                )
            previous = self._error
            self._error = None
            self._state = 'IDLE'
            self._active_kind = None
            self._active_name = None
            self._route_direction = None
            self._safety_abort_reason = None
            self._phase = (
                'checkpoint_ready' if self._checkpoint else 'ready'
            )
        self._force_zero()
        return {
            'cleared': previous,
            'checkpoint': deepcopy(self._checkpoint),
        }

    def _poll_arm_board_status(self) -> None:
        """Refresh board state asynchronously without blocking ROS timers."""
        with self._lock:
            pending = self._arm_status_poll_future
            if pending is not None and not pending.done():
                return
        if not self._arm_status_client.service_is_ready():
            with self._lock:
                self._arm_board_status.update({
                    'available': False,
                    'service_success': False,
                    'message': '/arm_board/status is unavailable',
                })
            return
        try:
            future = self._arm_status_client.call_async(Trigger.Request())
        except Exception as exc:
            with self._lock:
                self._arm_board_status.update({
                    'available': False,
                    'service_success': False,
                    'message': f'Arm status request failed: {exc}',
                })
            return
        with self._lock:
            self._arm_status_poll_future = future
        future.add_done_callback(self._arm_status_poll_done)

    def _arm_status_poll_done(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:
            with self._lock:
                self._arm_board_status.update({
                    'available': False,
                    'service_success': False,
                    'message': f'Arm status response failed: {exc}',
                })
                self._arm_status_poll_future = None
            return
        self._store_arm_status_response(response)
        with self._lock:
            self._arm_status_poll_future = None

    def _store_arm_status_response(self, response: Any) -> dict[str, Any]:
        message = '' if response is None else str(response.message)
        parsed = parse_arm_board_status(message)
        parsed.update({
            'available': response is not None,
            'service_success': bool(
                response is not None and response.success
            ),
            'updated_at_sec': time.time(),
            'message': message or 'Empty arm status response',
        })
        with self._lock:
            self._arm_board_status = parsed
        return parsed

    def arm_command(
        self,
        command: str,
        *,
        confirmed: bool = False,
    ) -> dict[str, Any]:
        """Run one explicit arm-board command requested by the operator."""
        normalized = str(command).strip().lower()
        client = self._arm_command_clients.get(normalized)
        if client is None:
            raise ApiError(f'Unknown arm command: {command}', 404)
        if normalized == 'disable' and not confirmed:
            raise ApiError('Disable requires explicit confirmation', 400)

        if normalized in {'disable', 'estop'}:
            self.stop()
        elif normalized != 'status':
            with self._lock:
                if self._worker is not None and self._worker.is_alive():
                    raise ApiError(
                        'Cannot run an arm-board command while an operation '
                        'is active',
                        409,
                    )

        timeout_key = (
            'arm_home_sec' if normalized == 'home' else 'arm_service_sec'
        )
        timeout_sec = float(self._config['timeouts'].get(
            timeout_key,
            185.0 if normalized == 'home' else 20.0,
        ))
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise ApiError(
                f'Arm {normalized} service is unavailable',
                503,
            )

        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout_sec):
            future.cancel()
            raise ApiError(f'Arm {normalized} service timed out', 504)
        try:
            response = future.result()
        except Exception as exc:
            raise ApiError(
                f'Arm {normalized} service failed: {exc}',
                503,
            ) from exc
        parsed_status = self._store_arm_status_response(response)
        if normalized == 'status':
            return {
                'command': normalized,
                'success': bool(response is not None and response.success),
                'message': '' if response is None else str(response.message),
                'arm_board_status': parsed_status,
            }
        if response is None or not bool(response.success):
            detail = '' if response is None else str(response.message)
            raise ApiError(
                f'Arm {normalized} was rejected: {detail}',
                409,
            )

        detail = str(response.message)
        self._record_event(
            'warning' if normalized in {'disable', 'estop'} else 'info',
            f'Arm board command {normalized}: {detail or "success"}',
        )
        return {
            'command': normalized,
            'success': True,
            'message': detail,
        }

    def discard_checkpoint(self) -> dict[str, Any]:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError('Cannot discard an active checkpoint', 409)
            previous = self._checkpoint
            self._checkpoint = None
        return {'discarded': previous}

    def _verify_arm_hardware(self, stop_event: threading.Event) -> None:
        if not bool(
            self._config['safety'].get('require_arm_fault_clear', True)
        ):
            return
        timeout = float(self._config['timeouts'].get('stop_sec', 2.0))
        deadline = _now() + timeout
        while not self._arm_status_client.wait_for_service(timeout_sec=0.1):
            self._check_stop(stop_event)
            if _now() >= deadline:
                raise RuntimeError('Arm status service is unavailable')
        future = self._arm_status_client.call_async(Trigger.Request())
        while not future.done():
            self._check_stop(stop_event)
            if _now() >= deadline:
                raise RuntimeError('Arm status service timed out')
            time.sleep(0.02)
        response = future.result()
        detail = '' if response is None else str(response.message)
        if (
            response is None
            or not response.success
            or 'accept_traj=False' in detail
        ):
            raise RuntimeError(f'Arm hardware is not ready: {detail}')

    def _motion_preflight(self, stop_event: threading.Event) -> None:
        readiness = self._readiness()
        if not readiness['drive_ready']:
            reasons = ', '.join(readiness['motion_block_reasons'])
            raise RuntimeError(f'Motion preflight failed: {reasons}')
        self._verify_arm_hardware(stop_event)

    def _turn_preflight(self, stop_event: threading.Event) -> None:
        readiness = self._readiness()
        if not readiness['turn_ready']:
            reasons = ', '.join(readiness['turn_block_reasons'])
            raise RuntimeError(f'Turn preflight failed: {reasons}')
        self._verify_arm_hardware(stop_event)

    def _require_runtime_ready(
        self,
        ready_field: str,
        reasons_field: str,
        operation: str,
    ) -> None:
        readiness = self._readiness()
        if readiness.get(ready_field, False):
            return
        reasons = ', '.join(readiness.get(reasons_field, [])) or 'not ready'
        raise ApiError(f'{operation} is not ready: {reasons}', 409)

    def _require_pose_ready(self, pose: dict[str, Any]) -> None:
        readiness = self._readiness()
        reasons: list[str] = []
        if not readiness['config']['ready']:
            reasons.append('config')
        if not readiness['arm_status_service']['ready']:
            reasons.append('arm_status_service')
        if pose['arm'].get('enabled') and not readiness['arm']['ready']:
            reasons.append('arm')
        if pose['gripper'].get('enabled') and not readiness['gripper']['ready']:
            reasons.append('gripper')
        if reasons:
            raise ApiError(
                f'Pose execution is not ready: {", ".join(reasons)}',
                409,
            )

    # ------------------------------------------------------------------
    # Straight two-marker route execution

    def start_route(self, direction: str) -> dict[str, Any]:
        if direction not in {'dropoff', 'pickup'}:
            raise ApiError('Route must be dropoff or pickup', 400)
        with self._lock:
            self._require_runtime_ready(
                'drive_ready', 'motion_block_reasons', 'Route motion'
            )
            route_key = 'outbound' if direction == 'dropoff' else 'return'
            return self._start_worker(
                kind='route',
                name=route_key,
                target=lambda stop: self._run_route(route_key, stop),
            )

    def start_manual_turn(self, degrees: Any) -> dict[str, Any]:
        try:
            target_degrees = float(degrees)
        except (TypeError, ValueError) as exc:
            raise ApiError('degrees must be a number', 400) from exc
        if (
            not math.isfinite(target_degrees)
            or target_degrees == 0.0
            or abs(target_degrees) >= 180.0
        ):
            raise ApiError(
                'Manual turn must be finite, nonzero, and strictly below 180°',
                400,
            )
        with self._lock:
            checkpoint_pending = self._checkpoint is not None
            workflow_recovery_pending = (
                self._workflow_recovery is not None
            )
            if checkpoint_pending:
                allowed = [
                    float(value)
                    for value in self._config['turn_control'][
                        'manual_steps_deg'
                    ]
                ]
                if not any(
                    math.isclose(
                        abs(target_degrees),
                        step,
                        rel_tol=0.0,
                        abs_tol=1e-9,
                    )
                    for step in allowed
                ):
                    raise ApiError(
                        'Only configured fine-turn steps are allowed while a '
                        'route checkpoint is pending',
                        409,
                    )
            self._require_runtime_ready(
                'turn_ready', 'turn_block_reasons', 'Manual turn'
            )
            return self._start_worker(
                kind='manual_turn',
                name=f'{target_degrees:+g} deg',
                target=lambda stop: self._run_exact_turn(
                    target_degrees, stop
                ),
                allow_checkpoint=checkpoint_pending,
                allow_workflow_recovery=(
                    checkpoint_pending and workflow_recovery_pending
                ),
            )

    def start_marker_test(self, marker_name: str) -> dict[str, Any]:
        """Run one configured marker approach without a route or turn."""
        with self._lock:
            if marker_name not in self._config['markers']:
                raise ApiError(f'Unknown marker: {marker_name}', 404)
            self._require_runtime_ready(
                'drive_ready', 'motion_block_reasons', 'Marker test'
            )
            route_for_marker = next(
                (
                    route
                    for route in self._config['route'].values()
                    if marker_name == route['destination_marker']
                ),
                None,
            )
            if route_for_marker is None:
                raise ApiError(
                    f'Marker {marker_name} is not assigned to a route', 409
                )
            motion_sign = float(route_for_marker['linear_direction'])

            def run(stop_event: threading.Event) -> None:
                self._motion_preflight(stop_event)
                self._set_phase(f'test_marker_{marker_name}')
                self._run_visual_approach(
                    marker_name=marker_name,
                    motion_sign=motion_sign,
                    stop_event=stop_event,
                )

            return self._start_worker(
                kind='marker_test',
                name=f'marker {marker_name}',
                target=run,
            )

    def continue_route(self) -> dict[str, Any]:
        with self._lock:
            checkpoint = deepcopy(self._checkpoint)
            if checkpoint is None:
                raise ApiError(
                    'There is no route checkpoint to continue', 409
                )
            if checkpoint.get('phase') != 'destination_alignment':
                raise ApiError(
                    'This checkpoint cannot be continued safely', 409
                )
            self._require_runtime_ready(
                'drive_ready', 'motion_block_reasons', 'Route continuation'
            )
            route_key = str(checkpoint['route'])
            return self._start_worker(
                kind='route_continue',
                name=f'{route_key} destination alignment',
                target=lambda stop: self._continue_destination(
                    route_key, checkpoint, stop
                ),
                allow_checkpoint=True,
            )

    def _run_route(
        self,
        route_key: str,
        stop_event: threading.Event,
    ) -> None:
        route = deepcopy(self._config['route'][route_key])
        with self._lock:
            self._route_direction = route_key
            self._checkpoint = None
            # A workflow can run both directions in the same worker.  Clear
            # any manual-turn telemetry before starting a straight segment.
            self._clear_turn_metrics_locked()
        self._set_phase('route_preflight')
        self._motion_preflight(stop_event)

        self._set_phase(f"verify_{route['start_marker']}")
        self._verify_marker(str(route['start_marker']), stop_event)

        checkpoint = {
            'route': route_key,
            'phase': 'destination_alignment',
            'marker': str(route['destination_marker']),
            'motion_sign': float(route['linear_direction']),
        }
        with self._lock:
            self._checkpoint = checkpoint
        self._set_phase(f"align_{route['destination_marker']}")
        self._run_visual_approach(
            marker_name=str(route['destination_marker']),
            motion_sign=float(route['linear_direction']),
            stop_event=stop_event,
        )
        with self._lock:
            self._checkpoint = None

    def _continue_destination(
        self,
        route_key: str,
        checkpoint: dict[str, Any],
        stop_event: threading.Event,
    ) -> None:
        with self._lock:
            self._route_direction = route_key
        self._motion_preflight(stop_event)
        marker_name = str(checkpoint['marker'])
        self._set_phase(f'continue_align_{marker_name}')
        self._run_visual_approach(
            marker_name=marker_name,
            motion_sign=float(checkpoint['motion_sign']),
            stop_event=stop_event,
        )
        with self._lock:
            self._checkpoint = None

    def _verify_marker(
        self,
        marker_name: str,
        stop_event: threading.Event,
    ) -> None:
        """Record the start marker when available, but never block departure."""
        marker = self._config['markers'][marker_name]
        camera = str(marker['camera'])
        marker_id = int(marker['id'])
        self._set_command(ZERO_COMMAND)
        self._check_stop(stop_event, require_lease=True)
        with self._lock:
            observation = self._observations.get((camera, marker_id))
        fresh = (
            observation is not None
            and _now() - float(observation.timestamp_sec)
            <= float(self._config['timeouts']['marker_observation_stale_sec'])
        )
        if fresh:
            self._set_marker_metrics(marker_name, observation)
            self._record_event(
                'info',
                f'Start marker {marker_name} (ID {marker_id}, {camera}) seen; '
                'departure continues without a pose-tolerance gate',
            )
            return
        self._record_event(
            'warning',
            f'Start marker {marker_name} (ID {marker_id}, {camera}) is not '
            'currently visible; demo mode continues straight toward the '
            'destination marker',
        )

    def _run_visual_approach(
        self,
        *,
        marker_name: str,
        motion_sign: float,
        stop_event: threading.Event,
    ) -> None:
        marker = self._config['markers'][marker_name]
        motion = self._config['motion_control']
        timeout = self._config['timeouts']
        camera = str(marker['camera'])
        camera_cfg = self._config['cameras'][camera]
        marker_id = int(marker['id'])
        controller = VisualApproachController(VisualApproachConfig(
            target_marker_id=marker_id,
            camera_name=camera,
            motion_sign=motion_sign,
            target_distance_m=float(marker['target_distance_m']),
            target_lateral_m=float(marker['target_lateral_m']),
            target_yaw_rad=math.radians(float(marker['target_yaw_deg'])),
            distance_tolerance_m=float(
                marker.get(
                    'distance_tolerance_m',
                    motion['distance_tolerance_m'],
                )
            ),
            lateral_tolerance_m=float(
                marker.get(
                    'lateral_tolerance_m',
                    motion['lateral_tolerance_m'],
                )
            ),
            yaw_tolerance_rad=math.radians(float(
                marker.get('yaw_tolerance_deg', motion['yaw_tolerance_deg'])
            )),
            linear_kp=float(motion['linear_kp']),
            lateral_kp=float(motion['lateral_kp']),
            yaw_kp=float(motion['yaw_kp']),
            angular_command_sign=float(camera_cfg['steering_sign']),
            max_linear_mps=float(
                marker.get('max_linear_mps', motion['max_linear_mps'])
            ),
            max_angular_rps=float(
                marker.get('max_angular_rps', motion['max_angular_rps'])
            ),
            linear_gate_angle_rad=math.radians(float(
                motion['linear_gate_angle_deg']
            )),
            min_steering_rps=float(motion['min_steering_rps']),
            steering_output_scale=float(
                motion.get('steering_output_scale', 0.50)
            ),
            steering_slow_band_ratio=float(
                motion.get('steering_slow_band_ratio', 2.0)
            ),
            alignment_hysteresis_ratio=float(
                motion.get('alignment_hysteresis_ratio', 1.5)
            ),
            distance_only_completion=(
                str(marker.get(
                    'completion_mode',
                    'full_pose',
                )) == 'distance_only'
            ),
            stable_detections=int(motion['stable_detections']),
            hold_time_sec=float(
                marker.get('hold_time_sec', motion['hold_time_sec'])
            ),
            acquire_timeout_sec=float(
                timeout.get('acquire_creep_sec', 60.0)
            ),
            marker_loss_timeout_sec=float(timeout['marker_loss_sec']),
            observation_stale_sec=float(
                timeout['marker_observation_stale_sec']
            ),
        ))
        with self._lock:
            self._metrics['active_camera'] = camera
            self._metrics['target_marker_id'] = marker_id
            self._clear_detection_metrics_locked()
        started = _now()
        deadline = started + float(timeout['straight_segment_sec'])
        alignment_deadline: float | None = None
        controller.start(started)
        period = 1.0 / float(motion['control_rate_hz'])
        acquire_creep_mps = float(motion.get(
            'acquire_creep_mps',
            min(0.03, float(motion['max_linear_mps'])),
        ))
        while True:
            self._check_stop(stop_event, require_lease=True)
            now = _now()
            if now >= deadline:
                self._set_command(ZERO_COMMAND)
                raise RecoverableOperationError(
                    f'Straight segment to {marker_name} timed out'
                )
            with self._lock:
                observation = self._observations.get((camera, marker_id))
                frame_at = self._frame_at.get(camera)
                camera_info_at = self._camera_info_at.get(camera)
            camera_fresh = (
                frame_at is not None
                and now - float(frame_at)
                <= float(timeout['camera_stale_sec'])
            )
            if camera_cfg.get('use_camera_info', False):
                camera_fresh = (
                    camera_fresh
                    and camera_info_at is not None
                    and now - float(camera_info_at)
                    <= float(timeout['camera_info_sec'])
                )
            result = controller.update(observation, now)
            if controller.acquired and alignment_deadline is None:
                alignment_deadline = now + float(timeout['alignment_sec'])
            if (
                alignment_deadline is not None
                and now >= alignment_deadline
                and not result.complete
            ):
                self._force_zero()
                raise RecoverableOperationError(
                    f'Alignment to {marker_name} timed out'
                )
            command, control_reason = self._visual_approach_command(
                result=result,
                acquired=controller.acquired,
                camera_fresh=camera_fresh,
                motion_sign=motion_sign,
                acquire_creep_mps=acquire_creep_mps,
            )
            self._set_command(command)
            if observation is not None:
                self._set_marker_metrics(marker_name, observation)
            with self._lock:
                self._metrics['control_reason'] = control_reason
                self._metrics['distance_error_m'] = result.distance_error_m
                self._metrics['lateral_error_m'] = result.lateral_error_m
                self._metrics['yaw_error_deg'] = (
                    None
                    if result.yaw_error_rad is None
                    else math.degrees(result.yaw_error_rad)
                )
            if result.complete:
                self._force_zero()
                return
            if result.failed:
                self._force_zero()
                raise RecoverableOperationError(
                    f'Marker alignment failed for {marker_name}: '
                    f'{result.reason}'
                )
            stop_event.wait(period)

    @staticmethod
    def _visual_approach_command(
        *,
        result: Any,
        acquired: bool,
        camera_fresh: bool,
        motion_sign: float,
        acquire_creep_mps: float,
    ) -> tuple[VelocityCommand, str]:
        """Select a bounded rail-search command before marker acquisition."""
        if not camera_fresh:
            return ZERO_COMMAND, 'camera_not_fresh'
        if (
            not acquired
            and result.reason in {
                'target_marker_not_visible',
                'stabilizing_target_marker',
                'waiting_for_new_marker_frame',
            }
            and not result.complete
            and not result.failed
        ):
            # The route is a known straight A-C rail.  Creep along that rail
            # until the exact configured camera/ID becomes visible, then
            # hand control back to marker alignment.  Wrong IDs and the
            # opposite camera never influence steering.
            return (
                VelocityCommand(
                    linear_x=motion_sign * acquire_creep_mps,
                ),
                f'acquire_creep:{result.reason}',
            )
        return result.command, str(result.reason)

    def _set_marker_metrics(
        self,
        marker_name: str,
        observation: Any,
    ) -> None:
        marker_id = int(self._config['markers'][marker_name]['id'])
        with self._lock:
            self._metrics.update({
                'active_camera': str(observation.camera_name),
                'target_marker_id': marker_id,
                'detected_marker_id': int(observation.marker_id),
                'distance_m': float(observation.distance_m),
                'lateral_m': float(observation.lateral_m),
                'marker_yaw_deg': math.degrees(float(observation.yaw_rad)),
            })

    def _clear_detection_metrics_locked(self) -> None:
        """Clear only transient marker data while preserving the target."""
        for key in (
            'detected_marker_id',
            'distance_m',
            'lateral_m',
            'yaw_error_deg',
            'marker_yaw_deg',
            'distance_error_m',
            'lateral_error_m',
        ):
            self._metrics[key] = None

    def _clear_turn_metrics_locked(self) -> None:
        """Clear a completed turn before another operation or route starts."""
        self._metrics['turn_progress_deg'] = None
        self._metrics['turn_target_deg'] = None

    def _run_exact_turn(
        self,
        target_degrees: float,
        stop_event: threading.Event,
        *,
        phase_name: str = 'exact_turn',
    ) -> None:
        # Publish the new target before preflight.  Otherwise the UI can show
        # the previous route's completed turn throughout preflight.
        self._set_phase(
            phase_name,
            turn_progress_deg=0.0,
            turn_target_deg=target_degrees,
        )
        self._turn_preflight(stop_event)
        turn = self._config['turn_control']
        timeout = self._config['timeouts']
        with self._lock:
            yaw = self._yaw_rad
            odom_at = self._odom_at
        if yaw is None or odom_at is None:
            raise RuntimeError('Odometry is unavailable for exact turn')
        controller = ExactAngleTurnController(TurnControlConfig(
            kp=float(turn['kp']),
            max_angular_rps=float(turn['max_angular_rps']),
            min_angular_rps=float(turn['min_angular_rps']),
            tolerance_deg=float(turn['tolerance_deg']),
            settle_time_sec=float(turn['settle_time_sec']),
            timeout_sec=float(timeout['turn_sec']),
            odom_stale_sec=float(timeout['odom_stale_sec']),
        ))
        controller.start(yaw, target_degrees, _now())
        period = 1.0 / float(turn['control_rate_hz'])
        while True:
            self._check_stop(stop_event, require_lease=True)
            now = _now()
            with self._lock:
                yaw = self._yaw_rad
                odom_at = self._odom_at
            if yaw is None:
                raise RuntimeError('Odometry disappeared during exact turn')
            result = controller.update(
                yaw,
                now,
                odom_timestamp_sec=odom_at,
                marker_visible=False,
            )
            self._set_command(result.command)
            with self._lock:
                self._metrics['control_reason'] = result.reason
                self._metrics['turn_progress_deg'] = (
                    None
                    if result.progress_rad is None
                    else math.degrees(result.progress_rad)
                )
                self._metrics['turn_target_deg'] = target_degrees
            if result.complete:
                self._force_zero()
                return
            if result.failed:
                self._force_zero()
                raise RuntimeError(f'Exact turn failed: {result.reason}')
            stop_event.wait(period)

    # ------------------------------------------------------------------
    # Saved poses and workflows

    def start_pose(self, pose_id: int) -> dict[str, Any]:
        canonical = canonical_entity_id(pose_id, field_name='pose_id')
        with self._lock:
            pose = self._config.get('poses', {}).get(canonical)
            if pose is None:
                raise ApiError(f'Pose {pose_id} does not exist', 404)
            self._require_pose_ready(pose)
            return self._start_worker(
                kind='pose',
                name=f"{pose_id}: {pose['name']}",
                target=lambda stop: self._execute_pose(canonical, stop),
            )

    def start_pose_preview(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            pose = self._pose_from_api(payload)
            candidate = deepcopy(self._config)
            temporary_id = int(candidate['next_pose_id'])
            candidate['poses'][str(temporary_id)] = pose
            candidate['next_pose_id'] = temporary_id + 1
            try:
                validate_document(candidate)
            except (ConfigStoreError, WorkflowCoreError, ValueError) as exc:
                raise ApiError(str(exc), 400) from exc
            self._require_pose_ready(pose)
            return self._start_worker(
                kind='pose_preview',
                name=str(pose['name'] or 'unsaved pose'),
                target=lambda stop: self._execute_pose_document(pose, stop),
            )

    def _execute_pose(
        self,
        pose_id: str,
        stop_event: threading.Event,
    ) -> None:
        pose = deepcopy(self._config['poses'].get(pose_id))
        if pose is None:
            raise RuntimeError(f'Pose {pose_id} disappeared')
        self._set_phase(f'pose_{pose_id}')
        self._execute_pose_document(pose, stop_event)

    def _execute_pose_document(
        self,
        pose: dict[str, Any],
        stop_event: threading.Event,
    ) -> None:
        self._verify_arm_hardware(stop_event)
        command = self._pose_command_for_executor(pose)
        if (
            command['arm_enabled']
            and command['arm_duration_sec']
            > float(self._config['arm']['axis4_duration_warning_sec'])
        ):
            self._record_event(
                'warning',
                'arm_joint_4 firmware clamps long profiles near '
                f"{self._config['arm']['axis4_duration_warning_sec']} sec",
            )
        margin = max(
            float(self._config['arm']['result_timeout_margin_sec']),
            float(self._config['gripper']['result_timeout_margin_sec']),
        )
        try:
            result = self._pose_executor.execute(
                command,
                stop_event=stop_event,
                server_timeout_sec=float(
                    self._config['timeouts']['stop_sec']
                ),
                result_timeout_margin_sec=margin,
            )
        except PoseCancellationUnconfirmed as exc:
            raise RuntimeError(str(exc)) from exc
        except PoseExecutionError as exc:
            if stop_event.is_set():
                raise OperationStopped('Pose execution was stopped') from exc
            raise RuntimeError(str(exc)) from exc
        if not result.success:
            raise RuntimeError(result.message)

    def _pose_command_for_executor(
        self,
        pose: dict[str, Any],
    ) -> dict[str, Any]:
        arm = pose['arm']
        gripper = pose['gripper']
        return {
            'arm_enabled': bool(arm['enabled']),
            'arm_positions_deg': [
                float(arm['positions_deg'][name])
                for name in self._config['arm']['joint_names']
            ],
            'arm_duration_sec': float(arm['duration_sec']),
            'gripper_enabled': bool(gripper['enabled']),
            'gripper_positions_deg': [
                float(gripper['positions_deg'][name])
                for name in self._config['gripper']['joint_names']
            ],
            'gripper_duration_sec': float(gripper['duration_sec']),
            'target_load_raw': int(gripper['target_load_raw']),
            'dwell_sec': float(pose['dwell_sec']),
        }

    def start_workflow(self, workflow_id: int) -> dict[str, Any]:
        canonical = canonical_entity_id(workflow_id, field_name='workflow_id')
        with self._lock:
            workflow = self._config.get('workflows', {}).get(canonical)
            if workflow is None:
                raise ApiError(f'Workflow {workflow_id} does not exist', 404)
            for step in workflow.get('steps', []):
                step_type = str(step.get('type', ''))
                if step_type in {'GO_DROPOFF', 'GO_PICKUP'}:
                    self._require_runtime_ready(
                        'drive_ready',
                        'motion_block_reasons',
                        'Workflow route motion',
                    )
                elif step_type == 'POSE':
                    pose_id = canonical_entity_id(
                        step.get('pose_id'), field_name='pose_id'
                    )
                    pose = self._config['poses'].get(pose_id)
                    if pose is None:
                        raise ApiError(
                            'Workflow references missing pose '
                            f"{step.get('pose_id')}",
                            409,
                        )
                    self._require_pose_ready(pose)
            return self._start_worker(
                kind='workflow',
                name=f"{workflow_id}: {workflow['name']}",
                target=lambda stop: self._run_workflow(canonical, stop),
            )

    def _workflow_step_label(self, step: dict[str, Any]) -> str:
        step_type = str(step['type'])
        if step_type == 'POSE':
            pose_id = canonical_entity_id(
                step['pose_id'], field_name='pose_id'
            )
            pose = self._config.get('poses', {}).get(pose_id, {})
            pose_name = str(pose.get('name', '')).strip()
            return (
                f'동작 {pose_id}: {pose_name}'
                if pose_name
                else f'동작 {pose_id}'
            )
        if step_type == 'GO_DROPOFF':
            return '놓기 위치로 이동'
        if step_type == 'WAIT_SECONDS':
            return f"{float(step['seconds']):g}초 대기"
        if step_type == 'WAIT_RETURN_CONFIRM':
            return '기존 복귀 확인 단계 · 자동 통과'
        if step_type == 'GO_PICKUP':
            return '집기 위치로 복귀'
        return step_type

    def _require_workflow_step_ready(
        self,
        step: dict[str, Any],
    ) -> None:
        step_type = str(step.get('type', ''))
        if step_type in {'GO_DROPOFF', 'GO_PICKUP'}:
            self._require_runtime_ready(
                'drive_ready',
                'motion_block_reasons',
                'Workflow route motion',
            )
        elif step_type == 'POSE':
            pose_id = canonical_entity_id(
                step.get('pose_id'), field_name='pose_id'
            )
            pose = self._config.get('poses', {}).get(pose_id)
            if pose is None:
                raise ApiError(
                    f"Workflow references missing pose {step.get('pose_id')}",
                    409,
                )
            self._require_pose_ready(pose)

    def _set_workflow_step_progress(
        self,
        *,
        workflow_id: str,
        workflow_name: str,
        step_index: int,
        total_steps: int,
        step: dict[str, Any],
    ) -> None:
        with self._lock:
            progress = deepcopy(self._workflow_progress)
            if (
                progress is None
                or str(progress.get('workflow_id')) != workflow_id
            ):
                progress = {
                    'workflow_id': int(workflow_id),
                    'workflow_name': workflow_name,
                    'total_steps': total_steps,
                    'completed_steps': 0,
                    'completed_step_numbers': [],
                    'skipped_steps': [],
                }
            progress.update({
                'workflow_id': int(workflow_id),
                'workflow_name': workflow_name,
                'total_steps': total_steps,
                'current_step': step_index + 1,
                'current_step_index': step_index,
                'current_step_type': str(step['type']),
                'current_step_label': self._workflow_step_label(step),
                'status': 'running',
                'error': None,
            })
            self._workflow_progress = progress

    def _pause_workflow_step(
        self,
        *,
        workflow_id: str,
        workflow_name: str,
        step_index: int,
        total_steps: int,
        step: dict[str, Any],
        error: Exception,
    ) -> None:
        message = str(error) or type(error).__name__
        with self._lock:
            route_checkpoint = deepcopy(self._checkpoint)
            recovery = {
                'workflow_id': int(workflow_id),
                'workflow_name': workflow_name,
                'step_index': step_index,
                'step_number': step_index + 1,
                'total_steps': total_steps,
                'step_type': str(step['type']),
                'step_label': self._workflow_step_label(step),
                'error': message,
                'route_checkpoint': route_checkpoint,
                'can_resume_route_destination': bool(
                    route_checkpoint is not None
                    and str(step['type']) in {
                        'GO_DROPOFF', 'GO_PICKUP'
                    }
                    and route_checkpoint.get('phase')
                    == 'destination_alignment'
                ),
            }
            self._workflow_recovery = recovery
            if self._workflow_progress is None:
                self._workflow_progress = {}
            self._workflow_progress.update({
                'workflow_id': int(workflow_id),
                'workflow_name': workflow_name,
                'total_steps': total_steps,
                'current_step': step_index + 1,
                'current_step_index': step_index,
                'current_step_type': str(step['type']),
                'current_step_label': self._workflow_step_label(step),
                'status': 'paused',
                'error': message,
            })
            self._phase = (
                f'workflow_paused_{step_index + 1}_of_{total_steps}'
            )

    def _run_workflow_step(
        self,
        *,
        step: dict[str, Any],
        stop_event: threading.Event,
        resume_route_checkpoint: dict[str, Any] | None = None,
    ) -> None:
        step_type = str(step['type'])
        if step_type == 'POSE':
            self._execute_pose(
                canonical_entity_id(step['pose_id'], field_name='pose_id'),
                stop_event,
            )
        elif step_type == 'GO_DROPOFF':
            if resume_route_checkpoint is not None:
                self._continue_destination(
                    'outbound', resume_route_checkpoint, stop_event
                )
            else:
                self._run_route('outbound', stop_event)
        elif step_type == 'WAIT_SECONDS':
            seconds = float(step['seconds'])
            if stop_event.wait(seconds):
                raise OperationStopped('Workflow wait was stopped')
        elif step_type == 'WAIT_RETURN_CONFIRM':
            # Backward compatibility for workflows saved before return
            # confirmation was removed.  Do not pause or require UI input.
            self._check_stop(stop_event)
        elif step_type == 'GO_PICKUP':
            if resume_route_checkpoint is not None:
                self._continue_destination(
                    'return', resume_route_checkpoint, stop_event
                )
            else:
                self._run_route('return', stop_event)
        else:
            raise RuntimeError(f'Unsupported workflow step: {step_type}')

    def _run_workflow(
        self,
        workflow_id: str,
        stop_event: threading.Event,
        *,
        start_index: int = 0,
        resume_route_checkpoint: dict[str, Any] | None = None,
    ) -> None:
        workflow = deepcopy(self._config['workflows'].get(workflow_id))
        if workflow is None:
            raise RuntimeError(f'Workflow {workflow_id} disappeared')
        steps = workflow['steps']
        total_steps = len(steps)
        workflow_name = str(workflow['name'])
        if start_index < 0 or start_index > total_steps:
            raise RuntimeError('Workflow resume index is out of range')
        if start_index == 0:
            with self._lock:
                self._workflow_progress = {
                    'workflow_id': int(workflow_id),
                    'workflow_name': workflow_name,
                    'total_steps': total_steps,
                    'completed_steps': 0,
                    'completed_step_numbers': [],
                    'skipped_steps': [],
                    'status': 'starting',
                    'error': None,
                }
                self._workflow_recovery = None
        for index in range(start_index, total_steps):
            step = steps[index]
            self._check_stop(stop_event)
            step_type = str(step['type'])
            self._set_workflow_step_progress(
                workflow_id=workflow_id,
                workflow_name=workflow_name,
                step_index=index,
                total_steps=total_steps,
                step=step,
            )
            self._set_phase(
                f'workflow_{index + 1}_of_{total_steps}_{step_type.lower()}'
            )
            self._record_event(
                'info',
                f'Workflow {workflow_id} step {index + 1}/{total_steps}: '
                f'{step_type}',
            )
            try:
                checkpoint = (
                    resume_route_checkpoint
                    if index == start_index
                    else None
                )
                self._run_workflow_step(
                    step=step,
                    stop_event=stop_event,
                    resume_route_checkpoint=checkpoint,
                )
            except OperationStopped:
                with self._lock:
                    if self._workflow_progress is not None:
                        self._workflow_progress['status'] = 'stopped'
                raise
            except Exception as exc:
                with self._lock:
                    safety_reason = self._safety_abort_reason
                executor_drained = bool(getattr(
                    getattr(self, '_pose_executor', None),
                    'shutdown_drained',
                    True,
                ))
                if safety_reason is not None or not executor_drained:
                    raise
                if self._shutdown_event.is_set() or stop_event.is_set():
                    raise OperationStopped(
                        'Workflow was stopped by user'
                    ) from exc
                self._pause_workflow_step(
                    workflow_id=workflow_id,
                    workflow_name=workflow_name,
                    step_index=index,
                    total_steps=total_steps,
                    step=step,
                    error=exc,
                )
                raise RecoverableOperationError(
                    f'Workflow {workflow_id} paused at step '
                    f'{index + 1}/{total_steps}: {exc}'
                ) from exc
            with self._lock:
                if self._workflow_progress is not None:
                    completed = list(
                        self._workflow_progress.get(
                            'completed_step_numbers', []
                        )
                    )
                    if not completed:
                        completed_count = int(
                            self._workflow_progress.get(
                                'completed_steps', 0
                            )
                        )
                        completed = list(range(1, completed_count + 1))
                    step_number = index + 1
                    if step_number not in completed:
                        completed.append(step_number)
                    completed = sorted(set(completed))
                    skipped = [
                        number for number in self._workflow_progress.get(
                            'skipped_steps', []
                        )
                        if int(number) != step_number
                    ]
                    self._workflow_progress['completed_step_numbers'] = (
                        completed
                    )
                    self._workflow_progress['completed_steps'] = len(completed)
                    self._workflow_progress['skipped_steps'] = skipped
                    self._workflow_progress['error'] = None
            resume_route_checkpoint = None
        with self._lock:
            skipped_steps = (
                []
                if self._workflow_progress is None
                else list(self._workflow_progress.get('skipped_steps', []))
            )
            if self._workflow_progress is not None:
                self._workflow_progress['status'] = (
                    'completed_with_skips'
                    if skipped_steps
                    else 'completed'
                )
                self._workflow_progress['error'] = None
            self._workflow_recovery = None

    def _resume_workflow(
        self,
        *,
        skip_failed_step: bool,
        selected_step_number: int | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError(
                    'Cannot resume while another operation is active', 409
                )
            recovery = deepcopy(self._workflow_recovery)
            if recovery is None:
                raise ApiError('There is no paused workflow step', 409)
            workflow_id = str(recovery['workflow_id'])
            workflow = self._config.get('workflows', {}).get(workflow_id)
            if workflow is None:
                raise ApiError(
                    f'Workflow {workflow_id} no longer exists', 409
                )
            failed_index = int(recovery['step_index'])
            steps = workflow['steps']
            if selected_step_number is not None:
                if isinstance(selected_step_number, bool):
                    raise ApiError('step_number must be an integer', 400)
                try:
                    selected_value = float(selected_step_number)
                except (TypeError, ValueError) as exc:
                    raise ApiError(
                        'step_number must be an integer', 400
                    ) from exc
                if (
                    not math.isfinite(selected_value)
                    or not selected_value.is_integer()
                ):
                    raise ApiError('step_number must be an integer', 400)
                selected_step_number = int(selected_value)
                if not 1 <= selected_step_number <= len(steps):
                    raise ApiError(
                        f'step_number must be between 1 and {len(steps)}',
                        400,
                    )
                start_index = selected_step_number - 1
            else:
                start_index = failed_index + (1 if skip_failed_step else 0)
            if start_index > len(steps):
                raise ApiError('Paused workflow step is invalid', 409)
            if start_index < len(steps):
                self._require_workflow_step_ready(steps[start_index])

            previous_progress = deepcopy(self._workflow_progress)
            previous_checkpoint = deepcopy(self._checkpoint)
            if self._workflow_progress is None:
                self._workflow_progress = {
                    'workflow_id': int(workflow_id),
                    'workflow_name': str(workflow['name']),
                    'total_steps': len(steps),
                    'completed_steps': failed_index,
                    'completed_step_numbers': list(
                        range(1, failed_index + 1)
                    ),
                    'skipped_steps': [],
                }
            completed = list(
                self._workflow_progress.get('completed_step_numbers', [])
            )
            if not completed:
                completed_count = int(
                    self._workflow_progress.get('completed_steps', 0)
                )
                completed = list(range(1, completed_count + 1))
            skipped = list(
                self._workflow_progress.get('skipped_steps', [])
            )
            if selected_step_number is not None:
                if start_index <= failed_index:
                    completed = [
                        number for number in completed
                        if int(number) < selected_step_number
                    ]
                    skipped = [
                        number for number in skipped
                        if int(number) < selected_step_number
                    ]
                else:
                    for step_number in range(
                        failed_index + 1, start_index + 1
                    ):
                        if step_number not in skipped:
                            skipped.append(step_number)
                self._checkpoint = None
            elif skip_failed_step:
                step_number = failed_index + 1
                if step_number not in skipped:
                    skipped.append(step_number)
                self._checkpoint = None
            self._workflow_progress['completed_step_numbers'] = sorted(
                {int(number) for number in completed}
            )
            self._workflow_progress['completed_steps'] = len(
                self._workflow_progress['completed_step_numbers']
            )
            self._workflow_progress['skipped_steps'] = sorted(
                {int(number) for number in skipped}
            )

            route_checkpoint = (
                deepcopy(recovery.get('route_checkpoint'))
                if (
                    not skip_failed_step
                    and start_index == failed_index
                )
                else None
            )
            self._workflow_recovery = None
            try:
                result = self._start_worker(
                    kind='workflow',
                    name=f"{workflow_id}: {workflow['name']}",
                    target=lambda stop: self._run_workflow(
                        workflow_id,
                        stop,
                        start_index=start_index,
                        resume_route_checkpoint=route_checkpoint,
                    ),
                    allow_checkpoint=route_checkpoint is not None,
                    allow_workflow_recovery=True,
                )
            except Exception:
                self._workflow_recovery = recovery
                self._workflow_progress = previous_progress
                self._checkpoint = previous_checkpoint
                raise
        if selected_step_number is not None:
            event_message = (
                f'Starting workflow {workflow_id} at selected step '
                f'{selected_step_number}'
            )
        elif skip_failed_step:
            event_message = (
                f'Skipped workflow {workflow_id} step {failed_index + 1}'
            )
        else:
            event_message = (
                f'Retrying workflow {workflow_id} step {failed_index + 1}'
            )
        self._record_event(
            (
                'warning'
                if skip_failed_step or selected_step_number is not None
                else 'info'
            ),
            event_message,
        )
        return result

    def retry_workflow_step(self) -> dict[str, Any]:
        return self._resume_workflow(skip_failed_step=False)

    def skip_workflow_step(self) -> dict[str, Any]:
        return self._resume_workflow(skip_failed_step=True)

    def resume_workflow_at_step(
        self,
        step_number: int,
    ) -> dict[str, Any]:
        return self._resume_workflow(
            skip_failed_step=False,
            selected_step_number=step_number,
        )

    def abort_paused_workflow(self) -> dict[str, Any]:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError(
                    'Cannot end a workflow while an operation is active', 409
                )
            recovery = deepcopy(self._workflow_recovery)
            if recovery is None:
                raise ApiError('There is no paused workflow step', 409)
            self._workflow_recovery = None
            self._checkpoint = None
            if self._workflow_progress is not None:
                self._workflow_progress['status'] = 'aborted'
                self._workflow_progress['error'] = None
            self._state = 'IDLE'
            self._phase = 'workflow_aborted'
            self._error = None
        self._force_zero()
        self._record_event(
            'warning',
            f"Ended paused workflow {recovery['workflow_id']}",
        )
        return {'aborted': recovery}

    def confirm_return(self) -> dict[str, Any]:
        with self._lock:
            if self._state != 'WAITING_RETURN_CONFIRM':
                raise ApiError(
                    'No workflow is waiting for return confirmation',
                    409,
                )
            self._operator_lease_at = _now()
            self._return_confirm_event.set()
        return {'confirmed': True}

    # ------------------------------------------------------------------
    # Revisioned category/pose/workflow API

    def _ensure_editable(self) -> None:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError('Configuration cannot change during a run', 409)
            if self._workflow_recovery is not None:
                raise ApiError(
                    'Configuration cannot change while a workflow step is '
                    'paused',
                    409,
                )

    @staticmethod
    def _expected_revision(
        payload: dict[str, Any] | None,
        current: int,
    ) -> int:
        if payload is None or 'expected_revision' not in payload:
            raise ApiError(
                f'expected_revision is required (current revision {current})',
                400,
            )
        value = payload['expected_revision']
        if isinstance(value, bool):
            raise ApiError('expected_revision must be an integer', 400)
        try:
            result = int(value)
        except (TypeError, ValueError) as exc:
            raise ApiError('expected_revision must be an integer', 400) from exc
        return result

    def _commit(
        self,
        payload: dict[str, Any] | None,
        callback: Callable[[dict[str, Any]], Any],
    ) -> dict[str, Any]:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError(
                    'Configuration cannot change during a run', 409
                )
            if self._workflow_recovery is not None:
                raise ApiError(
                    'Configuration cannot change while a workflow step is '
                    'paused',
                    409,
                )
            current_revision = int(self._config['revision'])
            expected = self._expected_revision(payload, current_revision)
            try:
                document = self._store.mutate(expected, callback)
            except ConfigConflictError as exc:
                raise ApiError(str(exc), 409) from exc
            except (ConfigStoreError, WorkflowCoreError, ValueError) as exc:
                raise ApiError(str(exc), 400) from exc
            self._config = document
            self._config_error = None
            self._tracker = ArucoTracker.from_config(document)
        self._publish_watchdog_config()
        return document

    @staticmethod
    def _category_to_api(category_id: str, value: dict[str, Any]) -> dict[str, Any]:
        return {'id': int(category_id), **deepcopy(value)}

    def list_categories(self) -> dict[str, Any]:
        with self._lock:
            values = [
                self._category_to_api(key, value)
                for key, value in sorted(
                    self._config['categories'].items(),
                    key=lambda item: (item[1]['order'], int(item[0])),
                )
            ]
        return {'categories': values}

    def create_category(self, payload: dict[str, Any]) -> dict[str, Any]:
        name = payload.get('name')
        requested_id = payload.get('id')
        created: dict[str, Any] = {}

        def mutate(document: dict[str, Any]) -> None:
            category_id = (
                int(document['next_category_id'])
                if requested_id is None
                else int(canonical_entity_id(
                    requested_id, field_name='category_id'
                ))
            )
            value = add_category(document, category_id, name)
            created.update(
                self._category_to_api(str(category_id), value)
            )

        document = self._commit(payload, mutate)
        return {'category': created, 'revision': document['revision']}

    def update_category(
        self,
        category_id: int,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        if 'id' in payload and int(payload['id']) != int(category_id):
            raise ApiError('Category IDs cannot be changed', 400)
        updated: dict[str, Any] = {}

        def mutate(document: dict[str, Any]) -> None:
            value = rename_category(document, category_id, payload.get('name'))
            updated.update(
                self._category_to_api(str(category_id), value)
            )

        document = self._commit(payload, mutate)
        return {'category': updated, 'revision': document['revision']}

    def delete_category(
        self,
        category_id: int,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        moved: dict[str, Any] = {}

        def mutate(document: dict[str, Any]) -> None:
            moved.update(core_delete_category(document, category_id))

        document = self._commit(payload, mutate)
        return {'deleted_id': category_id, **moved, 'revision': document['revision']}

    @staticmethod
    def _pose_to_api(pose_id: int, pose: dict[str, Any]) -> dict[str, Any]:
        arm = pose['arm']
        gripper = pose['gripper']
        return {
            'id': int(pose_id),
            'name': pose['name'],
            'category_id': int(pose['category_id']),
            'arm_enabled': bool(arm['enabled']),
            'arm_positions_deg': [
                float(arm['positions_deg'][name])
                for name in ARM_JOINT_NAMES
            ],
            'arm_duration_sec': float(arm['duration_sec']),
            'gripper_enabled': bool(gripper['enabled']),
            'gripper_positions_deg': [
                float(gripper['positions_deg'][name])
                for name in GRIPPER_JOINT_NAMES
            ],
            'gripper_duration_sec': float(gripper['duration_sec']),
            'target_load_raw': int(gripper['target_load_raw']),
            'dwell_sec': float(pose['dwell_sec']),
        }

    def _pose_from_api(
        self,
        payload: dict[str, Any],
        existing: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if existing is None:
            arm_default = {
                name: 0.0 for name in self._config['arm']['joint_names']
            }
            gripper_default = {
                name: 0.0 for name in self._config['gripper']['joint_names']
            }
            base = {
                'name': '',
                'category_id': int(
                    self._config['uncategorized_category_id']
                ),
                'arm': {
                    'enabled': True,
                    'positions_deg': arm_default,
                    'duration_sec': float(
                        self._config['arm']['default_duration_sec']
                    ),
                },
                'gripper': {
                    'enabled': False,
                    'positions_deg': gripper_default,
                    'duration_sec': float(
                        self._config['gripper']['default_duration_sec']
                    ),
                    'target_load_raw': int(
                        self._config['gripper']['target_load_raw']['default']
                    ),
                },
                'dwell_sec': 0.0,
            }
        else:
            base = deepcopy(existing)

        if 'name' in payload:
            base['name'] = payload['name']
        if 'category_id' in payload:
            base['category_id'] = int(payload['category_id'])
        if 'arm_enabled' in payload:
            base['arm']['enabled'] = bool(payload['arm_enabled'])
        if 'arm_positions_deg' in payload:
            values = payload['arm_positions_deg']
            if not isinstance(values, list) or len(values) != len(ARM_JOINT_NAMES):
                raise ApiError('arm_positions_deg must contain 5 values', 400)
            base['arm']['positions_deg'] = {
                name: float(value)
                for name, value in zip(ARM_JOINT_NAMES, values)
            }
        if 'arm_duration_sec' in payload:
            base['arm']['duration_sec'] = float(payload['arm_duration_sec'])
        if 'gripper_enabled' in payload:
            base['gripper']['enabled'] = bool(payload['gripper_enabled'])
        if 'gripper_positions_deg' in payload:
            values = payload['gripper_positions_deg']
            if not isinstance(values, list) or len(values) != len(GRIPPER_JOINT_NAMES):
                raise ApiError(
                    'gripper_positions_deg must contain 9 values', 400
                )
            base['gripper']['positions_deg'] = {
                name: float(value)
                for name, value in zip(GRIPPER_JOINT_NAMES, values)
            }
        if 'gripper_duration_sec' in payload:
            base['gripper']['duration_sec'] = float(
                payload['gripper_duration_sec']
            )
        if 'target_load_raw' in payload:
            base['gripper']['target_load_raw'] = int(
                payload['target_load_raw']
            )
        if 'dwell_sec' in payload:
            base['dwell_sec'] = float(payload['dwell_sec'])
        return base

    def list_poses(self) -> dict[str, Any]:
        with self._lock:
            values = [
                self._pose_to_api(int(key), value)
                for key, value in sorted(
                    self._config['poses'].items(),
                    key=lambda item: int(item[0]),
                )
            ]
        return {'poses': values}

    def create_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        requested_id = payload.get('id')
        created_id = 0

        def mutate(document: dict[str, Any]) -> None:
            nonlocal created_id
            created_id = (
                int(document['next_pose_id'])
                if requested_id is None
                else int(canonical_entity_id(requested_id, field_name='pose_id'))
            )
            key = str(created_id)
            if key in document['poses']:
                raise WorkflowCoreError(f'pose {created_id} already exists')
            document['poses'][key] = self._pose_from_api(payload)
            if created_id >= int(document['next_pose_id']):
                document['next_pose_id'] = created_id + 1

        document = self._commit(payload, mutate)
        return {
            'pose': self._pose_to_api(
                created_id, document['poses'][str(created_id)]
            ),
            'revision': document['revision'],
        }

    def update_pose(
        self,
        pose_id: int,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        if 'id' in payload and int(payload['id']) != int(pose_id):
            raise ApiError('Pose IDs cannot be changed', 400)
        key = canonical_entity_id(pose_id, field_name='pose_id')

        def mutate(document: dict[str, Any]) -> None:
            current = document['poses'].get(key)
            if current is None:
                raise WorkflowCoreError(f'pose {pose_id} does not exist')
            document['poses'][key] = self._pose_from_api(payload, current)

        document = self._commit(payload, mutate)
        return {
            'pose': self._pose_to_api(pose_id, document['poses'][key]),
            'revision': document['revision'],
        }

    def delete_pose(
        self,
        pose_id: int,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        removed: dict[str, Any] = {}

        def mutate(document: dict[str, Any]) -> None:
            removed.update(core_delete_pose(document, pose_id))

        document = self._commit(payload, mutate)
        return {'deleted_id': pose_id, 'revision': document['revision']}

    def list_workflows(self) -> dict[str, Any]:
        with self._lock:
            values = [
                {'id': int(key), **deepcopy(value)}
                for key, value in sorted(
                    self._config['workflows'].items(),
                    key=lambda item: int(item[0]),
                )
            ]
        return {'workflows': values}

    def create_workflow(self, payload: dict[str, Any]) -> dict[str, Any]:
        requested_id = payload.get('id')
        created_id = 0

        def mutate(document: dict[str, Any]) -> None:
            nonlocal created_id
            created_id = (
                int(document['next_workflow_id'])
                if requested_id is None
                else int(canonical_entity_id(
                    requested_id, field_name='workflow_id'
                ))
            )
            key = str(created_id)
            if key in document['workflows']:
                raise WorkflowCoreError(
                    f'workflow {created_id} already exists'
                )
            document['workflows'][key] = {
                'name': payload.get('name'),
                'category_id': int(payload.get(
                    'category_id', document['uncategorized_category_id']
                )),
                'steps': deepcopy(payload.get('steps')),
            }
            if created_id >= int(document['next_workflow_id']):
                document['next_workflow_id'] = created_id + 1

        document = self._commit(payload, mutate)
        return {
            'workflow': {
                'id': created_id,
                **deepcopy(document['workflows'][str(created_id)]),
            },
            'revision': document['revision'],
        }

    def update_workflow(
        self,
        workflow_id: int,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        if 'id' in payload and int(payload['id']) != int(workflow_id):
            raise ApiError('Workflow IDs cannot be changed', 400)
        key = canonical_entity_id(workflow_id, field_name='workflow_id')

        def mutate(document: dict[str, Any]) -> None:
            current = document['workflows'].get(key)
            if current is None:
                raise WorkflowCoreError(
                    f'workflow {workflow_id} does not exist'
                )
            if 'name' in payload:
                current['name'] = payload['name']
            if 'category_id' in payload:
                current['category_id'] = int(payload['category_id'])
            if 'steps' in payload:
                current['steps'] = deepcopy(payload['steps'])

        document = self._commit(payload, mutate)
        return {
            'workflow': {'id': workflow_id, **deepcopy(document['workflows'][key])},
            'revision': document['revision'],
        }

    def delete_workflow(
        self,
        workflow_id: int,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        key = canonical_entity_id(workflow_id, field_name='workflow_id')

        def mutate(document: dict[str, Any]) -> None:
            if key not in document['workflows']:
                raise WorkflowCoreError(
                    f'workflow {workflow_id} does not exist'
                )
            del document['workflows'][key]

        document = self._commit(payload, mutate)
        return {'deleted_id': workflow_id, 'revision': document['revision']}

    def reload_config(self) -> dict[str, Any]:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ApiError(
                    'Configuration cannot change during a run', 409
                )
            if self._workflow_recovery is not None:
                raise ApiError(
                    'Configuration cannot reload while a workflow step is '
                    'paused',
                    409,
                )
            try:
                candidate = self._store.snapshot()
            except ConfigStoreError as exc:
                self._config_error = str(exc)
                self._error = f'Invalid configuration: {exc}'
                self._state = 'ERROR_LATCHED'
                self._force_zero()
                raise ApiError(str(exc), 400) from exc
            restart_fields = []
            if candidate['topics'] != self._config['topics']:
                restart_fields.append('topics')
            if any(
                candidate['web'][field] != self._config['web'][field]
                for field in ('host', 'port')
            ):
                restart_fields.append('web')
            self._config = candidate
            self._config_error = None
            self._restart_required = restart_fields
            self._tracker = ArucoTracker.from_config(candidate)
        self._publish_watchdog_config()
        return {
            'revision': candidate['revision'],
            'restart_required': restart_fields,
        }

    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        self._stop_event.set()
        self._return_confirm_event.set()
        if rclpy.ok():
            self._force_zero()
        self._pose_executor.destroy()
        if self._web_server is not None:
            self._web_server.shutdown()
        if self._web_thread is not None:
            self._web_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: PresentationNode | None = None
    executor = MultiThreadedExecutor(num_threads=4)
    previous_handlers: dict[int, Any] = {}

    def request_shutdown(_signum: int, _frame: Any) -> None:
        if node is not None:
            node._shutdown_event.set()
            node._stop_event.set()
            node._return_confirm_event.set()
            node._pose_executor.cancel_active(wait_timeout_sec=0.0)
            node._force_zero()
        raise KeyboardInterrupt

    try:
        node = PresentationNode()
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_shutdown)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if node is not None:
            node._shutdown_event.set()
            node._stop_event.set()
            node._return_confirm_event.set()
            node._pose_executor.cancel_active(wait_timeout_sec=0.0)
            if rclpy.ok():
                node._force_zero()
            with node._lock:
                shutdown_timeout = float(
                    node._config['timeouts'].get('stop_sec', 2.0)
                )
            deadline = _now() + shutdown_timeout
            while rclpy.ok() and _now() < deadline:
                with node._lock:
                    worker = node._worker
                    worker_alive = (
                        worker is not None and worker.is_alive()
                    )
                if (
                    not worker_alive
                    and node._pose_executor.shutdown_drained
                ):
                    break
                executor.spin_once(timeout_sec=0.02)
            with node._lock:
                worker = node._worker
            if worker is not None and worker.is_alive():
                worker.join(timeout=0.1)
            if not node._pose_executor.shutdown_drained:
                node.get_logger().error(
                    'Shutdown deadline expired before all arm/gripper '
                    'submissions were confirmed terminal'
                )
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
