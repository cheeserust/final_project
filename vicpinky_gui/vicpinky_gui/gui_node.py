"""Serve a browser dashboard backed by ROS 2 actions, services, and topics."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import errno
import json
import math
import os
from pathlib import Path
import re
import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from flask import Flask, jsonify, request, send_from_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from vicpinky_interfaces.action import ExecuteMission
from vicpinky_interfaces.msg import MissionStatus
from werkzeug.serving import make_server
import yaml


BOARD3_TARGET_LOAD_MAX = 1023
MANUAL_MIN_DURATION_SEC = 0.1
MANUAL_MAX_DURATION_SEC = 30.0

ARM_COMMANDS = {
    'enable': '/arm_board/enable',
    'disable': '/arm_board/disable',
    'home_all': '/arm_board/home_all',
    'clear_error': '/arm_board/clear_error',
    'status': '/arm_board/status',
    'estop': '/arm_board/estop',
}

ARM_MANUAL_JOINTS = [
    {
        'key': 'base',
        'label': 'Base',
        'joint_name': 'base_joint',
        'min_deg': -90.0,
        'max_deg': 180.0,
        'default_deg': -90.0,
    },
    {
        'key': 'axis_1',
        'label': 'Axis 1',
        'joint_name': 'arm_joint_1',
        'min_deg': -90.0,
        'max_deg': 90.0,
        'default_deg': -90.0,
    },
    {
        'key': 'axis_2',
        'label': 'Axis 2',
        'joint_name': 'arm_joint_2',
        'min_deg': -80.0,
        'max_deg': 80.0,
        'default_deg': -80.0,
    },
    {
        'key': 'axis_3',
        'label': 'Axis 3',
        'joint_name': 'arm_joint_3',
        'min_deg': -90.0,
        'max_deg': 90.0,
        'default_deg': -90.0,
    },
    {
        'key': 'axis_4',
        'label': 'Axis 4',
        'joint_name': 'arm_joint_4',
        'min_deg': -170.0,
        'max_deg': 170.0,
        'default_deg': -170.0,
    },
]

GRIPPER_MANUAL_JOINTS = [
    {
        'key': 'finger_1_base',
        'label': 'F1 Base',
        'joint_name': 'finger_1_base_joint',
        'min_deg': -70.3,
        'max_deg': 70.3,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_1_middle',
        'label': 'F1 Middle',
        'joint_name': 'finger_1_middle_joint',
        'min_deg': -137.7,
        'max_deg': 52.7,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_1_tip',
        'label': 'F1 Tip',
        'joint_name': 'finger_1_tip_joint',
        'min_deg': -111.3,
        'max_deg': 111.3,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_2_base',
        'label': 'F2 Base',
        'joint_name': 'finger_2_base_joint',
        'min_deg': -70.3,
        'max_deg': 70.3,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_2_middle',
        'label': 'F2 Middle',
        'joint_name': 'finger_2_middle_joint',
        'min_deg': -137.7,
        'max_deg': 52.7,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_2_tip',
        'label': 'F2 Tip',
        'joint_name': 'finger_2_tip_joint',
        'min_deg': -111.3,
        'max_deg': 111.3,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_3_base',
        'label': 'F3 Base',
        'joint_name': 'finger_3_base_joint',
        'min_deg': -70.3,
        'max_deg': 70.3,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_3_middle',
        'label': 'F3 Middle',
        'joint_name': 'finger_3_middle_joint',
        'min_deg': -137.7,
        'max_deg': 52.7,
        'default_deg': 0.0,
    },
    {
        'key': 'finger_3_tip',
        'label': 'F3 Tip',
        'joint_name': 'finger_3_tip_joint',
        'min_deg': -111.3,
        'max_deg': 111.3,
        'default_deg': 0.0,
    },
]

MANUAL_CONTROLLERS = {
    'arm': {
        'label': 'Arm',
        'action_name': '/arm_controller/follow_joint_trajectory',
        'default_duration_sec': 2.0,
        'joints': ARM_MANUAL_JOINTS,
    },
    'gripper': {
        'label': 'Gripper',
        'action_name': '/gripper_controller/follow_joint_trajectory',
        'default_duration_sec': 1.0,
        'default_target_load_raw': 500,
        'target_load_min': 0,
        'target_load_max': BOARD3_TARGET_LOAD_MAX,
        'joints': GRIPPER_MANUAL_JOINTS,
    },
}

STATUS_NAME_BY_CODE = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class VicPinkyGuiNode(Node):
    """Expose mission and arm-board controls through a local browser UI."""

    def __init__(self) -> None:
        """Create ROS clients/subscribers and start the HTTP server."""
        super().__init__('vicpinky_gui')
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._started_at = time.time()

        self._declare_parameters()
        self._host = str(self.get_parameter('host').value)
        self._port = int(self.get_parameter('port').value)
        self._auto_port = bool(self.get_parameter('auto_port').value)
        self._port_search_limit = int(
            self.get_parameter('port_search_limit').value
        )
        self._request_timeout_s = float(
            self.get_parameter('request_timeout_sec').value
        )
        self._status_log_limit = int(
            self.get_parameter('status_log_limit').value
        )
        self._connection_timeout_s = float(
            self.get_parameter('connection_timeout_sec').value
        )
        self._recovered_hold_s = float(
            self.get_parameter('recovered_hold_sec').value
        )
        self._heartbeat_topic = str(
            self.get_parameter('heartbeat_topic').value
        )
        self._mission_event_topic = str(
            self.get_parameter('mission_event_topic').value
        )

        self._mission_status: dict[str, Any] | None = None
        self._mission_status_seen_at: float | None = None
        self._mission_feedback: dict[str, Any] | None = None
        self._mission_goal: dict[str, Any] | None = None
        self._mission_result: dict[str, Any] | None = None
        self._mission_goal_handle = None

        self._latest_arm_status_raw: str | None = None
        self._latest_arm_status_seen_at: float | None = None
        self._latest_arm_status: dict[str, Any] | None = None
        self._arm_status_log: deque[dict[str, Any]] = deque(
            maxlen=self._status_log_limit,
        )
        self._last_arm_command: dict[str, Any] | None = None

        self._joint_state: dict[str, Any] | None = None
        self._joint_state_seen_at: float | None = None
        self._event_log: deque[dict[str, Any]] = deque(maxlen=120)
        self._heartbeat: dict[str, Any] | None = None
        self._heartbeat_seen_at: float | None = None
        self._mission_event_log: deque[dict[str, Any]] = deque(maxlen=160)
        self._manual_goal_handles: dict[str, Any | None] = {
            name: None for name in MANUAL_CONTROLLERS
        }
        self._manual_last_commands: dict[str, dict[str, Any] | None] = {
            name: None for name in MANUAL_CONTROLLERS
        }
        self._manual_feedback: dict[str, dict[str, Any] | None] = {
            name: None for name in MANUAL_CONTROLLERS
        }
        self._robot_connection_state = 'WAITING'
        self._robot_disconnected_at: float | None = None
        self._robot_last_recovered_at: float | None = None
        self._last_disconnect_window: dict[str, float] | None = None

        self._gui_config = self._load_gui_config()

        self._mission_client = ActionClient(
            self,
            ExecuteMission,
            '/mission/execute',
            callback_group=self._callback_group,
        )
        self._arm_clients = {
            name: self.create_client(
                Trigger,
                service_name,
                callback_group=self._callback_group,
            )
            for name, service_name in ARM_COMMANDS.items()
        }
        self._manual_clients = {
            name: ActionClient(
                self,
                FollowJointTrajectory,
                str(config['action_name']),
                callback_group=self._callback_group,
            )
            for name, config in MANUAL_CONTROLLERS.items()
        }

        self.create_subscription(
            MissionStatus,
            '/mission/status',
            self._mission_status_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            '/arm_board/status_log',
            self._arm_status_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            self._heartbeat_topic,
            self._heartbeat_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            self._mission_event_topic,
            self._mission_event_callback,
            QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=self._callback_group,
        )

        self._flask_app: Flask | None = None
        self._http_server: Any | None = None
        self._http_thread: threading.Thread | None = None
        self._start_http_server()

    def _declare_parameters(self) -> None:
        mission_share = self._package_share_or_empty('mission_manager')
        default_locations = os.path.join(
            mission_share,
            'config',
            'locations.yaml',
        ) if mission_share else ''
        default_flow = os.path.join(
            mission_share,
            'config',
            'mission_flow.yaml',
        ) if mission_share else ''

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('auto_port', True)
        self.declare_parameter('port_search_limit', 20)
        self.declare_parameter('request_timeout_sec', 5.0)
        self.declare_parameter('status_log_limit', 80)
        self.declare_parameter('connection_timeout_sec', 5.0)
        self.declare_parameter('recovered_hold_sec', 12.0)
        self.declare_parameter('heartbeat_topic', '/robot/heartbeat')
        self.declare_parameter('mission_event_topic', '/mission/event_log')
        self.declare_parameter('locations_file', default_locations)
        self.declare_parameter('mission_flow_file', default_flow)

    def _start_http_server(self) -> None:
        static_dir = self._static_dir()
        app = self._create_flask_app(static_dir)
        server = self._bind_http_server(app)

        self._flask_app = app
        self._http_server = server
        self._http_thread = threading.Thread(
            target=server.serve_forever,
            name='vicpinky_gui_http',
            daemon=True,
        )
        self._http_thread.start()

        bind_host = self._host
        if bind_host in ('0.0.0.0', '::'):
            bind_host = 'localhost'
        actual_port = server.socket.getsockname()[1]
        self.get_logger().info(
            f'VicPinky GUI ready: http://{bind_host}:{actual_port}'
        )

    def _create_flask_app(self, static_dir: Path) -> Flask:
        app = Flask(
            'vicpinky_gui',
            static_folder=None,
        )
        app.json.ensure_ascii = False

        @app.after_request
        def add_no_store_headers(response):
            response.headers['Cache-Control'] = 'no-store'
            return response

        @app.get('/')
        def index():
            return send_from_directory(static_dir, 'index.html')

        @app.get('/static/<path:filename>')
        def static_file(filename: str):
            return send_from_directory(static_dir, filename)

        @app.get('/api/snapshot')
        def api_snapshot():
            return jsonify(self.snapshot())

        @app.post('/api/arm/<command>')
        def api_arm_command(command: str):
            response = self.call_arm_service(command)
            return jsonify(response), 200 if response.get('ok') else 503

        @app.post('/api/manual/<controller>')
        def api_manual_command(controller: str):
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                payload = {}
            response = self.send_manual_pose(controller, payload)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.post('/api/mission/start')
        def api_mission_start():
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                payload = {}
            response = self.start_mission(payload)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.post('/api/mission/cancel')
        def api_mission_cancel():
            response = self.cancel_mission()
            return jsonify(response), 200 if response.get('ok') else 409

        @app.errorhandler(404)
        def api_not_found(_error):
            return jsonify({
                'ok': False,
                'message': f'Not found: {request.path}',
            }), 404

        return app

    def _bind_http_server(self, app: Flask) -> Any:
        if self._port == 0:
            candidate_ports = [0]
        elif self._auto_port:
            limit = max(1, self._port_search_limit)
            candidate_ports = list(range(self._port, self._port + limit))
        else:
            candidate_ports = [self._port]

        last_error: OSError | None = None

        for candidate_port in candidate_ports:
            try:
                server = make_server(
                    self._host,
                    candidate_port,
                    app,
                    threaded=True,
                )
            except OSError as exc:
                last_error = exc
                if exc.errno == errno.EADDRINUSE and self._auto_port:
                    self.get_logger().warning(
                        f'HTTP port {candidate_port} is already in use; '
                        'trying the next port'
                    )
                    continue
                raise

            if candidate_port != self._port and self._port != 0:
                self.get_logger().warning(
                    f'HTTP port {self._port} was unavailable; '
                    f'using {candidate_port}'
                )
            return server

        message = (
            f'No free HTTP port found from {candidate_ports[0]} '
            f'to {candidate_ports[-1]}'
        )
        if last_error is not None:
            message = f'{message}: {last_error}'
        raise OSError(message)

    def _static_dir(self) -> Path:
        package_share = self._package_share_or_empty('vicpinky_gui')
        if package_share:
            static_dir = Path(package_share) / 'static'
            if static_dir.is_dir():
                return static_dir

        return Path(__file__).resolve().parents[1] / 'static'

    @staticmethod
    def _package_share_or_empty(package_name: str) -> str:
        try:
            return get_package_share_directory(package_name)
        except Exception:
            return ''

    def _load_gui_config(self) -> dict[str, Any]:
        locations_file = str(self.get_parameter('locations_file').value)
        mission_flow_file = str(self.get_parameter('mission_flow_file').value)

        locations: list[dict[str, Any]] = []
        mission_steps: list[dict[str, Any]] = []

        try:
            location_data = self._read_yaml(locations_file)
            raw_locations = location_data.get('locations', {})
            if isinstance(raw_locations, dict):
                for name, value in raw_locations.items():
                    item = {'name': str(name)}
                    if isinstance(value, dict):
                        if 'pose' not in value:
                            continue
                        item.update({
                            'type': value.get('type', ''),
                            'floor': value.get('floor', ''),
                            'marker_id': value.get('marker_id', ''),
                        })
                    locations.append(item)
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to load GUI locations from {locations_file}: {exc}'
            )

        try:
            flow_data = self._read_yaml(mission_flow_file)
            raw_steps = flow_data.get('mission', {}).get('steps', [])
            if isinstance(raw_steps, list):
                for index, value in enumerate(raw_steps, start=1):
                    if isinstance(value, dict):
                        mission_steps.append({
                            'index': index,
                            'state': str(value.get('state', '')),
                            'task': str(value.get('task', '')),
                            'target': str(value.get('target', '')),
                            'location': str(value.get('location', '')),
                        })
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to load GUI mission flow from {mission_flow_file}: {exc}'
            )

        return {
            'locations': locations,
            'mission_steps': mission_steps,
            'default_goal': {
                'mission_id': self._default_mission_id(),
                'pickup_location': 'object',
                'delivery_location': 'object_place',
                'target_floor': 5,
                'object_label': 'box',
            },
            'arm_commands': list(ARM_COMMANDS.keys()),
            'manual': deepcopy(MANUAL_CONTROLLERS),
        }

    @staticmethod
    def _read_yaml(path: str) -> dict[str, Any]:
        if not path:
            return {}
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        return data if isinstance(data, dict) else {}

    @staticmethod
    def _default_mission_id() -> str:
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        return f'gui_{timestamp}'

    def _mission_status_callback(self, msg: MissionStatus) -> None:
        with self._lock:
            self._mission_status = {
                'mission_id': msg.mission_id,
                'state': msg.state,
                'active_task': msg.active_task,
                'progress': float(msg.progress),
                'error': bool(msg.error),
                'message': msg.message,
                'stamp': self._stamp_to_float(msg.stamp),
            }
            self._mission_status_seen_at = time.time()

    def _arm_status_callback(self, msg: String) -> None:
        parsed_status = self._parse_arm_status(msg.data)
        entry = {
            'received_at': self._now_iso(),
            'message': msg.data,
        }

        with self._lock:
            self._latest_arm_status_raw = msg.data
            self._latest_arm_status_seen_at = time.time()
            self._latest_arm_status = parsed_status
            self._arm_status_log.append(entry)

    def _joint_state_callback(self, msg: JointState) -> None:
        joints = []
        for index, name in enumerate(msg.name):
            position = self._list_value(msg.position, index)
            velocity = self._list_value(msg.velocity, index)
            effort = self._list_value(msg.effort, index)
            joints.append({
                'name': name,
                'position': position,
                'velocity': velocity,
                'effort': effort,
            })

        with self._lock:
            self._joint_state = {
                'stamp': self._stamp_to_float(msg.header.stamp),
                'joints': joints,
            }
            self._joint_state_seen_at = time.time()

    def _heartbeat_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {
                'alive': True,
                'message': msg.data,
            }

        if not isinstance(payload, dict):
            payload = {'alive': True, 'message': str(payload)}

        with self._lock:
            self._heartbeat = payload
            self._heartbeat_seen_at = time.time()

    def _mission_event_callback(self, msg: String) -> None:
        received_at = time.time()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {
                'level': 'info',
                'message': msg.data,
            }

        if not isinstance(payload, dict):
            payload = {'level': 'info', 'message': str(payload)}

        try:
            event_stamp = float(payload.get('stamp', received_at))
        except (TypeError, ValueError):
            event_stamp = received_at

        payload['stamp'] = event_stamp
        payload.setdefault('time', self._now_iso(event_stamp))
        payload['received_at'] = self._now_iso(received_at)

        with self._lock:
            self._mission_event_log.append(payload)

    @staticmethod
    def _list_value(values: Any, index: int) -> float | None:
        try:
            return float(values[index])
        except (IndexError, TypeError, ValueError):
            return None

    @staticmethod
    def _stamp_to_float(stamp: Any) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def snapshot(self) -> dict[str, Any]:
        """Return the complete dashboard state as JSON-serializable data."""
        now = time.time()

        with self._lock:
            robot_connection = self._robot_connection_snapshot_locked(now)
            mission_status = deepcopy(self._mission_status)
            mission_feedback = deepcopy(self._mission_feedback)
            mission_goal = deepcopy(self._mission_goal)
            mission_result = deepcopy(self._mission_result)
            arm_status = deepcopy(self._latest_arm_status)
            arm_status_raw = self._latest_arm_status_raw
            arm_log = list(self._arm_status_log)
            last_arm_command = deepcopy(self._last_arm_command)
            joint_state = deepcopy(self._joint_state)
            event_log = list(self._event_log)
            mission_events = list(self._mission_event_log)
            mission_active = self._mission_goal_handle is not None
            manual_last_commands = deepcopy(self._manual_last_commands)
            manual_feedback = deepcopy(self._manual_feedback)
            manual_active = {
                name: self._manual_goal_handles.get(name) is not None
                for name in MANUAL_CONTROLLERS
            }
            mission_status_age = self._age_ms(self._mission_status_seen_at, now)
            arm_status_age = self._age_ms(self._latest_arm_status_seen_at, now)
            joint_state_age = self._age_ms(self._joint_state_seen_at, now)

        manual_controllers = {}
        for name, config in MANUAL_CONTROLLERS.items():
            manual_controllers[name] = {
                'label': config['label'],
                'action_name': config['action_name'],
                'ready': self._manual_action_ready(name),
                'active': manual_active.get(name, False),
            }

        return {
            'ok': True,
            'server': {
                'time': self._now_iso(now),
                'uptime_s': now - self._started_at,
            },
            'robot_connection': robot_connection,
            'mission': {
                'status': mission_status,
                'status_age_ms': mission_status_age,
                'feedback': mission_feedback,
                'goal': mission_goal,
                'result': mission_result,
                'active': mission_active,
                'action_ready': self._mission_action_ready(),
            },
            'arm': {
                'parsed_status': arm_status,
                'status_raw': arm_status_raw,
                'status_age_ms': arm_status_age,
                'services': self._arm_services_ready(),
                'last_command': last_arm_command,
                'log': arm_log,
            },
            'manual': {
                'controllers': manual_controllers,
                'last_commands': manual_last_commands,
                'feedback': manual_feedback,
                'mission_active': mission_active,
            },
            'joints': {
                'state': joint_state,
                'age_ms': joint_state_age,
            },
            'events': event_log,
            'mission_events': mission_events,
            'config': self._gui_config,
        }

    def _robot_connection_snapshot_locked(self, now: float) -> dict[str, Any]:
        seen_at = self._heartbeat_seen_at
        source = 'heartbeat'
        if seen_at is None:
            seen_at = self._mission_status_seen_at
            source = 'mission_status'

        previous_state = self._robot_connection_state
        age_s = None if seen_at is None else max(0.0, now - seen_at)
        timed_out = age_s is None or age_s > self._connection_timeout_s

        if timed_out:
            state = 'WAITING' if previous_state == 'WAITING' else 'LOST'
            if previous_state in ('ONLINE', 'RECOVERED') and seen_at is not None:
                self._robot_disconnected_at = seen_at
                self._append_event_locked(
                    kind='network',
                    level='error',
                    message='Robot connection lost',
                )
            elif self._robot_disconnected_at is None:
                self._robot_disconnected_at = seen_at or now
        else:
            if previous_state in ('LOST', 'WAITING'):
                self._robot_last_recovered_at = now
                self._last_disconnect_window = {
                    'start': self._robot_disconnected_at or seen_at or now,
                    'end': now,
                }
                self._robot_disconnected_at = None
                self._append_event_locked(
                    kind='network',
                    level='info',
                    message='Robot connection recovered',
                )

            if (
                self._robot_last_recovered_at is not None
                and now - self._robot_last_recovered_at <= self._recovered_hold_s
            ):
                state = 'RECOVERED'
            else:
                state = 'ONLINE'

        self._robot_connection_state = state

        active_window = (
            self._last_disconnect_window
            if self._last_disconnect_window is not None
            else (
                {'start': self._robot_disconnected_at, 'end': now}
                if self._robot_disconnected_at is not None
                else None
            )
        )
        events_during_disconnect = []
        if active_window is not None and active_window['start'] is not None:
            start = float(active_window['start'])
            end = float(active_window['end'])
            for event in self._mission_event_log:
                try:
                    stamp = float(event.get('stamp', 0.0))
                except (TypeError, ValueError):
                    continue
                if start <= stamp <= end:
                    events_during_disconnect.append(deepcopy(event))

        disconnected_duration_s = None
        if self._robot_disconnected_at is not None:
            disconnected_duration_s = max(0.0, now - self._robot_disconnected_at)
        elif self._last_disconnect_window is not None:
            disconnected_duration_s = max(
                0.0,
                self._last_disconnect_window['end']
                - self._last_disconnect_window['start'],
            )

        return {
            'state': state,
            'source': source,
            'heartbeat': deepcopy(self._heartbeat),
            'heartbeat_age_ms': self._age_ms(self._heartbeat_seen_at, now),
            'last_seen': self._now_iso(seen_at) if seen_at is not None else None,
            'timeout_s': self._connection_timeout_s,
            'disconnected_since': (
                self._now_iso(self._robot_disconnected_at)
                if self._robot_disconnected_at is not None
                else None
            ),
            'recovered_at': (
                self._now_iso(self._robot_last_recovered_at)
                if self._robot_last_recovered_at is not None
                else None
            ),
            'disconnected_duration_s': disconnected_duration_s,
            'events_during_disconnect': events_during_disconnect[-20:],
        }

    @staticmethod
    def _age_ms(seen_at: float | None, now: float) -> float | None:
        if seen_at is None:
            return None
        return max(0.0, (now - seen_at) * 1000.0)

    def _mission_action_ready(self) -> bool:
        try:
            return bool(self._mission_client.server_is_ready())
        except Exception:
            return False

    def _arm_services_ready(self) -> dict[str, bool]:
        return {
            name: bool(client.service_is_ready())
            for name, client in self._arm_clients.items()
        }

    def _manual_action_ready(self, controller: str) -> bool:
        client = self._manual_clients.get(controller)
        if client is None:
            return False
        try:
            return bool(client.server_is_ready())
        except Exception:
            return False

    def call_arm_service(self, command: str) -> dict[str, Any]:
        """Call one arm board Trigger service from an HTTP request."""
        if command not in self._arm_clients:
            return {
                'ok': False,
                'command': command,
                'message': f'Unknown arm command: {command}',
            }

        client = self._arm_clients[command]
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.25)

        if not client.service_is_ready():
            result = {
                'ok': False,
                'command': command,
                'message': f'Service is not ready: {ARM_COMMANDS[command]}',
                'finished_at': self._now_iso(),
            }
            self._record_arm_command(result)
            return result

        future = client.call_async(Trigger.Request())
        response = self._wait_for_future(future)

        if response is None:
            result = {
                'ok': False,
                'command': command,
                'message': f'Service timed out: {ARM_COMMANDS[command]}',
                'finished_at': self._now_iso(),
            }
            self._record_arm_command(result)
            return result

        result = {
            'ok': bool(response.success),
            'command': command,
            'service': ARM_COMMANDS[command],
            'message': response.message,
            'finished_at': self._now_iso(),
        }
        self._record_arm_command(result)
        return result

    def _record_arm_command(self, result: dict[str, Any]) -> None:
        with self._lock:
            self._last_arm_command = deepcopy(result)
            self._append_event_locked(
                kind='arm',
                level='info' if result.get('ok') else 'error',
                message=(
                    f'{result.get("command", "arm")} '
                    f'{"ok" if result.get("ok") else "failed"}: '
                    f'{result.get("message", "")}'
                ),
            )

    def send_manual_pose(
        self,
        controller: str,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        """Send one manual FollowJointTrajectory goal from the dashboard."""
        controller = str(controller)
        config = MANUAL_CONTROLLERS.get(controller)
        client = self._manual_clients.get(controller)
        if config is None or client is None:
            return {
                'ok': False,
                'status_code': 404,
                'message': f'Unknown manual controller: {controller}',
            }

        with self._lock:
            if self._mission_goal_handle is not None:
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'Manual control is blocked while a mission is active',
                }
            if self._manual_goal_handles.get(controller) is not None:
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': f'{config["label"]} manual goal is already active',
                }

        if not self._manual_action_ready(controller):
            client.wait_for_server(timeout_sec=0.25)

        if not self._manual_action_ready(controller):
            return {
                'ok': False,
                'status_code': 503,
                'message': (
                    f'Action server is not ready: {config["action_name"]}'
                ),
            }

        try:
            goal_msg, summary = self._manual_goal_from_payload(
                controller,
                payload,
            )
        except (TypeError, ValueError) as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': str(exc),
            }

        with self._lock:
            self._manual_last_commands[controller] = {
                **summary,
                'state': 'SENDING',
                'sent_at': self._now_iso(),
            }
            self._manual_feedback[controller] = None
            self._append_event_locked(
                kind='manual',
                level='info',
                message=f'Sending {config["label"]} manual goal',
            )

        send_future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg: self._manual_feedback_callback(
                controller,
                msg,
            ),
        )
        goal_handle = self._wait_for_future(send_future)

        if goal_handle is None:
            with self._lock:
                if self._manual_last_commands[controller] is not None:
                    self._manual_last_commands[controller]['state'] = (
                        'SEND_TIMEOUT'
                    )
                self._append_event_locked(
                    kind='manual',
                    level='error',
                    message=f'{config["label"]} manual send timed out',
                )
            return {
                'ok': False,
                'status_code': 504,
                'message': f'Timed out while sending {controller} goal',
            }

        if not goal_handle.accepted:
            with self._lock:
                if self._manual_last_commands[controller] is not None:
                    self._manual_last_commands[controller]['state'] = 'REJECTED'
                self._append_event_locked(
                    kind='manual',
                    level='error',
                    message=f'{config["label"]} manual goal rejected',
                )
            return {
                'ok': False,
                'status_code': 409,
                'message': f'{config["label"]} manual goal was rejected',
            }

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._manual_result_callback(controller, future)
        )

        with self._lock:
            self._manual_goal_handles[controller] = goal_handle
            if self._manual_last_commands[controller] is not None:
                self._manual_last_commands[controller]['state'] = 'ACCEPTED'
                self._manual_last_commands[controller]['accepted_at'] = (
                    self._now_iso()
                )
            self._append_event_locked(
                kind='manual',
                level='info',
                message=f'{config["label"]} manual goal accepted',
            )

        return {
            'ok': True,
            'message': f'{config["label"]} manual goal accepted',
            'controller': controller,
            'goal': summary,
        }

    def _manual_goal_from_payload(
        self,
        controller: str,
        payload: dict[str, Any],
    ) -> tuple[FollowJointTrajectory.Goal, dict[str, Any]]:
        config = MANUAL_CONTROLLERS[controller]
        joints = list(config['joints'])
        positions_deg = self._manual_positions_from_payload(
            payload.get('positions_deg'),
            joints,
        )
        duration_s = self._manual_duration_from_payload(
            payload.get('duration_sec'),
            float(config['default_duration_sec']),
        )
        target_load_raw = None
        if controller == 'gripper':
            target_load_raw = self._manual_target_load_from_payload(
                payload.get(
                    'target_load_raw',
                    config.get('default_target_load_raw'),
                )
            )

        point = JointTrajectoryPoint()
        point.positions = [math.radians(value) for value in positions_deg]
        if target_load_raw is not None:
            point.effort = [
                float(target_load_raw)
                for _ in positions_deg
            ]
        self._set_duration(point, duration_s)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            str(joint['joint_name'])
            for joint in joints
        ]
        goal_msg.trajectory.points = [point]

        summary: dict[str, Any] = {
            'controller': controller,
            'action_name': config['action_name'],
            'positions_deg': {
                str(joint['key']): positions_deg[index]
                for index, joint in enumerate(joints)
            },
            'joint_names': goal_msg.trajectory.joint_names,
            'duration_sec': duration_s,
        }
        if target_load_raw is not None:
            summary['target_load_raw'] = target_load_raw

        return goal_msg, summary

    @staticmethod
    def _manual_positions_from_payload(
        raw_positions: Any,
        joints: list[dict[str, Any]],
    ) -> list[float]:
        if not isinstance(raw_positions, (dict, list, tuple)):
            raise ValueError('positions_deg must be an object or list')

        positions = []
        for index, joint in enumerate(joints):
            key = str(joint['key'])
            joint_name = str(joint['joint_name'])
            if isinstance(raw_positions, dict):
                raw_value = raw_positions.get(key, raw_positions.get(joint_name))
            else:
                raw_value = (
                    raw_positions[index]
                    if index < len(raw_positions)
                    else None
                )

            if raw_value is None:
                raise ValueError(f'positions_deg is missing {key}')

            value = float(raw_value)
            if not math.isfinite(value):
                raise ValueError(f'{key} target is not finite')

            minimum = float(joint['min_deg'])
            maximum = float(joint['max_deg'])
            if value < minimum or value > maximum:
                raise ValueError(
                    f'{key} target {value:.3f} deg is outside '
                    f'[{minimum:.3f}, {maximum:.3f}] deg'
                )
            positions.append(value)

        return positions

    @staticmethod
    def _manual_duration_from_payload(
        raw_duration: Any,
        default_duration_s: float,
    ) -> float:
        duration_s = (
            default_duration_s
            if raw_duration is None
            else float(raw_duration)
        )
        if not math.isfinite(duration_s):
            raise ValueError('duration_sec is not finite')
        if (
            duration_s < MANUAL_MIN_DURATION_SEC
            or duration_s > MANUAL_MAX_DURATION_SEC
        ):
            raise ValueError(
                'duration_sec must be in '
                f'{MANUAL_MIN_DURATION_SEC}..{MANUAL_MAX_DURATION_SEC}'
            )
        return duration_s

    @staticmethod
    def _manual_target_load_from_payload(raw_load: Any) -> int:
        target_load = int(raw_load)
        if not 0 <= target_load <= BOARD3_TARGET_LOAD_MAX:
            raise ValueError(
                f'target_load_raw must be in 0..{BOARD3_TARGET_LOAD_MAX}'
            )
        return target_load

    @staticmethod
    def _set_duration(
        point: JointTrajectoryPoint,
        duration_s: float,
    ) -> None:
        whole_seconds = int(duration_s)
        nanoseconds = int(round(
            (duration_s - whole_seconds) * 1_000_000_000
        ))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds

    def _manual_feedback_callback(
        self,
        controller: str,
        feedback_msg: Any,
    ) -> None:
        feedback = feedback_msg.feedback
        with self._lock:
            self._manual_feedback[controller] = {
                'actual_deg': [
                    math.degrees(float(value))
                    for value in feedback.actual.positions
                ],
                'desired_deg': [
                    math.degrees(float(value))
                    for value in feedback.desired.positions
                ],
                'received_at': self._now_iso(),
            }

    def _manual_result_callback(
        self,
        controller: str,
        future: Any,
    ) -> None:
        config = MANUAL_CONTROLLERS[controller]
        try:
            result_response = future.result()
        except Exception as exc:
            result_payload = {
                'ok': False,
                'status': 'ERROR',
                'error_code': None,
                'error_string': str(exc),
                'received_at': self._now_iso(),
            }
        else:
            result = result_response.result
            status_name = STATUS_NAME_BY_CODE.get(
                result_response.status,
                f'STATUS_{result_response.status}',
            )
            result_payload = {
                'ok': (
                    result.error_code
                    == FollowJointTrajectory.Result.SUCCESSFUL
                ),
                'status': status_name,
                'error_code': int(result.error_code),
                'error_string': result.error_string,
                'received_at': self._now_iso(),
            }

        with self._lock:
            self._manual_goal_handles[controller] = None
            if self._manual_last_commands[controller] is not None:
                self._manual_last_commands[controller]['state'] = (
                    result_payload['status']
                )
                self._manual_last_commands[controller]['result'] = (
                    result_payload
                )
                self._manual_last_commands[controller]['finished_at'] = (
                    result_payload['received_at']
                )
            self._append_event_locked(
                kind='manual',
                level='info' if result_payload['ok'] else 'error',
                message=(
                    f'{config["label"]} manual {result_payload["status"]}: '
                    f'{result_payload["error_string"]}'
                ),
            )

    def start_mission(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Send an ExecuteMission goal from the dashboard."""
        with self._lock:
            if self._mission_goal_handle is not None:
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'A mission goal is already active',
                }

        if not self._mission_action_ready():
            self._mission_client.wait_for_server(timeout_sec=0.25)

        if not self._mission_action_ready():
            return {
                'ok': False,
                'status_code': 503,
                'message': '/mission/execute action server is not ready',
            }

        try:
            goal_msg, goal_summary = self._mission_goal_from_payload(payload)
        except (TypeError, ValueError) as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': str(exc),
            }

        with self._lock:
            self._mission_goal = {
                **goal_summary,
                'state': 'SENDING',
                'sent_at': self._now_iso(),
            }
            self._mission_feedback = None
            self._mission_result = None
            self._append_event_locked(
                kind='mission',
                level='info',
                message=f'Sending mission {goal_summary["mission_id"]}',
            )

        send_future = self._mission_client.send_goal_async(
            goal_msg,
            feedback_callback=self._mission_feedback_callback,
        )
        goal_handle = self._wait_for_future(send_future)

        if goal_handle is None:
            with self._lock:
                if self._mission_goal is not None:
                    self._mission_goal['state'] = 'SEND_TIMEOUT'
                self._append_event_locked(
                    kind='mission',
                    level='error',
                    message='Mission send timed out',
                )
            return {
                'ok': False,
                'status_code': 504,
                'message': 'Timed out while sending mission goal',
            }

        if not goal_handle.accepted:
            with self._lock:
                if self._mission_goal is not None:
                    self._mission_goal['state'] = 'REJECTED'
                self._append_event_locked(
                    kind='mission',
                    level='error',
                    message=f'Mission rejected: {goal_summary["mission_id"]}',
                )
            return {
                'ok': False,
                'status_code': 409,
                'message': 'Mission goal was rejected',
            }

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._mission_result_callback)

        with self._lock:
            self._mission_goal_handle = goal_handle
            if self._mission_goal is not None:
                self._mission_goal['state'] = 'ACCEPTED'
                self._mission_goal['accepted_at'] = self._now_iso()
            self._append_event_locked(
                kind='mission',
                level='info',
                message=f'Mission accepted: {goal_summary["mission_id"]}',
            )

        return {
            'ok': True,
            'message': 'Mission goal accepted',
            'goal': goal_summary,
        }

    def _mission_goal_from_payload(
        self,
        payload: dict[str, Any],
    ) -> tuple[ExecuteMission.Goal, dict[str, Any]]:
        defaults = self._gui_config['default_goal']
        mission_id = str(
            payload.get('mission_id') or self._default_mission_id()
        ).strip()
        pickup_location = str(
            payload.get('pickup_location')
            or defaults['pickup_location']
        ).strip()
        delivery_location = str(
            payload.get('delivery_location')
            or defaults['delivery_location']
        ).strip()
        object_label = str(
            payload.get('object_label')
            or defaults['object_label']
        ).strip()

        if not mission_id:
            raise ValueError('mission_id is required')
        if not pickup_location:
            raise ValueError('pickup_location is required')
        if not delivery_location:
            raise ValueError('delivery_location is required')
        if not object_label:
            raise ValueError('object_label is required')

        target_floor = int(payload.get(
            'target_floor',
            defaults['target_floor'],
        ))

        goal_msg = ExecuteMission.Goal()
        goal_msg.mission_id = mission_id
        goal_msg.pickup_location = pickup_location
        goal_msg.delivery_location = delivery_location
        goal_msg.target_floor = target_floor
        goal_msg.object_label = object_label

        return goal_msg, {
            'mission_id': mission_id,
            'pickup_location': pickup_location,
            'delivery_location': delivery_location,
            'target_floor': target_floor,
            'object_label': object_label,
        }

    def _mission_feedback_callback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        with self._lock:
            self._mission_feedback = {
                'current_state': feedback.current_state,
                'current_task': feedback.current_task,
                'progress': float(feedback.progress),
                'detail': feedback.detail,
                'received_at': self._now_iso(),
            }

    def _mission_result_callback(self, future: Any) -> None:
        try:
            result_response = future.result()
        except Exception as exc:
            result_payload = {
                'ok': False,
                'status': 'ERROR',
                'success': False,
                'final_state': 'ERROR',
                'message': str(exc),
                'received_at': self._now_iso(),
            }
        else:
            result = result_response.result
            status_name = STATUS_NAME_BY_CODE.get(
                result_response.status,
                f'STATUS_{result_response.status}',
            )
            result_payload = {
                'ok': bool(result.success),
                'status': status_name,
                'success': bool(result.success),
                'final_state': result.final_state,
                'message': result.message,
                'received_at': self._now_iso(),
            }

        with self._lock:
            self._mission_result = result_payload
            self._mission_goal_handle = None
            if self._mission_goal is not None:
                self._mission_goal['state'] = result_payload['status']
            self._append_event_locked(
                kind='mission',
                level='info' if result_payload['success'] else 'error',
                message=(
                    f'Mission {result_payload["status"]}: '
                    f'{result_payload["message"]}'
                ),
            )

    def cancel_mission(self) -> dict[str, Any]:
        """Cancel the active mission goal if one exists."""
        with self._lock:
            goal_handle = self._mission_goal_handle

        if goal_handle is None:
            return {
                'ok': False,
                'message': 'No active mission goal to cancel',
            }

        cancel_future = goal_handle.cancel_goal_async()
        response = self._wait_for_future(cancel_future)

        if response is None:
            return {
                'ok': False,
                'message': 'Timed out while canceling mission goal',
            }

        canceling = len(response.goals_canceling) > 0
        with self._lock:
            if self._mission_goal is not None:
                self._mission_goal['state'] = (
                    'CANCELING' if canceling else 'CANCEL_REJECTED'
                )
            self._append_event_locked(
                kind='mission',
                level='info' if canceling else 'error',
                message='Mission cancel requested'
                if canceling else 'Mission cancel rejected',
            )

        return {
            'ok': canceling,
            'message': 'Mission cancel requested'
            if canceling else 'Mission cancel rejected',
        }

    def _wait_for_future(self, future: Any) -> Any:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())

        if not event.wait(timeout=self._request_timeout_s):
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warning(f'Future failed: {exc}')
            return None

    def _append_event_locked(
        self,
        *,
        kind: str,
        level: str,
        message: str,
    ) -> None:
        self._event_log.append({
            'kind': kind,
            'level': level,
            'message': message,
            'time': self._now_iso(),
        })

    @staticmethod
    def _parse_arm_status(message: str) -> dict[str, Any]:
        prefix = ''
        body = message
        if ': ' in message:
            prefix, body = message.split(': ', 1)

        controllers = []
        for match in re.finditer(r'(\w+)\[(.*?)\](?=\s*(?:\|\||$))', body):
            controller_name = match.group(1)
            controller_body = match.group(2)
            controller = {
                'name': controller_name,
                'accept_traj': None,
                'boards': [],
            }

            for piece in controller_body.split(';'):
                piece = piece.strip()
                if not piece:
                    continue

                if piece.startswith('accept_traj='):
                    controller['accept_traj'] = _parse_scalar(
                        piece.split('=', 1)[1],
                    )
                    continue

                board = _parse_board_piece(piece)
                if board is not None:
                    controller['boards'].append(board)

            controllers.append(controller)

        return {
            'prefix': prefix,
            'controllers': controllers,
            'raw': message,
        }

    @staticmethod
    def _now_iso(value: float | None = None) -> str:
        timestamp = time.time() if value is None else value
        return datetime.fromtimestamp(
            timestamp,
            tz=timezone.utc,
        ).isoformat(timespec='milliseconds')

    def destroy_node(self) -> bool:
        """Stop the HTTP server before destroying the ROS node."""
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
        if self._http_thread is not None:
            self._http_thread.join(timeout=1.0)
        return super().destroy_node()


def _parse_board_piece(piece: str) -> dict[str, Any] | None:
    match = re.match(r'board(\d+)(?::|=)\s*(.*)', piece)
    if match is None:
        return None

    board_id = int(match.group(1))
    rest = match.group(2)
    fields: dict[str, Any] = {}
    notes: list[str] = []

    for token in rest.split(','):
        token = token.strip()
        if not token:
            continue
        if '=' not in token:
            notes.append(token)
            continue
        key, value = token.split('=', 1)
        fields[key.strip()] = _parse_scalar(value.strip())

    return {
        'board_id': board_id,
        'fields': fields,
        'notes': notes,
        'raw': piece,
    }


def _parse_scalar(value: str) -> Any:
    if value == 'True':
        return True
    if value == 'False':
        return False
    if value == 'None':
        return None

    try:
        if value.lower().startswith('0x'):
            return int(value, 16)
        return int(value)
    except ValueError:
        pass

    try:
        return float(value)
    except ValueError:
        return value


def main(args: list[str] | None = None) -> None:
    """Run the VicPinky GUI node."""
    rclpy.init(args=args)
    node = VicPinkyGuiNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
