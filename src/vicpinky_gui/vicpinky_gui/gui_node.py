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
import socket
import sqlite3
import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from flask import Flask, jsonify, request, send_from_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from vicpinky_interfaces.action import ExecuteArmGoal, ExecuteMission, RunTask
from vicpinky_interfaces.msg import MissionStatus
from werkzeug.serving import make_server
import yaml

from .saved_pose_store import (
    SavedPoseNotFoundError,
    SavedPoseStore,
    SavedPoseStoreError,
)


BOARD3_TARGET_LOAD_MAX = 1023
MANUAL_MIN_DURATION_SEC = 0.1
MANUAL_MAX_DURATION_SEC = 30.0

SEQUENCE_ACTIVE_STATES = {'STARTING', 'RUNNING', 'STOPPING'}
ACTION_BLOCKING_STATES = {
    'SENDING',
    'SEND_TIMEOUT_PENDING',
    'ACCEPTED',
    'CANCELING',
}

BUTTON_PRESS_STATES = {
    'PRESS_5F_BUTTON': 5,
    'PRESS_4F_BUTTON': 4,
}

ELEVATOR_CALL_BUTTON_STATES = {
    'PRESS_ELEVATOR_CALL_BUTTON',
    'PRESS_ELEVATOR_CALL_BUTTON_RETURN',
}

ELEVATOR_ENTER_STATES = {
    'ENTER_ELEVATOR',
    'ENTER_ELEVATOR_RETURN',
}

ELEVATOR_EXIT_STATES = {
    'EXIT_ELEVATOR',
    'EXIT_ELEVATOR_RETURN',
}

ELEVATOR_WAIT_FLOOR_STATES = {
    'WAIT_5F': 5,
    'WAIT_4F': 4,
}

ELEVATOR_FSM_STATES = (
    'ARM_HOMING',
    'ARM_READY_AT_PICKUP',
    'PICK_OBJECT_TO_TRAY',
    'GO_TO_ELEVATOR_FRONT',
    'ALIGN_ELEVATOR_TAG',
    'PRESS_ELEVATOR_CALL_BUTTON',
    'READY_AND_APPROACH_ELEVATOR_4F',
    'FACE_ELEVATOR_4F',
    'WAIT_ELEVATOR_OPEN',
    'ENTER_ELEVATOR',
    'PRESS_5F_BUTTON',
    'WAIT_5F',
    'EXIT_ELEVATOR',
    'SWITCH_5F_MAP',
    'GO_TO_TARGET_PLACE',
    'ROTATE_AT_DELIVERY',
    'DELIVER_OBJECT_FROM_TRAY',
    'RETURN_TO_ELEVATOR',
    'ALIGN_ELEVATOR_TAG_RETURN',
    'PRESS_ELEVATOR_CALL_BUTTON_RETURN',
    'READY_AND_APPROACH_ELEVATOR_5F',
    'FACE_ELEVATOR_5F',
    'WAIT_ELEVATOR_OPEN_RETURN',
    'ENTER_ELEVATOR_RETURN',
    'PRESS_4F_BUTTON',
    'WAIT_4F',
    'EXIT_ELEVATOR_RETURN',
    'SWITCH_4F_MAP',
    'RETURN_HOME',
    'DONE',
)

ELEVATOR_FSM_INDEX = {
    state: index for index, state in enumerate(ELEVATOR_FSM_STATES)
}

MAP_SWITCH_STATES = {
    'SWITCH_5F_MAP': 5,
    'SWITCH_4F_MAP': 4,
}

SUCCESS_EVENT_NEXT_STATE_BY_STATE = {
    'SWITCH_5F_MAP': 'GO_TO_TARGET_PLACE',
    'SWITCH_4F_MAP': 'RETURN_HOME',
}

SUCCESS_EVENT_NEXT_STATE_BY_TYPE_AND_STATE = {
    ('BUTTON_PRESS_SUCCESS', 'PRESS_5F_BUTTON'): 'WAIT_5F',
    ('BUTTON_PRESS_SUCCESS', 'PRESS_4F_BUTTON'): 'WAIT_4F',
    ('ELEVATOR_CALL_BUTTON_DONE', 'PRESS_ELEVATOR_CALL_BUTTON'):
        'READY_AND_APPROACH_ELEVATOR_4F',
    ('ELEVATOR_CALL_BUTTON_DONE', 'PRESS_ELEVATOR_CALL_BUTTON_RETURN'):
        'READY_AND_APPROACH_ELEVATOR_5F',
    ('ELEVATOR_ENTERED', 'ENTER_ELEVATOR'): 'PRESS_5F_BUTTON',
    ('ELEVATOR_ENTERED', 'ENTER_ELEVATOR_RETURN'): 'PRESS_4F_BUTTON',
    ('TARGET_FLOOR_ARRIVED', 'WAIT_5F'): 'EXIT_ELEVATOR',
    ('TARGET_FLOOR_ARRIVED', 'WAIT_4F'): 'EXIT_ELEVATOR_RETURN',
    ('ELEVATOR_EXIT_DONE', 'EXIT_ELEVATOR'): 'SWITCH_5F_MAP',
    ('ELEVATOR_EXIT_DONE', 'EXIT_ELEVATOR_RETURN'): 'SWITCH_4F_MAP',
}

ARM_COMMANDS = {
    'enable': '/arm_board/enable',
    'disable': '/arm_board/disable',
    'home_all': '/arm_board/home_all',
    'clear_error': '/arm_board/clear_error',
    'status': '/arm_board/status',
    'estop': '/arm_board/estop',
}

ARM_TASK_OPTIONS = [
    {
        'name': 'deliver_object_1_from_tray',
        'label': 'Object 1 (doll): tray to 5F',
    },
]

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
        'min_deg': -85.0,
        'max_deg': 90.0,
        'default_deg': -85.0,
    },
    {
        'key': 'axis_2',
        'label': 'Axis 2',
        'joint_name': 'arm_joint_2',
        'min_deg': -78.1,
        'max_deg': 80.0,
        'default_deg': -78.1,
    },
    {
        'key': 'axis_3',
        'label': 'Axis 3',
        'joint_name': 'arm_joint_3',
        'min_deg': -91.5,
        'max_deg': 90.0,
        'default_deg': -91.5,
    },
    {
        'key': 'axis_4',
        'label': 'Axis 4',
        'joint_name': 'arm_joint_4',
        'min_deg': -90.0,
        'max_deg': 90.0,
        'default_deg': -90.0,
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
        'action_name': '/arm_controller/execute_joint_goal',
        'interface': 'direct_arm_v3',
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
        self._home_service_timeout_s = float(
            self.get_parameter('home_service_timeout_sec').value
        )
        self._clear_service_timeout_s = float(
            self.get_parameter('clear_service_timeout_sec').value
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
        self._event_log_db_path = self._resolve_event_log_db_path(
            str(self.get_parameter('event_log_db_path').value)
        )
        self._event_log_db = self._open_event_log_db(self._event_log_db_path)
        self._fallback_event_seq = 0

        self._mission_status: dict[str, Any] | None = None
        self._mission_status_seen_at: float | None = None
        self._mission_feedback: dict[str, Any] | None = None
        self._mission_goal: dict[str, Any] | None = None
        self._mission_result: dict[str, Any] | None = None
        self._mission_goal_handle = None
        self._nav_goal: dict[str, Any] | None = None
        self._nav_feedback: dict[str, Any] | None = None
        self._nav_result: dict[str, Any] | None = None
        self._nav_goal_handle = None
        self._manual_controllers = self._manual_controllers_from_parameters()
        self._saved_pose_file = self._resolve_saved_pose_file(
            str(self.get_parameter('saved_pose_file').value)
        )
        self._saved_pose_store: SavedPoseStore | None = None
        self._saved_pose_store_error: str | None = None
        try:
            self._saved_pose_store = SavedPoseStore(self._saved_pose_file)
        except SavedPoseStoreError as exc:
            self._saved_pose_store_error = str(exc)
            self.get_logger().error(
                f'Failed to initialize saved pose store '
                f'{self._saved_pose_file}: {exc}'
            )
        sequence_margin = float(
            self.get_parameter('sequence_step_timeout_margin_sec').value
        )
        self._sequence_step_timeout_margin_s = (
            sequence_margin
            if math.isfinite(sequence_margin) and sequence_margin >= 0.0
            else 30.0
        )
        sequence_result_timeout = float(
            self.get_parameter('sequence_step_result_timeout_sec').value
        )
        self._sequence_step_result_timeout_s = (
            sequence_result_timeout
            if (
                math.isfinite(sequence_result_timeout)
                and sequence_result_timeout > 0.0
            )
            else 120.0
        )
        self._sequence_state = self._idle_sequence_state()
        self._sequence_thread: threading.Thread | None = None
        self._sequence_stop_event = threading.Event()
        self._sequence_cancel_lock = threading.Lock()
        self._sequence_cancel_requested: set[str] = set()

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
            name: None for name in self._manual_controllers
        }
        self._manual_last_commands: dict[str, dict[str, Any] | None] = {
            name: None for name in self._manual_controllers
        }
        self._manual_feedback: dict[str, dict[str, Any] | None] = {
            name: None for name in self._manual_controllers
        }
        self._manual_pending_latest_targets: dict[
            str, dict[str, Any] | None
        ] = {
            name: None for name in self._manual_controllers
        }
        self._manual_dispatching_latest_targets: dict[
            str, dict[str, Any] | None
        ] = {
            name: None for name in self._manual_controllers
        }
        self._manual_cancel_requested: dict[str, str | None] = {
            name: None for name in self._manual_controllers
        }
        self._robot_connection_state = 'WAITING'
        self._robot_disconnected_at: float | None = None
        self._robot_last_recovered_at: float | None = None
        self._last_disconnect_window: dict[str, float] | None = None
        self._active_board_faults: dict[tuple[str, int], dict[str, Any]] = {}

        self._map_topic = str(self.get_parameter('map_topic').value)
        self._amcl_pose_topic = str(
            self.get_parameter('amcl_pose_topic').value
        )
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._global_path_topic = str(
            self.get_parameter('global_path_topic').value
        )
        self._local_path_topic = str(
            self.get_parameter('local_path_topic').value
        )
        self._initial_pose_topic = '/initialpose'

        self._driving_map: dict[str, Any] | None = None
        self._driving_map_seen_at: float | None = None
        self._driving_map_revision = 0
        self._amcl_pose: dict[str, Any] | None = None
        self._amcl_pose_seen_at: float | None = None
        self._odom: dict[str, Any] | None = None
        self._odom_seen_at: float | None = None
        self._global_path: dict[str, Any] | None = None
        self._global_path_seen_at: float | None = None
        self._local_path: dict[str, Any] | None = None
        self._local_path_seen_at: float | None = None

        self._gui_config = self._load_gui_config()

        self._mission_client = ActionClient(
            self,
            ExecuteMission,
            '/mission/execute',
            callback_group=self._callback_group,
        )
        self._nav_client = ActionClient(
            self,
            RunTask,
            '/nav/go_to',
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
                (
                    ExecuteArmGoal
                    if config.get('interface') == 'direct_arm_v3'
                    else FollowJointTrajectory
                ),
                str(config['action_name']),
                callback_group=self._callback_group,
            )
            for name, config in self._manual_controllers.items()
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

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._map_callback,
            map_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_pose_topic,
            self._amcl_pose_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            NavPath,
            self._global_path_topic,
            self._global_path_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            NavPath,
            self._local_path_topic,
            self._local_path_callback,
            10,
            callback_group=self._callback_group,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self._initial_pose_topic,
            10,
        )

        self._flask_app: Flask | None = None
        self._http_server: Any | None = None
        self._http_thread: threading.Thread | None = None
        self._start_http_server()

    def _declare_parameters(self) -> None:
        mission_share = self._package_share_or_empty('mission_manager')
        driving_share = self._package_share_or_empty('vicpinky_task_servers')
        bridge_share = self._package_share_or_empty('arm_can_bridge')
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
        default_nav_points = os.path.join(
            driving_share,
            'config',
            'nav_points.yaml',
        ) if driving_share else ''
        default_bridge_config = os.path.join(
            bridge_share,
            'config',
            'arm_can_bridge.yaml',
        ) if bridge_share else ''

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('auto_port', True)
        self.declare_parameter('port_search_limit', 20)
        self.declare_parameter('request_timeout_sec', 5.0)
        self.declare_parameter('home_service_timeout_sec', 185.0)
        self.declare_parameter('clear_service_timeout_sec', 15.0)
        self.declare_parameter('status_log_limit', 80)
        self.declare_parameter('connection_timeout_sec', 5.0)
        self.declare_parameter('recovered_hold_sec', 5.0)
        self.declare_parameter('heartbeat_topic', '/robot/heartbeat')
        self.declare_parameter('mission_event_topic', '/mission/event_log')
        self.declare_parameter('event_log_db_path', '')
        self.declare_parameter('locations_file', default_locations)
        self.declare_parameter('mission_flow_file', default_flow)
        self.declare_parameter('nav_points_file', default_nav_points)
        self.declare_parameter(
            'arm_bridge_config_file',
            default_bridge_config,
        )
        self.declare_parameter('manual_arm_mode', 'full')
        self.declare_parameter('enable_manual_arm', True)
        self.declare_parameter('enable_manual_gripper', True)
        self.declare_parameter('saved_pose_file', '')
        # Physical motion can finish noticeably later than the trajectory's
        # requested duration on a slow, load-limited robot. Keep a generous
        # result margin while preserving immediate stop/cancel handling.
        self.declare_parameter('sequence_step_timeout_margin_sec', 30.0)
        self.declare_parameter('sequence_step_result_timeout_sec', 120.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('global_path_topic', '/plan')
        self.declare_parameter('local_path_topic', '/local_plan')

    def _manual_controllers_from_parameters(self) -> dict[str, Any]:
        controllers: dict[str, Any] = {}

        if bool(self.get_parameter('enable_manual_arm').value):
            mode = str(self.get_parameter('manual_arm_mode').value).lower()
            arm_config = deepcopy(MANUAL_CONTROLLERS['arm'])
            self._apply_bridge_joint_limits(arm_config, 'arm')

            if mode in ('full', 'all'):
                arm_config['mode'] = 'full'
            elif mode in ('board1', 'board1_only', 'board1-only'):
                arm_config['label'] = 'Board1 Arm'
                arm_config['mode'] = 'board1'
                arm_config['joints'] = [
                    joint
                    for joint in arm_config['joints']
                    if joint['joint_name'] != 'arm_joint_4'
                ]
            elif mode in ('board2', 'board2_only', 'board2-only'):
                arm_config['label'] = 'Board2 Joint4'
                arm_config['mode'] = 'board2'
                arm_config['joints'] = [
                    joint
                    for joint in arm_config['joints']
                    if joint['joint_name'] == 'arm_joint_4'
                ]
            else:
                raise ValueError(
                    'manual_arm_mode must be one of: full, board1, board2'
                )

            controllers['arm'] = arm_config

        if bool(self.get_parameter('enable_manual_gripper').value):
            gripper_config = deepcopy(MANUAL_CONTROLLERS['gripper'])
            self._apply_bridge_joint_limits(gripper_config, 'gripper')
            controllers['gripper'] = gripper_config

        return controllers

    def _apply_bridge_joint_limits(
        self,
        controller: dict[str, Any],
        prefix: str,
    ) -> None:
        config_file = str(
            self.get_parameter('arm_bridge_config_file').value
        )
        if not config_file:
            return
        try:
            data = self._read_yaml(config_file)
            params = data['arm_can_bridge']['ros__parameters']
            names = [str(value) for value in params[f'{prefix}_joint_names']]
            minimums = params[f'{prefix}_min_positions_rad']
            maximums = params[f'{prefix}_max_positions_rad']
            homes = params[f'{prefix}_home_positions_rad']
            if not (
                len(names) == len(minimums)
                == len(maximums) == len(homes)
            ):
                raise ValueError(f'{prefix} bridge arrays have unequal lengths')
            values_by_name = {
                name: (minimums[index], maximums[index], homes[index])
                for index, name in enumerate(names)
            }
            for joint in controller['joints']:
                values = values_by_name.get(str(joint['joint_name']))
                if values is None:
                    raise ValueError(
                        f'bridge config is missing {joint["joint_name"]}'
                    )
                minimum_deg, maximum_deg, home_deg = (
                    round(math.degrees(float(value)), 1)
                    for value in values
                )
                joint['min_deg'] = minimum_deg
                joint['max_deg'] = maximum_deg
                joint['default_deg'] = max(
                    minimum_deg,
                    min(home_deg, maximum_deg),
                )
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to apply {prefix} GUI limits from '
                f'{config_file}: {exc}'
            )

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

        @app.get('/api/driving/map')
        def api_driving_map():
            revision = request.args.get('revision', default=None, type=int)
            payload = self.driving_map_payload(revision)
            status_code = 200 if payload.get('ok') else 404
            return jsonify(payload), status_code

        @app.get('/api/logs')
        def api_logs():
            after_seq = request.args.get('after_seq', default=0, type=int)
            limit = request.args.get('limit', default=500, type=int)
            return jsonify(self.logs_after(after_seq, limit=limit))

        @app.post('/api/arm/<command>')
        def api_arm_command(command: str):
            payload = request.get_json(silent=True)
            if (
                command == 'disable'
                and (
                    not isinstance(payload, dict)
                    or payload.get('confirmed') is not True
                )
            ):
                return jsonify({
                    'ok': False,
                    'command': command,
                    'message': 'Disable requires confirmed=true',
                }), 400
            response = self.call_arm_service(command)
            return jsonify(response), 200 if response.get('ok') else 503

        @app.get('/api/manual/poses')
        def api_saved_poses():
            response = self.saved_poses_snapshot()
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.post('/api/manual/poses')
        def api_create_saved_pose():
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                payload = {}
            response = self.create_saved_pose(payload)
            status_code = 201 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.get('/api/manual/poses/<int:pose_id>')
        def api_get_saved_pose(pose_id: int):
            response = self.get_saved_pose(pose_id)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.patch('/api/manual/poses/<int:pose_id>')
        def api_update_saved_pose(pose_id: int):
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                payload = {}
            response = self.update_saved_pose(pose_id, payload)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.delete('/api/manual/poses/<int:pose_id>')
        def api_delete_saved_pose(pose_id: int):
            response = self.delete_saved_pose(pose_id)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.get('/api/manual/sequence')
        def api_manual_sequence():
            return jsonify({
                'ok': True,
                'sequence': self.manual_sequence_snapshot(),
            })

        @app.post('/api/manual/sequence/start')
        def api_manual_sequence_start():
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                payload = {}
            response = self.start_manual_sequence(payload)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.post('/api/manual/sequence/stop')
        def api_manual_sequence_stop():
            response = self.stop_manual_sequence()
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 409)
            )
            return jsonify(response), status_code

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

        @app.post('/api/nav/go-to')
        def api_nav_go_to():
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                payload = {}
            response = self.start_direct_nav(payload)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

        @app.post('/api/nav/cancel')
        def api_nav_cancel():
            response = self.cancel_direct_nav()
            return jsonify(response), 200 if response.get('ok') else 409

        @app.post('/api/driving/initial-pose')
        def api_driving_initial_pose():
            payload = request.get_json(silent=True)
            if not isinstance(payload, dict):
                payload = {}
            response = self.publish_initial_pose(payload)
            status_code = 200 if response.get('ok') else int(
                response.get('status_code', 400)
            )
            return jsonify(response), status_code

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

        last_error: BaseException | None = None

        for candidate_port in candidate_ports:
            if (
                candidate_port != 0
                and not self._http_port_available(self._host, candidate_port)
            ):
                last_error = OSError(
                    errno.EADDRINUSE,
                    f'HTTP port {candidate_port} is already in use',
                )
                if self._auto_port:
                    self.get_logger().warning(
                        f'HTTP port {candidate_port} is already in use; '
                        'trying the next port'
                    )
                    continue
                raise last_error

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
            except SystemExit as exc:
                last_error = exc
                if self._auto_port:
                    self.get_logger().warning(
                        f'HTTP port {candidate_port} could not be bound; '
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

    @staticmethod
    def _http_port_available(host: str, port: int) -> bool:
        try:
            address_infos = socket.getaddrinfo(
                host,
                port,
                type=socket.SOCK_STREAM,
            )
        except socket.gaierror:
            return False

        for family, socktype, proto, _canonname, sockaddr in address_infos:
            try:
                with socket.socket(family, socktype, proto) as probe:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    probe.bind(sockaddr)
            except OSError as exc:
                if exc.errno == errno.EADDRINUSE:
                    return False
                continue
            return True

        return False

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

    @staticmethod
    def _resolve_event_log_db_path(raw_path: str) -> Path:
        path_text = str(raw_path or '').strip()
        if not path_text:
            path_text = '~/.ros/vicpinky_gui/event_log.sqlite3'
        return Path(os.path.expandvars(path_text)).expanduser()

    @staticmethod
    def _resolve_saved_pose_file(raw_path: str) -> Path:
        path_text = str(raw_path or '').strip()
        if not path_text:
            path_text = '~/.ros/vicpinky_gui/saved_poses.json'
        return Path(os.path.expandvars(path_text)).expanduser()

    @staticmethod
    def _idle_sequence_state() -> dict[str, Any]:
        return {
            'run_id': None,
            'state': 'IDLE',
            'active': False,
            'pose_ids': [],
            'total': 0,
            'total_steps': 0,
            'current_index': None,
            'current_step': None,
            'current_pose_id': None,
            'name': None,
            'current_pose_name': None,
            'completed': 0,
            'completed_count': 0,
            'error': None,
            'cancel_results': {},
            'started_at': None,
            'finished_at': None,
        }

    def _open_event_log_db(self, path: Path) -> sqlite3.Connection | None:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            connection = sqlite3.connect(
                str(path),
                check_same_thread=False,
            )
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS event_log (
                    seq INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    source TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    message TEXT,
                    payload_json TEXT
                )
                """
            )
            connection.execute(
                """
                CREATE INDEX IF NOT EXISTS idx_event_log_source_time
                ON event_log(source, timestamp)
                """
            )
            connection.commit()
            return connection
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to open GUI event log DB {path}: {exc}'
            )
            return None

    def _load_gui_config(self) -> dict[str, Any]:
        locations_file = str(self.get_parameter('locations_file').value)
        mission_flow_file = str(self.get_parameter('mission_flow_file').value)
        nav_points_file = str(self.get_parameter('nav_points_file').value)

        locations: list[dict[str, Any]] = []
        direct_nav_locations: list[dict[str, Any]] = []
        mission_locations: list[dict[str, Any]] = []
        mission_steps: list[dict[str, Any]] = []
        location_data: dict[str, Any] = {}

        try:
            location_data = self._read_yaml(locations_file)
            location_options: dict[str, dict[str, Any]] = {}
            self._add_point_location_options(
                location_options,
                location_data.get('points'),
            )

            raw_locations = location_data.get('locations', {})
            if isinstance(raw_locations, dict):
                for name, value in raw_locations.items():
                    if not isinstance(value, dict):
                        continue

                    if 'pose' not in value and 'point' not in value:
                        continue

                    self._add_location_option(
                        location_options,
                        name=str(name),
                        location=value,
                    )

            locations = list(location_options.values())
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to load GUI locations from {locations_file}: {exc}'
            )

        try:
            nav_point_data = (
                self._read_yaml(nav_points_file)
                if nav_points_file and os.path.exists(nav_points_file)
                else location_data
            )
            direct_nav_locations = self._direct_nav_locations_from_points(
                nav_point_data.get('points'),
            )
            mission_locations = self._mission_locations_from_direct_nav(
                direct_nav_locations,
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to load direct nav points from '
                f'{nav_points_file or locations_file}: {exc}'
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
            'direct_nav_locations': direct_nav_locations,
            'mission_locations': mission_locations,
            'mission_steps': mission_steps,
            'default_goal': {
                'mission_id': self._default_mission_id(),
                'pickup_location': '402',
                'delivery_location': 'object_place',
                'target_floor': 5,
                'object_label': 'object_1',
                'arm_task_name': 'deliver_object_1_from_tray',
            },
            'arm_tasks': deepcopy(ARM_TASK_OPTIONS),
            'default_nav': {
                'location_name': '402',
                'location_id': '4:402',
                'target_floor': 4,
            },
            'arm_commands': list(ARM_COMMANDS.keys()),
            'manual': deepcopy(self._manual_controllers),
        }

    @classmethod
    def _add_point_location_options(
        cls,
        location_options: dict[str, dict[str, Any]],
        points: Any,
    ) -> None:
        if not isinstance(points, dict):
            return

        point_counts: dict[str, int] = {}
        for floor_points in points.values():
            if not isinstance(floor_points, dict):
                continue
            for point_name in floor_points:
                key = str(point_name)
                point_counts[key] = point_counts.get(key, 0) + 1

        for floor_key, floor_points in points.items():
            if not isinstance(floor_points, dict):
                continue

            try:
                floor = int(floor_key)
            except (TypeError, ValueError):
                floor = floor_key

            for point_name, pose in floor_points.items():
                name = str(point_name)
                location = {
                    'type': cls._infer_gui_point_type(name),
                    'floor': floor,
                    'marker_id': -1,
                    'nav_target': name,
                    'pose': pose,
                }

                cls._add_location_option(
                    location_options,
                    name=f'{name}_{floor}f',
                    location=location,
                )

                if point_counts.get(name, 0) == 1:
                    cls._add_location_option(
                        location_options,
                        name=name,
                        location=location,
                    )

                if name.isdigit():
                    cls._add_location_option(
                        location_options,
                        name=f'room_{name}',
                        location=location,
                    )

    @classmethod
    def _direct_nav_locations_from_points(
        cls,
        points: Any,
    ) -> list[dict[str, Any]]:
        if not isinstance(points, dict):
            return []

        locations: list[dict[str, Any]] = []
        for floor_key, floor_points in points.items():
            if not isinstance(floor_points, dict):
                continue

            try:
                floor = int(floor_key)
            except (TypeError, ValueError):
                continue

            for point_name, pose in floor_points.items():
                name = str(point_name)
                location = {
                    'id': f'{floor}:{name}',
                    'name': name,
                    'type': cls._infer_gui_point_type(name),
                    'floor': floor,
                    'marker_id': -1,
                    'nav_target': name,
                }
                if isinstance(pose, dict):
                    location['pose'] = deepcopy(pose)
                locations.append(location)

        return locations

    @staticmethod
    def _mission_locations_from_direct_nav(
        direct_nav_locations: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        point_counts: dict[str, int] = {}
        for location in direct_nav_locations:
            name = str(location.get('name', ''))
            if not name:
                continue
            point_counts[name] = point_counts.get(name, 0) + 1

        mission_locations: list[dict[str, Any]] = []
        for location in direct_nav_locations:
            name = str(location.get('name', ''))
            if not name:
                continue

            floor = location.get('floor', '')
            value = name
            if point_counts.get(name, 0) > 1:
                value = f'{name}_{floor}f'

            mission_locations.append({
                'name': value,
                'label': f'{floor}F {name}',
                'floor': floor,
                'type': location.get('type', ''),
                'nav_target': name,
            })

        return mission_locations

    @staticmethod
    def _add_location_option(
        location_options: dict[str, dict[str, Any]],
        *,
        name: str,
        location: dict[str, Any],
    ) -> None:
        item = {
            'name': name,
            'type': location.get('type', ''),
            'floor': location.get('floor', ''),
            'marker_id': location.get('marker_id', ''),
        }
        if 'nav_target' in location:
            item['nav_target'] = location.get('nav_target', '')
        elif 'point' in location:
            item['nav_target'] = str(location.get('point', ''))

        pose = location.get('pose')
        if isinstance(pose, dict):
            item['pose'] = deepcopy(pose)

        location_options[name] = item

    @staticmethod
    def _infer_gui_point_type(point_name: str) -> str:
        point_types = {
            'dock': 'dock',
            'home': 'home',
            'elevator_front': 'navigation_goal',
            '401': 'delivery_zone',
            '402': 'pickup_zone',
            '402_return_test': 'navigation_goal',
            '501': 'delivery_zone',
            'object_place': 'pickup_zone',
        }
        return point_types.get(point_name, 'navigation_goal')

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

    def _map_callback(self, msg: OccupancyGrid) -> None:
        origin = msg.info.origin
        with self._lock:
            self._driving_map_revision += 1
            self._driving_map = {
                'revision': self._driving_map_revision,
                'topic': self._map_topic,
                'stamp': self._stamp_to_float(msg.header.stamp),
                'frame_id': msg.header.frame_id,
                'width': int(msg.info.width),
                'height': int(msg.info.height),
                'resolution': float(msg.info.resolution),
                'origin': {
                    'x': float(origin.position.x),
                    'y': float(origin.position.y),
                    'z': float(origin.position.z),
                    'yaw': self._quaternion_to_yaw(origin.orientation),
                },
                'data': [int(value) for value in msg.data],
            }
            self._driving_map_seen_at = time.time()

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        yaw = self._quaternion_to_yaw(pose.orientation)
        with self._lock:
            self._amcl_pose = {
                'topic': self._amcl_pose_topic,
                'stamp': self._stamp_to_float(msg.header.stamp),
                'frame_id': msg.header.frame_id,
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z),
                'yaw': yaw,
                'yaw_deg': math.degrees(yaw),
            }
            self._amcl_pose_seen_at = time.time()

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        with self._lock:
            self._odom = {
                'topic': self._odom_topic,
                'stamp': self._stamp_to_float(msg.header.stamp),
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'yaw': self._quaternion_to_yaw(pose.orientation),
                'linear_x': float(twist.linear.x),
                'linear_y': float(twist.linear.y),
                'angular_z': float(twist.angular.z),
            }
            self._odom_seen_at = time.time()

    def _global_path_callback(self, msg: NavPath) -> None:
        with self._lock:
            self._global_path = self._path_to_dict(msg, self._global_path_topic)
            self._global_path_seen_at = time.time()

    def _local_path_callback(self, msg: NavPath) -> None:
        with self._lock:
            self._local_path = self._path_to_dict(msg, self._local_path_topic)
            self._local_path_seen_at = time.time()

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
            self._append_board_fault_events_locked(parsed_status)

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
        event_type, event_type_source = self._mission_event_type(payload)
        if 'seq' in payload:
            payload['source_seq'] = payload['seq']
        payload['event_type'] = event_type
        payload['event_type_source'] = event_type_source

        with self._lock:
            payload['seq'] = self._store_event_log_locked(
                source='mission',
                event_type=event_type,
                message=str(payload.get('message', '')),
                payload=payload,
                timestamp=payload['time'],
            )
            self._mission_event_log.append(payload)

    @staticmethod
    def _mission_event_type(payload: dict[str, Any]) -> tuple[str, str]:
        for key in ('event_type', 'event', 'type'):
            value = str(payload.get(key, '')).strip()
            if value:
                return value, 'reported'

        state = str(payload.get('state', '')).strip()
        level = str(payload.get('level', '')).strip().lower()
        message = str(payload.get('message', '')).strip().lower()

        if state in BUTTON_PRESS_STATES:
            if level == 'success':
                return 'BUTTON_PRESS_SUCCESS', 'inferred'
            if level in ('warning', 'error'):
                return 'BUTTON_PRESS_FAILED', 'inferred'
            if 'attempt' in message or level == 'info':
                return 'BUTTON_PRESS_START', 'inferred'
            return 'BUTTON_PRESS_START', 'inferred'

        if state in ELEVATOR_CALL_BUTTON_STATES:
            if level == 'success':
                return 'ELEVATOR_CALL_BUTTON_DONE', 'inferred'
            if level in ('warning', 'error'):
                return 'ELEVATOR_CALL_BUTTON_FAILED', 'inferred'
            return 'ELEVATOR_CALL_BUTTON_START', 'inferred'

        if state in ELEVATOR_ENTER_STATES:
            if level == 'success':
                return 'ELEVATOR_ENTERED', 'inferred'
            if level in ('warning', 'error'):
                return 'ELEVATOR_ENTER_FAILED', 'inferred'
            return 'ELEVATOR_ENTER_START', 'inferred'

        if state in ELEVATOR_WAIT_FLOOR_STATES:
            if level == 'success':
                return 'TARGET_FLOOR_ARRIVED', 'inferred'
            if level in ('warning', 'error'):
                return 'TARGET_FLOOR_WAIT_FAILED', 'inferred'
            return 'WAITING_TARGET_FLOOR', 'inferred'

        if state in ELEVATOR_EXIT_STATES:
            if level == 'success':
                return 'ELEVATOR_EXIT_DONE', 'inferred'
            if level in ('warning', 'error'):
                return 'ELEVATOR_EXIT_FAILED', 'inferred'
            return 'ELEVATOR_EXIT_START', 'inferred'

        if level == 'error':
            return 'MISSION_ERROR', 'inferred'
        if level == 'warning':
            return 'MISSION_WARNING', 'inferred'
        if state:
            return state, 'inferred'
        return 'MISSION_EVENT', 'inferred'

    def _append_board_fault_events_locked(
        self,
        parsed_status: dict[str, Any] | None,
    ) -> None:
        current_faults = self._board_faults_from_status(parsed_status)

        for key, fault in current_faults.items():
            if key in self._active_board_faults:
                continue
            self._active_board_faults[key] = deepcopy(fault)
            self._append_event_locked(
                kind=fault['source'],
                level='error',
                message=fault['message'],
                event_type=fault['event_type'],
                payload=fault,
            )

        cleared_keys = [
            key
            for key in self._active_board_faults
            if key not in current_faults
        ]
        for key in cleared_keys:
            fault = self._active_board_faults.pop(key)
            clear_type = (
                'GRIPPER_FAULT_CLEARED'
                if fault['event_type'].startswith('GRIPPER_')
                else 'ARM_FAULT_CLEARED'
            )
            self._append_event_locked(
                kind=fault['source'],
                level='info',
                message=f'{fault["label"]} cleared',
                event_type=clear_type,
                payload=fault,
            )

    @classmethod
    def _board_faults_from_status(
        cls,
        parsed_status: dict[str, Any] | None,
    ) -> dict[tuple[str, int], dict[str, Any]]:
        if not isinstance(parsed_status, dict):
            return {}

        faults: dict[tuple[str, int], dict[str, Any]] = {}
        for controller in parsed_status.get('controllers', []):
            if not isinstance(controller, dict):
                continue
            controller_name = str(controller.get('name', 'unknown'))
            controller_key = controller_name.lower()

            for board in controller.get('boards', []):
                if not isinstance(board, dict):
                    continue
                board_id = cls._optional_int(board.get('board_id'))
                if board_id is None:
                    continue

                fields = board.get('fields')
                if not isinstance(fields, dict):
                    fields = {}
                reason = cls._board_fault_reason(fields)
                if not reason:
                    continue

                is_gripper = (
                    'gripper' in controller_key
                    or board_id == 3
                )
                source = 'gripper_board' if is_gripper else 'arm_board'
                event_type = (
                    'GRIPPER_FAULT_DETECTED'
                    if is_gripper
                    else 'ARM_FAULT_DETECTED'
                )
                label = f'{controller_name} board {board_id}'
                faults[(controller_key, board_id)] = {
                    'source': source,
                    'event_type': event_type,
                    'label': label,
                    'controller': controller_name,
                    'board_id': board_id,
                    'reason': reason,
                    'message': f'{label} fault detected: {reason}',
                    'fields': deepcopy(fields),
                }

        return faults

    @staticmethod
    def _board_fault_reason(fields: dict[str, Any]) -> str:
        state = str(fields.get('state', '')).upper()
        if state in ('ERROR', 'ESTOP'):
            return f'state={state}'

        error = str(fields.get('error', '')).upper()
        if error and error != 'NONE':
            return f'error={error}'

        try:
            fault = int(fields.get('fault', 0))
        except (TypeError, ValueError):
            fault = 0
        if fault != 0:
            return f'fault=0x{fault:02X}'

        return ''

    @staticmethod
    def _list_value(values: Any, index: int) -> float | None:
        try:
            return float(values[index])
        except (IndexError, TypeError, ValueError):
            return None

    @staticmethod
    def _quaternion_to_yaw(orientation: Any) -> float:
        siny_cosp = 2.0 * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        )
        cosy_cosp = 1.0 - 2.0 * (
            float(orientation.y) * float(orientation.y)
            + float(orientation.z) * float(orientation.z)
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _assign_yaw_to_quaternion(orientation: Any, yaw: float) -> None:
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = math.sin(yaw / 2.0)
        orientation.w = math.cos(yaw / 2.0)

    @classmethod
    def _path_to_dict(cls, msg: NavPath, topic: str) -> dict[str, Any]:
        poses = []
        for stamped_pose in msg.poses:
            pose = stamped_pose.pose
            poses.append({
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'yaw': cls._quaternion_to_yaw(pose.orientation),
            })

        max_points = 240
        if len(poses) > max_points:
            stride = max(1, math.ceil(len(poses) / max_points))
            poses = poses[::stride]

        return {
            'topic': topic,
            'stamp': cls._stamp_to_float(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'count': len(msg.poses),
            'poses': poses,
        }

    @staticmethod
    def _stamp_to_float(stamp: Any) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    @staticmethod
    def _safe_float(value: Any, default: float | None = None) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @classmethod
    def _extract_fsm_state_from_text(cls, text: Any) -> str:
        candidate = str(text or '').strip()
        if not candidate:
            return ''
        if candidate in ELEVATOR_FSM_INDEX:
            return candidate
        for state in ELEVATOR_FSM_STATES:
            if state in candidate:
                return state
        return ''

    @classmethod
    def _extract_fsm_state_from_payload(
        cls,
        payload: dict[str, Any] | None,
    ) -> str:
        if not isinstance(payload, dict):
            return ''
        for key in (
            'display_state',
            'current_state',
            'state',
            'event_type',
            'message',
            'detail',
            'active_task',
        ):
            state = cls._extract_fsm_state_from_text(payload.get(key))
            if state:
                return state
        return ''

    @classmethod
    def _mission_event_display_state(
        cls,
        event: dict[str, Any],
    ) -> tuple[str, str]:
        state = cls._extract_fsm_state_from_payload(event)
        event_type = str(event.get('event_type') or '').strip()
        level = str(event.get('level') or '').strip().lower()

        if level == 'success':
            next_state = SUCCESS_EVENT_NEXT_STATE_BY_TYPE_AND_STATE.get(
                (event_type, state),
            )
            if next_state:
                return next_state, 'mission_event_success'

            next_state = SUCCESS_EVENT_NEXT_STATE_BY_STATE.get(state)
            if next_state:
                return next_state, 'mission_event_success'

        if event_type in ELEVATOR_FSM_INDEX:
            return event_type, 'mission_event'
        if state:
            return state, 'mission_event'

        raw_state = str(event.get('state') or '').strip()
        return raw_state, 'mission_event'

    @classmethod
    def _latest_relevant_mission_event(
        cls,
        mission_events: list[dict[str, Any]],
        mission_id: str,
    ) -> dict[str, Any] | None:
        for event in reversed(mission_events):
            if not isinstance(event, dict):
                continue
            event_mission_id = str(event.get('mission_id') or '').strip()
            if (
                mission_id
                and event_mission_id
                and event_mission_id != mission_id
            ):
                continue
            return event
        return None

    @classmethod
    def _mission_display_snapshot(
        cls,
        *,
        mission_status: dict[str, Any] | None,
        mission_feedback: dict[str, Any] | None,
        mission_goal: dict[str, Any] | None,
        mission_result: dict[str, Any] | None,
        mission_active: bool,
        mission_events: list[dict[str, Any]],
        status_age_ms: float | None,
    ) -> dict[str, Any]:
        mission_id = str(
            (mission_status or {}).get('mission_id') or ''
        ).strip()
        latest_event = cls._latest_relevant_mission_event(
            mission_events,
            mission_id,
        )
        event_state = ''
        event_source = ''
        if latest_event is not None:
            event_state, event_source = cls._mission_event_display_state(
                latest_event,
            )

        status_state = cls._extract_fsm_state_from_payload(mission_status)
        if not status_state:
            status_state = str(
                (mission_status or {}).get('state') or ''
            ).strip()

        feedback_state = cls._extract_fsm_state_from_payload(mission_feedback)
        goal_state = str((mission_goal or {}).get('state') or '').strip()
        result_state = str((mission_result or {}).get('status') or '').strip()

        selected_state = (
            status_state
            or feedback_state
            or goal_state
            or result_state
            or ('ACTIVE' if mission_active else 'IDLE')
        )
        selected_source = 'mission_status' if status_state else 'local'

        status_index = ELEVATOR_FSM_INDEX.get(status_state, -1)
        event_index = ELEVATOR_FSM_INDEX.get(event_state, -1)
        if event_state:
            if event_index >= 0 and event_index >= status_index:
                selected_state = event_state
                selected_source = event_source
            elif status_index < 0 and not status_state:
                selected_state = event_state
                selected_source = event_source

        active_task = (
            (mission_status or {}).get('active_task')
            or (mission_feedback or {}).get('current_task')
            or (latest_event or {}).get('active_task')
            or ''
        )
        progress = cls._safe_float(
            (mission_status or {}).get('progress'),
            None,
        )
        if progress is None:
            progress = cls._safe_float(
                (mission_feedback or {}).get('progress'),
                1.0 if (mission_result or {}).get('success') else 0.0,
            )

        message = (
            (latest_event or {}).get('message')
            if selected_source.startswith('mission_event')
            else None
        )
        if not message:
            message = (
                (mission_status or {}).get('message')
                or (mission_feedback or {}).get('detail')
                or goal_state
                or (mission_result or {}).get('message')
                or ''
            )

        return {
            'state': selected_state,
            'source': selected_source,
            'raw_status_state': str(
                (mission_status or {}).get('state') or '',
            ),
            'event_state': event_state,
            'event_type': (
                str((latest_event or {}).get('event_type') or '')
                if latest_event is not None
                else ''
            ),
            'event_level': (
                str((latest_event or {}).get('level') or '')
                if latest_event is not None
                else ''
            ),
            'event_seq': (
                (latest_event or {}).get('seq')
                if latest_event is not None
                else None
            ),
            'event_time': (
                (latest_event or {}).get('time')
                if latest_event is not None
                else None
            ),
            'active_task': active_task,
            'progress': progress,
            'message': str(message or ''),
            'status_age_ms': status_age_ms,
        }

    def _map_switch_snapshot_locked(
        self,
        now: float,
        mission_events: list[dict[str, Any]],
        mission_id: str = '',
    ) -> dict[str, Any]:
        request_event = None
        request_state = ''
        for event in reversed(mission_events):
            if not isinstance(event, dict):
                continue
            event_mission_id = str(event.get('mission_id') or '').strip()
            if (
                mission_id
                and event_mission_id
                and event_mission_id != mission_id
            ):
                continue
            event_state = self._extract_fsm_state_from_payload(event)
            if event_state in MAP_SWITCH_STATES:
                request_event = event
                request_state = event_state
                break

        if request_event is None:
            return {
                'state': 'IDLE',
                'target_floor': None,
                'requested_state': '',
                'requested_at': None,
                'applied_at': None,
                'elapsed_s': None,
                'current_map_revision': self._driving_map_revision,
                'current_map_age_ms': self._age_ms(
                    self._driving_map_seen_at,
                    now,
                ),
                'message': 'No map switch requested',
            }

        requested_stamp = self._safe_float(request_event.get('stamp'), None)
        if requested_stamp is None:
            requested_stamp = now

        elapsed_s = max(0.0, now - requested_stamp)
        applied = (
            self._driving_map_seen_at is not None
            and self._driving_map_seen_at >= requested_stamp
        )
        target_floor = MAP_SWITCH_STATES[request_state]

        if applied:
            state = 'APPLIED'
            message = f'{target_floor}F map applied'
        elif elapsed_s >= 5.0:
            state = 'WAITING_MAP'
            message = f'Waiting for {target_floor}F map data'
        else:
            state = 'SWITCHING'
            message = f'Switching to {target_floor}F map'

        return {
            'state': state,
            'target_floor': target_floor,
            'requested_state': request_state,
            'requested_at': self._now_iso(requested_stamp),
            'applied_at': (
                self._now_iso(self._driving_map_seen_at)
                if applied and self._driving_map_seen_at is not None
                else None
            ),
            'elapsed_s': elapsed_s,
            'current_map_revision': self._driving_map_revision,
            'current_map_age_ms': self._age_ms(self._driving_map_seen_at, now),
            'event_seq': request_event.get('seq'),
            'message': message,
        }

    def _driving_snapshot_locked(self, now: float) -> dict[str, Any]:
        if self._driving_map is not None:
            map_snapshot = {
                key: deepcopy(value)
                for key, value in self._driving_map.items()
                if key != 'data'
            }
            map_snapshot['available'] = True
            map_snapshot['age_ms'] = self._age_ms(
                self._driving_map_seen_at,
                now,
            )
            map_snapshot['data_url'] = (
                f'/api/driving/map?revision={map_snapshot["revision"]}'
            )
        else:
            map_snapshot = {
                'available': False,
                'topic': self._map_topic,
                'revision': 0,
                'age_ms': None,
                'data_url': None,
            }

        pose_snapshot = deepcopy(self._amcl_pose)
        if pose_snapshot is not None:
            pose_snapshot['available'] = True
            pose_snapshot['age_ms'] = self._age_ms(
                self._amcl_pose_seen_at,
                now,
            )
        else:
            pose_snapshot = {
                'available': False,
                'topic': self._amcl_pose_topic,
                'age_ms': None,
            }

        odom_snapshot = deepcopy(self._odom)
        if odom_snapshot is not None:
            odom_snapshot['available'] = True
            odom_snapshot['age_ms'] = self._age_ms(self._odom_seen_at, now)
        else:
            odom_snapshot = {
                'available': False,
                'topic': self._odom_topic,
                'age_ms': None,
            }

        global_path = deepcopy(self._global_path)
        if global_path is not None:
            global_path['available'] = True
            global_path['age_ms'] = self._age_ms(
                self._global_path_seen_at,
                now,
            )
        else:
            global_path = {
                'available': False,
                'topic': self._global_path_topic,
                'age_ms': None,
                'poses': [],
            }

        local_path = deepcopy(self._local_path)
        if local_path is not None:
            local_path['available'] = True
            local_path['age_ms'] = self._age_ms(
                self._local_path_seen_at,
                now,
            )
        else:
            local_path = {
                'available': False,
                'topic': self._local_path_topic,
                'age_ms': None,
                'poses': [],
            }

        return {
            'map': map_snapshot,
            'pose': pose_snapshot,
            'odom': odom_snapshot,
            'global_path': global_path,
            'local_path': local_path,
        }

    def driving_map_payload(
        self,
        requested_revision: int | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if self._driving_map is None:
                return {
                    'ok': False,
                    'message': f'No map received on {self._map_topic}',
                }

            if (
                requested_revision is not None
                and requested_revision != self._driving_map_revision
            ):
                return {
                    'ok': False,
                    'message': 'Requested map revision is no longer current',
                    'current_revision': self._driving_map_revision,
                }

            return {
                'ok': True,
                'map': deepcopy(self._driving_map),
            }

    def snapshot(self) -> dict[str, Any]:
        """Return the complete dashboard state as JSON-serializable data."""
        now = time.time()

        with self._lock:
            robot_connection = self._robot_connection_snapshot_locked(now)
            mission_status = deepcopy(self._mission_status)
            mission_feedback = deepcopy(self._mission_feedback)
            mission_goal = deepcopy(self._mission_goal)
            mission_result = deepcopy(self._mission_result)
            nav_goal = deepcopy(self._nav_goal)
            nav_feedback = deepcopy(self._nav_feedback)
            nav_result = deepcopy(self._nav_result)
            arm_status = deepcopy(self._latest_arm_status)
            arm_status_raw = self._latest_arm_status_raw
            arm_log = list(self._arm_status_log)
            last_arm_command = deepcopy(self._last_arm_command)
            joint_state = deepcopy(self._joint_state)
            event_log = list(self._event_log)
            mission_events = list(self._mission_event_log)
            mission_active = self._mission_motion_active_locked()
            nav_active = self._nav_motion_active_locked()
            manual_last_commands = deepcopy(self._manual_last_commands)
            manual_feedback = deepcopy(self._manual_feedback)
            manual_latest_target = (
                self._manual_latest_target_snapshot_locked()
            )
            manual_sequence = deepcopy(self._sequence_state)
            manual_motion_active = self._manual_motion_active_locked()
            manual_active = {
                name: manual_motion_active
                for name in self._manual_controllers
            }
            mission_status_age = self._age_ms(
                self._mission_status_seen_at,
                now,
            )
            arm_status_age = self._age_ms(self._latest_arm_status_seen_at, now)
            joint_state_age = self._age_ms(self._joint_state_seen_at, now)
            latest_log_seq = self._latest_event_seq_locked()
            driving = self._driving_snapshot_locked(now)
            driving['map_switch'] = self._map_switch_snapshot_locked(
                now,
                mission_events,
                str((mission_status or {}).get('mission_id') or '').strip(),
            )

        mission_display = self._mission_display_snapshot(
            mission_status=mission_status,
            mission_feedback=mission_feedback,
            mission_goal=mission_goal,
            mission_result=mission_result,
            mission_active=mission_active,
            mission_events=mission_events,
            status_age_ms=mission_status_age,
        )

        mission_nav_state = self._mission_driving_state(
            mission_status,
            mission_feedback,
            nav_feedback,
            nav_result,
            mission_display,
            driving.get('map_switch'),
        )
        driving['state'] = mission_nav_state

        manual_controllers = {}
        for name, config in self._manual_controllers.items():
            target_diagnostics = manual_latest_target['controllers'][name]
            manual_controllers[name] = {
                'label': config['label'],
                'action_name': config['action_name'],
                'mode': config.get('mode', name),
                'ready': self._manual_action_ready(name),
                'active': manual_active.get(name, False),
                **target_diagnostics,
            }

        elevator_button_task = self._elevator_button_task_snapshot(
            mission_status=mission_status,
            mission_feedback=mission_feedback,
            mission_goal=mission_goal,
            mission_events=mission_events,
            arm_status=arm_status,
        )

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
                'display': mission_display,
            },
            'elevator_button_task': elevator_button_task,
            'direct_nav': {
                'goal': nav_goal,
                'feedback': nav_feedback,
                'result': nav_result,
                'active': nav_active,
                'action_ready': self._nav_action_ready(),
            },
            'driving': driving,
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
                'sequence': manual_sequence,
                'latest_target': manual_latest_target,
            },
            'joints': {
                'state': joint_state,
                'age_ms': joint_state_age,
            },
            'events': event_log,
            'mission_events': mission_events,
            'log_sync': {
                'latest_seq': latest_log_seq,
                'db_enabled': self._event_log_db is not None,
            },
            'config': self._gui_config,
        }

    @staticmethod
    def _mission_driving_state(
        mission_status: dict[str, Any] | None,
        mission_feedback: dict[str, Any] | None,
        nav_feedback: dict[str, Any] | None,
        nav_result: dict[str, Any] | None,
        mission_display: dict[str, Any] | None = None,
        map_switch: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        status_state = str(
            (mission_display or {}).get('state')
            or (mission_status or {}).get('state')
            or ''
        )
        active_task = str(
            (mission_display or {}).get('active_task')
            or (mission_status or {}).get('active_task')
            or ''
        )
        message = str(
            (mission_display or {}).get('message')
            or (mission_status or {}).get('message')
            or ''
        )

        if (map_switch or {}).get('state') in ('SWITCHING', 'WAITING_MAP'):
            state = 'MAP_SWITCHING'
            floor = (map_switch or {}).get('target_floor')
            label = str((map_switch or {}).get('message') or 'Switching map')
        elif 'SWITCH_5F_MAP' in status_state:
            state = 'MAP_SWITCHING'
            label = 'Loading 5F map'
            floor = 5
        elif 'SWITCH_4F_MAP' in status_state:
            state = 'MAP_SWITCHING'
            label = 'Loading 4F map'
            floor = 4
        elif status_state in ('WAIT_5F', 'WAIT_4F'):
            state = 'WAITING_ELEVATOR'
            floor = 5 if status_state == 'WAIT_5F' else 4
            label = f'Waiting for {floor}F arrival'
        elif 'EXIT_ELEVATOR' in status_state:
            state = 'EXITING_ELEVATOR'
            floor = None
            label = 'Exiting elevator'
        elif active_task == '/nav/go_to' or nav_feedback:
            state = 'NAVIGATING'
            floor = None
            label = 'Navigating'
        elif nav_result:
            state = 'NAV_RESULT'
            floor = None
            label = str(nav_result.get('status') or 'Navigation result')
        elif status_state:
            state = status_state
            floor = None
            label = status_state
        else:
            state = 'IDLE'
            floor = None
            label = 'Waiting'

        detail = ''
        for source in (nav_feedback, mission_feedback):
            if not isinstance(source, dict):
                continue
            detail = str(source.get('message') or source.get('status') or '')
            if detail:
                break
        if not detail:
            detail = message

        return {
            'state': state,
            'label': label,
            'floor': floor,
            'mission_state': status_state,
            'raw_mission_state': str(
                (mission_status or {}).get('state') or '',
            ),
            'active_task': active_task,
            'detail': detail,
        }

    def logs_after(
        self,
        after_seq: int | None,
        *,
        limit: int = 500,
    ) -> dict[str, Any]:
        """Return stored event log rows after a GUI-local sequence number."""
        safe_after = max(0, int(after_seq or 0))
        safe_limit = max(1, min(2000, int(limit or 500)))

        with self._lock:
            logs = self._logs_after_locked(safe_after, safe_limit)
            latest_seq = self._latest_event_seq_locked()

        return {
            'ok': True,
            'success': True,
            'after_seq': safe_after,
            'latest_seq': latest_seq,
            'logs': logs,
        }

    def _logs_after_locked(
        self,
        after_seq: int,
        limit: int,
    ) -> list[dict[str, Any]]:
        if self._event_log_db is not None:
            try:
                cursor = self._event_log_db.execute(
                    """
                    SELECT
                        seq,
                        timestamp,
                        source,
                        event_type,
                        message,
                        payload_json
                    FROM event_log
                    WHERE seq > ?
                    ORDER BY seq ASC
                    LIMIT ?
                    """,
                    (after_seq, limit),
                )
                return [
                    self._event_log_row_to_dict(row)
                    for row in cursor.fetchall()
                ]
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to read GUI event log DB: {exc}'
                )

        rows = [
            *self._event_log,
            *self._mission_event_log,
        ]
        return sorted(
            (
                deepcopy(row)
                for row in rows
                if int(row.get('seq') or 0) > after_seq
            ),
            key=lambda row: int(row.get('seq') or 0),
        )[:limit]

    @staticmethod
    def _event_log_row_to_dict(row: tuple[Any, ...]) -> dict[str, Any]:
        seq, timestamp, source, event_type, message, payload_json = row
        payload: dict[str, Any] = {}
        try:
            parsed = json.loads(payload_json or '{}')
            if isinstance(parsed, dict):
                payload = parsed
        except json.JSONDecodeError:
            payload = {}

        payload.update({
            'seq': int(seq),
            'time': timestamp,
            'timestamp': timestamp,
            'source': source,
            'event_type': event_type,
            'message': message or payload.get('message', ''),
        })
        return payload

    def _store_event_log_locked(
        self,
        *,
        source: str,
        event_type: str,
        message: str,
        payload: dict[str, Any] | None = None,
        timestamp: str | None = None,
    ) -> int:
        event_time = timestamp or self._now_iso()
        row_payload = deepcopy(payload) if isinstance(payload, dict) else {}
        row_payload.setdefault('time', event_time)
        row_payload.setdefault('source', source)
        row_payload.setdefault('event_type', event_type)
        row_payload.setdefault('message', message)

        if self._event_log_db is not None:
            try:
                cursor = self._event_log_db.execute(
                    """
                    INSERT INTO event_log
                    (timestamp, source, event_type, message, payload_json)
                    VALUES (?, ?, ?, ?, ?)
                    """,
                    (
                        event_time,
                        source,
                        event_type,
                        message,
                        json.dumps(row_payload, ensure_ascii=False),
                    ),
                )
                self._event_log_db.commit()
                return int(cursor.lastrowid)
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to write GUI event log DB: {exc}'
                )

        self._fallback_event_seq += 1
        return self._fallback_event_seq

    def _latest_event_seq_locked(self) -> int:
        if self._event_log_db is not None:
            try:
                cursor = self._event_log_db.execute(
                    'SELECT COALESCE(MAX(seq), 0) FROM event_log'
                )
                row = cursor.fetchone()
                return int(row[0] if row else 0)
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to read latest GUI event seq: {exc}'
                )

        latest = self._fallback_event_seq
        for row in (*self._event_log, *self._mission_event_log):
            try:
                latest = max(latest, int(row.get('seq') or 0))
            except (TypeError, ValueError):
                continue
        return latest

    @classmethod
    def _elevator_button_task_snapshot(
        cls,
        *,
        mission_status: dict[str, Any] | None,
        mission_feedback: dict[str, Any] | None,
        mission_goal: dict[str, Any] | None,
        mission_events: list[dict[str, Any]],
        arm_status: dict[str, Any] | None,
    ) -> dict[str, Any]:
        fault_flags = cls._fault_flags_from_arm_status(arm_status)
        task = {
            'state': 'IDLE',
            'target_floor': cls._target_floor_from_sources(
                mission_goal,
                mission_status,
                mission_feedback,
            ),
            'button_press_started_at': None,
            'button_press_result': 'NOT_STARTED',
            'button_press_result_source': 'none',
            'physical_button_result': 'UNKNOWN',
            'physical_button_confirmed': False,
            'last_event': None,
            'last_event_time': None,
            'last_message': '',
            'arm_fault': fault_flags['arm_fault'],
            'gripper_fault': fault_flags['gripper_fault'],
            'robot_stopped_in_elevator': False,
            'entered_elevator': False,
            'target_floor_arrived': False,
            'exit_done': False,
        }

        for event in mission_events:
            cls._apply_elevator_event(task, event)

        current_state = str(
            (mission_feedback or {}).get('current_state')
            or (mission_status or {}).get('state')
            or ''
        )
        if (
            current_state in BUTTON_PRESS_STATES
            and task['button_press_result'] in (
                'NOT_STARTED',
                'UNKNOWN',
            )
        ):
            task['state'] = 'PRESSING'
            task['target_floor'] = BUTTON_PRESS_STATES[current_state]
            task['button_press_result'] = 'UNKNOWN'
            task['button_press_result_source'] = 'live_state'
        elif current_state in ELEVATOR_WAIT_FLOOR_STATES:
            task['state'] = 'WAITING_TARGET_FLOOR'
            task['target_floor'] = ELEVATOR_WAIT_FLOOR_STATES[current_state]

        if task['arm_fault'] or task['gripper_fault']:
            task['state'] = 'FAULT'

        return task

    @classmethod
    def _apply_elevator_event(
        cls,
        task: dict[str, Any],
        event: dict[str, Any],
    ) -> None:
        event_type = str(event.get('event_type') or '').strip()
        if not event_type:
            event_type, _source = cls._mission_event_type(event)

        state = str(event.get('state', '')).strip()
        if state in BUTTON_PRESS_STATES:
            task['target_floor'] = BUTTON_PRESS_STATES[state]
        elif state in ELEVATOR_WAIT_FLOOR_STATES:
            task['target_floor'] = ELEVATOR_WAIT_FLOOR_STATES[state]

        tracked_types = {
            'ELEVATOR_ENTERED',
            'ROBOT_STOPPED_IN_ELEVATOR',
            'BUTTON_PRESS_START',
            'ARM_BUTTON_PRESS_POSE_REACHED',
            'GRIPPER_PRESS_START',
            'BUTTON_PRESS_DONE',
            'BUTTON_PRESS_SUCCESS',
            'BUTTON_PRESS_FAILED',
            'WAITING_TARGET_FLOOR',
            'TARGET_FLOOR_ARRIVED',
            'ELEVATOR_EXIT_START',
            'ELEVATOR_EXIT_DONE',
        }
        if event_type in tracked_types:
            task['last_event'] = event_type
            task['last_event_time'] = (
                event.get('time')
                or event.get('timestamp')
            )
            task['last_message'] = str(event.get('message', ''))

        if event_type == 'ELEVATOR_ENTERED':
            task['entered_elevator'] = True
            task['state'] = 'ELEVATOR_ENTERED'
        elif event_type == 'ROBOT_STOPPED_IN_ELEVATOR':
            task['robot_stopped_in_elevator'] = True
            task['state'] = 'ROBOT_STOPPED'
        elif event_type in (
            'BUTTON_PRESS_START',
            'ARM_BUTTON_PRESS_POSE_REACHED',
            'GRIPPER_PRESS_START',
        ):
            task['button_press_started_at'] = (
                task['button_press_started_at']
                or event.get('time')
                or event.get('timestamp')
            )
            task['button_press_result'] = 'UNKNOWN'
            task['button_press_result_source'] = event.get(
                'event_type_source',
                'reported',
            )
            task['state'] = 'PRESSING'
        elif event_type in ('BUTTON_PRESS_DONE', 'BUTTON_PRESS_SUCCESS'):
            task['button_press_result'] = (
                'SUCCESS'
                if event.get('event_type_source') == 'reported'
                else 'ACTION_SUCCESS'
            )
            task['button_press_result_source'] = event.get(
                'event_type_source',
                'reported',
            )
            task['physical_button_confirmed'] = (
                event.get('event_type_source') == 'reported'
                and event_type == 'BUTTON_PRESS_SUCCESS'
            )
            task['physical_button_result'] = (
                'CONFIRMED'
                if task['physical_button_confirmed']
                else 'UNKNOWN'
            )
            task['state'] = 'BUTTON_ACTION_DONE'
        elif event_type == 'BUTTON_PRESS_FAILED':
            task['button_press_result'] = 'FAILED'
            task['button_press_result_source'] = event.get(
                'event_type_source',
                'reported',
            )
            task['physical_button_result'] = 'UNKNOWN'
            task['state'] = 'BUTTON_FAILED'
        elif event_type == 'WAITING_TARGET_FLOOR':
            task['state'] = 'WAITING_TARGET_FLOOR'
        elif event_type == 'TARGET_FLOOR_ARRIVED':
            task['target_floor_arrived'] = True
            task['state'] = 'TARGET_FLOOR_ARRIVED'
        elif event_type == 'ELEVATOR_EXIT_START':
            task['state'] = 'EXITING'
        elif event_type == 'ELEVATOR_EXIT_DONE':
            task['exit_done'] = True
            task['state'] = 'EXIT_DONE'

    @classmethod
    def _target_floor_from_sources(
        cls,
        mission_goal: dict[str, Any] | None,
        mission_status: dict[str, Any] | None,
        mission_feedback: dict[str, Any] | None,
    ) -> int | None:
        for source in (mission_goal, mission_status, mission_feedback):
            if not isinstance(source, dict):
                continue
            floor = cls._optional_int(source.get('target_floor'))
            if floor is not None:
                return floor

        for source in (mission_feedback, mission_status):
            if not isinstance(source, dict):
                continue
            text = ' '.join(
                str(source.get(key, ''))
                for key in ('current_state', 'state', 'message', 'detail')
            )
            match = re.search(r'(?:PRESS_|WAIT_)(\d+)F', text)
            if match:
                return int(match.group(1))

        return None

    @classmethod
    def _fault_flags_from_arm_status(
        cls,
        arm_status: dict[str, Any] | None,
    ) -> dict[str, bool]:
        faults = cls._board_faults_from_status(arm_status)
        return {
            'arm_fault': any(
                fault['event_type'] == 'ARM_FAULT_DETECTED'
                for fault in faults.values()
            ),
            'gripper_fault': any(
                fault['event_type'] == 'GRIPPER_FAULT_DETECTED'
                for fault in faults.values()
            ),
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

    def _pose_store(self) -> SavedPoseStore:
        if self._saved_pose_store is None:
            detail = self._saved_pose_store_error or 'store is unavailable'
            raise SavedPoseStoreError(detail)
        return self._saved_pose_store

    def saved_poses_snapshot(self) -> dict[str, Any]:
        """Return saved poses without depending on ROS board readiness."""
        try:
            snapshot = self._pose_store().snapshot()
        except SavedPoseStoreError as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Failed to read saved poses: {exc}',
            }
        return {
            'ok': True,
            **snapshot,
            'saved_pose_file': str(self._saved_pose_file),
        }

    def get_saved_pose(self, pose_id: int) -> dict[str, Any]:
        """Return one saved manual pose."""
        try:
            pose = self._pose_store().get(pose_id)
        except SavedPoseNotFoundError as exc:
            return {
                'ok': False,
                'status_code': 404,
                'message': str(exc),
            }
        except SavedPoseStoreError as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Failed to read saved pose: {exc}',
            }
        return {'ok': True, 'pose': pose}

    def create_saved_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Validate and persist a pose using canonical ROS joint names."""
        try:
            fields = self._normalize_saved_pose_fields(payload)
            requested_id = (
                self._saved_pose_id(payload['id'])
                if 'id' in payload
                else None
            )
            store = self._pose_store()
            pose = store.create(fields, requested_id)
            next_id = store.snapshot()['next_id']
        except (TypeError, ValueError) as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': str(exc),
            }
        except SavedPoseStoreError as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Failed to save pose: {exc}',
            }

        with self._lock:
            self._append_event_locked(
                kind='manual_pose',
                level='info',
                event_type='MANUAL_POSE_SAVED',
                message=f'Saved manual pose {pose["id"]}: {pose["name"]}',
                payload={
                    'pose_id': pose['id'],
                    'name': pose['name'],
                },
            )
        return {
            'ok': True,
            'pose': pose,
            'next_id': next_id,
        }

    def update_saved_pose(
        self,
        pose_id: int,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        """Update saved-pose metadata and controller angles."""
        allowed = {'id', 'name', 'dwell_sec', 'controllers'}
        unknown = sorted(set(payload) - allowed)
        if unknown:
            return {
                'ok': False,
                'status_code': 400,
                'message': (
                    'Only id, name, dwell_sec and controllers may be updated; '
                    'unexpected: '
                    + ', '.join(unknown)
                ),
            }
        if not payload:
            return {
                'ok': False,
                'status_code': 400,
                'message': (
                    'At least one of id, name, dwell_sec or controllers '
                    'is required'
                ),
            }

        try:
            changes: dict[str, Any] = {}
            new_pose_id = None
            store = self._pose_store()
            current_pose = store.get(pose_id)
            if 'id' in payload:
                new_pose_id = self._saved_pose_id(payload['id'])
            if 'name' in payload:
                changes['name'] = self._saved_pose_name(payload['name'])
            if 'dwell_sec' in payload:
                changes['dwell_sec'] = self._saved_pose_dwell(
                    payload['dwell_sec']
                )
            if 'controllers' in payload:
                normalized = self._normalize_saved_pose_fields(
                    {
                        'name': payload.get('name', current_pose['name']),
                        'dwell_sec': payload.get(
                            'dwell_sec', current_pose['dwell_sec']
                        ),
                        'controllers': payload['controllers'],
                    },
                    require_canonical_positions=True,
                )
                changes['controllers'] = normalized['controllers']
            pose = store.update(pose_id, changes, new_pose_id)
            next_id = store.snapshot()['next_id']
        except (TypeError, ValueError) as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': str(exc),
            }
        except SavedPoseNotFoundError as exc:
            return {
                'ok': False,
                'status_code': 404,
                'message': str(exc),
            }
        except SavedPoseStoreError as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Failed to update saved pose: {exc}',
            }

        with self._lock:
            self._append_event_locked(
                kind='manual_pose',
                level='info',
                event_type='MANUAL_POSE_UPDATED',
                message=f'Updated manual pose {pose["id"]}: {pose["name"]}',
                payload={
                    'pose_id': pose['id'],
                    'name': pose['name'],
                },
            )
        return {'ok': True, 'pose': pose, 'next_id': next_id}

    def delete_saved_pose(self, pose_id: int) -> dict[str, Any]:
        """Delete one pose and expose the newly available automatic ID."""
        try:
            store = self._pose_store()
            pose = store.delete(pose_id)
            next_id = store.snapshot()['next_id']
        except SavedPoseNotFoundError as exc:
            return {
                'ok': False,
                'status_code': 404,
                'message': str(exc),
            }
        except SavedPoseStoreError as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Failed to delete saved pose: {exc}',
            }

        with self._lock:
            self._append_event_locked(
                kind='manual_pose',
                level='info',
                event_type='MANUAL_POSE_DELETED',
                message=f'Deleted manual pose {pose["id"]}: {pose["name"]}',
                payload={
                    'pose_id': pose['id'],
                    'name': pose['name'],
                },
            )
        return {'ok': True, 'pose': pose, 'next_id': next_id}

    def _normalize_saved_pose_fields(
        self,
        payload: dict[str, Any],
        *,
        require_canonical_positions: bool = False,
    ) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError('Pose payload must be an object')

        name = self._saved_pose_name(payload.get('name'))
        dwell_sec = self._saved_pose_dwell(payload.get('dwell_sec', 0.0))
        raw_controllers = payload.get('controllers')
        if not isinstance(raw_controllers, dict):
            raise ValueError('controllers must be an object')
        if not all(isinstance(name, str) for name in raw_controllers):
            raise ValueError('controller names must be strings')

        configured = set(self._manual_controllers)
        supplied = set(raw_controllers)
        if supplied != configured:
            missing = sorted(configured - supplied)
            unexpected = sorted(supplied - configured)
            detail = []
            if missing:
                detail.append('missing ' + ', '.join(missing))
            if unexpected:
                detail.append('unexpected ' + ', '.join(unexpected))
            raise ValueError(
                'controllers must exactly match the current configuration'
                + (': ' + '; '.join(detail) if detail else '')
            )
        if not configured:
            raise ValueError('No manual controllers are configured')

        controllers: dict[str, Any] = {}
        for controller in sorted(configured):
            raw_controller = raw_controllers.get(controller)
            if not isinstance(raw_controller, dict):
                raise ValueError(f'{controller} controller must be an object')
            config = self._manual_controllers[controller]
            joints = list(config['joints'])
            raw_positions = raw_controller.get('positions_deg')
            if require_canonical_positions:
                expected_names = {
                    str(joint['joint_name']) for joint in joints
                }
                if not isinstance(raw_positions, dict):
                    raise ValueError(
                        f'{controller}.positions_deg must be an object'
                    )
                supplied_names = set(raw_positions)
                if supplied_names != expected_names:
                    raise ValueError(
                        f'{controller} saved joint configuration does not '
                        'match the current controller'
                    )

            positions = self._manual_positions_from_payload(
                raw_positions,
                joints,
            )
            duration_sec = self._manual_duration_from_payload(
                raw_controller.get('duration_sec'),
                float(config['default_duration_sec']),
            )
            normalized: dict[str, Any] = {
                'positions_deg': {
                    str(joint['joint_name']): positions[index]
                    for index, joint in enumerate(joints)
                },
                'duration_sec': duration_sec,
            }
            if controller == 'gripper':
                normalized['target_load_raw'] = (
                    self._manual_target_load_from_payload(
                        raw_controller.get(
                            'target_load_raw',
                            config.get('default_target_load_raw'),
                        )
                    )
                )
            controllers[controller] = normalized

        return {
            'name': name,
            'dwell_sec': dwell_sec,
            'controllers': controllers,
        }

    @staticmethod
    def _saved_pose_name(raw_name: Any) -> str:
        if not isinstance(raw_name, str):
            raise ValueError('name must be a string')
        name = raw_name.strip()
        if not name:
            raise ValueError('name is required')
        if len(name) > 100:
            raise ValueError('name must be at most 100 characters')
        return name

    @staticmethod
    def _saved_pose_id(raw_id: Any) -> int:
        if (
            isinstance(raw_id, bool)
            or not isinstance(raw_id, int)
            or raw_id <= 0
        ):
            raise ValueError('id must be a positive integer')
        return raw_id

    @staticmethod
    def _saved_pose_dwell(raw_dwell: Any) -> float:
        if isinstance(raw_dwell, bool):
            raise ValueError('dwell_sec must be a finite number')
        try:
            dwell_sec = float(raw_dwell)
        except (TypeError, ValueError) as exc:
            raise ValueError('dwell_sec must be a finite number') from exc
        if not math.isfinite(dwell_sec):
            raise ValueError('dwell_sec must be a finite number')
        if dwell_sec < 0.0:
            raise ValueError('dwell_sec must be greater than or equal to 0')
        return dwell_sec

    def manual_sequence_snapshot(self) -> dict[str, Any]:
        """Return a detached snapshot of the in-memory sequence state."""
        with self._lock:
            return deepcopy(self._sequence_state)

    def manual_latest_target_snapshot(self) -> dict[str, Any]:
        """Return latest-target-wins queue and cancel diagnostics."""
        with self._lock:
            return self._manual_latest_target_snapshot_locked()

    def _ensure_manual_latest_target_state_locked(self) -> None:
        """Initialize queue state for nodes built without ``__init__``."""
        controller_names = tuple(self._manual_controllers)
        if not hasattr(self, '_manual_pending_latest_targets'):
            self._manual_pending_latest_targets = {
                name: None for name in controller_names
            }
        if not hasattr(self, '_manual_dispatching_latest_targets'):
            self._manual_dispatching_latest_targets = {
                name: None for name in controller_names
            }
        if not hasattr(self, '_manual_cancel_requested'):
            self._manual_cancel_requested = {
                name: None for name in controller_names
            }
        for name in controller_names:
            self._manual_pending_latest_targets.setdefault(name, None)
            self._manual_dispatching_latest_targets.setdefault(name, None)
            self._manual_cancel_requested.setdefault(name, None)

    @staticmethod
    def _manual_pending_diagnostic(
        entry: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        if entry is None:
            return None
        return {
            'target': deepcopy(entry['summary']),
            'queued_at': entry['queued_at'],
            'request_id': entry['request_id'],
        }

    def _manual_latest_target_snapshot_locked(self) -> dict[str, Any]:
        self._ensure_manual_latest_target_state_locked()
        controllers: dict[str, Any] = {}
        for name in self._manual_controllers:
            pending = self._manual_pending_latest_targets.get(name)
            dispatching = self._manual_dispatching_latest_targets.get(name)
            controllers[name] = {
                'pending_latest_target': self._manual_pending_diagnostic(
                    pending if pending is not None else dispatching
                ),
                'cancel_requested': (
                    self._manual_cancel_requested.get(name) is not None
                ),
                'cancel_command_id': self._manual_cancel_requested.get(name),
                'dispatching_latest_target': dispatching is not None,
            }
        return {'controllers': controllers}

    def _sequence_active_locked(self) -> bool:
        return self._sequence_state.get('state') in SEQUENCE_ACTIVE_STATES

    def _manual_motion_active_locked(self) -> bool:
        self._ensure_manual_latest_target_state_locked()
        for controller, handle in self._manual_goal_handles.items():
            if handle is not None:
                return True
            state = (
                self._manual_last_commands.get(controller) or {}
            ).get('state')
            if state in ACTION_BLOCKING_STATES:
                return True
            if self._manual_pending_latest_targets.get(controller) is not None:
                return True
            if (
                self._manual_dispatching_latest_targets.get(controller)
                is not None
            ):
                return True
        return False

    def _mission_motion_active_locked(self) -> bool:
        return (
            self._mission_goal_handle is not None
            or (self._mission_goal or {}).get('state')
            in ACTION_BLOCKING_STATES
        )

    def _nav_motion_active_locked(self) -> bool:
        return (
            self._nav_goal_handle is not None
            or (self._nav_goal or {}).get('state')
            in ACTION_BLOCKING_STATES
        )

    def start_manual_sequence(
        self,
        payload: dict[str, Any],
    ) -> dict[str, Any]:
        """Preflight and start a saved-pose sequence in requested order."""
        with self._lock:
            conflict = self._manual_sequence_conflict_locked()
        if conflict is not None:
            return {
                'ok': False,
                'status_code': 409,
                'message': conflict,
            }

        try:
            store_snapshot = self._pose_store().snapshot()
        except SavedPoseStoreError as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Failed to read saved poses: {exc}',
            }

        poses_by_id = {
            int(pose['id']): pose
            for pose in store_snapshot.get('poses', [])
            if isinstance(pose, dict) and 'id' in pose
        }
        if 'pose_ids' not in payload:
            pose_ids = sorted(poses_by_id)
        else:
            raw_pose_ids = payload.get('pose_ids')
            if not isinstance(raw_pose_ids, list):
                return {
                    'ok': False,
                    'status_code': 400,
                    'message': 'pose_ids must be an array of positive integers',
                }
            pose_ids = []
            seen_ids: set[int] = set()
            for raw_pose_id in raw_pose_ids:
                if (
                    isinstance(raw_pose_id, bool)
                    or not isinstance(raw_pose_id, int)
                    or raw_pose_id <= 0
                ):
                    return {
                        'ok': False,
                        'status_code': 400,
                        'message': (
                            'pose_ids must contain only positive integers'
                        ),
                    }
                if raw_pose_id in seen_ids:
                    return {
                        'ok': False,
                        'status_code': 400,
                        'message': f'Duplicate pose ID: {raw_pose_id}',
                    }
                seen_ids.add(raw_pose_id)
                pose_ids.append(raw_pose_id)

        if not pose_ids:
            return {
                'ok': False,
                'status_code': 400,
                'message': 'At least one saved pose is required',
            }

        missing_ids = [pose_id for pose_id in pose_ids if pose_id not in poses_by_id]
        if missing_ids:
            return {
                'ok': False,
                'status_code': 404,
                'message': (
                    'Saved pose IDs were not found: '
                    + ', '.join(str(value) for value in missing_ids)
                ),
            }

        sequence_poses: list[dict[str, Any]] = []
        try:
            for pose_id in pose_ids:
                stored_pose = poses_by_id[pose_id]
                normalized = self._normalize_saved_pose_fields(
                    stored_pose,
                    require_canonical_positions=True,
                )
                sequence_poses.append({
                    **normalized,
                    'id': pose_id,
                    'created_at': stored_pose.get('created_at'),
                    'updated_at': stored_pose.get('updated_at'),
                })
        except (TypeError, ValueError) as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Saved pose preflight failed: {exc}',
            }

        offline = []
        for controller, config in self._manual_controllers.items():
            client = self._manual_clients.get(controller)
            if client is not None and not self._manual_action_ready(controller):
                try:
                    client.wait_for_server(timeout_sec=0.25)
                except Exception:
                    pass
            if not self._manual_action_ready(controller):
                offline.append({
                    'controller': controller,
                    'action_name': config['action_name'],
                })
        if offline:
            return {
                'ok': False,
                'status_code': 503,
                'message': 'Required manual action servers are not ready',
                'offline_controllers': offline,
            }

        run_id = f'manual-sequence-{time.time_ns()}'
        stop_event = threading.Event()
        with self._lock:
            conflict = self._manual_sequence_conflict_locked()
            if conflict is not None:
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': conflict,
                }
            self._sequence_stop_event = stop_event
            self._sequence_cancel_requested = set()
            self._sequence_state = {
                'run_id': run_id,
                'state': 'STARTING',
                'active': True,
                'pose_ids': list(pose_ids),
                'total': len(sequence_poses),
                'total_steps': len(sequence_poses),
                'current_index': None,
                'current_step': None,
                'current_pose_id': None,
                'name': None,
                'current_pose_name': None,
                'completed': 0,
                'completed_count': 0,
                'error': None,
                'cancel_results': {},
                'started_at': self._now_iso(),
                'finished_at': None,
            }
            worker = threading.Thread(
                target=self._run_manual_sequence_safe,
                args=(run_id, deepcopy(sequence_poses), stop_event),
                name=f'vicpinky_{run_id}',
                daemon=True,
            )
            self._sequence_thread = worker
            self._append_event_locked(
                kind='manual_sequence',
                level='info',
                event_type='MANUAL_SEQUENCE_START',
                message=(
                    f'Starting manual sequence {run_id} with '
                    f'{len(sequence_poses)} poses'
                ),
                payload={
                    'run_id': run_id,
                    'pose_ids': list(pose_ids),
                    'total': len(sequence_poses),
                },
            )

        try:
            worker.start()
        except Exception as exc:
            with self._lock:
                self._sequence_state['state'] = 'FAILED'
                self._sequence_state['active'] = False
                self._sequence_state['error'] = str(exc)
                self._sequence_state['finished_at'] = self._now_iso()
                self._sequence_thread = None
                self._append_event_locked(
                    kind='manual_sequence',
                    level='error',
                    event_type='MANUAL_SEQUENCE_FAILED',
                    message=f'Failed to start manual sequence: {exc}',
                    payload={'run_id': run_id, 'error': str(exc)},
                )
            return {
                'ok': False,
                'status_code': 500,
                'message': f'Failed to start sequence worker: {exc}',
            }

        return {
            'ok': True,
            'message': 'Manual sequence started',
            'sequence': self.manual_sequence_snapshot(),
        }

    def _run_manual_sequence_safe(
        self,
        run_id: str,
        poses: list[dict[str, Any]],
        stop_event: threading.Event,
    ) -> None:
        try:
            self._run_manual_sequence(run_id, poses, stop_event)
        except Exception as exc:
            stop_event.set()
            try:
                self._cancel_active_sequence_goals(run_id)
            except Exception as cancel_exc:
                self.get_logger().error(
                    f'Sequence {run_id} emergency cancel failed: '
                    f'{cancel_exc}'
                )
            with self._lock:
                if self._sequence_state.get('run_id') != run_id:
                    return
                self._sequence_state['state'] = 'FAILED'
                self._sequence_state['active'] = False
                self._sequence_state['error'] = (
                    f'Unhandled sequence worker error: {exc}'
                )
                self._sequence_state['finished_at'] = self._now_iso()
                self._sequence_thread = None
                self._append_event_locked(
                    kind='manual_sequence',
                    level='error',
                    event_type='MANUAL_SEQUENCE_FAILED',
                    message=(
                        f'Manual sequence {run_id} worker failed: {exc}'
                    ),
                    payload={
                        'run_id': run_id,
                        'error': str(exc),
                    },
                )

    def _manual_sequence_conflict_locked(self) -> str | None:
        if self._sequence_active_locked():
            return 'A manual sequence is already active'
        if self._mission_motion_active_locked():
            return 'Manual sequence is blocked while a mission is active'
        if self._nav_motion_active_locked():
            return 'Manual sequence is blocked while direct navigation is active'
        active_manual = [
            controller
            for controller, handle in self._manual_goal_handles.items()
            if (
                handle is not None
                or (
                    self._manual_last_commands.get(controller) or {}
                ).get('state') in ACTION_BLOCKING_STATES
            )
        ]
        if active_manual:
            return (
                'Manual sequence is blocked while manual goals are active: '
                + ', '.join(sorted(active_manual))
            )
        return None

    def stop_manual_sequence(self) -> dict[str, Any]:
        """Request immediate cancellation of the active sequence."""
        with self._lock:
            if not self._sequence_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'No active manual sequence to stop',
                    'sequence': deepcopy(self._sequence_state),
                }
            run_id = str(self._sequence_state['run_id'])
            self._sequence_state['state'] = 'STOPPING'
            self._sequence_stop_event.set()
            self._append_event_locked(
                kind='manual_sequence',
                level='info',
                event_type='MANUAL_SEQUENCE_STOP_REQUESTED',
                message=f'Stop requested for manual sequence {run_id}',
                payload={'run_id': run_id},
            )

        cancel_results = self._cancel_active_sequence_goals(run_id)
        return {
            'ok': True,
            'message': 'Manual sequence stop requested',
            'cancel_results': cancel_results,
            'sequence': self.manual_sequence_snapshot(),
        }

    def _run_manual_sequence(
        self,
        run_id: str,
        poses: list[dict[str, Any]],
        stop_event: threading.Event,
    ) -> None:
        terminal_state = 'COMPLETED'
        terminal_error: str | None = None

        with self._lock:
            if self._sequence_state.get('run_id') != run_id:
                return
            self._sequence_state['state'] = 'RUNNING'

        for index, pose in enumerate(poses, start=1):
            if stop_event.is_set():
                terminal_state = 'CANCELED'
                break

            with self._lock:
                if self._sequence_state.get('run_id') != run_id:
                    return
                self._sequence_state['current_index'] = index
                self._sequence_state['current_step'] = index
                self._sequence_state['current_pose_id'] = pose['id']
                self._sequence_state['name'] = pose['name']
                self._sequence_state['current_pose_name'] = pose['name']
                self._append_event_locked(
                    kind='manual_sequence',
                    level='info',
                    event_type='MANUAL_SEQUENCE_STEP_START',
                    message=(
                        f'Sequence {run_id} step {index}/{len(poses)}: '
                        f'pose {pose["id"]} ({pose["name"]})'
                    ),
                    payload={
                        'run_id': run_id,
                        'step_index': index,
                        'total': len(poses),
                        'pose_id': pose['id'],
                        'name': pose['name'],
                    },
                )

            step_state, step_error = self._execute_sequence_step(
                run_id,
                pose,
                stop_event,
            )
            if step_state != 'SUCCEEDED':
                terminal_state = step_state
                terminal_error = step_error
                break

            dwell_sec = float(pose['dwell_sec'])
            with self._lock:
                self._append_event_locked(
                    kind='manual_sequence',
                    level='info',
                    event_type='MANUAL_SEQUENCE_DWELL_START',
                    message=(
                        f'Sequence {run_id} pose {pose["id"]} dwell '
                        f'{dwell_sec:.3f}s'
                    ),
                    payload={
                        'run_id': run_id,
                        'pose_id': pose['id'],
                        'dwell_sec': dwell_sec,
                    },
                )

            if self._wait_sequence_dwell(stop_event, dwell_sec):
                terminal_state = 'CANCELED'
                break

            with self._lock:
                self._sequence_state['completed'] = index
                self._sequence_state['completed_count'] = index
                self._append_event_locked(
                    kind='manual_sequence',
                    level='info',
                    event_type='MANUAL_SEQUENCE_DWELL_COMPLETE',
                    message=(
                        f'Sequence {run_id} pose {pose["id"]} dwell complete'
                    ),
                    payload={
                        'run_id': run_id,
                        'pose_id': pose['id'],
                        'completed': index,
                    },
                )

        if stop_event.is_set() and terminal_state == 'COMPLETED':
            terminal_state = 'CANCELED'

        if terminal_state == 'CANCELED':
            cancel_results = self._cancel_active_sequence_goals(run_id)
            if not self._sequence_cancellation_clean(cancel_results):
                terminal_state = 'FAILED'
                terminal_error = 'One or more active goals could not be canceled'

        with self._lock:
            if self._sequence_state.get('run_id') != run_id:
                return
            self._sequence_state['state'] = terminal_state
            self._sequence_state['active'] = False
            self._sequence_state['error'] = terminal_error
            self._sequence_state['finished_at'] = self._now_iso()
            self._sequence_thread = None
            event_type = f'MANUAL_SEQUENCE_{terminal_state}'
            level = 'info' if terminal_state in {'COMPLETED', 'CANCELED'} else 'error'
            message = f'Manual sequence {run_id} {terminal_state.lower()}'
            if terminal_error:
                message += f': {terminal_error}'
            self._append_event_locked(
                kind='manual_sequence',
                level=level,
                event_type=event_type,
                message=message,
                payload={
                    'run_id': run_id,
                    'state': terminal_state,
                    'completed': self._sequence_state['completed'],
                    'total': self._sequence_state['total'],
                    'error': terminal_error,
                    'cancel_results': deepcopy(
                        self._sequence_state['cancel_results']
                    ),
                },
            )

    @staticmethod
    def _wait_sequence_dwell(
        stop_event: threading.Event,
        dwell_sec: float,
    ) -> bool:
        deadline = time.monotonic() + dwell_sec
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            if stop_event.wait(timeout=min(remaining, 1.0)):
                return True

    def _execute_sequence_step(
        self,
        run_id: str,
        pose: dict[str, Any],
        stop_event: threading.Event,
    ) -> tuple[str, str | None]:
        send_futures: dict[str, Any] = {}
        summaries: dict[str, dict[str, Any]] = {}
        for controller, controller_payload in pose['controllers'].items():
            try:
                goal_msg, summary = self._manual_goal_from_payload(
                    controller,
                    controller_payload,
                )
                send_futures[controller] = self._manual_clients[
                    controller
                ].send_goal_async(
                    goal_msg,
                    feedback_callback=lambda msg, name=controller: (
                        self._manual_feedback_callback(name, msg)
                    ),
                )
                summaries[controller] = summary
            except Exception as exc:
                for pending_controller, future in send_futures.items():
                    self._attach_late_sequence_cancel(
                        run_id,
                        pending_controller,
                        future,
                    )
                return 'FAILED', f'Failed to send {controller} goal: {exc}'

            with self._lock:
                self._manual_last_commands[controller] = {
                    **summary,
                    'state': 'SENDING',
                    'sent_at': self._now_iso(),
                    'run_id': run_id,
                    'pose_id': pose['id'],
                }
                self._manual_feedback[controller] = None
                self._append_event_locked(
                    kind='manual_sequence',
                    level='info',
                    event_type='MANUAL_SEQUENCE_CONTROLLER_SENT',
                    message=(
                        f'Sequence {run_id} sent {controller} for '
                        f'pose {pose["id"]}'
                    ),
                    payload={
                        'run_id': run_id,
                        'pose_id': pose['id'],
                        'controller': controller,
                        'duration_sec': summary['duration_sec'],
                    },
                )

        accepted: dict[str, Any] = {}
        pending = set(send_futures)
        wake = threading.Event()
        for future in send_futures.values():
            future.add_done_callback(lambda _future: wake.set())
        deadline = time.monotonic() + self._request_timeout_s

        while pending:
            for controller in list(pending):
                future = send_futures[controller]
                if not future.done():
                    continue
                pending.remove(controller)
                try:
                    goal_handle = future.result()
                except Exception as exc:
                    self._attach_pending_sequence_cancels(
                        run_id,
                        pending,
                        send_futures,
                    )
                    unresolved = self._cancel_and_drain_sequence_handles(
                        run_id,
                        int(pose['id']),
                        accepted,
                    )
                    message = f'{controller} goal send failed: {exc}'
                    if unresolved:
                        message += (
                            '; canceled peers have no terminal result: '
                            + ', '.join(sorted(unresolved))
                        )
                    return 'FAILED', message
                if goal_handle is None or not goal_handle.accepted:
                    with self._lock:
                        self._manual_last_commands[controller]['state'] = (
                            'REJECTED'
                        )
                        self._append_event_locked(
                            kind='manual_sequence',
                            level='error',
                            event_type='MANUAL_SEQUENCE_CONTROLLER_REJECTED',
                            message=(
                                f'Sequence {run_id} {controller} goal rejected'
                            ),
                            payload={
                                'run_id': run_id,
                                'pose_id': pose['id'],
                                'controller': controller,
                            },
                        )
                    self._attach_pending_sequence_cancels(
                        run_id,
                        pending,
                        send_futures,
                    )
                    unresolved = self._cancel_and_drain_sequence_handles(
                        run_id,
                        int(pose['id']),
                        accepted,
                    )
                    message = f'{controller} goal was rejected'
                    if unresolved:
                        message += (
                            '; canceled peers have no terminal result: '
                            + ', '.join(sorted(unresolved))
                        )
                    return 'FAILED', message

                accepted[controller] = goal_handle
                with self._lock:
                    self._manual_goal_handles[controller] = goal_handle
                    self._manual_last_commands[controller]['state'] = 'ACCEPTED'
                    self._manual_last_commands[controller]['accepted_at'] = (
                        self._now_iso()
                    )
                    self._append_event_locked(
                        kind='manual_sequence',
                        level='info',
                        event_type='MANUAL_SEQUENCE_CONTROLLER_ACCEPTED',
                        message=(
                            f'Sequence {run_id} {controller} goal accepted'
                        ),
                        payload={
                            'run_id': run_id,
                            'pose_id': pose['id'],
                            'controller': controller,
                        },
                    )

            if not pending:
                break
            if time.monotonic() >= deadline:
                self._attach_pending_sequence_cancels(
                    run_id,
                    pending,
                    send_futures,
                )
                unresolved = self._cancel_and_drain_sequence_handles(
                    run_id,
                    int(pose['id']),
                    accepted,
                )
                suffix = (
                    '; canceled peers have no terminal result: '
                    + ', '.join(sorted(unresolved))
                    if unresolved
                    else ''
                )
                if stop_event.is_set():
                    return 'FAILED', (
                        'Timed out while canceling pending goal sends' + suffix
                    )
                return 'FAILED', 'Timed out while sending sequence goals' + suffix
            if stop_event.is_set():
                self._cancel_active_sequence_goals(run_id)
            wake.wait(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
            wake.clear()

        result_futures: dict[str, Any] = {}
        result_setup_errors: list[tuple[str, Exception]] = []
        for controller, handle in accepted.items():
            try:
                result_futures[controller] = handle.get_result_async()
            except Exception as exc:
                result_setup_errors.append((controller, exc))

        if result_setup_errors:
            unresolved = self._cancel_and_drain_sequence_handles(
                run_id,
                int(pose['id']),
                accepted,
                result_futures=result_futures,
            )
            error_detail = '; '.join(
                f'{controller}: {exc}'
                for controller, exc in result_setup_errors
            )
            message = f'Failed to observe sequence results: {error_detail}'
            if unresolved:
                message += (
                    '; handles without a terminal result: '
                    + ', '.join(sorted(unresolved))
                )
            return 'FAILED', message

        max_duration = max(
            float(summary['duration_sec']) for summary in summaries.values()
        )
        return self._wait_for_sequence_results(
            run_id,
            pose,
            accepted,
            result_futures,
            stop_event,
            max(
                self._sequence_step_result_timeout_s,
                max_duration + self._sequence_step_timeout_margin_s,
            ),
        )

    def _wait_for_sequence_results(
        self,
        run_id: str,
        pose: dict[str, Any],
        handles: dict[str, Any],
        result_futures: dict[str, Any],
        stop_event: threading.Event,
        timeout_sec: float,
    ) -> tuple[str, str | None]:
        pending = set(result_futures)
        results: dict[str, dict[str, Any]] = {}
        wake = threading.Event()
        for future in result_futures.values():
            future.add_done_callback(lambda _future: wake.set())
        deadline = time.monotonic() + max(0.1, timeout_sec)
        cancellation_deadline: float | None = None
        failure_error: str | None = None

        while pending:
            for controller in list(pending):
                future = result_futures[controller]
                if not future.done():
                    continue
                pending.remove(controller)
                result_payload = self._sequence_result_payload(
                    future,
                    controller,
                )
                results[controller] = result_payload
                self._record_sequence_controller_result(
                    run_id,
                    int(pose['id']),
                    controller,
                    handles[controller],
                    result_payload,
                )

                if (
                    not result_payload['ok']
                    and not stop_event.is_set()
                    and failure_error is None
                ):
                    failure_error = (
                        f'{controller} {result_payload["status"]}: '
                        f'{result_payload["error_string"]}'
                    )
                    self._cancel_active_sequence_goals(run_id)
                    cancellation_deadline = (
                        time.monotonic() + self._request_timeout_s
                    )

            if not pending:
                break

            if stop_event.is_set():
                self._cancel_active_sequence_goals(run_id)
                if cancellation_deadline is None:
                    cancellation_deadline = (
                        time.monotonic() + self._request_timeout_s
                    )
            elif failure_error is None and time.monotonic() >= deadline:
                failure_error = (
                    f'Sequence step result timed out after '
                    f'{max(0.1, timeout_sec):.1f}s'
                )
                self._cancel_active_sequence_goals(run_id)
                cancellation_deadline = (
                    time.monotonic() + self._request_timeout_s
                )

            active_deadline = (
                cancellation_deadline
                if cancellation_deadline is not None
                else deadline
            )

            if time.monotonic() >= active_deadline:
                self._attach_sequence_result_cleanups(
                    run_id,
                    int(pose['id']),
                    pending,
                    handles,
                    result_futures,
                )
                unresolved = ', '.join(sorted(pending))
                if stop_event.is_set():
                    return 'FAILED', (
                        'Timed out waiting for canceled goals to reach a '
                        f'terminal result: {unresolved}'
                    )
                if failure_error:
                    return 'FAILED', (
                        f'{failure_error}; canceled peers did not reach a '
                        f'terminal result: {unresolved}'
                    )
                return 'FAILED', (
                    'Sequence goals did not reach a terminal result: '
                    + unresolved
                )

            wake.wait(
                timeout=min(
                    0.05,
                    max(0.0, active_deadline - time.monotonic()),
                )
            )
            wake.clear()

        if stop_event.is_set():
            cancel_results = self._cancel_active_sequence_goals(run_id)
            failed_cancels = {
                controller
                for controller, result in cancel_results.items()
                if not result.get('ok')
                and not results.get(controller, {}).get('ok')
                and results.get(controller, {}).get('status') != 'CANCELED'
            }
            if failed_cancels:
                return 'FAILED', (
                    'Cancellation failed for: '
                    + ', '.join(sorted(failed_cancels))
                )
            return 'CANCELED', None

        if failure_error is not None:
            return 'FAILED', failure_error
        return 'SUCCEEDED', None

    def _record_sequence_controller_result(
        self,
        run_id: str,
        pose_id: int,
        controller: str,
        goal_handle: Any,
        result_payload: dict[str, Any],
    ) -> None:
        with self._lock:
            if self._manual_goal_handles.get(controller) is not goal_handle:
                return
            self._manual_goal_handles[controller] = None
            command = self._manual_last_commands.get(controller)
            if command is not None and command.get('run_id') == run_id:
                command['state'] = result_payload['status']
                command['result'] = result_payload
                command['finished_at'] = result_payload['received_at']
            self._append_event_locked(
                kind='manual_sequence',
                level='info' if result_payload['ok'] else 'error',
                event_type='MANUAL_SEQUENCE_CONTROLLER_RESULT',
                message=(
                    f'Sequence {run_id} {controller} '
                    f'{result_payload["status"]}: '
                    f'{result_payload["error_string"]}'
                ),
                payload={
                    'run_id': run_id,
                    'pose_id': pose_id,
                    'controller': controller,
                    'status': result_payload['status'],
                    'error_code': result_payload['error_code'],
                    'error_string': result_payload['error_string'],
                },
            )

    def _attach_sequence_result_cleanups(
        self,
        run_id: str,
        pose_id: int,
        pending: set[str],
        handles: dict[str, Any],
        result_futures: dict[str, Any],
    ) -> None:
        for controller in pending:
            goal_handle = handles[controller]
            result_futures[controller].add_done_callback(
                lambda future, name=controller, handle=goal_handle: (
                    self._late_sequence_result_callback(
                        run_id,
                        pose_id,
                        name,
                        handle,
                        future,
                    )
                )
            )

    def _late_sequence_result_callback(
        self,
        run_id: str,
        pose_id: int,
        controller: str,
        goal_handle: Any,
        future: Any,
    ) -> None:
        self._record_sequence_controller_result(
            run_id,
            pose_id,
            controller,
            goal_handle,
            self._sequence_result_payload(future, controller),
        )

    def _cancel_and_drain_sequence_handles(
        self,
        run_id: str,
        pose_id: int,
        handles: dict[str, Any],
        *,
        result_futures: dict[str, Any] | None = None,
    ) -> set[str]:
        if not handles:
            return set()
        self._cancel_active_sequence_goals(run_id)

        observable_futures = dict(result_futures or {})
        unresolved = set(handles)
        if result_futures is None:
            for controller, goal_handle in handles.items():
                try:
                    observable_futures[
                        controller
                    ] = goal_handle.get_result_async()
                except Exception as exc:
                    self.get_logger().warning(
                        f'Failed to observe canceled {controller} result: '
                        f'{exc}'
                    )

        deadline = time.monotonic() + self._request_timeout_s
        pending = set(observable_futures)
        while pending and time.monotonic() < deadline:
            for controller in list(pending):
                future = observable_futures[controller]
                if not future.done():
                    continue
                pending.remove(controller)
                unresolved.discard(controller)
                self._record_sequence_controller_result(
                    run_id,
                    pose_id,
                    controller,
                    handles[controller],
                    self._sequence_result_payload(future, controller),
                )
            if pending:
                time.sleep(0.01)

        self._attach_sequence_result_cleanups(
            run_id,
            pose_id,
            pending,
            handles,
            observable_futures,
        )
        return unresolved

    def _sequence_result_payload(
        self,
        future: Any,
        controller: str,
    ) -> dict[str, Any]:
        try:
            result_response = future.result()
            result = result_response.result
            status_name = STATUS_NAME_BY_CODE.get(
                result_response.status,
                f'STATUS_{result_response.status}',
            )
            if controller == 'arm':
                error_code = None
                arm_success = bool(getattr(
                    result,
                    'success',
                    getattr(result, 'error_code', 1) == 0,
                ))
                error_string = str(getattr(
                    result,
                    'message',
                    getattr(result, 'error_string', ''),
                ))
                ok = (
                    result_response.status == GoalStatus.STATUS_SUCCEEDED
                    and arm_success
                )
            else:
                error_code = int(result.error_code)
                error_string = str(result.error_string)
                ok = (
                    result_response.status == GoalStatus.STATUS_SUCCEEDED
                    and error_code
                    == FollowJointTrajectory.Result.SUCCESSFUL
                )
        except Exception as exc:
            status_name = 'ERROR'
            error_code = None
            error_string = str(exc)
            ok = False
        return {
            'ok': ok,
            'status': status_name,
            'error_code': error_code,
            'error_string': error_string,
            'received_at': self._now_iso(),
        }

    def _cancel_active_sequence_goals(
        self,
        run_id: str,
    ) -> dict[str, dict[str, Any]]:
        with self._sequence_cancel_lock:
            with self._lock:
                if self._sequence_state.get('run_id') != run_id:
                    return {}
                handles = {
                    controller: handle
                    for controller, handle in self._manual_goal_handles.items()
                    if handle is not None
                    and controller not in self._sequence_cancel_requested
                }
                self._sequence_cancel_requested.update(handles)
                existing = deepcopy(
                    self._sequence_state.get('cancel_results') or {}
                )

            if not handles:
                return existing

            futures: dict[str, Any] = {}
            for controller, handle in handles.items():
                try:
                    futures[controller] = handle.cancel_goal_async()
                except Exception as exc:
                    existing[controller] = {
                        'ok': False,
                        'status': 'ERROR',
                        'message': str(exc),
                    }
                with self._lock:
                    self._append_event_locked(
                        kind='manual_sequence',
                        level='info',
                        event_type='MANUAL_SEQUENCE_CONTROLLER_CANCEL_REQUEST',
                        message=(
                            f'Sequence {run_id} cancel requested for '
                            f'{controller}'
                        ),
                        payload={
                            'run_id': run_id,
                            'controller': controller,
                        },
                    )

            deadline = time.monotonic() + self._request_timeout_s
            pending = set(futures)
            while pending and time.monotonic() < deadline:
                for controller in list(pending):
                    future = futures[controller]
                    if not future.done():
                        continue
                    pending.remove(controller)
                    try:
                        response = future.result()
                        canceling = bool(response.goals_canceling)
                        existing[controller] = {
                            'ok': canceling,
                            'status': (
                                'CANCELING' if canceling
                                else 'CANCEL_REJECTED'
                            ),
                            'message': (
                                'Cancel accepted' if canceling
                                else 'Cancel rejected'
                            ),
                        }
                    except Exception as exc:
                        existing[controller] = {
                            'ok': False,
                            'status': 'ERROR',
                            'message': str(exc),
                        }
                if pending:
                    time.sleep(0.01)

            for controller in pending:
                existing[controller] = {
                    'ok': False,
                    'status': 'TIMEOUT',
                    'message': 'Cancel request timed out',
                }

            with self._lock:
                if self._sequence_state.get('run_id') == run_id:
                    self._sequence_state['cancel_results'] = deepcopy(existing)
                    for controller in handles:
                        result = existing.get(controller, {})
                        self._append_event_locked(
                            kind='manual_sequence',
                            level='info' if result.get('ok') else 'error',
                            event_type='MANUAL_SEQUENCE_CONTROLLER_CANCEL_RESULT',
                            message=(
                                f'Sequence {run_id} {controller} cancel '
                                f'{result.get("status", "UNKNOWN").lower()}'
                            ),
                            payload={
                                'run_id': run_id,
                                'controller': controller,
                                **result,
                            },
                        )
            return existing

    @staticmethod
    def _sequence_cancellation_clean(
        cancel_results: dict[str, dict[str, Any]],
    ) -> bool:
        return all(result.get('ok') for result in cancel_results.values())

    def _attach_pending_sequence_cancels(
        self,
        run_id: str,
        pending: set[str],
        futures: dict[str, Any],
    ) -> None:
        for controller in pending:
            self._attach_late_sequence_cancel(
                run_id,
                controller,
                futures[controller],
            )

    def _attach_late_sequence_cancel(
        self,
        run_id: str,
        controller: str,
        send_future: Any,
    ) -> None:
        send_future.add_done_callback(
            lambda future: self._cancel_late_sequence_goal(
                run_id,
                controller,
                future,
            )
        )

    def _cancel_late_sequence_goal(
        self,
        run_id: str,
        controller: str,
        send_future: Any,
    ) -> None:
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get('run_id') == run_id:
                    command['state'] = 'SEND_ERROR'
                    command['result'] = {'error_string': str(exc)}
            self.get_logger().warning(
                f'Late {controller} send failed for {run_id}: {exc}'
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get('run_id') == run_id:
                    command['state'] = 'REJECTED'
            return

        with self._lock:
            existing = self._manual_goal_handles.get(controller)
            if existing is not None and existing is not goal_handle:
                self.get_logger().error(
                    f'Cannot track late {controller} goal for {run_id}: '
                    'another handle is active'
                )
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self._manual_goal_handles[controller] = goal_handle
            command = self._manual_last_commands.get(controller)
            pose_id = 0
            if command is not None and command.get('run_id') == run_id:
                command['state'] = 'CANCELING'
                pose_id = int(command.get('pose_id') or 0)
            self._append_event_locked(
                kind='manual_sequence',
                level='info',
                event_type='MANUAL_SEQUENCE_LATE_GOAL_CANCEL',
                message=(
                    f'Sequence {run_id} canceled late-accepted '
                    f'{controller} goal'
                ),
                payload={
                    'run_id': run_id,
                    'controller': controller,
                },
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            self.get_logger().error(
                f'Cannot observe late {controller} result for {run_id}: {exc}'
            )
            result_future = None
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(
                f'Failed to cancel late {controller} goal for {run_id}: {exc}'
            )
        if result_future is not None:
            result_future.add_done_callback(
                lambda future: self._late_sequence_result_callback(
                    run_id,
                    pose_id,
                    controller,
                    goal_handle,
                    future,
                )
            )

    def _mission_action_ready(self) -> bool:
        try:
            return bool(self._mission_client.server_is_ready())
        except Exception:
            return False

    def _nav_action_ready(self) -> bool:
        try:
            return bool(self._nav_client.server_is_ready())
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
        timeout_s = self._request_timeout_s
        if command == 'home_all':
            timeout_s = self._home_service_timeout_s
        elif command == 'clear_error':
            timeout_s = self._clear_service_timeout_s
        response = self._wait_for_future(future, timeout_s=timeout_s)

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
        """Send or coalesce one dashboard manual target."""
        controller = str(controller)
        config = self._manual_controllers.get(controller)
        client = self._manual_clients.get(controller)
        if config is None or client is None:
            return {
                'ok': False,
                'status_code': 404,
                'message': f'Unknown manual controller: {controller}',
            }

        gui_received_unix_ms = time.time_ns() // 1_000_000
        request_id = str(
            payload.get('request_id')
            or f'manual-{controller}-{time.time_ns()}'
        )
        try:
            web_created_unix_ms = int(
                payload.get('client_created_unix_ms')
                or gui_received_unix_ms
            )
        except (TypeError, ValueError):
            web_created_unix_ms = gui_received_unix_ms
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
        summary['request_id'] = request_id
        summary['web_created_unix_ms'] = web_created_unix_ms
        summary['gui_received_unix_ms'] = gui_received_unix_ms
        for field_name, value in (
            ('request_id', request_id),
            ('web_created_unix_ms', web_created_unix_ms),
            ('gui_received_unix_ms', gui_received_unix_ms),
        ):
            if hasattr(goal_msg, field_name):
                setattr(goal_msg, field_name, value)

        normalized_payload = self._manual_payload_from_summary(summary)
        with self._lock:
            admission = self._manual_admission_locked(
                controller,
                normalized_payload,
                summary,
            )
        if admission is not None:
            response, cancel_request = admission
            if cancel_request is not None:
                self._request_manual_replacement_cancel(*cancel_request)
            return response

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

        response = self._start_manual_goal(
            controller,
            normalized_payload,
            goal_msg,
            summary,
        )
        if response is None:
            return {
                'ok': False,
                'status_code': 409,
                'message': 'Manual target was superseded before dispatch',
            }
        return response

    @staticmethod
    def _manual_payload_from_summary(
        summary: dict[str, Any],
    ) -> dict[str, Any]:
        payload = {
            'positions_deg': deepcopy(summary['positions_deg']),
            'duration_sec': float(summary['duration_sec']),
        }
        if 'target_load_raw' in summary:
            payload['target_load_raw'] = int(summary['target_load_raw'])
        for field_name in (
            'request_id',
            'web_created_unix_ms',
            'gui_received_unix_ms',
        ):
            if field_name in summary:
                payload[field_name] = summary[field_name]
        return payload

    @staticmethod
    def _manual_targets_equal(
        first: dict[str, Any] | None,
        second: dict[str, Any] | None,
    ) -> bool:
        if first is None or second is None:
            return False
        return (
            first.get('controller') == second.get('controller')
            and first.get('joint_names') == second.get('joint_names')
            and first.get('positions_deg') == second.get('positions_deg')
            and first.get('duration_sec') == second.get('duration_sec')
            and first.get('target_load_raw')
            == second.get('target_load_raw')
        )

    def _manual_admission_locked(
        self,
        controller: str,
        payload: dict[str, Any],
        summary: dict[str, Any],
    ) -> tuple[
        dict[str, Any],
        tuple[str, str, Any] | None,
    ] | None:
        if self._sequence_active_locked():
            return ({
                'ok': False,
                'status_code': 409,
                'message': (
                    'Manual control is blocked while a sequence is active'
                ),
            }, None)
        if self._mission_motion_active_locked():
            return ({
                'ok': False,
                'status_code': 409,
                'message': 'Manual control is blocked while a mission is active',
            }, None)
        if self._nav_motion_active_locked():
            return ({
                'ok': False,
                'status_code': 409,
                'message': (
                    'Manual control is blocked while direct navigation '
                    'is active'
                ),
            }, None)

        self._ensure_manual_latest_target_state_locked()
        active_controllers = []
        for name, handle in self._manual_goal_handles.items():
            command_state = (
                self._manual_last_commands.get(name) or {}
            ).get('state')
            if (
                handle is not None
                or command_state in ACTION_BLOCKING_STATES
                or self._manual_pending_latest_targets.get(name) is not None
                or self._manual_dispatching_latest_targets.get(name)
                is not None
            ):
                active_controllers.append(name)

        if not active_controllers:
            return None
        if controller != 'arm' or active_controllers != ['arm']:
            return ({
                'ok': False,
                'status_code': 409,
                'message': 'Another manual motion is still active',
            }, None)

        return self._queue_latest_arm_target_locked(payload, summary)

    def _queue_latest_arm_target_locked(
        self,
        payload: dict[str, Any],
        summary: dict[str, Any],
    ) -> tuple[
        dict[str, Any],
        tuple[str, str, Any] | None,
    ]:
        controller = 'arm'
        dispatching = self._manual_dispatching_latest_targets.get(controller)
        command = self._manual_last_commands.get(controller) or {}
        pending = self._manual_pending_latest_targets.get(controller)

        if dispatching is not None and not (
            self._manual_goal_handles.get(controller) is not None
            or command.get('state') in ACTION_BLOCKING_STATES
        ):
            if self._manual_targets_equal(dispatching['summary'], summary):
                return ({
                    'ok': True,
                    'controller': controller,
                    'queued': True,
                    'deduplicated': True,
                    'message': 'Identical latest arm target is already queued',
                    'pending_latest_target': (
                        self._manual_pending_diagnostic(dispatching)
                    ),
                }, None)
            entry = self._new_manual_pending_entry(payload, summary)
            self._manual_dispatching_latest_targets[controller] = entry
            self._append_event_locked(
                kind='manual',
                level='info',
                event_type='MANUAL_LATEST_TARGET_REPLACED',
                message='Replaced arm target before automatic dispatch',
                payload={'request_id': entry['request_id']},
            )
            return ({
                'ok': True,
                'controller': controller,
                'queued': True,
                'deduplicated': False,
                'message': 'Latest arm target replaced before dispatch',
                'pending_latest_target': self._manual_pending_diagnostic(entry),
            }, None)

        if (
            pending is None
            and self._manual_targets_equal(command, summary)
        ):
            return ({
                'ok': True,
                'controller': controller,
                'queued': False,
                'deduplicated': True,
                'message': 'Identical arm target is already active',
                'goal': {
                    key: deepcopy(command.get(key))
                    for key in summary
                },
            }, None)
        if pending is not None and self._manual_targets_equal(
            pending['summary'],
            summary,
        ):
            return ({
                'ok': True,
                'controller': controller,
                'queued': True,
                'deduplicated': True,
                'message': 'Identical latest arm target is already queued',
                'pending_latest_target': self._manual_pending_diagnostic(
                    pending
                ),
            }, None)

        entry = self._new_manual_pending_entry(payload, summary)
        self._manual_pending_latest_targets[controller] = entry
        self._append_event_locked(
            kind='manual',
            level='info',
            event_type='MANUAL_LATEST_TARGET_QUEUED',
            message='Queued latest arm target and requested active goal cancel',
            payload={'request_id': entry['request_id']},
        )

        cancel_request = None
        goal_handle = self._manual_goal_handles.get(controller)
        command_id = str(command.get('command_id') or '')
        if (
            goal_handle is not None
            and command_id
            and self._manual_cancel_requested.get(controller) != command_id
        ):
            self._manual_cancel_requested[controller] = command_id
            command['state'] = 'CANCELING'
            command['cancel_requested_at'] = self._now_iso()
            cancel_request = (controller, command_id, goal_handle)

        return ({
            'ok': True,
            'controller': controller,
            'queued': True,
            'deduplicated': False,
            'message': 'Latest arm target queued; active goal cancellation requested',
            'pending_latest_target': self._manual_pending_diagnostic(entry),
        }, cancel_request)

    def _new_manual_pending_entry(
        self,
        payload: dict[str, Any],
        summary: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            'request_id': str(
                summary.get('request_id')
                or f'manual-latest-{time.time_ns()}'
            ),
            'payload': deepcopy(payload),
            'summary': deepcopy(summary),
            'queued_at': self._now_iso(),
        }

    def _start_manual_goal(
        self,
        controller: str,
        payload: dict[str, Any],
        goal_msg: Any,
        summary: dict[str, Any],
        *,
        expected_dispatch_request_id: str | None = None,
    ) -> dict[str, Any] | None:
        config = self._manual_controllers[controller]
        client = self._manual_clients[controller]
        command_id = f'manual-{controller}-{time.time_ns()}'
        admission = None
        with self._lock:
            self._ensure_manual_latest_target_state_locked()
            if expected_dispatch_request_id is None:
                admission = self._manual_admission_locked(
                    controller,
                    payload,
                    summary,
                )
            else:
                dispatching = (
                    self._manual_dispatching_latest_targets.get(controller)
                )
                if (
                    dispatching is None
                    or dispatching.get('request_id')
                    != expected_dispatch_request_id
                ):
                    return None
                self._manual_dispatching_latest_targets[controller] = None
            if admission is not None:
                pass
            else:
                self._manual_last_commands[controller] = {
                    **summary,
                    'command_id': command_id,
                    'state': 'SENDING',
                    'sent_at': self._now_iso(),
                }
                self._manual_feedback[controller] = None
                self._append_event_locked(
                    kind='manual',
                    level='info',
                    message=f'Sending {config["label"]} manual goal',
                )

        if admission is not None:
            response, cancel_request = admission
            if cancel_request is not None:
                self._request_manual_replacement_cancel(*cancel_request)
            return response

        try:
            send_future = client.send_goal_async(
                goal_msg,
                feedback_callback=lambda msg: self._manual_feedback_callback(
                    controller,
                    msg,
                ),
            )
        except Exception as exc:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['state'] = 'SEND_ERROR'
                    command['result'] = {'error_string': str(exc)}
                dispatch_entry = self._take_pending_manual_dispatch_locked(
                    controller
                )
            if dispatch_entry is not None:
                self._dispatch_latest_manual_target(controller, dispatch_entry)
            return {
                'ok': False,
                'status_code': 500,
                'message': f'Failed to send {controller} goal: {exc}',
            }
        goal_handle = self._wait_for_future(send_future)

        if goal_handle is None:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['state'] = 'SEND_TIMEOUT_PENDING'
                self._append_event_locked(
                    kind='manual',
                    level='error',
                    message=(
                        f'{config["label"]} manual send timed out; '
                        'late resolution is still pending'
                    ),
                )
            send_future.add_done_callback(
                lambda future: self._manual_late_send_callback(
                    controller,
                    command_id,
                    future,
                )
            )
            return {
                'ok': False,
                'status_code': 504,
                'message': (
                    f'Timed out while sending {controller} goal; '
                    'motion remains blocked until late resolution'
                ),
            }

        if not goal_handle.accepted:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['state'] = 'REJECTED'
                dispatch_entry = self._take_pending_manual_dispatch_locked(
                    controller
                )
                self._append_event_locked(
                    kind='manual',
                    level='error',
                    message=f'{config["label"]} manual goal rejected',
                )
            if dispatch_entry is not None:
                self._dispatch_latest_manual_target(controller, dispatch_entry)
            return {
                'ok': False,
                'status_code': 409,
                'message': f'{config["label"]} manual goal was rejected',
            }

        cancel_request = None
        with self._lock:
            self._manual_goal_handles[controller] = goal_handle
            command = self._manual_last_commands.get(controller)
            if command is not None and command.get(
                'command_id'
            ) == command_id:
                command['state'] = 'ACCEPTED'
                command['accepted_at'] = self._now_iso()
                if (
                    controller == 'arm'
                    and self._manual_pending_latest_targets.get(controller)
                    is not None
                    and self._manual_cancel_requested.get(controller)
                    != command_id
                ):
                    self._manual_cancel_requested[controller] = command_id
                    command['state'] = 'CANCELING'
                    command['cancel_requested_at'] = self._now_iso()
                    cancel_request = (
                        controller,
                        command_id,
                        goal_handle,
                    )
            self._append_event_locked(
                kind='manual',
                level='info',
                message=f'{config["label"]} manual goal accepted',
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['state'] = 'CANCELING'
                    command['result'] = {'error_string': str(exc)}
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return {
                'ok': False,
                'status_code': 500,
                'message': (
                    f'Could not observe {controller} result: {exc}; '
                    'goal cancellation requested'
                ),
            }
        if cancel_request is not None:
            self._request_manual_replacement_cancel(*cancel_request)
        result_future.add_done_callback(
            lambda future: self._manual_result_callback(
                controller,
                future,
                expected_handle=goal_handle,
                command_id=command_id,
            )
        )

        return {
            'ok': True,
            'message': f'{config["label"]} manual goal accepted',
            'controller': controller,
            'goal': summary,
        }

    def _take_pending_manual_dispatch_locked(
        self,
        controller: str,
    ) -> dict[str, Any] | None:
        self._ensure_manual_latest_target_state_locked()
        self._manual_cancel_requested[controller] = None
        entry = self._manual_pending_latest_targets.get(controller)
        self._manual_pending_latest_targets[controller] = None
        if entry is not None:
            self._manual_dispatching_latest_targets[controller] = entry
        return entry

    def _dispatch_latest_manual_target(
        self,
        controller: str,
        entry: dict[str, Any],
    ) -> None:
        """Dispatch the newest queued target after the old goal is terminal."""
        client = self._manual_clients[controller]
        config = self._manual_controllers[controller]
        while True:
            with self._lock:
                self._ensure_manual_latest_target_state_locked()
                current = self._manual_dispatching_latest_targets.get(
                    controller
                )
                if current is None:
                    return
                entry = current
                request_id = str(entry['request_id'])
                payload = deepcopy(entry['payload'])

            try:
                goal_msg, summary = self._manual_goal_from_payload(
                    controller,
                    payload,
                )
                for field_name in (
                    'request_id',
                    'web_created_unix_ms',
                    'gui_received_unix_ms',
                ):
                    if field_name in payload:
                        summary[field_name] = payload[field_name]
                        if hasattr(goal_msg, field_name):
                            setattr(goal_msg, field_name, payload[field_name])
            except (TypeError, ValueError) as exc:
                superseded = False
                with self._lock:
                    current = self._manual_dispatching_latest_targets.get(
                        controller
                    )
                    superseded = (
                        current is not None
                        and current.get('request_id') != request_id
                    )
                    if not superseded and (
                        current is not None
                        and current.get('request_id') == request_id
                    ):
                        self._manual_dispatching_latest_targets[controller] = (
                            None
                        )
                    if not superseded:
                        self._append_event_locked(
                            kind='manual',
                            level='error',
                            event_type='MANUAL_LATEST_TARGET_INVALID',
                            message=(
                                'Could not dispatch latest '
                                f'{controller} target: {exc}'
                            ),
                            payload={'request_id': request_id},
                        )
                if superseded:
                    continue
                return

            if not self._manual_action_ready(controller):
                client.wait_for_server(timeout_sec=0.25)
            if not self._manual_action_ready(controller):
                superseded = False
                with self._lock:
                    current = self._manual_dispatching_latest_targets.get(
                        controller
                    )
                    superseded = (
                        current is not None
                        and current.get('request_id') != request_id
                    )
                    if not superseded and (
                        current is not None
                        and current.get('request_id') == request_id
                    ):
                        self._manual_dispatching_latest_targets[controller] = (
                            None
                        )
                    if not superseded:
                        self._append_event_locked(
                            kind='manual',
                            level='error',
                            event_type=(
                                'MANUAL_LATEST_TARGET_SERVER_UNAVAILABLE'
                            ),
                            message=(
                                'Could not automatically dispatch latest '
                                'target; action server is not ready: '
                                f'{config["action_name"]}'
                            ),
                            payload={'request_id': request_id},
                        )
                if superseded:
                    continue
                return

            response = self._start_manual_goal(
                controller,
                payload,
                goal_msg,
                summary,
                expected_dispatch_request_id=request_id,
            )
            if response is not None:
                return

    def _request_manual_replacement_cancel(
        self,
        controller: str,
        command_id: str,
        goal_handle: Any,
    ) -> None:
        """Request ROS action cancellation exactly once for one command."""
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['cancel_error'] = str(exc)
                self._append_event_locked(
                    kind='manual',
                    level='error',
                    event_type='MANUAL_REPLACEMENT_CANCEL_ERROR',
                    message=(
                        f'Failed to request {controller} replacement cancel: '
                        f'{exc}'
                    ),
                    payload={'command_id': command_id},
                )
            return

        cancel_future.add_done_callback(
            lambda future: self._manual_replacement_cancel_callback(
                controller,
                command_id,
                future,
            )
        )

    def _manual_replacement_cancel_callback(
        self,
        controller: str,
        command_id: str,
        future: Any,
    ) -> None:
        try:
            response = future.result()
            accepted = bool(getattr(response, 'goals_canceling', []))
            error = ''
        except Exception as exc:
            accepted = False
            error = str(exc)

        with self._lock:
            command = self._manual_last_commands.get(controller)
            if command is None or command.get('command_id') != command_id:
                return
            command['cancel_acknowledged'] = accepted
            command['cancel_ack_at'] = self._now_iso()
            if error:
                command['cancel_error'] = error
            self._append_event_locked(
                kind='manual',
                level='info' if accepted else 'error',
                event_type='MANUAL_REPLACEMENT_CANCEL_ACK',
                message=(
                    f'{controller} replacement cancel '
                    f'{"acknowledged" if accepted else "not acknowledged"}'
                ),
                payload={
                    'command_id': command_id,
                    'accepted': accepted,
                    'error': error,
                },
            )

    def _manual_goal_from_payload(
        self,
        controller: str,
        payload: dict[str, Any],
    ) -> tuple[Any, dict[str, Any]]:
        config = self._manual_controllers[controller]
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

        joint_names = [
            str(joint['joint_name'])
            for joint in joints
        ]
        if controller == 'arm':
            duration_ms = round(duration_s * 1000.0)
            if not 1 <= duration_ms <= 0xFFFF:
                raise ValueError('Arm duration must be 1..65535 ms')
            goal_msg = ExecuteArmGoal.Goal()
            goal_msg.joint_names = joint_names
            goal_msg.positions = [
                math.radians(value) for value in positions_deg
            ]
            goal_msg.duration_ms = int(duration_ms)
        else:
            point = JointTrajectoryPoint()
            point.positions = [
                math.radians(value) for value in positions_deg
            ]
            if target_load_raw is not None:
                point.effort = [
                    float(target_load_raw)
                    for _ in positions_deg
                ]
            self._set_duration(point, duration_s)
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = joint_names
            goal_msg.trajectory.points = [point]

        summary: dict[str, Any] = {
            'controller': controller,
            'action_name': config['action_name'],
            'positions_deg': {
                str(joint['key']): positions_deg[index]
                for index, joint in enumerate(joints)
            },
            'joint_names': joint_names,
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
            if isinstance(raw_value, bool):
                raise ValueError(f'{key} target must be a finite number')

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
        if isinstance(raw_duration, bool):
            raise ValueError('duration_sec must be a finite number')
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
        if isinstance(raw_load, bool):
            raise ValueError(
                f'target_load_raw must be an integer in '
                f'0..{BOARD3_TARGET_LOAD_MAX}'
            )
        try:
            numeric_load = float(raw_load)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'target_load_raw must be an integer in '
                f'0..{BOARD3_TARGET_LOAD_MAX}'
            ) from exc
        if not math.isfinite(numeric_load) or not numeric_load.is_integer():
            raise ValueError(
                f'target_load_raw must be an integer in '
                f'0..{BOARD3_TARGET_LOAD_MAX}'
            )
        target_load = int(numeric_load)
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
            if controller == 'arm':
                self._manual_feedback[controller] = {
                    'goal_id': int(feedback.goal_id),
                    'phase': int(feedback.phase),
                    'detail': str(feedback.detail),
                    'received_at': self._now_iso(),
                }
            else:
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

    def _manual_late_send_callback(
        self,
        controller: str,
        command_id: str,
        send_future: Any,
    ) -> None:
        config = self._manual_controllers[controller]
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['state'] = 'SEND_ERROR'
                    command['result'] = {'error_string': str(exc)}
                dispatch_entry = self._take_pending_manual_dispatch_locked(
                    controller
                )
            if dispatch_entry is not None:
                self._dispatch_latest_manual_target(controller, dispatch_entry)
            return

        if goal_handle is None or not goal_handle.accepted:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['state'] = 'REJECTED'
                dispatch_entry = self._take_pending_manual_dispatch_locked(
                    controller
                )
            if dispatch_entry is not None:
                self._dispatch_latest_manual_target(controller, dispatch_entry)
            return

        with self._lock:
            command = self._manual_last_commands.get(controller)
            if command is None or command.get('command_id') != command_id:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            current_handle = self._manual_goal_handles.get(controller)
            if current_handle is not None and current_handle is not goal_handle:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self._manual_goal_handles[controller] = goal_handle
            command['state'] = 'CANCELING'
            command['accepted_at'] = self._now_iso()
            command['cancel_requested_at'] = self._now_iso()
            self._manual_cancel_requested[controller] = command_id
            self._append_event_locked(
                kind='manual',
                level='error',
                message=(
                    f'{config["label"]} late goal accepted; '
                    'cancellation requested'
                ),
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            with self._lock:
                command = self._manual_last_commands.get(controller)
                if command is not None and command.get(
                    'command_id'
                ) == command_id:
                    command['result'] = {'error_string': str(exc)}
            result_future = None
        self._request_manual_replacement_cancel(
            controller,
            command_id,
            goal_handle,
        )
        if result_future is not None:
            result_future.add_done_callback(
                lambda future: self._manual_result_callback(
                    controller,
                    future,
                    expected_handle=goal_handle,
                    command_id=command_id,
                )
            )

    def _manual_result_callback(
        self,
        controller: str,
        future: Any,
        *,
        expected_handle: Any | None = None,
        command_id: str | None = None,
    ) -> None:
        config = self._manual_controllers[controller]
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
            if controller == 'arm':
                arm_success = bool(getattr(
                    result,
                    'success',
                    getattr(result, 'error_code', 1) == 0,
                ))
                result_payload = {
                    'ok': (
                        result_response.status
                        == GoalStatus.STATUS_SUCCEEDED
                        and arm_success
                    ),
                    'status': status_name,
                    'error_code': None,
                    'error_string': str(getattr(
                        result,
                        'message',
                        getattr(result, 'error_string', ''),
                    )),
                    'goal_id': int(getattr(result, 'goal_id', 0)),
                    'received_at': self._now_iso(),
                }
            else:
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

        dispatch_entry = None
        with self._lock:
            if (
                expected_handle is not None
                and self._manual_goal_handles.get(controller)
                is not expected_handle
            ):
                return
            command = self._manual_last_commands.get(controller)
            if (
                command_id is not None
                and (
                    command is None
                    or command.get('command_id') != command_id
                )
            ):
                return
            self._manual_goal_handles[controller] = None
            if command is not None:
                command['state'] = result_payload['status']
                command['result'] = result_payload
                command['finished_at'] = result_payload['received_at']
            self._append_event_locked(
                kind='manual',
                level='info' if result_payload['ok'] else 'error',
                message=(
                    f'{config["label"]} manual {result_payload["status"]}: '
                    f'{result_payload["error_string"]}'
                ),
            )
            dispatch_entry = self._take_pending_manual_dispatch_locked(
                controller
            )

        if dispatch_entry is not None:
            self._dispatch_latest_manual_target(controller, dispatch_entry)

    def start_direct_nav(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Send one same-floor /nav/go_to RunTask goal."""
        with self._lock:
            if self._sequence_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Direct navigation is blocked while a manual '
                        'sequence is active'
                    ),
                }
            if self._manual_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Direct navigation is blocked while manual motion '
                        'is active'
                    ),
                }
            if self._mission_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'Direct navigation is blocked while a mission is active',
                }
            if self._nav_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'A direct navigation goal is already active',
                }

        if not self._nav_action_ready():
            self._nav_client.wait_for_server(timeout_sec=0.25)

        if not self._nav_action_ready():
            return {
                'ok': False,
                'status_code': 503,
                'message': '/nav/go_to action server is not ready',
            }

        try:
            goal_msg, goal_summary = self._nav_goal_from_payload(payload)
        except (TypeError, ValueError) as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': str(exc),
            }

        request_id = f'nav-{time.time_ns()}'
        with self._lock:
            if self._sequence_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Direct navigation is blocked while a manual '
                        'sequence is active'
                    ),
                }
            if self._manual_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Direct navigation is blocked while manual motion '
                        'is active'
                    ),
                }
            if self._mission_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Direct navigation is blocked while a mission is active'
                    ),
                }
            if self._nav_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'A direct navigation goal is already active',
                }
            self._nav_goal = {
                **goal_summary,
                'request_id': request_id,
                'state': 'SENDING',
                'sent_at': self._now_iso(),
            }
            self._nav_feedback = None
            self._nav_result = None
            self._append_event_locked(
                kind='nav',
                level='info',
                message=(
                    f'Sending direct nav to '
                    f'{goal_summary["location_name"]}'
                ),
            )

        try:
            send_future = self._nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self._nav_feedback_callback,
            )
        except Exception as exc:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['state'] = 'SEND_ERROR'
                    self._nav_goal['error'] = str(exc)
            return {
                'ok': False,
                'status_code': 500,
                'message': f'Failed to send direct navigation goal: {exc}',
            }
        goal_handle = self._wait_for_future(send_future)

        if goal_handle is None:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['state'] = 'SEND_TIMEOUT_PENDING'
                self._append_event_locked(
                    kind='nav',
                    level='error',
                    message=(
                        'Direct nav send timed out; late resolution is '
                        'still pending'
                    ),
                )
            send_future.add_done_callback(
                lambda future: self._nav_late_send_callback(
                    request_id,
                    future,
                )
            )
            return {
                'ok': False,
                'status_code': 504,
                'message': (
                    'Timed out while sending direct navigation goal; '
                    'motion remains blocked until late resolution'
                ),
            }

        if not goal_handle.accepted:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['state'] = 'REJECTED'
                self._append_event_locked(
                    kind='nav',
                    level='error',
                    message=(
                        f'Direct nav rejected: '
                        f'{goal_summary["location_name"]}'
                    ),
                )
            return {
                'ok': False,
                'status_code': 409,
                'message': 'Direct navigation goal was rejected',
            }

        with self._lock:
            self._nav_goal_handle = goal_handle
            if (
                self._nav_goal is not None
                and self._nav_goal.get('request_id') == request_id
            ):
                self._nav_goal['state'] = 'ACCEPTED'
                self._nav_goal['accepted_at'] = self._now_iso()
            self._append_event_locked(
                kind='nav',
                level='info',
                message=(
                    f'Direct nav accepted: '
                    f'{goal_summary["location_name"]}'
                ),
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['state'] = 'CANCELING'
                    self._nav_goal['error'] = str(exc)
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return {
                'ok': False,
                'status_code': 500,
                'message': (
                    f'Could not observe direct navigation result: {exc}; '
                    'goal cancellation requested'
                ),
            }
        result_future.add_done_callback(
            lambda future: self._nav_result_callback(
                future,
                expected_handle=goal_handle,
                request_id=request_id,
            )
        )

        return {
            'ok': True,
            'message': 'Direct navigation goal accepted',
            'goal': goal_summary,
        }

    def _nav_goal_from_payload(
        self,
        payload: dict[str, Any],
    ) -> tuple[RunTask.Goal, dict[str, Any]]:
        location_id = str(payload.get('location_id') or '').strip()
        location_name = str(
            payload.get('location_name')
            or payload.get('target')
            or ''
        ).strip()
        if not location_id and not location_name:
            raise ValueError('location_id is required')

        location = self._nav_location_from_payload(
            location_id=location_id,
            location_name=location_name,
        )
        if location is None:
            target = location_id or location_name
            raise ValueError(f'Unknown navigation location: {target}')

        location_name = str(location.get('name') or location_name).strip()

        if 'nav_target' not in location and 'pose' not in location:
            raise ValueError(
                f'Location is not a saved navigation point: {location_name}'
            )

        location_floor = self._optional_int(location.get('floor'))
        target_floor = self._required_int(
            payload.get('target_floor', location_floor),
            'target_floor',
        )
        if target_floor <= 0:
            raise ValueError('target_floor must be greater than zero')

        if location_floor is not None and location_floor != target_floor:
            raise ValueError(
                f'{location_name} is on floor {location_floor}, '
                f'but current floor is {target_floor}'
            )

        target_name = str(
            location.get('nav_target') or location_name
        ).strip()
        if not target_name:
            target_name = location_name

        marker_id = self._optional_int(location.get('marker_id'))
        if marker_id is None:
            marker_id = -1

        extra_payload: dict[str, Any] = {
            'location_name': location_name,
            'location_type': str(location.get('type', '')),
            'direct_nav': True,
        }
        pose = location.get('pose')
        if isinstance(pose, dict):
            extra_payload['pose'] = deepcopy(pose)

        goal_msg = RunTask.Goal()
        goal_msg.task_id = 'go_to'
        goal_msg.target_name = target_name
        goal_msg.target_floor = target_floor
        goal_msg.marker_id = marker_id
        goal_msg.extra_json = json.dumps(
            extra_payload,
            ensure_ascii=False,
        )

        return goal_msg, {
            'task_id': goal_msg.task_id,
            'server': '/nav/go_to',
            'location_id': str(location.get('id', '')),
            'location_name': location_name,
            'target_name': target_name,
            'target_floor': target_floor,
            'marker_id': marker_id,
            'extra_json': goal_msg.extra_json,
        }

    def _nav_location_from_payload(
        self,
        *,
        location_id: str,
        location_name: str,
    ) -> dict[str, Any] | None:
        locations = self._gui_config.get('direct_nav_locations', [])
        if not locations:
            locations = self._gui_config.get('locations', [])

        for location in locations:
            if not isinstance(location, dict):
                continue
            if location_id and str(location.get('id', '')) == location_id:
                return location
            if location_id:
                continue
            if str(location.get('name', '')) == location_name:
                return location
        return None

    @staticmethod
    def _optional_int(value: Any) -> int | None:
        if value is None or value == '':
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @classmethod
    def _required_int(cls, value: Any, field_name: str) -> int:
        result = cls._optional_int(value)
        if result is None:
            raise ValueError(f'{field_name} must be an integer')
        return result

    def _nav_feedback_callback(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        with self._lock:
            self._nav_feedback = {
                'phase': feedback.phase,
                'progress': float(feedback.progress),
                'detail': feedback.detail,
                'received_at': self._now_iso(),
            }

    def _nav_late_send_callback(
        self,
        request_id: str,
        send_future: Any,
    ) -> None:
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['state'] = 'SEND_ERROR'
                    self._nav_goal['error'] = str(exc)
            return

        if goal_handle is None or not goal_handle.accepted:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['state'] = 'REJECTED'
            return

        with self._lock:
            if (
                self._nav_goal is None
                or self._nav_goal.get('request_id') != request_id
            ):
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            if (
                self._nav_goal_handle is not None
                and self._nav_goal_handle is not goal_handle
            ):
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self._nav_goal_handle = goal_handle
            self._nav_goal['state'] = 'CANCELING'
            self._nav_goal['accepted_at'] = self._now_iso()
            self._append_event_locked(
                kind='nav',
                level='error',
                message=(
                    'Late direct navigation goal accepted; cancellation '
                    'requested'
                ),
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['error'] = str(exc)
            result_future = None
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            with self._lock:
                if (
                    self._nav_goal is not None
                    and self._nav_goal.get('request_id') == request_id
                ):
                    self._nav_goal['error'] = str(exc)
        if result_future is not None:
            result_future.add_done_callback(
                lambda future: self._nav_result_callback(
                    future,
                    expected_handle=goal_handle,
                    request_id=request_id,
                )
            )

    def _nav_result_callback(
        self,
        future: Any,
        *,
        expected_handle: Any | None = None,
        request_id: str | None = None,
    ) -> None:
        try:
            result_response = future.result()
        except Exception as exc:
            result_payload = {
                'ok': False,
                'status': 'ERROR',
                'success': False,
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
                'message': result.message,
                'received_at': self._now_iso(),
            }

        with self._lock:
            if (
                expected_handle is not None
                and self._nav_goal_handle is not expected_handle
            ):
                return
            if (
                request_id is not None
                and (
                    self._nav_goal is None
                    or self._nav_goal.get('request_id') != request_id
                )
            ):
                return
            self._nav_result = result_payload
            self._nav_goal_handle = None
            if self._nav_goal is not None:
                self._nav_goal['state'] = result_payload['status']
                self._nav_goal['finished_at'] = result_payload['received_at']
            self._append_event_locked(
                kind='nav',
                level='info' if result_payload['success'] else 'error',
                message=(
                    f'Direct nav {result_payload["status"]}: '
                    f'{result_payload["message"]}'
                ),
            )

    def cancel_direct_nav(self) -> dict[str, Any]:
        """Cancel the active direct navigation goal if one exists."""
        with self._lock:
            goal_handle = self._nav_goal_handle

        if goal_handle is None:
            return {
                'ok': False,
                'message': 'No active direct navigation goal to cancel',
            }

        cancel_future = goal_handle.cancel_goal_async()
        response = self._wait_for_future(cancel_future)

        if response is None:
            return {
                'ok': False,
                'message': 'Timed out while canceling direct navigation goal',
            }

        canceling = len(response.goals_canceling) > 0
        with self._lock:
            if self._nav_goal is not None:
                self._nav_goal['state'] = (
                    'CANCELING' if canceling else 'CANCEL_REJECTED'
                )
            self._append_event_locked(
                kind='nav',
                level='info' if canceling else 'error',
                message='Direct nav cancel requested'
                if canceling else 'Direct nav cancel rejected',
            )

        return {
            'ok': canceling,
            'message': 'Direct nav cancel requested'
            if canceling else 'Direct nav cancel rejected',
        }

    def publish_initial_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Publish an AMCL initial pose from the dashboard map."""
        try:
            x = float(payload['x'])
            y = float(payload['y'])
            yaw = float(payload['yaw'])
        except (KeyError, TypeError, ValueError) as exc:
            return {
                'ok': False,
                'status_code': 400,
                'message': f'Invalid initial pose payload: {exc}',
            }

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return {
                'ok': False,
                'status_code': 400,
                'message': 'Initial pose values must be finite numbers',
            }

        with self._lock:
            frame_id = str(
                payload.get('frame_id')
                or (self._driving_map or {}).get('frame_id')
                or 'map'
            )

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        self._assign_yaw_to_quaternion(msg.pose.pose.orientation, yaw)

        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.06853892326654787
        msg.pose.covariance = covariance

        self._initial_pose_publisher.publish(msg)

        summary = {
            'topic': self._initial_pose_topic,
            'frame_id': frame_id,
            'x': x,
            'y': y,
            'yaw': yaw,
            'yaw_deg': math.degrees(yaw),
            'sent_at': self._now_iso(),
        }
        with self._lock:
            self._append_event_locked(
                kind='driving',
                level='info',
                event_type='INITIAL_POSE',
                message=(
                    f'Initial pose published: x={x:.3f}, y={y:.3f}, '
                    f'yaw={math.degrees(yaw):.1f} deg'
                ),
                payload=summary,
            )

        return {
            'ok': True,
            'message': 'Initial pose published',
            'initial_pose': summary,
        }

    def start_mission(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Send an ExecuteMission goal from the dashboard."""
        with self._lock:
            if self._sequence_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Mission start is blocked while a manual sequence '
                        'is active'
                    ),
                }
            if self._manual_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Mission start is blocked while manual motion is '
                        'active'
                    ),
                }
            if self._mission_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'A mission goal is already active',
                }
            if self._nav_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'Mission start is blocked while direct navigation is active',
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

        request_id = f'mission-{time.time_ns()}'
        with self._lock:
            if self._sequence_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Mission start is blocked while a manual sequence '
                        'is active'
                    ),
                }
            if self._manual_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Mission start is blocked while manual motion is '
                        'active'
                    ),
                }
            if self._mission_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': 'A mission goal is already active',
                }
            if self._nav_motion_active_locked():
                return {
                    'ok': False,
                    'status_code': 409,
                    'message': (
                        'Mission start is blocked while direct navigation '
                        'is active'
                    ),
                }
            self._mission_goal = {
                **goal_summary,
                'request_id': request_id,
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

        try:
            send_future = self._mission_client.send_goal_async(
                goal_msg,
                feedback_callback=self._mission_feedback_callback,
            )
        except Exception as exc:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
                    self._mission_goal['state'] = 'SEND_ERROR'
                    self._mission_goal['error'] = str(exc)
            return {
                'ok': False,
                'status_code': 500,
                'message': f'Failed to send mission goal: {exc}',
            }
        goal_handle = self._wait_for_future(send_future)

        if goal_handle is None:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
                    self._mission_goal['state'] = 'SEND_TIMEOUT_PENDING'
                self._append_event_locked(
                    kind='mission',
                    level='error',
                    message=(
                        'Mission send timed out; late resolution is still '
                        'pending'
                    ),
                )
            send_future.add_done_callback(
                lambda future: self._mission_late_send_callback(
                    request_id,
                    future,
                )
            )
            return {
                'ok': False,
                'status_code': 504,
                'message': (
                    'Timed out while sending mission goal; motion remains '
                    'blocked until late resolution'
                ),
            }

        if not goal_handle.accepted:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
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

        with self._lock:
            self._mission_goal_handle = goal_handle
            if (
                self._mission_goal is not None
                and self._mission_goal.get('request_id') == request_id
            ):
                self._mission_goal['state'] = 'ACCEPTED'
                self._mission_goal['accepted_at'] = self._now_iso()
            self._append_event_locked(
                kind='mission',
                level='info',
                message=f'Mission accepted: {goal_summary["mission_id"]}',
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
                    self._mission_goal['state'] = 'CANCELING'
                    self._mission_goal['error'] = str(exc)
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return {
                'ok': False,
                'status_code': 500,
                'message': (
                    f'Could not observe mission result: {exc}; '
                    'goal cancellation requested'
                ),
            }
        result_future.add_done_callback(
            lambda future: self._mission_result_callback(
                future,
                expected_handle=goal_handle,
                request_id=request_id,
            )
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
        arm_task_name = str(payload.get('arm_task_name') or '').strip()

        if not mission_id:
            raise ValueError('mission_id is required')
        if not pickup_location:
            raise ValueError('pickup_location is required')
        if not delivery_location:
            raise ValueError('delivery_location is required')
        if not object_label:
            raise ValueError('object_label is required')
        if object_label != 'object_1':
            raise ValueError('the calibrated final scenario supports object_1 only')
        if pickup_location not in {'402', '402_4f', 'room_402'}:
            raise ValueError('the calibrated pickup location is 402')
        if delivery_location != 'object_place':
            raise ValueError('the calibrated delivery location is object_place')
        allowed_arm_tasks = {
            option['name'] for option in ARM_TASK_OPTIONS
        }
        if arm_task_name not in allowed_arm_tasks:
            raise ValueError(
                'arm_task_name must be one of: '
                + ', '.join(sorted(allowed_arm_tasks))
            )

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
        goal_msg.arm_task_name = arm_task_name

        return goal_msg, {
            'mission_id': mission_id,
            'pickup_location': pickup_location,
            'delivery_location': delivery_location,
            'target_floor': target_floor,
            'object_label': object_label,
            'arm_task_name': arm_task_name,
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

    def _mission_late_send_callback(
        self,
        request_id: str,
        send_future: Any,
    ) -> None:
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
                    self._mission_goal['state'] = 'SEND_ERROR'
                    self._mission_goal['error'] = str(exc)
            return

        if goal_handle is None or not goal_handle.accepted:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
                    self._mission_goal['state'] = 'REJECTED'
            return

        with self._lock:
            if (
                self._mission_goal is None
                or self._mission_goal.get('request_id') != request_id
            ):
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            if (
                self._mission_goal_handle is not None
                and self._mission_goal_handle is not goal_handle
            ):
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                return
            self._mission_goal_handle = goal_handle
            self._mission_goal['state'] = 'CANCELING'
            self._mission_goal['accepted_at'] = self._now_iso()
            self._append_event_locked(
                kind='mission',
                level='error',
                message='Late mission goal accepted; cancellation requested',
            )

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
                    self._mission_goal['error'] = str(exc)
            result_future = None
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            with self._lock:
                if (
                    self._mission_goal is not None
                    and self._mission_goal.get('request_id') == request_id
                ):
                    self._mission_goal['error'] = str(exc)
        if result_future is not None:
            result_future.add_done_callback(
                lambda future: self._mission_result_callback(
                    future,
                    expected_handle=goal_handle,
                    request_id=request_id,
                )
            )

    def _mission_result_callback(
        self,
        future: Any,
        *,
        expected_handle: Any | None = None,
        request_id: str | None = None,
    ) -> None:
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
            if (
                expected_handle is not None
                and self._mission_goal_handle is not expected_handle
            ):
                return
            if (
                request_id is not None
                and (
                    self._mission_goal is None
                    or self._mission_goal.get('request_id') != request_id
                )
            ):
                return
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

    def _wait_for_future(
        self,
        future: Any,
        *,
        timeout_s: float | None = None,
    ) -> Any:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())

        wait_timeout = (
            self._request_timeout_s
            if timeout_s is None
            else max(0.1, float(timeout_s))
        )
        if not event.wait(timeout=wait_timeout):
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
        event_type: str | None = None,
        payload: dict[str, Any] | None = None,
    ) -> None:
        event_time = self._now_iso()
        entry = {
            'kind': kind,
            'source': kind,
            'level': level,
            'event_type': event_type or kind.upper(),
            'message': message,
            'time': event_time,
        }
        if isinstance(payload, dict):
            entry['payload'] = deepcopy(payload)
        entry['seq'] = self._store_event_log_locked(
            source=kind,
            event_type=entry['event_type'],
            message=message,
            payload=entry,
            timestamp=event_time,
        )
        self._event_log.append(entry)

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
        with self._lock:
            if self._sequence_active_locked():
                self._sequence_state['state'] = 'STOPPING'
                self._sequence_stop_event.set()
            sequence_handles = [
                handle
                for handle in self._manual_goal_handles.values()
                if handle is not None
            ]
            sequence_thread = self._sequence_thread
        for goal_handle in sequence_handles:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to cancel sequence goal during shutdown: {exc}'
                )
        if sequence_thread is not None and sequence_thread.is_alive():
            sequence_thread.join(
                timeout=max(1.0, min(6.0, self._request_timeout_s + 0.5))
            )
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
        if self._http_thread is not None:
            self._http_thread.join(timeout=1.0)
        if self._event_log_db is not None:
            self._event_log_db.close()
            self._event_log_db = None
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
