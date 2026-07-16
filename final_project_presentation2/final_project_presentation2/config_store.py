"""Strict, revisioned, atomic JSON storage for the presentation system."""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from copy import deepcopy
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from typing import Any

from .workflow_core import (
    canonical_entity_id,
    validate_collections,
    WorkflowCoreError,
)


SCHEMA_VERSION = 1

ARM_JOINT_NAMES = (
    'base_joint',
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
)

GRIPPER_JOINT_NAMES = (
    'finger_1_base_joint',
    'finger_1_middle_joint',
    'finger_1_tip_joint',
    'finger_2_base_joint',
    'finger_2_middle_joint',
    'finger_2_tip_joint',
    'finger_3_base_joint',
    'finger_3_middle_joint',
    'finger_3_tip_joint',
)

MARKER_NAMES = (
    'pickup',
    'dropoff',
)

ARUCO_DICTIONARY_CAPACITY = {
    'DICT_4X4_50': 50,
    'DICT_4X4_100': 100,
    'DICT_4X4_250': 250,
    'DICT_4X4_1000': 1000,
    'DICT_5X5_50': 50,
    'DICT_5X5_100': 100,
    'DICT_5X5_250': 250,
    'DICT_5X5_1000': 1000,
    'DICT_6X6_50': 50,
    'DICT_6X6_100': 100,
    'DICT_6X6_250': 250,
    'DICT_6X6_1000': 1000,
    'DICT_7X7_50': 50,
    'DICT_7X7_100': 100,
    'DICT_7X7_250': 250,
    'DICT_7X7_1000': 1000,
    'DICT_ARUCO_ORIGINAL': 1024,
}

TOP_LEVEL_FIELDS = {
    'schema_version',
    'revision',
    'next_pose_id',
    'next_workflow_id',
    'next_category_id',
    'uncategorized_category_id',
    'web',
    'topics',
    'cameras',
    'aruco',
    'markers',
    'route',
    'motion_control',
    'turn_control',
    'timeouts',
    'safety',
    'arm',
    'gripper',
    'categories',
    'poses',
    'workflows',
}


class ConfigStoreError(RuntimeError):
    """Base class for safe configuration failures."""


class ConfigValidationError(ConfigStoreError):
    """Raised when a candidate document does not match schema v1."""


class ConfigConflictError(ConfigStoreError):
    """Raised when optimistic revision locking detects a stale writer."""


class ConfigNotFoundError(ConfigStoreError):
    """Raised when the authoritative live configuration does not exist."""


class _DuplicateJsonKeyError(ValueError):
    """Raised when a JSON object repeats a key."""


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise _DuplicateJsonKeyError(f'duplicate key {key!r}')
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ValueError(f'non-finite JSON number {value!r} is not allowed')


def _read_json_stream(source: Any) -> Any:
    return json.load(
        source,
        object_pairs_hook=_unique_json_object,
        parse_constant=_reject_json_constant,
    )


class ConfigStore:
    """Process-safe JSON store with atomic replace and optimistic revisions."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self.path = Path(path).expanduser()
        self._thread_lock = threading.RLock()

    def snapshot(self) -> dict[str, Any]:
        """Return a detached, validated snapshot of the current document."""
        with self._file_lock():
            return deepcopy(self._read_document())

    def initialize(self, document: Mapping[str, Any]) -> dict[str, Any]:
        """
        Create a missing live file from a revision-zero template.

        Initialization never overwrites an existing file.  Normal updates must
        use :meth:`replace` or :meth:`mutate`.
        """
        candidate = (
            deepcopy(dict(document))
            if isinstance(document, Mapping)
            else document
        )
        validate_document(candidate)
        if candidate['revision'] != 0:
            raise ConfigValidationError(
                'an initial configuration must have revision 0'
            )
        return self.seed(candidate)

    def seed(self, document: Mapping[str, Any]) -> dict[str, Any]:
        """Create a missing JSON document without changing its revision."""
        candidate = (
            deepcopy(dict(document))
            if isinstance(document, Mapping)
            else document
        )
        validate_document(candidate)
        with self._file_lock():
            if self.path.exists():
                raise ConfigStoreError(
                    f'configuration already exists: {self.path}'
                )
            self._write_document(candidate)
            return deepcopy(candidate)

    def replace(
        self,
        expected_revision: int,
        document: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Replace the document if ``expected_revision`` is still current."""
        _revision(expected_revision, 'expected_revision')
        if not isinstance(document, Mapping):
            raise ConfigValidationError('configuration must be a mapping')

        with self._file_lock():
            current = self._read_document()
            self._check_revision(current, expected_revision)
            candidate = deepcopy(dict(document))
            candidate_revision = candidate.get('revision')
            if candidate_revision != expected_revision:
                raise ConfigConflictError(
                    'replacement document revision must equal '
                    f'expected_revision {expected_revision}'
                )
            candidate['revision'] = expected_revision + 1
            validate_document(candidate)
            self._write_document(candidate)
            return deepcopy(candidate)

    def mutate(
        self,
        expected_revision: int,
        callback: Callable[[dict[str, Any]], Mapping[str, Any] | None],
    ) -> dict[str, Any]:
        """
        Apply ``callback`` under the lock and atomically commit its result.

        The callback may mutate its detached argument and return ``None``, or
        return a complete replacement mapping.  The store alone advances the
        revision.
        """
        _revision(expected_revision, 'expected_revision')
        if not callable(callback):
            raise ConfigStoreError('callback must be callable')

        with self._file_lock():
            current = self._read_document()
            self._check_revision(current, expected_revision)
            working = deepcopy(current)
            result = callback(working)
            if result is None:
                candidate: Any = working
            elif isinstance(result, Mapping):
                candidate = deepcopy(dict(result))
            else:
                raise ConfigValidationError(
                    'mutation callback must return a mapping or None'
                )
            if candidate.get('revision') != expected_revision:
                raise ConfigConflictError(
                    'mutation callback must not change revision'
                )
            candidate['revision'] = expected_revision + 1
            validate_document(candidate)
            self._write_document(candidate)
            return deepcopy(candidate)

    @staticmethod
    def _check_revision(
        document: Mapping[str, Any],
        expected_revision: int,
    ) -> None:
        current_revision = document['revision']
        if current_revision != expected_revision:
            raise ConfigConflictError(
                'configuration changed '
                f'(expected revision {expected_revision}, '
                f'current revision {current_revision})'
            )

    @contextmanager
    def _file_lock(self) -> Iterator[None]:
        with self._thread_lock:
            directory_fd: int | None = None
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                directory_fd = os.open(
                    self.path.parent,
                    os.O_RDONLY | os.O_DIRECTORY,
                )
            except OSError as exc:
                raise ConfigStoreError(
                    'could not lock configuration directory '
                    f'{self.path.parent}: {exc}'
                ) from exc
            try:
                fcntl.flock(directory_fd, fcntl.LOCK_EX)
                yield
            except ConfigStoreError:
                raise
            except OSError as exc:
                raise ConfigStoreError(
                    f'could not access configuration {self.path}: {exc}'
                ) from exc
            finally:
                try:
                    fcntl.flock(directory_fd, fcntl.LOCK_UN)
                finally:
                    os.close(directory_fd)

    def _read_document(self) -> dict[str, Any]:
        if not self.path.exists():
            raise ConfigNotFoundError(
                f'configuration does not exist: {self.path}'
            )
        try:
            with self.path.open('r', encoding='utf-8') as source:
                document = _read_json_stream(source)
        except (
            OSError,
            UnicodeError,
            json.JSONDecodeError,
            _DuplicateJsonKeyError,
            ValueError,
        ) as exc:
            raise ConfigStoreError(
                f'could not read configuration {self.path}: {exc}'
            ) from exc
        validate_document(document)
        return document

    def _write_document(self, document: Mapping[str, Any]) -> None:
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                dir=self.path.parent,
                prefix=f'.{self.path.name}.',
                suffix='.tmp',
                delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                json.dump(
                    dict(document),
                    temporary,
                    ensure_ascii=False,
                    indent=2,
                    allow_nan=False,
                )
                temporary.write('\n')
                temporary.flush()
                os.fsync(temporary.fileno())

            os.replace(temporary_path, self.path)
            temporary_path = None
            directory_fd = os.open(
                self.path.parent,
                os.O_RDONLY | os.O_DIRECTORY,
            )
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except (OSError, TypeError, ValueError) as exc:
            raise ConfigStoreError(
                f'could not write configuration {self.path}: {exc}'
            ) from exc
        finally:
            if temporary_path is not None:
                try:
                    temporary_path.unlink()
                except OSError:
                    pass


def load_json_document(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Load and validate a JSON document without creating a lock file."""
    source_path = Path(path).expanduser()
    try:
        with source_path.open('r', encoding='utf-8') as source:
            document = _read_json_stream(source)
    except (
        OSError,
        UnicodeError,
        json.JSONDecodeError,
        _DuplicateJsonKeyError,
        ValueError,
    ) as exc:
        raise ConfigStoreError(
            f'could not read configuration {source_path}: {exc}'
        ) from exc
    validate_document(document)
    return deepcopy(document)


def validate_document(document: Any) -> None:
    """Validate every field and cross-reference in a schema-v1 document."""
    try:
        _validate_document(document)
    except ConfigValidationError:
        raise
    except WorkflowCoreError as exc:
        raise ConfigValidationError(str(exc)) from exc


def _validate_document(document: Any) -> None:
    root = _mapping(document, 'configuration')
    _exact_fields(root, TOP_LEVEL_FIELDS, 'configuration')
    version = _integer(root['schema_version'], 'schema_version', 1, 1)
    if version != SCHEMA_VERSION:
        raise ConfigValidationError(
            f'unsupported schema_version {version}; expected {SCHEMA_VERSION}'
        )
    _revision(root['revision'], 'revision')
    next_pose_id = _positive_integer(root['next_pose_id'], 'next_pose_id')
    next_workflow_id = _positive_integer(
        root['next_workflow_id'], 'next_workflow_id'
    )
    next_category_id = _positive_integer(
        root['next_category_id'], 'next_category_id'
    )
    _positive_integer(
        root['uncategorized_category_id'],
        'uncategorized_category_id',
    )

    _validate_web(root['web'])
    _validate_topics(root['topics'])
    _validate_cameras(root['cameras'])
    dictionary_capacity = _validate_aruco(root['aruco'])
    _validate_markers(root['markers'], dictionary_capacity)
    _validate_route(root['route'], root['markers'])
    _validate_motion_control(root['motion_control'])
    _validate_turn_control(root['turn_control'])
    _validate_timeouts(root['timeouts'])
    _validate_arm(root['arm'])
    _validate_gripper(root['gripper'])
    _validate_pose_bodies(root)
    validate_collections(root)
    _validate_monotonic_ids(
        root,
        next_pose_id,
        next_workflow_id,
        next_category_id,
    )
    _validate_safety(root['safety'], root)


def _validate_web(raw: Any) -> None:
    value = _mapping(raw, 'web')
    _exact_fields(value, {'host', 'port', 'poll_interval_sec'}, 'web')
    _nonempty_string(value['host'], 'web.host', maximum=255)
    _integer(value['port'], 'web.port', 1, 65535)
    _number(value['poll_interval_sec'], 'web.poll_interval_sec', 0.05, 10.0)


def _validate_topics(raw: Any) -> None:
    value = _mapping(raw, 'topics')
    optional_fields = {
        'arm_enable_service',
        'arm_disable_service',
        'arm_home_service',
        'arm_clear_error_service',
        'arm_estop_service',
    }
    fields = {
        'front_image',
        'front_camera_info',
        'rear_image',
        'rear_camera_info',
        'odom',
        'joint_states',
        'cmd_vel_raw',
        'cmd_vel',
        'watchdog_config',
        'arm_action',
        'gripper_action',
        'arm_status_service',
        'arm_enable_service',
        'arm_disable_service',
        'arm_home_service',
        'arm_clear_error_service',
        'arm_estop_service',
    }
    _fields_with_optional(value, fields, optional_fields, 'topics')
    for field in value:
        topic = _nonempty_string(
            value[field], f'topics.{field}', maximum=255
        )
        if not topic.startswith('/') or ' ' in topic:
            raise ConfigValidationError(
                f'topics.{field} must be an absolute ROS name without spaces'
            )
    if value['cmd_vel_raw'] == value['cmd_vel']:
        raise ConfigValidationError(
            'topics.cmd_vel_raw and topics.cmd_vel must be distinct'
        )
    fixed_safety_topics = {
        'cmd_vel_raw': '/final_project_presentation2/cmd_vel_raw',
        'watchdog_config': '/final_project_presentation2/watchdog_config',
        'cmd_vel': '/cmd_vel',
    }
    for field, required in fixed_safety_topics.items():
        if value[field] != required:
            raise ConfigValidationError(
                f'topics.{field} is a fixed safety interface and must equal '
                f'{required!r}'
            )


def _validate_cameras(raw: Any) -> None:
    cameras = _mapping(raw, 'cameras')
    _exact_fields(cameras, {'front', 'rear'}, 'cameras')
    fields = {
        'use_camera_info',
        'transport',
        'flip_horizontal',
        'flip_vertical',
        'steering_sign',
        'camera_matrix',
        'distortion_coefficients',
    }
    for name in ('front', 'rear'):
        camera = _mapping(cameras[name], f'cameras.{name}')
        _exact_fields(camera, fields, f'cameras.{name}')
        for field in ('use_camera_info', 'flip_horizontal', 'flip_vertical'):
            _boolean(camera[field], f'cameras.{name}.{field}')
        if camera['transport'] != 'raw':
            raise ConfigValidationError(
                f'cameras.{name}.transport must be raw in schema v1'
            )
        if camera['steering_sign'] not in {-1, 1} or isinstance(
            camera['steering_sign'], bool
        ):
            raise ConfigValidationError(
                f'cameras.{name}.steering_sign must be -1 or 1'
            )
        matrix = _number_list(
            camera['camera_matrix'],
            f'cameras.{name}.camera_matrix',
            exact_length=9,
        )
        if matrix[0] <= 0.0 or matrix[4] <= 0.0 or matrix[8] == 0.0:
            raise ConfigValidationError(
                f'cameras.{name}.camera_matrix has invalid focal/homogeneous values'
            )
        coefficients = _number_list(
            camera['distortion_coefficients'],
            f'cameras.{name}.distortion_coefficients',
            minimum_length=4,
        )
        if len(coefficients) > 14:
            raise ConfigValidationError(
                f'cameras.{name}.distortion_coefficients has too many values'
            )


def _validate_aruco(raw: Any) -> int:
    value = _mapping(raw, 'aruco')
    _exact_fields(
        value,
        {'dictionary', 'marker_size_m'},
        'aruco',
    )
    dictionary = value['dictionary']
    if dictionary not in ARUCO_DICTIONARY_CAPACITY:
        raise ConfigValidationError(
            'aruco.dictionary is not a supported predefined dictionary'
        )
    _number(value['marker_size_m'], 'aruco.marker_size_m', 0.01, 1.0)
    return ARUCO_DICTIONARY_CAPACITY[dictionary]


def _validate_markers(raw: Any, dictionary_capacity: int) -> None:
    markers = _mapping(raw, 'markers')
    _exact_fields(markers, set(MARKER_NAMES), 'markers')
    expected_ids = {'pickup': 1, 'dropoff': 3}
    required_fields = {
        'id',
        'camera',
        'target_distance_m',
        'target_lateral_m',
        'target_yaw_deg',
    }
    override_fields = {
        'completion_mode',
        'distance_tolerance_m',
        'lateral_tolerance_m',
        'yaw_tolerance_deg',
        'hold_time_sec',
        'max_linear_mps',
        'max_angular_rps',
    }
    ids: list[int] = []
    for name in MARKER_NAMES:
        marker = _mapping(markers[name], f'markers.{name}')
        marker_fields = set(marker)
        missing = sorted(required_fields - marker_fields)
        unknown = sorted(
            marker_fields - required_fields - override_fields
        )
        if missing:
            raise ConfigValidationError(
                f'markers.{name} is missing fields: {", ".join(missing)}'
            )
        if unknown:
            raise ConfigValidationError(
                f'markers.{name} has unknown fields: {", ".join(unknown)}'
            )
        marker_id = _integer(
            marker['id'],
            f'markers.{name}.id',
            0,
            dictionary_capacity - 1,
        )
        ids.append(marker_id)
        if marker_id != expected_ids[name]:
            raise ConfigValidationError(
                f'markers.{name}.id must remain {expected_ids[name]}'
            )
        if marker['camera'] not in {'front', 'rear'}:
            raise ConfigValidationError(
                f'markers.{name}.camera must be front or rear'
            )
        if (
            'completion_mode' in marker
            and marker['completion_mode'] not in {'full_pose', 'distance_only'}
        ):
            raise ConfigValidationError(
                f'markers.{name}.completion_mode must be full_pose or '
                'distance_only'
            )
        _number(
            marker['target_distance_m'],
            f'markers.{name}.target_distance_m',
            0.05,
            3.0,
        )
        _number(
            marker['target_lateral_m'],
            f'markers.{name}.target_lateral_m',
            -1.0,
            1.0,
        )
        _number(
            marker['target_yaw_deg'],
            f'markers.{name}.target_yaw_deg',
            -90.0,
            90.0,
        )
        for field, maximum in (
            ('distance_tolerance_m', 0.5),
            ('lateral_tolerance_m', 0.5),
            ('yaw_tolerance_deg', 45.0),
            ('hold_time_sec', 10.0),
            ('max_linear_mps', 1.0),
            ('max_angular_rps', 3.0),
        ):
            if field not in marker:
                continue
            _number(
                marker[field],
                f'markers.{name}.{field}',
                0.001,
                maximum,
            )
    if len(set(ids)) != len(ids):
        raise ConfigValidationError('both ArUco marker ids must be distinct')


def _validate_route(raw: Any, markers: Mapping[str, Any]) -> None:
    route = _mapping(raw, 'route')
    _exact_fields(route, {'outbound', 'return'}, 'route')
    fields = {
        'camera',
        'start_marker',
        'destination_marker',
        'linear_direction',
    }
    for name in ('outbound', 'return'):
        segment = _mapping(route[name], f'route.{name}')
        _exact_fields(segment, fields, f'route.{name}')
        camera = segment['camera']
        if camera not in {'front', 'rear'}:
            raise ConfigValidationError(
                f'route.{name}.camera must be front or rear'
            )
        linear_direction = _integer(
            segment['linear_direction'],
            f'route.{name}.linear_direction',
            -1,
            1,
        )
        if linear_direction == 0:
            raise ConfigValidationError(
                f'route.{name}.linear_direction must be -1 or 1'
            )
        for field in ('start_marker', 'destination_marker'):
            marker_name = segment[field]
            if marker_name not in markers:
                raise ConfigValidationError(
                    f'route.{name}.{field} must name a configured marker'
                )


def _validate_motion_control(raw: Any) -> None:
    value = _mapping(raw, 'motion_control')
    optional_fields = {
        'acquire_creep_mps',
        'steering_output_scale',
        'steering_slow_band_ratio',
        'alignment_hysteresis_ratio',
    }
    fields = {
        'control_rate_hz',
        'linear_kp',
        'lateral_kp',
        'yaw_kp',
        'max_linear_mps',
        'max_angular_rps',
        'linear_gate_angle_deg',
        'min_steering_rps',
        'distance_tolerance_m',
        'lateral_tolerance_m',
        'yaw_tolerance_deg',
        'stable_detections',
        'hold_time_sec',
        'acquire_creep_mps',
        'steering_output_scale',
        'steering_slow_band_ratio',
        'alignment_hysteresis_ratio',
    }
    _fields_with_optional(value, fields, optional_fields, 'motion_control')
    bounds = {
        'control_rate_hz': (1.0, 100.0),
        'linear_kp': (0.001, 20.0),
        'lateral_kp': (0.001, 20.0),
        'yaw_kp': (0.001, 20.0),
        'max_linear_mps': (0.001, 1.0),
        'max_angular_rps': (0.001, 3.0),
        'linear_gate_angle_deg': (0.1, 90.0),
        'min_steering_rps': (0.0, 3.0),
        'distance_tolerance_m': (0.001, 0.5),
        'lateral_tolerance_m': (0.001, 0.5),
        'yaw_tolerance_deg': (0.1, 45.0),
        'hold_time_sec': (0.0, 10.0),
        'acquire_creep_mps': (0.001, 0.25),
        'steering_output_scale': (0.05, 1.0),
        'steering_slow_band_ratio': (1.01, 10.0),
        'alignment_hysteresis_ratio': (1.0, 5.0),
    }
    for field, (minimum, maximum) in bounds.items():
        if field in value:
            _number(value[field], f'motion_control.{field}', minimum, maximum)
    if value['min_steering_rps'] > value['max_angular_rps']:
        raise ConfigValidationError(
            'motion_control.min_steering_rps cannot exceed max_angular_rps'
        )
    if (
        'acquire_creep_mps' in value
        and value['acquire_creep_mps'] > value['max_linear_mps']
    ):
        raise ConfigValidationError(
            'motion_control.acquire_creep_mps cannot exceed max_linear_mps'
        )
    _integer(
        value['stable_detections'],
        'motion_control.stable_detections',
        1,
        100,
    )


def _validate_turn_control(raw: Any) -> None:
    value = _mapping(raw, 'turn_control')
    fields = {
        'control_rate_hz',
        'kp',
        'min_angular_rps',
        'max_angular_rps',
        'tolerance_deg',
        'settle_time_sec',
        'manual_steps_deg',
    }
    _exact_fields(value, fields, 'turn_control')
    for field, minimum, maximum in (
        ('control_rate_hz', 1.0, 100.0),
        ('kp', 0.001, 20.0),
        ('min_angular_rps', 0.001, 3.0),
        ('max_angular_rps', 0.001, 3.0),
        ('tolerance_deg', 0.1, 20.0),
        ('settle_time_sec', 0.0, 10.0),
    ):
        _number(value[field], f'turn_control.{field}', minimum, maximum)
    if float(value['min_angular_rps']) > float(value['max_angular_rps']):
        raise ConfigValidationError(
            'turn_control.min_angular_rps cannot exceed max_angular_rps'
        )
    steps = _number_list(
        value['manual_steps_deg'],
        'turn_control.manual_steps_deg',
        exact_length=2,
    )
    if any(step <= 0.0 or step > 45.0 for step in steps):
        raise ConfigValidationError(
            'turn_control.manual_steps_deg must contain two values in (0, 45]'
        )
    if len(set(steps)) != len(steps):
        raise ConfigValidationError(
            'turn_control.manual_steps_deg must not contain duplicates'
        )


def _validate_timeouts(raw: Any) -> None:
    value = _mapping(raw, 'timeouts')
    optional_fields = {
        'arm_service_sec',
        'arm_home_sec',
        'acquire_creep_sec',
    }
    fields = {
        'camera_stale_sec',
        'camera_info_sec',
        'odom_stale_sec',
        'joint_state_stale_sec',
        'marker_acquire_sec',
        'marker_loss_sec',
        'marker_observation_stale_sec',
        'straight_segment_sec',
        'turn_sec',
        'alignment_sec',
        'stop_sec',
        'arm_service_sec',
        'arm_home_sec',
        'acquire_creep_sec',
    }
    _fields_with_optional(value, fields, optional_fields, 'timeouts')
    for field in value:
        _number(value[field], f'timeouts.{field}', 0.01, 3600.0)
    if value['marker_observation_stale_sec'] > value['marker_loss_sec']:
        raise ConfigValidationError(
            'marker_observation_stale_sec cannot exceed marker_loss_sec'
        )


def _validate_arm(raw: Any) -> None:
    value = _mapping(raw, 'arm')
    fields = {
        'joint_names',
        'joint_limits_deg',
        'duration_min_sec',
        'duration_max_sec',
        'default_duration_sec',
        'result_timeout_margin_sec',
        'axis4_duration_warning_sec',
    }
    _exact_fields(value, fields, 'arm')
    _joint_names(value['joint_names'], ARM_JOINT_NAMES, 'arm.joint_names')
    _joint_limits(value['joint_limits_deg'], ARM_JOINT_NAMES, 'arm')
    _duration_fields(value, 'arm')
    _number(
        value['axis4_duration_warning_sec'],
        'arm.axis4_duration_warning_sec',
        0.1,
        30.0,
    )


def _validate_gripper(raw: Any) -> None:
    value = _mapping(raw, 'gripper')
    fields = {
        'joint_names',
        'joint_limits_deg',
        'duration_min_sec',
        'duration_max_sec',
        'default_duration_sec',
        'result_timeout_margin_sec',
        'target_load_raw',
    }
    _exact_fields(value, fields, 'gripper')
    _joint_names(
        value['joint_names'], GRIPPER_JOINT_NAMES, 'gripper.joint_names'
    )
    _joint_limits(value['joint_limits_deg'], GRIPPER_JOINT_NAMES, 'gripper')
    _duration_fields(value, 'gripper')
    load = _mapping(value['target_load_raw'], 'gripper.target_load_raw')
    _exact_fields(load, {'min', 'max', 'default'}, 'gripper.target_load_raw')
    minimum = _integer(load['min'], 'gripper.target_load_raw.min', 0, 1023)
    maximum = _integer(load['max'], 'gripper.target_load_raw.max', 0, 1023)
    default = _integer(load['default'], 'gripper.target_load_raw.default', 0, 1023)
    if minimum > maximum or not minimum <= default <= maximum:
        raise ConfigValidationError(
            'gripper target-load values must satisfy min <= default <= max'
        )


def _validate_pose_bodies(root: Mapping[str, Any]) -> None:
    arm = root['arm']
    gripper = root['gripper']
    poses = _mapping(root['poses'], 'poses')
    for raw_pose_id, raw_pose in poses.items():
        pose_id = canonical_entity_id(raw_pose_id, field_name='pose id')
        pose = _mapping(raw_pose, f'poses.{pose_id}')
        arm_command = _mapping(pose.get('arm'), f'poses.{pose_id}.arm')
        _exact_fields(
            arm_command,
            {'enabled', 'positions_deg', 'duration_sec'},
            f'poses.{pose_id}.arm',
        )
        gripper_command = _mapping(
            pose.get('gripper'), f'poses.{pose_id}.gripper'
        )
        _exact_fields(
            gripper_command,
            {'enabled', 'positions_deg', 'duration_sec', 'target_load_raw'},
            f'poses.{pose_id}.gripper',
        )
        arm_enabled = _boolean(
            arm_command['enabled'], f'poses.{pose_id}.arm.enabled'
        )
        gripper_enabled = _boolean(
            gripper_command['enabled'],
            f'poses.{pose_id}.gripper.enabled',
        )
        if not arm_enabled and not gripper_enabled:
            raise ConfigValidationError(
                f'poses.{pose_id} must enable arm, gripper, or both'
            )
        _pose_positions(
            arm_command['positions_deg'],
            ARM_JOINT_NAMES,
            arm['joint_limits_deg'],
            f'poses.{pose_id}.arm.positions_deg',
        )
        _number(
            arm_command['duration_sec'],
            f'poses.{pose_id}.arm.duration_sec',
            arm['duration_min_sec'],
            arm['duration_max_sec'],
        )
        _pose_positions(
            gripper_command['positions_deg'],
            GRIPPER_JOINT_NAMES,
            gripper['joint_limits_deg'],
            f'poses.{pose_id}.gripper.positions_deg',
        )
        _number(
            gripper_command['duration_sec'],
            f'poses.{pose_id}.gripper.duration_sec',
            gripper['duration_min_sec'],
            gripper['duration_max_sec'],
        )
        load = gripper_command['target_load_raw']
        _integer(
            load,
            f'poses.{pose_id}.gripper.target_load_raw',
            gripper['target_load_raw']['min'],
            gripper['target_load_raw']['max'],
        )
        _number(pose['dwell_sec'], f'poses.{pose_id}.dwell_sec', 0.0, 60.0)


def _validate_monotonic_ids(
    root: Mapping[str, Any],
    next_pose_id: int,
    next_workflow_id: int,
    next_category_id: int,
) -> None:
    for field, next_id, collection_name in (
        ('next_pose_id', next_pose_id, 'poses'),
        ('next_workflow_id', next_workflow_id, 'workflows'),
        ('next_category_id', next_category_id, 'categories'),
    ):
        ids = [
            int(canonical_entity_id(key, field_name=f'{collection_name} id'))
            for key in root[collection_name]
        ]
        if ids and next_id <= max(ids):
            raise ConfigValidationError(
                f'{field} must be greater than every existing {collection_name} id'
            )


def _validate_safety(raw: Any, root: Mapping[str, Any]) -> None:
    value = _mapping(raw, 'safety')
    fields = {
        'watchdog_rate_hz',
        'watchdog_cmd_timeout_sec',
        'operator_lease_timeout_sec',
        'max_linear_mps',
        'max_angular_rps',
        'reject_nonfinite_commands',
        'require_arm_fault_clear',
    }
    _exact_fields(value, fields, 'safety')
    _number(value['watchdog_rate_hz'], 'safety.watchdog_rate_hz', 1.0, 200.0)
    if float(value['watchdog_rate_hz']) != 20.0:
        raise ConfigValidationError(
            'safety.watchdog_rate_hz is a fixed safety rate of 20 Hz'
        )
    _number(
        value['watchdog_cmd_timeout_sec'],
        'safety.watchdog_cmd_timeout_sec',
        0.05,
        2.0,
    )
    _number(
        value['operator_lease_timeout_sec'],
        'safety.operator_lease_timeout_sec',
        0.2,
        60.0,
    )
    _number(value['max_linear_mps'], 'safety.max_linear_mps', 0.001, 1.0)
    _number(value['max_angular_rps'], 'safety.max_angular_rps', 0.001, 3.0)
    for field in (
        'reject_nonfinite_commands',
        'require_arm_fault_clear',
    ):
        _boolean(value[field], f'safety.{field}')
    if value['reject_nonfinite_commands'] is not True:
        raise ConfigValidationError(
            'safety.reject_nonfinite_commands must remain true'
        )
    if float(root['web']['poll_interval_sec']) * 3.0 > float(
        value['operator_lease_timeout_sec']
    ):
        raise ConfigValidationError(
            'web.poll_interval_sec must allow at least three operator lease '
            'heartbeats before safety.operator_lease_timeout_sec'
        )
    motion = root['motion_control']
    commanded_linear = max(
        motion['max_linear_mps'],
        *(
            marker.get('max_linear_mps', motion['max_linear_mps'])
            for marker in root['markers'].values()
        ),
    )
    commanded_angular = max(
        motion['max_angular_rps'],
        root['turn_control']['max_angular_rps'],
        *(
            marker.get('max_angular_rps', motion['max_angular_rps'])
            for marker in root['markers'].values()
        ),
    )
    if motion['min_steering_rps'] > min(
        marker.get('max_angular_rps', motion['max_angular_rps'])
        for marker in root['markers'].values()
    ):
        raise ConfigValidationError(
            'motion_control.min_steering_rps cannot exceed a marker '
            'max_angular_rps'
        )
    if value['max_linear_mps'] < commanded_linear:
        raise ConfigValidationError(
            'safety.max_linear_mps is below a configured controller maximum'
        )
    if value['max_angular_rps'] < commanded_angular:
        raise ConfigValidationError(
            'safety.max_angular_rps is below a configured controller maximum'
        )


def _duration_fields(value: Mapping[str, Any], path: str) -> None:
    minimum = _number(
        value['duration_min_sec'], f'{path}.duration_min_sec', 0.01, 30.0
    )
    maximum = _number(
        value['duration_max_sec'], f'{path}.duration_max_sec', 0.01, 300.0
    )
    default = _number(
        value['default_duration_sec'],
        f'{path}.default_duration_sec',
        0.01,
        300.0,
    )
    if minimum > maximum or not minimum <= default <= maximum:
        raise ConfigValidationError(
            f'{path} durations must satisfy min <= default <= max'
        )
    _number(
        value['result_timeout_margin_sec'],
        f'{path}.result_timeout_margin_sec',
        0.0,
        60.0,
    )


def _joint_names(raw: Any, expected: tuple[str, ...], path: str) -> None:
    if not isinstance(raw, list) or tuple(raw) != expected:
        raise ConfigValidationError(
            f'{path} must exactly equal {list(expected)!r}'
        )


def _joint_limits(raw: Any, names: tuple[str, ...], path: str) -> None:
    limits = _mapping(raw, f'{path}.joint_limits_deg')
    _exact_fields(limits, set(names), f'{path}.joint_limits_deg')
    for name in names:
        pair = _number_list(
            limits[name],
            f'{path}.joint_limits_deg.{name}',
            exact_length=2,
        )
        if pair[0] >= pair[1]:
            raise ConfigValidationError(
                f'{path}.joint_limits_deg.{name} minimum must be below maximum'
            )


def _pose_positions(
    raw: Any,
    names: tuple[str, ...],
    limits: Mapping[str, Any],
    path: str,
) -> None:
    positions = _mapping(raw, path)
    _exact_fields(positions, set(names), path)
    for name in names:
        minimum, maximum = limits[name]
        _number(positions[name], f'{path}.{name}', minimum, maximum)


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigValidationError(f'{path} must be a mapping')
    return value


def _exact_fields(
    value: Mapping[str, Any],
    expected: set[str],
    path: str,
) -> None:
    actual = set(value)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing:
        raise ConfigValidationError(
            f'{path} is missing fields: {", ".join(missing)}'
        )
    if unknown:
        raise ConfigValidationError(
            f'{path} has unknown fields: {", ".join(unknown)}'
        )


def _fields_with_optional(
    value: Mapping[str, Any],
    allowed: set[str],
    optional: set[str],
    path: str,
) -> None:
    """Validate a strict mapping while allowing additive schema defaults."""
    actual = set(value)
    missing = sorted((allowed - optional) - actual)
    unknown = sorted(actual - allowed)
    if missing:
        raise ConfigValidationError(
            f'{path} is missing fields: {", ".join(missing)}'
        )
    if unknown:
        raise ConfigValidationError(
            f'{path} has unknown fields: {", ".join(unknown)}'
        )


def _nonempty_string(value: Any, path: str, *, maximum: int) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigValidationError(f'{path} must be a non-empty string')
    if len(value) > maximum:
        raise ConfigValidationError(f'{path} must be at most {maximum} characters')
    return value


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigValidationError(f'{path} must be a boolean')
    return value


def _revision(value: Any, path: str) -> int:
    return _integer(value, path, 0, 2**63 - 1)


def _positive_integer(value: Any, path: str) -> int:
    return _integer(value, path, 1, 2**63 - 1)


def _integer(
    value: Any,
    path: str,
    minimum: int,
    maximum: int,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
        or value > maximum
    ):
        raise ConfigValidationError(
            f'{path} must be an integer in [{minimum}, {maximum}]'
        )
    return value


def _number(
    value: Any,
    path: str,
    minimum: float,
    maximum: float,
) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < float(minimum)
        or float(value) > float(maximum)
    ):
        raise ConfigValidationError(
            f'{path} must be a finite number in [{minimum}, {maximum}]'
        )
    return float(value)


def _number_list(
    value: Any,
    path: str,
    *,
    exact_length: int | None = None,
    minimum_length: int | None = None,
) -> list[float]:
    if not isinstance(value, list):
        raise ConfigValidationError(f'{path} must be a list')
    if exact_length is not None and len(value) != exact_length:
        raise ConfigValidationError(
            f'{path} must contain exactly {exact_length} values'
        )
    if minimum_length is not None and len(value) < minimum_length:
        raise ConfigValidationError(
            f'{path} must contain at least {minimum_length} values'
        )
    return [
        _number(item, f'{path}[{index}]', -1.0e12, 1.0e12)
        for index, item in enumerate(value)
    ]
