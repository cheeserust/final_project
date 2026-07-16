"""Tests for the strict, revisioned presentation configuration store."""

import json
from pathlib import Path

from final_project_presentation2.config_store import ConfigConflictError
from final_project_presentation2.config_store import ConfigStore
from final_project_presentation2.config_store import ConfigStoreError
from final_project_presentation2.config_store import ConfigValidationError
from final_project_presentation2.config_store import load_json_document
from final_project_presentation2.config_store import validate_document

import pytest


TEMPLATE = (
    Path(__file__).parents[1]
    / 'config'
    / 'final_project_presentation2.json'
)


def _default_document():
    return load_json_document(TEMPLATE)


def _pose(category_id=1):
    return {
        'name': 'transport safe',
        'category_id': category_id,
        'arm': {
            'enabled': True,
            'positions_deg': {
                'base_joint': 0.0,
                'arm_joint_1': -20.0,
                'arm_joint_2': -40.0,
                'arm_joint_3': -60.0,
                'arm_joint_4': 0.0,
            },
            'duration_sec': 2.0,
        },
        'gripper': {
            'enabled': False,
            'positions_deg': {
                'finger_1_base_joint': 0.0,
                'finger_1_middle_joint': 0.0,
                'finger_1_tip_joint': 0.0,
                'finger_2_base_joint': 0.0,
                'finger_2_middle_joint': 0.0,
                'finger_2_tip_joint': 0.0,
                'finger_3_base_joint': 0.0,
                'finger_3_middle_joint': 0.0,
                'finger_3_tip_joint': 0.0,
            },
            'duration_sec': 1.0,
            'target_load_raw': 500,
        },
        'dwell_sec': 0.3,
    }


def test_template_is_complete_and_matches_selected_route_orientation():
    document = _default_document()

    assert document['schema_version'] == 1
    assert document['revision'] == 0
    assert document['aruco']['dictionary'] == 'DICT_4X4_1000'
    assert document['aruco']['marker_size_m'] == 0.05
    assert tuple(document['markers']) == ('pickup', 'dropoff')
    assert [document['markers'][name]['id'] for name in (
        'pickup', 'dropoff'
    )] == [1, 3]
    assert {
        document['markers'][name]['id']: document['markers'][name]['camera']
        for name in ('pickup', 'dropoff')
    } == {1: 'rear', 3: 'front'}
    assert document['route']['outbound']['camera'] == 'front'
    assert document['route']['outbound']['start_marker'] == 'pickup'
    assert document['route']['outbound']['destination_marker'] == 'dropoff'
    assert document['route']['outbound']['linear_direction'] == 1
    assert document['route']['return']['camera'] == 'rear'
    assert document['route']['return']['start_marker'] == 'dropoff'
    assert document['route']['return']['destination_marker'] == 'pickup'
    assert document['markers']['pickup']['target_distance_m'] == 0.90
    assert document['markers']['dropoff']['target_distance_m'] == 0.20
    assert document['markers']['pickup']['completion_mode'] == 'distance_only'
    assert document['markers']['dropoff']['completion_mode'] == 'distance_only'
    assert document['cameras']['front']['use_camera_info'] is False
    assert document['cameras']['rear']['use_camera_info'] is False
    assert document['timeouts']['marker_acquire_sec'] == 30.0
    assert document['timeouts']['acquire_creep_sec'] == 60.0
    assert document['timeouts']['marker_loss_sec'] == 1.0
    assert document['timeouts']['marker_observation_stale_sec'] == 0.25
    assert document['arm']['result_timeout_margin_sec'] == 10.0
    assert document['route']['return']['linear_direction'] == -1
    assert document['safety']['operator_lease_timeout_sec'] == 2.5
    assert document['safety']['require_arm_fault_clear'] is False
    assert len(document['arm']['joint_names']) == 5
    assert len(document['gripper']['joint_names']) == 9
    assert document['next_pose_id'] == 1
    assert 'require_drive_safe_pose' not in document['safety']
    assert 'drive_safe_pose_ids' not in document['safety']
    assert document['cameras']['front']['steering_sign'] == -1
    assert document['cameras']['rear']['steering_sign'] == -1
    assert document['motion_control']['linear_gate_angle_deg'] == 12.0
    assert document['motion_control']['stable_detections'] == 1
    assert document['motion_control']['acquire_creep_mps'] == 0.05
    assert document['motion_control']['steering_output_scale'] == 0.50
    assert document['motion_control']['steering_slow_band_ratio'] == 2.0
    assert document['motion_control']['alignment_hysteresis_ratio'] == 1.5
    assert document['timeouts']['alignment_sec'] == 120.0
    assert document['markers']['pickup']['lateral_tolerance_m'] == 0.07
    assert document['markers']['pickup']['yaw_tolerance_deg'] == 10.0
    assert document['poses'] == {}
    assert document['workflows'] == {}


def test_previous_live_config_accepts_new_arm_command_defaults():
    """An already seeded demo config must keep working after this update."""
    document = _default_document()
    for field in (
        'arm_enable_service',
        'arm_disable_service',
        'arm_home_service',
        'arm_clear_error_service',
        'arm_estop_service',
    ):
        del document['topics'][field]
    del document['timeouts']['arm_service_sec']
    del document['timeouts']['arm_home_sec']
    del document['timeouts']['acquire_creep_sec']
    del document['motion_control']['acquire_creep_mps']
    del document['motion_control']['steering_output_scale']
    del document['motion_control']['steering_slow_band_ratio']
    del document['motion_control']['alignment_hysteresis_ratio']
    for marker in document['markers'].values():
        del marker['completion_mode']
    validate_document(document)


def test_acquire_creep_cannot_exceed_normal_linear_limit():
    document = _default_document()
    document['motion_control']['acquire_creep_mps'] = 0.09

    with pytest.raises(ConfigValidationError, match='acquire_creep_mps'):
        validate_document(document)


def test_route_orientation_is_configurable_as_a_consistent_set():
    document = _default_document()
    document['markers']['pickup']['camera'] = 'front'
    document['markers']['dropoff']['camera'] = 'rear'
    document['route']['outbound']['camera'] = 'rear'
    document['route']['outbound']['linear_direction'] = -1
    document['route']['return']['camera'] = 'front'
    document['route']['return']['linear_direction'] = 1

    validate_document(document)


def test_route_semantic_combinations_do_not_block_loading():
    document = _default_document()
    document['route']['outbound']['camera'] = 'rear'
    document['route']['outbound']['start_marker'] = 'dropoff'
    document['route']['outbound']['destination_marker'] = 'pickup'
    document['route']['return']['camera'] = 'front'
    document['route']['return']['start_marker'] = 'pickup'
    document['route']['return']['destination_marker'] = 'dropoff'
    document['route']['return']['linear_direction'] = 1

    validate_document(document)


def test_route_keeps_only_minimum_runtime_shape_checks():
    document = _default_document()
    document['route']['outbound']['camera'] = 'side'
    with pytest.raises(ConfigValidationError, match='must be front or rear'):
        validate_document(document)

    document = _default_document()
    document['route']['outbound']['linear_direction'] = 0
    with pytest.raises(ConfigValidationError, match='must be -1 or 1'):
        validate_document(document)

    document = _default_document()
    document['route']['return']['destination_marker'] = 'missing'
    with pytest.raises(ConfigValidationError, match='configured marker'):
        validate_document(document)


def test_watchdog_transport_topics_are_fixed_safety_interfaces():
    document = _default_document()
    document['topics']['cmd_vel_raw'] = '/unsafe/raw'
    with pytest.raises(ConfigValidationError, match='fixed safety interface'):
        validate_document(document)


def test_dictionary_is_configurable_but_route_marker_ids_are_locked():
    document = _default_document()
    document['aruco']['dictionary'] = 'DICT_4X4_100'
    validate_document(document)

    document['markers']['dropoff']['id'] = 75
    with pytest.raises(ConfigValidationError, match='must remain 3'):
        validate_document(document)


def test_marker_control_overrides_are_optional():
    document = _default_document()
    for field in (
        'distance_tolerance_m',
        'lateral_tolerance_m',
        'yaw_tolerance_deg',
        'hold_time_sec',
        'max_linear_mps',
        'max_angular_rps',
    ):
        del document['markers']['pickup'][field]

    validate_document(document)


def test_marker_unknown_control_override_is_rejected():
    document = _default_document()
    document['markers']['pickup']['unexpected_override'] = 0.1

    with pytest.raises(ConfigValidationError, match='unknown fields'):
        validate_document(document)


@pytest.mark.parametrize(
    'mutator, message',
    [
        (
            lambda value: value['markers']['dropoff'].__setitem__('id', 1),
            'must remain 3',
        ),
        (
            lambda value: value['route']['outbound'].__setitem__(
                'turn_deg', 80.0
            ),
            'unknown fields',
        ),
        (
            lambda value: value['route']['return'].__setitem__(
                'corner_marker', 'pickup'
            ),
            'unknown fields',
        ),
        (
            lambda value: value['arm'].__setitem__(
                'joint_names', value['arm']['joint_names'][:-1]
            ),
            'must exactly equal',
        ),
        (
            lambda value: value['gripper']['joint_limits_deg'].pop(
                'finger_3_tip_joint'
            ),
            'missing fields',
        ),
        (
            lambda value: value['timeouts'].__setitem__(
                'marker_observation_stale_sec', 3.1
            ),
            'cannot exceed',
        ),
        (
            lambda value: value['safety'].__setitem__(
                'operator_lease_timeout_sec', 0.0
            ),
            'finite number',
        ),
        (
            lambda value: value['safety'].__setitem__(
                'require_drive_safe_pose', True
            ),
            'unknown fields',
        ),
        (
            lambda value: value['safety'].__setitem__(
                'watchdog_cmd_timeout_sec', 2.1
            ),
            'finite number',
        ),
        (
            lambda value: value['web'].__setitem__(
                'poll_interval_sec', 1.0
            ),
            'three operator lease heartbeats',
        ),
    ],
)
def test_strict_validation_rejects_unsafe_documents(mutator, message):
    document = _default_document()
    mutator(document)

    with pytest.raises(ConfigValidationError, match=message):
        validate_document(document)


def test_pose_and_workflow_references_are_validated():
    document = _default_document()
    document['poses']['1'] = _pose()
    document['poses']['2'] = _pose()
    document['next_pose_id'] = 3
    document['workflows']['1'] = {
        'name': 'object delivery',
        'category_id': 2,
        'steps': [
            {'type': 'POSE', 'pose_id': 2},
            {'type': 'POSE', 'pose_id': 1},
            {'type': 'GO_DROPOFF'},
            {'type': 'WAIT_RETURN_CONFIRM'},
            {'type': 'GO_PICKUP'},
        ],
    }
    document['next_workflow_id'] = 2
    validate_document(document)
    document['workflows']['1']['steps'][0]['pose_id'] = 999
    with pytest.raises(ConfigValidationError, match='missing pose 999'):
        validate_document(document)


def test_workflow_allows_any_operator_selected_pose_before_driving():
    document = _default_document()
    document['poses']['1'] = _pose()
    document['next_pose_id'] = 2
    document['workflows']['1'] = {
        'name': 'unsafe drive sequence',
        'category_id': 2,
        'steps': [
            {'type': 'POSE', 'pose_id': 1},
            {'type': 'GO_DROPOFF'},
        ],
    }
    document['next_workflow_id'] = 2

    validate_document(document)


def test_pose_joint_positions_must_be_complete_and_within_limits():
    document = _default_document()
    document['poses']['1'] = _pose()
    document['next_pose_id'] = 2
    del document['poses']['1']['arm']['positions_deg']['arm_joint_4']

    with pytest.raises(ConfigValidationError, match='missing fields'):
        validate_document(document)

    document = _default_document()
    document['poses']['1'] = _pose()
    document['next_pose_id'] = 2
    document['poses']['1']['arm']['positions_deg']['arm_joint_1'] = 100.0
    with pytest.raises(ConfigValidationError, match='finite number'):
        validate_document(document)


def test_initialize_snapshot_replace_and_mutate_are_revisioned(tmp_path):
    store = ConfigStore(tmp_path / 'live.json')
    initialized = store.initialize(_default_document())
    assert initialized['revision'] == 0

    detached = store.snapshot()
    detached['web']['port'] = 9090
    assert store.snapshot()['web']['port'] == 8080

    detached['web']['port'] = 9090
    replaced = store.replace(0, detached)
    assert replaced['revision'] == 1
    assert store.snapshot()['web']['port'] == 9090

    mutated = store.mutate(
        1,
        lambda value: value['web'].__setitem__('poll_interval_sec', 0.5),
    )
    assert mutated['revision'] == 2
    assert mutated['web']['poll_interval_sec'] == 0.5
    serialized = json.loads(store.path.read_text(encoding='utf-8'))
    assert serialized['revision'] == 2
    assert serialized['categories']['1']['name'] == '공통'
    assert not list(tmp_path.glob('*.bak'))
    assert not list(tmp_path.glob('*.lock'))

    with pytest.raises(ConfigConflictError, match='current revision 2'):
        store.mutate(1, lambda value: None)


def test_invalid_mutation_never_changes_the_authoritative_file(tmp_path):
    store = ConfigStore(tmp_path / 'live.json')
    store.initialize(_default_document())

    def invalidate(value):
        value['markers']['pickup']['id'] = 3

    with pytest.raises(ConfigValidationError, match='must remain 1'):
        store.mutate(0, invalidate)

    snapshot = store.snapshot()
    assert snapshot['revision'] == 0
    assert snapshot['markers']['pickup']['id'] == 1
    assert not list(tmp_path.glob('.live.json.*.tmp'))


def test_replace_rejects_document_revision_changed_by_client(tmp_path):
    store = ConfigStore(tmp_path / 'live.json')
    store.initialize(_default_document())
    candidate = store.snapshot()
    candidate['revision'] = 10

    with pytest.raises(ConfigConflictError, match='must equal'):
        store.replace(0, candidate)


def test_json_loader_rejects_duplicate_keys(tmp_path):
    duplicate = tmp_path / 'duplicate.json'
    duplicate.write_text(
        '{"schema_version": 1, "schema_version": 1}',
        encoding='utf-8',
    )

    with pytest.raises(ConfigStoreError, match='duplicate key'):
        load_json_document(duplicate)


def test_json_loader_rejects_nonfinite_numbers(tmp_path):
    invalid = tmp_path / 'nonfinite.json'
    text = json.dumps(_default_document(), ensure_ascii=False)
    invalid.write_text(
        text.replace('"port": 8080', '"port": NaN'),
        encoding='utf-8',
    )

    with pytest.raises(ConfigStoreError, match='non-finite JSON number'):
        load_json_document(invalid)


def test_next_ids_must_remain_monotonic():
    document = _default_document()
    document['poses']['1'] = _pose()

    with pytest.raises(ConfigValidationError, match='next_pose_id'):
        validate_document(document)
