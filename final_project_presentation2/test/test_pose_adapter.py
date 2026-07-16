"""Tests for the flat UI to nested authoritative pose adapter."""

from copy import deepcopy
from pathlib import Path
import threading
import time
from types import SimpleNamespace

from final_project_presentation2.arm_executor import (
    PoseCancellationUnconfirmed,
)
from final_project_presentation2.config_store import (
    load_json_document,
    validate_document,
)
from final_project_presentation2.control_core import (
    VelocityCommand,
)
import final_project_presentation2.main_node as main_node_module
from final_project_presentation2.main_node import PresentationNode
from final_project_presentation2.main_node import RecoverableOperationError

import pytest


def _template():
    return load_json_document(
        Path(__file__).parents[1] / 'config' / 'final_project_presentation2.json'
    )


def _pose_document():
    node = object.__new__(PresentationNode)
    node._config = _template()
    return node._pose_from_api({
        'name': 'operator pose',
        'category_id': 1,
        'arm_enabled': True,
        'arm_positions_deg': [-80.0, -20.0, 10.0, 30.0, 0.0],
        'arm_duration_sec': 2.0,
        'gripper_enabled': False,
        'gripper_positions_deg': [0.0] * 9,
        'gripper_duration_sec': 1.0,
        'target_load_raw': 500,
        'dwell_sec': 0.3,
    })


def test_default_config_is_the_json_in_the_package_folder(monkeypatch):
    package_root = Path(__file__).parents[1]
    monkeypatch.setattr(
        main_node_module,
        'get_package_share_directory',
        lambda _package_name: str(package_root),
    )
    node = SimpleNamespace()

    path = PresentationNode._resolve_config_path(node)

    expected = (
        package_root / 'config' / 'final_project_presentation2.json'
    ).resolve()
    assert path == expected


def test_default_config_rejects_a_copied_non_source_install(
    monkeypatch,
    tmp_path,
):
    installed_share = tmp_path / 'share'
    installed_config = installed_share / 'config'
    installed_config.mkdir(parents=True)
    (installed_config / 'final_project_presentation2.json').write_text(
        '{}\n',
        encoding='utf-8',
    )
    monkeypatch.setattr(
        main_node_module,
        'get_package_share_directory',
        lambda _package_name: str(installed_share),
    )
    node = SimpleNamespace()

    with pytest.raises(RuntimeError, match='--symlink-install'):
        PresentationNode._resolve_config_path(node)


def test_pose_library_starts_empty_and_operator_pose_round_trips():
    """Joint order and the arm/gripper composition survive API conversion."""
    document = _template()
    assert document['poses'] == {}
    assert document['next_pose_id'] == 1

    api_pose = PresentationNode._pose_to_api(1, _pose_document())
    assert api_pose['id'] == 1
    assert api_pose['arm_enabled'] is True
    assert api_pose['gripper_enabled'] is False
    assert api_pose['arm_positions_deg'] == [
        -80.0,
        -20.0,
        10.0,
        30.0,
        0.0,
    ]


def test_source_timestamp_gate_rejects_queued_and_future_samples():
    """Only accept sensor headers close to the central ROS clock."""
    now_sec = 100.0
    clock = SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(now_sec * 1e9))
    )
    node = SimpleNamespace(
        get_clock=lambda: clock,
        _stamp_seconds=PresentationNode._stamp_seconds,
    )

    def stamp(seconds):
        whole = int(seconds)
        return SimpleNamespace(
            sec=whole,
            nanosec=int(round((seconds - whole) * 1e9)),
        )

    assert PresentationNode._source_stamp_is_fresh(node, stamp(99.8), 0.4)
    assert not PresentationNode._source_stamp_is_fresh(
        node, stamp(99.0), 0.4
    )
    assert not PresentationNode._source_stamp_is_fresh(
        node, stamp(100.2), 0.4
    )
    assert not PresentationNode._source_stamp_is_fresh(node, stamp(0.0), 0.4)


def test_missing_start_marker_is_advisory_and_does_not_block_departure():
    """A missing start marker must not block the straight destination run."""
    events = []
    node = SimpleNamespace(
        _config=_template(),
        _lock=threading.RLock(),
        _observations={},
        _set_command=lambda _command: None,
        _check_stop=lambda _event, **_kwargs: None,
        _record_event=lambda level, message: events.append((level, message)),
    )

    PresentationNode._verify_marker(node, 'pickup', threading.Event())

    assert events
    assert events[0][0] == 'warning'
    assert 'demo mode continues' in events[0][1]


@pytest.mark.parametrize(
    (
        'route_key',
        'start',
        'destination',
        'destination_id',
        'motion_sign',
    ),
    [
        ('outbound', 'pickup', 'dropoff', 3, 1.0),
        ('return', 'dropoff', 'pickup', 1, -1.0),
    ],
)
def test_route_runs_one_straight_destination_segment_without_a_turn(
    route_key,
    start,
    destination,
    destination_id,
    motion_sign,
):
    document = _template()
    assert document['markers'][destination]['id'] == destination_id
    calls = []
    node = object.__new__(PresentationNode)
    node._config = document
    node._lock = threading.RLock()
    node._route_direction = None
    node._checkpoint = None
    node._metrics = {
        'turn_progress_deg': -79.3,
        'turn_target_deg': -80.0,
    }
    node._motion_preflight = lambda _stop: calls.append(('preflight',))
    node._set_phase = lambda phase, **_metrics: calls.append(
        ('phase', phase)
    )
    node._verify_marker = lambda marker, _stop: calls.append(
        ('verify', marker)
    )

    def approach(**kwargs):
        calls.append((
            'approach',
            kwargs['marker_name'],
            kwargs['motion_sign'],
            deepcopy(node._checkpoint),
        ))

    node._run_visual_approach = approach
    node._run_exact_turn = lambda *_args, **_kwargs: pytest.fail(
        'straight routes must not call the turn controller'
    )

    PresentationNode._run_route(node, route_key, threading.Event())

    assert ('verify', start) in calls
    approach_call = next(call for call in calls if call[0] == 'approach')
    assert approach_call[:3] == ('approach', destination, motion_sign)
    assert approach_call[3] == {
        'route': route_key,
        'phase': 'destination_alignment',
        'marker': destination,
        'motion_sign': motion_sign,
    }
    assert sum(call[0] == 'approach' for call in calls) == 1
    assert node._checkpoint is None
    assert node._metrics['turn_progress_deg'] is None
    assert node._metrics['turn_target_deg'] is None


def test_manual_turn_publishes_new_positive_target_before_preflight():
    calls = []
    node = object.__new__(PresentationNode)
    node._metrics = {
        'turn_progress_deg': -79.3,
        'turn_target_deg': -80.0,
    }

    def set_phase(phase, **metrics):
        calls.append(('phase', phase, deepcopy(metrics)))
        node._metrics.update(metrics)

    def stop_at_preflight(_stop):
        calls.append(('preflight', deepcopy(node._metrics)))
        raise RuntimeError('stop after checking preflight telemetry')

    node._set_phase = set_phase
    node._turn_preflight = stop_at_preflight

    with pytest.raises(RuntimeError, match='preflight telemetry'):
        PresentationNode._run_exact_turn(
            node,
            80.0,
            threading.Event(),
            phase_name='exact_turn',
        )

    assert calls[0] == (
        'phase',
        'exact_turn',
        {'turn_progress_deg': 0.0, 'turn_target_deg': 80.0},
    )
    assert calls[1][0] == 'preflight'
    assert calls[1][1]['turn_progress_deg'] == 0.0
    assert calls[1][1]['turn_target_deg'] == 80.0


def test_straight_routes_have_no_corner_or_automatic_turn_fields():
    document = _template()
    assert set(document['markers']) == {'pickup', 'dropoff'}
    for route in document['route'].values():
        assert 'corner_marker' not in route
        assert 'turn_deg' not in route


def test_workflow_keeps_pose_id_order_and_inserts_straight_base_motion():
    calls = []
    node = object.__new__(PresentationNode)
    node._lock = threading.RLock()
    node._checkpoint = None
    node._workflow_progress = None
    node._workflow_recovery = None
    node._safety_abort_reason = None
    node._shutdown_event = threading.Event()
    node._pose_executor = SimpleNamespace(shutdown_drained=True)
    node._config = {
        'workflows': {
            '1': {
                'name': 'ordered arm and base demo',
                'steps': [
                    {'type': 'POSE', 'pose_id': 3},
                    {'type': 'POSE', 'pose_id': 1},
                    {'type': 'GO_DROPOFF'},
                    {'type': 'POSE', 'pose_id': 2},
                ],
            },
        },
    }
    node._check_stop = lambda _stop: None
    node._set_phase = lambda _phase: None
    node._record_event = lambda *_args: None
    node._execute_pose = lambda pose_id, _stop: calls.append(
        ('pose', pose_id)
    )
    node._run_route = lambda route, _stop: calls.append(('route', route))

    PresentationNode._run_workflow(node, '1', threading.Event())

    assert calls == [
        ('pose', '3'),
        ('pose', '1'),
        ('route', 'outbound'),
        ('pose', '2'),
    ]


def test_workflow_start_uses_normal_drive_readiness_without_a_pose_gate():
    document = _template()
    document['poses']['1'] = _pose_document()
    document['poses']['2'] = _pose_document()
    document['workflows']['1'] = {
        'name': 'arm then straight drive',
        'category_id': 1,
        'steps': [
            {'type': 'POSE', 'pose_id': 2},
            {'type': 'POSE', 'pose_id': 1},
            {'type': 'GO_DROPOFF'},
        ],
    }
    readiness_checks = []
    node = object.__new__(PresentationNode)
    node._config = document
    node._lock = threading.RLock()
    node._require_pose_ready = lambda _pose: None
    node._require_runtime_ready = (
        lambda ready, _reasons, _operation: readiness_checks.append(ready)
    )
    node._start_worker = lambda **_kwargs: {'started': True}

    result = PresentationNode.start_workflow(node, 1)

    assert result == {'started': True}
    assert readiness_checks == ['drive_ready']


@pytest.mark.parametrize(
    ('motion_sign', 'expected_linear'),
    [(1.0, 0.03), (-1.0, -0.03)],
)
def test_marker_acquire_creep_follows_route_direction(
    motion_sign,
    expected_linear,
):
    result = SimpleNamespace(
        command=VelocityCommand(),
        reason='target_marker_not_visible',
        complete=False,
        failed=False,
    )

    command, reason = PresentationNode._visual_approach_command(
        result=result,
        acquired=False,
        camera_fresh=True,
        motion_sign=motion_sign,
        acquire_creep_mps=0.03,
    )

    assert command.linear_x == pytest.approx(expected_linear)
    assert command.angular_z == 0.0
    assert reason == 'acquire_creep:target_marker_not_visible'


@pytest.mark.parametrize(
    ('reason',),
    [
        ('stabilizing_target_marker',),
        ('waiting_for_new_marker_frame',),
    ],
)
def test_marker_acquire_creep_continues_during_initial_detection(reason):
    result = SimpleNamespace(
        command=VelocityCommand(),
        reason=reason,
        complete=False,
        failed=False,
    )

    command, output_reason = PresentationNode._visual_approach_command(
        result=result,
        acquired=False,
        camera_fresh=True,
        motion_sign=1.0,
        acquire_creep_mps=0.03,
    )

    assert command.linear_x == pytest.approx(0.03)
    assert output_reason == f'acquire_creep:{reason}'


def test_marker_motion_stops_when_camera_is_not_fresh():
    result = SimpleNamespace(
        command=VelocityCommand(linear_x=0.08),
        reason='tracking_target_distance_only',
        complete=False,
        failed=False,
    )

    command, reason = PresentationNode._visual_approach_command(
        result=result,
        acquired=True,
        camera_fresh=False,
        motion_sign=1.0,
        acquire_creep_mps=0.03,
    )

    assert command.is_zero
    assert reason == 'camera_not_fresh'


def test_recoverable_marker_failure_returns_idle_without_error_latch():
    events = []
    zero_calls = []
    node = object.__new__(PresentationNode)
    node._lock = threading.RLock()
    node._safety_abort_reason = None
    node._shutdown_event = threading.Event()
    node._state = 'RUNNING'
    node._phase = 'align_pickup'
    node._active_kind = 'route'
    node._active_name = 'return'
    node._route_direction = 'return'
    node._error = 'old error'
    node._metrics = {'active_camera': 'front', 'target_marker_id': 1}
    node._workflow_progress = None
    node._workflow_recovery = None
    node._force_zero = lambda: zero_calls.append(True)
    node._clear_detection_metrics_locked = lambda: None
    node._record_event = lambda level, message: events.append((level, message))
    stop_event = threading.Event()

    def fail_recoverably(_stop_event):
        raise RecoverableOperationError('target marker acquire timeout')

    PresentationNode._worker_entry(node, fail_recoverably, stop_event)

    assert zero_calls
    assert node._state == 'IDLE'
    assert node._phase == 'incomplete'
    assert node._active_kind is None
    assert node._error is None
    assert any('may be retried' in message for _level, message in events)


def test_safety_abort_wins_over_recoverable_marker_failure():
    latched = []
    node = object.__new__(PresentationNode)
    node._lock = threading.RLock()
    node._safety_abort_reason = None
    node._shutdown_event = threading.Event()
    node._force_zero = lambda: None
    node._record_event = lambda *_args: None
    node._latch_worker_error = latched.append
    stop_event = threading.Event()

    def fail_during_safety_abort(_stop_event):
        node._safety_abort_reason = 'watchdog graph changed'
        raise RecoverableOperationError('target marker acquire timeout')

    PresentationNode._worker_entry(node, fail_during_safety_abort, stop_event)

    assert latched == ['watchdog graph changed']


def _workflow_test_node(document):
    node = object.__new__(PresentationNode)
    node._config = document
    node._lock = threading.RLock()
    node._checkpoint = None
    node._workflow_progress = None
    node._workflow_recovery = None
    node._safety_abort_reason = None
    node._shutdown_event = threading.Event()
    node._pose_executor = SimpleNamespace(shutdown_drained=True)
    node._return_confirm_event = threading.Event()
    node._operator_lease_at = 0.0
    node._check_stop = lambda _stop, **_kwargs: None
    node._set_phase = lambda phase, **_metrics: setattr(
        node, '_phase', phase
    )
    node._record_event = lambda *_args: None
    node._force_zero = lambda: None
    return node


def test_legacy_return_confirmation_step_does_not_wait_for_user_input():
    node = _workflow_test_node(_template())

    class UnexpectedReturnConfirmationAccess:
        def clear(self):
            raise AssertionError('return confirmation must not be cleared')

        def wait(self, _timeout):
            raise AssertionError('return confirmation must not be awaited')

    node._return_confirm_event = UnexpectedReturnConfirmationAccess()

    PresentationNode._run_workflow_step(
        node,
        step={'type': 'WAIT_RETURN_CONFIRM'},
        stop_event=threading.Event(),
    )


def test_workflow_step_failure_is_paused_with_retry_context():
    document = _template()
    document['workflows']['1'] = {
        'name': '데모 운반',
        'category_id': 1,
        'steps': [
            {'type': 'POSE', 'pose_id': 1},
            {'type': 'WAIT_SECONDS', 'seconds': 0.01},
        ],
    }
    node = _workflow_test_node(document)
    node._execute_pose = lambda _pose_id, _stop: (_ for _ in ()).throw(
        RuntimeError('temporary CAN response error')
    )

    with pytest.raises(RecoverableOperationError, match='paused at step 1/2'):
        PresentationNode._run_workflow(node, '1', threading.Event())

    assert node._workflow_recovery['workflow_id'] == 1
    assert node._workflow_recovery['step_number'] == 1
    assert node._workflow_recovery['step_type'] == 'POSE'
    assert 'temporary CAN' in node._workflow_recovery['error']
    assert node._workflow_progress['status'] == 'paused'
    assert node._workflow_progress['completed_steps'] == 0


def test_worker_returns_paused_workflow_to_idle_without_error_latch():
    document = _template()
    document['workflows']['1'] = {
        'name': '일시정지 통합 시험',
        'category_id': 1,
        'steps': [{'type': 'POSE', 'pose_id': 1}],
    }
    node = _workflow_test_node(document)
    node._state = 'RUNNING'
    node._phase = 'starting'
    node._active_kind = 'workflow'
    node._active_name = '1: 일시정지 통합 시험'
    node._route_direction = None
    node._error = None
    node._metrics = {'active_camera': None, 'target_marker_id': None}
    node._clear_detection_metrics_locked = lambda: None
    node._execute_pose = lambda _pose_id, _stop: (_ for _ in ()).throw(
        RuntimeError('temporary trajectory rejection')
    )

    PresentationNode._worker_entry(
        node,
        lambda stop: PresentationNode._run_workflow(node, '1', stop),
        threading.Event(),
    )

    assert node._state == 'IDLE'
    assert node._phase == 'workflow_paused_1_of_1'
    assert node._error is None
    assert node._active_kind is None
    assert node._workflow_recovery['step_number'] == 1


def test_resumed_workflow_continues_at_requested_step_and_keeps_skip_count():
    document = _template()
    document['workflows']['1'] = {
        'name': '재개 시험',
        'category_id': 1,
        'steps': [
            {'type': 'POSE', 'pose_id': 1},
            {'type': 'POSE', 'pose_id': 1},
            {'type': 'POSE', 'pose_id': 1},
        ],
    }
    node = _workflow_test_node(document)
    node._workflow_progress = {
        'workflow_id': 1,
        'workflow_name': '재개 시험',
        'total_steps': 3,
        'completed_steps': 1,
        'skipped_steps': [2],
        'status': 'paused',
    }
    executed = []
    node._execute_pose = lambda pose_id, _stop: executed.append(pose_id)

    PresentationNode._run_workflow(
        node,
        '1',
        threading.Event(),
        start_index=2,
    )

    assert executed == ['1']
    assert node._workflow_progress['completed_steps'] == 2
    assert node._workflow_progress['skipped_steps'] == [2]
    assert node._workflow_progress['status'] == 'completed_with_skips'


def test_workflow_route_retry_uses_destination_checkpoint_not_full_route():
    document = _template()
    node = _workflow_test_node(document)
    calls = []
    node._continue_destination = lambda route, checkpoint, _stop: calls.append(
        ('continue', route, checkpoint['marker'])
    )
    node._run_route = lambda route, _stop: calls.append(('full', route))
    checkpoint = {
        'route': 'outbound',
        'phase': 'destination_alignment',
        'marker': 'dropoff',
        'motion_sign': 1.0,
    }

    PresentationNode._run_workflow_step(
        node,
        step={'type': 'GO_DROPOFF'},
        stop_event=threading.Event(),
        resume_route_checkpoint=checkpoint,
    )

    assert calls == [('continue', 'outbound', 'dropoff')]


def test_retry_and_skip_buttons_resume_the_saved_workflow_index():
    document = _template()
    document['workflows']['1'] = {
        'name': '버튼 복구 시험',
        'category_id': 1,
        'steps': [
            {'type': 'GO_DROPOFF'},
            {'type': 'POSE', 'pose_id': 1},
        ],
    }
    checkpoint = {
        'route': 'outbound',
        'phase': 'destination_alignment',
        'marker': 'dropoff',
        'motion_sign': 1.0,
    }

    def make_node():
        node = _workflow_test_node(document)
        node._worker = None
        node._workflow_progress = {
            'workflow_id': 1,
            'workflow_name': '버튼 복구 시험',
            'total_steps': 2,
            'completed_steps': 0,
            'skipped_steps': [],
            'status': 'paused',
        }
        node._workflow_recovery = {
            'workflow_id': 1,
            'workflow_name': '버튼 복구 시험',
            'step_index': 0,
            'route_checkpoint': deepcopy(checkpoint),
        }
        node._checkpoint = deepcopy(checkpoint)
        node._require_workflow_step_ready = lambda _step: None
        calls = []
        node._run_workflow = lambda workflow_id, _stop, **kwargs: calls.append(
            ('run', workflow_id, kwargs)
        )

        def start_worker(**kwargs):
            calls.append((
                'start',
                kwargs['allow_checkpoint'],
                kwargs['allow_workflow_recovery'],
            ))
            kwargs['target'](threading.Event())
            return {'started': True}

        node._start_worker = start_worker
        node._record_event = lambda *_args: None
        return node, calls

    retry_node, retry_calls = make_node()
    PresentationNode.retry_workflow_step(retry_node)
    assert retry_node._workflow_recovery is None
    assert retry_node._checkpoint == checkpoint
    assert retry_calls[0] == ('start', True, True)
    assert retry_calls[1][0:2] == ('run', '1')
    assert retry_calls[1][2]['start_index'] == 0
    assert retry_calls[1][2]['resume_route_checkpoint'] == checkpoint

    skip_node, skip_calls = make_node()
    PresentationNode.skip_workflow_step(skip_node)
    assert skip_node._workflow_recovery is None
    assert skip_node._checkpoint is None
    assert skip_node._workflow_progress['skipped_steps'] == [1]
    assert skip_calls[0] == ('start', False, True)
    assert skip_calls[1][2]['start_index'] == 1
    assert skip_calls[1][2]['resume_route_checkpoint'] is None


def test_operator_selected_step_updates_completed_and_skipped_stage_lists():
    document = _template()
    document['workflows']['1'] = {
        'name': '선택 단계 복구 시험',
        'category_id': 1,
        'steps': [
            {'type': 'WAIT_SECONDS', 'seconds': 0.1},
            {'type': 'WAIT_SECONDS', 'seconds': 0.1},
            {'type': 'WAIT_SECONDS', 'seconds': 0.1},
            {'type': 'WAIT_SECONDS', 'seconds': 0.1},
        ],
    }

    def make_node():
        node = _workflow_test_node(document)
        node._worker = None
        node._workflow_progress = {
            'workflow_id': 1,
            'workflow_name': '선택 단계 복구 시험',
            'total_steps': 4,
            'completed_steps': 1,
            'completed_step_numbers': [1],
            'skipped_steps': [],
            'status': 'paused',
        }
        node._workflow_recovery = {
            'workflow_id': 1,
            'workflow_name': '선택 단계 복구 시험',
            'step_index': 1,
            'route_checkpoint': {'phase': 'destination_alignment'},
        }
        node._checkpoint = {'phase': 'destination_alignment'}
        calls = []
        node._run_workflow = lambda workflow_id, _stop, **kwargs: calls.append(
            ('run', workflow_id, kwargs)
        )

        def start_worker(**kwargs):
            calls.append((
                'start',
                kwargs['allow_checkpoint'],
                kwargs['allow_workflow_recovery'],
            ))
            kwargs['target'](threading.Event())
            return {'started': True}

        node._start_worker = start_worker
        return node, calls

    forward_node, forward_calls = make_node()
    PresentationNode.resume_workflow_at_step(forward_node, 4)

    assert forward_node._checkpoint is None
    assert forward_node._workflow_progress['completed_step_numbers'] == [1]
    assert forward_node._workflow_progress['completed_steps'] == 1
    assert forward_node._workflow_progress['skipped_steps'] == [2, 3]
    assert forward_calls[0] == ('start', False, True)
    assert forward_calls[1][2]['start_index'] == 3
    assert forward_calls[1][2]['resume_route_checkpoint'] is None

    rewind_node, rewind_calls = make_node()
    PresentationNode.resume_workflow_at_step(rewind_node, 1)

    assert rewind_node._workflow_progress['completed_step_numbers'] == []
    assert rewind_node._workflow_progress['completed_steps'] == 0
    assert rewind_node._workflow_progress['skipped_steps'] == []
    assert rewind_calls[1][2]['start_index'] == 0
    assert rewind_calls[1][2]['resume_route_checkpoint'] is None


def test_workflow_safety_abort_is_not_converted_to_manual_skip():
    document = _template()
    document['workflows']['1'] = {
        'name': '안전 오류 시험',
        'category_id': 1,
        'steps': [{'type': 'POSE', 'pose_id': 1}],
    }
    node = _workflow_test_node(document)

    def fail_for_safety(_pose_id, _stop):
        node._safety_abort_reason = 'watchdog graph changed'
        raise RuntimeError('watchdog graph changed')

    node._execute_pose = fail_for_safety

    with pytest.raises(RuntimeError, match='watchdog graph changed'):
        PresentationNode._run_workflow(node, '1', threading.Event())

    assert node._workflow_recovery is None


def test_flat_ui_pose_becomes_valid_nested_json():
    """A UI payload is stored in the strict schema without leaking UI keys."""
    document = _template()
    node = object.__new__(PresentationNode)
    node._config = document
    payload = {
        'name': '물병 집기',
        'category_id': 4,
        'arm_enabled': True,
        'arm_positions_deg': [-80, -20, 10, 30, 0],
        'arm_duration_sec': 2.0,
        'gripper_enabled': True,
        'gripper_positions_deg': [0] * 9,
        'gripper_duration_sec': 1.0,
        'target_load_raw': 500,
        'dwell_sec': 0.3,
    }
    nested = node._pose_from_api(payload)
    candidate = deepcopy(document)
    candidate['poses']['1'] = nested
    candidate['next_pose_id'] = 2
    validate_document(candidate)
    assert nested['arm']['positions_deg']['base_joint'] == -80
    assert nested['gripper']['enabled'] is True


def test_velocity_graph_requires_exclusive_raw_and_hardware_publishers():
    document = _template()
    raw_topic = document['topics']['cmd_vel_raw']
    hardware_topic = document['topics']['cmd_vel']
    central = SimpleNamespace(
        node_name='final_project_presentation2',
        topic_type='geometry_msgs/msg/TwistStamped',
    )
    watchdog_raw = SimpleNamespace(
        node_name='final_project_presentation2_watchdog',
        topic_type='geometry_msgs/msg/TwistStamped',
    )
    watchdog_hardware = SimpleNamespace(
        node_name='final_project_presentation2_watchdog',
        topic_type='geometry_msgs/msg/Twist',
    )
    node = SimpleNamespace(
        _lock=threading.RLock(),
        _config=document,
        get_subscriptions_info_by_topic=lambda _topic: [watchdog_raw],
        get_publishers_info_by_topic=lambda topic: (
            [central]
            if topic == raw_topic
            else ([watchdog_hardware] if topic == hardware_topic else [])
        ),
    )

    status = PresentationNode._velocity_graph_status(node)
    assert status['ready']

    node.get_publishers_info_by_topic = lambda topic: (
        [
            central,
            SimpleNamespace(
                node_name='teleop',
                topic_type='geometry_msgs/msg/TwistStamped',
            ),
        ]
        if topic == raw_topic
        else ([watchdog_hardware] if topic == hardware_topic else [])
    )
    status = PresentationNode._velocity_graph_status(node)
    assert not status['ready']
    assert status['raw_publisher_count'] == 2


def test_unconfirmed_pose_stop_is_an_error_not_normal_stop():
    document = _template()
    node = object.__new__(PresentationNode)
    node._config = document
    node._verify_arm_hardware = lambda _stop: None
    node._record_event = lambda *_args: None

    class UnconfirmedExecutor:

        def execute(self, *_args, **_kwargs):
            raise PoseCancellationUnconfirmed('terminal state unknown')

    node._pose_executor = UnconfirmedExecutor()
    stop_event = threading.Event()
    stop_event.set()

    with pytest.raises(RuntimeError, match='terminal state unknown'):
        node._execute_pose_document(_pose_document(), stop_event)


def test_publish_timer_cannot_reissue_motion_after_stop():
    document = _template()
    messages = []
    node = object.__new__(PresentationNode)
    node._lock = threading.RLock()
    node._config = document
    node._worker = None
    node._active_kind = 'route'
    node._state = 'RUNNING'
    node._desired_command = VelocityCommand(0.08, 0.20)
    node._desired_command_at = time.monotonic()
    node._operator_lease_at = time.monotonic()
    node._lease_expiry_triggered = False
    node._shutdown_event = threading.Event()
    node._stop_event = threading.Event()
    node._stop_event.set()
    node._return_confirm_event = threading.Event()
    node._safety_abort_reason = None
    node._raw_cmd_publisher = SimpleNamespace(publish=messages.append)
    node._pose_executor = SimpleNamespace(cancel_active=lambda **_kwargs: True)
    node._record_event = lambda *_args: None
    stamp = SimpleNamespace(sec=100, nanosec=0)
    ros_now = SimpleNamespace(to_msg=lambda: stamp)
    clock = SimpleNamespace(now=lambda: ros_now)
    node.get_clock = lambda: clock

    node._publish_command()

    assert len(messages) == 1
    assert messages[0].twist.linear.x == 0.0
    assert messages[0].twist.angular.z == 0.0
