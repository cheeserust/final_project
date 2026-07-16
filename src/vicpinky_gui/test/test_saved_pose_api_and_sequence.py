"""Focused API and sequence-safety tests for saved manual poses."""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import pytest

from vicpinky_gui.gui_node import VicPinkyGuiNode
from vicpinky_gui.saved_pose_store import SavedPoseStore


class FakeActionClient:
    """Small action-client fake with observable readiness and sends."""

    def __init__(self, *, ready: bool, goal_handle=None) -> None:
        self.ready = ready
        self.goal_handle = goal_handle
        self.server_is_ready_calls = 0
        self.wait_for_server_calls = 0
        self.sent_goals = []

    def server_is_ready(self) -> bool:
        self.server_is_ready_calls += 1
        return self.ready

    def wait_for_server(self, *, timeout_sec: float) -> bool:
        self.wait_for_server_calls += 1
        return self.ready

    def send_goal_async(self, goal, *, feedback_callback):
        self.sent_goals.append((goal, feedback_callback))
        if self.goal_handle is None:
            raise AssertionError('This fake was not configured to send goals')
        return ImmediateFuture(self.goal_handle)


class ImmediateFuture:
    """Future whose result is already available."""

    def __init__(self, value) -> None:
        self._value = value

    def done(self) -> bool:
        return True

    def result(self):
        return self._value

    def add_done_callback(self, callback) -> None:
        callback(self)


class ControlledFuture:
    """Thread-safe future completed explicitly by a test."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._done = False
        self._value = None
        self._callbacks = []

    def done(self) -> bool:
        with self._lock:
            return self._done

    def result(self):
        with self._lock:
            if not self._done:
                raise RuntimeError('future is not complete')
            return self._value

    def add_done_callback(self, callback) -> None:
        with self._lock:
            if self._done:
                call_now = True
            else:
                self._callbacks.append(callback)
                call_now = False
        if call_now:
            callback(self)

    def set_result(self, value) -> None:
        with self._lock:
            if self._done:
                raise RuntimeError('future is already complete')
            self._value = value
            self._done = True
            callbacks = list(self._callbacks)
            self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeGoalHandle:
    """Accepted goal handle with controllable result completion."""

    def __init__(self, result_future) -> None:
        self.accepted = True
        self.result_future = result_future
        self.cancel_calls = 0
        self.cancel_requested = threading.Event()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        self.cancel_requested.set()
        return ImmediateFuture(SimpleNamespace(goals_canceling=[self]))


def _controller_config() -> dict:
    return {
        'arm': {
            'label': 'Arm',
            'action_name': '/arm/follow_joint_trajectory',
            'default_duration_sec': 1.0,
            'joints': [
                {
                    'key': 'axis_1',
                    'joint_name': 'arm_joint_1',
                    'min_deg': -90.0,
                    'max_deg': 90.0,
                    'default_deg': 0.0,
                },
            ],
        },
        'gripper': {
            'label': 'Gripper',
            'action_name': '/gripper/follow_joint_trajectory',
            'default_duration_sec': 1.0,
            'default_target_load_raw': 500,
            'joints': [
                {
                    'key': 'finger_1',
                    'joint_name': 'finger_1_joint',
                    'min_deg': -40.0,
                    'max_deg': 40.0,
                    'default_deg': 0.0,
                },
            ],
        },
    }


def _pose_payload(
    name: str = '물건 내려놓기',
    *,
    arm_position: float = 12.5,
    gripper_position: float = -8.0,
    dwell_sec: float = 0.25,
) -> dict:
    return {
        'name': name,
        'dwell_sec': dwell_sec,
        'controllers': {
            'arm': {
                'positions_deg': {'axis_1': arm_position},
                'duration_sec': 1.0,
            },
            'gripper': {
                'positions_deg': {'finger_1': gripper_position},
                'duration_sec': 0.75,
                'target_load_raw': 600,
            },
        },
    }


def _action_result(status: int, error_code: int, error_string: str = ''):
    return SimpleNamespace(
        status=status,
        result=SimpleNamespace(
            error_code=error_code,
            error_string=error_string,
        ),
    )


@pytest.fixture
def gui_node(tmp_path: Path) -> VicPinkyGuiNode:
    """Build only the state needed by the HTTP/manual-sequence methods."""
    node = VicPinkyGuiNode.__new__(VicPinkyGuiNode)
    node._lock = threading.RLock()
    node._saved_pose_file = tmp_path / 'saved_poses.json'
    node._saved_pose_store = SavedPoseStore(node._saved_pose_file)
    node._saved_pose_store_error = None
    node._manual_controllers = _controller_config()
    node._manual_clients = {
        name: FakeActionClient(ready=False)
        for name in node._manual_controllers
    }
    node._manual_goal_handles = {
        name: None for name in node._manual_controllers
    }
    node._manual_last_commands = {
        name: None for name in node._manual_controllers
    }
    node._manual_feedback = {
        name: None for name in node._manual_controllers
    }
    node._mission_goal_handle = None
    node._mission_goal = None
    node._nav_goal_handle = None
    node._nav_goal = None
    node._sequence_state = node._idle_sequence_state()
    node._sequence_thread = None
    node._sequence_stop_event = threading.Event()
    node._sequence_cancel_lock = threading.Lock()
    node._sequence_cancel_requested = set()
    node._sequence_step_timeout_margin_s = 0.0
    node._sequence_step_result_timeout_s = 0.0
    node._request_timeout_s = 0.2
    node._home_service_timeout_s = 185.0
    node._clear_service_timeout_s = 15.0
    node._append_event_locked = Mock()
    node.get_logger = Mock(return_value=Mock())
    return node


@pytest.mark.parametrize(
    ('command', 'expected_timeout'),
    [('home_all', 185.0), ('clear_error', 15.0)],
)
def test_arm_service_uses_command_specific_timeout(
    gui_node,
    command,
    expected_timeout,
):
    response = SimpleNamespace(success=True, message='ok')
    future = object()
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = future
    gui_node._arm_clients = {command: client}
    gui_node._wait_for_future = Mock(return_value=response)
    gui_node._record_arm_command = Mock()

    result = gui_node.call_arm_service(command)

    assert result['ok'] is True
    gui_node._wait_for_future.assert_called_once_with(
        future,
        timeout_s=expected_timeout,
    )


def _test_client(node: VicPinkyGuiNode, tmp_path: Path):
    app = node._create_flask_app(tmp_path)
    app.testing = True
    return app.test_client()


def test_offline_saved_pose_crud_does_not_touch_ros_clients(
    gui_node,
    tmp_path,
):
    client = _test_client(gui_node, tmp_path)

    created_response = client.post(
        '/api/manual/poses',
        json=_pose_payload(),
    )
    assert created_response.status_code == 201
    created = created_response.get_json()['pose']
    assert created['id'] == 1
    assert created['name'] == '물건 내려놓기'
    assert created['controllers']['arm']['positions_deg'] == {
        'arm_joint_1': 12.5,
    }
    assert created['controllers']['gripper']['positions_deg'] == {
        'finger_1_joint': -8.0,
    }

    listed = client.get('/api/manual/poses')
    assert listed.status_code == 200
    assert listed.get_json()['poses'] == [created]

    loaded = client.get('/api/manual/poses/1')
    assert loaded.status_code == 200
    assert loaded.get_json()['pose'] == created

    updated = client.patch(
        '/api/manual/poses/1',
        json={'name': '수정된 자세', 'dwell_sec': 2.0},
    )
    assert updated.status_code == 200
    assert updated.get_json()['pose']['name'] == '수정된 자세'
    assert updated.get_json()['pose']['dwell_sec'] == 2.0
    assert updated.get_json()['pose']['controllers'] == created['controllers']

    edited_controllers = deepcopy(created['controllers'])
    edited_controllers['arm']['positions_deg']['arm_joint_1'] = 20.0
    edited_controllers['gripper']['target_load_raw'] = 777
    angle_updated = client.patch(
        '/api/manual/poses/1',
        json={'controllers': edited_controllers},
    )
    assert angle_updated.status_code == 200
    assert (
        angle_updated.get_json()['pose']['controllers']['arm']
        ['positions_deg']['arm_joint_1']
    ) == 20.0
    assert (
        angle_updated.get_json()['pose']['controllers']['gripper']
        ['target_load_raw']
    ) == 777

    deleted = client.delete('/api/manual/poses/1')
    assert deleted.status_code == 200
    assert deleted.get_json()['next_id'] == 1
    assert client.get('/api/manual/poses/1').status_code == 404
    snapshot = client.get('/api/manual/poses').get_json()
    assert snapshot['poses'] == []
    assert snapshot['next_id'] == 1

    recreated_payload = _pose_payload('빈 ID 재사용')
    recreated_payload['id'] = 1
    recreated = client.post('/api/manual/poses', json=recreated_payload)
    assert recreated.status_code == 201
    assert recreated.get_json()['pose']['id'] == 1

    moved = client.patch('/api/manual/poses/1', json={'id': 3})
    assert moved.status_code == 200
    assert moved.get_json()['pose']['id'] == 3
    assert moved.get_json()['next_id'] == 1

    for action_client in gui_node._manual_clients.values():
        assert action_client.server_is_ready_calls == 0
        assert action_client.wait_for_server_calls == 0
        assert action_client.sent_goals == []


def test_pose_normalization_uses_joint_names_and_validates_all_ranges(
    gui_node,
    tmp_path,
):
    normalized = gui_node._normalize_saved_pose_fields(_pose_payload())
    assert normalized['controllers']['arm']['positions_deg'] == {
        'arm_joint_1': 12.5,
    }
    assert normalized['controllers']['gripper'] == {
        'positions_deg': {'finger_1_joint': -8.0},
        'duration_sec': 0.75,
        'target_load_raw': 600,
    }

    invalid_payloads = []

    payload = _pose_payload(arm_position=90.001)
    invalid_payloads.append(payload)

    payload = _pose_payload()
    payload['controllers']['arm']['duration_sec'] = 30.001
    invalid_payloads.append(payload)

    payload = _pose_payload()
    payload['controllers']['gripper']['target_load_raw'] = 1024
    invalid_payloads.append(payload)

    payload = _pose_payload(dwell_sec=-0.001)
    invalid_payloads.append(payload)

    payload = _pose_payload()
    payload['controllers']['arm']['positions_deg']['axis_1'] = True
    invalid_payloads.append(payload)

    payload = _pose_payload()
    payload['controllers']['arm']['duration_sec'] = True
    invalid_payloads.append(payload)

    payload = _pose_payload()
    payload['controllers']['gripper']['target_load_raw'] = 600.5
    invalid_payloads.append(payload)

    payload = _pose_payload()
    payload['controllers']['gripper']['target_load_raw'] = True
    invalid_payloads.append(payload)

    client = _test_client(gui_node, tmp_path)
    for invalid_payload in invalid_payloads:
        response = client.post('/api/manual/poses', json=invalid_payload)
        assert response.status_code == 400, response.get_json()
        assert response.get_json()['ok'] is False

    assert gui_node._saved_pose_store.snapshot()['poses'] == []


def test_arm_and_gripper_preserve_configured_duration(gui_node):
    arm_goal, arm_summary = gui_node._manual_goal_from_payload('arm', {
        'positions_deg': {'axis_1': 12.5},
        'duration_sec': 2.0,
    })

    assert arm_goal.duration_ms == 2000
    assert arm_summary['duration_sec'] == 2.0

    _, gripper_summary = gui_node._manual_goal_from_payload('gripper', {
        'positions_deg': {'finger_1': -8.0},
        'duration_sec': 2.0,
        'target_load_raw': 600,
    })
    assert gripper_summary['duration_sec'] == 2.0


def test_disable_requires_literal_true_before_calling_service(
    gui_node,
    tmp_path,
):
    gui_node.call_arm_service = Mock(return_value={
        'ok': True,
        'command': 'disable',
        'message': 'disabled',
    })
    client = _test_client(gui_node, tmp_path)

    assert client.post('/api/arm/disable').status_code == 400
    assert client.post(
        '/api/arm/disable',
        json={'confirmed': False},
    ).status_code == 400
    assert client.post(
        '/api/arm/disable',
        json={'confirmed': 1},
    ).status_code == 400
    gui_node.call_arm_service.assert_not_called()

    confirmed = client.post(
        '/api/arm/disable',
        json={'confirmed': True},
    )
    assert confirmed.status_code == 200
    gui_node.call_arm_service.assert_called_once_with('disable')


def test_offline_sequence_preflight_is_503_and_sends_no_goal(
    gui_node,
    tmp_path,
):
    assert gui_node.create_saved_pose(_pose_payload())['ok'] is True
    client = _test_client(gui_node, tmp_path)

    response = client.post('/api/manual/sequence/start', json={})

    assert response.status_code == 503
    body = response.get_json()
    assert body['ok'] is False
    assert {item['controller'] for item in body['offline_controllers']} == {
        'arm',
        'gripper',
    }
    assert gui_node.manual_sequence_snapshot()['state'] == 'IDLE'
    for action_client in gui_node._manual_clients.values():
        assert action_client.wait_for_server_calls == 1
        assert action_client.sent_goals == []


def test_sequence_validates_ids_and_preserves_selected_order(
    gui_node,
    tmp_path,
):
    assert gui_node.create_saved_pose(_pose_payload('첫 자세'))['ok'] is True
    assert gui_node.create_saved_pose(_pose_payload('둘째 자세'))['ok'] is True
    client = _test_client(gui_node, tmp_path)

    duplicate = client.post(
        '/api/manual/sequence/start',
        json={'pose_ids': [1, 1]},
    )
    assert duplicate.status_code == 400
    assert 'Duplicate pose ID' in duplicate.get_json()['message']

    missing = client.post(
        '/api/manual/sequence/start',
        json={'pose_ids': [99]},
    )
    assert missing.status_code == 404
    assert '99' in missing.get_json()['message']

    gui_node._manual_clients = {
        name: FakeActionClient(ready=True)
        for name in gui_node._manual_controllers
    }
    captured_ids = []
    worker_finished = threading.Event()

    def capture_worker(_run_id, poses, _stop_event):
        captured_ids.extend(pose['id'] for pose in poses)
        worker_finished.set()

    gui_node._run_manual_sequence = capture_worker
    started = client.post(
        '/api/manual/sequence/start',
        json={'pose_ids': [2, 1]},
    )

    assert started.status_code == 200
    assert started.get_json()['sequence']['pose_ids'] == [2, 1]
    assert worker_finished.wait(timeout=1.0)
    assert captured_ids == [2, 1]
    for action_client in gui_node._manual_clients.values():
        assert action_client.sent_goals == []


def test_sequence_step_dispatches_all_controllers_and_waits_for_success(
    gui_node,
):
    success = _action_result(
        GoalStatus.STATUS_SUCCEEDED,
        FollowJointTrajectory.Result.SUCCESSFUL,
    )
    handles = {
        name: FakeGoalHandle(ImmediateFuture(success))
        for name in gui_node._manual_controllers
    }
    gui_node._manual_clients = {
        name: FakeActionClient(ready=True, goal_handle=handles[name])
        for name in gui_node._manual_controllers
    }
    run_id = 'manual-sequence-success'
    gui_node._sequence_state.update({
        'run_id': run_id,
        'state': 'RUNNING',
        'active': True,
    })
    pose = {
        **gui_node._normalize_saved_pose_fields(_pose_payload()),
        'id': 1,
    }

    outcome = gui_node._execute_sequence_step(
        run_id,
        pose,
        threading.Event(),
    )

    assert outcome == ('SUCCEEDED', None)
    assert all(
        len(client.sent_goals) == 1
        for client in gui_node._manual_clients.values()
    )
    assert all(
        handle is None
        for handle in gui_node._manual_goal_handles.values()
    )


def test_failed_controller_keeps_peer_blocking_until_peer_is_terminal(
    gui_node,
):
    """A cancel acknowledgement alone must not release the moving peer."""
    failed_result = ImmediateFuture(_action_result(
        GoalStatus.STATUS_ABORTED,
        FollowJointTrajectory.Result.INVALID_GOAL,
        'arm failed',
    ))
    peer_result = ControlledFuture()
    arm_handle = FakeGoalHandle(failed_result)
    peer_handle = FakeGoalHandle(peer_result)
    gui_node._manual_clients = {
        'arm': FakeActionClient(ready=True, goal_handle=arm_handle),
        'gripper': FakeActionClient(ready=True, goal_handle=peer_handle),
    }
    run_id = 'manual-sequence-peer-cancel'
    gui_node._sequence_state.update({
        'run_id': run_id,
        'state': 'RUNNING',
        'active': True,
    })
    pose = {
        **gui_node._normalize_saved_pose_fields(_pose_payload()),
        'id': 1,
    }
    outcome = []
    returned = threading.Event()

    def execute_step():
        outcome.append(gui_node._execute_sequence_step(
            run_id,
            pose,
            threading.Event(),
        ))
        returned.set()

    worker = threading.Thread(target=execute_step, daemon=True)
    worker.start()
    assert peer_handle.cancel_requested.wait(timeout=1.0)
    try:
        assert gui_node._manual_goal_handles['gripper'] is peer_handle
        assert not returned.wait(timeout=0.05), (
            'sequence released a peer after cancel acknowledgement but before '
            'its terminal result'
        )
    finally:
        peer_result.set_result(_action_result(
            GoalStatus.STATUS_CANCELED,
            FollowJointTrajectory.Result.SUCCESSFUL,
            'canceled',
        ))

    assert returned.wait(timeout=1.0)
    worker.join(timeout=1.0)
    assert outcome[0][0] == 'FAILED'
    assert gui_node._manual_goal_handles['gripper'] is None


def test_sequence_worker_retains_unresolved_peer_after_cancel_timeout(
    gui_node,
):
    """A bounded wait may fail, but must not release an unresolved goal."""
    gui_node._request_timeout_s = 0.05
    failed_result = ImmediateFuture(_action_result(
        GoalStatus.STATUS_ABORTED,
        FollowJointTrajectory.Result.INVALID_GOAL,
        'arm failed',
    ))
    peer_result = ControlledFuture()
    arm_handle = FakeGoalHandle(failed_result)
    peer_handle = FakeGoalHandle(peer_result)
    gui_node._manual_clients = {
        'arm': FakeActionClient(ready=True, goal_handle=arm_handle),
        'gripper': FakeActionClient(ready=True, goal_handle=peer_handle),
    }
    run_id = 'manual-sequence-unresolved-peer'
    gui_node._sequence_state.update({
        'run_id': run_id,
        'state': 'STARTING',
        'active': True,
        'total': 1,
        'total_steps': 1,
    })
    pose = {
        **gui_node._normalize_saved_pose_fields(_pose_payload()),
        'id': 1,
    }
    worker = threading.Thread(
        target=gui_node._run_manual_sequence,
        args=(run_id, [pose], threading.Event()),
        daemon=True,
    )
    worker.start()

    assert peer_handle.cancel_requested.wait(timeout=1.0)
    worker.join(timeout=1.0)
    assert not worker.is_alive()
    assert gui_node._sequence_state['state'] == 'FAILED'
    assert gui_node._manual_goal_handles['gripper'] is peer_handle
    assert gui_node._manual_sequence_conflict_locked() is not None

    peer_result.set_result(_action_result(
        GoalStatus.STATUS_CANCELED,
        FollowJointTrajectory.Result.SUCCESSFUL,
        'canceled after timeout',
    ))
    assert gui_node._manual_goal_handles['gripper'] is None


def test_late_send_resolution_never_leaves_stale_sending_state(gui_node):
    run_id = 'manual-sequence-late-send'
    gui_node._sequence_state.update({
        'run_id': run_id,
        'state': 'FAILED',
        'active': False,
    })
    gui_node._manual_last_commands['gripper'] = {
        'state': 'SENDING',
        'run_id': run_id,
        'pose_id': 7,
    }

    gui_node._cancel_late_sequence_goal(
        run_id,
        'gripper',
        ImmediateFuture(SimpleNamespace(accepted=False)),
    )
    assert gui_node._manual_last_commands['gripper']['state'] == 'REJECTED'
    assert not gui_node._manual_motion_active_locked()

    peer_result = ControlledFuture()
    peer_handle = FakeGoalHandle(peer_result)
    gui_node._manual_last_commands['gripper'] = {
        'state': 'SENDING',
        'run_id': run_id,
        'pose_id': 7,
    }
    gui_node._cancel_late_sequence_goal(
        run_id,
        'gripper',
        ImmediateFuture(peer_handle),
    )

    assert peer_handle.cancel_requested.is_set()
    assert gui_node._manual_goal_handles['gripper'] is peer_handle
    assert gui_node._manual_last_commands['gripper']['state'] == 'CANCELING'
    assert gui_node._manual_motion_active_locked()

    peer_result.set_result(_action_result(
        GoalStatus.STATUS_CANCELED,
        FollowJointTrajectory.Result.SUCCESSFUL,
        'late goal canceled',
    ))
    assert gui_node._manual_goal_handles['gripper'] is None
    assert gui_node._manual_last_commands['gripper']['state'] == 'CANCELED'
    assert not gui_node._manual_motion_active_locked()


def test_result_future_setup_failure_is_contained_and_cancels_peers(gui_node):
    canceled_result = ImmediateFuture(_action_result(
        GoalStatus.STATUS_CANCELED,
        FollowJointTrajectory.Result.SUCCESSFUL,
        'canceled after peer setup error',
    ))
    broken_handle = FakeGoalHandle(ControlledFuture())
    broken_handle.get_result_async = Mock(
        side_effect=RuntimeError('cannot observe action result'),
    )
    peer_handle = FakeGoalHandle(canceled_result)
    gui_node._manual_clients = {
        'arm': FakeActionClient(ready=True, goal_handle=broken_handle),
        'gripper': FakeActionClient(ready=True, goal_handle=peer_handle),
    }
    run_id = 'manual-sequence-result-future-error'
    gui_node._sequence_state.update({
        'run_id': run_id,
        'state': 'STARTING',
        'active': True,
        'total': 2,
        'total_steps': 2,
    })
    pose = {
        **gui_node._normalize_saved_pose_fields(_pose_payload()),
        'id': 1,
    }
    next_pose = {
        **gui_node._normalize_saved_pose_fields(_pose_payload('다음 자세')),
        'id': 2,
    }

    gui_node._run_manual_sequence_safe(
        run_id,
        [pose, next_pose],
        threading.Event(),
    )

    assert gui_node._sequence_state['state'] == 'FAILED'
    assert 'cannot observe action result' in gui_node._sequence_state['error']
    assert broken_handle.cancel_calls == 1
    assert peer_handle.cancel_calls == 1
    assert gui_node._manual_goal_handles['arm'] is broken_handle
    assert gui_node._manual_goal_handles['gripper'] is None
    assert all(
        len(client.sent_goals) == 1
        for client in gui_node._manual_clients.values()
    )


def test_manual_send_timeout_blocks_sequence_until_late_goal_is_terminal(
    gui_node,
):
    gui_node._request_timeout_s = 0.02
    delayed_send = ControlledFuture()
    action_client = FakeActionClient(ready=True)
    action_client.send_goal_async = Mock(return_value=delayed_send)
    gui_node._manual_clients['arm'] = action_client

    response = gui_node.send_manual_pose(
        'arm',
        _pose_payload()['controllers']['arm'],
    )

    assert response['status_code'] == 504
    assert gui_node._manual_motion_active_locked()
    assert gui_node._manual_sequence_conflict_locked() is not None

    result_future = ControlledFuture()
    late_handle = FakeGoalHandle(result_future)
    delayed_send.set_result(late_handle)

    assert late_handle.cancel_requested.is_set()
    assert gui_node._manual_goal_handles['arm'] is late_handle
    assert gui_node._manual_motion_active_locked()
    assert gui_node._manual_sequence_conflict_locked() is not None

    result_future.set_result(_action_result(
        GoalStatus.STATUS_CANCELED,
        FollowJointTrajectory.Result.SUCCESSFUL,
        'late manual goal canceled',
    ))
    assert gui_node._manual_goal_handles['arm'] is None
    assert gui_node._manual_last_commands['arm']['state'] == 'CANCELED'
    assert not gui_node._manual_motion_active_locked()


def test_identical_active_arm_target_is_deduplicated(gui_node):
    result_future = ControlledFuture()
    goal_handle = FakeGoalHandle(result_future)
    action_client = FakeActionClient(ready=True, goal_handle=goal_handle)
    gui_node._manual_clients['arm'] = action_client
    payload = {
        'positions_deg': {'axis_1': 12.5},
        'duration_sec': 1.0,
    }

    first = gui_node.send_manual_pose('arm', payload)
    duplicate = gui_node.send_manual_pose('arm', deepcopy(payload))

    assert first['ok']
    assert duplicate == {
        'ok': True,
        'controller': 'arm',
        'queued': False,
        'deduplicated': True,
        'message': 'Identical arm target is already active',
        'goal': first['goal'],
    }
    assert len(action_client.sent_goals) == 1
    assert goal_handle.cancel_calls == 0
    diagnostics = gui_node.manual_latest_target_snapshot()
    assert diagnostics['controllers']['arm']['pending_latest_target'] is None
    assert not diagnostics['controllers']['arm']['cancel_requested']

    result_future.set_result(_action_result(
        GoalStatus.STATUS_SUCCEEDED,
        FollowJointTrajectory.Result.SUCCESSFUL,
    ))


def test_canceling_arm_keeps_only_latest_target_and_dispatches_it(gui_node):
    first_result = ControlledFuture()
    first_handle = FakeGoalHandle(first_result)
    second_result = ControlledFuture()
    second_handle = FakeGoalHandle(second_result)
    action_client = FakeActionClient(ready=True, goal_handle=first_handle)
    gui_node._manual_clients['arm'] = action_client

    first = gui_node.send_manual_pose('arm', {
        'positions_deg': {'axis_1': 10.0},
        'duration_sec': 1.0,
    })
    queued_intermediate = gui_node.send_manual_pose('arm', {
        'positions_deg': {'axis_1': 20.0},
        'duration_sec': 1.0,
    })
    queued_latest = gui_node.send_manual_pose('arm', {
        'positions_deg': {'axis_1': 30.0},
        'duration_sec': 1.0,
    })

    assert first['ok']
    assert queued_intermediate['queued']
    assert queued_latest['queued']
    assert not queued_latest['deduplicated']
    assert first_handle.cancel_calls == 1
    assert len(action_client.sent_goals) == 1
    diagnostics = gui_node.manual_latest_target_snapshot()
    arm_diagnostics = diagnostics['controllers']['arm']
    assert arm_diagnostics['cancel_requested']
    assert (
        arm_diagnostics['pending_latest_target']['target']['positions_deg']
        == {'axis_1': 30.0}
    )

    action_client.goal_handle = second_handle
    first_result.set_result(_action_result(
        GoalStatus.STATUS_CANCELED,
        FollowJointTrajectory.Result.SUCCESSFUL,
        'replaced by latest target',
    ))

    assert len(action_client.sent_goals) == 2
    dispatched_goal, _feedback_callback = action_client.sent_goals[1]
    assert dispatched_goal.positions == pytest.approx([
        math.radians(30.0),
    ])
    assert gui_node._manual_goal_handles['arm'] is second_handle
    assert gui_node._manual_last_commands['arm']['positions_deg'] == {
        'axis_1': 30.0,
    }
    diagnostics = gui_node.manual_latest_target_snapshot()
    assert diagnostics['controllers']['arm']['pending_latest_target'] is None
    assert not diagnostics['controllers']['arm']['cancel_requested']

    second_result.set_result(_action_result(
        GoalStatus.STATUS_SUCCEEDED,
        FollowJointTrajectory.Result.SUCCESSFUL,
    ))
    assert gui_node._manual_goal_handles['arm'] is None


def test_nav_late_accept_is_canceled_and_identity_cleared(gui_node):
    request_id = 'nav-timeout-request'
    result_future = ControlledFuture()
    late_handle = FakeGoalHandle(result_future)
    gui_node._nav_goal = {
        'request_id': request_id,
        'state': 'SEND_TIMEOUT_PENDING',
    }
    gui_node._nav_result = None

    gui_node._nav_late_send_callback(
        request_id,
        ImmediateFuture(late_handle),
    )

    assert late_handle.cancel_requested.is_set()
    assert gui_node._nav_goal_handle is late_handle
    assert gui_node._nav_goal['state'] == 'CANCELING'
    assert gui_node._nav_motion_active_locked()
    assert gui_node._manual_sequence_conflict_locked() is not None

    result_future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED,
        result=SimpleNamespace(
            success=False,
            message='late navigation goal canceled',
        ),
    ))

    assert gui_node._nav_goal_handle is None
    assert gui_node._nav_goal['state'] == 'CANCELED'
    assert not gui_node._nav_motion_active_locked()


def test_sequence_worker_stops_at_first_failure_and_interrupts_dwell(
    gui_node,
):
    run_id = 'manual-sequence-worker'
    poses = [
        {'id': 1, 'name': '첫 자세', 'dwell_sec': 0.0},
        {'id': 2, 'name': '둘째 자세', 'dwell_sec': 0.0},
        {'id': 3, 'name': '셋째 자세', 'dwell_sec': 0.0},
    ]
    gui_node._sequence_state.update({
        'run_id': run_id,
        'state': 'STARTING',
        'active': True,
        'total': len(poses),
        'total_steps': len(poses),
    })
    visited = []

    def execute(_run_id, pose, _stop_event):
        visited.append(pose['id'])
        if pose['id'] == 2:
            return 'FAILED', 'deterministic failure'
        return 'SUCCEEDED', None

    gui_node._execute_sequence_step = execute
    gui_node._run_manual_sequence(run_id, deepcopy(poses), threading.Event())

    assert visited == [1, 2]
    assert gui_node._sequence_state['state'] == 'FAILED'
    assert gui_node._sequence_state['completed_count'] == 1
    assert gui_node._sequence_state['error'] == 'deterministic failure'

    dwell_run_id = 'manual-sequence-dwell'
    dwell_started = threading.Event()
    stop_event = threading.Event()
    gui_node._sequence_state = gui_node._idle_sequence_state()
    gui_node._sequence_state.update({
        'run_id': dwell_run_id,
        'state': 'STARTING',
        'active': True,
        'total': 1,
        'total_steps': 1,
    })
    gui_node._execute_sequence_step = Mock(return_value=('SUCCEEDED', None))

    def observe_event(**kwargs):
        if kwargs.get('event_type') == 'MANUAL_SEQUENCE_DWELL_START':
            dwell_started.set()

    gui_node._append_event_locked = Mock(side_effect=observe_event)
    dwell_worker = threading.Thread(
        target=gui_node._run_manual_sequence,
        args=(
            dwell_run_id,
            [{'id': 1, 'name': '대기 자세', 'dwell_sec': 30.0}],
            stop_event,
        ),
        daemon=True,
    )
    dwell_worker.start()
    assert dwell_started.wait(timeout=1.0)
    stop_event.set()
    dwell_worker.join(timeout=1.0)

    assert not dwell_worker.is_alive()
    assert gui_node._sequence_state['state'] == 'CANCELED'
    assert gui_node._sequence_state['completed_count'] == 0
