"""HTTP contract tests for the standalone presentation UI."""

from pathlib import Path

from final_project_presentation2.web_app import create_app


class FakeRuntime:
    """Small recorder implementing the methods exercised by these tests."""

    def __init__(self) -> None:
        self.calls = []

    def report_http_exception(self, error):
        self.calls.append(('error', str(error)))

    def snapshot(self):
        return {'state': {'name': 'IDLE'}, 'revision': 3}

    def start_pose_preview(self, payload):
        self.calls.append(('preview', payload))
        return {'started': True}

    def start_marker_test(self, name):
        self.calls.append(('marker', name))
        return {'started': True}

    def arm_command(self, command, *, confirmed=False):
        self.calls.append(('arm_command', command, confirmed))
        return {'command': command, 'success': True}

    def camera_jpeg(self, name):
        return b'jpeg' if name == 'front' else None

    def retry_workflow_step(self):
        self.calls.append(('retry_workflow_step',))
        return {'started': True}

    def skip_workflow_step(self):
        self.calls.append(('skip_workflow_step',))
        return {'started': True}

    def resume_workflow_at_step(self, step_number):
        self.calls.append(('resume_workflow_at_step', step_number))
        return {'started': True}

    def abort_paused_workflow(self):
        self.calls.append(('abort_paused_workflow',))
        return {'aborted': True}


class RevisionDeleteRuntime(FakeRuntime):
    """Recorder for the revision-aware delete contract."""

    def delete_category(self, category_id, payload):
        self.calls.append(('delete_category', category_id, payload))
        return {'deleted_id': category_id, 'revision': 4}

    def delete_pose(self, pose_id, payload):
        self.calls.append(('delete_pose', pose_id, payload))
        return {'deleted_id': pose_id, 'revision': 4}

    def delete_workflow(self, workflow_id, payload):
        self.calls.append(('delete_workflow', workflow_id, payload))
        return {'deleted_id': workflow_id, 'revision': 4}


def _client():
    static_dir = Path(__file__).parents[1] / 'static'
    runtime = FakeRuntime()
    app = create_app(runtime, static_dir)
    app.testing = True
    return app.test_client(), runtime


def _client_for(runtime):
    static_dir = Path(__file__).parents[1] / 'static'
    app = create_app(runtime, static_dir)
    app.testing = True
    return app.test_client()


def test_static_assets_and_snapshot_are_served():
    """The installed UI paths and primary polling endpoint stay compatible."""
    client, _runtime = _client()
    index = client.get('/')
    script = client.get('/static/app.js')
    assert index.status_code == 200
    assert script.status_code == 200
    assert 'no-store' in index.headers['Cache-Control']
    assert 'no-store' in script.headers['Cache-Control']
    response = client.get('/api/snapshot')
    assert response.status_code == 200
    assert response.get_json()['state']['name'] == 'IDLE'


def test_unsaved_pose_and_marker_test_contracts():
    """UI-only preview and segment-test endpoints pass their payloads through."""
    client, runtime = _client()
    payload = {'name': 'preview', 'arm_enabled': True}
    assert client.post('/api/poses/preview', json=payload).status_code == 202
    assert client.post('/api/test/marker/dropoff').status_code == 202
    assert runtime.calls == [
        ('preview', payload),
        ('marker', 'dropoff'),
    ]


def test_camera_endpoint_rejects_unknown_or_missing_frames():
    """Camera previews return explicit status codes instead of broken JSON."""
    client, _runtime = _client()
    front = client.get('/api/camera/front.jpg')
    assert front.status_code == 200
    assert front.data == b'jpeg'
    rear = client.get('/api/camera/rear.jpg')
    side = client.get('/api/camera/side.jpg')
    assert rear.status_code == 503
    assert side.status_code == 404
    for response in (front, rear, side):
        assert 'no-store' in response.headers['Cache-Control']


def test_only_disable_requires_explicit_confirmation():
    client, runtime = _client()
    for command in ('status', 'enable', 'home', 'clear', 'estop'):
        assert client.post(f'/api/arm/{command}', json={}).status_code == 200
    assert client.post('/api/arm/disable', json={}).status_code == 400
    assert client.post(
        '/api/arm/disable', json={'confirmed': True}
    ).status_code == 200
    assert runtime.calls == [
        ('arm_command', 'status', False),
        ('arm_command', 'enable', False),
        ('arm_command', 'home', False),
        ('arm_command', 'clear', False),
        ('arm_command', 'estop', False),
        ('arm_command', 'disable', True),
    ]


def test_paused_workflow_recovery_endpoints_are_manual_and_immediate():
    client, runtime = _client()

    assert client.post('/api/workflow/retry-step').status_code == 202
    assert client.post('/api/workflow/skip-step').status_code == 202
    assert client.post('/api/workflow/abort-paused').status_code == 200
    assert runtime.calls == [
        ('retry_workflow_step',),
        ('skip_workflow_step',),
        ('abort_paused_workflow',),
    ]


def test_paused_workflow_can_resume_at_an_operator_selected_step():
    client, runtime = _client()

    response = client.post(
        '/api/workflow/resume-at-step',
        json={'step_number': 4},
    )

    assert response.status_code == 202
    assert runtime.calls == [('resume_workflow_at_step', 4)]
    assert client.post(
        '/api/workflow/resume-at-step', json={}
    ).status_code == 400


def test_framework_404_and_405_keep_their_http_status():
    """Generic exception handling must not turn routing failures into 500s."""
    client, runtime = _client()
    missing = client.get('/missing-static-file.txt')
    wrong_method = client.post('/api/snapshot')
    assert missing.status_code == 404
    assert wrong_method.status_code == 405
    assert not [call for call in runtime.calls if call[0] == 'error']


def test_revision_payload_is_forwarded_for_all_delete_mutations():
    runtime = RevisionDeleteRuntime()
    client = _client_for(runtime)
    payload = {'expected_revision': 3}
    assert client.delete('/api/categories/2', json=payload).status_code == 200
    assert client.delete('/api/poses/4', json=payload).status_code == 200
    assert client.delete('/api/workflows/6', json=payload).status_code == 200
    assert runtime.calls == [
        ('delete_category', 2, payload),
        ('delete_pose', 4, payload),
        ('delete_workflow', 6, payload),
    ]


def test_ui_contains_revision_and_checkpoint_safety_contracts():
    """A light static check protects critical browser-only safeguards."""
    app_js = (
        Path(__file__).parents[1] / 'static' / 'app.js'
    ).read_text(encoding='utf-8')
    index = (
        Path(__file__).parents[1] / 'static' / 'index.html'
    ).read_text(encoding='utf-8')
    app_css = (
        Path(__file__).parents[1] / 'static' / 'app.css'
    ).read_text(encoding='utf-8')
    assert 'withExpectedRevision' in app_js
    assert 'dialog.returnValue = ""' in app_js
    assert 'data-checkpoint-allowed' in index
    assert 'restartRequiredPanel' in index
    assert '집기 20 시험' not in index
    assert 'latestVisibleMarker(snapshot)' in app_js
    assert 'idleMarker?.camera_name' in app_js
    assert '<meta name="color-scheme" content="light">' in index
    assert 'color-scheme: light;' in app_css
    assert '<meta name="color-scheme" content="dark">' not in index
    assert app_js.count('confirmAction(') == 2
    assert 'command === "disable"' in app_js
    assert 'manualArmExecute' in index
    assert 'manualGripperExecute' in index
    assert 'id="clearErrorTopbar"' in index
    assert '$("#clearErrorTopbar").addEventListener("click", clearLatchedError)' in app_js
    assert index.index('id="fillCurrentJoints"') > index.index('id="armFieldset"')
    assert index.index('id="fillCurrentJoints"') < index.index('class="joint-table"')
    assert '>현재값으로 바꾸기</button>' in index
    assert 'workflowProgressPanel' in index
    assert 'panel.hidden = false' in app_js
    assert 'id="workflowProgressPanel" class="workflow-progress-panel workflow-stage-panel" aria-labelledby="workflowProgressTitle" hidden' not in index
    assert '/static/app.js?v=20260714-workflow-actions-enabled' in index
    assert 'id="confirmReturn"' not in index
    assert '<option value="WAIT_RETURN_CONFIRM">' not in index
    assert 'retryWorkflowStep' in index
    assert 'skipWorkflowStep' in index
    assert 'resumeWorkflowAtStep' in index
    assert 'abortPausedWorkflow' in index
    assert (
        'id="workflowRecoveryActions" class="workflow-recovery-actions" '
        'hidden' not in index
    )
    assert 'recoveryActions.hidden = false' in app_js
    assert 'data-lock="workflow-recovery"' not in index
    assert "$$('[data-lock=\"workflow-recovery\"]')" not in app_js
    assert '/workflow/retry-step' in app_js
    assert '/workflow/skip-step' in app_js
    assert '/workflow/resume-at-step' in app_js
    assert '/workflow/abort-paused' in app_js
    assert 'renderWorkflowProgress()' in app_js
    assert 'armBoardGrid' in index
    assert 'armStatusRefresh' in index
    assert 'renderArmBoardStatus(snapshot)' in app_js
    assert 'markerTargetDiagnostic' in index
    assert 'vision.target_status' in app_js
    assert 'acquire_creep:' in app_js
    assert '저속 이동 중' in app_js
    assert 'routeRobot' in index
    assert 'renderRouteRobot(snapshot, run, currentPhase)' in app_js
    assert '.route-robot' in app_css
    assert '실제 위치는 오차가 있을 수 있음' not in index
    assert 'pickup: { x: 89, y: 50 }' in app_js
    assert 'dropoff: { x: 11, y: 50 }' in app_js
    assert '.route-node.pickup { top: 30px; right: 0;' in app_css
    assert '.route-node.dropoff { top: 30px; left: 0;' in app_css
    assert 'workflowExecutionSteps' in index
    assert 'workflow-execution-step.completed' in app_css
    assert 'workflow-execution-step.failed' in app_css
    assert 'fault !== 0' not in app_js
    assert 'boardMetric("Limit bits"' in app_js
    assert 'A → C · 마커 3 직선 전진·거리 정지' in app_js
    assert 'C → A · 마커 1 직선 후진·거리 정지' in app_js
    assert '마커 1 → 마커 3 직선 전진' in index
    assert '마커 3 → 마커 1 직선 후진' in index
    assert index.count('data-marker-test=') == 2
    assert 'corner_outbound' not in app_js + index
    assert 'corner_return' not in app_js + index
    assert 'outboundTurnValue' not in app_js + index
    assert 'returnTurnValue' not in app_js + index
