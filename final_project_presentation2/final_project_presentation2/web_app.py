"""Flask façade for the standalone presentation node."""

from __future__ import annotations

import inspect
from pathlib import Path
from typing import Any, Callable

from flask import Flask, jsonify, request, Response, send_from_directory
from werkzeug.exceptions import HTTPException


class ApiError(RuntimeError):
    """An expected HTTP API failure with an explicit status code."""

    def __init__(self, message: str, status_code: int = 400, **details: Any):
        super().__init__(message)
        self.message = message
        self.status_code = int(status_code)
        self.details = details


def _json_body() -> dict[str, Any]:
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        raise ApiError('JSON object body is required', 400)
    return payload


def create_app(runtime: Any, static_dir: Path) -> Flask:
    """Create the HTTP API without coupling it to ROS implementation details."""
    app = Flask(
        __name__,
        static_folder=None,
    )

    def result(call: Callable[[], Any], status_code: int = 200):
        value = call()
        if value is None:
            value = {'ok': True}
        elif isinstance(value, dict) and 'ok' not in value:
            value = {'ok': True, **value}
        return jsonify(value), status_code

    def delete_payload() -> dict[str, Any]:
        """Read an optional revision body without making DELETE unusable."""
        payload = request.get_json(silent=True)
        if payload is None:
            revision = request.args.get('expected_revision')
            return {} if revision is None else {'expected_revision': revision}
        if not isinstance(payload, dict):
            raise ApiError('JSON object body is required', 400)
        return payload

    def revisioned_delete(
        method: Callable[..., Any],
        entity_id: int,
        payload: dict[str, Any],
    ) -> Any:
        """
        Call the revision-aware runtime API, with legacy compatibility.

        New runtimes accept ``(id, payload)`` and perform the revision check in
        the same atomic config mutation.  The compatibility branch keeps an
        older one-argument runtime usable and rejects an already-stale UI
        revision before invoking it.
        """
        parameters = inspect.signature(method).parameters
        if len(parameters) >= 2:
            return method(entity_id, payload)

        expected = payload.get('expected_revision')
        if expected is not None:
            try:
                expected_int = int(expected)
            except (TypeError, ValueError) as exc:
                raise ApiError(
                    'expected_revision must be an integer', 400
                ) from exc
            snapshot = runtime.snapshot()
            current = snapshot.get('revision') if isinstance(snapshot, dict) else None
            if current is None or int(current) != expected_int:
                raise ApiError(
                    'Configuration revision changed; refresh and try again',
                    409,
                    expected_revision=expected_int,
                    current_revision=current,
                )
        return method(entity_id)

    @app.errorhandler(ApiError)
    def handle_api_error(error: ApiError):
        return jsonify({
            'ok': False,
            'message': error.message,
            **error.details,
        }), error.status_code

    @app.errorhandler(HTTPException)
    def handle_http_error(error: HTTPException):
        return jsonify({
            'ok': False,
            'message': error.description,
        }), error.code

    @app.errorhandler(Exception)
    def handle_unexpected(error: Exception):
        runtime.report_http_exception(error)
        return jsonify({
            'ok': False,
            'message': f'Internal server error: {error}',
        }), 500

    @app.after_request
    def prevent_live_ui_caching(response: Response):
        if (
            request.path == '/'
            or request.path.startswith('/static/')
            or request.path.startswith('/api/camera/')
        ):
            response.headers['Cache-Control'] = (
                'no-store, no-cache, must-revalidate, max-age=0'
            )
            response.headers['Pragma'] = 'no-cache'
            response.headers['Expires'] = '0'
        return response

    @app.get('/')
    def index():
        return send_from_directory(static_dir, 'index.html')

    @app.get('/static/<path:name>')
    def static_asset(name: str):
        return send_from_directory(static_dir, name)

    @app.get('/<path:name>')
    def static_file(name: str):
        if name.startswith('api/'):
            raise ApiError('API endpoint was not found', 404)
        return send_from_directory(static_dir, name)

    @app.get('/api/snapshot')
    def snapshot():
        return result(runtime.snapshot)

    @app.post('/api/lease')
    def lease():
        return result(runtime.renew_operator_lease)

    @app.post('/api/stop')
    def stop():
        return result(runtime.stop)

    @app.post('/api/error/clear')
    def clear_error():
        return result(runtime.clear_error)

    @app.post('/api/arm/<command>')
    def arm_command(command: str):
        if command not in {
            'status', 'enable', 'home', 'disable', 'clear', 'estop'
        }:
            raise ApiError('Unknown arm command', 404)
        payload = request.get_json(silent=True)
        if payload is None:
            payload = {}
        if not isinstance(payload, dict):
            raise ApiError('JSON object body is required', 400)
        confirmed = payload.get('confirmed') is True
        if command == 'disable' and not confirmed:
            raise ApiError('Disable requires explicit confirmation', 400)
        return result(
            lambda: runtime.arm_command(command, confirmed=confirmed)
        )

    @app.post('/api/route/dropoff')
    def route_dropoff():
        return result(lambda: runtime.start_route('dropoff'), 202)

    @app.post('/api/route/pickup')
    def route_pickup():
        return result(lambda: runtime.start_route('pickup'), 202)

    @app.post('/api/route/confirm-return')
    def confirm_return():
        return result(runtime.confirm_return)

    @app.post('/api/route/continue')
    def continue_route():
        return result(runtime.continue_route, 202)

    @app.post('/api/route/discard-checkpoint')
    def discard_checkpoint():
        return result(runtime.discard_checkpoint)

    @app.post('/api/manual/turn')
    def manual_turn():
        payload = _json_body()
        return result(
            lambda: runtime.start_manual_turn(payload.get('degrees')),
            202,
        )

    @app.post('/api/test/marker/<marker_name>')
    def test_marker(marker_name: str):
        return result(lambda: runtime.start_marker_test(marker_name), 202)

    @app.get('/api/categories')
    def categories():
        return result(runtime.list_categories)

    @app.post('/api/categories')
    def create_category():
        payload = _json_body()
        return result(lambda: runtime.create_category(payload), 201)

    @app.patch('/api/categories/<int:category_id>')
    def update_category(category_id: int):
        payload = _json_body()
        return result(
            lambda: runtime.update_category(category_id, payload)
        )

    @app.delete('/api/categories/<int:category_id>')
    def delete_category(category_id: int):
        payload = delete_payload()
        return result(lambda: revisioned_delete(
            runtime.delete_category, category_id, payload
        ))

    @app.get('/api/poses')
    def poses():
        return result(runtime.list_poses)

    @app.post('/api/poses')
    def create_pose():
        payload = _json_body()
        return result(lambda: runtime.create_pose(payload), 201)

    @app.patch('/api/poses/<int:pose_id>')
    def update_pose(pose_id: int):
        payload = _json_body()
        return result(lambda: runtime.update_pose(pose_id, payload))

    @app.delete('/api/poses/<int:pose_id>')
    def delete_pose(pose_id: int):
        payload = delete_payload()
        return result(lambda: revisioned_delete(
            runtime.delete_pose, pose_id, payload
        ))

    @app.post('/api/poses/<int:pose_id>/execute')
    def execute_pose(pose_id: int):
        return result(lambda: runtime.start_pose(pose_id), 202)

    @app.post('/api/poses/preview')
    def preview_pose():
        payload = _json_body()
        return result(lambda: runtime.start_pose_preview(payload), 202)

    @app.get('/api/workflows')
    def workflows():
        return result(runtime.list_workflows)

    @app.post('/api/workflows')
    def create_workflow():
        payload = _json_body()
        return result(lambda: runtime.create_workflow(payload), 201)

    @app.patch('/api/workflows/<int:workflow_id>')
    def update_workflow(workflow_id: int):
        payload = _json_body()
        return result(
            lambda: runtime.update_workflow(workflow_id, payload)
        )

    @app.delete('/api/workflows/<int:workflow_id>')
    def delete_workflow(workflow_id: int):
        payload = delete_payload()
        return result(lambda: revisioned_delete(
            runtime.delete_workflow, workflow_id, payload
        ))

    @app.post('/api/workflows/<int:workflow_id>/run')
    def run_workflow(workflow_id: int):
        return result(lambda: runtime.start_workflow(workflow_id), 202)

    @app.post('/api/workflow/retry-step')
    def retry_workflow_step():
        return result(runtime.retry_workflow_step, 202)

    @app.post('/api/workflow/skip-step')
    def skip_workflow_step():
        return result(runtime.skip_workflow_step, 202)

    @app.post('/api/workflow/resume-at-step')
    def resume_workflow_at_step():
        payload = _json_body()
        if 'step_number' not in payload:
            raise ApiError('step_number is required', 400)
        return result(
            lambda: runtime.resume_workflow_at_step(
                payload['step_number']
            ),
            202,
        )

    @app.post('/api/workflow/abort-paused')
    def abort_paused_workflow():
        return result(runtime.abort_paused_workflow)

    @app.post('/api/config/reload')
    def reload_config():
        return result(runtime.reload_config)

    @app.get('/api/camera/<camera_name>.jpg')
    def camera_jpeg(camera_name: str):
        if camera_name not in {'front', 'rear'}:
            raise ApiError('camera_name must be front or rear', 404)
        jpeg = runtime.camera_jpeg(camera_name)
        if jpeg is None:
            raise ApiError(f'{camera_name} camera frame is unavailable', 503)
        return Response(jpeg, mimetype='image/jpeg')

    return app
