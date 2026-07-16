"""Pure validation and result helpers for ready-and-approach coordination."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Iterable


ARM_READY_TASK_NAME = 'arm_ready'
MAX_ABS_DISTANCE_M = 1.0
MAX_SPEED_MPS = 0.3


class ReadyAndApproachConfigError(ValueError):
    """Raised when a coordinator RunTask goal is unsafe or incomplete."""


@dataclass(frozen=True)
class ReadyAndApproachRequest:
    """Validated motion values extracted from a parent RunTask goal."""

    arm_task_name: str
    arm_start_to_drive_delay_sec: float
    distance_m: float
    speed_mps: float


@dataclass(frozen=True)
class ChildOutcome:
    """Normalized terminal result from one child action."""

    name: str
    success: bool
    message: str
    canceled: bool = False


@dataclass(frozen=True)
class CoordinationDecision:
    """Combined terminal decision for all child actions."""

    success: bool
    canceled: bool
    message: str


def _required_finite_number(payload: dict[str, Any], key: str) -> float:
    if key not in payload:
        raise ReadyAndApproachConfigError(
            f'extra_json.{key} is required'
        )

    value = payload[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ReadyAndApproachConfigError(
            f'extra_json.{key} must be a finite number'
        )

    converted = float(value)
    if not math.isfinite(converted):
        raise ReadyAndApproachConfigError(
            f'extra_json.{key} must be a finite number'
        )

    return converted


def parse_ready_and_approach_request(
    raw_extra_json: str,
) -> ReadyAndApproachRequest:
    """Validate and normalize one coordinator extra_json object."""
    try:
        payload = json.loads(raw_extra_json or '{}')
    except json.JSONDecodeError as exc:
        raise ReadyAndApproachConfigError(
            f'Invalid extra_json: {exc}'
        ) from exc

    if not isinstance(payload, dict):
        raise ReadyAndApproachConfigError(
            'extra_json must contain a JSON object'
        )

    arm_task_name = str(
        payload.get('arm_task_name', ARM_READY_TASK_NAME)
    ).strip()
    if arm_task_name != ARM_READY_TASK_NAME:
        raise ReadyAndApproachConfigError(
            'extra_json.arm_task_name must be arm_ready'
        )

    delay_sec = _required_finite_number(
        payload,
        'arm_start_to_drive_delay_sec',
    )
    distance_m = _required_finite_number(payload, 'distance_m')
    speed_mps = _required_finite_number(payload, 'speed_mps')

    if delay_sec < 0.0:
        raise ReadyAndApproachConfigError(
            'extra_json.arm_start_to_drive_delay_sec cannot be negative'
        )
    if distance_m == 0.0:
        raise ReadyAndApproachConfigError(
            'extra_json.distance_m cannot be zero'
        )
    if abs(distance_m) > MAX_ABS_DISTANCE_M:
        raise ReadyAndApproachConfigError(
            'extra_json.distance_m absolute value cannot exceed '
            f'{MAX_ABS_DISTANCE_M} m'
        )
    if speed_mps <= 0.0:
        raise ReadyAndApproachConfigError(
            'extra_json.speed_mps must be greater than zero'
        )
    if speed_mps > MAX_SPEED_MPS:
        raise ReadyAndApproachConfigError(
            f'extra_json.speed_mps cannot exceed {MAX_SPEED_MPS} m/s'
        )

    return ReadyAndApproachRequest(
        arm_task_name=arm_task_name,
        arm_start_to_drive_delay_sec=delay_sec,
        distance_m=distance_m,
        speed_mps=speed_mps,
    )


def child_extra_json(
    request: ReadyAndApproachRequest,
) -> tuple[str, str]:
    """Return isolated arm and base payloads for the two child goals."""
    arm_payload = json.dumps(
        {'arm_task_name': request.arm_task_name},
        sort_keys=True,
        separators=(',', ':'),
    )
    base_payload = json.dumps(
        {
            'distance_m': request.distance_m,
            'speed_mps': request.speed_mps,
        },
        sort_keys=True,
        separators=(',', ':'),
    )
    return arm_payload, base_payload


def combine_child_outcomes(
    outcomes: Iterable[ChildOutcome],
) -> CoordinationDecision:
    """Join child terminal states without hiding any failure reason."""
    collected = tuple(outcomes)
    if not collected:
        return CoordinationDecision(
            success=False,
            canceled=False,
            message='Ready-and-approach has no child outcomes',
        )

    failures = tuple(outcome for outcome in collected if not outcome.success)

    if failures:
        details = '; '.join(
            f'{outcome.name}: '
            f'{outcome.message or "child action did not succeed"}'
            for outcome in failures
        )
        return CoordinationDecision(
            success=False,
            canceled=any(outcome.canceled for outcome in failures),
            message=f'Ready-and-approach child failure: {details}',
        )

    names = ', '.join(outcome.name for outcome in collected)
    return CoordinationDecision(
        success=True,
        canceled=False,
        message=f'Ready-and-approach succeeded: {names}',
    )
