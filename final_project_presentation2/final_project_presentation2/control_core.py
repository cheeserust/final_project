"""
ROS-independent motion controllers for the presentation route.

The controllers in this module intentionally know nothing about ROS messages or
Nav2.  Callers provide timestamped odometry/marker samples and receive a small,
finite velocity command.  This makes the route logic deterministic and easy to
stop when an input becomes stale.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Optional


def _finite(value: Any, name: str) -> float:
    """Return *value* as a finite float or raise a useful error."""
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be a finite number')
    return result


def _positive(value: Any, name: str) -> float:
    """Validate a finite number greater than zero."""
    result = _finite(value, name)
    if result <= 0.0:
        raise ValueError(f'{name} must be greater than zero')
    return result


def _nonnegative(value: Any, name: str) -> float:
    """Validate a finite number greater than or equal to zero."""
    result = _finite(value, name)
    if result < 0.0:
        raise ValueError(f'{name} must be nonnegative')
    return result


def _sign(value: Any, name: str) -> float:
    """Validate a configuration sign and normalize it to -1 or +1."""
    result = _finite(value, name)
    if result == 0.0:
        raise ValueError(f'{name} must not be zero')
    return math.copysign(1.0, result)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a finite value to an inclusive range."""
    checked = _finite(value, 'value')
    return max(lower, min(upper, checked))


def bounded_scale(value: float, scale: float, limit: float) -> float:
    """Multiply and saturate without allowing floating-point overflow."""
    checked_value = _finite(value, 'value')
    checked_scale = _finite(scale, 'scale')
    checked_limit = _positive(limit, 'limit')
    if checked_value == 0.0 or checked_scale == 0.0:
        return 0.0
    sign = math.copysign(1.0, checked_value) * math.copysign(
        1.0, checked_scale
    )
    if abs(checked_value) >= checked_limit / abs(checked_scale):
        return sign * checked_limit
    return checked_value * checked_scale


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    angle = _finite(angle_rad, 'angle_rad')
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class VelocityCommand:
    """A planar base command that can never contain NaN or infinity."""

    linear_x: float = 0.0
    angular_z: float = 0.0

    def __post_init__(self) -> None:
        """Validate both command components."""
        object.__setattr__(
            self, 'linear_x', _finite(self.linear_x, 'linear_x')
        )
        object.__setattr__(
            self, 'angular_z', _finite(self.angular_z, 'angular_z')
        )

    @property
    def is_zero(self) -> bool:
        """Return whether this is an exact stop command."""
        return self.linear_x == 0.0 and self.angular_z == 0.0


STOP_COMMAND = VelocityCommand()


class ControlStatus(str, Enum):
    """Lifecycle state shared by both route controllers."""

    IDLE = 'idle'
    WAITING = 'waiting'
    RUNNING = 'running'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


@dataclass(frozen=True)
class ControlResult:
    """One controller decision and optional alignment telemetry."""

    status: ControlStatus
    command: VelocityCommand = STOP_COMMAND
    reason: str = ''
    distance_error_m: Optional[float] = None
    lateral_error_m: Optional[float] = None
    yaw_error_rad: Optional[float] = None
    progress_rad: Optional[float] = None
    remaining_rad: Optional[float] = None

    @property
    def complete(self) -> bool:
        """Return true only after the controller's success condition."""
        return self.status is ControlStatus.SUCCEEDED

    @property
    def failed(self) -> bool:
        """Return true after a terminal safety or timeout failure."""
        return self.status is ControlStatus.FAILED


class YawUnwrapper:
    """Accumulate relative yaw while safely crossing the +/-pi boundary."""

    def __init__(self) -> None:
        """Create an unwrapper with no initial yaw sample."""
        self._last_yaw: Optional[float] = None
        self._relative_yaw = 0.0

    @property
    def relative_yaw(self) -> float:
        """Return accumulated rotation since the last reset."""
        return self._relative_yaw

    def reset(self, yaw_rad: float) -> None:
        """Start a new relative-angle measurement at *yaw_rad*."""
        self._last_yaw = normalize_angle(yaw_rad)
        self._relative_yaw = 0.0

    def update(self, yaw_rad: float) -> float:
        """Add the shortest step from the preceding wrapped yaw sample."""
        current = normalize_angle(yaw_rad)
        if self._last_yaw is None:
            self.reset(current)
            return self._relative_yaw
        delta = normalize_angle(current - self._last_yaw)
        self._relative_yaw += delta
        self._last_yaw = current
        return self._relative_yaw


@dataclass(frozen=True)
class TurnControlConfig:
    """Tuning and safety limits for a relative odometry turn."""

    kp: float = 1.8
    max_angular_rps: float = 0.65
    min_angular_rps: float = 0.08
    tolerance_deg: float = 1.5
    settle_time_sec: float = 0.20
    timeout_sec: float = 8.0
    odom_stale_sec: float = 0.25

    def __post_init__(self) -> None:
        """Validate controller tuning and safety limits."""
        object.__setattr__(self, 'kp', _positive(self.kp, 'kp'))
        maximum = _positive(self.max_angular_rps, 'max_angular_rps')
        minimum = _nonnegative(self.min_angular_rps, 'min_angular_rps')
        if minimum > maximum:
            raise ValueError(
                'min_angular_rps must not exceed max_angular_rps'
            )
        object.__setattr__(self, 'max_angular_rps', maximum)
        object.__setattr__(self, 'min_angular_rps', minimum)
        object.__setattr__(
            self, 'tolerance_deg', _positive(
                self.tolerance_deg, 'tolerance_deg'
            )
        )
        object.__setattr__(
            self, 'settle_time_sec', _nonnegative(
                self.settle_time_sec, 'settle_time_sec'
            )
        )
        object.__setattr__(
            self, 'timeout_sec', _positive(self.timeout_sec, 'timeout_sec')
        )
        object.__setattr__(
            self, 'odom_stale_sec', _positive(
                self.odom_stale_sec, 'odom_stale_sec'
            )
        )


class ExactAngleTurnController:
    """
    Turn through the configured relative odometry angle exactly.

    Marker visibility is accepted as telemetry by :meth:`update` so callers do
    not need a special call path while processing camera frames.  It is
    deliberately ignored for completion: only odometry angle and settle time
    can complete the turn.
    """

    def __init__(self, config: TurnControlConfig) -> None:
        """Create an idle controller using immutable tuning."""
        if not isinstance(config, TurnControlConfig):
            raise TypeError('config must be a TurnControlConfig')
        self.config = config
        self._unwrapper = YawUnwrapper()
        self._target_rad = 0.0
        self._started_at: Optional[float] = None
        self._settled_since: Optional[float] = None
        self._terminal: Optional[ControlResult] = None

    @property
    def active(self) -> bool:
        """Return whether a nonterminal turn is in progress."""
        return self._started_at is not None and self._terminal is None

    @property
    def target_angle_rad(self) -> float:
        """Return the active signed relative target."""
        return self._target_rad

    def start(
        self,
        current_yaw_rad: float,
        target_angle_deg: float,
        now_sec: float,
    ) -> None:
        """Start a new signed relative turn from the current odometry yaw."""
        now = _finite(now_sec, 'now_sec')
        target_deg = _finite(target_angle_deg, 'target_angle_deg')
        if target_deg == 0.0:
            raise ValueError('target_angle_deg must not be zero')
        if abs(target_deg) > 720.0:
            raise ValueError('target_angle_deg must be within +/-720 degrees')
        self._unwrapper.reset(current_yaw_rad)
        self._target_rad = math.radians(target_deg)
        self._started_at = now
        self._settled_since = None
        self._terminal = None

    def stop(self, reason: str = 'stopped') -> ControlResult:
        """Stop an active turn and latch a failure result."""
        self._terminal = ControlResult(
            ControlStatus.FAILED, STOP_COMMAND, str(reason)
        )
        return self._terminal

    def update(
        self,
        current_yaw_rad: float,
        now_sec: float,
        odom_timestamp_sec: Optional[float] = None,
        marker_visible: bool = False,
    ) -> ControlResult:
        """
        Compute the next angular command from an odometry sample.

        ``marker_visible`` is intentionally unused.  A target marker appearing
        before the requested angle is reached must never shorten the turn.
        """
        del marker_visible
        if self._terminal is not None:
            return self._terminal
        if self._started_at is None:
            return ControlResult(
                ControlStatus.IDLE, STOP_COMMAND, 'turn_not_started'
            )

        try:
            now = _finite(now_sec, 'now_sec')
            stamp = now if odom_timestamp_sec is None else _finite(
                odom_timestamp_sec, 'odom_timestamp_sec'
            )
            yaw = _finite(current_yaw_rad, 'current_yaw_rad')
        except ValueError as exc:
            return self.stop(f'invalid_odometry:{exc}')

        if now < self._started_at:
            return self.stop('clock_moved_backwards')
        if stamp > now + 0.05:
            return self.stop('odometry_timestamp_in_future')
        if now - stamp > self.config.odom_stale_sec:
            return self.stop('odometry_stale')
        if now - self._started_at > self.config.timeout_sec:
            return self.stop('turn_timeout')

        progress = self._unwrapper.update(yaw)
        remaining = self._target_rad - progress
        tolerance = math.radians(self.config.tolerance_deg)

        if abs(remaining) <= tolerance:
            if self._settled_since is None:
                self._settled_since = now
            if now - self._settled_since >= self.config.settle_time_sec:
                self._terminal = ControlResult(
                    ControlStatus.SUCCEEDED,
                    STOP_COMMAND,
                    'target_angle_reached',
                    progress_rad=progress,
                    remaining_rad=remaining,
                )
                return self._terminal
            return ControlResult(
                ControlStatus.RUNNING,
                STOP_COMMAND,
                'settling_at_target_angle',
                progress_rad=progress,
                remaining_rad=remaining,
            )

        self._settled_since = None
        speed = bounded_scale(
            remaining,
            self.config.kp,
            self.config.max_angular_rps,
        )
        if abs(speed) < self.config.min_angular_rps:
            speed = math.copysign(self.config.min_angular_rps, remaining)
        return ControlResult(
            ControlStatus.RUNNING,
            VelocityCommand(angular_z=speed),
            'turning_to_relative_angle',
            progress_rad=progress,
            remaining_rad=remaining,
        )


@dataclass(frozen=True)
class VisualApproachConfig:
    """Marker alignment targets, signs, gains, and safety timeouts."""

    target_marker_id: int
    camera_name: str
    motion_sign: float
    target_distance_m: float
    target_lateral_m: float = 0.0
    target_yaw_rad: float = 0.0
    distance_tolerance_m: float = 0.025
    lateral_tolerance_m: float = 0.025
    yaw_tolerance_rad: float = math.radians(4.0)
    linear_kp: float = 0.7
    lateral_kp: float = 1.6
    yaw_kp: float = 0.8
    lateral_error_sign: float = 1.0
    yaw_error_sign: float = 1.0
    angular_command_sign: float = 1.0
    max_linear_mps: float = 0.12
    max_angular_rps: float = 0.45
    linear_gate_angle_rad: float = math.radians(12.0)
    min_steering_rps: float = 0.05
    steering_output_scale: float = 1.0
    steering_slow_band_ratio: float = 2.0
    alignment_hysteresis_ratio: float = 1.5
    distance_only_completion: bool = False
    stable_detections: int = 3
    hold_time_sec: float = 0.30
    acquire_timeout_sec: float = 4.0
    marker_loss_timeout_sec: float = 0.60
    observation_stale_sec: float = 0.25
    future_tolerance_sec: float = 0.10

    def __post_init__(self) -> None:
        """Validate target, sign, gain, and timeout values."""
        if isinstance(self.target_marker_id, bool):
            raise ValueError('target_marker_id must be a nonnegative integer')
        marker_id = int(self.target_marker_id)
        if marker_id != self.target_marker_id or marker_id < 0:
            raise ValueError('target_marker_id must be a nonnegative integer')
        object.__setattr__(self, 'target_marker_id', marker_id)
        camera_name = str(self.camera_name).strip()
        if not camera_name:
            raise ValueError('camera_name must not be empty')
        object.__setattr__(self, 'camera_name', camera_name)
        object.__setattr__(
            self, 'motion_sign', _sign(self.motion_sign, 'motion_sign')
        )
        object.__setattr__(
            self,
            'lateral_error_sign',
            _sign(self.lateral_error_sign, 'lateral_error_sign'),
        )
        object.__setattr__(
            self,
            'yaw_error_sign',
            _sign(self.yaw_error_sign, 'yaw_error_sign'),
        )
        object.__setattr__(
            self,
            'angular_command_sign',
            _sign(self.angular_command_sign, 'angular_command_sign'),
        )

        finite_fields = (
            'target_distance_m',
            'target_lateral_m',
            'target_yaw_rad',
        )
        for field_name in finite_fields:
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), field_name),
            )
        if self.target_distance_m <= 0.0:
            raise ValueError('target_distance_m must be greater than zero')

        positive_fields = (
            'distance_tolerance_m',
            'lateral_tolerance_m',
            'yaw_tolerance_rad',
            'linear_kp',
            'lateral_kp',
            'yaw_kp',
            'max_linear_mps',
            'max_angular_rps',
            'linear_gate_angle_rad',
            'acquire_timeout_sec',
            'marker_loss_timeout_sec',
            'observation_stale_sec',
        )
        for field_name in positive_fields:
            object.__setattr__(
                self,
                field_name,
                _positive(getattr(self, field_name), field_name),
            )
        object.__setattr__(
            self,
            'hold_time_sec',
            _nonnegative(self.hold_time_sec, 'hold_time_sec'),
        )
        minimum_steering = _nonnegative(
            self.min_steering_rps, 'min_steering_rps'
        )
        if minimum_steering > self.max_angular_rps:
            raise ValueError(
                'min_steering_rps must not exceed max_angular_rps'
            )
        object.__setattr__(
            self, 'min_steering_rps', minimum_steering
        )
        steering_scale = _positive(
            self.steering_output_scale, 'steering_output_scale'
        )
        if steering_scale > 1.0:
            raise ValueError('steering_output_scale must not exceed 1.0')
        object.__setattr__(
            self, 'steering_output_scale', steering_scale
        )
        slow_band = _positive(
            self.steering_slow_band_ratio, 'steering_slow_band_ratio'
        )
        if slow_band <= 1.0:
            raise ValueError('steering_slow_band_ratio must exceed 1.0')
        object.__setattr__(
            self, 'steering_slow_band_ratio', slow_band
        )
        hysteresis = _positive(
            self.alignment_hysteresis_ratio,
            'alignment_hysteresis_ratio',
        )
        if hysteresis < 1.0:
            raise ValueError(
                'alignment_hysteresis_ratio must be at least 1.0'
            )
        object.__setattr__(
            self, 'alignment_hysteresis_ratio', hysteresis
        )
        if not isinstance(self.distance_only_completion, bool):
            raise ValueError('distance_only_completion must be boolean')
        object.__setattr__(
            self,
            'future_tolerance_sec',
            _nonnegative(self.future_tolerance_sec, 'future_tolerance_sec'),
        )
        if isinstance(self.stable_detections, bool):
            raise ValueError('stable_detections must be a positive integer')
        stable = int(self.stable_detections)
        if stable != self.stable_detections or stable <= 0:
            raise ValueError('stable_detections must be a positive integer')
        object.__setattr__(self, 'stable_detections', stable)


class VisualApproachController:
    """Approach one marker, with an optional straight distance-only mode."""

    def __init__(self, config: VisualApproachConfig) -> None:
        """Create an idle controller with empty detection history."""
        if not isinstance(config, VisualApproachConfig):
            raise TypeError('config must be a VisualApproachConfig')
        self.config = config
        self._started_at: Optional[float] = None
        self._last_detection_stamp: Optional[float] = None
        self._last_seen_at: Optional[float] = None
        self._hold_since: Optional[float] = None
        self._stable_count = 0
        self._acquired = False
        self._terminal: Optional[ControlResult] = None

    @property
    def acquired(self) -> bool:
        """Return whether enough unique consecutive frames were received."""
        return self._acquired

    def start(self, now_sec: float) -> None:
        """Reset detection history and begin an approach attempt."""
        self._started_at = _finite(now_sec, 'now_sec')
        self._last_detection_stamp = None
        self._last_seen_at = None
        self._hold_since = None
        self._stable_count = 0
        self._acquired = False
        self._terminal = None

    def stop(self, reason: str = 'stopped') -> ControlResult:
        """Stop the approach and latch a terminal failure."""
        self._terminal = ControlResult(
            ControlStatus.FAILED, STOP_COMMAND, str(reason)
        )
        return self._terminal

    @staticmethod
    def _field(observation: Any, name: str) -> Any:
        if isinstance(observation, dict):
            return observation.get(name)
        return getattr(observation, name, None)

    def _missing(self, now: float, reason: str) -> ControlResult:
        self._stable_count = 0
        self._hold_since = None
        if self._acquired:
            if (
                self._last_seen_at is None
                or now - self._last_seen_at
                > self.config.marker_loss_timeout_sec
            ):
                return self.stop('target_marker_lost')
            if self.config.distance_only_completion:
                return ControlResult(
                    ControlStatus.RUNNING,
                    VelocityCommand(
                        linear_x=(
                            self.config.motion_sign
                            * self.config.max_linear_mps
                        )
                    ),
                    'distance_only_marker_temporarily_lost',
                )
            return ControlResult(ControlStatus.WAITING, STOP_COMMAND, reason)
        if now - self._started_at > self.config.acquire_timeout_sec:
            return self.stop('target_marker_acquire_timeout')
        return ControlResult(ControlStatus.WAITING, STOP_COMMAND, reason)

    def update(self, observation: Any, now_sec: float) -> ControlResult:
        """
        Return a command for one new timestamped marker observation.

        Full-pose mode stops immediately on a missing, wrong, old, stale, or
        not-yet-stable observation.  Distance-only mode may keep its bounded
        straight command through a brief loss, but fails when the configured
        marker-loss timeout expires.
        """
        if self._terminal is not None:
            return self._terminal
        try:
            now = _finite(now_sec, 'now_sec')
        except ValueError as exc:
            return self.stop(f'invalid_time:{exc}')
        if self._started_at is None:
            self.start(now)
        if now < self._started_at:
            return self.stop('clock_moved_backwards')
        if observation is None:
            return self._missing(now, 'target_marker_not_visible')

        marker_id = self._field(observation, 'marker_id')
        camera_name = self._field(observation, 'camera_name')
        if marker_id != self.config.target_marker_id:
            return self._missing(now, 'wrong_marker_id')
        if camera_name != self.config.camera_name:
            return self._missing(now, 'wrong_camera')

        try:
            stamp = _finite(
                self._field(observation, 'timestamp_sec'),
                'observation.timestamp_sec',
            )
            distance = _finite(
                self._field(observation, 'distance_m'),
                'observation.distance_m',
            )
            lateral = _finite(
                self._field(observation, 'lateral_m'),
                'observation.lateral_m',
            )
            yaw = _finite(
                self._field(observation, 'yaw_rad'),
                'observation.yaw_rad',
            )
        except ValueError as exc:
            return self.stop(f'invalid_marker_observation:{exc}')
        if distance <= 0.0:
            return self.stop(
                'invalid_marker_observation:distance_not_positive'
            )
        if stamp > now + self.config.future_tolerance_sec:
            return self.stop('marker_timestamp_in_future')
        if now - stamp > self.config.observation_stale_sec:
            return self._missing(now, 'target_marker_observation_stale')
        if (
            self._last_detection_stamp is not None
            and stamp < self._last_detection_stamp
        ):
            return self._missing(now, 'old_marker_observation')

        is_new_detection = stamp != self._last_detection_stamp
        if is_new_detection:
            self._last_detection_stamp = stamp
            self._last_seen_at = now
            self._stable_count += 1
        elif not self._acquired:
            # A control timer commonly runs faster than the camera.  Reusing a
            # still-fresh frame must not count as another stable detection, but
            # it also must not break a sequence of genuinely new frames.
            return ControlResult(
                ControlStatus.WAITING,
                STOP_COMMAND,
                'waiting_for_new_marker_frame',
            )
        if self._stable_count < self.config.stable_detections:
            return ControlResult(
                ControlStatus.WAITING,
                STOP_COMMAND,
                'stabilizing_target_marker',
            )
        self._acquired = True

        try:
            distance_error = _finite(
                distance - self.config.target_distance_m,
                'distance_error',
            )
            lateral_error = _finite(
                lateral - self.config.target_lateral_m,
                'lateral_error',
            )
            yaw_error = normalize_angle(
                _finite(
                    yaw - self.config.target_yaw_rad,
                    'yaw_error',
                )
            )
        except ValueError as exc:
            return self.stop(f'invalid_marker_error:{exc}')
        # Once the hold has begun, use a wider exit band.  This prevents one
        # noisy pose estimate at the tolerance boundary from restarting the
        # entire hold and causing left-right hunting until timeout.
        tolerance_scale = (
            self.config.alignment_hysteresis_ratio
            if self._hold_since is not None
            else 1.0
        )
        if self.config.distance_only_completion:
            # This mode treats the marker as a one-sided stop sensor.  Once
            # the robot is at or closer than the far edge of the distance
            # band it must stop; it must never reverse to recover overshoot.
            distance_aligned = (
                distance_error
                <= self.config.distance_tolerance_m * tolerance_scale
            )
        else:
            distance_aligned = (
                abs(distance_error)
                <= self.config.distance_tolerance_m * tolerance_scale
            )
        pose_aligned = (
            abs(lateral_error)
            <= self.config.lateral_tolerance_m * tolerance_scale
            and abs(yaw_error)
            <= self.config.yaw_tolerance_rad * tolerance_scale
        )
        aligned = distance_aligned and (
            self.config.distance_only_completion or pose_aligned
        )
        telemetry = {
            'distance_error_m': distance_error,
            'lateral_error_m': lateral_error,
            'yaw_error_rad': yaw_error,
        }
        if aligned:
            if self._hold_since is None:
                self._hold_since = now
            if now - self._hold_since >= self.config.hold_time_sec:
                self._terminal = ControlResult(
                    ControlStatus.SUCCEEDED,
                    STOP_COMMAND,
                    'marker_alignment_held',
                    **telemetry,
                )
                return self._terminal
            return ControlResult(
                ControlStatus.RUNNING,
                STOP_COMMAND,
                'holding_marker_alignment',
                **telemetry,
            )
        self._hold_since = None

        if self.config.distance_only_completion:
            return ControlResult(
                ControlStatus.RUNNING,
                VelocityCommand(
                    linear_x=(
                        self.config.motion_sign
                        * self.config.max_linear_mps
                    )
                ),
                'tracking_target_distance_only',
                **telemetry,
            )

        linear = bounded_scale(
            distance_error,
            self.config.motion_sign * self.config.linear_kp,
            self.config.max_linear_mps,
        )
        lateral_term = bounded_scale(
            lateral_error,
            self.config.lateral_kp * self.config.lateral_error_sign,
            self.config.max_angular_rps,
        )
        yaw_term = bounded_scale(
            yaw_error,
            self.config.yaw_kp * self.config.yaw_error_sign,
            self.config.max_angular_rps,
        )
        angular = clamp(
            self.config.angular_command_sign * (lateral_term + yaw_term),
            -self.config.max_angular_rps,
            self.config.max_angular_rps,
        )
        lateral_needs_correction = (
            abs(lateral_error) > self.config.lateral_tolerance_m
        )
        yaw_needs_correction = (
            abs(yaw_error) > self.config.yaw_tolerance_rad
        )
        if lateral_needs_correction or yaw_needs_correction:
            lateral_slow_limit = (
                self.config.lateral_tolerance_m
                * self.config.steering_slow_band_ratio
            )
            yaw_slow_limit = (
                self.config.yaw_tolerance_rad
                * self.config.steering_slow_band_ratio
            )
            if (
                abs(lateral_error) >= lateral_slow_limit
                or abs(yaw_error) >= yaw_slow_limit
            ):
                slow_band_progress = 1.0
            else:
                normalized_steering_error = max(
                    abs(lateral_error)
                    / self.config.lateral_tolerance_m,
                    abs(yaw_error) / self.config.yaw_tolerance_rad,
                )
                slow_band_progress = clamp(
                    (
                        normalized_steering_error - 1.0
                    ) / (
                        self.config.steering_slow_band_ratio - 1.0
                    ),
                    0.0,
                    1.0,
                )
            angular *= slow_band_progress
            if (
                slow_band_progress >= 1.0
                and abs(angular) < self.config.min_steering_rps
            ):
                if lateral_needs_correction:
                    steering_error = (
                        self.config.lateral_error_sign * lateral_error
                    )
                else:
                    steering_error = self.config.yaw_error_sign * yaw_error
                angular = math.copysign(
                    self.config.min_steering_rps,
                    self.config.angular_command_sign * steering_error,
                )
        else:
            angular = 0.0
        angular *= self.config.steering_output_scale

        heading_error = (
            math.atan2(
                self.config.lateral_error_sign * lateral_error,
                max(distance, 1e-6),
            )
            + self.config.yaw_error_sign * yaw_error
        )
        heading_gated = (
            abs(heading_error) >= self.config.linear_gate_angle_rad
        )
        if heading_gated:
            linear = 0.0
            # Once translation is deliberately gated, a steering command below
            # the base's deadband can leave the controller permanently stopped:
            # it cannot reduce the heading error, so translation never resumes.
            # Keep the gentle scaled steering while moving, but guarantee the
            # configured usable minimum while turning in place.
            if abs(angular) < self.config.min_steering_rps:
                if abs(angular) > 1e-9:
                    steering_direction = angular
                else:
                    # The individual lateral/yaw errors can each be inside
                    # their intentionally loose tolerances while their sum is
                    # still outside the tighter translation gate.  Steering
                    # from the combined heading error avoids a zero-linear,
                    # zero-angular deadlock in that case.
                    steering_direction = (
                        self.config.angular_command_sign
                        * heading_error
                    )
                angular = math.copysign(
                    self.config.min_steering_rps,
                    steering_direction,
                )
        return ControlResult(
            ControlStatus.RUNNING,
            VelocityCommand(linear_x=linear, angular_z=angular),
            (
                'heading_gate_turn_in_place'
                if heading_gated
                else 'tracking_target_marker'
            ),
            **telemetry,
        )
