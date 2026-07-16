"""
Pinky-local fixed-topic cmd_vel watchdog.

Only this node publishes the hardware ``/cmd_vel`` topic.  It accepts raw
commands from the presentation PC after receiving a valid transient-local JSON
configuration and forces a zero command on invalid input or timeout.
"""

from dataclasses import dataclass
import json
import math
import signal
import threading
import time
from typing import Any, Mapping, Optional, Tuple

try:
    from geometry_msgs.msg import Twist, TwistStamped
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import HistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    ROS_AVAILABLE = True
    ROS_IMPORT_ERROR = None
except ImportError as ros_import_error:
    # Keep WatchdogCore importable in non-ROS unit tests.
    rclpy = None
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = ros_import_error

    class Node:  # type: ignore[no-redef]
        """Placeholder used only while importing the pure watchdog core."""

    class Twist:  # type: ignore[no-redef]
        """Placeholder for type annotations outside a ROS environment."""

    class TwistStamped:  # type: ignore[no-redef]
        """Placeholder for type annotations outside a ROS environment."""

    class String:  # type: ignore[no-redef]
        """Placeholder for type annotations outside a ROS environment."""


RAW_CMD_TOPIC = '/final_project_presentation2/cmd_vel_raw'
CONFIG_TOPIC = '/final_project_presentation2/watchdog_config'
HARDWARE_CMD_TOPIC = '/cmd_vel'
PUBLISH_RATE_HZ = 20.0
DEFAULT_STALE_TIMEOUT_SEC = 0.25
DEFAULT_MAX_LINEAR_MPS = 0.20
DEFAULT_MAX_ANGULAR_RPS = 1.00
ABSOLUTE_MAX_LINEAR_MPS = 1.00
ABSOLUTE_MAX_ANGULAR_RPS = 4.00
ABSOLUTE_MAX_TIMEOUT_SEC = 2.00
SOURCE_FUTURE_TOLERANCE_SEC = 0.10


def _has_exclusive_publisher(
    publisher_infos: Any,
    expected_node_name: str,
    expected_topic_type: str,
) -> bool:
    """Require one graph publisher with the exact node and message type."""
    infos = list(publisher_infos)
    return (
        len(infos) == 1
        and getattr(infos[0], 'node_name', None) == expected_node_name
        and getattr(infos[0], 'topic_type', None) == expected_topic_type
    )


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be a finite number')
    return result


def _bounded_positive(value: Any, name: str, upper: float) -> float:
    result = _finite(value, name)
    if result <= 0.0 or result > upper:
        raise ValueError(f'{name} must be greater than zero and <= {upper:g}')
    return result


def _source_command_age_sec(
    source_sec: Any,
    now_sec: Any,
    max_age_sec: Any,
) -> Optional[float]:
    """Return source age when a stamped command is safe to relay."""
    try:
        source = _finite(source_sec, 'source_sec')
        now = _finite(now_sec, 'now_sec')
        maximum = _bounded_positive(
            max_age_sec,
            'max_age_sec',
            ABSOLUTE_MAX_TIMEOUT_SEC,
        )
    except ValueError:
        return None
    if source <= 0.0:
        return None
    age = now - source
    if age < -SOURCE_FUTURE_TOLERANCE_SEC or age > maximum:
        return None
    return max(0.0, age)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class SafeTwist:
    """Transport-neutral, finite representation of all Twist components."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    def __post_init__(self) -> None:
        """Reject a command containing NaN, infinity, or booleans."""
        for name in (
            'linear_x',
            'linear_y',
            'linear_z',
            'angular_x',
            'angular_y',
            'angular_z',
        ):
            object.__setattr__(self, name, _finite(getattr(self, name), name))

    @property
    def values(self) -> Tuple[float, ...]:
        """Return components in ROS Twist order."""
        return (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        )

    @property
    def is_zero(self) -> bool:
        """Return whether every component is exactly zero."""
        return all(value == 0.0 for value in self.values)

    def clamped(
        self, max_linear_mps: float, max_angular_rps: float
    ) -> 'SafeTwist':
        """Clamp every linear and angular component independently."""
        return SafeTwist(
            linear_x=_clamp(self.linear_x, max_linear_mps),
            linear_y=_clamp(self.linear_y, max_linear_mps),
            linear_z=_clamp(self.linear_z, max_linear_mps),
            angular_x=_clamp(self.angular_x, max_angular_rps),
            angular_y=_clamp(self.angular_y, max_angular_rps),
            angular_z=_clamp(self.angular_z, max_angular_rps),
        )


ZERO_TWIST = SafeTwist()


@dataclass(frozen=True)
class WatchdogConfig:
    """Validated live safety limits delivered by the presentation PC."""

    schema_version: int = 1
    revision: int = 0
    valid: bool = True
    stale_timeout_sec: float = DEFAULT_STALE_TIMEOUT_SEC
    publish_rate_hz: float = PUBLISH_RATE_HZ
    max_linear_mps: float = DEFAULT_MAX_LINEAR_MPS
    max_angular_rps: float = DEFAULT_MAX_ANGULAR_RPS
    reject_nonfinite_commands: bool = True

    def __post_init__(self) -> None:
        """Validate limits against local hard safety ceilings."""
        if (
            isinstance(self.schema_version, bool)
            or not isinstance(self.schema_version, int)
            or self.schema_version != 1
        ):
            raise ValueError('schema_version must be integer 1')
        if (
            isinstance(self.revision, bool)
            or not isinstance(self.revision, int)
            or self.revision < 0
        ):
            raise ValueError('revision must be a nonnegative integer')
        if self.valid is not True:
            raise ValueError('valid must be true')
        object.__setattr__(
            self,
            'stale_timeout_sec',
            _bounded_positive(
                self.stale_timeout_sec,
                'stale_timeout_sec',
                ABSOLUTE_MAX_TIMEOUT_SEC,
            ),
        )
        publish_rate = _finite(self.publish_rate_hz, 'publish_rate_hz')
        if publish_rate != PUBLISH_RATE_HZ:
            raise ValueError(
                f'publish_rate_hz must be exactly {PUBLISH_RATE_HZ:g}'
            )
        object.__setattr__(self, 'publish_rate_hz', publish_rate)
        object.__setattr__(
            self,
            'max_linear_mps',
            _bounded_positive(
                self.max_linear_mps,
                'max_linear_mps',
                ABSOLUTE_MAX_LINEAR_MPS,
            ),
        )
        object.__setattr__(
            self,
            'max_angular_rps',
            _bounded_positive(
                self.max_angular_rps,
                'max_angular_rps',
                ABSOLUTE_MAX_ANGULAR_RPS,
            ),
        )
        if self.reject_nonfinite_commands is not True:
            raise ValueError('reject_nonfinite_commands must be true')

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> 'WatchdogConfig':
        """Validate the exact fail-closed payload published by the PC node."""
        if not isinstance(payload, Mapping):
            raise ValueError('watchdog configuration must be a JSON object')
        expected_fields = {
            'schema_version',
            'revision',
            'valid',
            'cmd_timeout_sec',
            'publish_rate_hz',
            'max_linear_mps',
            'max_angular_rps',
            'reject_nonfinite_commands',
        }
        actual_fields = set(payload)
        if actual_fields != expected_fields:
            missing = sorted(expected_fields - actual_fields)
            extra = sorted(actual_fields - expected_fields)
            details = []
            if missing:
                details.append(f'missing fields: {", ".join(missing)}')
            if extra:
                details.append(f'extra fields: {", ".join(extra)}')
            raise ValueError(
                'watchdog configuration fields do not match schema ('
                + '; '.join(details)
                + ')'
            )
        return cls(
            schema_version=payload['schema_version'],
            revision=payload['revision'],
            valid=payload['valid'],
            stale_timeout_sec=payload['cmd_timeout_sec'],
            publish_rate_hz=payload['publish_rate_hz'],
            max_linear_mps=payload['max_linear_mps'],
            max_angular_rps=payload['max_angular_rps'],
            reject_nonfinite_commands=payload[
                'reject_nonfinite_commands'
            ],
        )


@dataclass(frozen=True)
class WatchdogDecision:
    """One publish-tick decision from the ROS-independent core."""

    command: SafeTwist
    configured: bool
    fresh: bool
    became_stale: bool
    reason: str


class WatchdogCore:
    """Thread-safe, ROS-independent command validation and timeout state."""

    def __init__(self) -> None:
        """Create an unconfigured, stopped watchdog state."""
        self._lock = threading.Lock()
        self._config: Optional[WatchdogConfig] = None
        self._last_command: Optional[SafeTwist] = None
        self._last_update_sec: Optional[float] = None
        self._was_fresh = False

    @property
    def configured(self) -> bool:
        """Return whether a valid live configuration has been received."""
        with self._lock:
            return self._config is not None

    @property
    def config(self) -> Optional[WatchdogConfig]:
        """Return the immutable active config, if any."""
        with self._lock:
            return self._config

    def invalidate(self) -> None:
        """Invalidate configuration and command state immediately."""
        with self._lock:
            self._config = None
            self._last_command = None
            self._last_update_sec = None
            self._was_fresh = False

    def clear_command(self) -> None:
        """Keep valid limits but force the next sample to zero."""
        with self._lock:
            self._last_command = None
            self._last_update_sec = None
            self._was_fresh = False

    def configure(self, payload: Mapping[str, Any]) -> WatchdogConfig:
        """
        Atomically validate and install a mapping configuration.

        Any invalid replacement invalidates the prior configuration.  Keeping
        an old speed limit after the PC reports a malformed new revision would
        make the watchdog's live safety state ambiguous.
        """
        try:
            checked = WatchdogConfig.from_mapping(payload)
        except (TypeError, ValueError):
            self.invalidate()
            raise
        with self._lock:
            if checked == self._config:
                # The PC republishes the transient config as a heartbeat.  An
                # identical payload must not interrupt a live command.
                return checked
            self._config = checked
            # Never replay a command received under an earlier revision.
            self._last_command = None
            self._last_update_sec = None
            self._was_fresh = False
        return checked

    def configure_json(self, payload: str) -> WatchdogConfig:
        """Parse and install a transient-local std_msgs/String payload."""
        if not isinstance(payload, str):
            self.invalidate()
            raise ValueError('watchdog configuration payload must be text')
        try:
            decoded = json.loads(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.invalidate()
            raise ValueError(
                'watchdog configuration is not valid JSON'
            ) from exc
        return self.configure(decoded)

    def record(self, command: SafeTwist, now_sec: float) -> bool:
        """
        Validate, clamp, and record a raw command.

        Nonzero commands are refused until a valid configuration is installed.
        The return value tells the ROS adapter whether input was accepted.
        """
        if not isinstance(command, SafeTwist):
            raise TypeError('command must be a SafeTwist')
        now = _finite(now_sec, 'now_sec')
        with self._lock:
            if self._config is None:
                self._last_command = None
                self._last_update_sec = None
                self._was_fresh = False
                return command.is_zero
            self._last_command = command.clamped(
                self._config.max_linear_mps,
                self._config.max_angular_rps,
            )
            self._last_update_sec = now
            return True

    def sample(self, now_sec: float) -> WatchdogDecision:
        """Return a clamped fresh command or a guaranteed zero command."""
        now = _finite(now_sec, 'now_sec')
        with self._lock:
            if self._config is None:
                self._was_fresh = False
                return WatchdogDecision(
                    ZERO_TWIST, False, False, False, 'configuration_missing'
                )
            if self._last_command is None or self._last_update_sec is None:
                self._was_fresh = False
                return WatchdogDecision(
                    ZERO_TWIST, True, False, False, 'command_missing'
                )
            age = now - self._last_update_sec
            fresh = 0.0 <= age <= self._config.stale_timeout_sec
            became_stale = self._was_fresh and not fresh
            self._was_fresh = fresh
            if not fresh:
                reason = (
                    'clock_moved_backwards' if age < 0.0 else 'command_stale'
                )
                return WatchdogDecision(
                    ZERO_TWIST, True, False, became_stale, reason
                )
            return WatchdogDecision(
                self._last_command, True, True, False, 'command_fresh'
            )


class FinalProjectPresentationWatchdog(Node):
    """Fixed-topic ROS adapter around :class:`WatchdogCore`."""

    def __init__(self) -> None:
        """Create fixed subscriptions, publisher, and 20 Hz safety timer."""
        super().__init__('final_project_presentation2_watchdog')
        self._core = WatchdogCore()
        hardware_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher_graph_state: Optional[Tuple[bool, bool]] = None
        self._publisher = self.create_publisher(
            Twist, HARDWARE_CMD_TOPIC, hardware_qos
        )
        raw_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._raw_subscription = self.create_subscription(
            TwistStamped, RAW_CMD_TOPIC, self._raw_callback, raw_qos
        )
        config_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._config_subscription = self.create_subscription(
            String, CONFIG_TOPIC, self._config_callback, config_qos
        )
        self._timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self._publish_tick
        )
        self._publisher.publish(Twist())
        self.get_logger().info(
            f'watchdog waiting for config on {CONFIG_TOPIC}; '
            f'{RAW_CMD_TOPIC} -> {HARDWARE_CMD_TOPIC} at '
            f'{PUBLISH_RATE_HZ:.0f} Hz'
        )

    def _config_callback(self, message: String) -> None:
        try:
            config = self._core.configure_json(message.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f'invalid watchdog config; motion disabled: {exc}'
            )
            self._publisher.publish(Twist())
            return
        self.get_logger().info(
            'watchdog config accepted: '
            f'timeout={config.stale_timeout_sec:.3f}s, '
            f'linear<={config.max_linear_mps:.3f}m/s, '
            f'angular<={config.max_angular_rps:.3f}rad/s, '
            f'revision={config.revision!r}'
        )

    def _raw_callback(self, message: TwistStamped) -> None:
        config = self._core.config
        max_age = (
            DEFAULT_STALE_TIMEOUT_SEC
            if config is None
            else config.stale_timeout_sec
        )
        source_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1_000_000_000.0
        )
        now_ros_sec = (
            self.get_clock().now().nanoseconds / 1_000_000_000.0
        )
        source_age = _source_command_age_sec(
            source_sec,
            now_ros_sec,
            max_age,
        )
        if source_age is None:
            self._core.clear_command()
            self.get_logger().warning(
                'stale, future, or invalid stamped raw cmd_vel; forcing zero'
            )
            self._publisher.publish(Twist())
            return
        try:
            twist = message.twist
            command = SafeTwist(
                linear_x=twist.linear.x,
                linear_y=twist.linear.y,
                linear_z=twist.linear.z,
                angular_x=twist.angular.x,
                angular_y=twist.angular.y,
                angular_z=twist.angular.z,
            )
            accepted = self._core.record(
                command,
                time.monotonic() - source_age,
            )
        except (TypeError, ValueError) as exc:
            self._core.clear_command()
            self.get_logger().error(
                f'invalid raw cmd_vel; forcing zero: {exc}'
            )
            self._publisher.publish(Twist())
            return
        if not accepted and not command.is_zero:
            self.get_logger().warning(
                'nonzero raw cmd_vel refused before valid config'
            )

    @staticmethod
    def _message(command: SafeTwist) -> Twist:
        result = Twist()
        result.linear.x = command.linear_x
        result.linear.y = command.linear_y
        result.linear.z = command.linear_z
        result.angular.x = command.angular_x
        result.angular.y = command.angular_y
        result.angular.z = command.angular_z
        return result

    def _publish_tick(self) -> None:
        try:
            raw_publishers = self.get_publishers_info_by_topic(RAW_CMD_TOPIC)
            hardware_publishers = self.get_publishers_info_by_topic(
                HARDWARE_CMD_TOPIC
            )
            raw_ready = _has_exclusive_publisher(
                raw_publishers,
                'final_project_presentation2',
                'geometry_msgs/msg/TwistStamped',
            )
            hardware_ready = _has_exclusive_publisher(
                hardware_publishers,
                'final_project_presentation2_watchdog',
                'geometry_msgs/msg/Twist',
            )
        except Exception:
            raw_publishers = []
            hardware_publishers = []
            raw_ready = False
            hardware_ready = False

        graph_state = (raw_ready, hardware_ready)
        if graph_state != self._publisher_graph_state:
            self._publisher_graph_state = graph_state
            if raw_ready and hardware_ready:
                self.get_logger().info(
                    'exclusive velocity publisher graph is ready'
                )
            else:
                self.get_logger().error(
                    'velocity publisher graph is unsafe; forcing zero '
                    f'(raw_publishers={len(raw_publishers)}, '
                    f'cmd_vel_publishers={len(hardware_publishers)})'
                )
        if not raw_ready or not hardware_ready:
            self._core.clear_command()

        decision = self._core.sample(time.monotonic())
        self._publisher.publish(self._message(decision.command))
        if decision.became_stale:
            self.get_logger().warning(
                'raw cmd_vel became stale; forcing /cmd_vel to zero'
            )

    def force_stop(self) -> None:
        """Clear command state and publish a final hardware stop."""
        self._core.clear_command()
        self._publisher.publish(Twist())


def main(args=None) -> None:
    """Run the Pinky-local watchdog process."""
    if not ROS_AVAILABLE:
        raise RuntimeError(
            f'ROS 2 Python message packages are unavailable: '
            f'{ROS_IMPORT_ERROR}'
        )
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FinalProjectPresentationWatchdog()
    previous_handlers = {}

    def request_shutdown(_signum, _frame) -> None:
        node.force_stop()
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_shutdown)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        try:
            if rclpy.ok():
                node.force_stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
