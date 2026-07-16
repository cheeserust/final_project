"""Tests for the pure watchdog configuration and relay state."""

import json
from types import SimpleNamespace

from final_project_presentation2.watchdog_node import _has_exclusive_publisher
from final_project_presentation2.watchdog_node import _source_command_age_sec
from final_project_presentation2.watchdog_node import SafeTwist
from final_project_presentation2.watchdog_node import WatchdogCore

import pytest


def test_stamped_raw_command_uses_original_source_age():
    assert _source_command_age_sec(99.8, 100.0, 0.25) == pytest.approx(0.2)
    assert _source_command_age_sec(100.05, 100.0, 0.25) == 0.0
    assert _source_command_age_sec(99.7, 100.0, 0.25) is None
    assert _source_command_age_sec(100.2, 100.0, 0.25) is None
    assert _source_command_age_sec(0.0, 100.0, 0.25) is None
    assert _source_command_age_sec(float('nan'), 100.0, 0.25) is None


def test_velocity_publisher_graph_requires_one_expected_node():
    expected = SimpleNamespace(
        node_name='final_project_presentation2',
        topic_type='geometry_msgs/msg/TwistStamped',
    )
    unexpected = SimpleNamespace(
        node_name='teleop_twist_keyboard',
        topic_type='geometry_msgs/msg/TwistStamped',
    )
    wrong_type = SimpleNamespace(
        node_name='final_project_presentation2',
        topic_type='geometry_msgs/msg/Twist',
    )

    assert _has_exclusive_publisher(
        [expected],
        'final_project_presentation2',
        'geometry_msgs/msg/TwistStamped',
    )
    assert not _has_exclusive_publisher(
        [],
        'final_project_presentation2',
        'geometry_msgs/msg/TwistStamped',
    )
    assert not _has_exclusive_publisher(
        [unexpected],
        'final_project_presentation2',
        'geometry_msgs/msg/TwistStamped',
    )
    assert not _has_exclusive_publisher(
        [wrong_type],
        'final_project_presentation2',
        'geometry_msgs/msg/TwistStamped',
    )
    assert not _has_exclusive_publisher(
        [expected, expected],
        'final_project_presentation2',
        'geometry_msgs/msg/TwistStamped',
    )


def full_config(timeout=0.25, linear=0.1, angular=0.4, revision=3):
    """Return the exact flat live payload sent by the PC node."""
    return {
        'schema_version': 1,
        'revision': revision,
        'valid': True,
        'cmd_timeout_sec': timeout,
        'publish_rate_hz': 20.0,
        'max_linear_mps': linear,
        'max_angular_rps': angular,
        'reject_nonfinite_commands': True,
    }


def test_nonzero_command_is_refused_until_valid_config():
    """Start safe even if raw commands arrive before transient config."""
    core = WatchdogCore()
    accepted = core.record(SafeTwist(linear_x=0.05), 1.0)
    decision = core.sample(1.0)

    assert not accepted
    assert not decision.configured
    assert not decision.fresh
    assert decision.command.is_zero


def test_full_json_config_clamps_and_times_out_at_default_rate():
    """Clamp every component and replace stale input with zero."""
    core = WatchdogCore()
    config = core.configure_json(json.dumps(full_config()))
    assert config.revision == 3
    assert core.record(
        SafeTwist(
            linear_x=0.5,
            linear_y=-0.5,
            angular_x=2.0,
            angular_z=-2.0,
        ),
        10.0,
    )

    fresh = core.sample(10.25)
    stale = core.sample(10.251)

    assert fresh.fresh
    assert fresh.command.linear_x == pytest.approx(0.1)
    assert fresh.command.linear_y == pytest.approx(-0.1)
    assert fresh.command.angular_x == pytest.approx(0.4)
    assert fresh.command.angular_z == pytest.approx(-0.4)
    assert stale.command.is_zero
    assert stale.became_stale


def test_new_config_revision_clears_old_command():
    """Never replay a velocity recorded under prior live limits."""
    core = WatchdogCore()
    core.configure(full_config(revision=1))
    core.record(SafeTwist(linear_x=0.05), 20.0)
    assert core.sample(20.0).fresh

    core.configure(full_config(revision=2))
    replacement = core.sample(20.01)
    assert replacement.configured
    assert not replacement.fresh
    assert replacement.command.is_zero


def test_identical_config_heartbeat_does_not_interrupt_command():
    """Allow the PC to periodically republish one transient revision."""
    core = WatchdogCore()
    payload = full_config(revision=9)
    core.configure(payload)
    core.record(SafeTwist(linear_x=0.05), 25.0)

    core.configure(payload)
    decision = core.sample(25.01)

    assert decision.fresh
    assert decision.command.linear_x == pytest.approx(0.05)


def test_invalid_replacement_config_invalidates_prior_config():
    """Fail closed rather than silently retaining an ambiguous old revision."""
    core = WatchdogCore()
    core.configure(full_config())

    with pytest.raises(ValueError, match='max_linear_mps'):
        core.configure(full_config(linear=float('nan')))

    decision = core.sample(30.0)
    assert not decision.configured
    assert decision.command.is_zero


def test_clock_rollback_and_nonfinite_twist_force_safe_behavior():
    """Treat time reversal as stale and reject NaN before state mutation."""
    core = WatchdogCore()
    core.configure(full_config())
    core.record(SafeTwist(linear_x=0.05), 40.0)
    rolled_back = core.sample(39.9)

    assert not rolled_back.fresh
    assert rolled_back.command.is_zero
    assert rolled_back.reason == 'clock_moved_backwards'
    with pytest.raises(ValueError, match='finite'):
        SafeTwist(angular_z=float('inf'))


@pytest.mark.parametrize(
    'payload',
    [
        'not-json',
        '[]',
        '{}',
        '{"valid":false}',
    ],
)
def test_invalid_json_or_incomplete_payload_is_rejected(payload):
    """Reject malformed JSON and every payload missing the exact schema."""
    core = WatchdogCore()
    with pytest.raises(ValueError):
        core.configure_json(payload)
    assert not core.configured


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('schema_version', True),
        ('schema_version', 2),
        ('revision', True),
        ('revision', -1),
        ('valid', 1),
        ('valid', False),
        ('cmd_timeout_sec', True),
        ('cmd_timeout_sec', 0.0),
        ('cmd_timeout_sec', 2.01),
        ('publish_rate_hz', True),
        ('publish_rate_hz', 19.99),
        ('max_linear_mps', True),
        ('max_linear_mps', 0.0),
        ('max_linear_mps', 1.01),
        ('max_angular_rps', True),
        ('max_angular_rps', 0.0),
        ('max_angular_rps', 4.01),
        ('reject_nonfinite_commands', 1),
        ('reject_nonfinite_commands', False),
    ],
)
def test_wrong_types_flags_and_unsafe_limits_are_rejected(field, value):
    """Fail closed on type coercion, disabled guards, or unsafe limits."""
    core = WatchdogCore()
    payload = full_config()
    payload[field] = value

    with pytest.raises(ValueError):
        core.configure(payload)

    assert not core.configured


def test_missing_or_extra_fields_are_rejected():
    """Accept no defaults, aliases, nested safety object, or extensions."""
    core = WatchdogCore()
    missing = full_config()
    del missing['cmd_timeout_sec']
    extra = full_config()
    extra['safety'] = {}

    with pytest.raises(ValueError, match='missing fields'):
        core.configure(missing)
    with pytest.raises(ValueError, match='extra fields'):
        core.configure(extra)

    assert not core.configured
