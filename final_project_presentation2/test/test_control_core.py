"""Tests for the ROS-independent route motion controllers."""

import math

from final_project_presentation2.control_core import ControlStatus
from final_project_presentation2.control_core import ExactAngleTurnController
from final_project_presentation2.control_core import TurnControlConfig
from final_project_presentation2.control_core import VelocityCommand
from final_project_presentation2.control_core import VisualApproachConfig
from final_project_presentation2.control_core import VisualApproachController
from final_project_presentation2.control_core import YawUnwrapper

import pytest


def observation(stamp, distance=0.8, lateral=0.0, yaw=0.0, **changes):
    """Create a controller-compatible marker observation mapping."""
    result = {
        'marker_id': 22,
        'camera_name': 'front',
        'timestamp_sec': stamp,
        'distance_m': distance,
        'lateral_m': lateral,
        'yaw_rad': yaw,
    }
    result.update(changes)
    return result


def test_yaw_unwrapper_crosses_pi_boundary_in_both_directions():
    """Accumulate relative yaw without a 360-degree discontinuity."""
    unwrap = YawUnwrapper()
    unwrap.reset(math.radians(179.0))
    assert unwrap.update(math.radians(-179.0)) == pytest.approx(
        math.radians(2.0)
    )
    assert unwrap.update(math.radians(178.0)) == pytest.approx(
        math.radians(-1.0)
    )


def test_turn_uses_exact_relative_odom_angle_and_ignores_marker():
    """Never stop a configured turn merely because its marker appeared."""
    controller = ExactAngleTurnController(
        TurnControlConfig(
            kp=2.0,
            max_angular_rps=0.5,
            min_angular_rps=0.05,
            tolerance_deg=1.0,
            settle_time_sec=0.15,
            timeout_sec=3.0,
        )
    )
    controller.start(0.0, -90.0, 10.0)

    early = controller.update(
        math.radians(-35.0), 10.1, marker_visible=True
    )
    target_first = controller.update(
        math.radians(-89.5), 10.2, marker_visible=True
    )
    done = controller.update(
        math.radians(-90.0), 10.36, marker_visible=True
    )

    assert early.status is ControlStatus.RUNNING
    assert early.command.angular_z < 0.0
    assert not early.command.is_zero
    assert target_first.status is ControlStatus.RUNNING
    assert target_first.command.is_zero
    assert done.status is ControlStatus.SUCCEEDED
    assert done.command.is_zero


def test_turn_output_is_bounded_and_stale_odom_fails_stopped():
    """Bound angular speed and fail closed on an old odometry sample."""
    controller = ExactAngleTurnController(
        TurnControlConfig(max_angular_rps=0.3, odom_stale_sec=0.2)
    )
    controller.start(0.0, 90.0, 1.0)
    running = controller.update(0.0, 1.05)
    stale = controller.update(0.1, 1.5, odom_timestamp_sec=1.0)

    assert running.command.angular_z == pytest.approx(0.3)
    assert stale.failed
    assert stale.command.is_zero
    assert stale.reason == 'odometry_stale'


def test_front_approach_requires_unique_stable_frames_before_motion():
    """Do not move until the target has appeared in consecutive frames."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.3,
            stable_detections=3,
            max_linear_mps=0.1,
        )
    )
    controller.start(20.0)

    first = controller.update(observation(20.01), 20.01)
    duplicate = controller.update(observation(20.01), 20.02)
    second = controller.update(observation(20.03), 20.03)
    third = controller.update(observation(20.04), 20.04)
    stable = controller.update(observation(20.05), 20.05)

    assert first.command.is_zero
    assert duplicate.command.is_zero
    assert second.command.is_zero
    assert third.status is ControlStatus.RUNNING
    assert third.command.linear_x == pytest.approx(0.1)
    assert stable.status is ControlStatus.RUNNING
    assert stable.command.linear_x == pytest.approx(0.1)


def test_opposite_direction_marker_and_camera_are_ignored():
    """A return-side marker cannot drive an outbound target controller."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=2,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.3,
            stable_detections=1,
        )
    )
    controller.start(25.0)

    wrong_id = controller.update(
        observation(25.01, marker_id=4, camera_name='front'), 25.01
    )
    wrong_camera = controller.update(
        observation(25.02, marker_id=2, camera_name='rear'), 25.02
    )

    assert wrong_id.status is ControlStatus.WAITING
    assert wrong_id.reason == 'wrong_marker_id'
    assert wrong_id.command.is_zero
    assert wrong_camera.status is ControlStatus.WAITING
    assert wrong_camera.reason == 'wrong_camera'
    assert wrong_camera.command.is_zero


def test_rear_approach_and_steering_signs_are_configurable():
    """Issue negative x for rear travel and apply rear-specific sign maps."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=23,
            camera_name='rear',
            motion_sign=-1,
            target_distance_m=0.25,
            stable_detections=1,
            lateral_error_sign=-1,
            yaw_error_sign=-1,
            angular_command_sign=1,
            max_linear_mps=0.12,
            max_angular_rps=0.4,
        )
    )
    result = controller.update(
        observation(
            30.0,
            distance=0.8,
            lateral=0.1,
            yaw=0.1,
            marker_id=23,
            camera_name='rear',
        ),
        30.0,
    )

    # A large heading error gates translation until steering is corrected.
    assert result.command.linear_x == pytest.approx(0.0)
    assert result.command.angular_z < 0.0
    assert abs(result.command.angular_z) <= 0.4


def test_lost_or_stale_marker_stops_immediately_then_fails():
    """Never continue blind after acquisition and latch prolonged loss."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.3,
            stable_detections=1,
            observation_stale_sec=0.1,
            marker_loss_timeout_sec=0.5,
        )
    )
    moving = controller.update(observation(40.0), 40.0)
    stale = controller.update(observation(40.0), 40.2)
    failed = controller.update(None, 40.51)

    assert not moving.command.is_zero
    assert stale.status is ControlStatus.WAITING
    assert stale.command.is_zero
    assert failed.failed
    assert failed.command.is_zero
    assert failed.reason == 'target_marker_lost'


def test_alignment_must_be_held_before_success():
    """Require a stable in-tolerance interval rather than one lucky frame."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.3,
            stable_detections=1,
            hold_time_sec=0.2,
        )
    )
    first = controller.update(observation(50.0, distance=0.3), 50.0)
    held = controller.update(observation(50.21, distance=0.3), 50.21)

    assert first.status is ControlStatus.RUNNING
    assert first.command.is_zero
    assert held.complete


def test_distance_only_completion_ignores_pose_error():
    """The generic distance-only controller mode ignores pose error."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='rear',
            motion_sign=-1,
            target_distance_m=0.5,
            distance_tolerance_m=0.05,
            lateral_tolerance_m=0.02,
            yaw_tolerance_rad=math.radians(3.0),
            distance_only_completion=True,
            stable_detections=1,
            hold_time_sec=0.0,
        )
    )

    result = controller.update(
        observation(
            52.0,
            distance=0.52,
            lateral=0.18,
            yaw=math.radians(25.0),
            camera_name='rear',
        ),
        52.0,
    )

    assert result.complete
    assert result.command.is_zero


def test_distance_only_moves_straight_at_constant_speed_until_close():
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.2,
            distance_tolerance_m=0.02,
            lateral_tolerance_m=0.01,
            yaw_tolerance_rad=math.radians(2.0),
            distance_only_completion=True,
            max_linear_mps=0.08,
            stable_detections=1,
            hold_time_sec=0.0,
        )
    )

    far = controller.update(
        observation(
            52.5,
            distance=0.8,
            lateral=0.3,
            yaw=math.radians(35.0),
        ),
        52.5,
    )
    close = controller.update(
        observation(
            52.6,
            distance=0.21,
            lateral=0.3,
            yaw=math.radians(35.0),
        ),
        52.6,
    )

    assert far.command.linear_x == pytest.approx(0.08)
    assert far.command.angular_z == 0.0
    assert far.reason == 'tracking_target_distance_only'
    assert close.complete
    assert close.command.is_zero


def test_distance_only_bridges_brief_marker_loss_then_fails():
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='rear',
            motion_sign=-1,
            target_distance_m=0.2,
            distance_only_completion=True,
            max_linear_mps=0.08,
            stable_detections=1,
            marker_loss_timeout_sec=0.35,
        )
    )

    controller.update(
        observation(54.0, distance=0.8, camera_name='rear'),
        54.0,
    )
    brief_loss = controller.update(None, 54.2)
    failed = controller.update(None, 54.36)

    assert brief_loss.command.linear_x == pytest.approx(-0.08)
    assert brief_loss.command.angular_z == 0.0
    assert brief_loss.reason == 'distance_only_marker_temporarily_lost'
    assert failed.failed
    assert failed.command.is_zero


def test_destination_full_pose_still_requires_lateral_and_yaw_alignment():
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='rear',
            motion_sign=-1,
            target_distance_m=0.5,
            distance_tolerance_m=0.05,
            lateral_tolerance_m=0.02,
            yaw_tolerance_rad=math.radians(3.0),
            distance_only_completion=False,
            stable_detections=1,
            hold_time_sec=0.0,
        )
    )

    result = controller.update(
        observation(
            53.0,
            distance=0.52,
            lateral=0.18,
            yaw=math.radians(25.0),
            camera_name='rear',
        ),
        53.0,
    )

    assert not result.complete


def test_steering_tapers_near_tolerance_and_scales_output():
    """Avoid a minimum-speed kick that repeatedly crosses the target."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.3,
            lateral_tolerance_m=0.05,
            yaw_tolerance_rad=0.1,
            lateral_kp=1.0,
            min_steering_rps=0.05,
            steering_output_scale=0.5,
            steering_slow_band_ratio=2.0,
            stable_detections=1,
        )
    )

    near = controller.update(
        observation(55.0, distance=0.3, lateral=0.06),
        55.0,
    )

    assert 0.0 < near.command.angular_z < 0.02
    assert near.command.angular_z == pytest.approx(0.006)


def test_alignment_hold_uses_hysteresis_for_small_pose_jitter():
    """One noisy boundary frame must not restart the alignment hold."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.3,
            distance_tolerance_m=0.02,
            lateral_tolerance_m=0.02,
            yaw_tolerance_rad=0.05,
            alignment_hysteresis_ratio=1.5,
            stable_detections=1,
            hold_time_sec=0.2,
        )
    )

    entered = controller.update(
        observation(56.0, distance=0.3, lateral=0.019, yaw=0.049),
        56.0,
    )
    jitter = controller.update(
        observation(56.1, distance=0.325, lateral=0.025, yaw=0.06),
        56.1,
    )
    complete = controller.update(
        observation(56.21, distance=0.325, lateral=0.025, yaw=0.06),
        56.21,
    )

    assert entered.command.is_zero
    assert jitter.command.is_zero
    assert jitter.reason == 'holding_marker_alignment'
    assert complete.complete


def test_extreme_finite_errors_and_gains_still_return_bounded_output():
    """Saturate safely even when a multiplication would overflow float."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            target_distance_m=0.3,
            stable_detections=1,
            linear_kp=1e308,
            lateral_kp=1e308,
            max_linear_mps=0.1,
            max_angular_rps=0.2,
        )
    )
    result = controller.update(
        observation(60.0, distance=1e308, lateral=1e308), 60.0
    )

    assert math.isfinite(result.command.linear_x)
    assert math.isfinite(result.command.angular_z)
    assert result.command.linear_x == pytest.approx(0.0)
    assert result.command.angular_z == pytest.approx(0.2)


def test_default_camera_convention_steers_positive_lateral_error_negative():
    """Match the installed camera/controller steering convention."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            stable_detections=1,
            target_distance_m=0.3,
            angular_command_sign=-1,
        )
    )

    result = controller.update(
        observation(70.0, distance=0.8, lateral=0.05), 70.0
    )

    assert result.command.angular_z < 0.0


def test_translation_resumes_once_heading_is_inside_gate():
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            stable_detections=1,
            target_distance_m=0.3,
            angular_command_sign=-1,
            linear_gate_angle_rad=math.radians(12.0),
        )
    )

    gated = controller.update(
        observation(80.0, distance=0.8, lateral=0.2), 80.0
    )
    moving = controller.update(
        observation(80.1, distance=0.8, lateral=0.01), 80.1
    )

    assert gated.command.linear_x == pytest.approx(0.0)
    assert gated.reason == 'heading_gate_turn_in_place'
    assert moving.command.linear_x > 0.0
    assert moving.reason == 'tracking_target_marker'


def test_heading_gate_keeps_usable_minimum_after_steering_scale():
    """A gated robot must not stall on a sub-deadband turn command."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=22,
            camera_name='front',
            motion_sign=1,
            stable_detections=1,
            target_distance_m=0.2,
            lateral_tolerance_m=0.02,
            lateral_kp=0.01,
            min_steering_rps=0.05,
            steering_output_scale=0.5,
            angular_command_sign=-1,
            linear_gate_angle_rad=math.radians(12.0),
        )
    )

    result = controller.update(
        observation(81.0, distance=0.4, lateral=0.1), 81.0
    )

    assert result.command.linear_x == pytest.approx(0.0)
    assert abs(result.command.angular_z) == pytest.approx(0.05)
    assert result.reason == 'heading_gate_turn_in_place'


def test_combined_heading_gate_cannot_deadlock_inside_pose_tolerances():
    """Loose individual tolerances must not produce a zero/zero command."""
    controller = VisualApproachController(
        VisualApproachConfig(
            target_marker_id=3,
            camera_name='rear',
            motion_sign=-1,
            stable_detections=1,
            target_distance_m=0.2,
            lateral_tolerance_m=0.07,
            yaw_tolerance_rad=math.radians(10.0),
            min_steering_rps=0.05,
            steering_output_scale=0.5,
            angular_command_sign=-1,
            linear_gate_angle_rad=math.radians(12.0),
        )
    )

    # Each pose component is acceptable, but together they exceed the 12
    # degree translation gate (atan2(0.05, 0.4) + 6 deg ~= 13.1 deg).
    result = controller.update(
        observation(
            82.0,
            distance=0.4,
            lateral=0.05,
            yaw=math.radians(6.0),
            camera_name='rear',
            marker_id=3,
        ),
        82.0,
    )

    assert result.command.linear_x == pytest.approx(0.0)
    assert abs(result.command.angular_z) == pytest.approx(0.05)
    assert result.reason == 'heading_gate_turn_in_place'


@pytest.mark.parametrize('bad_value', [float('nan'), float('inf')])
def test_velocity_command_rejects_nonfinite_output(bad_value):
    """Make it impossible to pass a nonfinite base command downstream."""
    with pytest.raises(ValueError, match='finite'):
        VelocityCommand(linear_x=bad_value)
