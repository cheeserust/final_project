"""Cross-package validation for arm calibration and task configuration."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import yaml


LIMIT_TOLERANCE_RAD = 1e-3
HOME_TOLERANCE_RAD = 1e-3


@dataclass(frozen=True)
class ValidationReport:
    """Configuration validation errors and non-blocking warnings."""

    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    @property
    def ok(self) -> bool:
        """Return whether hardware execution can use the configuration."""
        return not self.errors


def load_yaml(path: str) -> dict[str, Any]:
    """Load one YAML mapping."""
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'YAML root must be a mapping: {path}')
    return data


def load_urdf_limits(paths: Sequence[str]) -> dict[str, tuple[float, float]]:
    """Read revolute-joint limits from the description xacro files."""
    limits: dict[str, tuple[float, float]] = {}
    for path in paths:
        root = ET.parse(path).getroot()
        for joint in root.iter('joint'):
            limit = joint.find('limit')
            if limit is None or 'lower' not in limit.attrib:
                continue
            name = str(joint.attrib.get('name', '')).replace('${prefix}', '')
            if not name:
                continue
            limits[name] = (
                float(limit.attrib['lower']),
                float(limit.attrib['upper']),
            )
    return limits


def _bridge_parameters(config: Mapping[str, Any]) -> Mapping[str, Any]:
    try:
        return config['arm_can_bridge']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise ValueError('Invalid arm_can_bridge configuration root') from exc


def _joint_table(
    params: Mapping[str, Any],
    prefix: str,
) -> dict[str, dict[str, float]]:
    names = [str(value) for value in params[f'{prefix}_joint_names']]
    minimums = params[f'{prefix}_min_positions_rad']
    maximums = params[f'{prefix}_max_positions_rad']
    homes = params[f'{prefix}_home_positions_rad']
    signs = params.get(f'{prefix}_raw_position_signs', [1.0] * len(names))
    offsets = params.get(
        f'{prefix}_raw_position_offsets_rad',
        [0.0] * len(names),
    )
    sequences = (minimums, maximums, homes, signs, offsets)
    if any(len(values) != len(names) for values in sequences):
        raise ValueError(f'{prefix} bridge arrays must have equal lengths')
    return {
        name: {
            'min': float(minimums[index]),
            'max': float(maximums[index]),
            'home': float(homes[index]),
            'sign': float(signs[index]),
            'offset': float(offsets[index]),
        }
        for index, name in enumerate(names)
    }


def _check_position(
    errors: list[str],
    label: str,
    joint: str,
    value: float,
    bounds: Mapping[str, float],
) -> None:
    if not math.isfinite(value):
        errors.append(f'{label}.{joint} is not finite')
        return
    if value < bounds['min'] or value > bounds['max']:
        errors.append(
            f'{label}.{joint}={value:.8f} is outside '
            f'[{bounds["min"]:.8f}, {bounds["max"]:.8f}]'
        )


def _check_limit_source(
    errors: list[str],
    source_name: str,
    source: Mapping[str, Any],
    bridge: Mapping[str, Mapping[str, float]],
) -> None:
    for joint, bridge_values in bridge.items():
        values = source.get(joint)
        if values is None:
            errors.append(f'{source_name} is missing joint {joint}')
            continue
        if isinstance(values, Mapping):
            minimum = float(values['min_position'])
            maximum = float(values['max_position'])
        else:
            minimum, maximum = (float(value) for value in values)
        if abs(minimum - bridge_values['min']) > LIMIT_TOLERANCE_RAD:
            errors.append(
                f'{source_name}.{joint}.min={minimum:.8f} does not match '
                f'bridge {bridge_values["min"]:.8f}'
            )
        if abs(maximum - bridge_values['max']) > LIMIT_TOLERANCE_RAD:
            errors.append(
                f'{source_name}.{joint}.max={maximum:.8f} does not match '
                f'bridge {bridge_values["max"]:.8f}'
            )


def validate_configuration(
    *,
    fixed_config: Mapping[str, Any],
    gripper_config: Mapping[str, Any],
    bridge_config: Mapping[str, Any],
    moveit_limits: Mapping[str, Any],
    urdf_limits: Mapping[str, tuple[float, float]],
) -> ValidationReport:
    """Validate all runtime joint names, limits, homes, and task poses."""
    errors: list[str] = []
    warnings: list[str] = []
    params = _bridge_parameters(bridge_config)
    arm = _joint_table(params, 'arm')
    gripper = _joint_table(params, 'gripper')

    expected_arm = [str(value) for value in fixed_config['joint_order']['arm']]
    expected_gripper = [
        str(value) for value in fixed_config['joint_order']['gripper']
    ]
    if set(expected_arm) != set(arm):
        errors.append('fixed_poses arm joint set does not match arm_can_bridge')
    if set(expected_gripper) != set(gripper):
        errors.append(
            'fixed_poses gripper joint set does not match arm_can_bridge'
        )

    for controller in (arm, gripper):
        for joint, bounds in controller.items():
            _check_position(
                errors,
                'bridge_home',
                joint,
                bounds['home'],
                bounds,
            )
            if bounds['sign'] not in (-1.0, 1.0):
                errors.append(f'bridge sign for {joint} must be -1 or 1')
            if not math.isfinite(bounds['offset']):
                errors.append(f'bridge offset for {joint} is not finite')

    moveit = moveit_limits.get('joint_limits', {})
    moveit_arm = {
        joint: values
        for joint, values in moveit.items()
        if joint in arm
    }
    _check_limit_source(errors, 'moveit_limits', moveit_arm, arm)
    _check_limit_source(
        errors,
        'urdf_limits',
        urdf_limits,
        {**arm, **gripper},
    )

    for pose_name, pose in fixed_config.get('arm_named_poses', {}).items():
        positions = pose.get('positions_rad', [])
        if len(positions) != len(expected_arm):
            errors.append(f'arm pose {pose_name} has wrong position count')
            continue
        for joint, raw_value in zip(expected_arm, positions):
            if joint in arm:
                _check_position(
                    errors,
                    f'arm_pose.{pose_name}',
                    joint,
                    float(raw_value),
                    arm[joint],
                )

    fixed_home = fixed_config.get('arm_named_poses', {}).get('home', {})
    for joint, raw_value in zip(
        expected_arm,
        fixed_home.get('positions_rad', []),
    ):
        if joint in arm and abs(float(raw_value) - arm[joint]['home']) > HOME_TOLERANCE_RAD:
            errors.append(
                f'fixed home for {joint}={float(raw_value):.8f} does not '
                f'match bridge home {arm[joint]["home"]:.8f}'
            )

    gripper_home = fixed_config.get('gripper_named_poses', {}).get(
        'open',
        {},
    )
    for joint, raw_value in zip(
        expected_gripper,
        gripper_home.get('positions_rad', []),
    ):
        if (
            joint in gripper
            and abs(float(raw_value) - gripper[joint]['home'])
            > HOME_TOLERANCE_RAD
        ):
            errors.append(
                f'fixed gripper open for {joint}={float(raw_value):.8f} '
                f'does not match bridge home {gripper[joint]["home"]:.8f}'
            )

    gripper_profiles: list[tuple[str, Sequence[Any]]] = []
    for name, pose in fixed_config.get('gripper_named_poses', {}).items():
        gripper_profiles.append((f'named.{name}', pose.get('positions_rad', [])))
    for name, profile in gripper_config.get('objects', {}).items():
        gripper_profiles.append((f'object.{name}', profile.get('gripper_close_rad', [])))
    button_profile = gripper_config.get('buttons', {}).get('press_pose_rad')
    if button_profile is not None:
        gripper_profiles.append(('buttons.press', button_profile))

    for profile_name, positions in gripper_profiles:
        if len(positions) != len(expected_gripper):
            errors.append(f'gripper profile {profile_name} has wrong position count')
            continue
        for joint, raw_value in zip(expected_gripper, positions):
            if joint in gripper:
                _check_position(
                    errors,
                    f'gripper_profile.{profile_name}',
                    joint,
                    float(raw_value),
                    gripper[joint],
                )

    calibration = fixed_config.get('calibration', {})
    if not bool(calibration.get('complete', False)):
        warnings.append('fixed_poses calibration.complete is false')

    return ValidationReport(tuple(errors), tuple(warnings))
