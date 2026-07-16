"""Keep runtime retry and timeout tuning in one external YAML file."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = PACKAGE_ROOT / 'config' / 'retry_timeout.yaml'
MAIN_CONFIG_PATH = PACKAGE_ROOT / 'config' / 'arm_can_bridge.yaml'

PROFILE_DEFAULTS = {
    'arm_v3_ready_timeout_ms': 250,
    'arm_v3_max_stage_attempts': 200,
    'arm_v3_communication_timeout_ms': 1000,
    'can_tx_retry_count': 10,
    'can_tx_retry_delay_ms': 10.0,
    'can_batch_inter_frame_delay_ms': 8.0,
    'can_writer_batch_timeout_ms': 2000,
    'clear_active_goal_timeout_ms': 7000,
    'control_wait_timeout_ms': 3000,
    'homing_wait_timeout_ms': 180000,
    'status_timeout_ms': 500,
    'queue_wait_timeout_ms': 3000,
    'completion_grace_ms': 3000,
}


def parameters(path):
    """Load the arm_can_bridge ROS parameter mapping from ``path``."""
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    return data['arm_can_bridge']['ros__parameters']


def test_retry_timeout_profile_contains_all_operational_tuning_values():
    assert parameters(PROFILE_PATH) == PROFILE_DEFAULTS


def test_main_hardware_config_does_not_duplicate_retry_timeout_values():
    main_parameters = parameters(MAIN_CONFIG_PATH)
    assert PROFILE_DEFAULTS.keys().isdisjoint(main_parameters)


def test_every_bridge_launch_loads_the_retry_timeout_profile():
    launches = (PACKAGE_ROOT / 'launch').glob('arm_can_bridge*.launch.py')
    for launch in launches:
        text = launch.read_text(encoding='utf-8')
        assert "'retry_timeout.yaml'" in text, launch.name
