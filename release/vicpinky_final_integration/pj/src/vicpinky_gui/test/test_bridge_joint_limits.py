"""Tests for GUI joint ranges loaded from the CAN bridge config."""

import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from vicpinky_gui.gui_node import VicPinkyGuiNode


def test_arm_gui_limits_follow_bridge_configuration():
    workspace_src = Path(__file__).resolve().parents[2]
    config_file = workspace_src / 'arm_can_bridge/config/arm_can_bridge.yaml'
    node = VicPinkyGuiNode.__new__(VicPinkyGuiNode)
    node.get_parameter = Mock(return_value=SimpleNamespace(
        value=str(config_file),
    ))
    node.get_logger = Mock(return_value=Mock())
    controller = {
        'joints': [
            {
                'joint_name': 'arm_joint_1',
                'min_deg': 0.0,
                'max_deg': 0.0,
                'default_deg': 0.0,
            },
        ],
    }

    node._apply_bridge_joint_limits(controller, 'arm')

    joint = controller['joints'][0]
    assert math.isclose(joint['min_deg'], -86.5, abs_tol=1e-5)
    assert math.isclose(joint['max_deg'], 90.0, abs_tol=1e-5)
    assert math.isclose(joint['default_deg'], -86.5, abs_tol=1e-5)
