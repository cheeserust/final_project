import json
from pathlib import Path

from mission_manager.mission_flow_loader import MissionFlowLoader
from mission_manager.mission_state import MissionContext


def _plan():
    package_root = Path(__file__).resolve().parents[1]
    loader = MissionFlowLoader(
        str(package_root / 'config' / 'mission_flow.yaml'),
        str(package_root / 'config' / 'locations.yaml'),
        str(package_root / 'config' / 'action_servers.yaml'),
    )
    return loader.build_plan(
        MissionContext(
            mission_id='native-floor-test',
            pickup_location='object',
            delivery_location='object_place',
            target_floor=5,
            object_label='box',
        )
    )


def test_exit_switch_and_delayed_nav_order_on_both_floors():
    plan = _plan()
    by_state = {step.state: step for step in plan}
    states = [step.state for step in plan]

    assert states.index('EXIT_ELEVATOR_5F') < states.index('SWITCH_5F_MAP')
    assert states.index('SWITCH_5F_MAP') < states.index(
        'GO_TO_DELIVERY_LOCATION'
    )
    assert states.index('EXIT_ELEVATOR_4F') < states.index('SWITCH_4F_MAP')
    assert states.index('SWITCH_4F_MAP') < states.index('RETURN_TO_START')

    exit_5f = by_state['EXIT_ELEVATOR_5F']
    assert exit_5f.server == '/elevator/exit'
    assert exit_5f.target_floor == 5
    assert json.loads(exit_5f.extra_json)['exit_target_distance_cm'] == 70.0

    exit_4f = by_state['EXIT_ELEVATOR_4F']
    assert exit_4f.server == '/elevator/exit'
    assert exit_4f.target_floor == 4
    assert json.loads(exit_4f.extra_json)['exit_target_distance_cm'] == 70.0

    go_5f = by_state['GO_TO_DELIVERY_LOCATION']
    assert go_5f.server == '/nav/go_to'
    assert json.loads(go_5f.extra_json)['start_delay_sec'] == 3.0

    go_4f = by_state['RETURN_TO_START']
    assert go_4f.server == '/nav/go_to'
    assert json.loads(go_4f.extra_json)['start_delay_sec'] == 3.0
