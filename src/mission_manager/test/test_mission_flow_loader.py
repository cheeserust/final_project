import json
import math
from pathlib import Path

from mission_manager.mission_flow_loader import MissionFlowLoader
from mission_manager.mission_state import MissionContext
import yaml


def _write_yaml(path, data):
    path.write_text(yaml.safe_dump(data), encoding='utf-8')


def _base_actions():
    return {
        'actions': {
            'go_to': {
                'server': '/nav/go_to',
                'task_id': 'go_to',
                'timeout_sec': 10.0,
                'retry': 0,
            },
        },
    }


def test_points_are_expanded_to_mission_locations(tmp_path):
    flow_file = tmp_path / 'mission_flow.yaml'
    locations_file = tmp_path / 'locations.yaml'
    actions_file = tmp_path / 'action_servers.yaml'

    _write_yaml(
        flow_file,
        {
            'mission': {
                'steps': [
                    {
                        'state': 'GO_TO_ELEVATOR',
                        'task': 'go_to',
                        'target': 'elevator_front_4f',
                        'location': 'elevator_front_4f',
                    },
                    {
                        'state': 'GO_TO_ROOM',
                        'task': 'go_to',
                        'target': 'room_402',
                        'location': 'room_402',
                    },
                    {
                        'state': 'GO_TO_OBJECT_PLACE',
                        'task': 'go_to',
                        'target': 'object_place',
                        'location': 'object_place',
                    },
                ],
            },
        },
    )
    _write_yaml(
        locations_file,
        {
            'points': {
                '4': {
                    'elevator_front': {
                        'frame_id': 'map',
                        'x': 0.470,
                        'y': -0.455,
                        'yaw': 1.983,
                    },
                    '402': {
                        'frame_id': 'map',
                        'x': 2.998,
                        'y': -12.433,
                        'yaw': 0.0,
                    },
                },
                '5': {
                    'elevator_front': {
                        'frame_id': 'map',
                        'x': 15.771,
                        'y': 2.634,
                        'yaw': 0.0,
                    },
                    'object_place': {
                        'frame_id': 'map',
                        'x': 6.318,
                        'y': 1.524,
                        'yaw': 0.0,
                    },
                },
            },
        },
    )
    _write_yaml(actions_file, _base_actions())

    loader = MissionFlowLoader(
        mission_flow_file=str(flow_file),
        locations_file=str(locations_file),
        action_servers_file=str(actions_file),
    )
    plan = loader.build_plan(
        MissionContext(
            mission_id='test',
            pickup_location='room_402',
            delivery_location='object_place',
            target_floor=5,
            object_label='object_1',
            arm_task_name='deliver_object_1_from_tray',
        )
    )

    assert [step.target_name for step in plan] == [
        'elevator_front',
        '402',
        'object_place',
    ]
    assert [step.target_floor for step in plan] == [4, 4, 5]

    first_payload = json.loads(plan[0].extra_json)
    assert first_payload['pose'] == {
        'frame_id': 'map',
        'x': 0.470,
        'y': -0.455,
        'yaw': 1.983,
    }


def test_location_can_reference_point_without_repeating_pose(tmp_path):
    flow_file = tmp_path / 'mission_flow.yaml'
    locations_file = tmp_path / 'locations.yaml'
    actions_file = tmp_path / 'action_servers.yaml'

    _write_yaml(
        flow_file,
        {
            'mission': {
                'steps': [
                    {
                        'state': 'GO_TO_LEGACY_DOCK',
                        'task': 'go_to',
                        'target': 'dock',
                        'location': 'dock',
                    },
                ],
            },
        },
    )
    _write_yaml(
        locations_file,
        {
            'points': {
                '4': {
                    'dock': {
                        'frame_id': 'map',
                        'x': 3.748,
                        'y': -18.219,
                        'yaw': 0.0,
                    },
                },
                '5': {
                    'dock': {
                        'frame_id': 'map',
                        'x': -0.226,
                        'y': -0.154,
                        'yaw': 0.0,
                    },
                },
            },
            'locations': {
                'dock': {
                    'floor': 4,
                    'point': 'dock',
                    'type': 'dock',
                },
            },
        },
    )
    _write_yaml(actions_file, _base_actions())

    loader = MissionFlowLoader(
        mission_flow_file=str(flow_file),
        locations_file=str(locations_file),
        action_servers_file=str(actions_file),
    )
    plan = loader.build_plan(
        MissionContext(
            mission_id='test',
            pickup_location='dock',
            delivery_location='dock',
            target_floor=4,
            object_label='box',
            arm_task_name='pick_object_2',
        )
    )

    assert plan[0].target_floor == 4
    assert plan[0].target_name == 'dock'
    assert json.loads(plan[0].extra_json)['pose']['x'] == 3.748


def test_project_config_matches_driving_team_task_servers():
    config_dir = Path(__file__).resolve().parents[1] / 'config'

    loader = MissionFlowLoader(
        mission_flow_file=str(config_dir / 'mission_flow.yaml'),
        locations_file=str(config_dir / 'locations.yaml'),
        action_servers_file=str(config_dir / 'action_servers.yaml'),
    )
    plan = loader.build_plan(
        MissionContext(
            mission_id='test',
            pickup_location='room_402',
            delivery_location='object_place',
            target_floor=5,
            object_label='object_1',
            arm_task_name='deliver_object_1_from_tray',
        )
    )

    by_state = {step.state: step for step in plan}
    task_ids = [step.task_id for step in plan]

    assert 'marker_cmd_vel' not in task_ids
    assert by_state['GO_TO_ELEVATOR_FRONT'].target_name == 'elevator_front'
    assert by_state['GO_TO_ELEVATOR_FRONT'].server == '/nav/go_to'
    assert by_state['ENTER_ELEVATOR'].task_id == 'board_elevator'
    assert by_state['ENTER_ELEVATOR'].server == '/elevator/board'
    assert by_state['WAIT_ELEVATOR_OPEN'].task_id == 'wait_door_open'
    assert by_state['WAIT_ELEVATOR_OPEN'].server == (
        '/elevator/wait_door_open'
    )
    assert json.loads(
        by_state['WAIT_ELEVATOR_OPEN'].extra_json
    )['scan_topic'] == '/scan_filtered'
    assert by_state['EXIT_ELEVATOR'].task_id == 'exit_elevator'
    assert by_state['EXIT_ELEVATOR'].server == '/elevator/exit'
    assert by_state['EXIT_ELEVATOR'].target_floor == 5
    assert by_state['EXIT_ELEVATOR'].marker_id == 5
    assert by_state['RETURN_TO_ELEVATOR'].target_name == 'elevator_front'
    assert by_state['ENTER_ELEVATOR_RETURN'].task_id == 'board_elevator'
    assert by_state['WAIT_ELEVATOR_OPEN_RETURN'].server == (
        '/elevator/wait_door_open'
    )
    assert by_state['EXIT_ELEVATOR_RETURN'].target_floor == 4
    assert by_state['EXIT_ELEVATOR_RETURN'].marker_id == 4
    assert by_state['WAIT_5F'].marker_id == 5
    assert by_state['WAIT_4F'].marker_id == 4
    assert by_state['ARM_HOMING'].server == '/arm/homing'
    assert by_state['PICK_OBJECT_TO_TRAY'].server == '/arm/execute'
    assert by_state['DELIVER_OBJECT_FROM_TRAY'].server == '/arm/execute'
    assert json.loads(
        by_state['DELIVER_OBJECT_FROM_TRAY'].extra_json
    )['arm_task_name'] == 'deliver_object_1_from_tray'
    assert by_state['PRESS_ELEVATOR_CALL_BUTTON'].marker_id == 50
    assert by_state['PRESS_ELEVATOR_CALL_BUTTON_RETURN'].marker_id == 53
    assert by_state['PRESS_5F_BUTTON'].marker_id == 52
    assert by_state['PRESS_4F_BUTTON'].marker_id == 51
    assert json.loads(
        by_state['ALIGN_ELEVATOR_TAG'].extra_json
    )['target_distance_m'] == 1.27
    assert json.loads(
        by_state['ALIGN_ELEVATOR_TAG'].extra_json
    )['aligned_hold_sec'] == 3.0
    assert json.loads(
        by_state['ENTER_ELEVATOR'].extra_json
    )['target_distance_cm'] == 50.0
    assert by_state['READY_AND_APPROACH_ELEVATOR_4F'].server == (
        '/mission/ready_and_approach'
    )
    assert by_state['RETURN_HOME'].target_name == '402'

    states = [step.state for step in plan]
    assert states.index('FACE_ELEVATOR_4F') < states.index(
        'WAIT_ELEVATOR_OPEN'
    ) < states.index('ENTER_ELEVATOR')
    assert states.index('FACE_ELEVATOR_5F') < states.index(
        'WAIT_ELEVATOR_OPEN_RETURN'
    ) < states.index('ENTER_ELEVATOR_RETURN')
    assert states.index('SWITCH_5F_MAP') > states.index('EXIT_ELEVATOR')
    assert states.index('SWITCH_4F_MAP') > states.index(
        'EXIT_ELEVATOR_RETURN'
    )


def test_driving_only_flow_matches_full_mission_base_driving():
    config_dir = Path(__file__).resolve().parents[1] / 'config'
    context = MissionContext(
        mission_id='driving-only-test',
        pickup_location='402',
        delivery_location='object_place',
        target_floor=5,
        object_label='object_1',
        arm_task_name='deliver_object_1_from_tray',
    )

    full_plan = MissionFlowLoader(
        mission_flow_file=str(config_dir / 'mission_flow.yaml'),
        locations_file=str(config_dir / 'locations.yaml'),
        action_servers_file=str(config_dir / 'action_servers.yaml'),
    ).build_plan(context)
    driving_plan = MissionFlowLoader(
        mission_flow_file=str(
            config_dir / 'mission_flow_driving_only.yaml'
        ),
        locations_file=str(config_dir / 'locations.yaml'),
        action_servers_file=str(config_dir / 'action_servers.yaml'),
    ).build_plan(context)

    full = {step.state: step for step in full_plan}
    driving = {step.state: step for step in driving_plan}

    assert all(not step.server.startswith('/arm/') for step in driving_plan)
    assert driving['RETURN_HOME'].target_name == '402'

    common_states = (
        'GO_TO_ELEVATOR_FRONT',
        'ALIGN_ELEVATOR_TAG',
        'FACE_ELEVATOR_4F',
        'WAIT_ELEVATOR_OPEN',
        'ENTER_ELEVATOR',
        'EXIT_ELEVATOR',
        'SWITCH_5F_MAP',
        'GO_TO_TARGET_PLACE',
        'ROTATE_AT_DELIVERY',
        'RETURN_TO_ELEVATOR',
        'ALIGN_ELEVATOR_TAG_RETURN',
        'FACE_ELEVATOR_5F',
        'WAIT_ELEVATOR_OPEN_RETURN',
        'ENTER_ELEVATOR_RETURN',
        'EXIT_ELEVATOR_RETURN',
        'SWITCH_4F_MAP',
        'RETURN_HOME',
    )
    for state in common_states:
        assert driving[state] == full[state]

    assert 'WAIT_5F' not in driving
    assert 'WAIT_4F' not in driving

    gate_states = (
        'CONFIRM_ALIGNED_4F',
        'CONFIRM_EXIT_5F',
        'CONFIRM_TARGET_PLACE',
        'CONFIRM_ALIGNED_5F',
        'CONFIRM_EXIT_4F',
    )
    for state in gate_states:
        assert driving[state].server == '/operator/confirm'
        assert driving[state].task_id == 'operator_confirm'
        assert math.isinf(driving[state].timeout_sec)
        assert json.loads(driving[state].extra_json)['prompt']

    driving_states = [step.state for step in driving_plan]
    assert driving_states.index('ALIGN_ELEVATOR_TAG') < (
        driving_states.index('CONFIRM_ALIGNED_4F')
    ) < driving_states.index('DRIVE_APPROACH_ELEVATOR_4F')
    assert driving_states.index('ENTER_ELEVATOR') < (
        driving_states.index('CONFIRM_EXIT_5F')
    ) < driving_states.index('EXIT_ELEVATOR')
    assert driving_states.index('GO_TO_TARGET_PLACE') < (
        driving_states.index('CONFIRM_TARGET_PLACE')
    ) < driving_states.index('ROTATE_AT_DELIVERY')
    assert driving_states.index('ALIGN_ELEVATOR_TAG_RETURN') < (
        driving_states.index('CONFIRM_ALIGNED_5F')
    ) < driving_states.index('DRIVE_APPROACH_ELEVATOR_5F')
    assert driving_states.index('ENTER_ELEVATOR_RETURN') < (
        driving_states.index('CONFIRM_EXIT_4F')
    ) < driving_states.index('EXIT_ELEVATOR_RETURN')

    for state in ('ALIGN_ELEVATOR_TAG', 'ALIGN_ELEVATOR_TAG_RETURN'):
        assert driving[state].server == full[state].server == '/dock/align'
        assert json.loads(driving[state].extra_json)[
            'target_distance_m'
        ] == json.loads(full[state].extra_json)['target_distance_m'] == 1.27
        assert json.loads(driving[state].extra_json)[
            'aligned_hold_sec'
        ] == json.loads(full[state].extra_json)['aligned_hold_sec'] == 3.0

    drive_state_pairs = (
        (
            'DRIVE_APPROACH_ELEVATOR_4F',
            'READY_AND_APPROACH_ELEVATOR_4F',
        ),
        (
            'DRIVE_APPROACH_ELEVATOR_5F',
            'READY_AND_APPROACH_ELEVATOR_5F',
        ),
    )
    for driving_state, full_state in drive_state_pairs:
        step = driving[driving_state]
        driving_extra = json.loads(step.extra_json)
        full_extra = json.loads(full[full_state].extra_json)
        assert step.server == '/base/drive_straight'
        assert driving_extra['start_delay_sec'] == 2.0
        assert driving_extra['distance_m'] == full_extra['distance_m'] == 0.27
        assert driving_extra['speed_mps'] == full_extra['speed_mps'] == 0.15
        assert full_extra['arm_start_to_drive_delay_sec'] == 2.0

    for state in ('FACE_ELEVATOR_4F', 'FACE_ELEVATOR_5F'):
        assert driving[state].server == full[state].server == '/base/rotate'
        assert json.loads(driving[state].extra_json)[
            'angle_deg'
        ] == json.loads(full[state].extra_json)['angle_deg'] == 80.0

    board_state_pairs = (
        ('ENTER_ELEVATOR', 'ENTER_ELEVATOR'),
        ('ENTER_ELEVATOR_RETURN', 'ENTER_ELEVATOR_RETURN'),
    )
    for driving_state, full_state in board_state_pairs:
        assert driving[driving_state].server == '/elevator/board'
        assert json.loads(driving[driving_state].extra_json)[
            'target_distance_cm'
        ] == json.loads(full[full_state].extra_json)[
            'target_distance_cm'
        ] == 50.0

    assert driving['ROTATE_AT_DELIVERY'].server == '/base/rotate'
    assert json.loads(driving['GO_TO_TARGET_PLACE'].extra_json)[
        'start_delay_sec'
    ] == json.loads(full['GO_TO_TARGET_PLACE'].extra_json)[
        'start_delay_sec'
    ] == 3.0
    assert json.loads(driving['RETURN_HOME'].extra_json)[
        'start_delay_sec'
    ] == json.loads(full['RETURN_HOME'].extra_json)[
        'start_delay_sec'
    ] == 3.0
    for state in ('EXIT_ELEVATOR', 'EXIT_ELEVATOR_RETURN'):
        assert json.loads(driving[state].extra_json)[
            'exit_target_distance_cm'
        ] == json.loads(full[state].extra_json)[
            'exit_target_distance_cm'
        ] == 70.0
    assert json.loads(driving['ROTATE_AT_DELIVERY'].extra_json)[
        'angle_deg'
    ] == json.loads(full['ROTATE_AT_DELIVERY'].extra_json)[
        'angle_deg'
    ] == 180.0
