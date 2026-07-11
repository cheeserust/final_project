import json
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
            object_label='box',
            arm_task_name='pick_object_2',
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
            object_label='box',
            arm_task_name='pick_object_2',
        )
    )

    by_state = {step.state: step for step in plan}
    task_ids = [step.task_id for step in plan]

    assert 'marker_cmd_vel' not in task_ids
    assert by_state['GO_TO_ELEVATOR_FRONT'].target_name == 'elevator_front'
    assert by_state['GO_TO_ELEVATOR_FRONT'].server == '/nav/go_to'
    assert by_state['ENTER_ELEVATOR'].task_id == 'board_elevator'
    assert by_state['ENTER_ELEVATOR'].server == '/elevator/board'
    assert by_state['EXIT_ELEVATOR'].task_id == 'exit_elevator'
    assert by_state['EXIT_ELEVATOR'].server == '/elevator/exit'
    assert by_state['EXIT_ELEVATOR'].target_floor == 5
    assert by_state['EXIT_ELEVATOR'].marker_id == 5
    assert by_state['RETURN_TO_ELEVATOR'].target_name == 'elevator_front'
    assert by_state['ENTER_ELEVATOR_RETURN'].task_id == 'board_elevator'
    assert by_state['EXIT_ELEVATOR_RETURN'].target_floor == 4
    assert by_state['EXIT_ELEVATOR_RETURN'].marker_id == 4
    assert by_state['WAIT_5F'].marker_id == 5
    assert by_state['WAIT_4F'].marker_id == 4
    assert by_state['ARM_TASK_AT_TARGET'].server == '/arm/execute'
    assert json.loads(
        by_state['ARM_TASK_AT_TARGET'].extra_json
    )['arm_task_name'] == 'pick_object_2'

    states = [step.state for step in plan]
    assert states.index('SWITCH_5F_MAP') > states.index('EXIT_ELEVATOR')
    assert states.index('SWITCH_4F_MAP') > states.index(
        'EXIT_ELEVATOR_RETURN'
    )
