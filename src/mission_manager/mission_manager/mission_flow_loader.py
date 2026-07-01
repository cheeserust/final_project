import json
import os
from typing import Any, Dict, List

import yaml

from .mission_state import MissionContext, MissionStep


class MissionConfigError(ValueError):
    """미션 설정 파일의 형식이나 값이 잘못된 경우 발생한다."""


class MissionFlowLoader:
    def __init__(
        self,
        mission_flow_file: str,
        locations_file: str,
        action_servers_file: str,
    ):
        self._mission_flow_file = mission_flow_file
        self._locations_file = locations_file
        self._action_servers_file = action_servers_file

        flow_data = self._load_yaml(mission_flow_file)
        location_data = self._load_yaml(locations_file)
        action_data = self._load_yaml(action_servers_file)

        try:
            self._raw_steps = flow_data['mission']['steps']
        except (KeyError, TypeError) as exc:
            raise MissionConfigError(
                'mission_flow.yaml must contain mission.steps'
            ) from exc

        try:
            self._locations = location_data['locations']
        except (KeyError, TypeError) as exc:
            raise MissionConfigError(
                'locations.yaml must contain locations'
            ) from exc

        try:
            self._actions = action_data['actions']
        except (KeyError, TypeError) as exc:
            raise MissionConfigError(
                'action_servers.yaml must contain actions'
            ) from exc

        self._validate_static_config()

    @staticmethod
    def _load_yaml(path: str) -> Dict[str, Any]:
        if not os.path.exists(path):
            raise FileNotFoundError(f'Config file not found: {path}')

        with open(path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise MissionConfigError(
                f'YAML root must be a mapping: {path}'
            )

        return data

    def _validate_static_config(self) -> None:
        if not isinstance(self._raw_steps, list) or not self._raw_steps:
            raise MissionConfigError(
                'mission.steps must be a non-empty list'
            )

        if not isinstance(self._locations, dict):
            raise MissionConfigError('locations must be a mapping')

        if not isinstance(self._actions, dict):
            raise MissionConfigError('actions must be a mapping')

        for index, step in enumerate(self._raw_steps):
            if not isinstance(step, dict):
                raise MissionConfigError(
                    f'mission.steps[{index}] must be a mapping'
                )

            for key in ('state', 'task', 'target', 'location'):
                if key not in step:
                    raise MissionConfigError(
                        f'mission.steps[{index}] missing key: {key}'
                    )

            task_profile = str(step['task'])

            if task_profile not in self._actions:
                raise MissionConfigError(
                    f'Unknown task profile "{task_profile}" '
                    f'at mission.steps[{index}]'
                )

        for name, action in self._actions.items():
            if not isinstance(action, dict):
                raise MissionConfigError(
                    f'actions.{name} must be a mapping'
                )

            for key in ('server', 'timeout_sec', 'retry'):
                if key not in action:
                    raise MissionConfigError(
                        f'actions.{name} missing key: {key}'
                    )

            if float(action['timeout_sec']) <= 0.0:
                raise MissionConfigError(
                    f'actions.{name}.timeout_sec must be greater than zero'
                )

            if int(action['retry']) < 0:
                raise MissionConfigError(
                    f'actions.{name}.retry cannot be negative'
                )

    @staticmethod
    def _context_values(context: MissionContext) -> Dict[str, Any]:
        return {
            'mission_id': context.mission_id,
            'pickup_location': context.pickup_location,
            'delivery_location': context.delivery_location,
            'target_floor': context.target_floor,
            'object_label': context.object_label,
        }

    def _resolve_value(
        self,
        value: Any,
        context_values: Dict[str, Any],
    ) -> Any:
        """Replace MissionContext variables recursively."""
        if isinstance(value, str) and value.startswith('$'):
            key = value[1:]

            if key not in context_values:
                raise MissionConfigError(
                    f'Unknown mission variable: {value}'
                )

            return context_values[key]

        if isinstance(value, list):
            return [
                self._resolve_value(item, context_values)
                for item in value
            ]

        if isinstance(value, dict):
            return {
                key: self._resolve_value(item, context_values)
                for key, item in value.items()
            }

        return value

    def _resolve_marker_id(
        self,
        location_name: str,
        location: Dict[str, Any],
        floor: int,
        context_values: Dict[str, Any],
    ) -> int:
        if 'marker_id_by_floor' in location:
            marker_map = location['marker_id_by_floor']

            if not isinstance(marker_map, dict):
                raise MissionConfigError(
                    f'locations.{location_name}.marker_id_by_floor '
                    'must be a mapping'
                )

            if floor in marker_map:
                marker_value = marker_map[floor]
            elif str(floor) in marker_map:
                marker_value = marker_map[str(floor)]
            else:
                raise MissionConfigError(
                    f'No marker ID configured for floor {floor} '
                    f'in location "{location_name}"'
                )

            return int(
                self._resolve_value(marker_value, context_values)
            )

        return int(
            self._resolve_value(
                location.get('marker_id', -1),
                context_values,
            )
        )

    def _resolve_location_floor(
        self,
        location_name: str,
        context_values: Dict[str, Any],
    ) -> int:
        if location_name not in self._locations:
            raise MissionConfigError(f'Unknown location "{location_name}"')

        location = self._locations[location_name]
        if not isinstance(location, dict):
            raise MissionConfigError(
                f'locations.{location_name} must be a mapping'
            )

        return int(
            self._resolve_value(
                location.get('floor', -1),
                context_values,
            )
        )

    def build_plan(
        self,
        context: MissionContext,
    ) -> List[MissionStep]:
        """세 YAML 파일을 합쳐 실행 가능한 MissionStep 목록을 만든다."""
        context_values = self._context_values(context)
        pickup_floor = self._resolve_location_floor(
            context.pickup_location,
            context_values,
        )
        context_values['pickup_floor'] = pickup_floor
        context_values.update({
            'pickup_elevator_front_location': (
                f'elevator_front_{pickup_floor}f'
            ),
            'pickup_elevator_panel_location': (
                f'elevator_panel_{pickup_floor}f'
            ),
            'pickup_elevator_call_button_location': (
                f'elevator_call_button_{pickup_floor}f'
            ),
            'pickup_door_center_location': f'door_center_{pickup_floor}f',
        })
        plan: List[MissionStep] = []

        for index, raw_step in enumerate(self._raw_steps):
            skip_if_same_floor = bool(
                raw_step.get('skip_if_same_floor', False)
            )
            if skip_if_same_floor and context.target_floor == pickup_floor:
                continue

            state = str(raw_step['state'])
            task_profile = str(raw_step['task'])

            action_config = self._actions[task_profile]

            target_name = str(
                self._resolve_value(
                    raw_step['target'],
                    context_values,
                )
            )

            location_name = str(
                self._resolve_value(
                    raw_step['location'],
                    context_values,
                )
            )

            if location_name not in self._locations:
                raise MissionConfigError(
                    f'Unknown location "{location_name}" '
                    f'at mission.steps[{index}]'
                )

            location = self._locations[location_name]

            if not isinstance(location, dict):
                raise MissionConfigError(
                    f'locations.{location_name} must be a mapping'
                )

            target_floor = int(
                self._resolve_value(
                    location.get('floor', -1),
                    context_values,
                )
            )

            marker_id = self._resolve_marker_id(
                location_name=location_name,
                location=location,
                floor=target_floor,
                context_values=context_values,
            )

            extra_payload: Dict[str, Any] = {
                'location_name': location_name,
            }

            if 'type' in location:
                extra_payload['location_type'] = str(location['type'])

            # 좌표 소유권을 중앙서버가 가지기로 결정했을 때만
            # locations.yaml에 pose를 추가하면 자동 전달된다.
            if 'pose' in location:
                extra_payload['pose'] = self._resolve_value(
                    location['pose'],
                    context_values,
                )

            if 'extra' in location:
                location_extra = self._resolve_value(
                    location['extra'],
                    context_values,
                )

                if not isinstance(location_extra, dict):
                    raise MissionConfigError(
                        f'locations.{location_name}.extra '
                        'must be a mapping'
                    )

                extra_payload.update(location_extra)

            if 'extra' in raw_step:
                step_extra = self._resolve_value(
                    raw_step['extra'],
                    context_values,
                )

                if not isinstance(step_extra, dict):
                    raise MissionConfigError(
                        f'mission.steps[{index}].extra '
                        'must be a mapping'
                    )

                extra_payload.update(step_extra)

            task_id = str(
                action_config.get('task_id', task_profile)
            )

            plan.append(
                MissionStep(
                    state=state,
                    task_id=task_id,
                    server=str(action_config['server']),
                    target_name=target_name,
                    target_floor=target_floor,
                    marker_id=marker_id,
                    timeout_sec=float(action_config['timeout_sec']),
                    retry=int(action_config['retry']),
                    extra_json=json.dumps(
                        extra_payload,
                        ensure_ascii=False,
                    ),
                )
            )

        return plan
