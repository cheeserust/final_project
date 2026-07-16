"""
Pure category, pose-reference, and workflow document helpers.

This module deliberately has no ROS or web framework imports.  It defines the
small workflow language stored in the presentation configuration and supplies
safe mutations that can be passed directly to :class:`ConfigStore.mutate`.
"""

from __future__ import annotations

from collections.abc import Mapping, MutableMapping, Sequence
import math
from typing import Any


WORKFLOW_STEP_TYPES = frozenset({
    'POSE',
    'GO_DROPOFF',
    'WAIT_SECONDS',
    # Kept only so workflows saved by older releases remain loadable.
    'WAIT_RETURN_CONFIRM',
    'GO_PICKUP',
})


class WorkflowCoreError(ValueError):
    """Raised when workflow content or a requested mutation is invalid."""


def validate_collections(document: Mapping[str, Any]) -> None:
    """
    Validate categories, poses, workflows, and their cross-references.

    Numeric joint ranges are validated by ``config_store.validate_document``;
    this function owns the collection shapes and semantic relationships.
    """
    if not isinstance(document, Mapping):
        raise WorkflowCoreError('document must be a mapping')

    categories = _mapping(document.get('categories'), 'categories')
    poses = _mapping(document.get('poses'), 'poses')
    workflows = _mapping(document.get('workflows'), 'workflows')

    uncategorized_id = canonical_entity_id(
        document.get('uncategorized_category_id'),
        field_name='uncategorized_category_id',
    )
    if uncategorized_id not in categories:
        raise WorkflowCoreError(
            'uncategorized_category_id must reference an existing category'
        )

    seen_orders: set[int] = set()
    seen_names: dict[str, str] = {}
    for category_id, category in categories.items():
        _canonical_mapping_key(category_id, 'category')
        canonical_entity_id(category_id, field_name='category id')
        category_map = _mapping(
            category,
            f'categories.{category_id}',
        )
        _exact_fields(
            category_map,
            {'name', 'protected', 'order'},
            f'categories.{category_id}',
        )
        category_name = _nonempty_string(
            category_map['name'],
            f'categories.{category_id}.name',
            maximum=80,
        )
        name_key = _category_name_key(category_name)
        previous_id = seen_names.get(name_key)
        if previous_id is not None:
            raise WorkflowCoreError(
                'duplicate category name: '
                f'{category_name!r} (categories {previous_id} and {category_id})'
            )
        seen_names[name_key] = str(category_id)
        if not isinstance(category_map['protected'], bool):
            raise WorkflowCoreError(
                f'categories.{category_id}.protected must be a boolean'
            )
        order = _integer(
            category_map['order'],
            f'categories.{category_id}.order',
            minimum=0,
        )
        if order in seen_orders:
            raise WorkflowCoreError(f'duplicate category order: {order}')
        seen_orders.add(order)

    if not categories[uncategorized_id]['protected']:
        raise WorkflowCoreError(
            f'categories.{uncategorized_id}.protected must be true'
        )

    pose_ids = set()
    for raw_pose_id, pose in poses.items():
        _canonical_mapping_key(raw_pose_id, 'pose')
        pose_id = canonical_entity_id(raw_pose_id, field_name='pose id')
        if pose_id in pose_ids:
            raise WorkflowCoreError(f'duplicate pose id: {pose_id}')
        pose_ids.add(pose_id)
        pose_map = _mapping(pose, f'poses.{pose_id}')
        _exact_fields(
            pose_map,
            {'name', 'category_id', 'arm', 'gripper', 'dwell_sec'},
            f'poses.{pose_id}',
        )
        _nonempty_string(
            pose_map['name'],
            f'poses.{pose_id}.name',
            maximum=100,
        )
        _known_category(
            pose_map['category_id'],
            categories,
            f'poses.{pose_id}.category_id',
        )

    for workflow_id, workflow in workflows.items():
        _canonical_mapping_key(workflow_id, 'workflow')
        canonical_entity_id(workflow_id, field_name='workflow id')
        workflow_map = _mapping(
            workflow,
            f'workflows.{workflow_id}',
        )
        _exact_fields(
            workflow_map,
            {'name', 'category_id', 'steps'},
            f'workflows.{workflow_id}',
        )
        _nonempty_string(
            workflow_map['name'],
            f'workflows.{workflow_id}.name',
            maximum=100,
        )
        _known_category(
            workflow_map['category_id'],
            categories,
            f'workflows.{workflow_id}.category_id',
        )
        validate_workflow_steps(
            workflow_map['steps'],
            pose_ids=pose_ids,
            path=f'workflows.{workflow_id}.steps',
        )


def validate_workflow_steps(
    steps: Any,
    *,
    pose_ids: set[str] | None = None,
    path: str = 'steps',
) -> None:
    """Validate a list using the intentionally small v1 workflow language."""
    if (
        not isinstance(steps, Sequence)
        or isinstance(steps, (str, bytes, bytearray))
    ):
        raise WorkflowCoreError(f'{path} must be a list')
    if not steps:
        raise WorkflowCoreError(f'{path} must not be empty')
    if len(steps) > 200:
        raise WorkflowCoreError(f'{path} must contain at most 200 steps')

    outbound_pending = False
    route_location: str | None = None
    for index, raw_step in enumerate(steps):
        step_path = f'{path}[{index}]'
        step = _mapping(raw_step, step_path)
        step_type = step.get('type')
        if step_type not in WORKFLOW_STEP_TYPES:
            allowed = ', '.join(sorted(WORKFLOW_STEP_TYPES))
            raise WorkflowCoreError(
                f'{step_path}.type must be one of: {allowed}'
            )

        if step_type == 'POSE':
            _exact_fields(step, {'type', 'pose_id'}, step_path)
            pose_id = canonical_entity_id(step['pose_id'], field_name='pose id')
            if pose_ids is not None and pose_id not in pose_ids:
                raise WorkflowCoreError(
                    f'{step_path}.pose_id references missing pose {pose_id}'
                )
        elif step_type == 'WAIT_SECONDS':
            _exact_fields(step, {'type', 'seconds'}, step_path)
            _finite_number(
                step['seconds'],
                f'{step_path}.seconds',
                minimum=0.0,
                maximum=3600.0,
            )
        else:
            _exact_fields(step, {'type'}, step_path)

        if step_type == 'GO_DROPOFF':
            if route_location == 'dropoff':
                raise WorkflowCoreError(
                    f'{step_path} repeats GO_DROPOFF without returning'
                )
            route_location = 'dropoff'
            outbound_pending = True
        elif step_type == 'WAIT_RETURN_CONFIRM':
            if not outbound_pending:
                raise WorkflowCoreError(
                    f'{step_path} is only valid after GO_DROPOFF'
                )
        elif step_type == 'GO_PICKUP':
            if route_location == 'pickup':
                raise WorkflowCoreError(
                    f'{step_path} repeats GO_PICKUP without going to dropoff'
                )
            route_location = 'pickup'
            outbound_pending = False


def add_category(
    document: MutableMapping[str, Any],
    category_id: int | str,
    name: str,
    *,
    order: int | None = None,
) -> dict[str, Any]:
    """Add a non-protected category to ``document`` in place."""
    validate_collections(document)
    canonical = canonical_entity_id(category_id, field_name='category_id')
    _nonempty_string(name, 'category name', maximum=80)
    categories = document['categories']
    if canonical in categories:
        raise WorkflowCoreError(f'category {canonical} already exists')
    name_key = _category_name_key(name)
    if any(
        _category_name_key(category['name']) == name_key
        for category in categories.values()
    ):
        raise WorkflowCoreError(f'category name {name.strip()!r} already exists')

    used_orders = {category['order'] for category in categories.values()}
    if order is None:
        order = 0
        while order in used_orders:
            order += 1
    else:
        _integer(order, 'category order', minimum=0)
        if order in used_orders:
            raise WorkflowCoreError(f'category order {order} is already used')

    category = {'name': name.strip(), 'protected': False, 'order': order}
    categories[canonical] = category
    numeric_id = int(canonical)
    if numeric_id >= document['next_category_id']:
        document['next_category_id'] = numeric_id + 1
    validate_collections(document)
    return category


def rename_category(
    document: MutableMapping[str, Any],
    category_id: int | str,
    name: str,
) -> dict[str, Any]:
    """Change a category's display name without changing its stable ID."""
    validate_collections(document)
    _nonempty_string(name, 'category name', maximum=80)
    category = _find_category(document, category_id)
    canonical = canonical_entity_id(category_id, field_name='category_id')
    name_key = _category_name_key(name)
    if any(
        key != canonical and _category_name_key(value['name']) == name_key
        for key, value in document['categories'].items()
    ):
        raise WorkflowCoreError(f'category name {name.strip()!r} already exists')
    category['name'] = name.strip()
    validate_collections(document)
    return category


def _category_name_key(name: Any) -> str:
    """Normalize display names for duplicate checks without changing labels."""
    return ' '.join(str(name).split()).casefold()


def delete_category(
    document: MutableMapping[str, Any],
    category_id: int | str,
) -> dict[str, int]:
    """Delete a category and move its members to ``uncategorized``."""
    validate_collections(document)
    category = _find_category(document, category_id)
    if category['protected']:
        raise WorkflowCoreError(
            f'protected category {category_id!r} cannot be deleted'
        )

    moved_poses = 0
    canonical = canonical_entity_id(category_id, field_name='category_id')
    uncategorized_id = int(document['uncategorized_category_id'])
    for pose in document['poses'].values():
        if canonical_entity_id(
            pose['category_id'], field_name='pose category_id'
        ) == canonical:
            pose['category_id'] = uncategorized_id
            moved_poses += 1

    moved_workflows = 0
    for workflow in document['workflows'].values():
        if canonical_entity_id(
            workflow['category_id'], field_name='workflow category_id'
        ) == canonical:
            workflow['category_id'] = uncategorized_id
            moved_workflows += 1

    del document['categories'][canonical]
    validate_collections(document)
    return {
        'moved_poses': moved_poses,
        'moved_workflows': moved_workflows,
    }


def referenced_pose_ids(
    document: Mapping[str, Any],
) -> dict[str, list[str]]:
    """Return workflow paths grouped by referenced pose ID."""
    validate_collections(document)
    references: dict[str, list[str]] = {}
    for workflow_id, workflow in document['workflows'].items():
        for index, step in enumerate(workflow['steps']):
            if step['type'] != 'POSE':
                continue
            pose_id = canonical_entity_id(step['pose_id'], field_name='pose id')
            references.setdefault(pose_id, []).append(
                f'workflows.{workflow_id}.steps[{index}]'
            )
    return references


def delete_pose(
    document: MutableMapping[str, Any],
    pose_id: int | str,
) -> dict[str, Any]:
    """
    Delete an unreferenced pose and return it.

    IDs are deliberately not recycled; ``next_pose_id`` remains unchanged.
    """
    validate_collections(document)
    canonical = canonical_entity_id(pose_id, field_name='pose id')
    poses = document['poses']
    if canonical not in poses:
        raise WorkflowCoreError(f'pose {canonical} does not exist')
    references = referenced_pose_ids(document).get(canonical, [])
    if references:
        raise WorkflowCoreError(
            f'pose {canonical} is referenced by: {", ".join(references)}'
        )
    pose = poses.pop(canonical)
    validate_collections(document)
    return pose


def canonical_entity_id(value: Any, *, field_name: str = 'id') -> str:
    """Return the canonical string representation of a positive entity ID."""
    if isinstance(value, bool):
        raise WorkflowCoreError(f'{field_name} must be a positive integer')
    if isinstance(value, int):
        if value <= 0:
            raise WorkflowCoreError(
                f'{field_name} must be a positive integer'
            )
        return str(value)
    if isinstance(value, str) and value.isdigit() and value[0] != '0':
        return value
    raise WorkflowCoreError(
        f'{field_name} must be a canonical positive integer'
    )


# Backward-friendly descriptive alias used by callers dealing specifically
# with pose references.
canonical_pose_id = canonical_entity_id


def _canonical_mapping_key(value: Any, entity_name: str) -> None:
    if not isinstance(value, str):
        raise WorkflowCoreError(
            f'{entity_name} mapping keys must be canonical positive-integer strings'
        )


def _find_category(
    document: MutableMapping[str, Any],
    category_id: int | str,
) -> MutableMapping[str, Any]:
    canonical = canonical_entity_id(category_id, field_name='category_id')
    category = document['categories'].get(canonical)
    if category is None:
        raise WorkflowCoreError(f'category {canonical} does not exist')
    return category


def _known_category(
    value: Any,
    categories: Mapping[str, Any],
    path: str,
) -> None:
    canonical = canonical_entity_id(value, field_name=path)
    if canonical not in categories:
        raise WorkflowCoreError(f'{path} references unknown category {value!r}')


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise WorkflowCoreError(f'{path} must be a mapping')
    return value


def _exact_fields(
    value: Mapping[str, Any],
    expected: set[str],
    path: str,
) -> None:
    actual = set(value)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing:
        raise WorkflowCoreError(
            f'{path} is missing fields: {", ".join(missing)}'
        )
    if unknown:
        raise WorkflowCoreError(
            f'{path} has unknown fields: {", ".join(unknown)}'
        )


def _nonempty_string(
    value: Any,
    path: str,
    *,
    maximum: int,
) -> str:
    if not isinstance(value, str) or not value.strip():
        raise WorkflowCoreError(f'{path} must be a non-empty string')
    if len(value) > maximum:
        raise WorkflowCoreError(f'{path} must be at most {maximum} characters')
    return value


def _integer(
    value: Any,
    path: str,
    *,
    minimum: int,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise WorkflowCoreError(
            f'{path} must be an integer greater than or equal to {minimum}'
        )
    return value


def _finite_number(
    value: Any,
    path: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < minimum
        or float(value) > maximum
    ):
        raise WorkflowCoreError(
            f'{path} must be a finite number in [{minimum}, {maximum}]'
        )
    return float(value)
