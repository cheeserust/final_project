"""Tests for the pure category and workflow domain helpers."""

from pathlib import Path

from final_project_presentation2.config_store import load_json_document
from final_project_presentation2.workflow_core import add_category
from final_project_presentation2.workflow_core import delete_category
from final_project_presentation2.workflow_core import delete_pose
from final_project_presentation2.workflow_core import referenced_pose_ids
from final_project_presentation2.workflow_core import rename_category
from final_project_presentation2.workflow_core import validate_collections
from final_project_presentation2.workflow_core import validate_workflow_steps
from final_project_presentation2.workflow_core import WorkflowCoreError

import pytest


TEMPLATE = (
    Path(__file__).parents[1]
    / 'config'
    / 'final_project_presentation2.json'
)


def _document():
    return load_json_document(TEMPLATE)


def _pose(category_id=2):
    return {
        'name': 'pick ready',
        'category_id': category_id,
        'arm': {
            'enabled': True,
            'positions_deg': {},
            'duration_sec': 2.0,
        },
        'gripper': {
            'enabled': False,
            'positions_deg': {},
            'duration_sec': 1.0,
            'target_load_raw': 500,
        },
        'dwell_sec': 0.2,
    }


def test_all_five_workflow_step_types_are_accepted():
    validate_workflow_steps(
        [
            {'type': 'POSE', 'pose_id': 4},
            {'type': 'GO_DROPOFF'},
            {'type': 'WAIT_SECONDS', 'seconds': 0.0},
            {'type': 'WAIT_RETURN_CONFIRM'},
            {'type': 'GO_PICKUP'},
        ],
        pose_ids={'4'},
    )


@pytest.mark.parametrize(
    'steps, message',
    [
        ([], 'must not be empty'),
        ([{'type': 'SHELL', 'command': 'true'}], 'must be one of'),
        ([{'type': 'GO_PICKUP', 'extra': 1}], 'unknown fields'),
        ([{'type': 'POSE', 'pose_id': 8}], 'missing pose 8'),
        ([{'type': 'WAIT_SECONDS', 'seconds': -1}], 'finite number'),
    ],
)
def test_workflow_step_language_is_strict(steps, message):
    with pytest.raises(WorkflowCoreError, match=message):
        validate_workflow_steps(steps, pose_ids={'4'})


def test_dropoff_to_pickup_does_not_require_user_confirmation():
    validate_workflow_steps([
        {'type': 'GO_DROPOFF'},
        {'type': 'POSE', 'pose_id': 4},
        {'type': 'GO_PICKUP'},
    ], pose_ids={'4'})

    validate_workflow_steps([
        {'type': 'GO_DROPOFF'},
        {'type': 'POSE', 'pose_id': 4},
        {'type': 'WAIT_RETURN_CONFIRM'},
        {'type': 'GO_PICKUP'},
    ], pose_ids={'4'})

    validate_workflow_steps([
        {'type': 'GO_DROPOFF'},
        {'type': 'WAIT_RETURN_CONFIRM'},
        {'type': 'POSE', 'pose_id': 4},
        {'type': 'GO_PICKUP'},
    ], pose_ids={'4'})


def test_workflow_rejects_impossible_repeated_route_transitions():
    with pytest.raises(WorkflowCoreError, match='repeats GO_DROPOFF'):
        validate_workflow_steps([
            {'type': 'GO_DROPOFF'},
            {'type': 'GO_DROPOFF'},
        ])

    with pytest.raises(WorkflowCoreError, match='repeats GO_PICKUP'):
        validate_workflow_steps([
            {'type': 'GO_PICKUP'},
            {'type': 'GO_PICKUP'},
        ])

    with pytest.raises(WorkflowCoreError, match='only valid after'):
        validate_workflow_steps([{'type': 'WAIT_RETURN_CONFIRM'}])


def test_category_crud_keeps_stable_integer_id_and_editable_name():
    document = _document()
    category = add_category(document, 8, '  새 물체  ', order=60)
    assert category['name'] == '새 물체'
    assert document['next_category_id'] == 9

    renamed = rename_category(document, 8, '새 이름')
    assert renamed['name'] == '새 이름'
    assert '8' in document['categories']
    validate_collections(document)


def test_category_names_are_unique_after_spacing_and_case_normalization():
    document = _document()
    add_category(document, 8, 'Demo Item', order=60)
    with pytest.raises(WorkflowCoreError, match='already exists'):
        add_category(document, 9, '  demo   item  ', order=70)
    with pytest.raises(WorkflowCoreError, match='already exists'):
        rename_category(document, 3, 'DEMO ITEM')


def test_deleting_category_moves_members_to_protected_uncategorized():
    document = _document()
    document['poses']['2'] = _pose(category_id=2)
    document['workflows']['1'] = {
        'name': 'baseball demo',
        'category_id': 2,
        'steps': [{'type': 'POSE', 'pose_id': 2}],
    }

    result = delete_category(document, 2)

    assert result == {'moved_poses': 1, 'moved_workflows': 1}
    assert '2' not in document['categories']
    assert document['poses']['2']['category_id'] == 7
    assert document['workflows']['1']['category_id'] == 7


def test_protected_category_cannot_be_deleted():
    document = _document()
    with pytest.raises(WorkflowCoreError, match='protected'):
        delete_category(document, document['uncategorized_category_id'])


def test_referenced_pose_cannot_be_deleted_and_paths_are_reported():
    document = _document()
    document['poses']['2'] = _pose()
    document['workflows']['1'] = {
        'name': 'baseball demo',
        'category_id': 2,
        'steps': [
            {'type': 'POSE', 'pose_id': 2},
            {'type': 'GO_DROPOFF'},
            {'type': 'POSE', 'pose_id': 2},
        ],
    }

    assert referenced_pose_ids(document) == {
        '2': [
            'workflows.1.steps[0]',
            'workflows.1.steps[2]',
        ]
    }
    with pytest.raises(WorkflowCoreError, match='workflows.1.steps'):
        delete_pose(document, 2)


def test_unreferenced_pose_delete_does_not_recycle_next_id():
    document = _document()
    document['poses']['2'] = _pose()
    document['next_pose_id'] = 3

    removed = delete_pose(document, '2')

    assert removed['name'] == 'pick ready'
    assert document['poses'] == {}
    assert document['next_pose_id'] == 3


def test_operator_can_use_and_delete_pose_id_one():
    document = _document()
    document['poses']['1'] = _pose()
    document['next_pose_id'] = 2

    removed = delete_pose(document, 1)

    assert removed['name'] == 'pick ready'
    assert document['poses'] == {}


def test_collection_keys_must_be_canonical_positive_integers():
    document = _document()
    document['workflows']['water_bottle'] = {
        'name': 'bad id',
        'category_id': 4,
        'steps': [{'type': 'GO_DROPOFF'}],
    }

    with pytest.raises(WorkflowCoreError, match='canonical positive integer'):
        validate_collections(document)

    document = _document()
    document['workflows'][1] = {
        'name': 'integer mapping key',
        'category_id': 4,
        'steps': [{'type': 'GO_DROPOFF'}],
    }
    with pytest.raises(WorkflowCoreError, match='mapping keys'):
        validate_collections(document)
