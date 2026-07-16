"""Tests for the atomic JSON saved-pose store."""

from concurrent.futures import ThreadPoolExecutor
import json

import pytest

from vicpinky_gui.saved_pose_store import SavedPoseStore, SavedPoseStoreError


def _fields(name='물건 내려놓기'):
    return {
        'name': name,
        'dwell_sec': 1.5,
        'controllers': {
            'arm': {
                'duration_sec': 1.0,
                'joints': {'arm_joint_1': 12.5},
            },
        },
    }


def test_missing_file_returns_empty_document(tmp_path):
    path = tmp_path / 'saved_poses.json'

    snapshot = SavedPoseStore(path).snapshot()

    assert snapshot == {'version': 1, 'next_id': 1, 'poses': []}
    assert not path.exists()


def test_korean_text_round_trips_without_ascii_escaping(tmp_path):
    path = tmp_path / 'saved_poses.json'
    store = SavedPoseStore(path)

    created = store.create(_fields('물건 내려놓기'))

    assert created['name'] == '물건 내려놓기'
    assert store.get(created['id'])['name'] == '물건 내려놓기'
    assert '물건 내려놓기' in path.read_text(encoding='utf-8')
    assert '\\ubb3c' not in path.read_text(encoding='utf-8')


def test_reopen_persists_pose_and_returns_detached_data(tmp_path):
    path = tmp_path / 'saved_poses.json'
    created = SavedPoseStore(path).create(_fields())

    reopened = SavedPoseStore(path)
    loaded = reopened.get(created['id'])
    loaded['controllers']['arm']['joints']['arm_joint_1'] = 99.0
    snapshot = reopened.snapshot()
    snapshot['poses'].clear()

    assert reopened.get(created['id'])['controllers']['arm']['joints'] == {
        'arm_joint_1': 12.5,
    }
    assert len(reopened.snapshot()['poses']) == 1


def test_deleted_id_is_reused_as_smallest_available_id(tmp_path):
    store = SavedPoseStore(tmp_path / 'saved_poses.json')
    first = store.create(_fields('첫 자세'))

    deleted = store.delete(first['id'])
    second = store.create(_fields('둘째 자세'))

    assert deleted == first
    assert second['id'] == 1
    assert store.snapshot()['next_id'] == 2


def test_requested_id_and_id_change_require_an_empty_slot(tmp_path):
    store = SavedPoseStore(tmp_path / 'saved_poses.json')
    first = store.create(_fields('첫 자세'), 3)
    second = store.create(_fields('둘째 자세'))

    assert first['id'] == 3
    assert second['id'] == 1
    assert store.snapshot()['next_id'] == 2

    moved = store.update(3, {'name': '이동됨'}, new_pose_id=2)
    assert moved['id'] == 2
    assert store.snapshot()['next_id'] == 3

    with pytest.raises(SavedPoseStoreError, match='already in use'):
        store.update(2, {}, new_pose_id=1)
    with pytest.raises(SavedPoseStoreError, match='already in use'):
        store.create(_fields('중복'), 1)


def test_existing_monotonic_document_recalculates_first_empty_id(tmp_path):
    path = tmp_path / 'saved_poses.json'
    path.write_text(
        json.dumps({
            'version': 1,
            'next_id': 4,
            'poses': [
                {'id': 2, **_fields('둘째 자세')},
                {'id': 3, **_fields('셋째 자세')},
            ],
        }),
        encoding='utf-8',
    )

    store = SavedPoseStore(path)
    assert store.snapshot()['next_id'] == 1
    assert store.create(_fields('첫 자세'))['id'] == 1


def test_update_preserves_identity_and_created_time(tmp_path):
    store = SavedPoseStore(tmp_path / 'saved_poses.json')
    created = store.create(_fields())

    updated = store.update(
        created['id'],
        {
            'id': 999,
            'created_at': 'not-allowed',
            'updated_at': 'not-allowed',
            'name': '수정된 이름',
            'dwell_sec': 3.0,
        },
    )

    assert updated['id'] == created['id']
    assert updated['created_at'] == created['created_at']
    assert updated['updated_at'] != 'not-allowed'
    assert updated['name'] == '수정된 이름'
    assert updated['dwell_sec'] == 3.0


def test_malformed_file_is_preserved_and_never_overwritten(tmp_path):
    path = tmp_path / 'saved_poses.json'
    malformed = b'{ definitely not JSON\n'
    path.write_bytes(malformed)
    store = SavedPoseStore(path)

    with pytest.raises(SavedPoseStoreError):
        store.snapshot()
    with pytest.raises(SavedPoseStoreError):
        store.create(_fields())

    assert path.read_bytes() == malformed


def test_invalid_document_is_preserved(tmp_path):
    path = tmp_path / 'saved_poses.json'
    invalid = {
        'version': 1,
        'next_id': 1,
        'poses': [{'id': 1}, {'id': 1}],
    }
    path.write_text(json.dumps(invalid), encoding='utf-8')
    original = path.read_bytes()

    with pytest.raises(SavedPoseStoreError, match='duplicate'):
        SavedPoseStore(path).create(_fields())

    assert path.read_bytes() == original


def test_concurrent_thread_creates_have_unique_consecutive_ids(tmp_path):
    path = tmp_path / 'saved_poses.json'

    def create(index):
        # Separate instances also exercise the cross-process-style file lock.
        return SavedPoseStore(path).create(_fields(f'자세 {index}'))

    with ThreadPoolExecutor(max_workers=8) as executor:
        created = list(executor.map(create, range(40)))

    ids = sorted(pose['id'] for pose in created)
    snapshot = SavedPoseStore(path).snapshot()
    assert ids == list(range(1, 41))
    assert [pose['id'] for pose in snapshot['poses']] == ids
    assert snapshot['next_id'] == 41
