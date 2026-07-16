"""Atomic JSON persistence for manually saved robot poses."""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import tempfile
import threading
from typing import Any, Dict, Iterator, Mapping, Union


_DOCUMENT_VERSION = 1


class SavedPoseStoreError(RuntimeError):
    """Raised when the saved-pose document cannot be safely used."""


class SavedPoseNotFoundError(SavedPoseStoreError):
    """Raised when a requested saved pose does not exist."""


class SavedPoseStore:
    """Store saved poses in an atomically replaced, process-safe JSON file."""

    def __init__(self, path: Union[str, os.PathLike[str]]) -> None:
        self.path = Path(path).expanduser()
        self._lock_path = self.path.with_name(f'{self.path.name}.lock')
        self._thread_lock = threading.RLock()

    def snapshot(self) -> Dict[str, Any]:
        """Return a detached, ID-sorted snapshot of the current document."""
        with self._file_lock():
            document = self._read_document()
            return deepcopy(document)

    def get(self, pose_id: int) -> Dict[str, Any]:
        """Return one detached saved pose."""
        with self._file_lock():
            document = self._read_document()
            pose = self._find_pose(document, pose_id)
            return deepcopy(pose)

    def create(
        self,
        fields: Mapping[str, Any],
        pose_id: int | None = None,
    ) -> Dict[str, Any]:
        """Create a pose using a requested or the smallest available ID."""
        if not isinstance(fields, Mapping):
            raise SavedPoseStoreError('saved pose fields must be an object')

        with self._file_lock():
            document = self._read_document()
            now = self._timestamp()
            pose = deepcopy(dict(fields))
            used_ids = {item['id'] for item in document['poses']}
            selected_id = (
                self._first_available_id(used_ids)
                if pose_id is None
                else self._positive_id(pose_id)
            )
            if selected_id in used_ids:
                raise SavedPoseStoreError(
                    f'saved pose id {selected_id} is already in use'
                )
            pose['id'] = selected_id
            pose['created_at'] = now
            pose['updated_at'] = now

            document['poses'].append(pose)
            document['poses'].sort(key=lambda item: item['id'])
            document['next_id'] = self._first_available_id(
                used_ids | {selected_id}
            )
            self._validate_document(document)
            self._write_document(document)
            return deepcopy(pose)

    def update(
        self,
        pose_id: int,
        changes: Mapping[str, Any],
        new_pose_id: int | None = None,
    ) -> Dict[str, Any]:
        """Update mutable fields and optionally move to an unused ID."""
        if not isinstance(changes, Mapping):
            raise SavedPoseStoreError('saved pose changes must be an object')

        with self._file_lock():
            document = self._read_document()
            pose = self._find_pose(document, pose_id)
            original_id = pose['id']
            selected_id = (
                original_id
                if new_pose_id is None
                else self._positive_id(new_pose_id)
            )
            if selected_id != original_id and any(
                item['id'] == selected_id for item in document['poses']
            ):
                raise SavedPoseStoreError(
                    f'saved pose id {selected_id} is already in use'
                )
            had_created_at = 'created_at' in pose
            original_created_at = pose.get('created_at')

            pose.update(deepcopy(dict(changes)))
            pose['id'] = selected_id
            if had_created_at:
                pose['created_at'] = original_created_at
            else:
                pose.pop('created_at', None)
            pose['updated_at'] = self._timestamp()

            document['poses'].sort(key=lambda item: item['id'])
            document['next_id'] = self._first_available_id(
                {item['id'] for item in document['poses']}
            )
            self._validate_document(document)
            self._write_document(document)
            return deepcopy(pose)

    def delete(self, pose_id: int) -> Dict[str, Any]:
        """Delete and return a pose, making its ID available again."""
        with self._file_lock():
            document = self._read_document()
            pose = self._find_pose(document, pose_id)
            document['poses'].remove(pose)
            document['next_id'] = self._first_available_id(
                {item['id'] for item in document['poses']}
            )
            self._validate_document(document)
            self._write_document(document)
            return deepcopy(pose)

    @contextmanager
    def _file_lock(self) -> Iterator[None]:
        """Serialize threads and processes using a stable sibling lock file."""
        with self._thread_lock:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                lock_file = self._lock_path.open('a+b')
            except OSError as exc:
                raise SavedPoseStoreError(
                    f'could not open saved-pose lock file {self._lock_path}: {exc}'
                ) from exc

            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
                yield
            except SavedPoseStoreError:
                raise
            except OSError as exc:
                raise SavedPoseStoreError(
                    f'could not access saved-pose file {self.path}: {exc}'
                ) from exc
            finally:
                try:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
                finally:
                    lock_file.close()

    def _read_document(self) -> Dict[str, Any]:
        if not self.path.exists():
            return {
                'version': _DOCUMENT_VERSION,
                'next_id': 1,
                'poses': [],
            }

        try:
            with self.path.open('r', encoding='utf-8') as source:
                document = json.load(source)
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise SavedPoseStoreError(
                f'could not read saved-pose file {self.path}: {exc}'
            ) from exc

        self._validate_document(document)
        document['poses'].sort(key=lambda item: item['id'])
        document['next_id'] = self._first_available_id(
            {item['id'] for item in document['poses']}
        )
        return document

    def _write_document(self, document: Dict[str, Any]) -> None:
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                dir=self.path.parent,
                prefix=f'.{self.path.name}.',
                suffix='.tmp',
                delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                json.dump(
                    document,
                    temporary,
                    ensure_ascii=False,
                    indent=2,
                )
                temporary.write('\n')
                temporary.flush()
                os.fsync(temporary.fileno())

            os.replace(temporary_path, self.path)
            temporary_path = None
            self._fsync_parent_directory()
        except (OSError, TypeError, ValueError) as exc:
            raise SavedPoseStoreError(
                f'could not write saved-pose file {self.path}: {exc}'
            ) from exc
        finally:
            if temporary_path is not None:
                try:
                    temporary_path.unlink()
                except FileNotFoundError:
                    pass
                except OSError:
                    pass

    def _fsync_parent_directory(self) -> None:
        directory_fd = None
        try:
            directory_fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            os.fsync(directory_fd)
        except OSError:
            # The file replacement is already durable on common Linux filesystems;
            # directory fsync is an additional best-effort safeguard.
            pass
        finally:
            if directory_fd is not None:
                os.close(directory_fd)

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _find_pose(document: Dict[str, Any], pose_id: int) -> Dict[str, Any]:
        if isinstance(pose_id, bool) or not isinstance(pose_id, int):
            raise SavedPoseNotFoundError(f'saved pose {pose_id!r} was not found')
        for pose in document['poses']:
            if pose['id'] == pose_id:
                return pose
        raise SavedPoseNotFoundError(f'saved pose {pose_id} was not found')

    @staticmethod
    def _positive_id(pose_id: Any) -> int:
        if (
            isinstance(pose_id, bool)
            or not isinstance(pose_id, int)
            or pose_id <= 0
        ):
            raise SavedPoseStoreError(
                'saved pose id must be a positive integer'
            )
        return pose_id

    @staticmethod
    def _first_available_id(used_ids: set[int]) -> int:
        pose_id = 1
        while pose_id in used_ids:
            pose_id += 1
        return pose_id

    @staticmethod
    def _validate_document(document: Any) -> None:
        if not isinstance(document, dict):
            raise SavedPoseStoreError('saved-pose document must be an object')

        version = document.get('version')
        if isinstance(version, bool) or not isinstance(version, int):
            raise SavedPoseStoreError('saved-pose document version must be an integer')
        if version != _DOCUMENT_VERSION:
            raise SavedPoseStoreError(
                f'unsupported saved-pose document version: {version}'
            )

        next_id = document.get('next_id')
        if (
            isinstance(next_id, bool)
            or not isinstance(next_id, int)
            or next_id <= 0
        ):
            raise SavedPoseStoreError(
                'saved-pose document next_id must be a positive integer'
            )

        poses = document.get('poses')
        if not isinstance(poses, list):
            raise SavedPoseStoreError('saved-pose document poses must be a list')

        seen_ids = set()
        for pose in poses:
            if not isinstance(pose, dict):
                raise SavedPoseStoreError('every saved pose must be an object')
            pose_id = pose.get('id')
            if (
                isinstance(pose_id, bool)
                or not isinstance(pose_id, int)
                or pose_id <= 0
            ):
                raise SavedPoseStoreError(
                    'every saved pose id must be a positive integer'
                )
            if pose_id in seen_ids:
                raise SavedPoseStoreError(f'duplicate saved pose id: {pose_id}')
            seen_ids.add(pose_id)
