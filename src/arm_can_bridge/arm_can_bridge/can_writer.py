"""Serialize every outbound frame through one retrying CAN writer."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import errno
import threading
import time
from typing import Callable, Iterable, Mapping

from .can_protocol import CanFrame
from .socketcan_transport import SocketCanTransport, SocketCanTransportError


@dataclass(frozen=True)
class WriteReceipt:
    """Timing evidence for one completely transmitted application batch."""

    enqueued_at: float
    started_at: float
    completed_at: float


@dataclass
class _WriteRequest:
    frames: tuple[CanFrame, ...]
    goal_id: int | None
    category: str
    enqueued_at: float = field(default_factory=time.monotonic)
    started_at: float | None = None
    completed_at: float | None = None
    done: threading.Event = field(default_factory=threading.Event)
    error: Exception | None = None


class SerializedCanWriter:
    """Own the only application path that calls ``send_frame``."""

    MOTION_CATEGORIES = frozenset({'goal', 'start', 'cancel', 'motion'})

    def __init__(
        self,
        transport: SocketCanTransport,
        *,
        retry_count: int = 3,
        retry_delay_s: float = 0.01,
        batch_inter_frame_delay_s: float = 0.008,
        request_timeout_s: float = 2.0,
        event_callback: Callable[[Mapping[str, object]], None] | None = None,
    ) -> None:
        if retry_count < 0:
            raise ValueError('retry_count cannot be negative')
        if retry_delay_s < 0.0:
            raise ValueError('retry_delay_s cannot be negative')
        if batch_inter_frame_delay_s < 0.0:
            raise ValueError(
                'batch_inter_frame_delay_s cannot be negative'
            )
        if request_timeout_s <= 0.0:
            raise ValueError('request_timeout_s must be positive')
        self._transport = transport
        self._retry_count = int(retry_count)
        self._retry_delay_s = float(retry_delay_s)
        self._batch_inter_frame_delay_s = float(batch_inter_frame_delay_s)
        self._request_timeout_s = float(request_timeout_s)
        self._event_callback = event_callback
        self._queue: deque[_WriteRequest] = deque()
        self._condition = threading.Condition()
        self._active_request: _WriteRequest | None = None
        self._stopped = False
        self._thread = threading.Thread(
            target=self._run,
            name='vicpinky-can-writer',
            daemon=True,
        )
        self._thread.start()

    def send_batch(
        self,
        frames: Iterable[CanFrame],
        *,
        goal_id: int | None = None,
        timeout_s: float | None = None,
        category: str = 'control',
    ) -> WriteReceipt:
        """Enqueue an atomic application batch and wait for its final frame."""
        request = _WriteRequest(
            tuple(frames),
            None if goal_id is None else int(goal_id),
            str(category),
        )
        if not request.frames:
            raise ValueError('CAN batch cannot be empty')
        self._enqueue(request)
        return self._wait_request(
            request,
            self._request_timeout_s if timeout_s is None else timeout_s,
        )

    def send_emergency(
        self,
        frame: CanFrame,
        *,
        timeout_s: float | None = None,
    ) -> WriteReceipt:
        """Drop queued motion and put one E-stop ahead of other queued work."""
        request = _WriteRequest((frame,), None, 'estop')
        with self._condition:
            if self._stopped:
                raise RuntimeError('CAN writer is stopped')
            removed = self._discard_queued_locked(
                lambda item: item.category in self.MOTION_CATEGORIES,
                'CAN motion batch discarded by E-stop',
            )
            self._queue.appendleft(request)
            self._condition.notify()
        self._emit(
            'estop_enqueued',
            discarded_batches=removed,
            can_id=frame.can_id,
            payload=frame.data.hex().upper(),
        )
        return self._wait_request(
            request,
            self._request_timeout_s if timeout_s is None else timeout_s,
        )

    def send_frame(self, frame: CanFrame) -> WriteReceipt:
        """Compatibility entry point for Board3's unchanged streamer."""
        return self.send_batch((frame,), category='motion')

    def discard_goal(self, goal_id: int) -> int:
        """Remove queued, not-currently-sending batches for one goal."""
        with self._condition:
            return self._discard_queued_locked(
                lambda item: item.goal_id == int(goal_id),
                f'CAN batch discarded for canceled goal {goal_id}',
            )

    def wait_goal_idle(
        self,
        goal_id: int,
        timeout_s: float | None = None,
    ) -> bool:
        """Wait until no active or queued batch belongs to ``goal_id``."""
        effective_timeout = (
            self._request_timeout_s if timeout_s is None else float(timeout_s)
        )
        deadline = time.monotonic() + effective_timeout
        with self._condition:
            while time.monotonic() < deadline:
                active = (
                    self._active_request is not None
                    and self._active_request.goal_id == int(goal_id)
                )
                queued = any(
                    item.goal_id == int(goal_id) for item in self._queue
                )
                if not active and not queued:
                    return True
                self._condition.wait(timeout=0.01)
            return False

    def close(self) -> None:
        """Finish the active write, fail queued requests, and stop."""
        with self._condition:
            if self._stopped:
                return
            self._stopped = True
            self._discard_queued_locked(
                lambda item: True,
                'CAN writer closed before batch transmission',
            )
            self._condition.notify_all()
        self._thread.join(timeout=2.0)

    def _enqueue(self, request: _WriteRequest) -> None:
        with self._condition:
            if self._stopped:
                raise RuntimeError('CAN writer is stopped')
            self._queue.append(request)
            self._condition.notify()
        self._emit(
            'batch_enqueued',
            category=request.category,
            goal_id=request.goal_id,
            frame_count=len(request.frames),
        )

    def _wait_request(
        self,
        request: _WriteRequest,
        timeout_s: float,
    ) -> WriteReceipt:
        if not request.done.wait(float(timeout_s)):
            removed = False
            with self._condition:
                try:
                    self._queue.remove(request)
                    removed = True
                except ValueError:
                    pass
                if removed:
                    request.error = SocketCanTransportError(
                        'Timeout waiting for queued CAN writer request'
                    )
                    request.done.set()
            state = 'queued request removed' if removed else 'write in progress'
            raise SocketCanTransportError(
                f'Timeout waiting for CAN writer ({state})'
            )
        if request.error is not None:
            raise request.error
        if request.started_at is None or request.completed_at is None:
            raise SocketCanTransportError('CAN writer produced no timing receipt')
        return WriteReceipt(
            enqueued_at=request.enqueued_at,
            started_at=request.started_at,
            completed_at=request.completed_at,
        )

    def _discard_queued_locked(
        self,
        predicate: Callable[[_WriteRequest], bool],
        reason: str,
    ) -> int:
        kept: deque[_WriteRequest] = deque()
        removed = 0
        while self._queue:
            item = self._queue.popleft()
            if predicate(item):
                item.error = RuntimeError(reason)
                item.done.set()
                removed += 1
            else:
                kept.append(item)
        self._queue = kept
        self._condition.notify_all()
        return removed

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._queue and not self._stopped:
                    self._condition.wait()
                if self._stopped and not self._queue:
                    return
                request = self._queue.popleft()
                self._active_request = request
                request.started_at = time.monotonic()
            try:
                for index, frame in enumerate(request.frames):
                    self._send_with_retry(
                        frame,
                        request=request,
                        frame_index=index,
                    )
                    if (
                        index + 1 < len(request.frames)
                        and self._batch_inter_frame_delay_s > 0.0
                    ):
                        time.sleep(self._batch_inter_frame_delay_s)
            except Exception as exc:  # propagate to submitting callback
                request.error = exc
            finally:
                request.completed_at = time.monotonic()
                request.done.set()
                with self._condition:
                    self._active_request = None
                    self._condition.notify_all()
                self._emit(
                    'batch_complete',
                    category=request.category,
                    goal_id=request.goal_id,
                    success=request.error is None,
                    elapsed_ms=(
                        (request.completed_at - request.started_at) * 1000.0
                    ),
                    error=(None if request.error is None else str(request.error)),
                )

    def _send_with_retry(
        self,
        frame: CanFrame,
        *,
        request: _WriteRequest,
        frame_index: int,
    ) -> None:
        for attempt in range(self._retry_count + 1):
            started_at = time.monotonic()
            try:
                self._transport.send_frame(frame)
                self._emit(
                    'frame_sent',
                    category=request.category,
                    goal_id=request.goal_id,
                    frame_index=frame_index,
                    can_id=frame.can_id,
                    payload=frame.data.hex().upper(),
                    attempt=attempt + 1,
                    elapsed_ms=(time.monotonic() - started_at) * 1000.0,
                )
                return
            except SocketCanTransportError as exc:
                cause = exc.__cause__
                error_number = (
                    cause.errno if isinstance(cause, OSError) else None
                )
                retryable = error_number in (errno.ENOBUFS, errno.EAGAIN)
                self._emit(
                    'frame_send_error',
                    category=request.category,
                    goal_id=request.goal_id,
                    frame_index=frame_index,
                    can_id=frame.can_id,
                    payload=frame.data.hex().upper(),
                    attempt=attempt + 1,
                    errno=error_number,
                    enobufs=error_number == errno.ENOBUFS,
                    retryable=retryable,
                    error=str(exc),
                )
                if not retryable or attempt >= self._retry_count:
                    raise
                time.sleep(self._retry_delay_s * (attempt + 1))

    def _emit(self, event: str, **fields: object) -> None:
        callback = self._event_callback
        if callback is None:
            return
        payload: dict[str, object] = {
            'component': 'can_writer',
            'event': event,
            'monotonic_s': time.monotonic(),
            **fields,
        }
        try:
            callback(payload)
        except Exception:
            pass
