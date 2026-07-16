"""Concurrency and failure-contract tests for the single CAN writer."""

import errno
import threading
import time

from arm_can_bridge.can_protocol import CanFrame, pack_estop
from arm_can_bridge.can_writer import SerializedCanWriter
from arm_can_bridge.socketcan_transport import SocketCanTransportError
import pytest


class RecordingTransport:
    """Record frames while optionally blocking or injecting send errors."""

    def __init__(self):
        self.frames = []
        self.lock = threading.Lock()
        self.block_first = False
        self.first_started = threading.Event()
        self.release_first = threading.Event()
        self.failures = []

    def send_frame(self, frame):
        if self.failures:
            error = self.failures.pop(0)
            if error is not None:
                raise SocketCanTransportError('injected send failure') from error
        if self.block_first and not self.first_started.is_set():
            self.first_started.set()
            assert self.release_first.wait(1.0)
        with self.lock:
            self.frames.append(frame)
        time.sleep(0.001)


def frame(can_id, value):
    return CanFrame(can_id, bytes([value]))


def test_concurrent_batches_never_interleave_frames():
    transport = RecordingTransport()
    writer = SerializedCanWriter(transport, batch_inter_frame_delay_s=0.0)
    barrier = threading.Barrier(4)

    def send(value):
        barrier.wait()
        writer.send_batch(
            (frame(0x101, value), frame(0x101, value)),
            goal_id=value,
            category='goal',
        )

    workers = [threading.Thread(target=send, args=(value,)) for value in (1, 2, 3)]
    for worker in workers:
        worker.start()
    barrier.wait()
    for worker in workers:
        worker.join(timeout=1.0)
    writer.close()

    values = [item.data[0] for item in transport.frames]
    assert sorted(values) == [1, 1, 2, 2, 3, 3]
    assert all(values[index] == values[index + 1] for index in (0, 2, 4))


def test_enobufs_is_logged_and_retried_until_full_send_succeeds():
    transport = RecordingTransport()
    transport.failures = [OSError(errno.ENOBUFS, 'full'), None]
    events = []
    writer = SerializedCanWriter(
        transport,
        retry_count=1,
        retry_delay_s=0.0,
        event_callback=events.append,
    )

    writer.send_batch((frame(0x101, 7),), goal_id=7, category='goal')
    writer.close()

    assert [item.data[0] for item in transport.frames] == [7]
    failure = next(event for event in events if event['event'] == 'frame_send_error')
    assert failure['errno'] == errno.ENOBUFS
    assert failure['enobufs'] is True
    sent = next(event for event in events if event['event'] == 'frame_sent')
    assert sent['attempt'] == 2


def test_non_retryable_short_write_error_is_not_reported_as_success():
    transport = RecordingTransport()
    transport.failures = [RuntimeError('short write')]
    writer = SerializedCanWriter(transport, retry_count=3)

    with pytest.raises(SocketCanTransportError, match='injected'):
        writer.send_batch((frame(0x101, 9),), category='goal')
    writer.close()

    assert transport.frames == []


def test_estop_discards_queued_motion_and_runs_before_later_control():
    transport = RecordingTransport()
    transport.block_first = True
    writer = SerializedCanWriter(transport, batch_inter_frame_delay_s=0.0)
    outcomes = {}

    def active_goal():
        writer.send_batch(
            (frame(0x101, 1), frame(0x101, 2)),
            goal_id=1,
            category='goal',
        )

    def queued_goal():
        try:
            writer.send_batch(
                (frame(0x102, 3),),
                goal_id=2,
                category='goal',
            )
        except Exception as exc:
            outcomes['queued_error'] = exc

    active = threading.Thread(target=active_goal)
    active.start()
    assert transport.first_started.wait(1.0)
    queued = threading.Thread(target=queued_goal)
    queued.start()
    deadline = time.monotonic() + 1.0
    while writer.wait_goal_idle(2, timeout_s=0.001) and time.monotonic() < deadline:
        time.sleep(0.001)

    estop_done = threading.Event()

    def emergency():
        writer.send_emergency(pack_estop())
        estop_done.set()

    estop = threading.Thread(target=emergency)
    estop.start()
    time.sleep(0.01)
    transport.release_first.set()
    for worker in (active, queued, estop):
        worker.join(timeout=1.0)
    writer.close()

    assert estop_done.is_set()
    assert isinstance(outcomes['queued_error'], RuntimeError)
    assert [item.can_id for item in transport.frames] == [0x101, 0x101, 0x001]
    assert all(item.data != b'\x03' for item in transport.frames)


def test_writer_timeout_removes_a_still_queued_motion_request():
    transport = RecordingTransport()
    transport.block_first = True
    writer = SerializedCanWriter(transport, batch_inter_frame_delay_s=0.0)

    first = threading.Thread(
        target=lambda: writer.send_batch(
            (frame(0x101, 1),), goal_id=1, category='goal'
        )
    )
    first.start()
    assert transport.first_started.wait(1.0)

    with pytest.raises(SocketCanTransportError, match='queued request removed'):
        writer.send_batch(
            (frame(0x102, 2),),
            goal_id=2,
            category='goal',
            timeout_s=0.01,
        )

    transport.release_first.set()
    first.join(timeout=1.0)
    writer.close()
    assert [item.data[0] for item in transport.frames] == [1]
