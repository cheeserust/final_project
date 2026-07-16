"""Contract tests for the Board1 V3 + Board2 legacy coordinator."""

import math
import struct
import threading
import time
from types import SimpleNamespace

from arm_can_bridge.arm_goal_v3 import (
    ArmGoalRuntimeState,
    ArmGoalV3AbortedByEstop,
    ArmGoalV3Canceled,
    ArmGoalV3Coordinator,
    ArmGoalV3Error,
    build_arm_goal_frames_v3,
)
from arm_can_bridge.can_protocol import (
    ArmGoalAck,
    ArmGoalAckResult,
    BoardError,
    BoardState,
    BoardStatus,
    decode_control_byte,
    pack_arm_goal_control_v3,
    pack_arm_goal_v3,
    pack_estop,
    pack_position_command,
    unpack_arm_goal_ack_v3,
    unpack_status,
)

import pytest


JOINTS = [
    'base_joint',
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
]


class FakeWriter:
    """Synchronous writer exposing the coordinator's serialized-writer API."""

    def __init__(self, *, goal_delay_s=0.0):
        self.calls = []
        self.on_batch = None
        self.on_emergency = None
        self.goal_delay_s = float(goal_delay_s)
        self.discarded_goal_ids = []
        self.waited_goal_ids = []

    def send_batch(
        self,
        frames,
        *,
        goal_id=None,
        timeout_s=2.0,
        category='control',
    ):
        del timeout_s
        batch = tuple(frames)
        started_at = time.monotonic()
        if category == 'goal' and self.goal_delay_s > 0.0:
            time.sleep(self.goal_delay_s)
        call = SimpleNamespace(
            frames=batch,
            goal_id=goal_id,
            category=category,
        )
        self.calls.append(call)
        if self.on_batch is not None:
            self.on_batch(batch, goal_id, category)
        return SimpleNamespace(
            started_at=started_at,
            completed_at=time.monotonic(),
        )

    def send_emergency(self, frame, *, timeout_s=2.0):
        del timeout_s
        now = time.monotonic()
        call = SimpleNamespace(
            frames=(frame,),
            goal_id=None,
            category='estop',
        )
        self.calls.append(call)
        if self.on_emergency is not None:
            self.on_emergency(frame)
        return SimpleNamespace(started_at=now, completed_at=time.monotonic())

    def discard_goal(self, goal_id):
        self.discarded_goal_ids.append(goal_id)
        return 0

    def wait_goal_idle(self, goal_id, timeout_s=2.0):
        del timeout_s
        self.waited_goal_ids.append(goal_id)
        return True

    def calls_for(self, *, can_id=None, category=None, command=None):
        result = []
        for call in self.calls:
            frame = call.frames[0]
            if can_id is not None and frame.can_id != can_id:
                continue
            if category is not None and call.category != category:
                continue
            if command is not None and frame.data[0] != command:
                continue
            result.append(call)
        return result


def ack(board_id, result, goal_id, mask, duration=1000):
    """Build one decoded V3 ACK snapshot."""
    return ArmGoalAck(
        board_id=board_id,
        protocol_version=3,
        result=result,
        goal_id=goal_id,
        received_axis_mask=mask,
        state_snapshot=1,
        duration_ms=duration,
    )


def status(
    board_id,
    *,
    state=BoardState.IDLE,
    slot=None,
    sequence=1,
    enabled=True,
    axis01=None,
    error_code=0,
    limit_status_bits=0,
):
    """Build one coherent Board1 V3 or Board2 legacy status snapshot."""
    if slot is None:
        slot = 1 if board_id == 1 else 32
    if axis01 is None and state == BoardState.MOVING:
        axis01 = 0x55 if board_id == 1 else 0x07
        axis23 = 0x55 if board_id == 1 else 0
    elif axis01 is None:
        axis01 = 0x99 if board_id == 1 else 0x0B
        axis23 = 0x99 if board_id == 1 else 0
    else:
        axis23 = 0x99 if board_id == 1 else 0
    return BoardStatus(
        board_id=board_id,
        state=state,
        error_code=error_code,
        homing_done_bits=axis01,
        moving_motor_id=axis23,
        limit_status_bits=limit_status_bits,
        queue_free=slot,
        enabled=enabled,
        reserved=sequence,
    )


def prime_capability(coordinator):
    """Supply both passive status snapshots required before execution."""
    coordinator.update_status(status(1))
    coordinator.update_status(status(2))
    assert coordinator.probe_capability()


def send_cancelled(coordinator, goal_id, duration=1000):
    """Acknowledge CANCEL from Board1; legacy Board2 has no ACK."""
    coordinator.update_ack(
        ack(1, ArmGoalAckResult.CANCELLED, goal_id, 0, duration)
    )


def send_ready(coordinator, goal_id, duration=1000, *, result=None, mask=0x0F):
    """Supply Board1 full-mask READY or DUPLICATE evidence."""
    coordinator.update_ack(
        ack(
            1,
            ArmGoalAckResult.READY if result is None else result,
            goal_id,
            mask,
            duration,
        )
    )


def send_started(coordinator, goal_id, duration=1000):
    """Supply Board1 STARTED; legacy Board2 has no ACK channel."""
    coordinator.update_ack(
        ack(1, ArmGoalAckResult.STARTED, goal_id, 0x0F, duration)
    )


def send_done_both(coordinator):
    """Supply post-START completion status from both boards."""
    coordinator.update_status(status(1))
    coordinator.update_status(status(2))


def assert_no_disable(writer):
    assert all(
        frame.can_id != 0x010 or frame.data[0] != 0
        for call in writer.calls
        for frame in call.frames
    )


def execute_in_thread(coordinator, *, duration_ms=1000, cancel_requested=None):
    """Start one execute call and capture either its result or exception."""
    outcome = {}

    def target():
        try:
            outcome['result'] = coordinator.execute(
                joint_names=JOINTS,
                positions_rad=[0.0] * 5,
                duration_ms=duration_ms,
                cancel_requested=cancel_requested,
            )
        except ArmGoalV3Error as exc:
            outcome['error'] = exc

    worker = threading.Thread(target=target)
    worker.start()
    return worker, outcome


def wait_until(predicate, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return bool(predicate())


def test_exact_board1_v3_board2_legacy_payload_and_name_based_mapping():
    goal = build_arm_goal_frames_v3(
        joint_names=JOINTS,
        positions_rad=[
            math.radians(90.0),
            math.radians(30.0),
            math.radians(-15.5),
            0.0,
            math.radians(10.0),
        ],
        duration_ms=1000,
        goal_id=0x2A,
    )
    assert [frame.can_id for frame in goal.board1] == [0x101] * 4
    assert [frame.data.hex(' ').upper() for frame in goal.board1] == [
        '90 B8 0B 00 00 2A E8 03',
        '91 F2 F9 FF FF 2A E8 03',
        '92 00 00 00 00 2A E8 03',
        '93 28 23 00 00 2A E8 03',
    ]
    assert len(goal.board2) == 1
    assert goal.board2[0].can_id == 0x102
    assert goal.board2[0].data.hex(' ').upper() == (
        '80 E8 03 00 00 00 00 C8'
    )


@pytest.mark.parametrize('duration_ms', [1, 5000, 65535])
def test_duration_is_uint16_little_endian(duration_ms):
    frame = pack_arm_goal_v3(
        board_id=1,
        motor_id=0,
        target_pos=-1550,
        goal_id=7,
        duration_ms=duration_ms,
    )
    assert frame.data[5] == 7
    assert struct.unpack_from('<H', frame.data, 6)[0] == duration_ms
    control = decode_control_byte(frame.data[0])
    assert control.execute
    assert control.reserved
    assert not control.relative
    assert not control.step_mode
    assert struct.unpack_from('<i', frame.data, 1)[0] == -1550


@pytest.mark.parametrize('duration_ms', [0, 65536, -1])
def test_invalid_duration_is_rejected_not_clamped(duration_ms):
    with pytest.raises(ValueError):
        pack_arm_goal_v3(
            board_id=1,
            motor_id=0,
            target_pos=0,
            goal_id=1,
            duration_ms=duration_ms,
        )


@pytest.mark.parametrize(
    ('duration_ms', 'duration_ticks'),
    [(1, 1), (5, 1), (6, 2), (1275, 255), (5000, 255), (65535, 255)],
)
def test_board2_legacy_duration_is_five_ms_ticks_capped_at_255(
    duration_ms,
    duration_ticks,
):
    goal = build_arm_goal_frames_v3(
        joint_names=list(reversed(JOINTS)),
        positions_rad=[0.0] * 5,
        duration_ms=duration_ms,
        goal_id=255,
    )
    for frame in goal.board1:
        assert frame.data[5] == 255
        assert struct.unpack_from('<H', frame.data, 6)[0] == duration_ms

    legacy = goal.board2[0]
    assert legacy.data[0] == 0x80
    assert legacy.data[5:7] == b'\x00\x00'
    assert legacy.data[7] == duration_ticks


def test_start_and_cancel_control_frames_are_zero_padded():
    assert pack_arm_goal_control_v3(1, 0x2A).data == bytes(
        [1, 0x2A, 0, 0, 0, 0, 0, 0]
    )
    assert pack_arm_goal_control_v3(2, 0x2B).data == bytes(
        [2, 0x2B, 0, 0, 0, 0, 0, 0]
    )


@pytest.mark.parametrize('result', list(ArmGoalAckResult))
def test_ack_results_zero_through_seven(result):
    decoded = unpack_arm_goal_ack_v3(
        bytes([3, int(result), 9, 0x0F, 1, 0, 0x88, 0x13]),
        board_id=1,
    )
    assert decoded.result == result
    assert decoded.goal_id == 9
    assert decoded.received_axis_mask == 0x0F
    assert decoded.duration_ms == 5000


def test_board1_status_goal_slot_and_axis_masks_are_atomic():
    decoded = unpack_status(
        bytes([1, 0, 0x88, 0x88, 0, 1, 1, 44]),
        board_id=1,
    )
    assert decoded.status_sequence == 44
    assert decoded.goal_slot_free == 1
    assert decoded.moving_mask == 0
    assert decoded.target_reached_mask == 0x0F


def test_board2_status_accepts_legacy_queue_capacity_32():
    decoded = unpack_status(
        bytes([1, 0, 0x09, 0, 0, 32, 1, 44]),
        board_id=2,
        board2_legacy=True,
    )
    assert decoded.queue_free == 32
    assert decoded.target_reached_mask == 0x01


def test_board2_legacy_status_rejects_queue_credit_over_capacity():
    with pytest.raises(ValueError, match='0..32'):
        unpack_status(
            bytes([1, 0, 0x09, 0, 0, 33, 1, 44]),
            board_id=2,
            board2_legacy=True,
        )


@pytest.mark.parametrize('legacy_value', [32, 124])
def test_board1_legacy_queue_free_is_protocol_mismatch(legacy_value):
    with pytest.raises(ValueError, match='protocol mismatch'):
        unpack_status(
            bytes([1, 0, 0x88, 0x88, 0, legacy_value, 1, 1]),
            board_id=1,
        )


def test_error_with_full_ready_mask_is_rejected_as_contradictory():
    with pytest.raises(ValueError, match='contradictory'):
        unpack_status(
            bytes([BoardState.ERROR, 1, 0x22, 0x22, 0, 0, 1, 8]),
            board_id=1,
        )


def test_board3_legacy_packer_snapshot_is_unchanged():
    frame = pack_position_command(
        motor_id=8,
        target_pos=-1234,
        speed=500,
        duration_ticks=40,
        board_id=3,
    )
    assert frame.can_id == 0x103
    assert frame.data == struct.pack('<BiHB', 0x88, -1234, 500, 40)


@pytest.mark.parametrize(
    'board1_result',
    [ArmGoalAckResult.READY, ArmGoalAckResult.DUPLICATE],
)
def test_board1_full_mask_ready_or_duplicate_unlocks_legacy_board2(
    board1_result,
):
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(writer, ack_timeout_s=0.01)
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id, result=board1_result)
        elif category == 'start':
            send_started(coordinator, goal_id)
            send_done_both(coordinator)

    writer.on_batch = firmware
    completed = coordinator.execute(
        joint_names=JOINTS,
        positions_rad=[0.0] * 5,
        duration_ms=1000,
    )

    assert len(writer.calls_for(category='start', command=1)) == 1
    assert len(writer.calls_for(can_id=0x102, category='goal')) == 1
    assert completed.goal_id == 0


def test_valid_board1_limit_error_still_aborts_and_cancels():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(writer, ack_timeout_s=0.01)
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id)
        elif category == 'start':
            send_started(coordinator, goal_id)
            coordinator.update_status(status(
                1,
                state=BoardState.ERROR,
                slot=0,
                error_code=BoardError.LIMIT_DETECTED,
                limit_status_bits=0x02,
            ))
        elif category == 'cancel':
            send_cancelled(coordinator, goal_id)

    writer.on_batch = firmware
    with pytest.raises(
        ArmGoalV3Error,
        match='state=4, error=2.*limit_bits=0x02',
    ):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert len(writer.calls_for(category='cancel', command=2)) == 1
    assert_no_disable(writer)


@pytest.mark.parametrize('partial_mask', [0x00, 0x07])
def test_board1_partial_duplicate_is_not_ready(partial_mask):
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.005,
        communication_timeout_s=0.05,
        max_stage_attempts=1,
    )
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(
                coordinator,
                goal_id,
                result=ArmGoalAckResult.DUPLICATE,
                mask=partial_mask,
            )
        elif category == 'cancel':
            send_cancelled(coordinator, goal_id)

    writer.on_batch = firmware
    with pytest.raises(ArmGoalV3Error, match='READY timeout'):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert writer.calls_for(category='start') == []
    assert writer.calls_for(can_id=0x102, category='goal') == []
    assert len(writer.calls_for(category='cancel', command=2)) == 1
    assert_no_disable(writer)


def test_board2_ack_cannot_replace_board1_ready_evidence():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.001,
        communication_timeout_s=0.05,
        max_stage_attempts=1,
    )
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            coordinator.update_ack(
                ack(2, ArmGoalAckResult.READY, goal_id, 0x01)
            )
        elif category == 'cancel':
            send_cancelled(coordinator, goal_id)

    writer.on_batch = firmware
    with pytest.raises(ArmGoalV3Error, match='READY timeout'):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert len(writer.calls_for(can_id=0x101, category='goal')) == 1
    assert writer.calls_for(can_id=0x102, category='goal') == []
    assert_no_disable(writer)


def test_stale_goal_and_wrong_duration_acks_cannot_unlock_start():
    writer = FakeWriter()
    events = []
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.02,
        event_callback=events.append,
    )
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, (goal_id + 1) & 0xFF)
            send_ready(coordinator, goal_id, duration=999)
            assert writer.calls_for(category='start') == []
            send_ready(coordinator, goal_id)
        elif category == 'start':
            send_started(coordinator, goal_id)
            send_done_both(coordinator)

    writer.on_batch = firmware
    coordinator.execute(
        joint_names=JOINTS,
        positions_rad=[0.0] * 5,
        duration_ms=1000,
    )

    stale_reasons = {
        event.get('stale_reason')
        for event in events
        if event.get('event') == 'ack_received' and event.get('stale')
    }
    assert stale_reasons == {'goal_id_mismatch', 'duration_mismatch'}
    assert len(writer.calls_for(category='start', command=1)) == 1


def test_busy_is_terminal_without_retry_cancel_or_disable():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.01,
        communication_timeout_s=0.05,
    )
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            coordinator.update_ack(
                ack(1, ArmGoalAckResult.BUSY, goal_id, 0)
            )

    writer.on_batch = firmware
    with pytest.raises(ArmGoalV3Error, match='Board1 BUSY'):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert len(writer.calls_for(can_id=0x101, category='goal')) == 1
    assert writer.calls_for(can_id=0x102, category='goal') == []
    assert writer.calls_for(category='cancel') == []
    assert_no_disable(writer)


@pytest.mark.parametrize(
    'result',
    [ArmGoalAckResult.INVALID, ArmGoalAckResult.CONFLICT],
)
def test_invalid_or_conflict_fails_immediately_then_cancels_board1(result):
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.01,
        communication_timeout_s=0.05,
    )
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            coordinator.update_ack(ack(1, result, goal_id, 0))
        elif category == 'cancel':
            send_cancelled(coordinator, goal_id)

    writer.on_batch = firmware
    with pytest.raises(ArmGoalV3Error, match=result.name):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert len(writer.calls_for(can_id=0x101, category='goal')) == 1
    assert len(writer.calls_for(category='cancel', command=2)) == 1
    assert_no_disable(writer)


def test_staging_timeout_logs_missing_axes_and_retries_full_board1_batch():
    writer = FakeWriter()
    events = []
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.02,
        event_callback=events.append,
    )
    prime_capability(coordinator)
    board1_attempts = 0

    def firmware(batch, goal_id, category):
        nonlocal board1_attempts
        if category == 'goal' and batch[0].can_id == 0x101:
            board1_attempts += 1
            if board1_attempts == 1:
                coordinator.update_ack(
                    ack(1, ArmGoalAckResult.STAGING_TIMEOUT, goal_id, 0x05)
                )
            else:
                send_ready(coordinator, goal_id)
        elif category == 'start':
            send_started(coordinator, goal_id)
            send_done_both(coordinator)

    writer.on_batch = firmware
    coordinator.execute(
        joint_names=JOINTS,
        positions_rad=[0.0] * 5,
        duration_ms=1000,
    )

    board1_calls = writer.calls_for(can_id=0x101, category='goal')
    assert len(board1_calls) == 2
    assert all(
        [frame.data[0] & 0x0F for frame in call.frames] == [0, 1, 2, 3]
        for call in board1_calls
    )
    assert len(writer.calls_for(can_id=0x102, category='goal')) == 1
    timeout_event = next(
        event for event in events if event.get('event') == 'staging_timeout'
    )
    attempt_event = next(
        event for event in events if event.get('event') == 'staging_attempt'
    )
    assert attempt_event['max_attempts'] == 200
    assert timeout_event['received_mask'] == '0x05'
    assert timeout_event['missing_mask'] == '0x0A'
    assert timeout_event['missing_motor_ids'] == [1, 3]


def test_default_ready_timeout_retries_board1_exactly_200_times():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.0001,
        communication_timeout_s=0.05,
    )
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'cancel':
            send_cancelled(coordinator, goal_id)

    writer.on_batch = firmware
    with pytest.raises(ArmGoalV3Error, match='after 200 staging attempts'):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert len(writer.calls_for(can_id=0x101, category='goal')) == 200
    assert writer.calls_for(can_id=0x102, category='goal') == []
    assert len(writer.calls_for(category='cancel', command=2)) == 1
    assert_no_disable(writer)


def test_ready_timeout_starts_after_each_batch_send_completes():
    writer = FakeWriter(goal_delay_s=0.02)
    coordinator = ArmGoalV3Coordinator(writer, ack_timeout_s=0.005)
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id)
        elif category == 'start':
            send_started(coordinator, goal_id)
            send_done_both(coordinator)

    writer.on_batch = firmware
    coordinator.execute(
        joint_names=JOINTS,
        positions_rad=[0.0] * 5,
        duration_ms=1000,
    )

    assert len(writer.calls_for(can_id=0x101, category='goal')) == 1
    assert len(writer.calls_for(can_id=0x102, category='goal')) == 1
    assert len(writer.calls_for(category='start', command=1)) == 1


def test_legacy_board2_is_sent_once_after_board1_ready_before_start():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(writer, ack_timeout_s=0.02)
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            assert writer.calls_for(can_id=0x102, category='goal') == []
            send_ready(coordinator, goal_id)
        elif category == 'goal' and batch[0].can_id == 0x102:
            assert writer.calls_for(category='start') == []
        elif category == 'start':
            send_started(coordinator, goal_id)
            send_done_both(coordinator)

    writer.on_batch = firmware
    completed = coordinator.execute(
        joint_names=JOINTS,
        positions_rad=[0.0] * 5,
        duration_ms=1000,
    )

    starts = writer.calls_for(category='start', command=1)
    assert len(starts) == 1
    assert starts[0].frames[0].data[1] == completed.goal_id
    order = [call.category for call in writer.calls]
    assert order == ['goal', 'goal', 'start']


def test_completion_waits_for_board1_started_even_with_both_done_statuses():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.02,
        communication_timeout_s=0.2,
    )
    prime_capability(coordinator)
    start_seen = threading.Event()
    goal_id_holder = []

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id)
        elif category == 'start':
            goal_id_holder.append(goal_id)
            send_done_both(coordinator)
            start_seen.set()

    writer.on_batch = firmware
    worker, outcome = execute_in_thread(coordinator)
    assert start_seen.wait(timeout=1.0)
    time.sleep(0.01)
    assert worker.is_alive()

    send_started(coordinator, goal_id_holder[0])
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert 'error' not in outcome


def test_fresh_moving_heartbeats_outlive_requested_duration_without_cancel():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.02,
        communication_timeout_s=0.04,
    )
    prime_capability(coordinator)
    moving_started = threading.Event()

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id, duration=1)
        elif category == 'start':
            send_started(coordinator, goal_id, duration=1)
            coordinator.update_status(
                status(1, state=BoardState.MOVING, slot=0)
            )
            coordinator.update_status(
                status(2, state=BoardState.MOVING, slot=31)
            )
            moving_started.set()

    writer.on_batch = firmware
    worker, outcome = execute_in_thread(coordinator, duration_ms=1)
    assert moving_started.wait(timeout=1.0)

    for sequence in range(2, 9):
        time.sleep(0.01)
        coordinator.update_status(
            status(1, state=BoardState.MOVING, slot=0, sequence=sequence)
        )
        coordinator.update_status(
            status(2, state=BoardState.MOVING, slot=31, sequence=sequence)
        )

    assert worker.is_alive()
    assert writer.calls_for(category='cancel') == []
    send_done_both(coordinator)
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert 'error' not in outcome
    assert_no_disable(writer)


def test_disabled_heartbeat_fails_active_goal_instead_of_waiting_forever():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.02,
        communication_timeout_s=0.05,
    )
    prime_capability(coordinator)

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id)
        elif category == 'start':
            send_started(coordinator, goal_id)
            coordinator.update_status(
                status(1, state=BoardState.DISABLED, enabled=False)
            )
            coordinator.update_status(
                status(2, state=BoardState.DISABLED, enabled=False)
            )
        elif category == 'cancel':
            send_cancelled(coordinator, goal_id)

    writer.on_batch = firmware
    with pytest.raises(ArmGoalV3Error, match='enabled=False'):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert len(writer.calls_for(category='cancel', command=2)) == 1
    assert_no_disable(writer)


def test_cancel_after_legacy_send_waits_for_fresh_board2_safe_status():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.02,
        communication_timeout_s=0.2,
    )
    prime_capability(coordinator)
    cancel_requested = threading.Event()
    movement_started = threading.Event()
    cancel_seen = threading.Event()

    def firmware(batch, goal_id, category):
        if category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id)
        elif category == 'start':
            send_started(coordinator, goal_id)
            coordinator.update_status(
                status(1, state=BoardState.MOVING, slot=0)
            )
            # Safe-looking Board2 evidence received before CANCEL is stale for
            # cancellation and must not be reused as post-CANCEL evidence.
            coordinator.update_status(status(2, slot=32, sequence=1))
            movement_started.set()
        elif category == 'cancel':
            send_cancelled(coordinator, goal_id)
            cancel_seen.set()

    writer.on_batch = firmware
    worker, outcome = execute_in_thread(
        coordinator,
        cancel_requested=cancel_requested.is_set,
    )
    assert movement_started.wait(timeout=1.0)
    assert len(writer.calls_for(can_id=0x102, category='goal')) == 1

    cancel_requested.set()
    assert cancel_seen.wait(timeout=1.0)
    time.sleep(0.01)
    assert worker.is_alive(), outcome

    coordinator.update_status(status(2, slot=32, sequence=2))
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert isinstance(outcome.get('error'), ArmGoalV3Canceled)
    assert_no_disable(writer)


def test_cancel_waits_for_board1_then_next_goal_uses_new_id():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.02,
        communication_timeout_s=0.1,
    )
    prime_capability(coordinator)
    cancel_id = []

    def firmware(batch, goal_id, category):
        if category == 'cancel':
            cancel_id.append(goal_id)
            send_cancelled(coordinator, goal_id)
        elif category == 'goal' and batch[0].can_id == 0x101:
            send_ready(coordinator, goal_id)
        elif category == 'start':
            send_started(coordinator, goal_id)
            send_done_both(coordinator)

    writer.on_batch = firmware
    with pytest.raises(ArmGoalV3Canceled):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
            cancel_requested=lambda: True,
        )

    completed = coordinator.execute(
        joint_names=JOINTS,
        positions_rad=[0.0] * 5,
        duration_ms=1000,
    )

    assert cancel_id == [0]
    assert completed.goal_id == 1
    assert writer.discarded_goal_ids == [0]
    assert writer.waited_goal_ids == [0]
    assert_no_disable(writer)


def test_estop_aborts_active_goal_without_cancel_or_disable():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.2,
        communication_timeout_s=0.2,
    )
    prime_capability(coordinator)
    staging_started = threading.Event()

    def firmware(batch, goal_id, category):
        del goal_id
        if category == 'goal':
            staging_started.set()

    writer.on_batch = firmware
    worker, outcome = execute_in_thread(coordinator)
    assert staging_started.wait(timeout=1.0)
    assert wait_until(lambda: coordinator.active_goal_id is not None)

    coordinator.abort_by_estop(pack_estop())
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert isinstance(outcome.get('error'), ArmGoalV3AbortedByEstop)
    assert coordinator.state == ArmGoalRuntimeState.ABORTED_BY_ESTOP
    estops = writer.calls_for(can_id=0x001, category='estop')
    assert len(estops) == 1
    assert estops[0].frames[0].data == bytes.fromhex('0100000000000000')
    assert writer.calls_for(category='cancel') == []
    assert_no_disable(writer)

    call_count = len(writer.calls)
    coordinator.clear_estop_latch()
    assert coordinator.state == ArmGoalRuntimeState.IDLE
    assert coordinator.active_goal_id is None
    assert len(writer.calls) == call_count


@pytest.mark.parametrize(
    ('condition', 'board2_status'),
    [
        ('busy', status(2, state=BoardState.IDLE, axis01=0x07)),
        (
            'not_idle',
            status(2, state=BoardState.MOVING, axis01=0x0B),
        ),
        ('not_enabled', status(2, enabled=False)),
        ('not_homed', status(2, axis01=0x08)),
        ('queue_not_empty', status(2, slot=31)),
    ],
)
def test_execute_rejects_board2_that_is_not_safely_idle(
    condition,
    board2_status,
):
    writer = FakeWriter()
    events = []
    coordinator = ArmGoalV3Coordinator(writer, event_callback=events.append)
    coordinator.update_status(status(1))
    coordinator.update_status(board2_status)
    coordinator.probe_capability()

    with pytest.raises(ArmGoalV3Error, match='Board2'):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert writer.calls == [], condition
    assert_no_disable(writer)


def test_execute_rechecks_board2_status_freshness_before_transmitting():
    writer = FakeWriter()
    coordinator = ArmGoalV3Coordinator(
        writer,
        communication_timeout_s=0.01,
    )
    prime_capability(coordinator)
    time.sleep(0.02)
    coordinator.update_status(status(1, sequence=2))

    with pytest.raises(ArmGoalV3Error, match='Board2'):
        coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )

    assert writer.calls == []
    assert_no_disable(writer)


def test_capability_probe_is_passive_and_never_cancels_an_unknown_goal():
    writer = FakeWriter()
    events = []
    coordinator = ArmGoalV3Coordinator(writer, event_callback=events.append)

    assert not coordinator.probe_capability()
    assert writer.calls == []

    coordinator.update_status(status(1))
    coordinator.update_status(status(2))
    assert coordinator.probe_capability()

    assert writer.calls == []
    probes = [
        event for event in events if event.get('event') == 'capability_probe'
    ]
    assert [event['destructive'] for event in probes] == [False, False]
