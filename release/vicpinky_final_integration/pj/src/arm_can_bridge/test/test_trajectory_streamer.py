"""Unit tests for trajectory streaming completion timing."""

from arm_can_bridge import trajectory_streamer as streamer_module
from arm_can_bridge.can_protocol import CAN_ID_BOARD1_POSITION_COMMAND
from arm_can_bridge.can_protocol import CAN_ID_BOARD2_POSITION_COMMAND
from arm_can_bridge.can_protocol import CAN_ID_BOARD3_SERVO_COMMAND
from arm_can_bridge.can_protocol import CanFrame
from arm_can_bridge.trajectory_converter import TrajectoryBatch
from arm_can_bridge.trajectory_streamer import TrajectoryStreamer


class _AlwaysCompleteBoardState:

    def __init__(self) -> None:
        self.completion_check_times: list[float] = []

    def is_trajectory_complete(self) -> bool:
        self.completion_check_times.append(streamer_module.time.monotonic())
        return True

    def has_error(self) -> bool:
        return False


class _UnusedTransport:

    pass


class _ReadyBoardState(_AlwaysCompleteBoardState):

    def can_accept_new_trajectory(self) -> bool:
        return True

    def can_stream_slots(
        self,
        required_slots_by_board,
        *,
        max_in_flight_slots_by_board=None,
    ) -> bool:
        del required_slots_by_board
        del max_in_flight_slots_by_board
        return True

    def reserve_queue_slots(
        self,
        required_slots_by_board,
        *,
        max_in_flight_slots_by_board=None,
    ) -> bool:
        del required_slots_by_board
        del max_in_flight_slots_by_board
        return True

    def refund_queue_slots(self, slots_by_board) -> None:
        del slots_by_board


class _RecordingTransport:

    def __init__(self) -> None:
        self.frames: list[CanFrame] = []

    def send_frame(self, frame: CanFrame) -> None:
        self.frames.append(frame)


def test_completion_wait_ignores_complete_status_until_min_wait(monkeypatch):
    board_state = _AlwaysCompleteBoardState()
    now = {'value': 0.0}

    monkeypatch.setattr(streamer_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        streamer_module.time,
        'monotonic',
        lambda: now['value'],
    )

    def sleep(duration_s):
        now['value'] += duration_s

    monkeypatch.setattr(streamer_module.time, 'sleep', sleep)

    streamer = TrajectoryStreamer(
        board_state=board_state,
        transport=_UnusedTransport(),
        queue_wait_timeout_ms=100,
        completion_grace_ms=100,
    )

    streamer._wait_for_completion(
        timeout_s=1.0,
        min_wait_s=0.06,
        cancel_requested=None,
    )

    assert board_state.completion_check_times
    assert board_state.completion_check_times[0] >= 0.06


def test_board3_inter_frame_delay_is_inserted(monkeypatch):
    board_state = _ReadyBoardState()
    transport = _RecordingTransport()
    now = {'value': 0.0}
    sleeps = []

    monkeypatch.setattr(streamer_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        streamer_module.time,
        'monotonic',
        lambda: now['value'],
    )

    def sleep(duration_s):
        sleeps.append(duration_s)
        now['value'] += duration_s

    monkeypatch.setattr(streamer_module.time, 'sleep', sleep)

    streamer = TrajectoryStreamer(
        board_state=board_state,
        transport=transport,
        queue_wait_timeout_ms=100,
        completion_grace_ms=100,
        board3_inter_frame_delay_ms=3.0,
    )
    frames = (
        CanFrame(CAN_ID_BOARD3_SERVO_COMMAND, b'\x80' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD3_SERVO_COMMAND, b'\x81' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD3_SERVO_COMMAND, b'\x82' + b'\x00' * 7),
    )
    batch = TrajectoryBatch(
        source_point_index=0,
        duration_ticks=1,
        target_positions_rad=(0.0, 0.0, 0.0),
        frames=frames,
    )

    streamer.stream((batch,))

    assert transport.frames == list(frames)
    assert sleeps[:2] == [0.003, 0.003]


def test_board1_board2_frames_do_not_use_board3_delay(monkeypatch):
    board_state = _ReadyBoardState()
    transport = _RecordingTransport()
    now = {'value': 0.0}
    sleeps = []

    monkeypatch.setattr(streamer_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        streamer_module.time,
        'monotonic',
        lambda: now['value'],
    )

    def sleep(duration_s):
        sleeps.append(duration_s)
        now['value'] += duration_s

    monkeypatch.setattr(streamer_module.time, 'sleep', sleep)

    streamer = TrajectoryStreamer(
        board_state=board_state,
        transport=transport,
        queue_wait_timeout_ms=100,
        completion_grace_ms=100,
        board3_inter_frame_delay_ms=3.0,
    )
    frames = (
        CanFrame(CAN_ID_BOARD2_POSITION_COMMAND, b'\x80' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x80' + b'\x00' * 7),
    )
    batch = TrajectoryBatch(
        source_point_index=0,
        duration_ticks=0,
        target_positions_rad=(0.0, 0.0),
        frames=frames,
    )

    streamer.stream((batch,))

    assert transport.frames == list(frames)
    assert sleeps == []


def test_arm_inter_frame_delay_is_inserted(monkeypatch):
    board_state = _ReadyBoardState()
    transport = _RecordingTransport()
    now = {'value': 0.0}
    sleeps = []

    monkeypatch.setattr(streamer_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        streamer_module.time,
        'monotonic',
        lambda: now['value'],
    )

    def sleep(duration_s):
        sleeps.append(duration_s)
        now['value'] += duration_s

    monkeypatch.setattr(streamer_module.time, 'sleep', sleep)

    streamer = TrajectoryStreamer(
        board_state=board_state,
        transport=transport,
        queue_wait_timeout_ms=100,
        completion_grace_ms=100,
        arm_inter_frame_delay_ms=7.0,
    )
    frames = (
        CanFrame(CAN_ID_BOARD2_POSITION_COMMAND, b'\x80' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x80' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x81' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x82' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x83' + b'\x00' * 7),
    )
    batch = TrajectoryBatch(
        source_point_index=0,
        duration_ticks=1,
        target_positions_rad=(0.0, 0.0, 0.0, 0.0, 0.0),
        frames=frames,
    )

    streamer.stream((batch,))

    assert transport.frames == list(frames)
    assert sleeps[:4] == [0.007, 0.007, 0.007, 0.007]


def test_arm_inter_batch_delay_is_inserted(monkeypatch):
    board_state = _ReadyBoardState()
    transport = _RecordingTransport()
    now = {'value': 0.0}
    sleeps = []

    monkeypatch.setattr(streamer_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        streamer_module.time,
        'monotonic',
        lambda: now['value'],
    )

    def sleep(duration_s):
        sleeps.append(duration_s)
        now['value'] += duration_s

    monkeypatch.setattr(streamer_module.time, 'sleep', sleep)

    streamer = TrajectoryStreamer(
        board_state=board_state,
        transport=transport,
        queue_wait_timeout_ms=100,
        completion_grace_ms=100,
        arm_inter_frame_delay_ms=7.0,
    )
    first_frames = (
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x80' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x81' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x82' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x83' + b'\x00' * 7),
    )
    second_frames = (
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x80' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x81' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x82' + b'\x00' * 7),
        CanFrame(CAN_ID_BOARD1_POSITION_COMMAND, b'\x83' + b'\x00' * 7),
    )
    batches = (
        TrajectoryBatch(
            source_point_index=0,
            duration_ticks=4,
            target_positions_rad=(0.0, 0.0, 0.0, 0.0),
            frames=first_frames,
        ),
        TrajectoryBatch(
            source_point_index=0,
            duration_ticks=4,
            target_positions_rad=(0.0, 0.0, 0.0, 0.0),
            frames=second_frames,
        ),
    )

    streamer.stream(batches)

    assert transport.frames == list(first_frames + second_frames)
    assert sleeps[:7] == [0.007] * 7
