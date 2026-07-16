"""Real SocketCAN/vcan scenarios for Board1 V3 + Board2 legacy mode."""

from pathlib import Path
import struct
import threading
import time

from arm_can_bridge.arm_goal_v3 import (
    ArmGoalV3Canceled,
    ArmGoalV3Coordinator,
)
from arm_can_bridge.can_protocol import (
    ACK_CAN_IDS,
    ArmGoalAckResult,
    CAN_ID_ARM_GOAL_CONTROL_V3,
    CAN_ID_BOARD1_ACK_V3,
    CAN_ID_BOARD1_POSITION_COMMAND,
    CAN_ID_BOARD1_STATUS,
    CAN_ID_BOARD2_POSITION_COMMAND,
    CAN_ID_BOARD2_STATUS,
    CanFrame,
    unpack_arm_goal_ack_v3,
    unpack_status,
)
from arm_can_bridge.can_writer import SerializedCanWriter
from arm_can_bridge.socketcan_transport import SocketCanTransport
import pytest


JOINTS = [
    'base_joint',
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
]
VCAN_INTERFACE = 'vcan0'


pytestmark = pytest.mark.skipif(
    not (Path('/sys/class/net') / VCAN_INTERFACE).exists(),
    reason='vcan0 is not configured',
)


def board1_ack_frame(result, goal_id, mask, duration_ms):
    return CanFrame(
        CAN_ID_BOARD1_ACK_V3,
        bytes((3, int(result), goal_id, mask, 1, 0))
        + struct.pack('<H', duration_ms),
    )


def status_frame(board_id, sequence):
    return CanFrame(
        CAN_ID_BOARD1_STATUS if board_id == 1 else CAN_ID_BOARD2_STATUS,
        bytes((
            1,
            0,
            0x99 if board_id == 1 else 0x09,
            0x99 if board_id == 1 else 0,
            0,
            1 if board_id == 1 else 32,
            1,
            sequence,
        )),
    )


class VcanFirmware:
    """Wire-level Board1 V3 and Board2 legacy integration-test responder."""

    def __init__(self, *, lose_first_board1_ready=False, cancel_race=False):
        self.transport = SocketCanTransport(
            VCAN_INTERFACE,
            receive_ids=(
                CAN_ID_BOARD1_POSITION_COMMAND,
                CAN_ID_BOARD2_POSITION_COMMAND,
                CAN_ID_ARM_GOAL_CONTROL_V3,
            ),
            frame_callback=self.on_frame,
        )
        self.lose_first_board1_ready = lose_first_board1_ready
        self.cancel_race = cancel_race
        self.board1_masks = {}
        self.board1_full_attempts = 0
        self.goal_id = None
        self.duration_ms = None
        self.board1_ready_sent = threading.Event()
        self.board2_legacy_frames = []
        self.board2_sent_after_ready = False
        self.start_seen = threading.Event()
        self.cancel_seen = threading.Event()

    def start(self):
        self.transport.open()

    def close(self):
        self.transport.close()

    def send_initial_status(self):
        self.transport.send_frame(status_frame(1, 1))
        self.transport.send_frame(status_frame(2, 1))

    def on_frame(self, frame):
        if frame.can_id == CAN_ID_BOARD1_POSITION_COMMAND:
            goal_id = frame.data[5]
            duration_ms = struct.unpack_from('<H', frame.data, 6)[0]
            self.goal_id = goal_id
            self.duration_ms = duration_ms
            mask = self.board1_masks.get(goal_id, 0)
            mask |= 1 << (frame.data[0] & 0x0F)
            self.board1_masks[goal_id] = mask
            if mask == 0x0F:
                self.board1_full_attempts += 1
                self.board1_masks[goal_id] = 0
                if self.cancel_race and self.board1_full_attempts == 1:
                    self.transport.send_frame(
                        board1_ack_frame(
                            ArmGoalAckResult.STAGING_TIMEOUT,
                            goal_id,
                            0x07,
                            duration_ms,
                        )
                    )
                elif (
                    self.lose_first_board1_ready
                    and self.board1_full_attempts == 1
                ):
                    return
                else:
                    result = (
                        ArmGoalAckResult.DUPLICATE
                        if self.lose_first_board1_ready
                        else ArmGoalAckResult.READY
                    )
                    self.transport.send_frame(
                        board1_ack_frame(result, goal_id, 0x0F, duration_ms)
                    )
                    self.board1_ready_sent.set()
            return

        if frame.can_id == CAN_ID_BOARD2_POSITION_COMMAND:
            # Legacy Board2 accepts the point immediately. Byte0 bit 4 must
            # stay clear; bytes5..6 are speed and byte7 is a 5 ms duration.
            self.board2_legacy_frames.append(frame)
            self.board2_sent_after_ready = self.board1_ready_sent.is_set()
            return

        command, goal_id = frame.data[:2]
        duration_ms = int(self.duration_ms or 0)
        if command == 1:
            self.start_seen.set()
            if self.cancel_race:
                return
            self.transport.send_frame(
                board1_ack_frame(
                    ArmGoalAckResult.STARTED,
                    goal_id,
                    0x0F,
                    duration_ms,
                )
            )
            self.transport.send_frame(status_frame(1, 2))
            self.transport.send_frame(status_frame(2, 2))
        elif command == 2:
            self.cancel_seen.set()
            if self.cancel_race:
                self.transport.send_frame(
                    board1_ack_frame(
                        ArmGoalAckResult.BUSY,
                        goal_id,
                        0,
                        duration_ms,
                    )
                )
            self.transport.send_frame(
                board1_ack_frame(
                    ArmGoalAckResult.CANCELLED,
                    goal_id,
                    0,
                    duration_ms,
                )
            )
            # Legacy Board2 has no CANCEL ACK. A later status heartbeat is
            # the only proof that its already accepted target is now idle.
            timer = threading.Timer(
                0.02,
                self.transport.send_frame,
                args=(status_frame(2, 3),),
            )
            timer.daemon = True
            timer.start()


def open_session(firmware):
    holder = {}

    def receive(frame):
        coordinator = holder['coordinator']
        if frame.can_id in ACK_CAN_IDS:
            coordinator.update_ack(
                unpack_arm_goal_ack_v3(
                    frame.data,
                    board_id=1,
                )
            )
        elif frame.can_id in (CAN_ID_BOARD1_STATUS, CAN_ID_BOARD2_STATUS):
            board_id = 1 if frame.can_id == CAN_ID_BOARD1_STATUS else 2
            coordinator.update_status(
                unpack_status(
                    frame.data,
                    board_id=board_id,
                    board2_legacy=(board_id == 2),
                )
            )

    host = SocketCanTransport(
        VCAN_INTERFACE,
        receive_ids=(
            CAN_ID_BOARD1_ACK_V3,
            CAN_ID_BOARD1_STATUS,
            CAN_ID_BOARD2_STATUS,
        ),
        frame_callback=receive,
    )
    host.open()
    firmware.start()
    writer = SerializedCanWriter(host, batch_inter_frame_delay_s=0.001)
    coordinator = ArmGoalV3Coordinator(
        writer,
        ack_timeout_s=0.05,
        communication_timeout_s=0.2,
    )
    holder['coordinator'] = coordinator
    firmware.send_initial_status()
    deadline = time.monotonic() + 1.0
    while not coordinator.probe_capability() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert coordinator.capability_confirmed
    return host, writer, coordinator


def close_session(host, writer, firmware):
    writer.close()
    firmware.close()
    host.close()


def test_vcan_lost_ready_recovers_from_full_mask_duplicate():
    firmware = VcanFirmware(lose_first_board1_ready=True)
    host, writer, coordinator = open_session(firmware)
    try:
        completed = coordinator.execute(
            joint_names=JOINTS,
            positions_rad=[0.0] * 5,
            duration_ms=1000,
        )
        assert completed.goal_id == 0
        assert firmware.board1_full_attempts == 2
        assert len(firmware.board2_legacy_frames) == 1
        board2_data = firmware.board2_legacy_frames[0].data
        assert board2_data[0] & 0x80
        assert not board2_data[0] & 0x10
        assert struct.unpack_from('<H', board2_data, 5)[0] == 0
        assert board2_data[7] == 200
        assert firmware.board2_sent_after_ready
        assert firmware.start_seen.is_set()
    finally:
        close_session(host, writer, firmware)


def test_vcan_staging_timeout_and_busy_race_do_not_break_cancel():
    firmware = VcanFirmware(cancel_race=True)
    host, writer, coordinator = open_session(firmware)
    cancel_requested = threading.Event()
    outcome = []

    def execute():
        try:
            coordinator.execute(
                joint_names=JOINTS,
                positions_rad=[0.0] * 5,
                duration_ms=1000,
                cancel_requested=cancel_requested.is_set,
            )
        except Exception as exc:
            outcome.append(exc)

    try:
        worker = threading.Thread(target=execute)
        worker.start()
        assert firmware.start_seen.wait(1.0)
        cancel_requested.set()
        worker.join(timeout=1.0)
        assert not worker.is_alive()
        assert len(outcome) == 1
        assert isinstance(outcome[0], ArmGoalV3Canceled)
        assert firmware.board1_full_attempts == 2
        assert len(firmware.board2_legacy_frames) == 1
        assert firmware.board2_sent_after_ready
        assert firmware.cancel_seen.is_set()
    finally:
        close_session(host, writer, firmware)
