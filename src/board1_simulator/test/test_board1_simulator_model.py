"""Unit tests for the pure Board1 simulator model."""

from arm_can_bridge.can_protocol import (
    ALL_MOTORS,
    BOARD_ID_BOARD3,
    QUEUE_CAPACITY,
    BoardError,
    BoardState,
    board_id_from_status_can_id,
    pack_clear_error,
    pack_enable,
    pack_estop,
    pack_homing,
    pack_position_command,
    unpack_status,
)
from board1_simulator.model import Board1SimulatorModel
from board1_simulator.model import make_board3_simulator_model


def status_of(model):
    """Return decoded status from the simulator model."""
    frame = model.build_status_frame()
    return unpack_status(
        frame.data,
        board_id=board_id_from_status_can_id(frame.can_id),
    )


def send_board1_point(
    model,
    *,
    target_pos=3000,
    duration_ticks=10,
):
    """Send one complete four-axis Board1 trajectory point."""
    for motor_id in range(4):
        model.handle_frame(
            pack_position_command(
                motor_id=motor_id,
                target_pos=target_pos,
                speed=0,
                duration_ticks=duration_ticks,
            )
        )


def send_board3_point(
    model,
    *,
    target_pos=1000,
    duration_ticks=10,
):
    """Send one complete nine-servo Board3 point."""
    for motor_id in range(9):
        model.handle_frame(
            pack_position_command(
                board_id=BOARD_ID_BOARD3,
                motor_id=motor_id,
                target_pos=target_pos,
                speed=0,
                duration_ticks=duration_ticks,
            )
        )


def test_initial_status_is_idle_disabled_unhomed():
    model = Board1SimulatorModel()
    status = status_of(model)

    assert status.state == BoardState.IDLE
    assert status.error_code == BoardError.NONE
    assert status.enabled is False
    assert status.homing_done_bits == 0
    assert status.queue_free == QUEUE_CAPACITY


def test_enable_and_disable():
    model = Board1SimulatorModel()

    model.handle_frame(pack_enable(True))
    assert status_of(model).enabled is True

    model.handle_frame(pack_enable(False))
    status = status_of(model)
    assert status.enabled is False
    assert status.state == BoardState.IDLE


def test_home_all_axes_after_enable():
    model = Board1SimulatorModel(homing_duration_s=0.1)

    model.handle_frame(pack_enable(True))
    model.handle_frame(pack_homing(ALL_MOTORS))

    assert status_of(model).state == BoardState.HOMING

    model.tick(0.2)
    status = status_of(model)

    assert status.state == BoardState.IDLE
    assert status.homing_done_bits == 0x0F


def test_move_before_homing_sets_invalid_command_error():
    model = Board1SimulatorModel()
    model.handle_frame(pack_enable(True))

    model.handle_frame(
        pack_position_command(
            motor_id=0,
            target_pos=3000,
            speed=0,
            duration_ticks=10,
        )
    )

    status = status_of(model)
    assert status.state == BoardState.ERROR
    assert status.error_code == BoardError.INVALID_CMD


def test_position_command_runs_after_homing():
    model = Board1SimulatorModel(homing_duration_s=0.0)
    model.handle_frame(pack_enable(True))
    model.handle_frame(pack_homing(ALL_MOTORS))

    send_board1_point(model, target_pos=3000, duration_ticks=10)

    status = status_of(model)
    assert status.state == BoardState.MOVING
    assert status.moving_motor_id == 0

    model.tick(0.1)
    status = status_of(model)
    assert status.state == BoardState.IDLE
    assert status.moving_motor_id == ALL_MOTORS
    assert model.commanded_angle_raw[0] == 3000


def test_queue_full_sets_error_and_keeps_existing_queue():
    model = Board1SimulatorModel(
        queue_capacity=4,
        homing_duration_s=0.0,
    )
    model.handle_frame(pack_enable(True))
    model.handle_frame(pack_homing(ALL_MOTORS))

    send_board1_point(model, target_pos=1000, duration_ticks=50)
    send_board1_point(model, target_pos=2000, duration_ticks=50)
    model.handle_frame(
        pack_position_command(
            motor_id=0,
            target_pos=3000,
            speed=0,
            duration_ticks=50,
        )
    )

    status = status_of(model)
    assert status.state == BoardState.ERROR
    assert status.error_code == BoardError.QUEUE_FULL


def test_clear_error_does_not_clear_estop():
    model = Board1SimulatorModel()

    model.handle_frame(pack_estop())
    model.handle_frame(pack_clear_error(ALL_MOTORS))

    assert status_of(model).state == BoardState.ESTOP


def test_enable_clears_estop_and_error():
    model = Board1SimulatorModel()

    model.handle_frame(pack_estop())
    model.handle_frame(pack_enable(True))

    status = status_of(model)
    assert status.state == BoardState.IDLE
    assert status.error_code == BoardError.NONE
    assert status.enabled is True


def test_board3_ready_and_servo_staging():
    model = make_board3_simulator_model()

    status = status_of(model)
    assert status.board_id == BOARD_ID_BOARD3
    assert status.homing_done_bits == 0

    model.handle_frame(pack_enable(True))
    status = status_of(model)

    assert status.enabled is True
    assert status.homing_done_bits == 1

    send_board3_point(model, target_pos=2500, duration_ticks=10)

    status = status_of(model)
    assert status.state == BoardState.MOVING
    assert status.board3_staging_count == 0

    model.tick(0.1)
    status = status_of(model)
    assert status.state == BoardState.IDLE
    assert status.board3_staging_count == 0
    assert status.board3_buffer_free == 9
    assert status.board3_fault_motor_id == ALL_MOTORS
    assert model.commanded_angle_raw == [2500] * 9
