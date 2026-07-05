#include "../Inc/board_can.h"
#include "../Inc/gpio.h"
#include "../Inc/stepper.h"
#include "../Inc/trajectory.h"

static volatile uint8_t g_status_event;
#if BOARD_ID == 1
static uint8_t g_status_sequence_counter;
#endif

static int32_t read_i32_le(const uint8_t *p)
{
    uint32_t v = ((uint32_t)p[0]) |
                 ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) |
                 ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));
}

#if BOARD_ID != 1
static void write_i32_le(uint8_t *p, int32_t value)
{
    uint32_t v = (uint32_t)value;
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
#endif

#if BOARD_ID == 1
static void write_i16_le(uint8_t *p, int16_t value)
{
    uint16_t v = (uint16_t)value;
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int16_t clamp_i16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}
#endif

static uint8_t make_position_flags(uint8_t motor_id,
                                   uint8_t state_snapshot,
                                   uint8_t error_snapshot,
                                   uint8_t enabled_snapshot,
                                   uint8_t homing_done_snapshot,
                                   uint8_t homing_active_snapshot,
                                   uint8_t motion_active_snapshot,
                                   const int32_t current_step_snapshot[AXIS_COUNT],
                                   const int32_t target_step_snapshot[AXIS_COUNT]);

static uint8_t frame_is_exact_8_bytes(const CanFrame *frame)
{
    return (frame != 0 && frame->dlc == 8) ? 1 : 0;
}

static uint8_t reserved_zero(const CanFrame *frame, uint8_t start_index)
{
    for (uint8_t i = start_index; i < 8; i++) {
        if (frame->data[i] != 0) return 0;
    }
    return 1;
}

void board_can_request_status_event(void)
{
    g_status_event = 1;
}

static void enter_error(uint8_t error_code)
{
    trajectory_cancel_staging();
    trajectory_stop_motion();
    g_error_code = error_code;
    if (!g_estop) g_state = STATE_ERROR;
    board_can_request_status_event();
}

static uint8_t motion_command_allowed(void)
{
    if (!g_enabled) return 0;
    if (g_estop) return 0;
    if (g_error_code != ERR_NONE) return 0;
    if (g_homing_active) return 0;
    if (!system_all_homed()) return 0;
    return 1;
}

static void handle_estop(const CanFrame *frame)
{
    if (!frame_is_exact_8_bytes(frame) || frame->data[0] != 1 || !reserved_zero(frame, 1)) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    g_estop = 1;
    g_enabled = 0;
    g_error_code = ERR_NONE;
    g_homing_active = 0;
    g_state = STATE_ESTOP;
    trajectory_clear();
    stepper_stop_all();
    motor_disable();
    board_can_request_status_event();
}

static void handle_enable_disable(const CanFrame *frame)
{
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 1)) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    if (frame->data[0] == 1) {
        g_estop = 0;
        g_error_code = ERR_NONE;
        g_enabled = 1;
        motor_enable();
        if (g_state == STATE_ESTOP || g_state == STATE_ERROR ||
            g_state == STATE_INIT || g_state == STATE_DISABLED) {
            g_state = STATE_IDLE;
        }
    } else if (frame->data[0] == 0) {
        trajectory_clear();
        stepper_stop_all();
        g_homing_active = 0;
        g_enabled = 0;
        motor_disable();
        g_state = STATE_DISABLED;
    } else {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    board_can_request_status_event();
}

static void handle_arm_homing(const CanFrame *frame)
{
    if (!frame_is_exact_8_bytes(frame) || frame->data[0] != HOMING_ALL_AXIS ||
        frame->data[1] != 0 || !reserved_zero(frame, 2)) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    if (!g_enabled || g_estop || g_error_code != ERR_NONE) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    g_error_code = ERR_NONE;
    trajectory_clear();
    stepper_stop_all();
    stepper_start_homing_all();
    board_can_request_status_event();
}

static void handle_clear_error(const CanFrame *frame)
{
    if (!frame_is_exact_8_bytes(frame) || frame->data[0] != HOMING_ALL_AXIS ||
        !reserved_zero(frame, 1)) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    g_error_code = ERR_NONE;
    trajectory_cancel_staging();
    if (!g_estop) {
        g_state = g_enabled ? STATE_IDLE : STATE_DISABLED;
    }
    board_can_request_status_event();
}

static void handle_board_move(const CanFrame *frame)
{
    uint8_t b0;
    uint8_t motor_id;
    uint8_t execute;
    uint8_t relative;
    uint8_t step_mode;
    uint8_t duration_5ms;
    int32_t target_raw;
    int32_t target_step;
    uint16_t speed;
    uint8_t result;

    if (!frame_is_exact_8_bytes(frame)) {
        enter_error(ERR_INVALID_CMD);
        return;
    }
    if (!motion_command_allowed()) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    b0 = frame->data[0];
    execute = (b0 & CAN_CTRL_EXECUTE) ? 1 : 0;
    relative = (b0 & CAN_CTRL_RELATIVE) ? 1 : 0;
    step_mode = (b0 & CAN_CTRL_STEP_MODE) ? 1 : 0;
    motor_id = b0 & CAN_CTRL_MOTOR_MASK;

    if (!execute || (b0 & CAN_CTRL_RESERVED) || motor_id >= AXIS_COUNT) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    target_raw = read_i32_le(&frame->data[1]);
    speed = read_u16_le(&frame->data[5]);
    duration_5ms = frame->data[7];

    if (!trajectory_resolve_target_step(motor_id, target_raw, relative, step_mode, &target_step)) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    result = trajectory_add_axis_command(motor_id, target_step, speed, duration_5ms);
    if (result == TRAJECTORY_STAGING_INVALID) {
        enter_error(ERR_INVALID_CMD);
        return;
    }
    if (result == TRAJECTORY_STAGING_QUEUE_FULL) {
        enter_error(ERR_QUEUE_FULL);
        return;
    }
    if (result == TRAJECTORY_STAGING_COMMITTED) {
        board_can_request_status_event();
    }
}

void board_can_handle_frame(const CanFrame *frame)
{
    if (frame == 0) return;

    switch (frame->id) {
    case CAN_ID_ESTOP:
        handle_estop(frame);
        break;
    case CAN_ID_ENABLE:
        handle_enable_disable(frame);
        break;
    case CAN_ID_HOMING:
        handle_arm_homing(frame);
        break;
    case CAN_ID_CLEAR_ERROR:
        handle_clear_error(frame);
        break;
    case BOARD_MOVE_CAN_ID:
        handle_board_move(frame);
        break;
    default:
        break;
    }
}

void board_can_send_status(void)
{
    CanFrame frame;

    frame.id = BOARD_STATUS_CAN_ID;
    frame.dlc = 8;
    frame.data[0] = g_state;
    frame.data[1] = g_error_code;
#if BOARD_ID == 1
    {
        int32_t current_snapshot[AXIS_COUNT];
        int32_t target_snapshot[AXIS_COUNT];
        uint8_t state_snapshot;
        uint8_t error_snapshot;
        uint8_t enabled_snapshot;
        uint8_t homing_done_snapshot;
        uint8_t homing_active_snapshot;
        uint8_t motion_active_snapshot;
        uint8_t axis_flags[AXIS_COUNT];

        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            current_snapshot[i] = g_current_step[i];
            target_snapshot[i] = g_target_step[i];
        }
        state_snapshot = g_state;
        error_snapshot = g_error_code;
        enabled_snapshot = g_enabled;
        homing_done_snapshot = g_homing_done_bits;
        homing_active_snapshot = g_homing_active;
        motion_active_snapshot = g_motion_active;

        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            axis_flags[i] = make_position_flags(i,
                                                state_snapshot,
                                                error_snapshot,
                                                enabled_snapshot,
                                                homing_done_snapshot,
                                                homing_active_snapshot,
                                                motion_active_snapshot,
                                                current_snapshot,
                                                target_snapshot) & 0x0F;
        }

        frame.data[0] = state_snapshot;
        frame.data[1] = error_snapshot;
        frame.data[2] = (uint8_t)(axis_flags[0] | (uint8_t)(axis_flags[1] << 4));
        frame.data[3] = (uint8_t)(axis_flags[2] | (uint8_t)(axis_flags[3] << 4));
    }
#else
    frame.data[2] = system_homing_done_bits();
    frame.data[3] = system_first_moving_axis();
#endif
    frame.data[4] = stepper_limit_switch_status_bits();
    frame.data[5] = get_free_axis_command_count();
    frame.data[6] = system_enabled_status();
#if BOARD_ID == 1
    frame.data[7] = g_status_sequence_counter++;
#else
    frame.data[7] = 0;
#endif

    (void)mcp2515_send_frame(&frame);
}

static uint8_t make_position_flags(uint8_t motor_id,
                                   uint8_t state_snapshot,
                                   uint8_t error_snapshot,
                                   uint8_t enabled_snapshot,
                                   uint8_t homing_done_snapshot,
                                   uint8_t homing_active_snapshot,
                                   uint8_t motion_active_snapshot,
                                   const int32_t current_step_snapshot[AXIS_COUNT],
                                   const int32_t target_step_snapshot[AXIS_COUNT])
{
    uint8_t flags = 0;
    uint8_t homed = (homing_done_snapshot & (uint8_t)(1 << motor_id)) ? 1 : 0;
    uint8_t moving = 0;

    if (homing_active_snapshot && !homed) moving = 1;
    if (motion_active_snapshot &&
        current_step_snapshot[motor_id] != target_step_snapshot[motor_id]) {
        moving = 1;
    }

    if (homed) flags |= 0x01;
    if (homed && enabled_snapshot && state_snapshot != STATE_ESTOP &&
        state_snapshot != STATE_ERROR && error_snapshot == ERR_NONE) {
        flags |= 0x02;
    }
    if (moving) flags |= 0x04;
    if (homed && !moving && !homing_active_snapshot &&
        current_step_snapshot[motor_id] == target_step_snapshot[motor_id]) {
        flags |= 0x08;
    }

    return flags;
}

#if BOARD_ID != 1
static void board_can_send_position_feedback(uint8_t motor_id,
                                             uint8_t state_snapshot,
                                             uint8_t error_snapshot,
                                             uint8_t enabled_snapshot,
                                             uint8_t homing_done_snapshot,
                                             uint8_t homing_active_snapshot,
                                             uint8_t motion_active_snapshot,
                                             const int32_t current_step_snapshot[AXIS_COUNT],
                                             const int32_t target_step_snapshot[AXIS_COUNT])
{
    static uint8_t sequence_counter;
    CanFrame frame;

    if (motor_id >= AXIS_COUNT) return;

    frame.id = BOARD_POSITION_CAN_ID;
    frame.dlc = 8;
    frame.data[0] = motor_id;
    frame.data[1] = make_position_flags(motor_id,
                                        state_snapshot,
                                        error_snapshot,
                                        enabled_snapshot,
                                        homing_done_snapshot,
                                        homing_active_snapshot,
                                        motion_active_snapshot,
                                        current_step_snapshot,
                                        target_step_snapshot);
    write_i32_le(&frame.data[2], step_to_angle(motor_id, current_step_snapshot[motor_id]));
    frame.data[6] = error_snapshot;
    frame.data[7] = sequence_counter++;

    (void)mcp2515_send_frame(&frame);
}
#endif

void board_can_send_position_feedback_all(void)
{
#if BOARD_ID == 1
    int32_t current_snapshot[AXIS_COUNT];
    CanFrame frame;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        current_snapshot[i] = g_current_step[i];
    }

    frame.id = BOARD_POSITION_CAN_ID;
    frame.dlc = 8;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        write_i16_le(&frame.data[i * 2],
                     clamp_i16(step_to_angle(i, current_snapshot[i])));
    }

    (void)mcp2515_send_frame(&frame);
#else
    int32_t current_snapshot[AXIS_COUNT];
    int32_t target_snapshot[AXIS_COUNT];
    uint8_t state_snapshot;
    uint8_t error_snapshot;
    uint8_t enabled_snapshot;
    uint8_t homing_done_snapshot;
    uint8_t homing_active_snapshot;
    uint8_t motion_active_snapshot;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        current_snapshot[i] = g_current_step[i];
        target_snapshot[i] = g_target_step[i];
    }
    state_snapshot = g_state;
    error_snapshot = g_error_code;
    enabled_snapshot = g_enabled;
    homing_done_snapshot = g_homing_done_bits;
    homing_active_snapshot = g_homing_active;
    motion_active_snapshot = g_motion_active;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        board_can_send_position_feedback(i,
                                         state_snapshot,
                                         error_snapshot,
                                         enabled_snapshot,
                                         homing_done_snapshot,
                                         homing_active_snapshot,
                                         motion_active_snapshot,
                                         current_snapshot,
                                         target_snapshot);
    }
#endif
}

void board_can_flush_status_event(void)
{
    if (!g_status_event) return;

    g_status_event = 0;
    board_can_send_status();
}
