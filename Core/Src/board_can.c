#include "../Inc/board_can.h"
#include "../Inc/gpio.h"
#include "../Inc/stepper.h"
#include "../Inc/move.h"

static volatile uint8_t g_status_event;
static uint8_t g_status_sequence_counter;
static CanFrame g_pending_status_frame;
static CanFrame g_pending_position_frame;
static uint8_t g_status_tx_pending;
static uint8_t g_position_tx_pending;
static uint32_t g_next_tx_retry_ms;
#define ACK_TX_QUEUE_SIZE 32
static CanFrame g_ack_tx_queue[ACK_TX_QUEUE_SIZE];
static uint8_t g_ack_tx_head;
static uint8_t g_ack_tx_tail;

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
    return frame->dlc == 8;
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
    move_clear();
    g_error_code = error_code;
    if (!ESTOP_ACTIVE()) g_state = STATE_ERROR;
    board_can_request_status_event();
}

static uint8_t move_command_allowed(void)
{
    return g_enabled && !ESTOP_ACTIVE() && g_error_code == ERR_NONE &&
           !g_homing_active && system_all_homed();
}

static uint8_t goal_ack_is_repetitive(uint8_t result)
{
    return (result == GOAL_ACK_DUPLICATE || result == GOAL_ACK_BUSY) ? 1 : 0;
}

static uint8_t goal_ack_matches(const CanFrame *frame, uint8_t result,
                                uint8_t goal_id, uint8_t mask,
                                uint16_t duration_ms)
{
    if (frame->data[1] != result || frame->data[2] != goal_id ||
        frame->data[3] != mask) {
        return 0;
    }

    return (frame->data[6] == (uint8_t)(duration_ms & 0xFF) &&
            frame->data[7] == (uint8_t)(duration_ms >> 8)) ? 1 : 0;
}

static void fill_goal_ack(CanFrame *frame, uint8_t result, uint8_t goal_id,
                          uint8_t mask, uint16_t duration_ms)
{
    frame->id = BOARD_ACK_CAN_ID;
    frame->dlc = 8;
    frame->data[0] = 3;
    frame->data[1] = result;
    frame->data[2] = goal_id;
    frame->data[3] = mask;
    frame->data[4] = g_state;
    frame->data[5] = 0;
    frame->data[6] = (uint8_t)(duration_ms & 0xFF);
    frame->data[7] = (uint8_t)(duration_ms >> 8);
}

static void queue_goal_ack(uint8_t result, uint8_t goal_id,
                           uint8_t mask, uint16_t duration_ms)
{
    uint8_t next = (uint8_t)((g_ack_tx_tail + 1) % ACK_TX_QUEUE_SIZE);
    uint8_t index;
    CanFrame *frame;

    if (goal_ack_is_repetitive(result)) {
        index = g_ack_tx_head;
        while (index != g_ack_tx_tail) {
            if (goal_ack_matches(&g_ack_tx_queue[index], result, goal_id,
                                 mask, duration_ms)) {
                return;
            }
            index = (uint8_t)((index + 1) % ACK_TX_QUEUE_SIZE);
        }
    }

    if (next == g_ack_tx_head) {
        if (goal_ack_is_repetitive(result)) return;

        index = g_ack_tx_head;
        while (index != g_ack_tx_tail) {
            if (goal_ack_is_repetitive(g_ack_tx_queue[index].data[1])) {
                fill_goal_ack(&g_ack_tx_queue[index], result, goal_id,
                              mask, duration_ms);
                return;
            }
            index = (uint8_t)((index + 1) % ACK_TX_QUEUE_SIZE);
        }
        return;
    }

    frame = &g_ack_tx_queue[g_ack_tx_tail];
    fill_goal_ack(frame, result, goal_id, mask, duration_ms);
    g_ack_tx_tail = next;
}

void board_can_cancel_goal_acks(void)
{
    g_ack_tx_head = g_ack_tx_tail;
}

void board_can_check_goal_timeout(void)
{
    uint8_t goal_id;
    uint8_t mask;
    uint16_t duration_ms;
    if (!move_check_staging_timeout(&goal_id, &mask, &duration_ms)) return;

    queue_goal_ack(GOAL_ACK_STAGING_TIMEOUT, goal_id, mask, duration_ms);
    board_can_request_status_event();
}

static void handle_estop(const CanFrame *frame)
{
#if ENABLE_ESTOP_LOGIC
    if (!frame_is_exact_8_bytes(frame) || frame->data[0] != 1 || !reserved_zero(frame, 1)) {
        /* A malformed frame must not break a locked goal. Only an exact,
         * explicit E-stop command is allowed to stop active motion. */
        return;
    }

    g_estop = 1;
    g_homing_active = 0;
    g_state = STATE_ESTOP;
    move_clear();
    stepper_stop_all();
    board_can_cancel_goal_acks();
    board_can_request_status_event();
#else
    (void)frame;
#endif
}

static void handle_enable_disable(const CanFrame *frame)
{
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 1)) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    if (frame->data[0] == 1) {
        if (g_error_code != ERR_NONE || g_state == STATE_ERROR || g_state == STATE_ESTOP) {
            move_clear();
        }
        g_estop = 0;
        g_error_code = ERR_NONE;
        g_enabled = 1;
        motor_enable();
        if (g_state == STATE_ESTOP || g_state == STATE_ERROR ||
            g_state == STATE_INIT || g_state == STATE_DISABLED) {
            g_state = STATE_IDLE;
        }
    } else if (frame->data[0] == 0) {
        move_clear();
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

    if (!g_enabled || ESTOP_ACTIVE() || g_error_code != ERR_NONE) {
        enter_error(ERR_INVALID_CMD);
        return;
    }

    g_error_code = ERR_NONE;
    move_clear();
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

    g_estop = 0;
    g_error_code = ERR_NONE;
    move_clear();
    g_state = g_enabled ? STATE_IDLE : STATE_DISABLED;
    board_can_request_status_event();
}

static void handle_board_move(const CanFrame *frame)
{
    uint8_t b0;
    uint8_t motor_id;
    uint8_t relative;
    uint8_t step_mode;
    uint8_t goal_id;
    uint8_t mask = 0;
    uint16_t duration_ms;
    int32_t target_raw;
    int32_t target_step;
    uint8_t result;

    if (!frame_is_exact_8_bytes(frame)) {
        queue_goal_ack(GOAL_ACK_INVALID, 0, 0, 0);
        return;
    }

    b0 = frame->data[0];
    relative = (b0 & CAN_CTRL_RELATIVE) ? 1 : 0;
    step_mode = (b0 & CAN_CTRL_STEP_MODE) ? 1 : 0;
    motor_id = b0 & CAN_CTRL_MOTOR_MASK;
    goal_id = frame->data[5];
    duration_ms = read_u16_le(&frame->data[6]);

    if (!move_command_allowed() || !(b0 & CAN_CTRL_EXECUTE) ||
        !(b0 & CAN_CTRL_GOAL_V3) || relative || motor_id >= AXIS_COUNT ||
        duration_ms == 0) {
        queue_goal_ack(GOAL_ACK_INVALID, goal_id, 0, duration_ms);
        return;
    }

    target_raw = read_i32_le(&frame->data[1]);

    if (!move_resolve_target_step(motor_id, target_raw, relative, step_mode, &target_step)) {
        queue_goal_ack(GOAL_ACK_INVALID, goal_id, 0, duration_ms);
        return;
    }

    result = move_stage_goal_axis(motor_id, target_step, goal_id,
                                  duration_ms, &mask);
    if (result == GOAL_STAGE_READY) {
        queue_goal_ack(GOAL_ACK_READY, goal_id, mask, duration_ms);
        board_can_request_status_event();
    } else if (result == GOAL_STAGE_DUPLICATE) {
        queue_goal_ack(GOAL_ACK_DUPLICATE, goal_id, mask, duration_ms);
    } else if (result == GOAL_STAGE_BUSY) {
        queue_goal_ack(GOAL_ACK_BUSY, goal_id, mask, duration_ms);
    } else if (result == GOAL_STAGE_INVALID) {
        queue_goal_ack(GOAL_ACK_CONFLICT, goal_id, mask, duration_ms);
    }
}

static void handle_goal_control(const CanFrame *frame)
{
    uint8_t goal_id;
    uint16_t duration_ms;
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 2)) {
        queue_goal_ack(GOAL_ACK_INVALID, frame->data[1], 0, 0);
        return;
    }
    goal_id = frame->data[1];
    duration_ms = move_goal_duration_ms();
    if (frame->data[0] == GOAL_CONTROL_START) {
        if (!move_command_allowed() || !move_start_goal(goal_id)) {
            queue_goal_ack(GOAL_ACK_INVALID, goal_id, move_goal_mask(), duration_ms);
            return;
        }
        queue_goal_ack(GOAL_ACK_STARTED, goal_id, move_goal_mask(), duration_ms);
    } else if (frame->data[0] == GOAL_CONTROL_CANCEL) {
        if (!move_cancel_goal(goal_id)) {
            queue_goal_ack(GOAL_ACK_CONFLICT, goal_id, move_goal_mask(), duration_ms);
            return;
        }
        queue_goal_ack(GOAL_ACK_CANCELLED, goal_id, 0, duration_ms);
    } else {
        queue_goal_ack(GOAL_ACK_INVALID, goal_id, 0, duration_ms);
        return;
    }
    board_can_request_status_event();
}

void board_can_handle_frame(const CanFrame *frame)
{
    /* Once START has made the goal active, the board owns that goal until it
     * reaches the target. Ignore cancel, disable, homing, clear-error, and new
     * move/start commands. A valid E-stop remains the only CAN command that
     * may interrupt active motion. */
    if (g_motion_active && frame->id != CAN_ID_ESTOP) return;

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
    case CAN_ID_GOAL_CONTROL:
        handle_goal_control(frame);
        break;
    default:
        break;
    }
}

void board_can_queue_status(void)
{
    CanFrame frame;
    int32_t current_snapshot[AXIS_COUNT];
    int32_t target_snapshot[AXIS_COUNT];
    uint8_t state_snapshot;
    uint8_t fatal_error_snapshot;
    uint8_t reported_error_snapshot;
    uint8_t enabled_snapshot;
    uint8_t homing_done_snapshot;
    uint8_t homing_active_snapshot;
    uint8_t motion_active_snapshot;
    uint8_t axis_flags[4] = {0, 0, 0, 0};

    frame.id = BOARD_STATUS_CAN_ID;
    frame.dlc = 8;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        current_snapshot[i] = g_current_step[i];
        target_snapshot[i] = g_target_step[i];
    }
    state_snapshot = g_state;
    fatal_error_snapshot = g_error_code;
    reported_error_snapshot = g_error_code;
    enabled_snapshot = g_enabled;
    homing_done_snapshot = g_homing_done_bits;
    homing_active_snapshot = g_homing_active;
    motion_active_snapshot = g_motion_active;

    for (uint8_t i = 0; i < AXIS_COUNT && i < 4; i++) {
        axis_flags[i] = make_position_flags(i,
                                            state_snapshot,
                                            fatal_error_snapshot,
                                            enabled_snapshot,
                                            homing_done_snapshot,
                                            homing_active_snapshot,
                                            motion_active_snapshot,
                                            current_snapshot,
                                            target_snapshot) & 0x0F;
    }

    frame.data[0] = state_snapshot;
    frame.data[1] = reported_error_snapshot;
    frame.data[2] = (uint8_t)(axis_flags[0] | (uint8_t)(axis_flags[1] << 4));
    frame.data[3] = (uint8_t)(axis_flags[2] | (uint8_t)(axis_flags[3] << 4));
    frame.data[4] = stepper_limit_switch_status_bits();
    frame.data[5] = ESTOP_ACTIVE() ? 0 : move_goal_slot_free();
    frame.data[6] = g_enabled;
    frame.data[7] = g_status_sequence_counter;

    g_pending_status_frame = frame;
    g_status_tx_pending = 1;
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
    if (homed && enabled_snapshot && (!ENABLE_ESTOP_LOGIC || state_snapshot != STATE_ESTOP) &&
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

void board_can_queue_position_feedback_all(void)
{
    int32_t current_snapshot[AXIS_COUNT];
    CanFrame frame;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        current_snapshot[i] = g_current_step[i];
    }

    frame.id = BOARD_POSITION_CAN_ID;
    frame.dlc = 8;
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = 0;
    }
    for (uint8_t i = 0; i < AXIS_COUNT && i < 4; i++) {
        write_i16_le(&frame.data[i * 2], clamp_i16(step_to_angle(i, current_snapshot[i])));
    }

    g_pending_position_frame = frame;
    g_position_tx_pending = 1;
}

void board_can_service_tx(void)
{
    Mcp2515SendResult result;

    if ((int32_t)(global_tick_ms - g_next_tx_retry_ms) < 0) return;

    if (g_ack_tx_head != g_ack_tx_tail) {
        result = mcp2515_send_frame(&g_ack_tx_queue[g_ack_tx_head]);
        if (result == MCP2515_SEND_OK) {
            g_ack_tx_head = (uint8_t)((g_ack_tx_head + 1) % ACK_TX_QUEUE_SIZE);
        } else {
            g_next_tx_retry_ms = global_tick_ms + 1;
            return;
        }
    }

    if (g_status_tx_pending) {
        result = mcp2515_send_frame(&g_pending_status_frame);
        if (result == MCP2515_SEND_OK) {
            g_status_tx_pending = 0;
            g_status_sequence_counter++;
        } else {
            g_next_tx_retry_ms = global_tick_ms + 1;
            return;
        }
    }

    if (g_position_tx_pending) {
        result = mcp2515_send_frame(&g_pending_position_frame);
        if (result == MCP2515_SEND_OK) {
            g_position_tx_pending = 0;
        } else {
            g_next_tx_retry_ms = global_tick_ms + 1;
        }
    }
}

void board_can_flush_status_event(void)
{
    if (!g_status_event) return;

    g_status_event = 0;
    board_can_queue_status();
}
