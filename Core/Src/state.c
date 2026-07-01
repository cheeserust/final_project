#include "../Inc/state.h"

MotorState axis[AXIS_COUNT];
volatile uint8_t global_motor_enabled = 0;
volatile uint8_t global_motor_state = STATE_INIT;
volatile uint8_t global_motor_error = ERR_NONE;
volatile uint8_t global_motor_estop = 0;
volatile uint32_t global_tick_ms = 0;

static uint8_t is_any_axis_homing(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].homing) return 1;
    }
    return 0;
}

static uint8_t is_any_axis_moving(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].moving) return 1;
    }
    return 0;
}

void system_update_state(void)
{
    if (global_motor_estop) global_motor_state = STATE_ESTOP;
    else if (global_motor_error != ERR_NONE) global_motor_state = STATE_ERROR;
    else if (!global_motor_enabled) global_motor_state = STATE_DISABLED;
    else if (is_any_axis_homing()) global_motor_state = STATE_HOMING;
    else if (is_any_axis_moving()) global_motor_state = STATE_MOVING;
    else global_motor_state = STATE_IDLE;
}

uint8_t system_homing_done_bits(void)
{
    uint8_t bits = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].homing_done) bits |= (uint8_t)(1 << i);
    }
    return bits;
}

uint8_t system_enabled_status(void)
{
    return global_motor_enabled ? 1 : 0;
}

uint8_t system_first_moving_axis(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].moving || axis[i].homing) return i;
    }
    return 255;
}
