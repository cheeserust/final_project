#include "../Inc/stepper.h"
#include "../Inc/gpio.h"

static uint16_t limit_debounce[AXIS_COUNT];
static const uint8_t home_dir[AXIS_COUNT] = {
    DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE
};

static void step_pin_high(uint8_t id)
{
    if (id == 0U) GPIO_SET_ODR(STEP1_PORT, STEP1_PIN);
    else if (id == 1U) GPIO_SET_ODR(STEP2_PORT, STEP2_PIN);
    else if (id == 2U) GPIO_SET_ODR(STEP3_PORT, STEP3_PIN);
    else if (id == 3U) GPIO_SET_ODR(STEP4_PORT, STEP4_PIN);
}

static void step_pin_low(uint8_t id)
{
    if (id == 0U) GPIO_CLEAR_ODR(STEP1_PORT, STEP1_PIN);
    else if (id == 1U) GPIO_CLEAR_ODR(STEP2_PORT, STEP2_PIN);
    else if (id == 2U) GPIO_CLEAR_ODR(STEP3_PORT, STEP3_PIN);
    else if (id == 3U) GPIO_CLEAR_ODR(STEP4_PORT, STEP4_PIN);
}

static void dir_set(uint8_t id, uint8_t dir)
{
    GPIO_TypeDef *port = GPIOA;
    uint8_t pin = 0;

    if (id == 0U) { port = DIR1_PORT; pin = DIR1_PIN; }
    else if (id == 1U) { port = DIR2_PORT; pin = DIR2_PIN; }
    else if (id == 2U) { port = DIR3_PORT; pin = DIR3_PIN; }
    else if (id == 3U) { port = DIR4_PORT; pin = DIR4_PIN; }
    else return;

    if (dir) GPIO_SET_ODR(port, pin);
    else GPIO_CLEAR_ODR(port, pin);
}

static uint8_t limit_raw_pressed(uint8_t id)
{
    uint8_t value = 1U;
    if (id == 0U) value = (uint8_t)GPIO_READ(LIM1_PORT, LIM1_PIN);
    else if (id == 1U) value = (uint8_t)GPIO_READ(LIM2_PORT, LIM2_PIN);
    else if (id == 2U) value = (uint8_t)GPIO_READ(LIM3_PORT, LIM3_PIN);
    else if (id == 3U) value = (uint8_t)GPIO_READ(LIM4_PORT, LIM4_PIN);
    return (value == LIMIT_ACTIVE_LEVEL);
}

static uint8_t limit_pressed_debounced(uint8_t id)
{
    if (limit_raw_pressed(id)) {
        if (limit_debounce[id] < LIMIT_DEBOUNCE_TICKS) limit_debounce[id]++;
    } else {
        limit_debounce[id] = 0;
    }
    return (limit_debounce[id] >= LIMIT_DEBOUNCE_TICKS);
}

uint8_t stepper_limit_status_bits(void)
{
    uint8_t bits = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (limit_raw_pressed(i)) bits |= (uint8_t)(1U << i);
    }
    return bits;
}

void stepper_init(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis[i].current_step = 0;
        axis[i].target_step = 0;
        axis[i].seg_start_step = 0;
        axis[i].seg_end_step = 0;
        axis[i].seg_delta_step = 0;
        axis[i].seg_total_ms = 0;
        axis[i].seg_elapsed_ms = 0;
        axis[i].moving = 0;
        axis[i].homing = 0;
        axis[i].homing_done = 0;
        axis[i].enabled = 0;
        limit_debounce[i] = 0;
        step_pin_low(i);
    }
}

void stepper_stop_axis(uint8_t id)
{
    if (id >= AXIS_COUNT) return;
    step_pin_low(id);
    axis[id].target_step = axis[id].current_step;
    axis[id].seg_start_step = axis[id].current_step;
    axis[id].seg_end_step = axis[id].current_step;
    axis[id].seg_delta_step = 0;
    axis[id].seg_total_ms = 0;
    axis[id].seg_elapsed_ms = 0;
    axis[id].moving = 0;
    axis[id].homing = 0;
}

void stepper_stop_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        stepper_stop_axis(i);
    }
}

void stepper_start_homing(uint8_t id)
{
    if (id >= AXIS_COUNT) {
        global_motor_error = ERR_INVALID_CMD;
        return;
    }
    if (!global_motor_enabled || global_motor_estop) return;

    axis[id].target_step = axis[id].current_step;
    axis[id].seg_total_ms = 0;
    axis[id].seg_elapsed_ms = 0;
    axis[id].moving = 1;
    axis[id].homing = 1;
    axis[id].homing_done = 0;
    limit_debounce[id] = 0;
    dir_set(id, home_dir[id]);
    step_pin_low(id);
}

void stepper_start_homing_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        stepper_start_homing(i);
    }
}

static void emit_step(uint8_t id)
{
    step_pin_high(id);
    for (volatile int delay = 0; delay < 10; delay++) {}
    step_pin_low(id);
}

void stepper_update_10us(void)
{
    static uint16_t homing_tick[AXIS_COUNT];

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].homing) {
            if (limit_pressed_debounced(i)) {
                axis[i].current_step = 0;
                axis[i].target_step = 0;
                axis[i].seg_start_step = 0;
                axis[i].seg_end_step = 0;
                axis[i].seg_delta_step = 0;
                axis[i].seg_total_ms = 0;
                axis[i].seg_elapsed_ms = 0;
                axis[i].homing = 0;
                axis[i].moving = 0;
                axis[i].homing_done = 1;
                step_pin_low(i);
                continue;
            }

            if (++homing_tick[i] >= HOMING_INTERVAL_TICKS) {
                homing_tick[i] = 0;
                dir_set(i, home_dir[i]);
                emit_step(i);
            }
            continue;
        }

        if (!axis[i].moving) continue;

        if (axis[i].current_step < axis[i].target_step) {
            dir_set(i, DIR_POSITIVE);
            emit_step(i);
            axis[i].current_step++;
        } else if (axis[i].current_step > axis[i].target_step) {
            dir_set(i, DIR_NEGATIVE);
            emit_step(i);
            axis[i].current_step--;
        }
    }
}
