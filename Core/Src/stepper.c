#include "../Inc/stepper.h"
#include "../Inc/gpio.h"
#include "../Inc/trajectory.h"

static uint16_t limit_switch_debounce[AXIS_COUNT];
static const uint8_t home_dir[AXIS_COUNT] = {
    DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE
};

static uint8_t limit_switch_pressed(uint8_t id)
{
    uint8_t value = 1;
    if (id == 0) value = (uint8_t)((LIM1_PORT->IDR & (1 << LIM1_PIN)) != 0);
    else if (id == 1) value = (uint8_t)((LIM2_PORT->IDR & (1 << LIM2_PIN)) != 0);
    else if (id == 2) value = (uint8_t)((LIM3_PORT->IDR & (1 << LIM3_PIN)) != 0);
    else if (id == 3) value = (uint8_t)((LIM4_PORT->IDR & (1 << LIM4_PIN)) != 0);
    return (value == LIMIT_SWITCH_ACTIVE_LEVEL);
}

static uint8_t limit_switch_pressed_stable(uint8_t id)
{
    if (limit_switch_pressed(id)) {
        if (limit_switch_debounce[id] < LIMIT_SWITCH_DEBOUNCE_TICKS)
        {
            limit_switch_debounce[id]++;
        }
    } else {
        limit_switch_debounce[id] = 0;
    }
    return (limit_switch_debounce[id] >= LIMIT_SWITCH_DEBOUNCE_TICKS);
}

uint8_t stepper_limit_switch_status_bits(void)
{
    uint8_t bits = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (limit_switch_pressed(i)) bits |= (uint8_t)(1 << i);
    }
    return bits;
}

void stepper_init(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis[i].current_step = 0;
        axis[i].target_step = 0;
        axis[i].move_start_step = 0;
        axis[i].move_end_step = 0;
        axis[i].move_step_offset = 0;
        axis[i].move_total_time_ms = 0;
        axis[i].move_elapsed_time_ms = 0;
        axis[i].moving = 0;
        axis[i].homing = 0;
        axis[i].homing_done = 0;
        axis[i].enabled = 0;
        limit_switch_debounce[i] = 0;

        if (i == 0) STEP1_PORT->BSRR = (1 << (STEP1_PIN + 16));
        else if (i == 1) STEP2_PORT->BSRR = (1 << (STEP2_PIN + 16));
        else if (i == 2) STEP3_PORT->BSRR = (1 << (STEP3_PIN + 16));
        else if (i == 3) STEP4_PORT->BSRR = (1 << (STEP4_PIN + 16));
    }
}

void stepper_stop_axis(uint8_t id)
{
    if (id >= AXIS_COUNT) return;

    if (id == 0) STEP1_PORT->BSRR = (1 << (STEP1_PIN + 16));
    else if (id == 1) STEP2_PORT->BSRR = (1 << (STEP2_PIN + 16));
    else if (id == 2) STEP3_PORT->BSRR = (1 << (STEP3_PIN + 16));
    else if (id == 3) STEP4_PORT->BSRR = (1 << (STEP4_PIN + 16));

    axis[id].target_step = axis[id].current_step;
    axis[id].move_start_step = axis[id].current_step;
    axis[id].move_end_step = axis[id].current_step;
    axis[id].move_step_offset = 0;
    axis[id].move_total_time_ms = 0;
    axis[id].move_elapsed_time_ms = 0;
    axis[id].moving = 0;
    axis[id].homing = 0;
}

void stepper_stop_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        stepper_stop_axis(i);
    }
}

static void stop_by_limit(void)
{
    stepper_stop_all();
    trajectory_clear();
    global_motor_error = ERR_LIMIT_SWITCH_DETECTED;
}

void stepper_start_homing(uint8_t id)
{
    if (id >= AXIS_COUNT) {
        global_motor_error = ERR_INVALID_CMD;
        return;
    }
    if (!global_motor_enabled || global_motor_estop) return;

    axis[id].target_step = axis[id].current_step;
    axis[id].move_total_time_ms = 0;
    axis[id].move_elapsed_time_ms = 0;
    axis[id].moving = 1;
    axis[id].homing = 1;
    axis[id].homing_done = 0;
    limit_switch_debounce[id] = 0;

    // BSRR 사용법
    // reset: BR(y): Port x reset bit y (y = 0..15) 16~31비트 -> +16 해야함
    // set: BS(y): Port x set bit y (y= 0..15) 0~15비트


    // set이라 +16 해ㅔ야함
    if (id == 0) {
        if (home_dir[id] == DIR_POSITIVE) DIR1_PORT->BSRR = (1 << DIR1_PIN);
        else DIR1_PORT->BSRR = (1 << (DIR1_PIN + 16));
        STEP1_PORT->BSRR = (1 << (STEP1_PIN + 16));
    } else if (id == 1) {
        if (home_dir[id] == DIR_POSITIVE) DIR2_PORT->BSRR = (1 << DIR2_PIN);
        else DIR2_PORT->BSRR = (1 << (DIR2_PIN + 16));
        STEP2_PORT->BSRR = (1 << (STEP2_PIN + 16));
    } else if (id == 2) {
        if (home_dir[id] == DIR_POSITIVE) DIR3_PORT->BSRR = (1 << DIR3_PIN);
        else DIR3_PORT->BSRR = (1 << (DIR3_PIN + 16));
        STEP3_PORT->BSRR = (1 << (STEP3_PIN + 16));
    } else if (id == 3) {
        if (home_dir[id] == DIR_POSITIVE) DIR4_PORT->BSRR = (1 << DIR4_PIN);
        else DIR4_PORT->BSRR = (1 << (DIR4_PIN + 16));
        STEP4_PORT->BSRR = (1 << (STEP4_PIN + 16));
    }
}

void stepper_start_homing_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        stepper_start_homing(i);
    }
}

void stepper_10us_interrupt(void)
{
    static uint16_t homing_tick[AXIS_COUNT];

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {

        // 홈으로 갈때 읽음
        if (axis[i].homing) {
            if (limit_switch_pressed_stable(i)) {
                int32_t home_step = angle_to_step(i, get_home_angle(i));
                axis[i].current_step = home_step;
                axis[i].target_step = home_step;
                axis[i].move_start_step = home_step;
                axis[i].move_end_step = home_step;
                axis[i].move_step_offset = 0;
                axis[i].move_total_time_ms = 0;
                axis[i].move_elapsed_time_ms = 0;
                axis[i].homing = 0;
                axis[i].moving = 0;
                axis[i].homing_done = 1;

                if (i == 0) STEP1_PORT->BSRR = (1 << (STEP1_PIN + 16));
                else if (i == 1) STEP2_PORT->BSRR = (1 << (STEP2_PIN + 16));
                else if (i == 2) STEP3_PORT->BSRR = (1 << (STEP3_PIN + 16));
                else if (i == 3) STEP4_PORT->BSRR = (1 << (STEP4_PIN + 16));

                continue;
            }

            if (++homing_tick[i] >= HOMING_INTERVAL_TICKS) {
                homing_tick[i] = 0;

                if (i == 0) {
                    if (home_dir[i] == DIR_POSITIVE) DIR1_PORT->BSRR = (1 << DIR1_PIN);
                    else DIR1_PORT->BSRR = (1 << (DIR1_PIN + 16));
                    STEP1_PORT->BSRR = (1 << STEP1_PIN);
                    for (volatile int delay = 0; delay < 10; delay++) {}
                    STEP1_PORT->BSRR = (1 << (STEP1_PIN + 16));
                } else if (i == 1) {
                    if (home_dir[i] == DIR_POSITIVE) DIR2_PORT->BSRR = (1 << DIR2_PIN);
                    else DIR2_PORT->BSRR = (1 << (DIR2_PIN + 16));
                    STEP2_PORT->BSRR = (1 << STEP2_PIN);
                    for (volatile int delay = 0; delay < 10; delay++) {}
                    STEP2_PORT->BSRR = (1 << (STEP2_PIN + 16));
                } else if (i == 2) {
                    if (home_dir[i] == DIR_POSITIVE) DIR3_PORT->BSRR = (1 << DIR3_PIN);
                    else DIR3_PORT->BSRR = (1 << (DIR3_PIN + 16));
                    STEP3_PORT->BSRR = (1 << STEP3_PIN);
                    for (volatile int delay = 0; delay < 10; delay++) {}
                    STEP3_PORT->BSRR = (1 << (STEP3_PIN + 16));
                } else if (i == 3) {
                    if (home_dir[i] == DIR_POSITIVE) DIR4_PORT->BSRR = (1 << DIR4_PIN);
                    else DIR4_PORT->BSRR = (1 << (DIR4_PIN + 16));
                    STEP4_PORT->BSRR = (1 << STEP4_PIN);
                    for (volatile int delay = 0; delay < 10; delay++) {}
                    STEP4_PORT->BSRR = (1 << (STEP4_PIN + 16));
                }
            }
            continue;
        }
        
        // 움직이지 않는 상태면 다음 축으로
        if (!axis[i].moving) continue;

        // 리미트 닿으면 멈춤
        if (limit_switch_pressed(i)) {
            stop_by_limit();
            return;
        }


        //=== 모터 1스탭씩 이동 ===
        // 현재 위치가 목표보다 작으면
        if (axis[i].current_step < axis[i].target_step) {
            if (i == 0) {
                DIR1_PORT->BSRR = (1 << DIR1_PIN);
                STEP1_PORT->BSRR = (1 << STEP1_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP1_PORT->BSRR = (1 << (STEP1_PIN + 16));
            } else if (i == 1) {
                DIR2_PORT->BSRR = (1 << DIR2_PIN);
                STEP2_PORT->BSRR = (1 << STEP2_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP2_PORT->BSRR = (1 << (STEP2_PIN + 16));
            } else if (i == 2) {
                DIR3_PORT->BSRR = (1 << DIR3_PIN);
                STEP3_PORT->BSRR = (1 << STEP3_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP3_PORT->BSRR = (1 << (STEP3_PIN + 16));
            } else if (i == 3) {
                DIR4_PORT->BSRR = (1 << DIR4_PIN);
                STEP4_PORT->BSRR = (1 << STEP4_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP4_PORT->BSRR = (1 << (STEP4_PIN + 16));
            }
            axis[i].current_step++;
        
        // 현재 위치가 목표보다 크면
        } else if (axis[i].current_step > axis[i].target_step) {
            if (i == 0) {
                DIR1_PORT->BSRR = (1 << (DIR1_PIN + 16));
                STEP1_PORT->BSRR = (1 << STEP1_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP1_PORT->BSRR = (1 << (STEP1_PIN + 16));
            } else if (i == 1) {
                DIR2_PORT->BSRR = (1 << (DIR2_PIN + 16));
                STEP2_PORT->BSRR = (1 << STEP2_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP2_PORT->BSRR = (1 << (STEP2_PIN + 16));
            } else if (i == 2) {
                DIR3_PORT->BSRR = (1 << (DIR3_PIN + 16));
                STEP3_PORT->BSRR = (1 << STEP3_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP3_PORT->BSRR = (1 << (STEP3_PIN + 16));
            } else if (i == 3) {
                DIR4_PORT->BSRR = (1 << (DIR4_PIN + 16));
                STEP4_PORT->BSRR = (1 << STEP4_PIN);
                for (volatile int delay = 0; delay < 10; delay++) {}
                STEP4_PORT->BSRR = (1 << (STEP4_PIN + 16));
            }
            axis[i].current_step--;
        }
    }
}
