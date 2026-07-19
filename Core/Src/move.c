#include "../Inc/move.h"
#include "../Inc/stepper.h"

// CAN으로 받은 축별 목표를 모아서 이동을 시작하고 끝내는 파일
typedef struct {
    uint8_t goal_id;
    uint8_t received_axis_mask;
    uint16_t duration_ms;
    uint32_t staging_start_ms;
    uint32_t motion_start_ms;
    uint8_t has_motion;
    int32_t target_step[AXIS_COUNT];
} GoalSlot;

#define ALL_AXIS_MASK ((uint8_t)((1 << AXIS_COUNT) - 1))

// 각도 단위는 0.01도
#if BOARD_ID == BOARD_ID_BOARD1
static const int32_t gear_ratio[AXIS_COUNT] = {20, 50, 30, 20};
static const int32_t motor_steps_per_rev[AXIS_COUNT] = {200, 200, 200, 200};
static const int32_t min_angle[AXIS_COUNT] = {-8650, -7810, -9150, -9000};
static const int32_t max_angle[AXIS_COUNT] = {9000, 8000, 9000, 18000};
static const int32_t home_angle[AXIS_COUNT] = {-8650, -7810, -9150, -9000};
#else
static const int32_t gear_ratio[AXIS_COUNT] = {20};
static const int32_t motor_steps_per_rev[AXIS_COUNT] = {200};
static const int32_t min_angle[AXIS_COUNT] = {-9000};
static const int32_t max_angle[AXIS_COUNT] = {18000};
static const int32_t home_angle[AXIS_COUNT] = {-9000};
#endif

static GoalSlot g_goal;
static uint8_t g_cancelled_goal_valid;
static uint8_t g_cancelled_goal_id;
static uint8_t g_completed_goal_valid;
static uint8_t g_completed_goal_id;

// 현재 모아 둔 목표를 비움
static void clear_goal_slot(void)
{
    g_goal.goal_id = 0;
    g_goal.received_axis_mask = 0;
    g_goal.duration_ms = 0;
    g_goal.staging_start_ms = 0;
    g_goal.motion_start_ms = 0;
    g_goal.has_motion = 0;
}

// 현재 위치에서 이동을 멈춤
static void stop_motion(void)
{
    g_motion_active = 0;
    stepper_cancel_motion();
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_target_step[i] = g_current_step[i];
        g_motion_start_step[i] = g_current_step[i];
    }
}

void move_clear(void)
{
    clear_goal_slot();
    stop_motion();
}

uint8_t move_goal_slot_free(void)
{
    return g_goal.received_axis_mask == 0;
}

uint16_t move_goal_duration_ms(void)
{
    return g_goal.duration_ms;
}

uint8_t move_goal_mask(void)
{
    return g_goal.received_axis_mask;
}

// 각도와 스텝 값 변환
int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw)
{
    int64_t value;
    value = (int64_t)angle_raw * gear_ratio[axis_id] *
            motor_steps_per_rev[axis_id] * MICROSTEP;
    return (int32_t)(value / 36000);
}

int32_t step_to_angle(uint8_t axis_id, int32_t step)
{
    int64_t value;
    int64_t steps_per_rev;
    value = (int64_t)step * 36000;
    steps_per_rev = (int64_t)gear_ratio[axis_id] *
                    motor_steps_per_rev[axis_id] * MICROSTEP;
    value += value >= 0 ? steps_per_rev / 2 : -(steps_per_rev / 2);
    return (int32_t)(value / steps_per_rev);
}

int32_t get_home_angle(uint8_t axis_id)
{
    return home_angle[axis_id];
}

// 받은 목표를 스텝으로 바꾸고 축 범위를 확인
uint8_t move_resolve_target_step(uint8_t axis_id, int32_t target_raw,
                                uint8_t relative, uint8_t step_mode,
                                int32_t *target_step)
{
    int64_t resolved;
    int32_t minimum;
    int32_t maximum;
    resolved = step_mode ? target_raw : angle_to_step(axis_id, target_raw);
    if (relative) resolved += g_current_step[axis_id];
    if (resolved < INT32_MIN || resolved > INT32_MAX) return 0;
    *target_step = (int32_t)resolved;

    minimum = angle_to_step(axis_id, min_angle[axis_id]);
    maximum = angle_to_step(axis_id, max_angle[axis_id]);
    return (*target_step >= minimum && *target_step <= maximum) ? 1 : 0;
}

// 같은 Goal ID의 축별 목표를 하나씩 저장
uint8_t move_stage_goal_axis(uint8_t motor_id, int32_t target_step,
                             uint8_t goal_id, uint16_t duration_ms,
                             uint8_t *received_axis_mask)
{
    uint8_t motor_bit;
    *received_axis_mask = 0;
    if (g_motion_active) return GOAL_STAGE_BUSY;

    if (g_goal.received_axis_mask == 0) {
        if (g_cancelled_goal_valid && goal_id == g_cancelled_goal_id) {
            return GOAL_STAGE_INVALID;
        }
        if (g_completed_goal_valid && goal_id == g_completed_goal_id) {
            *received_axis_mask = ALL_AXIS_MASK;
            return GOAL_STAGE_DUPLICATE;
        }
        clear_goal_slot();
        g_goal.goal_id = goal_id;
        g_goal.duration_ms = duration_ms;
        g_goal.staging_start_ms = global_tick_ms;
    } else if (g_goal.goal_id != goal_id) {
        return GOAL_STAGE_BUSY;
    } else if (g_goal.duration_ms != duration_ms) {
        clear_goal_slot();
        return GOAL_STAGE_INVALID;
    }

    motor_bit = (uint8_t)(1 << motor_id);
    if (g_goal.received_axis_mask & motor_bit) {
        *received_axis_mask = g_goal.received_axis_mask;
        if (g_goal.target_step[motor_id] == target_step) return GOAL_STAGE_DUPLICATE;
        clear_goal_slot();
        return GOAL_STAGE_INVALID;
    }

    g_goal.target_step[motor_id] = target_step;
    g_goal.received_axis_mask |= motor_bit;
    *received_axis_mask = g_goal.received_axis_mask;

    if (g_goal.received_axis_mask != ALL_AXIS_MASK) {
        return GOAL_STAGE_WAITING;
    }

    return GOAL_STAGE_READY;
}

// 모든 축 목표가 모이면 이동 시작
uint8_t move_start_goal(uint8_t goal_id)
{
    if (g_motion_active || g_goal.received_axis_mask != ALL_AXIS_MASK ||
        g_goal.goal_id != goal_id) return 0;

    g_goal.has_motion = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_motion_start_step[i] = g_current_step[i];
        g_target_step[i] = g_goal.target_step[i];
        if (g_target_step[i] != g_current_step[i]) g_goal.has_motion = 1;
    }
    g_goal.motion_start_ms = global_tick_ms;
    stepper_prepare_motion(g_goal.duration_ms);
    g_motion_active = 1;
    g_state = STATE_MOVING;
    return 1;
}

// 대기 중이거나 실행 중인 이동 취소
uint8_t move_cancel_goal(uint8_t goal_id)
{
    if (g_goal.received_axis_mask != 0 && g_goal.goal_id != goal_id) return 0;
    stop_motion();
    clear_goal_slot();
    g_cancelled_goal_valid = 1;
    g_cancelled_goal_id = goal_id;
    if (g_enabled && !ESTOP_ACTIVE() && g_error_code == ERR_NONE) g_state = STATE_IDLE;
    return 1;
}

// 축별 목표가 중간에 끊겼는지 확인
uint8_t move_check_staging_timeout(uint8_t *goal_id, uint8_t *mask,
                                   uint16_t *duration_ms)
{
    if (g_goal.received_axis_mask == 0 ||
        g_goal.received_axis_mask == ALL_AXIS_MASK || g_motion_active) return 0;
    if ((global_tick_ms - g_goal.staging_start_ms) <= STAGING_TIMEOUT_MS) return 0;

    *goal_id = g_goal.goal_id;
    *mask = g_goal.received_axis_mask;
    *duration_ms = g_goal.duration_ms;
    clear_goal_slot();
    return 1;
}

// 모든 축이 목표에 도착하면 이동 종료
void move_check_completion_1ms(void)
{
    uint8_t reached = 1;
    if (!g_motion_active) return;

    if (ESTOP_ACTIVE()) {
        stop_motion();
        return;
    }

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (g_current_step[i] != g_goal.target_step[i]) {
            reached = 0;
            break;
        }
    }
    if (!reached) return;
    if (!g_goal.has_motion &&
        (global_tick_ms - g_goal.motion_start_ms) < g_goal.duration_ms) return;

    g_completed_goal_valid = 1;
    g_completed_goal_id = g_goal.goal_id;
    g_motion_active = 0;
    stepper_cancel_motion();
    clear_goal_slot();
    g_state = STATE_IDLE;
}
