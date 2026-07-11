#include "../Inc/trajectory.h"
#include "../Inc/stepper.h"

typedef struct {
    uint8_t active;
    uint8_t expected_motor_id;
    uint8_t duration_5ms;
    uint32_t start_ms;
    TrajectoryPoint point;
} PendingTrajectoryPoint;

typedef struct {
    TrajectoryPoint points[TRAJECTORY_POINT_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} TrajectoryPointRingQueue;

#if BOARD_ID == BOARD_ID_BOARD1
static const int32_t gear_ratio[AXIS_COUNT] = { 20, 50, 30, 20 };
static const int32_t motor_steps_per_rev[AXIS_COUNT] = { 200, 200, 200, 200 };
static const int32_t min_angle[AXIS_COUNT] = { -8500, -7810, -9150, -9000 };
static const int32_t max_angle[AXIS_COUNT] = { 9000, 8000, 9000, 18000 };
static const int32_t home_angle[AXIS_COUNT] = { -8650, -7810, -9150, -9000 };
#elif BOARD_ID == BOARD_ID_BOARD2
static const int32_t gear_ratio[AXIS_COUNT] = { 20 };
static const int32_t motor_steps_per_rev[AXIS_COUNT] = { 200 };
static const int32_t min_angle[AXIS_COUNT] = { -9000 };
static const int32_t max_angle[AXIS_COUNT] = { 18000 };
static const int32_t home_angle[AXIS_COUNT] = { -9000 };
#endif

static PendingTrajectoryPoint g_pending_trajectory_point; // 다 모이면 queue에 push, 축별 명령을 하나의 TrajectoryPoint로 조립
static TrajectoryPointRingQueue g_trajectory_point_ring_queue; //실행 대기 원형 큐,  완성된 TrajectoryPoint들을 실행 순서대로 저장
static TrajectoryPoint g_current_trajectory_point; // 현재 실행 중인 명령, 지금 실행 중인 TrajectoryPoint를 보관
static volatile uint8_t g_queue_overflow_clear_request;
static volatile uint8_t g_queue_overflow_clear_ack;
static volatile int32_t g_planned_step[AXIS_COUNT];

static uint8_t trajectory_point_queue_push(const TrajectoryPoint *point)
{
    uint8_t tail = g_trajectory_point_ring_queue.tail;
    uint8_t next_tail = (uint8_t)((tail + 1) % TRAJECTORY_POINT_QUEUE_SIZE);

    if (next_tail == g_trajectory_point_ring_queue.head) return 0;

    g_trajectory_point_ring_queue.points[tail] = *point;
    g_trajectory_point_ring_queue.tail = next_tail;
    return 1;
}

static uint8_t trajectory_point_queue_pop(TrajectoryPoint *point)
{
    uint8_t head = g_trajectory_point_ring_queue.head;

    if (head == g_trajectory_point_ring_queue.tail) return 0;

    *point = g_trajectory_point_ring_queue.points[head];
    g_trajectory_point_ring_queue.head = (uint8_t)((head + 1) % TRAJECTORY_POINT_QUEUE_SIZE);
    return 1;
}

static void clear_trajectory_point(TrajectoryPoint *point)
{
    point->duration_ms = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        point->target_step[i] = 0;
        point->speed[i] = 0;
    }
}

static void reset_pending_trajectory_point(void)
{
    g_pending_trajectory_point.active = 0;
    g_pending_trajectory_point.expected_motor_id = 0;
    g_pending_trajectory_point.duration_5ms = 0;
    g_pending_trajectory_point.start_ms = 0;
    clear_trajectory_point(&g_pending_trajectory_point.point);
}

// 외부에서 사용할 함수
void trajectory_cancel_staging(void)
{
    reset_pending_trajectory_point();
}

void trajectory_stop_motion(void)
{
    g_motion_active = 0;
    stepper_cancel_motion();
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_target_step[i] = g_current_step[i];
        g_motion_start_step[i] = g_current_step[i];
    }
}

void trajectory_sync_planned_to_current(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_planned_step[i] = g_current_step[i];
    }
}

int32_t trajectory_get_planned_step(uint8_t axis_id)
{
    if (axis_id >= AXIS_COUNT) return 0;
    return g_planned_step[axis_id];
}

void trajectory_clear(void)
{
    g_trajectory_point_ring_queue.head = 0;
    g_trajectory_point_ring_queue.tail = 0;
    reset_pending_trajectory_point();
    trajectory_stop_motion();
    trajectory_sync_planned_to_current();
}

void trajectory_request_queue_overflow_clear(void)
{
    g_queue_overflow_clear_ack = 0;
    g_queue_overflow_clear_request = 1;
}

void trajectory_cancel_queue_overflow_clear(void)
{
    g_queue_overflow_clear_request = 0;
    g_queue_overflow_clear_ack = 0;
}

uint8_t trajectory_take_queue_overflow_clear_ack(void)
{
    if (!g_queue_overflow_clear_ack) return 0;
    g_queue_overflow_clear_ack = 0;
    return 1;
}

static void trajectory_service_queue_overflow_clear_request(void)
{
    if (!g_queue_overflow_clear_request || !g_queue_overflow) return;
    if (g_motion_active || g_pending_trajectory_point.active) return;
    if (g_trajectory_point_ring_queue.head != g_trajectory_point_ring_queue.tail) return;

    g_queue_overflow = 0;
    g_queue_overflow_clear_request = 0;
    g_queue_overflow_clear_ack = 1;
}

uint8_t get_free_axis_command_count(void)
{
    uint8_t head = g_trajectory_point_ring_queue.head;
    uint8_t tail = g_trajectory_point_ring_queue.tail;
    uint8_t used_points;
    uint8_t free_points;

    if (tail >= head) used_points = (uint8_t)(tail - head);
    else used_points = (uint8_t)(TRAJECTORY_POINT_QUEUE_SIZE - head + tail);

    free_points = (uint8_t)((TRAJECTORY_POINT_QUEUE_SIZE - 1) - used_points);
    return (uint8_t)(free_points * BOARD_STAGING_FRAME_COUNT);
}

int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw)
{
    int64_t step_value;

    if (axis_id >= AXIS_COUNT) return 0;
    step_value = (int64_t)angle_raw *
                 gear_ratio[axis_id] *
                 motor_steps_per_rev[axis_id] *
                 MICROSTEP;
    return (int32_t)(step_value / 36000);
}

int32_t step_to_angle(uint8_t axis_id, int32_t step)
{
    int64_t angle_value;
    int64_t steps_per_output_rev;

    if (axis_id >= AXIS_COUNT) return 0;

    angle_value = (int64_t)step * 36000;
    steps_per_output_rev = (int64_t)gear_ratio[axis_id] *
                           motor_steps_per_rev[axis_id] *
                           MICROSTEP;

    if (angle_value >= 0) angle_value += steps_per_output_rev / 2;
    else angle_value -= steps_per_output_rev / 2;

    return (int32_t)(angle_value / steps_per_output_rev);
}

int32_t get_home_angle(uint8_t axis_id)
{
    if (axis_id >= AXIS_COUNT) return 0;
    return home_angle[axis_id];
}

uint8_t trajectory_resolve_target_step(uint8_t axis_id,
                                       int32_t target_raw,
                                       uint8_t relative,
                                       uint8_t step_mode,
                                       int32_t *target_step)
{
    int64_t resolved;
    int32_t min_step;
    int32_t max_step;

    if (axis_id >= AXIS_COUNT || target_step == 0) return 0;

    resolved = step_mode ? target_raw : angle_to_step(axis_id, target_raw);
    if (relative) resolved += g_planned_step[axis_id];
    if (resolved < INT32_MIN || resolved > INT32_MAX) return 0;

    *target_step = (int32_t)resolved;

    min_step = angle_to_step(axis_id, min_angle[axis_id]);
    max_step = angle_to_step(axis_id, max_angle[axis_id]);
    if (min_step > max_step) {
        int32_t tmp = min_step;
        min_step = max_step;
        max_step = tmp;
    }

    if (*target_step < min_step) return 0;
    if (*target_step > max_step) return 0;
    return 1;
}

uint8_t trajectory_handle_staging_timeout(void)
{
    if (!g_pending_trajectory_point.active) return 0;
    if ((global_tick_ms - g_pending_trajectory_point.start_ms) <= STAGING_TIMEOUT_MS) return 0;

    trajectory_clear();
    g_error_code = ERR_INVALID_CMD;
    g_state = STATE_ERROR;
    return 1;
}

static uint16_t duration_5ms_to_ms(uint8_t duration_5ms)
{
    uint16_t duration_ms = (uint16_t)duration_5ms * 5;
    return duration_ms == 0 ? 1 : duration_ms;
}

uint8_t trajectory_add_axis_command(uint8_t motor_id, int32_t target_step, uint16_t speed, uint8_t duration_5ms)
{
    if (trajectory_handle_staging_timeout()) return TRAJECTORY_STAGING_INVALID;
    if (motor_id >= AXIS_COUNT) {
        reset_pending_trajectory_point();
        return TRAJECTORY_STAGING_INVALID;
    }

    if (!g_pending_trajectory_point.active) {
        if (motor_id != 0) return TRAJECTORY_STAGING_INVALID;

        reset_pending_trajectory_point();
        g_pending_trajectory_point.active = 1;
        g_pending_trajectory_point.expected_motor_id = 0;
        g_pending_trajectory_point.duration_5ms = duration_5ms;
        g_pending_trajectory_point.start_ms = global_tick_ms;
        g_pending_trajectory_point.point.duration_ms = duration_5ms_to_ms(duration_5ms);
    } else {
        if (motor_id != g_pending_trajectory_point.expected_motor_id) {
            reset_pending_trajectory_point();
            return TRAJECTORY_STAGING_INVALID;
        }
        if (duration_5ms != g_pending_trajectory_point.duration_5ms) {
            reset_pending_trajectory_point();
            return TRAJECTORY_STAGING_INVALID;
        }
    }

    g_pending_trajectory_point.point.target_step[motor_id] = target_step;
    g_pending_trajectory_point.point.speed[motor_id] = speed;
    g_pending_trajectory_point.expected_motor_id++;

    if (g_pending_trajectory_point.expected_motor_id < BOARD_STAGING_FRAME_COUNT) return TRAJECTORY_STAGING_WAITING;

    if (!trajectory_point_queue_push(&g_pending_trajectory_point.point)) {
        reset_pending_trajectory_point();
        return TRAJECTORY_STAGING_QUEUE_FULL;
    }

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_planned_step[i] = g_pending_trajectory_point.point.target_step[i];
    }

    reset_pending_trajectory_point();
    return TRAJECTORY_STAGING_COMMITTED;
}

// ========================
// 1ms 인터럽트
// ==========================

void trajectory_1ms_interrupt(void)
{
    TrajectoryPoint point;
    uint8_t reached;

    // Queue consumer인 TIM3만 상태를 판정하고 clear
    trajectory_service_queue_overflow_clear_request();

    // 1. 모션 허용 상태 체크
    if (!g_enabled || ESTOP_ACTIVE() || g_error_code != ERR_NONE ||  g_homing_active || !system_all_homed()) {
        if (g_motion_active) {
            trajectory_stop_motion();
        }
        return;
    }

    // 2. 모션이 비활성화 상태라면 즉시 큐에서 다음 명령 인출
    if (!g_motion_active) {
        if (trajectory_point_queue_pop(&point)) {
            g_current_trajectory_point = point;

            for (uint8_t i = 0; i < AXIS_COUNT; i++) {
                g_motion_start_step[i] = g_current_step[i];
                g_target_step[i] = point.target_step[i];
            }

            stepper_prepare_motion(point.duration_ms == 0 ? 1 : point.duration_ms);
            g_motion_active = 1;
            g_state = STATE_MOVING;
        } else if (g_state == STATE_MOVING) {
            g_state = STATE_IDLE;
        }
    }

    // 큐에서 새 모션을 시작하지 못했다면(큐 비었음) 즉시 탈출
    if (!g_motion_active) return;


    // 3. 목표 도달 여부 체크 (새 모션 시작 직후 바로 체크 가능하도록 순서 유지)
    reached = 1;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (g_current_step[i] != g_current_trajectory_point.target_step[i]) {
            reached = 0;
            break; // 한 축이라도 덜 갔으면 즉시 탈출
        }
    }

    // 4. 도달 완료 처리
    if (reached) {
        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            g_target_step[i] = g_current_trajectory_point.target_step[i];
        }
        g_motion_active = 0;
        stepper_cancel_motion();
        if (!trajectory_point_queue_pop(&point)) {
            g_state = STATE_IDLE;
        } else {
            g_current_trajectory_point = point;

            for (uint8_t i = 0; i < AXIS_COUNT; i++) {
                g_motion_start_step[i] = g_current_step[i];
                g_target_step[i] = point.target_step[i];
            }

            stepper_prepare_motion(point.duration_ms == 0 ? 1 : point.duration_ms);
            g_motion_active = 1;
            g_state = STATE_MOVING;
        }
    }
}
