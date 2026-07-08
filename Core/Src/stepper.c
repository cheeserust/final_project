#include "../Inc/stepper.h"
#include "../Inc/gpio.h"
#include "../Inc/trajectory.h"




static uint16_t limit_switch_debounce_count[AXIS_COUNT];
static uint16_t homing_tick[AXIS_COUNT];
static uint16_t step_wait_10us[AXIS_COUNT];
static uint32_t axis_dda_accumulator[AXIS_COUNT];
static uint32_t total_move_ticks;

static volatile uint8_t step_high_flag[AXIS_COUNT];
static volatile uint8_t g_limit_switch_bitmask = 0;
static volatile uint8_t g_homing_step_request_bits = 0;
static uint32_t g_axis_total_steps[AXIS_COUNT];


#if BOARD_ID == BOARD_ID_BOARD1
static const int8_t home_dir[AXIS_COUNT] = { DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE };
#elif BOARD_ID == BOARD_ID_BOARD2
static const int8_t home_dir[AXIS_COUNT] = { DIR_POSITIVE };
#else
#error "stepper.c: BOARD_ID가 정의되지 않았거나 지원하지 않는 값입니다. home_dir[]을 채울 수 없습니다."
#endif


//리밋스위치 읽기
static uint8_t limit_switch_pressed_raw(uint8_t id)
{
    uint8_t value = 1;

    if (id == 0) {
        value = (uint8_t)((LIM1_PORT->IDR & (1u << LIM1_PIN)) != 0);
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        value = (uint8_t)((LIM2_PORT->IDR & (1u << LIM2_PIN)) != 0);
    } else if (id == 2) {
        value = (uint8_t)((LIM3_PORT->IDR & (1u << LIM3_PIN)) != 0);
    } else if (id == 3) {
        value = (uint8_t)((LIM4_PORT->IDR & (1u << LIM4_PIN)) != 0);
    }
#endif

    return (value == LIMIT_SWITCH_ACTIVE_HIGH) ? 1 : 0;
}

/* 몇 틱 연속으로 눌려있어야 "진짜로 눌린 것"으로 인정 (채터링 방지) */
static uint8_t limit_switch_debouncing_filtering(uint8_t id)
{
    if (id >= AXIS_COUNT) return 0;

    if (limit_switch_pressed_raw(id)) {
        if (limit_switch_debounce_count[id] < LIMIT_SWITCH_DEBOUNCE_TICKS) {
            limit_switch_debounce_count[id]++;
        }
    } else {
        limit_switch_debounce_count[id] = 0;
    }

    return (limit_switch_debounce_count[id] >= LIMIT_SWITCH_DEBOUNCE_TICKS) ? 1 : 0;
}

/* 외부(상위 상태머신 등)에서 "지금 이 순간 어떤 리밋스위치가 눌려있는지"
 * 비트로 한번에 조회할 때 쓰는 함수. 이건 "외부에 보여주는 결과값"이라
 * 비트로 반환하는 게 자연스러워서 그대로 둔다 (내부 로직용 비트마스크와는 다름). */
uint8_t stepper_limit_switch_status_bits(void)
{
    uint8_t bits = 0;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (limit_switch_pressed_raw(i)) {
            bits |= (uint8_t)(1u << i);
        }
    }

    return bits;
}





static void clear_step_high_flag(void)
{
    if (step_high_flag[0]) {
        STEP1_PORT->BSRR = (1u << (STEP1_PIN + 16));
        step_high_flag[0] = 0;
    }
#if AXIS_COUNT > 1
    if (step_high_flag[1]) {
        STEP2_PORT->BSRR = (1u << (STEP2_PIN + 16));
        step_high_flag[1] = 0;
    }
    if (step_high_flag[2]) {
        STEP3_PORT->BSRR = (1u << (STEP3_PIN + 16));
        step_high_flag[2] = 0;
    }
    if (step_high_flag[3]) {
        STEP4_PORT->BSRR = (1u << (STEP4_PIN + 16));
        step_high_flag[3] = 0;
    }
#endif
}

//DIR 핀 방향 설정
static void set_dir(uint8_t id, int8_t dir)
{
    uint8_t positive = (dir > 0) ? 1 : 0;

    if (id == 0) {
        if (positive) DIR1_PORT->BSRR = (1u << DIR1_PIN);
        else          DIR1_PORT->BSRR = (1u << (DIR1_PIN + 16));
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        if (positive) DIR2_PORT->BSRR = (1u << (DIR2_PIN + 16));
        else          DIR2_PORT->BSRR = (1u << DIR2_PIN);
    } else if (id == 2) {
        if (positive) DIR3_PORT->BSRR = (1u << (DIR3_PIN + 16));
        else          DIR3_PORT->BSRR = (1u << DIR3_PIN);
    } else if (id == 3) {
        if (positive) DIR4_PORT->BSRR = (1u << DIR4_PIN);
        else          DIR4_PORT->BSRR = (1u << (DIR4_PIN + 16));
    }
#endif
}


static void step_pin_high(uint8_t id)
{
    if (id == 0) {
        STEP1_PORT->BSRR = (1u << STEP1_PIN);
        step_high_flag[0] = 1;
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        STEP2_PORT->BSRR = (1u << STEP2_PIN);
        step_high_flag[1] = 1;
    } else if (id == 2) {
        STEP3_PORT->BSRR = (1u << STEP3_PIN);
        step_high_flag[2] = 1;
    } else if (id == 3) {
        STEP4_PORT->BSRR = (1u << STEP4_PIN);
        step_high_flag[3] = 1;
    }
#endif
}

//GPIO 저수준 제어 (STEP DIR 핀 켜고 끄기)
static void step_pin_low(uint8_t id)
{
    if (id == 0) {
        STEP1_PORT->BSRR = (1u << (STEP1_PIN + 16));
        step_high_flag[0] = 0;
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        STEP2_PORT->BSRR = (1u << (STEP2_PIN + 16));
        step_high_flag[1] = 0;
    } else if (id == 2) {
        STEP3_PORT->BSRR = (1u << (STEP3_PIN + 16));
        step_high_flag[2] = 0;
    } else if (id == 3) {
        STEP4_PORT->BSRR = (1u << (STEP4_PIN + 16));
        step_high_flag[3] = 0;
    }
#endif
}


// ============================
// 외부에서

void stepper_prepare_motion(uint16_t duration_ms)
{
    uint32_t limited_duration_ms = duration_ms;

    if (limited_duration_ms == 0) limited_duration_ms = 1;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis_dda_accumulator[i] = 0;
        step_wait_10us[i] = 0;

        if (g_target_step[i] >= g_motion_start_step[i]) {
            g_axis_total_steps[i] = (uint32_t)(g_target_step[i] - g_motion_start_step[i]);
        } else {
            g_axis_total_steps[i] = (uint32_t)(g_motion_start_step[i] - g_target_step[i]);
        }

        if (g_axis_total_steps[i] > 0 && MOTION_MAX_STEP_RATE_SPS > 0) {
            uint32_t required_ms;

            required_ms = (g_axis_total_steps[i] * 1000u + MOTION_MAX_STEP_RATE_SPS - 1u) /
                          MOTION_MAX_STEP_RATE_SPS;
            if (required_ms > limited_duration_ms) {
                limited_duration_ms = required_ms;
            }
        }
    }

    total_move_ticks = limited_duration_ms * 100u;
    if (total_move_ticks == 0) total_move_ticks = 1;
}

void stepper_cancel_motion(void)
{
    total_move_ticks = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis_dda_accumulator[i] = 0;
        step_wait_10us[i] = 0;
    }
}

//
/* =========================================================================
 * 초기화 / 정지
 * ========================================================================= */

void stepper_init(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_current_step[i] = 0;
        g_target_step[i] = 0;
        g_motion_start_step[i] = 0;
        limit_switch_debounce_count[i] = 0;
        homing_tick[i] = 0;
        axis_dda_accumulator[i] = 0;
        step_wait_10us[i] = 0;
        step_pin_low(i);
    }
    total_move_ticks = 0;
}

void stepper_stop_axis(uint8_t id)
{
    if (id >= AXIS_COUNT) return;

    step_pin_low(id);
    axis_dda_accumulator[id] = 0;
    step_wait_10us[id] = 0;
    g_target_step[id] = g_current_step[id];
    g_motion_start_step[id] = g_current_step[id];
}

void stepper_stop_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        stepper_stop_axis(i);
    }
    total_move_ticks = 0;
}

// =======
// 홈
// ======

// 홈 1축만
void stepper_start_homing(uint8_t id)
{
    if (id >= AXIS_COUNT) {
        g_error_code = ERR_INVALID_CMD;
        g_state = STATE_ERROR;
        return;
    }
    if (ESTOP_ACTIVE() || !g_enabled || g_error_code != ERR_NONE) return;

    g_homing_active = 1;
    g_homing_done_bits &= (uint8_t)~(1u << id);
    g_state = STATE_HOMING;
    limit_switch_debounce_count[id] = 0;
    homing_tick[id] = 0;
    step_wait_10us[id] = 0;
    g_target_step[id] = g_current_step[id];
    step_pin_low(id);
    set_dir(id, home_dir[id]);
}

// 모든 축 홈
void stepper_start_homing_all(void)
{
    if (ESTOP_ACTIVE() || !g_enabled || g_error_code != ERR_NONE) return;

    g_homing_active = 1;
    g_homing_done_bits = 0;
    g_motion_active = 0;
    stepper_cancel_motion();
    g_state = STATE_HOMING;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        limit_switch_debounce_count[i] = 0;
        homing_tick[i] = 0;
        step_wait_10us[i] = 0;
        g_target_step[i] = g_current_step[i];
        step_pin_low(i);
        set_dir(i, home_dir[i]);
    }
}

void stepper_homing_1ms(void)
{
    if (!g_homing_active) return; 

    if (ESTOP_ACTIVE() || !g_enabled || g_error_code != ERR_NONE) {
        stepper_stop_all();
        g_homing_active = 0;
        return;
    }

    // 기존 10us 틱 기준을 1ms 기준으로 변환
    const uint16_t HOMING_INTERVAL_MS = (HOMING_INTERVAL_TICKS / 100) > 0 ? (HOMING_INTERVAL_TICKS / 100) : 1;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (g_homing_done_bits & (uint8_t)(1 << i)) continue;

        // 리밋 스위치 체크
        if (g_limit_switch_bitmask & (1 << i)) {
            int32_t home_step = angle_to_step(i, get_home_angle(i));
            g_current_step[i] = home_step;
            g_target_step[i] = home_step;
            g_motion_start_step[i] = home_step;
            g_homing_done_bits |= (uint8_t)(1 << i);
            step_pin_low(i); 
            continue;
        }

        // 이동 주기 도달 시
        if (++homing_tick[i] >= HOMING_INTERVAL_MS) {
            homing_tick[i] = 0;
            
            // ★ [변경] 직접 핀을 켜지 않고, 10us 방에 "스텝 쏴줘!" 라고 플래그 세우기
            g_homing_step_request_bits |= (uint8_t)(1 << i); 
            
            // 위치 카운트는 여기서 미리 진행
            if (home_dir[i] > 0) g_current_step[i]++;
            else                 g_current_step[i]--;
        }
    }

    if (system_all_homed()) {
        g_homing_active = 0;
        g_state = STATE_IDLE;
    }
}



// 리미트스위치용 인터럽트
void stepper_1ms_interrupt(void)
{
    uint8_t bitmask = 0;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (limit_switch_debouncing_filtering(i)) {
            bitmask |= (1 << i); // i번째 축 리밋 스위치 눌림 감지
        }
    }

    // 결과를 전역 변수에 반영
    g_limit_switch_bitmask = bitmask;
}

/* =========================================================================
 * 메인 진입점: 10us 타이머 인터럽트에서 호출됨
 * ========================================================================= */
void stepper_10us_interrupt(void)
{
    // 1. 이전 스텝 핀 클리어
    clear_step_high_flag();

    // 2. 시스템 상태 및 에러 체크
    if (ESTOP_ACTIVE() || !g_enabled || g_error_code != ERR_NONE) {
        return;
    }

    // 3. 홈으로 가는 플래그 처리
    if (g_homing_active) {
        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            // 1ms 방에서 이 축에 스텝을 쏘라고 요청했는지 확인
            if (g_homing_step_request_bits & (uint8_t)(1 << i)) {
                
                // 1. 요청을 확인했으니 플래그를 즉시 클리어 (중복 처리 방지)
                g_homing_step_request_bits &= (uint8_t)~(1 << i);
                
                // 2. 정확한 하드웨어 타이밍으로 신호 출력
                set_dir(i, home_dir[i]);
                step_pin_high(i); 
            }
        }
        return; // 홈잉 중에는 아래의 일반 모션 로직을 타지 않음
    }

    // 3. 모션 구동 중이 아니라면 종료
    if (!g_motion_active || total_move_ticks == 0) return;

    // 4. 각 축별 스텝 생성
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        uint32_t axis_delta_steps;
        int8_t dir;

        // 목표에 도달하면
        if (g_current_step[i] == g_target_step[i]) {
            axis_dda_accumulator[i] = 0;
            continue;
        }

        // 탈조 안나게 대기시간
        if (step_wait_10us[i] > 0) {
            step_wait_10us[i]--;
        }

        axis_delta_steps = g_axis_total_steps[i];

        // DDA 알고리즘: 먼저 스텝을 모은 후 step_accumulation이 move_duration_10us보다 커져야지 실행됨
        axis_dda_accumulator[i] += axis_delta_steps;
        if (axis_dda_accumulator[i] < total_move_ticks || step_wait_10us[i] > 0) {
            continue;
        }
        axis_dda_accumulator[i] -= total_move_ticks;

        // 방향
        dir = (g_current_step[i] < g_target_step[i]) ? DIR_POSITIVE : DIR_NEGATIVE;

        // 홈 방향으로 갈떄 리미트 스위치 누르면 멈추기
        if (dir == home_dir[i] && (g_limit_switch_bitmask & (1 << i))) {
            trajectory_clear();
            g_homing_active = 0;
            g_error_code = ERR_LIMIT_SWITCH_DETECTED;
            g_state = STATE_ERROR;
            return;
        }

        // 스텝 출력
        set_dir(i, dir);
        step_pin_high(i);
        if (dir > 0) g_current_step[i]++;
        else         g_current_step[i]--;

        // 최소 MIN_STEP_INTERVAL_TICKS 만큼은 쉬어야함 더 빠르면 탈조
        step_wait_10us[i] = MIN_STEP_INTERVAL_TICKS;
    }
}
