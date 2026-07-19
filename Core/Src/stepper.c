#include "../Inc/stepper.h"
#include "../Inc/gpio.h"
#include "../Inc/move.h"




static uint16_t limit_switch_debounce_count[AXIS_COUNT];
static uint16_t homing_tick[AXIS_COUNT];
static uint16_t step_wait_10us[AXIS_COUNT];

// 인터럽트에서 float 연산을 피하려고 속도와 비율을 Q16으로 저장한다.
// 저장할 때 실제 값에 65536을 곱하고, Q16끼리 곱한 뒤에는 >> 16 한다.
// motion_feed_q16은 0이면 정지, 65536이면 기준 속도의 100%이다.
static uint64_t axis_phase_q16[AXIS_COUNT];
static uint32_t axis_nominal_rate_q16[AXIS_COUNT];
static uint32_t motion_feed_q16;       // 현재 속도 비율: 0 ~ 65536
static uint32_t motion_feed_step_q16;  // 1ms마다 바뀌는 속도 비율
static int8_t axis_last_dir[AXIS_COUNT];
static uint8_t axis_dir_setup_wait[AXIS_COUNT];

static volatile uint8_t step_high_flag[AXIS_COUNT];
static volatile uint8_t g_limit_switch_bitmask = 0;
/* TIM3가 요청하고 더 높은 우선순위의 TIM2가 소비한다.
 * 축별 byte 대입을 사용해 bitmask read-modify-write 선점 경쟁을 피한다. */
static volatile uint8_t g_homing_step_request[AXIS_COUNT];
#if BOARD_ID == BOARD_ID_BOARD1
static const uint32_t max_step_rate_sps[AXIS_COUNT] = {1000, 1000, 1000, 1000};
static const uint32_t acceleration_sps2[AXIS_COUNT] = {500, 500, 500, 500};
#else
static const uint32_t max_step_rate_sps[AXIS_COUNT] = {1000};
static const uint32_t acceleration_sps2[AXIS_COUNT] = {500};
#endif


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
        // 핀이  high -> 1, low -> 0
        value = (uint8_t)((LIM1_PORT->IDR & ((uint32_t)1 << LIM1_PIN)) != 0);
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        value = (uint8_t)((LIM2_PORT->IDR & ((uint32_t)1 << LIM2_PIN)) != 0);
    } else if (id == 2) {
        value = (uint8_t)((LIM3_PORT->IDR & ((uint32_t)1 << LIM3_PIN)) != 0);
    } else if (id == 3) {
        value = (uint8_t)((LIM4_PORT->IDR & ((uint32_t)1 << LIM4_PIN)) != 0);
    }
#endif

    return (value == LIMIT_SWITCH_ACTIVE_HIGH) ? 1 : 0;
}

/* 몇 틱 연속으로 눌려있어야 "진짜로 눌린 것"으로 인정 (채터링 방지) */
static uint8_t limit_switch_debouncing_filtering(uint8_t id)
{
    if (limit_switch_pressed_raw(id)) {
        if (limit_switch_debounce_count[id] < LIMIT_SWITCH_DEBOUNCE_TICKS) {
            limit_switch_debounce_count[id]++;
        }
    } else {
        limit_switch_debounce_count[id] = 0;
    }

    return (limit_switch_debounce_count[id] >= LIMIT_SWITCH_DEBOUNCE_TICKS) ? 1 : 0;
}

uint8_t stepper_limit_switch_status_bits(void)
{
    uint8_t bits = 0;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (limit_switch_pressed_raw(i)) {
            bits |= (uint8_t)(1 << i);
        }
    }

    return bits;
}





static void clear_step_high_flag(void)
{
    if (step_high_flag[0]) {
        STEP1_PORT->BSRR = ((uint32_t)1 << (STEP1_PIN + 16));
        step_high_flag[0] = 0;
    }
#if AXIS_COUNT > 1
    if (step_high_flag[1]) {
        STEP2_PORT->BSRR = ((uint32_t)1 << (STEP2_PIN + 16));
        step_high_flag[1] = 0;
    }
    if (step_high_flag[2]) {
        STEP3_PORT->BSRR = ((uint32_t)1 << (STEP3_PIN + 16));
        step_high_flag[2] = 0;
    }
    if (step_high_flag[3]) {
        STEP4_PORT->BSRR = ((uint32_t)1 << (STEP4_PIN + 16));
        step_high_flag[3] = 0;
    }
#endif
}

//DIR 핀 방향 설정
static void set_dir(uint8_t id, int8_t dir)
{
    uint8_t positive = (dir > 0) ? 1 : 0;

    if (id == 0) {
        if (positive) DIR1_PORT->BSRR = ((uint32_t)1 << DIR1_PIN);
        else          DIR1_PORT->BSRR = ((uint32_t)1 << (DIR1_PIN + 16));
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        if (positive) DIR2_PORT->BSRR = ((uint32_t)1 << (DIR2_PIN + 16));
        else          DIR2_PORT->BSRR = ((uint32_t)1 << DIR2_PIN);
    } else if (id == 2) {
        if (positive) DIR3_PORT->BSRR = ((uint32_t)1 << (DIR3_PIN + 16));
        else          DIR3_PORT->BSRR = ((uint32_t)1 << DIR3_PIN);
    } else if (id == 3) {
        if (positive) DIR4_PORT->BSRR = ((uint32_t)1 << (DIR4_PIN + 16));
        else          DIR4_PORT->BSRR = ((uint32_t)1 << DIR4_PIN);
    }
#endif
}


static void step_pin_high(uint8_t id)
{
    if (id == 0) {
        STEP1_PORT->BSRR = ((uint32_t)1 << STEP1_PIN);
        step_high_flag[0] = 1;
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        STEP2_PORT->BSRR = ((uint32_t)1 << STEP2_PIN);
        step_high_flag[1] = 1;
    } else if (id == 2) {
        STEP3_PORT->BSRR = ((uint32_t)1 << STEP3_PIN);
        step_high_flag[2] = 1;
    } else if (id == 3) {
        STEP4_PORT->BSRR = ((uint32_t)1 << STEP4_PIN);
        step_high_flag[3] = 1;
    }
#endif
}

//STEP DIR 핀 켜고 끄기
static void step_pin_low(uint8_t id)
{
    if (id == 0) {
        STEP1_PORT->BSRR = ((uint32_t)1 << (STEP1_PIN + 16));
        step_high_flag[0] = 0;
    }
#if AXIS_COUNT > 1
    else if (id == 1) {
        STEP2_PORT->BSRR = ((uint32_t)1 << (STEP2_PIN + 16));
        step_high_flag[1] = 0;
    } else if (id == 2) {
        STEP3_PORT->BSRR = ((uint32_t)1 << (STEP3_PIN + 16));
        step_high_flag[2] = 0;
    } else if (id == 3) {
        STEP4_PORT->BSRR = ((uint32_t)1 << (STEP4_PIN + 16));
        step_high_flag[3] = 0;
    }
#endif
}


// ============================
// 외부에서

void stepper_prepare_motion(uint16_t duration_ms)
{
    uint32_t limited_duration_ms = duration_ms;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        uint32_t total_steps;

        if (g_target_step[i] >= g_motion_start_step[i]) {
            total_steps = (uint32_t)(g_target_step[i] - g_motion_start_step[i]);
        } else {
            total_steps = (uint32_t)(g_motion_start_step[i] - g_target_step[i]);
        }

        uint32_t required_ms;

        required_ms = (total_steps * 1000 + max_step_rate_sps[i] - 1) /
                      max_step_rate_sps[i];
        if (required_ms > limited_duration_ms) {
            limited_duration_ms = required_ms;
        }
    }

    motion_feed_step_q16 = MOTION_FEED_ONE_Q16;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        uint32_t total_steps = g_target_step[i] >= g_motion_start_step[i] ?
            (uint32_t)(g_target_step[i] - g_motion_start_step[i]) :
            (uint32_t)(g_motion_start_step[i] - g_target_step[i]);
        uint32_t nominal_sps;
        uint32_t feed_step;

        // 이동 거리와 시간으로 축별 기준 속도를 계산한다.
        axis_nominal_rate_q16[i] = (uint32_t)(
            ((uint64_t)total_steps * 1000 * MOTION_FEED_ONE_Q16) / limited_duration_ms);
        nominal_sps = (axis_nominal_rate_q16[i] + MOTION_FEED_ONE_Q16 - 1) >> 16;
        if (nominal_sps == 0) continue;
        feed_step = (uint32_t)(((uint64_t)acceleration_sps2[i] * MOTION_FEED_ONE_Q16 +
                               (uint64_t)nominal_sps * 1000 - 1) /
                              ((uint64_t)nominal_sps * 1000));
        if (feed_step < motion_feed_step_q16) motion_feed_step_q16 = feed_step;
    }
    motion_feed_q16 = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis_phase_q16[i] = 0;
        step_wait_10us[i] = 0;
        axis_last_dir[i] = 0;
        axis_dir_setup_wait[i] = 0;
    }
}

void stepper_cancel_motion(void)
{
    motion_feed_q16 = 0;
    motion_feed_step_q16 = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis_phase_q16[i] = 0;
        axis_nominal_rate_q16[i] = 0;
        step_wait_10us[i] = 0;
        axis_dir_setup_wait[i] = 0;
    }
}

//
//=========================================
 //초기화 / 정지
 //========================================

void stepper_init(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_current_step[i] = 0;
        g_target_step[i] = 0;
        g_motion_start_step[i] = 0;
        limit_switch_debounce_count[i] = 0;
        homing_tick[i] = 0;
        g_homing_step_request[i] = 0;
        axis_phase_q16[i] = 0;
        axis_nominal_rate_q16[i] = 0;
        axis_last_dir[i] = 0;
        axis_dir_setup_wait[i] = 0;
        step_wait_10us[i] = 0;
        step_pin_low(i);
    }
    motion_feed_q16 = 0;
    motion_feed_step_q16 = 0;
}

void stepper_stop_axis(uint8_t id)
{
    step_pin_low(id);
    g_homing_step_request[id] = 0;
    axis_phase_q16[id] = 0;
    axis_nominal_rate_q16[id] = 0;
    step_wait_10us[id] = 0;
    g_target_step[id] = g_current_step[id];
    g_motion_start_step[id] = g_current_step[id];
}

void stepper_stop_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        stepper_stop_axis(i);
    }
    motion_feed_q16 = 0;
}

// =======
// 홈
// ======

// 홈 1축만
void stepper_start_homing(uint8_t id)
{
    if (ESTOP_ACTIVE() || !g_enabled || g_error_code != ERR_NONE) return;

    g_homing_done_bits &= (uint8_t)~(1 << id);
    g_state = STATE_HOMING;
    limit_switch_debounce_count[id] = 0;
    homing_tick[id] = 0;
    g_homing_step_request[id] = 0;
    step_wait_10us[id] = 0;
    g_target_step[id] = g_current_step[id];
    step_pin_low(id);
    set_dir(id, home_dir[id]);
    g_homing_active = 1;
}

// 모든 축 홈
void stepper_start_homing_all(void)
{
    if (ESTOP_ACTIVE() || !g_enabled || g_error_code != ERR_NONE) return;

    g_homing_done_bits = 0;
    g_motion_active = 0;
    stepper_cancel_motion();
    g_state = STATE_HOMING;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        limit_switch_debounce_count[i] = 0;
        homing_tick[i] = 0;
        g_homing_step_request[i] = 0;
        step_wait_10us[i] = 0;
        g_target_step[i] = g_current_step[i];
        step_pin_low(i);
        set_dir(i, home_dir[i]);
    }
    g_homing_active = 1;
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
    const uint16_t HOMING_INTERVAL_MS = HOMING_INTERVAL_TICKS / 100;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (g_homing_done_bits & (uint8_t)(1 << i)) continue;

        // 리밋 스위치 체크
        if (g_limit_switch_bitmask & (1 << i)) {
            int32_t home_step = angle_to_step(i, get_home_angle(i));
            g_homing_step_request[i] = 0;
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

            // 실제 펄스와 위치 갱신은 10us ISR에서 함께 처리한다.
            g_homing_step_request[i] = 1;
        }
    }

    if (system_all_homed()) {
        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            g_homing_step_request[i] = 0;
        }
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

static uint32_t integer_sqrt_u64(uint64_t value)
{
    uint64_t bit = (uint64_t)1 << 62;
    uint64_t result = 0;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result > 0xFFFFFFFF ? 0xFFFFFFFF : (uint32_t)result;
}

void stepper_motion_1ms_interrupt(void)
{
    uint32_t target_feed = MOTION_FEED_ONE_Q16;
    if (!g_motion_active || g_homing_active) return;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        uint64_t remaining;
        uint32_t allowed_sps;
        uint32_t cap_feed;
        if (axis_nominal_rate_q16[i] == 0) continue;
        remaining = g_target_step[i] >= g_current_step[i] ?
                    (uint64_t)(g_target_step[i] - g_current_step[i]) :
                    (uint64_t)(g_current_step[i] - g_target_step[i]);

        // 남은 거리 안에서 멈출 수 있는 속도 비율을 구한다.
        allowed_sps = integer_sqrt_u64(2 * (uint64_t)acceleration_sps2[i] * remaining);
        cap_feed = (uint32_t)(((uint64_t)allowed_sps << 32) /
                              axis_nominal_rate_q16[i]);
        if (cap_feed < target_feed) target_feed = cap_feed;
    }

    if (motion_feed_q16 < target_feed) {
        uint32_t next = motion_feed_q16 + motion_feed_step_q16;
        motion_feed_q16 = next > target_feed ? target_feed : next;
    } else if (motion_feed_q16 > target_feed) {
        motion_feed_q16 = motion_feed_q16 - target_feed > motion_feed_step_q16 ?
                          motion_feed_q16 - motion_feed_step_q16 : target_feed;
    }
}


 //10us 타이머 인터럽트에서 호출됨
void stepper_10us_interrupt(void)
{
    // 1. 이전 스텝 핀 클리어
    clear_step_high_flag();

    // 2. START된 Goal은 ESTOP 외의 상태/에러 변경으로 중단하지 않음
    if (ESTOP_ACTIVE()) return;

    // 3. 홈으로 가는 플래그 처리
    if (g_homing_active) {
        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            // 1ms 방에서 이 축에 스텝을 쏘라고 요청했는지 확인
            if (g_homing_step_request[i]) {

                // 1. 요청을 확인 플래그 클리어
                g_homing_step_request[i] = 0;

                // 2. 하드웨어 타이밍으로 신호 출력
                set_dir(i, home_dir[i]);
                step_pin_high(i);
                if (home_dir[i] > 0) g_current_step[i]++;
                else                 g_current_step[i]--;
            }
        }
        return; // 홈잉 중에는 여기서 끝
    }

    // 3. 모션 구동 중이 아니라면 종료
    if (!g_motion_active || motion_feed_q16 == 0) return;

    // 4. 각 축별 스텝 생성
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        uint64_t actual_rate_q16;
        // 10us 인터럽트가 1초에 100000번 실행된다.
        const uint64_t phase_threshold = (uint64_t)100000 << 16;
        int8_t dir;

        // 목표에 도달하면
        if (g_current_step[i] == g_target_step[i]) continue;

        // 탈조 안나게 대기시간
        if (step_wait_10us[i] > 0) {
            step_wait_10us[i]--;
        }

        // 기준 속도에 현재 가감속 비율을 적용한다.
        actual_rate_q16 = ((uint64_t)axis_nominal_rate_q16[i] * motion_feed_q16) >> 16;
        axis_phase_q16[i] += actual_rate_q16;
        if (axis_phase_q16[i] < phase_threshold || step_wait_10us[i] > 0) {
            continue;
        }

        // 방향
        dir = (g_current_step[i] < g_target_step[i]) ? DIR_POSITIVE : DIR_NEGATIVE;

        if (axis_last_dir[i] != dir) {
            set_dir(i, dir);
            axis_last_dir[i] = dir;
            axis_dir_setup_wait[i] = 1;
            continue;
        }
        if (axis_dir_setup_wait[i] > 0) {
            axis_dir_setup_wait[i]--;
            continue;
        }

        // 홈 방향으로 갈 때 리미트 스위치가 누르면 해당 축만 정지
        if (dir == home_dir[i] && (g_limit_switch_bitmask & (1 << i))) {
            /* Limit blocking is axis-local and nonfatal. Stop only the axis
             * moving farther into its home limit. Other axes in the same goal
             * continue, and a later command in the opposite direction is
             * allowed without Clear Error. */
            stepper_stop_axis(i);
            continue;
        }

        // 스텝 출력
        axis_phase_q16[i] -= phase_threshold;
        step_pin_high(i);
        if (dir > 0) g_current_step[i]++;
        else         g_current_step[i]--;

        // 최소 MIN_STEP_INTERVAL_TICKS 만큼은 쉬어야함 더 빠르면 탈조
        step_wait_10us[i] = MIN_STEP_INTERVAL_TICKS;
    }
}
