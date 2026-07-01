#include "device_driver.h"
#include <stdint.h>
#include <stdio.h>

/* ============================================================
 *  Board2 CAN-only test firmware for STM32F411RE + MCP2515
 * ============================================================
 * Purpose:
 *   - Board2 protocol bring-up without motor driver connected.
 *   - Verify CAN RX/TX, command parsing, status 0x202 response,
 *     homing-done policy, queue policy, and simulated motion state.
 *
 * Hardware kept from the working Board1/F411RE CAN test project:
 *   MCP2515 SCK  -> PB13 / SPI2_SCK
 *   MCP2515 MISO -> PB14 / SPI2_MISO
 *   MCP2515 MOSI -> PB15 / SPI2_MOSI
 *   MCP2515 CS   -> PB12 / GPIO output
 *   MCP2515 INT  -> PB4  / EXTI4 active-low + polling fallback
 *   LED          -> PA5  / Nucleo user LED
 *
 * Implemented Board2 CAN IDs:
 *   0x001 : Emergency Stop broadcast       RPi/Central -> STM32
 *   0x010 : Enable / Disable broadcast     RPi/Central -> STM32
 *   0x020 : Stepper Homing broadcast       RPi/Central -> STM32
 *   0x030 : Clear Error broadcast          RPi/Central -> STM32
 *   0x102 : Board2 trajectory              RPi/Central -> STM32
 *   0x202 : Board2 status                  STM32 -> RPi/Central
 *   0x302 : Board2 position feedback  STM32 -> RPi/Central
 *
 * Final no-payload-board-id policy:
 *   - 0x010 Byte0 is only Enable value; Byte1~7 must be 0.
 *   - 0x020 Byte0 is Target Motor; final stepper homing uses 0xFF.
 *   - 0x030 Byte0 is Target Motor; final clear error uses 0xFF.
 *   - Old target-board payload formats such as 020#02FF... are rejected.
 *
 * Board2 local-motor0 protocol used here:
 *   - Board2 is the base 1-axis board.
 *   - Server global joint id is 0, but CAN payload Motor ID is local 0.
 *   - CAN ID 0x102 already selects Board2.
 *   - Normal absolute angle command Byte0 is 0x80.
 *
 * Hardware-stepper behavior:
 *   - STEP/DIR pins are driven directly from the 1 ms scheduler.
 *   - Homing uses one base-axis limit switch with debounce.
 *   - Motion uses duration-based interpolation as a desired-position generator,
 *     then emits STEP pulses until the actual current_step follows the desired value.
 *   - Position feedback 0x302 is sent every 100 ms for MoveIt2 /joint_states.
 *   - Speed is stored but not used for acceleration profiling.
 *   - Duration byte uses 5 ms units. Duration 0 is treated as 1 ms.
 */

#define CAN_ID_ESTOP             0x001U
#define CAN_ID_ENABLE_DISABLE    0x010U
#define CAN_ID_HOMING_START      0x020U
#define CAN_ID_CLEAR_ERROR       0x030U
#define CAN_ID_POSITION_CMD      0x102U
#define CAN_ID_STATUS            0x202U
#define CAN_ID_POS_FB       0x302U

#define BOARD2_GLOBAL_JOINT_ID   0U
#define BOARD2_LOCAL_MOTOR_ID    0U
#define MOTOR_ALL_ID             255U
#define MOVING_MOTOR_NONE        255U

#define TRAJ_QUEUE_SIZE          32U
#define STATUS_PERIOD_MS         100U
#define POS_FB_PERIOD_MS    100U

#define BOARD2_GEAR_RATIO        20LL
#define MOTOR_STEPS_PER_REV      200LL
#define MICROSTEP                16LL
#define BOARD2_MIN_POS_001DEG    (-9000)
#define BOARD2_MAX_POS_001DEG    (18000)
#define BOARD2_HOME_POS_001DEG   (-9000)

/* Status state values */
#define STATE_INIT               0U
#define STATE_IDLE               1U
#define STATE_HOMING             2U
#define STATE_MOVING             3U
#define STATE_ERROR              4U
#define STATE_ESTOP              5U
#define STATE_DISABLED           6U

/* Error code values */
#define ERR_NONE                 0U
#define ERR_INVALID_CMD          1U
#define ERR_LIMIT_SWITCH_DETECTED 2U
#define ERR_DRIVER_FAULT         3U
#define ERR_HOMING_FAIL          4U
#define ERR_QUEUE_FULL           5U
#define ERR_RESERVED             6U

/* ============================================================
 *  Board2 real STEP/DIR + limit switch hardware pin map
 * ============================================================
 * Board2 담당 축 = 1-axis / base_joint.
 * Limit switch pin received from the motor-control team: PA7.
 * STEP/DIR pins remain as the previous editable default unless the base motor
 * STEP/DIR pin map is provided separately.
 *
 * Limit switch default assumption:
 *   GPIO input ---- switch COM
 *   GND        ---- switch NO
 *   STM32 internal pull-up enabled
 *   normal = 1, pressed = 0
 */
#define LIMIT_SWITCH_ACTIVE_LEVEL     0U
#define LIMIT_SWITCH_DEBOUNCE_TICKS   5U
#define STEP_PULSE_DELAY_LOOPS        10U
#define HW_MAX_STEP_PULSES_PER_MS     50U

#define DIR_POSITIVE                  1
#define DIR_NEGATIVE                 -1
#define BOARD2_HOME_DIR              DIR_NEGATIVE

#define BOARD2_STEP_PORT              GPIOC
#define BOARD2_STEP_PIN               0U
#define BOARD2_DIR_PORT               GPIOC
#define BOARD2_DIR_PIN                1U
#define BOARD2_LIM_PORT               GPIOA
#define BOARD2_LIM_PIN                7U

static volatile uint16_t g_limit_debounce = 0U;

typedef struct
{
    uint8_t  motor_id;
    int32_t  target_step;
    uint16_t speed;
    uint16_t duration_ms;
    uint8_t  relative;
    uint8_t  step_mode;
} TrajectoryPoint_t;

static volatile uint32_t g_ms_tick = 0U;
static volatile uint8_t  g_status_event = 0U;
static volatile uint8_t  g_pos_fb_event = 0U;
static volatile uint8_t  g_pos_fb_sequence = 0U;

static volatile uint8_t  g_state = STATE_INIT;
static volatile uint8_t  g_error_code = ERR_NONE;
static volatile uint8_t  g_homing_done = 0U;
static volatile uint8_t  g_limit_status = 0U;
static volatile uint8_t  g_enabled = 0U;
static volatile uint8_t  g_moving_motor_id = MOVING_MOTOR_NONE;

static TrajectoryPoint_t g_queue[TRAJ_QUEUE_SIZE];
static volatile uint8_t  g_q_head = 0U;
static volatile uint8_t  g_q_tail = 0U;
static volatile uint8_t  g_q_count = 0U;

static volatile uint8_t  g_motion_active = 0U;
static TrajectoryPoint_t g_active_point;
static volatile uint32_t g_motion_elapsed_ms = 0U;
static volatile uint32_t g_motion_duration_ms = 0U;
static volatile int32_t  g_segment_start_step = 0;
static volatile int32_t  g_current_step = 0;
static volatile int32_t  g_target_step = 0;

static volatile uint8_t  g_homing_active = 0U;
static volatile uint16_t g_homing_remaining_ms = 0U;


static void queue_clear(void);
static void stop_motion_only(void);
static void request_status_event(void);
static void request_pos_fb_event(void);
static void enter_error(uint8_t error_code);

static void delay_loop(volatile unsigned int n)
{
    while (n--) { __NOP(); }
}

static void led_direct_init(void)
{
    RCC->AHB1ENR |= (1U << 0);
    (void)RCC->AHB1ENR;
    GPIOA->MODER &= ~(0x3U << (5U * 2U));
    GPIOA->MODER |=  (0x1U << (5U * 2U));
    GPIOA->OTYPER &= ~(1U << 5U);
}

static void led_direct_toggle(void)
{
    GPIOA->ODR ^= (1U << 5U);
}

static void blink_direct(int count, unsigned int delay)
{
    int i;
    for (i = 0; i < count; i++)
    {
        led_direct_toggle();
        delay_loop(delay);
        led_direct_toggle();
        delay_loop(delay);
    }
}

static void gpio_enable_clock(GPIO_TypeDef *port)
{
    if (port == GPIOA) RCC->AHB1ENR |= (1U << 0);
    else if (port == GPIOB) RCC->AHB1ENR |= (1U << 1);
    else if (port == GPIOC) RCC->AHB1ENR |= (1U << 2);
#ifdef GPIOD
    else if (port == GPIOD) RCC->AHB1ENR |= (1U << 3);
#endif
#ifdef GPIOE
    else if (port == GPIOE) RCC->AHB1ENR |= (1U << 4);
#endif
#ifdef GPIOH
    else if (port == GPIOH) RCC->AHB1ENR |= (1U << 7);
#endif
    (void)RCC->AHB1ENR;
}

static void gpio_config_output_pp(GPIO_TypeDef *port, uint8_t pin)
{
    gpio_enable_clock(port);
    port->MODER &= ~(0x3U << ((uint32_t)pin * 2U));
    port->MODER |=  (0x1U << ((uint32_t)pin * 2U));
    port->OTYPER &= ~(1U << pin);
    port->OSPEEDR |= (0x2U << ((uint32_t)pin * 2U));
    port->PUPDR &= ~(0x3U << ((uint32_t)pin * 2U));
}

static void gpio_config_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    gpio_enable_clock(port);
    port->MODER &= ~(0x3U << ((uint32_t)pin * 2U));
    port->PUPDR &= ~(0x3U << ((uint32_t)pin * 2U));
    port->PUPDR |=  (0x1U << ((uint32_t)pin * 2U));
}

static void gpio_write(GPIO_TypeDef *port, uint8_t pin, uint8_t high)
{
    if (high) port->BSRR = (1U << pin);
    else      port->BSRR = (1U << (pin + 16U));
}

static uint8_t gpio_read(GPIO_TypeDef *port, uint8_t pin)
{
    return (port->IDR & (1U << pin)) ? 1U : 0U;
}

static void stepper_outputs_low_all(void)
{
    gpio_write(BOARD2_STEP_PORT, BOARD2_STEP_PIN, 0U);
}

static void stepper_hw_init(void)
{
    gpio_config_output_pp(BOARD2_STEP_PORT, BOARD2_STEP_PIN);
    gpio_config_output_pp(BOARD2_DIR_PORT, BOARD2_DIR_PIN);
    gpio_config_input_pullup(BOARD2_LIM_PORT, BOARD2_LIM_PIN);
    gpio_write(BOARD2_STEP_PORT, BOARD2_STEP_PIN, 0U);
    gpio_write(BOARD2_DIR_PORT, BOARD2_DIR_PIN, 0U);
    g_limit_debounce = 0U;
}

static uint8_t limit_switch_pressed_raw(void)
{
    uint8_t v = gpio_read(BOARD2_LIM_PORT, BOARD2_LIM_PIN);
    return (v == LIMIT_SWITCH_ACTIVE_LEVEL) ? 1U : 0U;
}

static uint8_t limit_switch_pressed_stable(void)
{
    if (limit_switch_pressed_raw())
    {
        if (g_limit_debounce < LIMIT_SWITCH_DEBOUNCE_TICKS)
        {
            g_limit_debounce++;
        }
    }
    else
    {
        g_limit_debounce = 0U;
    }

    return (g_limit_debounce >= LIMIT_SWITCH_DEBOUNCE_TICKS) ? 1U : 0U;
}

static void enter_limit_error(void)
{
    queue_clear();
    stop_motion_only();
    g_homing_active = 0U;
    g_limit_status = limit_switch_pressed_raw();
    enter_error(ERR_LIMIT_SWITCH_DETECTED);
}

static uint8_t stepper_step_once(int8_t dir)
{
    if (dir == 0) return 0U;

    if (dir == BOARD2_HOME_DIR && limit_switch_pressed_stable())
    {
        enter_limit_error();
        return 0U;
    }

    gpio_write(BOARD2_DIR_PORT, BOARD2_DIR_PIN, (dir > 0) ? 1U : 0U);
    gpio_write(BOARD2_STEP_PORT, BOARD2_STEP_PIN, 1U);
    delay_loop(STEP_PULSE_DELAY_LOOPS);
    gpio_write(BOARD2_STEP_PORT, BOARD2_STEP_PIN, 0U);

    if (dir > 0) g_current_step++;
    else         g_current_step--;

    return 1U;
}

static void stepper_drive_toward(int32_t desired_step)
{
    uint16_t pulses = 0U;

    while (g_current_step != desired_step &&
           pulses < HW_MAX_STEP_PULSES_PER_MS &&
           g_state != STATE_ERROR &&
           g_state != STATE_ESTOP)
    {
        int8_t dir = (g_current_step < desired_step) ? DIR_POSITIVE : DIR_NEGATIVE;
        if (!stepper_step_once(dir)) break;
        pulses++;
    }
}

static void Sys_Init(int baud)
{
    SCB->CPACR |= (0x3U << (10U * 2U)) | (0x3U << (11U * 2U));
    Clock_Init();
    Uart2_Init(baud);
    setvbuf(stdout, NULL, _IONBF, 0);
    LED_Init();
}

static uint16_t read_u16_le(const uint8_t *d)
{
    return (uint16_t)(((uint16_t)d[0]) | ((uint16_t)d[1] << 8));
}

static int32_t read_i32_le(const uint8_t *d)
{
    uint32_t u = ((uint32_t)d[0]) |
                 ((uint32_t)d[1] << 8) |
                 ((uint32_t)d[2] << 16) |
                 ((uint32_t)d[3] << 24);
    return (int32_t)u;
}

static int32_t clamp_i64_to_i32(int64_t v)
{
    if (v > 2147483647LL) return 2147483647;
    if (v < (-2147483647LL - 1LL)) return (int32_t)(-2147483647 - 1);
    return (int32_t)v;
}

static int32_t angle_001deg_to_step(int32_t angle_raw)
{
    int64_t steps;

    /* Board2 formula:
     * step = angle_raw * gear_ratio * 200 * 16 / 36000
     * angle_raw unit = 0.01 degree
     * gear_ratio = 20
     */
    steps = ((int64_t)angle_raw *
             BOARD2_GEAR_RATIO *
             MOTOR_STEPS_PER_REV *
             MICROSTEP) / 36000LL;
    return clamp_i64_to_i32(steps);
}

static int32_t step_to_angle_001deg_i32(int32_t step)
{
    int64_t denom = BOARD2_GEAR_RATIO * MOTOR_STEPS_PER_REV * MICROSTEP;
    int64_t raw;

    if (denom == 0LL) return 0;

    /* Inverse of angle_001deg_to_step():
     * angle_raw = step * 36000 / (gear_ratio * 200 * 16)
     * angle_raw unit = 0.01 degree.
     */
    raw = ((int64_t)step * 36000LL) / denom;
    return clamp_i64_to_i32(raw);
}

static uint8_t angle_raw_within_board2_limit(int32_t angle_raw)
{
    return (angle_raw >= BOARD2_MIN_POS_001DEG &&
            angle_raw <= BOARD2_MAX_POS_001DEG) ? 1U : 0U;
}

static void write_i32_le(uint8_t *d, int32_t v)
{
    uint32_t u = (uint32_t)v;
    d[0] = (uint8_t)(u & 0xFFU);
    d[1] = (uint8_t)((u >> 8) & 0xFFU);
    d[2] = (uint8_t)((u >> 16) & 0xFFU);
    d[3] = (uint8_t)((u >> 24) & 0xFFU);
}

static uint8_t make_position_feedback_flags(void)
{
    uint8_t flags = 0U;
    uint8_t moving = 0U;

    /* Byte1 flags:
     * bit0 = position valid
     * bit1 = homed / ready
     * bit2 = moving
     * bit3 = target reached
     * bit4~7 = reserved 0
     */
    flags |= 0x01U; /* position valid */

    if (g_state == STATE_MOVING || g_state == STATE_HOMING ||
        g_motion_active || g_homing_active)
    {
        moving = 1U;
        flags |= 0x04U;
    }

    if (g_enabled && g_homing_done &&
        g_state != STATE_ERROR && g_state != STATE_ESTOP &&
        g_error_code == ERR_NONE)
    {
        flags |= 0x02U; /* homed / ready */
    }

    if (g_enabled && g_homing_done && !moving &&
        g_q_count == 0U && g_error_code == ERR_NONE &&
        g_state == STATE_IDLE)
    {
        flags |= 0x08U; /* target reached */
    }

    return flags;
}

static uint8_t queue_free_slots(void)
{
    if (g_q_count >= TRAJ_QUEUE_SIZE) return 0U;
    return (uint8_t)(TRAJ_QUEUE_SIZE - g_q_count);
}

static void queue_clear(void)
{
    g_q_head = 0U;
    g_q_tail = 0U;
    g_q_count = 0U;
}

static int queue_push(const TrajectoryPoint_t *p)
{
    if (g_q_count >= TRAJ_QUEUE_SIZE)
    {
        return -1;
    }

    g_queue[g_q_tail] = *p;
    g_q_tail = (uint8_t)((g_q_tail + 1U) % TRAJ_QUEUE_SIZE);
    g_q_count++;
    return 0;
}

static int queue_pop(TrajectoryPoint_t *p)
{
    if (g_q_count == 0U)
    {
        return -1;
    }

    *p = g_queue[g_q_head];
    g_q_head = (uint8_t)((g_q_head + 1U) % TRAJ_QUEUE_SIZE);
    g_q_count--;
    return 0;
}

static uint8_t find_moving_motor(void)
{
    if (g_state == STATE_HOMING && g_homing_active) return BOARD2_LOCAL_MOTOR_ID;
    if (g_motion_active) return BOARD2_LOCAL_MOTOR_ID;
    return MOVING_MOTOR_NONE;
}

static void send_status(void)
{
    CAN_Frame_t st;
    uint8_t i;

    g_limit_status = limit_switch_pressed_raw();

    st.id = CAN_ID_STATUS;
    st.dlc = 8U;
    for (i = 0U; i < 8U; i++) st.data[i] = 0U;

    st.data[0] = g_state;
    st.data[1] = g_error_code;
    st.data[2] = g_homing_done ? 1U : 0U;
    st.data[3] = find_moving_motor();
    st.data[4] = g_limit_status ? 1U : 0U;
    st.data[5] = queue_free_slots();
    st.data[6] = g_enabled ? 1U : 0U;
    st.data[7] = 0U;

    (void)MCP2515_Send_Frame(&st);
}

static void send_position_feedback(void)
{
    CAN_Frame_t fb;

    g_limit_status = limit_switch_pressed_raw();
    int32_t current_pos_001deg;
    uint8_t i;

    fb.id = CAN_ID_POS_FB;
    fb.dlc = 8U;
    for (i = 0U; i < 8U; i++) fb.data[i] = 0U;

    current_pos_001deg = step_to_angle_001deg_i32(g_current_step);

    /* Board2 position feedback payload, final integrated format:
     *   Byte0   = Local Motor ID, Board2 uses 0
     *   Byte1   = flags
     *   Byte2~5 = current_pos_001deg, int32 little-endian
     *   Byte6   = error/fault code, 0 when none
     *   Byte7   = sequence counter
     */
    fb.data[0] = BOARD2_LOCAL_MOTOR_ID;
    fb.data[1] = make_position_feedback_flags();
    write_i32_le(&fb.data[2], current_pos_001deg);
    fb.data[6] = g_error_code;
    fb.data[7] = g_pos_fb_sequence++;

    (void)MCP2515_Send_Frame(&fb);
}

static void request_status_event(void)
{
    g_status_event = 1U;
}

static void request_pos_fb_event(void)
{
    g_pos_fb_event = 1U;
}

static void stop_motion_only(void)
{
    stepper_outputs_low_all();
    g_motion_active = 0U;
    g_motion_elapsed_ms = 0U;
    g_motion_duration_ms = 0U;
    g_moving_motor_id = MOVING_MOTOR_NONE;
}

static void enter_error(uint8_t error_code)
{
    g_error_code = error_code;
    if (g_state != STATE_ESTOP)
    {
        g_state = STATE_ERROR;
    }
    stop_motion_only();
    request_status_event();
    request_pos_fb_event();
}

static void emergency_stop(void)
{
    __disable_irq();
    queue_clear();
    stop_motion_only();
    g_homing_active = 0U;
    g_homing_remaining_ms = 0U;
    g_enabled = 0U;
    g_state = STATE_ESTOP;
    g_error_code = ERR_NONE;
    __enable_irq();

    stepper_outputs_low_all();
    printf("ESTOP\n");
    send_status();
    send_position_feedback();
}

static uint8_t is_motion_allowed(void)
{
    if (!g_enabled) return 0U;
    if (g_state == STATE_ESTOP) return 0U;
    if (g_state == STATE_ERROR) return 0U;
    if (g_state == STATE_HOMING) return 0U;
    if (g_error_code != ERR_NONE) return 0U;
    return 1U;
}

static void start_next_motion_if_possible(void)
{
    TrajectoryPoint_t p;

    if (g_motion_active) return;
    if (!is_motion_allowed()) return;

    if (queue_pop(&p) != 0)
    {
        g_state = STATE_IDLE;
        g_moving_motor_id = MOVING_MOTOR_NONE;
        return;
    }

    g_active_point = p;
    g_segment_start_step = g_current_step;
    g_motion_elapsed_ms = 0U;
    g_motion_duration_ms = (p.duration_ms == 0U) ? 1U : p.duration_ms;
    g_target_step = p.target_step;
    g_motion_active = 1U;
    g_state = STATE_MOVING;
    g_moving_motor_id = BOARD2_LOCAL_MOTOR_ID;

    printf("MOVE START local_motor=%u global_joint=%u start=%ld target=%ld dur=%lu speed=%u q_free=%u\n",
           BOARD2_LOCAL_MOTOR_ID,
           BOARD2_GLOBAL_JOINT_ID,
           (long)g_segment_start_step,
           (long)p.target_step,
           (unsigned long)g_motion_duration_ms,
           p.speed,
           queue_free_slots());
}

static void complete_active_motion_if_ready(void)
{
    if (!g_motion_active) return;

    if (g_motion_elapsed_ms >= g_motion_duration_ms &&
        g_current_step == g_active_point.target_step)
    {
        g_target_step = g_active_point.target_step;
        g_motion_active = 0U;
        g_moving_motor_id = MOVING_MOTOR_NONE;

        printf("MOVE DONE local_motor=%u current=%ld q=%u\n",
               BOARD2_LOCAL_MOTOR_ID,
               (long)g_current_step,
               g_q_count);

        if (g_q_count == 0U)
        {
            g_state = STATE_IDLE;
            request_status_event();
            request_pos_fb_event();
        }
        else
        {
            start_next_motion_if_possible();
        }
    }
}

static void trajectory_tick_1ms(void)
{
    g_limit_status = limit_switch_pressed_raw();

    if (g_state == STATE_ERROR || g_state == STATE_ESTOP)
    {
        stepper_outputs_low_all();
        return;
    }

    if (g_state == STATE_HOMING && g_homing_active)
    {
        if (limit_switch_pressed_stable())
        {
            g_current_step = angle_001deg_to_step(BOARD2_HOME_POS_001DEG);
            g_target_step = g_current_step;
            g_segment_start_step = g_current_step;
            g_homing_active = 0U;
            g_homing_done = 1U;
            g_state = STATE_IDLE;
            stepper_outputs_low_all();
            printf("HOMING DONE local_motor=%u global_joint=%u\n",
                   BOARD2_LOCAL_MOTOR_ID,
                   BOARD2_GLOBAL_JOINT_ID);
            request_status_event();
            request_pos_fb_event();
        }
        else
        {
            (void)stepper_step_once(BOARD2_HOME_DIR);
        }
        return;
    }

    if (!g_motion_active)
    {
        start_next_motion_if_possible();
        return;
    }

    if (g_motion_elapsed_ms < g_motion_duration_ms)
    {
        g_motion_elapsed_ms++;
    }

    {
        int64_t start = (int64_t)g_segment_start_step;
        int64_t target = (int64_t)g_active_point.target_step;
        int64_t delta = target - start;
        int64_t value;

        if (g_motion_elapsed_ms >= g_motion_duration_ms)
        {
            value = target;
        }
        else
        {
            value = start + ((delta * (int64_t)g_motion_elapsed_ms) /
                             (int64_t)g_motion_duration_ms);
        }

        stepper_drive_toward(clamp_i64_to_i32(value));
    }

    complete_active_motion_if_ready();
}

static void tim3_1ms_init(void)
{
    Macro_Set_Bit(RCC->APB1ENR, 1);  /* TIM3 clock */
    (void)RCC->APB1ENR;

    Macro_Clear_Bit(TIM3->CR1, 0);
    TIM3->PSC = (uint32_t)(TIMXCLK / 1000000U) - 1U; /* 1 MHz */
    TIM3->ARR = 1000U - 1U;                           /* 1 ms */
    TIM3->CNT = 0U;
    Macro_Set_Bit(TIM3->EGR, 0);
    Macro_Clear_Bit(TIM3->SR, 0);

    NVIC_SetPriority(TIM3_IRQn, 6);
    NVIC_EnableIRQ(TIM3_IRQn);
    Macro_Set_Bit(TIM3->DIER, 0);
    Macro_Set_Bit(TIM3->CR1, 0);
}

static uint8_t reserved_zero(const CAN_Frame_t *frame, uint8_t start_index)
{
    uint8_t i;

    for (i = start_index; i < frame->dlc; i++)
    {
        if (frame->data[i] != 0U) return 0U;
    }
    return 1U;
}

static uint8_t frame_is_exact_8_bytes(const CAN_Frame_t *frame)
{
    return (frame->dlc == 8U) ? 1U : 0U;
}

static void handle_estop(const CAN_Frame_t *frame)
{
    /* Final strict form used by the integrated protocol test:
     *   001#0100000000000000
     */
    if (!frame_is_exact_8_bytes(frame) || frame->data[0] != 1U || !reserved_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("ESTOP invalid final-format dlc=%u\n", frame->dlc);
        send_status();
        send_position_feedback();
        return;
    }

    emergency_stop();
}

static void handle_enable_disable(const CAN_Frame_t *frame)
{
    uint8_t cmd;

    /* Final Board2 protocol:
     *   CAN ID 0x010
     *   Byte0 = 0:disable, 1:enable
     *   Byte1~7 = reserved 0
     *   No payload Board ID / Target Board field.
     */
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("ENABLE/DISABLE invalid final-format dlc=%u byte1=0x%02X\n",
               frame->dlc,
               (frame->dlc > 1U) ? frame->data[1] : 0U);
        send_status();
        return;
    }

    cmd = frame->data[0];

    if (cmd == 1U)
    {
        __disable_irq();
        g_enabled = 1U;
        g_error_code = ERR_NONE;
        if (g_state == STATE_ESTOP || g_state == STATE_ERROR ||
            g_state == STATE_INIT || g_state == STATE_DISABLED)
        {
            g_state = STATE_IDLE;
        }
        __enable_irq();
        printf("ENABLE broadcast final-format\n");
    }
    else if (cmd == 0U)
    {
        __disable_irq();
        queue_clear();
        stop_motion_only();
        g_homing_active = 0U;
        g_homing_remaining_ms = 0U;
        g_enabled = 0U;
        g_state = STATE_DISABLED;
        __enable_irq();
        printf("DISABLE broadcast final-format\n");
    }
    else
    {
        enter_error(ERR_INVALID_CMD);
        printf("ENABLE/DISABLE invalid cmd=%u\n", cmd);
    }

    send_status();
    send_position_feedback();
}

static void handle_homing_start(const CAN_Frame_t *frame)
{
    uint8_t target_motor;
    uint8_t mode;

    /* Final Board2 protocol:
     *   CAN ID 0x020
     *   Byte0 = Target Motor, final arm homing uses 0xFF only
     *   Byte1 = Homing Mode, currently 0 only
     *   Byte2~7 = reserved 0
     *   No payload Board ID / Target Board field.
     */
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 2U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("HOMING invalid final-format dlc=%u byte2=0x%02X\n",
               frame->dlc,
               (frame->dlc > 2U) ? frame->data[2] : 0U);
        send_status();
        return;
    }

    target_motor = frame->data[0];
    mode = frame->data[1];

    if (!g_enabled || g_state == STATE_ESTOP)
    {
        enter_error(ERR_INVALID_CMD);
        printf("HOMING rejected enabled=%u state=%u\n", g_enabled, g_state);
        send_status();
        return;
    }

    if (target_motor != MOTOR_ALL_ID || mode != 0U)
    {
        enter_error(ERR_INVALID_CMD);
        printf("HOMING invalid target_motor=0x%02X mode=%u; expected FF/0\n",
               target_motor,
               mode);
        send_status();
        return;
    }

    __disable_irq();
    queue_clear();
    stop_motion_only();
    g_homing_active = 1U;
    g_homing_remaining_ms = 0U;
    g_homing_done = 0U;
    g_limit_debounce = 0U;
    g_state = STATE_HOMING;
    g_error_code = ERR_NONE;
    __enable_irq();

    printf("STEPPER HOMING START final-format: Board2 base local_motor=%u global_joint=%u\n",
           BOARD2_LOCAL_MOTOR_ID,
           BOARD2_GLOBAL_JOINT_ID);
    send_status();
    send_position_feedback();
}

static void handle_clear_error(const CAN_Frame_t *frame)
{
    uint8_t target_motor;

    /* Final Board2 protocol:
     *   CAN ID 0x030
     *   Byte0 = Target Motor, final broadcast clear uses 0xFF only
     *   Byte1~7 = reserved 0
     *   No payload Board ID / Target Board field.
     */
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("CLEAR ERROR invalid final-format dlc=%u byte1=0x%02X\n",
               frame->dlc,
               (frame->dlc > 1U) ? frame->data[1] : 0U);
        send_status();
        return;
    }

    target_motor = frame->data[0];

    if (target_motor != MOTOR_ALL_ID)
    {
        enter_error(ERR_INVALID_CMD);
        printf("CLEAR ERROR invalid target_motor=0x%02X; expected FF\n", target_motor);
        send_status();
        return;
    }

    __disable_irq();
    g_error_code = ERR_NONE;
    if (g_state == STATE_ERROR)
    {
        g_state = g_enabled ? STATE_IDLE : STATE_DISABLED;
    }
    /* ESTOP is not released by Clear Error. Enable=1 releases ESTOP. */
    __enable_irq();

    printf("CLEAR ERROR broadcast final-format state=%u\n", g_state);
    send_status();
    send_position_feedback();
}

static void handle_position_command(const CAN_Frame_t *frame)
{
    uint8_t byte0;
    uint8_t motor_id;
    uint8_t flags;
    uint8_t execute;
    uint8_t relative;
    uint8_t step_mode;
    int32_t raw_target;
    int32_t target_step;
    uint16_t speed;
    uint16_t duration_ms;
    TrajectoryPoint_t p;

    if (frame->dlc != 8U)
    {
        enter_error(ERR_INVALID_CMD);
        printf("POS invalid DLC=%u\n", frame->dlc);
        send_status();
        return;
    }

    byte0 = frame->data[0];
    motor_id = byte0 & 0x0FU;
    flags = byte0 >> 4;
    execute = (flags & 0x08U) ? 1U : 0U;
    relative = (flags & 0x04U) ? 1U : 0U;
    step_mode = (flags & 0x02U) ? 1U : 0U;

    if (!execute)
    {
        printf("POS ignored execute=0 byte0=0x%02X\n", byte0);
        return;
    }

    if ((flags & 0x01U) != 0U || motor_id != BOARD2_LOCAL_MOTOR_ID)
    {
        enter_error(ERR_INVALID_CMD);
        printf("POS invalid flags=0x%X motor=%u expected_motor=%u\n",
               flags,
               motor_id,
               BOARD2_LOCAL_MOTOR_ID);
        send_status();
        return;
    }

    if (!is_motion_allowed())
    {
        if (g_state != STATE_ESTOP && g_state != STATE_ERROR && g_error_code == ERR_NONE)
        {
            enter_error(ERR_INVALID_CMD);
        }
        printf("POS rejected state=%u enabled=%u err=%u\n", g_state, g_enabled, g_error_code);
        send_status();
        return;
    }

    if (!g_homing_done)
    {
        enter_error(ERR_INVALID_CMD);
        printf("POS rejected: Board2 not homed\n");
        send_status();
        return;
    }

    raw_target = read_i32_le(&frame->data[1]);
    speed = read_u16_le(&frame->data[5]);
    duration_ms = (uint16_t)frame->data[7] * 5U;
    if (duration_ms == 0U) duration_ms = 1U;

    if (step_mode)
    {
        target_step = raw_target;
    }
    else
    {
        target_step = angle_001deg_to_step(raw_target);
    }

    if (relative)
    {
        target_step = clamp_i64_to_i32((int64_t)g_current_step +
                                       (int64_t)target_step);
    }

    /* Board2 is base_joint. Apply final integrated base limits.
     * Angle mode validates the output-shaft angle directly.
     * Step mode is also checked after inverse conversion to output-shaft angle.
     */
    {
        int32_t final_angle_raw = step_to_angle_001deg_i32(target_step);
        if (!angle_raw_within_board2_limit(final_angle_raw))
        {
            enter_error(ERR_INVALID_CMD);
            printf("POS rejected by base limit: final_raw=%ld min=%ld max=%ld\n",
                   (long)final_angle_raw,
                   (long)BOARD2_MIN_POS_001DEG,
                   (long)BOARD2_MAX_POS_001DEG);
            send_status();
            return;
        }
    }

    p.motor_id = BOARD2_LOCAL_MOTOR_ID;
    p.target_step = target_step;
    p.speed = speed;
    p.duration_ms = duration_ms;
    p.relative = relative;
    p.step_mode = step_mode;

    __disable_irq();
    if (g_q_count >= TRAJ_QUEUE_SIZE)
    {
        __enable_irq();
        enter_error(ERR_QUEUE_FULL);
        printf("POS queue full: drop target=%ld\n", (long)target_step);
        send_status();
        return;
    }
    (void)queue_push(&p);
    __enable_irq();

    printf("POS queued local_motor=%u raw=%ld target_step=%ld speed=%u dur_ms=%u rel=%u step=%u q_free=%u\n",
           BOARD2_LOCAL_MOTOR_ID,
           (long)raw_target,
           (long)target_step,
           speed,
           duration_ms,
           relative,
           step_mode,
           queue_free_slots());
}

static void process_can_frame(const CAN_Frame_t *frame)
{
    uint8_t i;

    printf("RX 0x%03X DLC=%u D=", frame->id, frame->dlc);
    for (i = 0U; i < frame->dlc; i++) printf("%02X", frame->data[i]);
    printf("\n");

    switch (frame->id)
    {
    case CAN_ID_ESTOP:
        handle_estop(frame);
        break;

    case CAN_ID_ENABLE_DISABLE:
        handle_enable_disable(frame);
        break;

    case CAN_ID_HOMING_START:
        handle_homing_start(frame);
        break;

    case CAN_ID_CLEAR_ERROR:
        handle_clear_error(frame);
        break;

    case CAN_ID_POSITION_CMD:
        handle_position_command(frame);
        break;

    default:
        break;
    }
}

static void process_mcp2515_irq_event(void)
{
    CAN_Frame_t frame;

    while (MCP2515_Read_Frame(&frame))
    {
        LED_Toggle();
        process_can_frame(&frame);
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & 0x01U)
    {
        Macro_Clear_Bit(TIM3->SR, 0);

        g_ms_tick++;
        trajectory_tick_1ms();

        if ((g_ms_tick % STATUS_PERIOD_MS) == 0U)
        {
            g_status_event = 1U;
        }

        if ((g_ms_tick % POS_FB_PERIOD_MS) == 0U)
        {
            g_pos_fb_event = 1U;
        }
    }
}

void Main(void)
{
    int init_ret;

    led_direct_init();
    blink_direct(3, 800000U);

    Sys_Init(115200);

    printf("\nBoard2 CAN + STEP/DIR hardware test firmware - STM32F411RE + MCP2515\n");
    printf("CAN IDs: ESTOP=0x001 ENABLE=0x010 HOME=0x020 CLEAR=0x030 POS=0x102 STATUS=0x202 POS_FB=0x302\n");
    printf("Board2 mapping: global joint id 0 / base_joint -> payload local Motor ID 0\n");
    printf("MCP2515 pins: PB13=SCK PB14=MISO PB15=MOSI PB12=CS PB4=INT\n");
    printf("Stepper pins: STEP=PC0 DIR=PC1 LIM=PA7, active-low pull-up. Board2 handles base / 1-axis.\n");

    queue_clear();
    stepper_hw_init();

    MCP2515_SPI_Init(64U);
    init_ret = MCP2515_Init(MCP2515_OSC_8MHZ);
    if (init_ret != 0)
    {
        g_state = STATE_ERROR;
        g_error_code = ERR_DRIVER_FAULT;
        printf("MCP2515 init failed: %d\n", init_ret);
        while (1)
        {
            led_direct_toggle();
            delay_loop(300000U);
        }
    }

    __disable_irq();
    g_state = STATE_DISABLED;
    g_error_code = ERR_NONE;
    g_homing_done = 0U;
    g_limit_status = 0U;
    g_enabled = 0U;
    g_moving_motor_id = MOVING_MOTOR_NONE;
    g_current_step = angle_001deg_to_step(BOARD2_HOME_POS_001DEG);
    g_target_step = g_current_step;
    __enable_irq();

    tim3_1ms_init();

    printf("MCP2515 init OK. Status 0x202 every %u ms; position feedback 0x302 every %u ms.\n", STATUS_PERIOD_MS, POS_FB_PERIOD_MS);
    printf("Command order: 010#0100000000000000 -> 020#FF00000000000000 -> 102#80B80B0000E8030A\n");
    printf("Final protocol rejects old payload Board ID forms such as 020#02FF... and 030#02FF...\n");
    printf("Example move: cansend can0 102#80B80B0000E8030A   // base_joint 30.00 deg, 50 ms\n");
    send_status();
    send_position_feedback();

    {
        uint32_t last_mcp_poll_ms = 0xFFFFFFFFU;

        for (;;)
        {
            uint8_t service_mcp = 0U;

            if (g_mcp2515_irq)
            {
                __disable_irq();
                g_mcp2515_irq = 0U;
                __enable_irq();
                service_mcp = 1U;
            }

            if (MCP2515_Int_Asserted())
            {
                service_mcp = 1U;
            }

            if (last_mcp_poll_ms != g_ms_tick)
            {
                last_mcp_poll_ms = g_ms_tick;
                service_mcp = 1U;
            }

            if (service_mcp)
            {
                process_mcp2515_irq_event();
            }

            if (g_status_event)
            {
                __disable_irq();
                g_status_event = 0U;
                __enable_irq();
                send_status();
            }

            if (g_pos_fb_event)
            {
                __disable_irq();
                g_pos_fb_event = 0U;
                __enable_irq();
                send_position_feedback();
            }

            __WFI();
        }
    }
}
