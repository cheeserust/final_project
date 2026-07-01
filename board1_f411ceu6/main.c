#include "device_driver.h"
#include <stdint.h>
#include <stdio.h>

/* ============================================================
 *  Board1 test firmware for STM32F411CEU6 + MCP2515
 * ============================================================
 * Purpose:
 *   - Board1 protocol bring-up aligned with integrated Board1/Board2/Board3 protocol.
 *   - Verify CAN RX/TX, command parsing, 0x101 4-frame trajectory
 *     point assembly for Board1 arm axes 2~5, status 0x201 response,
 *     queue policy, and simulated 4-axis simultaneous motion state.
 *
 * Hardware wiring updated for the current STM32F411CEU6 test:
 *   MCP2515 SCK  -> PB13 / SPI2_SCK
 *   MCP2515 MISO -> PB14 / SPI2_MISO
 *   MCP2515 MOSI -> PB15 / SPI2_MOSI
 *   MCP2515 CS   -> PA9  / GPIO output
 *   MCP2515 INT  -> PA10 / EXTI10 active-low + polling fallback
 *   LED          -> PC13 / BlackPill-style LED, active-low
 *
 * Implemented Board1 CAN IDs:
 *   0x001 : Emergency Stop       RPi/Central -> STM32
 *   0x010 : Enable / Disable     RPi/Central -> STM32, broadcast, no Board ID payload
 *   0x020 : Arm Homing           RPi/Central -> STM32, Board1+Board2 broadcast
 *   0x030 : Clear Error          RPi/Central -> STM32, broadcast, no Board ID payload
 *   0x101 : Board1 trajectory    RPi/Central -> STM32
 *   0x201 : Board1 status        STM32 -> RPi/Central, 100 ms
 *   0x301 : Board1 position feedback STM32 -> RPi/Central, 100 ms, 4 per-motor frames
 *
 * Board1 0x101 final protocol implemented here:
 *   - One CAN frame does NOT immediately move one motor.
 *   - A single MoveIt trajectory point is made from four 0x101 frames.
 *   - Required frame order: local Motor ID 0 -> 1 -> 2 -> 3.
 *   - All four frames must arrive within BOARD1_POINT_TIMEOUT_MS = 20 ms.
 *   - All four frames must have the same Duration byte.
 *   - Non-moving axes must still be sent with their hold target position.
 *   - After frame 3 is received, the assembled 4-axis point is pushed into queue.
 *   - The four axes start together in the same 1 ms scheduler tick.
 *
 * Current hardware-stepper behavior:
 *   - STEP/DIR pins are driven directly from the 1 ms scheduler.
 *   - Limit switch homing uses input debounce and the Board1 home raw angles.
 *   - Motion uses duration-based interpolation as a desired-position generator,
 *     then emits STEP pulses until the actual current_step follows the desired value.
 *   - 0x301 position feedback sends local Motor ID 0->1->2->3 frames every 100 ms.
 *   - Speed is stored but not used for acceleration profiling.
 *   - Duration byte uses 5 ms units. Duration 0 is treated as 1 ms.
 */

#define CAN_ID_ESTOP             0x001U
#define CAN_ID_ENABLE_DISABLE    0x010U
#define CAN_ID_ARM_HOMING        0x020U
#define CAN_ID_CLEAR_ERROR       0x030U
#define CAN_ID_BOARD1_MOVE       0x101U
#define CAN_ID_BOARD1_STATUS     0x201U
#define CAN_ID_BOARD1_FEEDBACK   0x301U

#define BOARD1_MOTOR_COUNT       4U
#define MOTOR_ALL_ID             255U
#define MOVING_MOTOR_NONE        255U

#define TRAJ_QUEUE_SIZE          8U   /* internal 4-axis point queue: 8 points = 32 external 0x101 command slots */
#define STATUS_PERIOD_MS         100U
#define FEEDBACK_PERIOD_MS       100U
#define BOARD1_POINT_TIMEOUT_MS  20U

#define MICROSTEP                16LL
#define EXTERNAL_CMD_SLOTS_PER_POINT 4U

/* Board1 local Motor ID 0..3 = actual arm axes 2~5.
 * Integrated protocol parameters:
 *   Motor 0 / arm 2-axis: gear 20,  motor 200 full-steps/rev, home -90.00 deg
 *   Motor 1 / arm 3-axis: gear 50,  motor 200 full-steps/rev, home -80.00 deg
 *   Motor 2 / arm 4-axis: gear 30,  motor 200 full-steps/rev, home -90.00 deg
 *   Motor 3 / arm 5-axis: gear 120, motor 48  full-steps/rev, home -170.00 deg
 */
static const int64_t g_gear_ratio[BOARD1_MOTOR_COUNT] = {20LL, 50LL, 30LL, 120LL};
static const int64_t g_motor_steps_per_rev[BOARD1_MOTOR_COUNT] = {200LL, 200LL, 200LL, 48LL};
static const int32_t g_home_raw_001deg[BOARD1_MOTOR_COUNT] = {-9000, -8000, -9000, -17000};
static const int32_t g_min_raw_001deg[BOARD1_MOTOR_COUNT]  = {-9000, -8000, -9000, -17000};
static const int32_t g_max_raw_001deg[BOARD1_MOTOR_COUNT]  = { 9000,  8000,  9000,  17000};

/* 0x101 Byte0 control flags */
#define CTRL_EXECUTE             0x80U
#define CTRL_RELATIVE            0x40U
#define CTRL_STEP_MODE           0x20U
#define CTRL_RESERVED            0x10U
#define CTRL_MOTOR_MASK          0x0FU

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
#define ERR_LIMIT_DETECTED       2U
#define ERR_DRIVER_FAULT         3U
#define ERR_HOMING_FAIL          4U
#define ERR_QUEUE_FULL           5U
#define ERR_RESERVED             6U

/* ============================================================
 *  Board1 real STEP/DIR + TMC5160 support pin map
 * ============================================================
 * This block follows the motor-control team pin map received on
 * 2026-07-01.
 *
 * Motor driver / TMC5160 side:
 *   Motor 1: STEP=PA1,  DIR=PA0,  CS=PA5
 *   Motor 2: STEP=PC15, DIR=PC14, CS=PA4
 *   Motor 3: STEP=PB9,  DIR=PB8,  CS=PB10
 *   Motor 4: STEP=PB7,  DIR=PB6,  CS=PB2
 *   TMC SPI: MOSI=PB1, MISO=PB0, CLK=PA6
 *   MOTOR_ENABLE=PB3, active-low by default
 *
 * Limit switch pins received later:
 *   base / 1-axis limit = PA7  -> Board2, not Board1
 *   2-axis limit        = PA15 -> Board1 local motor 0 / arm 2-axis
 *   3-axis limit        = PB4  -> Board1 local motor 1 / arm 3-axis
 *   4-axis limit        = PB12 -> Board1 local motor 2 / arm 4-axis
 *
 * Board1 still needs the 5-axis limit switch pin for local motor 3.
 * Until that pin is defined, 0x020 homing is rejected safely with
 * ERR_HOMING_FAIL instead of moving motor 3 forever without a limit input.
 *
 * Limit switch default assumption when enabled:
 *   GPIO input ---- switch COM
 *   GND        ---- switch NO
 *   STM32 internal pull-up enabled
 *   normal = 1, pressed = 0
 */
#define BOARD1_LIMIT_SWITCHES_ASSIGNED 1U
#define BOARD1_LIMIT_SWITCH_ASSIGNED_MASK 0x07U  /* bit0=PA15, bit1=PB4, bit2=PB12, bit3=not assigned yet */
#define BOARD1_HOMING_REQUIRED_LIMIT_MASK 0x0FU
#define LIMIT_SWITCH_ACTIVE_LEVEL      0U
#define LIMIT_SWITCH_DEBOUNCE_TICKS    5U      /* 5 ms because TIM3 scheduler is 1 ms */
#define STEP_PULSE_DELAY_LOOPS         10U
#define HW_MAX_STEP_PULSES_PER_MS      50U     /* low-level safety cap for direct GPIO pulse generation */
#define MOTOR_ENABLE_ACTIVE_LEVEL      0U      /* TMC EN is usually active-low */

#define DIR_POSITIVE                   1
#define DIR_NEGATIVE                  -1

typedef struct
{
    GPIO_TypeDef *step_port;
    uint8_t step_pin;
    GPIO_TypeDef *dir_port;
    uint8_t dir_pin;
    GPIO_TypeDef *lim_port;
    uint8_t lim_pin;
    GPIO_TypeDef *driver_cs_port;
    uint8_t driver_cs_pin;
} StepperHwPin_t;

static const StepperHwPin_t g_stepper_hw[BOARD1_MOTOR_COUNT] = {
    /* motor 0: arm 2-axis */ {GPIOA, 1U, GPIOA, 0U, GPIOA,15U, GPIOA,  5U},
    /* motor 1: arm 3-axis */ {GPIOC,15U, GPIOC,14U, GPIOB, 4U, GPIOA,  4U},
    /* motor 2: arm 4-axis */ {GPIOB, 9U, GPIOB, 8U, GPIOB,12U, GPIOB, 10U},
    /* motor 3: arm 5-axis */ {GPIOB, 7U, GPIOB, 6U, (GPIO_TypeDef *)0, 0U, GPIOB,  2U},
};

#define TMC_SPI_MOSI_PORT GPIOB
#define TMC_SPI_MOSI_PIN  1U
#define TMC_SPI_MISO_PORT GPIOB
#define TMC_SPI_MISO_PIN  0U
#define TMC_SPI_CLK_PORT  GPIOA
#define TMC_SPI_CLK_PIN   6U
#define MOTOR_ENABLE_PORT GPIOB
#define MOTOR_ENABLE_PIN  3U

static const int8_t g_home_dir[BOARD1_MOTOR_COUNT] = {
    DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE, DIR_NEGATIVE
};

static volatile uint16_t g_limit_debounce[BOARD1_MOTOR_COUNT] = {0U, 0U, 0U, 0U};

typedef struct
{
    int32_t  target_step[BOARD1_MOTOR_COUNT];
    uint16_t speed[BOARD1_MOTOR_COUNT];
    uint16_t duration_ms;
    uint8_t  relative_mask;
    uint8_t  step_mode_mask;
} TrajectoryPoint4_t;

typedef struct
{
    uint8_t active;
    uint8_t expected_motor_id;
    uint8_t duration_5ms;
    uint32_t start_ms;
    TrajectoryPoint4_t point;
} Staging4_t;

static volatile uint32_t g_ms_tick = 0U;
static volatile uint8_t  g_status_event = 0U;

static volatile uint8_t  g_state = STATE_INIT;
static volatile uint8_t  g_error_code = ERR_NONE;
static volatile uint8_t  g_homing_done_bits = 0U;
static volatile uint8_t  g_limit_status_bits = 0U;
static volatile uint8_t  g_enabled = 0U;
static volatile uint8_t  g_moving_motor_id = MOVING_MOTOR_NONE;

static TrajectoryPoint4_t g_queue[TRAJ_QUEUE_SIZE];
static volatile uint8_t  g_q_head = 0U;
static volatile uint8_t  g_q_tail = 0U;
static volatile uint8_t  g_q_count = 0U;

static volatile uint8_t  g_motion_active = 0U;
static TrajectoryPoint4_t g_active_point;
static volatile uint32_t g_motion_elapsed_ms = 0U;
static volatile uint32_t g_motion_duration_ms = 0U;
static volatile int32_t  g_segment_start_step[BOARD1_MOTOR_COUNT] = {0, 0, 0, 0};
static volatile int32_t  g_current_step[BOARD1_MOTOR_COUNT] = {0, 0, 0, 0};
static volatile int32_t  g_target_step[BOARD1_MOTOR_COUNT] = {0, 0, 0, 0};

static volatile uint8_t  g_homing_active = 0U;
static volatile uint16_t g_homing_remaining_ms = 0U;

static Staging4_t g_staging;


static void queue_clear(void);
static void staging_reset(void);
static void stop_motion_only(void);
static void request_status_event(void);
static void enter_error(uint8_t error_code);

static void delay_loop(volatile unsigned int n)
{
    while (n--) { __NOP(); }
}

static void led_direct_init(void)
{
    /* STM32F411CEU6 BlackPill-style LED: PC13, active-low */
    RCC->AHB1ENR |= (1U << 2);
    (void)RCC->AHB1ENR;
    GPIOC->MODER &= ~(0x3U << (13U * 2U));
    GPIOC->MODER |=  (0x1U << (13U * 2U));
    GPIOC->OTYPER &= ~(1U << 13U);
    GPIOC->ODR |= (1U << 13U);       /* OFF */
}

static void led_direct_toggle(void)
{
    GPIOC->ODR ^= (1U << 13U);
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
    if (port == (GPIO_TypeDef *)0) return;
    gpio_enable_clock(port);
    port->MODER &= ~(0x3U << ((uint32_t)pin * 2U));
    port->MODER |=  (0x1U << ((uint32_t)pin * 2U));
    port->OTYPER &= ~(1U << pin);
    port->OSPEEDR |= (0x2U << ((uint32_t)pin * 2U));
    port->PUPDR &= ~(0x3U << ((uint32_t)pin * 2U));
}

static void gpio_config_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    if (port == (GPIO_TypeDef *)0) return;
    gpio_enable_clock(port);
    port->MODER &= ~(0x3U << ((uint32_t)pin * 2U));
    port->PUPDR &= ~(0x3U << ((uint32_t)pin * 2U));
    port->PUPDR |=  (0x1U << ((uint32_t)pin * 2U));
}

static void gpio_config_input_nopull(GPIO_TypeDef *port, uint8_t pin)
{
    if (port == (GPIO_TypeDef *)0) return;
    gpio_enable_clock(port);
    port->MODER &= ~(0x3U << ((uint32_t)pin * 2U));
    port->PUPDR &= ~(0x3U << ((uint32_t)pin * 2U));
}

static void gpio_write(GPIO_TypeDef *port, uint8_t pin, uint8_t high)
{
    if (port == (GPIO_TypeDef *)0) return;
    if (high) port->BSRR = (1U << pin);
    else      port->BSRR = (1U << (pin + 16U));
}

static uint8_t gpio_read(GPIO_TypeDef *port, uint8_t pin)
{
    if (port == (GPIO_TypeDef *)0) return 1U;
    return (port->IDR & (1U << pin)) ? 1U : 0U;
}

static void motor_enable_hw(uint8_t enable)
{
    uint8_t level;
    level = enable ? MOTOR_ENABLE_ACTIVE_LEVEL : (uint8_t)(1U - MOTOR_ENABLE_ACTIVE_LEVEL);
    gpio_write(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, level);
}

static void tmc_gpio_init(void)
{
    uint8_t i;

    gpio_config_output_pp(TMC_SPI_MOSI_PORT, TMC_SPI_MOSI_PIN);
    gpio_config_input_nopull(TMC_SPI_MISO_PORT, TMC_SPI_MISO_PIN);
    gpio_config_output_pp(TMC_SPI_CLK_PORT,  TMC_SPI_CLK_PIN);
    gpio_write(TMC_SPI_MOSI_PORT, TMC_SPI_MOSI_PIN, 0U);
    gpio_write(TMC_SPI_CLK_PORT,  TMC_SPI_CLK_PIN,  0U);

    gpio_config_output_pp(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
    motor_enable_hw(0U);

    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        gpio_config_output_pp(g_stepper_hw[i].driver_cs_port, g_stepper_hw[i].driver_cs_pin);
        gpio_write(g_stepper_hw[i].driver_cs_port, g_stepper_hw[i].driver_cs_pin, 1U);
    }
}

static void stepper_outputs_low_all(void)
{
    uint8_t i;
    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        gpio_write(g_stepper_hw[i].step_port, g_stepper_hw[i].step_pin, 0U);
    }
}

static void stepper_hw_init(void)
{
    uint8_t i;

    tmc_gpio_init();

    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        gpio_config_output_pp(g_stepper_hw[i].step_port, g_stepper_hw[i].step_pin);
        gpio_config_output_pp(g_stepper_hw[i].dir_port,  g_stepper_hw[i].dir_pin);
#if BOARD1_LIMIT_SWITCHES_ASSIGNED
        if (g_stepper_hw[i].lim_port != (GPIO_TypeDef *)0)
        {
            gpio_config_input_pullup(g_stepper_hw[i].lim_port, g_stepper_hw[i].lim_pin);
        }
#endif
        gpio_write(g_stepper_hw[i].step_port, g_stepper_hw[i].step_pin, 0U);
        gpio_write(g_stepper_hw[i].dir_port,  g_stepper_hw[i].dir_pin,  0U);
        g_limit_debounce[i] = 0U;
    }
}

static uint8_t limit_switch_pressed_raw(uint8_t motor_id)
{
    uint8_t v;
    if (motor_id >= BOARD1_MOTOR_COUNT) return 0U;
#if BOARD1_LIMIT_SWITCHES_ASSIGNED
    if (g_stepper_hw[motor_id].lim_port == (GPIO_TypeDef *)0) return 0U;
    v = gpio_read(g_stepper_hw[motor_id].lim_port, g_stepper_hw[motor_id].lim_pin);
    return (v == LIMIT_SWITCH_ACTIVE_LEVEL) ? 1U : 0U;
#else
    (void)v;
    return 0U;
#endif
}

static uint8_t limit_switch_pressed_stable(uint8_t motor_id)
{
    if (motor_id >= BOARD1_MOTOR_COUNT) return 0U;

    if (limit_switch_pressed_raw(motor_id))
    {
        if (g_limit_debounce[motor_id] < LIMIT_SWITCH_DEBOUNCE_TICKS)
        {
            g_limit_debounce[motor_id]++;
        }
    }
    else
    {
        g_limit_debounce[motor_id] = 0U;
    }

    return (g_limit_debounce[motor_id] >= LIMIT_SWITCH_DEBOUNCE_TICKS) ? 1U : 0U;
}

static uint8_t stepper_limit_status_bits_hw(void)
{
    uint8_t bits = 0U;
    uint8_t i;
    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        if (limit_switch_pressed_raw(i)) bits |= (uint8_t)(1U << i);
    }
    return bits;
}

static uint8_t is_home_direction(uint8_t motor_id, int8_t dir)
{
    if (motor_id >= BOARD1_MOTOR_COUNT) return 0U;
    return (dir == g_home_dir[motor_id]) ? 1U : 0U;
}

static void enter_limit_error(uint8_t motor_id)
{
    (void)motor_id;
    queue_clear();
    staging_reset();
    stop_motion_only();
    g_homing_active = 0U;
    g_limit_status_bits = stepper_limit_status_bits_hw();
    enter_error(ERR_LIMIT_DETECTED);
}

static uint8_t stepper_step_once(uint8_t motor_id, int8_t dir)
{
    if (motor_id >= BOARD1_MOTOR_COUNT) return 0U;
    if (dir == 0) return 0U;

    /* Limit safety policy:
     * - Moving toward the home/limit direction while the switch is active is blocked.
     * - Moving away from the home switch is allowed, so the axis can release the switch
     *   after homing without immediately re-entering limit error.
     */
    if (is_home_direction(motor_id, dir) && limit_switch_pressed_stable(motor_id))
    {
        enter_limit_error(motor_id);
        return 0U;
    }

    gpio_write(g_stepper_hw[motor_id].dir_port,
               g_stepper_hw[motor_id].dir_pin,
               (dir > 0) ? 1U : 0U);

    gpio_write(g_stepper_hw[motor_id].step_port, g_stepper_hw[motor_id].step_pin, 1U);
    delay_loop(STEP_PULSE_DELAY_LOOPS);
    gpio_write(g_stepper_hw[motor_id].step_port, g_stepper_hw[motor_id].step_pin, 0U);

    if (dir > 0) g_current_step[motor_id]++;
    else         g_current_step[motor_id]--;

    return 1U;
}

static void stepper_drive_axis_toward(uint8_t motor_id, int32_t desired_step)
{
    uint16_t pulses = 0U;

    while (motor_id < BOARD1_MOTOR_COUNT &&
           g_current_step[motor_id] != desired_step &&
           pulses < HW_MAX_STEP_PULSES_PER_MS &&
           g_state != STATE_ERROR &&
           g_state != STATE_ESTOP)
    {
        int8_t dir = (g_current_step[motor_id] < desired_step) ? DIR_POSITIVE : DIR_NEGATIVE;
        if (!stepper_step_once(motor_id, dir)) break;
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

static int32_t angle_001deg_to_step(uint8_t motor_id, int32_t angle_raw)
{
    int64_t steps;

    /* Board1 formula:
     * step = angle_raw * gear_ratio[motor_id] * motor_steps_per_rev[motor_id] * 16 / 36000
     * angle_raw unit = 0.01 degree
     */
    steps = ((int64_t)angle_raw *
             g_gear_ratio[motor_id] *
             g_motor_steps_per_rev[motor_id] *
             MICROSTEP) / 36000LL;
    return clamp_i64_to_i32(steps);
}

static uint8_t queue_free_slots(void)
{
    /* Status Byte5 reports external 0x101 command slots.
     * Internal queue stores 8 four-axis points; each point uses four 0x101 frames.
     * Therefore Queue Free range is 0..32, not 0..8.
     */
    if (g_q_count >= TRAJ_QUEUE_SIZE) return 0U;
    return (uint8_t)((TRAJ_QUEUE_SIZE - g_q_count) * EXTERNAL_CMD_SLOTS_PER_POINT);
}

static void queue_clear(void)
{
    g_q_head = 0U;
    g_q_tail = 0U;
    g_q_count = 0U;
}

static int queue_push(const TrajectoryPoint4_t *p)
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

static int queue_pop(TrajectoryPoint4_t *p)
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

static void staging_reset(void)
{
    uint8_t i;

    g_staging.active = 0U;
    g_staging.expected_motor_id = 0U;
    g_staging.duration_5ms = 0U;
    g_staging.start_ms = 0U;
    g_staging.point.duration_ms = 0U;
    g_staging.point.relative_mask = 0U;
    g_staging.point.step_mode_mask = 0U;
    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        g_staging.point.target_step[i] = 0;
        g_staging.point.speed[i] = 0U;
    }
}

static uint8_t first_changed_motor_id(void)
{
    uint8_t i;

    if (g_state == STATE_HOMING && g_homing_active) return 0U;

    if (g_motion_active)
    {
        for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
        {
            if (g_current_step[i] != g_target_step[i]) return i;
        }
        return 0U;   /* segment active but all targets are hold positions */
    }

    return MOVING_MOTOR_NONE;
}

static void send_status(void)
{
    CAN_Frame_t st;
    uint8_t i;

    g_limit_status_bits = stepper_limit_status_bits_hw();

    st.id = CAN_ID_BOARD1_STATUS;
    st.dlc = 8U;
    for (i = 0U; i < 8U; i++) st.data[i] = 0U;

    st.data[0] = g_state;
    st.data[1] = g_error_code;
    st.data[2] = g_homing_done_bits & 0x0FU;
    st.data[3] = first_changed_motor_id();
    st.data[4] = g_limit_status_bits & 0x0FU;
    st.data[5] = queue_free_slots();
    st.data[6] = g_enabled ? 1U : 0U;
    st.data[7] = 0U;

    (void)MCP2515_Send_Frame(&st);
}


static int32_t step_to_angle_001deg_i32(uint8_t motor_id, int32_t step)
{
    int64_t denom;
    int64_t raw;

    if (motor_id >= BOARD1_MOTOR_COUNT) return 0;

    /* Inverse of:
     * step = angle_raw * gear_ratio[motor_id] * motor_steps_per_rev[motor_id] * 16 / 36000
     * angle_raw unit = 0.01 degree, output-axis angle for MoveIt2 joint_states.
     */
    denom = g_gear_ratio[motor_id] * g_motor_steps_per_rev[motor_id] * MICROSTEP;
    if (denom == 0LL) return 0;

    raw = ((int64_t)step * 36000LL) / denom;
    return clamp_i64_to_i32(raw);
}

static uint8_t board1_angle_raw_within_limit(uint8_t motor_id, int32_t angle_raw)
{
    if (motor_id >= BOARD1_MOTOR_COUNT) return 0U;

    if (angle_raw < g_min_raw_001deg[motor_id]) return 0U;
    if (angle_raw > g_max_raw_001deg[motor_id]) return 0U;

    return 1U;
}

static uint8_t board1_target_step_within_limit(uint8_t motor_id, int32_t target_step)
{
    int32_t final_angle_raw;

    if (motor_id >= BOARD1_MOTOR_COUNT) return 0U;

    final_angle_raw = step_to_angle_001deg_i32(motor_id, target_step);
    return board1_angle_raw_within_limit(motor_id, final_angle_raw);
}

static void write_i32_le(uint8_t *d, int32_t v)
{
    uint32_t u = (uint32_t)v;
    d[0] = (uint8_t)(u & 0xFFU);
    d[1] = (uint8_t)((u >> 8) & 0xFFU);
    d[2] = (uint8_t)((u >> 16) & 0xFFU);
    d[3] = (uint8_t)((u >> 24) & 0xFFU);
}

static uint8_t make_position_flags(uint8_t motor_id,
                                   uint8_t state_snapshot,
                                   uint8_t error_snapshot,
                                   uint8_t enabled_snapshot,
                                   uint8_t homing_done_snapshot,
                                   uint8_t motion_active_snapshot,
                                   const int32_t current_step_snapshot[BOARD1_MOTOR_COUNT],
                                   const int32_t target_step_snapshot[BOARD1_MOTOR_COUNT])
{
    uint8_t flags = 0U;
    uint8_t position_valid;
    uint8_t homed_ready;
    uint8_t moving;
    uint8_t target_reached;

    if (motor_id >= BOARD1_MOTOR_COUNT) return 0U;

    position_valid = ((homing_done_snapshot & (uint8_t)(1U << motor_id)) != 0U) ? 1U : 0U;
    homed_ready = (position_valid && enabled_snapshot &&
                   state_snapshot != STATE_ESTOP &&
                   state_snapshot != STATE_ERROR &&
                   error_snapshot == ERR_NONE) ? 1U : 0U;
    moving = (motion_active_snapshot &&
              current_step_snapshot[motor_id] != target_step_snapshot[motor_id]) ? 1U : 0U;
    target_reached = (position_valid && !moving &&
                      state_snapshot != STATE_HOMING &&
                      current_step_snapshot[motor_id] == target_step_snapshot[motor_id]) ? 1U : 0U;

    if (position_valid)  flags |= 0x01U; /* bit0: position valid */
    if (homed_ready)     flags |= 0x02U; /* bit1: homed / ready */
    if (moving)          flags |= 0x04U; /* bit2: moving */
    if (target_reached)  flags |= 0x08U; /* bit3: target reached */

    return flags;
}

static uint8_t make_position_error_code(uint8_t motor_id,
                                        uint8_t error_snapshot,
                                        uint8_t limit_snapshot)
{
    if (motor_id >= BOARD1_MOTOR_COUNT) return ERR_INVALID_CMD;

    if ((limit_snapshot & (uint8_t)(1U << motor_id)) != 0U)
    {
        return ERR_LIMIT_DETECTED;
    }

    return error_snapshot;
}

static void send_position_feedback(void)
{
    CAN_Frame_t fb;
    int32_t current_snapshot[BOARD1_MOTOR_COUNT];
    int32_t target_snapshot[BOARD1_MOTOR_COUNT];
    int32_t angle_raw[BOARD1_MOTOR_COUNT];
    uint8_t state_snapshot;
    uint8_t error_snapshot;
    uint8_t enabled_snapshot;
    uint8_t homing_done_snapshot;
    uint8_t motion_active_snapshot;
    uint8_t limit_snapshot;
    uint8_t motor_id;
    static uint8_t seq_counter = 0U;

    g_limit_status_bits = stepper_limit_status_bits_hw();

    __disable_irq();
    for (motor_id = 0U; motor_id < BOARD1_MOTOR_COUNT; motor_id++)
    {
        current_snapshot[motor_id] = g_current_step[motor_id];
        target_snapshot[motor_id] = g_target_step[motor_id];
    }
    state_snapshot = g_state;
    error_snapshot = g_error_code;
    enabled_snapshot = g_enabled;
    homing_done_snapshot = g_homing_done_bits;
    motion_active_snapshot = g_motion_active;
    limit_snapshot = g_limit_status_bits;
    __enable_irq();

    for (motor_id = 0U; motor_id < BOARD1_MOTOR_COUNT; motor_id++)
    {
        angle_raw[motor_id] = step_to_angle_001deg_i32(motor_id, current_snapshot[motor_id]);
    }

    fb.id = CAN_ID_BOARD1_FEEDBACK;
    fb.dlc = 8U;

    /* Board1 sends four 0x301 frames every 100 ms:
     * local Motor ID 0 -> 1 -> 2 -> 3.
     * Payload:
     *   Byte0    = Local Motor ID
     *   Byte1    = flags
     *   Byte2~5  = current_pos_001deg, int32 little-endian
     *   Byte6    = error/fault code, 0 if none
     *   Byte7    = sequence counter
     */
    for (motor_id = 0U; motor_id < BOARD1_MOTOR_COUNT; motor_id++)
    {
        fb.data[0] = motor_id;
        fb.data[1] = make_position_flags(motor_id, state_snapshot, error_snapshot,
                                         enabled_snapshot, homing_done_snapshot,
                                         motion_active_snapshot, current_snapshot,
                                         target_snapshot);
        write_i32_le(&fb.data[2], angle_raw[motor_id]);
        fb.data[6] = make_position_error_code(motor_id, error_snapshot, limit_snapshot);
        fb.data[7] = seq_counter++;
        (void)MCP2515_Send_Frame(&fb);
    }
}

static void request_status_event(void)
{
    g_status_event = 1U;
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
    staging_reset();
    request_status_event();
}

static void emergency_stop(void)
{
    __disable_irq();
    queue_clear();
    staging_reset();
    stop_motion_only();
    g_homing_active = 0U;
    g_homing_remaining_ms = 0U;
    g_enabled = 0U;
    g_state = STATE_ESTOP;
    g_error_code = ERR_NONE;
    __enable_irq();

    stepper_outputs_low_all();
    motor_enable_hw(0U);
    printf("BOARD1 ESTOP\n");
    send_status();
}

static uint8_t is_motion_allowed(void)
{
    if (!g_enabled) return 0U;
    if (g_state == STATE_ESTOP) return 0U;
    if (g_state == STATE_ERROR) return 0U;
    if (g_state == STATE_HOMING) return 0U;
    if (g_error_code != ERR_NONE) return 0U;
    if ((g_homing_done_bits & 0x0FU) != 0x0FU) return 0U;
    return 1U;
}

static void start_next_motion_if_possible(void)
{
    TrajectoryPoint4_t p;
    uint8_t i;

    if (g_motion_active) return;
    if (!is_motion_allowed()) return;

    if (queue_pop(&p) != 0)
    {
        g_state = STATE_IDLE;
        g_moving_motor_id = MOVING_MOTOR_NONE;
        return;
    }

    g_active_point = p;
    g_motion_elapsed_ms = 0U;
    g_motion_duration_ms = (p.duration_ms == 0U) ? 1U : p.duration_ms;

    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        g_segment_start_step[i] = g_current_step[i];
        g_target_step[i] = p.target_step[i];
    }

    g_motion_active = 1U;
    g_state = STATE_MOVING;
    g_moving_motor_id = first_changed_motor_id();

    printf("BOARD1 POINT START start=[%ld,%ld,%ld,%ld] target=[%ld,%ld,%ld,%ld] dur=%lu q_free=%u\n",
           (long)g_segment_start_step[0], (long)g_segment_start_step[1],
           (long)g_segment_start_step[2], (long)g_segment_start_step[3],
           (long)p.target_step[0], (long)p.target_step[1],
           (long)p.target_step[2], (long)p.target_step[3],
           (unsigned long)g_motion_duration_ms,
           queue_free_slots());
}

static void complete_active_motion_if_ready(void)
{
    uint8_t i;
    uint8_t all_reached = 1U;

    if (!g_motion_active) return;

    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        if (g_current_step[i] != g_active_point.target_step[i])
        {
            all_reached = 0U;
            break;
        }
    }

    if (g_motion_elapsed_ms >= g_motion_duration_ms && all_reached)
    {
        for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
        {
            g_target_step[i] = g_active_point.target_step[i];
        }

        g_motion_active = 0U;
        g_moving_motor_id = MOVING_MOTOR_NONE;

        printf("BOARD1 POINT DONE current=[%ld,%ld,%ld,%ld] q=%u\n",
               (long)g_current_step[0], (long)g_current_step[1],
               (long)g_current_step[2], (long)g_current_step[3],
               g_q_count);

        if (g_q_count == 0U)
        {
            g_state = STATE_IDLE;
            request_status_event();
        }
        else
        {
            start_next_motion_if_possible();
        }
    }
}

static void trajectory_tick_1ms(void)
{
    uint8_t i;

    g_limit_status_bits = stepper_limit_status_bits_hw();

    if (g_state == STATE_ERROR || g_state == STATE_ESTOP)
    {
        stepper_outputs_low_all();
        return;
    }

    if (g_state == STATE_HOMING && g_homing_active)
    {
        for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
        {
            if ((g_homing_done_bits & (uint8_t)(1U << i)) != 0U)
            {
                continue;
            }

            if (limit_switch_pressed_stable(i))
            {
                int32_t home_step = angle_001deg_to_step(i, g_home_raw_001deg[i]);
                g_current_step[i] = home_step;
                g_target_step[i] = home_step;
                g_segment_start_step[i] = home_step;
                g_homing_done_bits |= (uint8_t)(1U << i);
                gpio_write(g_stepper_hw[i].step_port, g_stepper_hw[i].step_pin, 0U);
                continue;
            }

            (void)stepper_step_once(i, g_home_dir[i]);
        }

        if ((g_homing_done_bits & 0x0FU) == 0x0FU)
        {
            g_homing_active = 0U;
            g_state = STATE_IDLE;
            printf("BOARD1 HOMING DONE motors=0..3\n");
            request_status_event();
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

    for (i = 0U; i < BOARD1_MOTOR_COUNT; i++)
    {
        int64_t start = (int64_t)g_segment_start_step[i];
        int64_t target = (int64_t)g_active_point.target_step[i];
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

        stepper_drive_axis_toward(i, clamp_i64_to_i32(value));
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

static uint8_t board1_motor_target_matches(uint8_t target_motor)
{
    if (target_motor == MOTOR_ALL_ID) return 1U;
    return 0U;
}

static uint8_t frame_is_exact_8_bytes(const CAN_Frame_t *frame)
{
    return (frame->dlc == 8U) ? 1U : 0U;
}

static uint8_t reserved_zero(const CAN_Frame_t *frame, uint8_t start_index)
{
    uint8_t i;

    if (frame->dlc > 8U) return 0U;

    for (i = start_index; i < frame->dlc; i++)
    {
        if (frame->data[i] != 0U) return 0U;
    }

    return 1U;
}

static void handle_enable_disable(const CAN_Frame_t *frame)
{
    uint8_t cmd;

    /* Final integrated protocol:
     *   010#0100000000000000 = Enable
     *   010#0000000000000000 = Disable
     *   Byte1~7 must be 0. No payload Board ID is allowed.
     */
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("BOARD1 ENABLE/DISABLE invalid final-format dlc=%u\n", frame->dlc);
        send_status();
        return;
    }

    cmd = frame->data[0];

    if (cmd == 1U)
    {
        __disable_irq();
        g_enabled = 1U;
        motor_enable_hw(1U);
        g_error_code = ERR_NONE;
        if (g_state == STATE_ESTOP || g_state == STATE_ERROR ||
            g_state == STATE_INIT || g_state == STATE_DISABLED)
        {
            g_state = STATE_IDLE;
        }
        __enable_irq();
        printf("BOARD1 ENABLE final-format\n");
    }
    else if (cmd == 0U)
    {
        __disable_irq();
        queue_clear();
        staging_reset();
        stop_motion_only();
        g_homing_active = 0U;
        g_homing_remaining_ms = 0U;
        g_enabled = 0U;
        motor_enable_hw(0U);
        g_state = STATE_DISABLED;
        __enable_irq();
        printf("BOARD1 DISABLE final-format\n");
    }
    else
    {
        enter_error(ERR_INVALID_CMD);
        printf("BOARD1 ENABLE/DISABLE invalid cmd=%u\n", cmd);
    }

    send_status();
}

static void handle_arm_homing(const CAN_Frame_t *frame)
{
    uint8_t target_motor;
    uint8_t homing_mode;

    /* Final integrated protocol:
     *   020#FF00000000000000
     *   Byte0 = 0xFF, Byte1 = 0, Byte2~7 = 0.
     *   No payload Board ID is allowed.
     */
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 2U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("BOARD1 HOMING invalid final-format dlc=%u\n", frame->dlc);
        send_status();
        return;
    }

    target_motor = frame->data[0];
    homing_mode = frame->data[1];

    if (!board1_motor_target_matches(target_motor) || homing_mode != 0U)
    {
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    if (!g_enabled || g_state == STATE_ESTOP || g_state == STATE_ERROR)
    {
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

#if BOARD1_LIMIT_SWITCHES_ASSIGNED == 0U
    enter_error(ERR_HOMING_FAIL);
    printf("BOARD1 HOMING rejected: no limit switch pins are assigned\n");
    send_status();
    return;
#endif

    if ((BOARD1_LIMIT_SWITCH_ASSIGNED_MASK & BOARD1_HOMING_REQUIRED_LIMIT_MASK) !=
        BOARD1_HOMING_REQUIRED_LIMIT_MASK)
    {
        enter_error(ERR_HOMING_FAIL);
        printf("BOARD1 HOMING rejected: 5-axis limit switch pin is not assigned yet. assigned_mask=0x%02X required=0x%02X\n",
               BOARD1_LIMIT_SWITCH_ASSIGNED_MASK, BOARD1_HOMING_REQUIRED_LIMIT_MASK);
        send_status();
        return;
    }

    __disable_irq();
    queue_clear();
    staging_reset();
    stop_motion_only();
    g_homing_done_bits = 0U;
    g_homing_active = 1U;
    g_homing_remaining_ms = 0U;
    for (target_motor = 0U; target_motor < BOARD1_MOTOR_COUNT; target_motor++)
    {
        g_limit_debounce[target_motor] = 0U;
    }
    g_state = STATE_HOMING;
    g_error_code = ERR_NONE;
    __enable_irq();

    printf("BOARD1 HOMING START motors=0..3\n");
    send_status();
}

static void handle_clear_error(const CAN_Frame_t *frame)
{
    uint8_t target_motor;

    /* Final integrated protocol:
     *   030#FF00000000000000
     *   Byte0 = 0xFF, Byte1~7 = 0.
     *   No payload Board ID is allowed.
     */
    if (!frame_is_exact_8_bytes(frame) || !reserved_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("BOARD1 CLEAR ERROR invalid final-format dlc=%u\n", frame->dlc);
        send_status();
        return;
    }

    target_motor = frame->data[0];

    if (!board1_motor_target_matches(target_motor))
    {
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    __disable_irq();
    g_error_code = ERR_NONE;
    staging_reset();
    if (g_state != STATE_ESTOP)
    {
        if (g_enabled) g_state = STATE_IDLE;
        else g_state = STATE_DISABLED;
    }
    __enable_irq();

    printf("BOARD1 CLEAR ERROR\n");
    send_status();
}

static int32_t convert_payload_to_step(uint8_t motor_id,
                                       int32_t target_raw,
                                       uint8_t relative,
                                       uint8_t step_mode)
{
    int32_t step;

    if (step_mode)
    {
        step = target_raw;
    }
    else
    {
        step = angle_001deg_to_step(motor_id, target_raw);
    }

    if (relative)
    {
        step = clamp_i64_to_i32((int64_t)g_current_step[motor_id] + (int64_t)step);
    }

    return step;
}

static void validate_staging_timeout_before_rx(void)
{
    uint32_t age;

    if (!g_staging.active) return;

    age = g_ms_tick - g_staging.start_ms;
    if (age > BOARD1_POINT_TIMEOUT_MS)
    {
        printf("BOARD1 STAGING TIMEOUT age=%lu expected=%u\n",
               (unsigned long)age,
               g_staging.expected_motor_id);
        enter_error(ERR_INVALID_CMD);
    }
}

static void handle_board1_move(const CAN_Frame_t *frame)
{
    uint8_t b0;
    uint8_t motor_id;
    uint8_t execute;
    uint8_t relative;
    uint8_t step_mode;
    uint8_t duration_5ms;
    int32_t target_raw;
    uint16_t speed;
    int32_t target_step_value;
    uint32_t age;

    if (frame->dlc != 8U)
    {
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    if (!is_motion_allowed())
    {
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    validate_staging_timeout_before_rx();
    if (g_state == STATE_ERROR)
    {
        send_status();
        return;
    }

    b0 = frame->data[0];
    execute = (b0 & CTRL_EXECUTE) ? 1U : 0U;
    relative = (b0 & CTRL_RELATIVE) ? 1U : 0U;
    step_mode = (b0 & CTRL_STEP_MODE) ? 1U : 0U;
    motor_id = b0 & CTRL_MOTOR_MASK;

    if (!execute || (b0 & CTRL_RESERVED) || motor_id >= BOARD1_MOTOR_COUNT)
    {
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    target_raw = read_i32_le(&frame->data[1]);
    speed = read_u16_le(&frame->data[5]);
    duration_5ms = frame->data[7];
    target_step_value = convert_payload_to_step(motor_id, target_raw, relative, step_mode);

    if (!board1_target_step_within_limit(motor_id, target_step_value))
    {
        int32_t final_angle_raw = step_to_angle_001deg_i32(motor_id, target_step_value);
        enter_error(ERR_INVALID_CMD);
        printf("BOARD1 MOVE rejected by joint limit motor=%u final_raw=%ld min=%ld max=%ld\n",
               motor_id,
               (long)final_angle_raw,
               (long)g_min_raw_001deg[motor_id],
               (long)g_max_raw_001deg[motor_id]);
        send_status();
        return;
    }

    if (motor_id == 0U)
    {
        if (g_staging.active)
        {
            printf("BOARD1 STAGING RESTART REJECT expected=%u\n", g_staging.expected_motor_id);
            enter_error(ERR_INVALID_CMD);
            send_status();
            return;
        }

        staging_reset();
        g_staging.active = 1U;
        g_staging.expected_motor_id = 1U;
        g_staging.duration_5ms = duration_5ms;
        g_staging.start_ms = g_ms_tick;
        g_staging.point.duration_ms = (duration_5ms == 0U) ? 1U : ((uint16_t)duration_5ms * 5U);
        g_staging.point.target_step[0] = target_step_value;
        g_staging.point.speed[0] = speed;
        if (relative) g_staging.point.relative_mask |= 0x01U;
        if (step_mode) g_staging.point.step_mode_mask |= 0x01U;

        printf("BOARD1 STAGE motor=0 target=%ld duration5=%u\n",
               (long)target_step_value, duration_5ms);
        return;
    }

    if (!g_staging.active)
    {
        printf("BOARD1 STAGING MISSING motor=%u\n", motor_id);
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    age = g_ms_tick - g_staging.start_ms;
    if (age > BOARD1_POINT_TIMEOUT_MS)
    {
        printf("BOARD1 STAGING TIMEOUT age=%lu motor=%u\n",
               (unsigned long)age, motor_id);
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    if (motor_id != g_staging.expected_motor_id)
    {
        printf("BOARD1 STAGING ORDER ERROR got=%u expected=%u\n",
               motor_id, g_staging.expected_motor_id);
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    if (duration_5ms != g_staging.duration_5ms)
    {
        printf("BOARD1 STAGING DURATION MISMATCH got=%u expected=%u\n",
               duration_5ms, g_staging.duration_5ms);
        enter_error(ERR_INVALID_CMD);
        send_status();
        return;
    }

    g_staging.point.target_step[motor_id] = target_step_value;
    g_staging.point.speed[motor_id] = speed;
    if (relative) g_staging.point.relative_mask |= (uint8_t)(1U << motor_id);
    if (step_mode) g_staging.point.step_mode_mask |= (uint8_t)(1U << motor_id);

    printf("BOARD1 STAGE motor=%u target=%ld age=%lu\n",
           motor_id,
           (long)target_step_value,
           (unsigned long)age);

    if (motor_id < 3U)
    {
        g_staging.expected_motor_id = (uint8_t)(motor_id + 1U);
        return;
    }

    /* Motor ID 3 completed the 4-frame trajectory point. Queue push happens now. */
    if (queue_push(&g_staging.point) != 0)
    {
        enter_error(ERR_QUEUE_FULL);
        send_status();
        return;
    }

    printf("BOARD1 POINT QUEUED target=[%ld,%ld,%ld,%ld] duration_ms=%u q_free=%u\n",
           (long)g_staging.point.target_step[0],
           (long)g_staging.point.target_step[1],
           (long)g_staging.point.target_step[2],
           (long)g_staging.point.target_step[3],
           g_staging.point.duration_ms,
           queue_free_slots());

    staging_reset();
    request_status_event();
}

static void handle_can_frame(const CAN_Frame_t *frame)
{
    switch (frame->id)
    {
    case CAN_ID_ESTOP:
        if (frame_is_exact_8_bytes(frame) && frame->data[0] == 1U && reserved_zero(frame, 1U))
        {
            emergency_stop();
        }
        else
        {
            enter_error(ERR_INVALID_CMD);
            printf("BOARD1 ESTOP invalid final-format dlc=%u\n", frame->dlc);
            send_status();
        }
        break;

    case CAN_ID_ENABLE_DISABLE:
        handle_enable_disable(frame);
        break;

    case CAN_ID_ARM_HOMING:
        handle_arm_homing(frame);
        break;

    case CAN_ID_CLEAR_ERROR:
        handle_clear_error(frame);
        break;

    case CAN_ID_BOARD1_MOVE:
        handle_board1_move(frame);
        break;

    default:
        break;
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & 0x1U)
    {
        TIM3->SR = (uint16_t)~0x1U;
        g_ms_tick++;
        trajectory_tick_1ms();
    }
}

void Main(void)
{
    CAN_Frame_t rx;
    uint32_t last_status_ms;
    uint32_t last_feedback_ms;
    int init_ret;

    led_direct_init();
    blink_direct(3, 400000U);

    Sys_Init(115200);
    printf("\nBoard1 STM32F411CEU6 arm axes 2~5 trajectory CAN test start\n");
    printf("MCP2515 pins: SCK=PB13 MISO=PB14 MOSI=PB15 CS=PA9 INT=PA10\n");
    printf("Protocol: 0x101 requires Motor ID 0->1->2->3 within 20ms\n");
    printf("Position feedback: 0x301 sends arm axes 2~5 as Motor ID 0->1->2->3 frames every 100ms\n");

    staging_reset();
    stepper_hw_init();
    printf("Stepper pins: M1 STEP=PA1 DIR=PA0 CS=PA5, M2 STEP=PC15 DIR=PC14 CS=PA4, M3 STEP=PB9 DIR=PB8 CS=PB10, M4 STEP=PB7 DIR=PB6 CS=PB2\n");
    printf("TMC SPI pins: MOSI=PB1 MISO=PB0 CLK=PA6, MOTOR_ENABLE=PB3 active-low.\n");
    printf("Limit pins: arm2(local0)=PA15 arm3(local1)=PB4 arm4(local2)=PB12 arm5(local3)=NOT_ASSIGNED_YET. 0x020 homing is rejected until arm5 limit pin is assigned.\n");

    MCP2515_SPI_Init(64U);
    init_ret = MCP2515_Init(MCP2515_OSC_8MHZ);
    if (init_ret != 0)
    {
        printf("MCP2515 init failed ret=%d. Trying 16MHz config...\n", init_ret);
        init_ret = MCP2515_Init(MCP2515_OSC_16MHZ);
    }

    if (init_ret != 0)
    {
        g_state = STATE_ERROR;
        g_error_code = ERR_DRIVER_FAULT;
        printf("MCP2515 init failed ret=%d\n", init_ret);
    }
    else
    {
        g_state = STATE_DISABLED;
        g_error_code = ERR_NONE;
        printf("MCP2515 init OK\n");
    }

    tim3_1ms_init();
    last_status_ms = g_ms_tick;
    last_feedback_ms = g_ms_tick;
    send_status();
    send_position_feedback();

    while (1)
    {
        /* Use both EXTI flag and level polling fallback. */
        if (g_mcp2515_irq || MCP2515_Int_Asserted())
        {
            g_mcp2515_irq = 0U;

            while (MCP2515_Read_Frame(&rx) > 0)
            {
                printf("RX id=0x%03X dlc=%u data=%02X%02X%02X%02X%02X%02X%02X%02X\n",
                       rx.id,
                       rx.dlc,
                       rx.data[0], rx.data[1], rx.data[2], rx.data[3],
                       rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
                handle_can_frame(&rx);
            }
        }

        if (g_staging.active && ((g_ms_tick - g_staging.start_ms) > BOARD1_POINT_TIMEOUT_MS))
        {
            validate_staging_timeout_before_rx();
        }

        if ((g_ms_tick - last_feedback_ms) >= FEEDBACK_PERIOD_MS)
        {
            last_feedback_ms = g_ms_tick;
            send_position_feedback();
        }

        if ((g_ms_tick - last_status_ms) >= STATUS_PERIOD_MS)
        {
            last_status_ms = g_ms_tick;
            send_status();
        }

        if (g_status_event)
        {
            g_status_event = 0U;
            send_status();
        }
    }
}
