#include "../Inc/config.h"
#include "../Inc/gpio.h"
#include "../Inc/stepper.h"
#include "../Inc/trajectory.h"
#include <stdint.h>

#define UART_BAUD              115200
#define DEBUG_USART            USART2
#define DEBUG_UART_PCLK_HZ     (SYSCLK_HZ / 2)
#define DEBUG_UART_TX_PIN      2
#define DEBUG_UART_RX_PIN      3
#define DEBUG_UART_NAME        "UART2"
#define DEBUG_UART_PINS        "PA2 TX / PA3 RX"
#define UART_RX_WORK_LIMIT     16
#define UART_LINE_MAX          48
#define DEFAULT_MOVE_STEPS     400
#define DEFAULT_DURATION_MS    500
#define MIN_DURATION_MS        5
#define MAX_DURATION_MS        1275
#define DEFAULT_JOG_PULSES     50
#define MAX_JOG_PULSES         10000
#define JOG_STEP_HIGH_US       5
#define JOG_STEP_LOW_US        995
#define TMC_REG_GSTAT          0x01
#define TMC_REG_IOIN           0x04
#define TMC_REG_DRV_CONF       0x0A
#define TMC_REG_GLOBAL_SCALER  0x0B
#define TMC_REG_IHOLD_IRUN     0x10
#define TMC_REG_TPOWERDOWN     0x11
#define TMC_REG_CHOPCONF       0x6C
#define TMC_REG_DRV_STATUS     0x6F

static uint8_t s_uart_enabled;
static uint8_t s_uart_ready;
static char s_line[UART_LINE_MAX];
static uint8_t s_line_len;
static uint8_t s_last_was_cr;
static uint32_t s_last_heartbeat_ms;

static void print_prompt(void);

static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SYSCLK_HZ / 1000000);

    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {}
}

static void gpio_af7(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3 << (pin * 2));
    port->MODER |=  (2 << (pin * 2));
    port->OTYPER &= ~(1 << pin);
    port->OSPEEDR &= ~(3 << (pin * 2));
    port->OSPEEDR |=  (3 << (pin * 2));
    port->PUPDR &= ~(3 << (pin * 2));
    port->PUPDR |=  (1 << (pin * 2));
    port->AFR[pin / 8] &= ~(0xF << ((pin % 8) * 4));
    port->AFR[pin / 8] |=  (0x7 << ((pin % 8) * 4));
}

static void gpio_set_pin(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1 << pin);
}

static void gpio_clear_pin(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1 << (pin + 16));
}

static void uart2_putc_raw(char ch)
{
    while ((DEBUG_USART->SR & USART_SR_TXE) == 0) {}
    DEBUG_USART->DR = (uint8_t)ch;
}

static void uart2_putc(char ch)
{
    if (!s_uart_enabled || !s_uart_ready) return;
    if (ch == '\n') uart2_putc_raw('\r');
    uart2_putc_raw(ch);
}

static void uart2_puts(const char *str)
{
    while (*str != '\0') {
        uart2_putc(*str++);
    }
}

static uint8_t uart2_readable(void)
{
    return (DEBUG_USART->SR & USART_SR_RXNE) ? 1 : 0;
}

static char uart2_getc(void)
{
    return (char)(DEBUG_USART->DR & 0xFF);
}

int __io_putchar(int ch)
{
    uart2_putc((char)ch);
    return ch;
}

static char lower_char(char ch)
{
    if (ch >= 'A' && ch <= 'Z') return (char)(ch - 'A' + 'a');
    return ch;
}

static uint8_t word_equals(const char *line, const char *word)
{
    while (*line != '\0' && *word != '\0') {
        if (lower_char(*line) != *word) return 0;
        line++;
        word++;
    }
    return (*line == '\0' && *word == '\0') ? 1 : 0;
}

static const char *skip_spaces(const char *str)
{
    while (*str == ' ' || *str == '\t') {
        str++;
    }
    return str;
}

static void trim_trailing_spaces(char *line)
{
    uint8_t len = 0;

    while (line[len] != '\0') {
        len++;
    }
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[--len] = '\0';
    }
}

static uint8_t parse_u32(const char **cursor, uint32_t *value)
{
    const char *p = *cursor;
    uint32_t result = 0;

    if (*p < '0' || *p > '9') return 0;

    while (*p >= '0' && *p <= '9') {
        uint32_t digit = (uint32_t)(*p - '0');
        if (result > ((UINT32_MAX - digit) / 10)) return 0;
        result = (result * 10) + digit;
        p++;
    }

    *cursor = p;
    *value = result;
    return 1;
}

static void uart2_put_u32(uint32_t value)
{
    char buf[10];
    uint8_t len = 0;

    if (value == 0) {
        uart2_putc('0');
        return;
    }

    while (value != 0 && len < (uint8_t)sizeof(buf)) {
        buf[len++] = (char)('0' + (value % 10));
        value /= 10;
    }
    while (len > 0) {
        uart2_putc(buf[--len]);
    }
}

static void uart2_put_i32(int32_t value)
{
    if (value < 0) {
        uart2_putc('-');
        uart2_put_u32((uint32_t)(-(value + 1)) + 1);
    } else {
        uart2_put_u32((uint32_t)value);
    }
}

static void uart2_put_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    uart2_putc('0');
    uart2_putc('x');
    uart2_putc(hex[(value >> 4) & 0x0F]);
    uart2_putc(hex[value & 0x0F]);
}

static void uart2_put_hex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    uart2_puts("0x");
    for (int8_t shift = 28; shift >= 0; shift -= 4) {
        uart2_putc(hex[(value >> shift) & 0x0F]);
    }
}

static void tmc_uart_cs_low(uint8_t axis_id)
{
#if BOARD_IS_BOARD2_2
    if (axis_id == 0) gpio_clear_pin(CS4_PORT, CS4_PIN);
#else
    if (axis_id == 0) gpio_clear_pin(CS1_PORT, CS1_PIN);
#if AXIS_COUNT > 1
    else if (axis_id == 1) gpio_clear_pin(CS2_PORT, CS2_PIN);
    else if (axis_id == 2) gpio_clear_pin(CS3_PORT, CS3_PIN);
    else if (axis_id == 3) gpio_clear_pin(CS4_PORT, CS4_PIN);
#endif
#endif
}

static void tmc_uart_cs_high(uint8_t axis_id)
{
#if BOARD_IS_BOARD2_2
    if (axis_id == 0) gpio_set_pin(CS4_PORT, CS4_PIN);
#else
    if (axis_id == 0) gpio_set_pin(CS1_PORT, CS1_PIN);
#if AXIS_COUNT > 1
    else if (axis_id == 1) gpio_set_pin(CS2_PORT, CS2_PIN);
    else if (axis_id == 2) gpio_set_pin(CS3_PORT, CS3_PIN);
    else if (axis_id == 3) gpio_set_pin(CS4_PORT, CS4_PIN);
#endif
#endif
}

static uint8_t tmc_uart_transfer_byte(uint8_t tx)
{
    uint8_t rx = 0;

    for (int8_t bit = 7; bit >= 0; bit--) {
        gpio_clear_pin(TMC_CLK_PORT, TMC_CLK_PIN);
        if (tx & (uint8_t)(1 << bit)) gpio_set_pin(TMC_MOSI_PORT, TMC_MOSI_PIN);
        else gpio_clear_pin(TMC_MOSI_PORT, TMC_MOSI_PIN);
        for (volatile int delay = 0; delay < 20; delay++) {}
        gpio_set_pin(TMC_CLK_PORT, TMC_CLK_PIN);
        if (TMC_MISO_PORT->IDR & (1 << TMC_MISO_PIN)) {
            rx |= (uint8_t)(1 << bit);
        }
        for (volatile int delay = 0; delay < 20; delay++) {}
    }
    return rx;
}

static uint8_t tmc_uart_transfer(uint8_t axis_id, uint8_t addr, uint32_t tx_data, uint32_t *rx_data)
{
    uint8_t status;
    uint32_t rx = 0;

    if (axis_id >= AXIS_COUNT) return 0;

    tmc_uart_cs_low(axis_id);
    status = tmc_uart_transfer_byte(addr);
    rx |= (uint32_t)tmc_uart_transfer_byte((uint8_t)(tx_data >> 24)) << 24;
    rx |= (uint32_t)tmc_uart_transfer_byte((uint8_t)(tx_data >> 16)) << 16;
    rx |= (uint32_t)tmc_uart_transfer_byte((uint8_t)(tx_data >> 8)) << 8;
    rx |= (uint32_t)tmc_uart_transfer_byte((uint8_t)tx_data);
    tmc_uart_cs_high(axis_id);

    if (rx_data != 0) *rx_data = rx;
    return status;
}

static uint8_t tmc_uart_read(uint8_t axis_id, uint8_t reg, uint32_t *value)
{
    (void)tmc_uart_transfer(axis_id, (uint8_t)(reg & 0x7F), 0, 0);
    return tmc_uart_transfer(axis_id, (uint8_t)(reg & 0x7F), 0, value);
}

static void print_tmc_reg(uint8_t axis_id, const char *name, uint8_t reg)
{
    uint32_t value;
    uint8_t status = tmc_uart_read(axis_id, reg, &value);

    uart2_puts(name);
    uart2_putc('=');
    uart2_put_hex32(value);
    uart2_puts(" spi_status=");
    uart2_put_hex8(status);
    uart2_puts("\n");
}

static void print_tmc_status(void)
{
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        uart2_puts("TMC axis");
        uart2_put_u32((uint32_t)axis + 1);
        uart2_puts("\n");
        print_tmc_reg(axis, "  GSTAT", TMC_REG_GSTAT);
        print_tmc_reg(axis, "  IOIN", TMC_REG_IOIN);
        print_tmc_reg(axis, "  DRV_STATUS", TMC_REG_DRV_STATUS);
        print_tmc_reg(axis, "  CHOPCONF", TMC_REG_CHOPCONF);
#if BOARD_ID == BOARD_ID_BOARD1
        if (axis == 3) {
            print_tmc_reg(axis, "  DRV_CONF", TMC_REG_DRV_CONF);
            print_tmc_reg(axis, "  GLOBAL_SCALER", TMC_REG_GLOBAL_SCALER);
            print_tmc_reg(axis, "  IHOLD_IRUN", TMC_REG_IHOLD_IRUN);
            print_tmc_reg(axis, "  TPOWERDOWN", TMC_REG_TPOWERDOWN);
        }
#endif
    }
    print_prompt();
}

static void print_prompt(void)
{
    uart2_puts("> ");
}

static void print_help(void)
{
    uart2_puts("\n");
    uart2_puts(DEBUG_UART_NAME);
    uart2_puts(" debug, ");
    uart2_puts(DEBUG_UART_PINS);
    uart2_puts(", 115200 8N1\n");
    uart2_puts("Commands:\n");
    uart2_puts("  ?              help\n");
    uart2_puts("  p              print status\n");
    uart2_puts("  e              enable motor drivers\n");
    uart2_puts("  d              disable motor drivers\n");
    uart2_puts("  c              clear error\n");
    uart2_puts("  home           start all-axis homing\n");
    uart2_puts("  home1          start single-axis homing\n");
    uart2_puts("  s              stop queued/current motion\n");
    uart2_puts("  x              estop");
    if (!ENABLE_ESTOP_LOGIC) uart2_puts(" disabled");
    uart2_puts("\n");
    uart2_puts("  tmc            print TMC status registers\n");
    uart2_puts("  1+400 500      relative step move, duration ms optional\n");
    uart2_puts("  j1+100         raw low-speed jog before homing, press s to stop\n");
    print_prompt();
}

static uint8_t motion_command_allowed(void)
{
    if (!g_enabled) return 0;
    if (ESTOP_ACTIVE()) return 0;
    if (g_error_code != ERR_NONE) return 0;
    if (g_homing_active) return 0;
    if (!system_all_homed()) return 0;
    return 1;
}

static void print_status(void)
{
    int32_t current[AXIS_COUNT];
    int32_t target[AXIS_COUNT];
    uint32_t tick_snapshot;
    uint8_t state_snapshot;
    uint8_t error_snapshot;
    uint8_t enabled_snapshot;
    uint8_t estop_snapshot;
    uint8_t homed_snapshot;
    uint8_t limit_snapshot;

    __disable_irq();
    tick_snapshot = global_tick_ms;
    state_snapshot = g_state;
    error_snapshot = g_error_code;
    enabled_snapshot = g_enabled;
    estop_snapshot = g_estop;
    homed_snapshot = g_homing_done_bits;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        current[i] = g_current_step[i];
        target[i] = g_target_step[i];
    }
    __enable_irq();

    limit_snapshot = stepper_limit_switch_status_bits();

    uart2_puts("ms=");
    uart2_put_u32(tick_snapshot);
    uart2_puts(" state=");
    uart2_put_u32(state_snapshot);
    uart2_puts(" err=");
    uart2_put_u32(error_snapshot);
    uart2_puts(" enabled=");
    uart2_put_u32(enabled_snapshot);
    uart2_puts(" estop=");
    uart2_put_u32(estop_snapshot);
    uart2_puts(" homed=");
    uart2_put_hex8((uint8_t)(homed_snapshot & ((1 << AXIS_COUNT) - 1)));
    uart2_puts(" limits=");
    uart2_put_hex8(limit_snapshot);
    uart2_puts("\n");

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        uart2_puts("axis");
        uart2_put_u32((uint32_t)i + 1);
        uart2_puts(" cur_step=");
        uart2_put_i32(current[i]);
        uart2_puts(" tgt_step=");
        uart2_put_i32(target[i]);
        uart2_puts(" cur_angle=");
        uart2_put_i32(step_to_angle(i, current[i]));
        uart2_puts(" tgt_angle=");
        uart2_put_i32(step_to_angle(i, target[i]));
        uart2_puts("\n");
    }
    print_prompt();
}

static void handle_enable(void)
{
    g_estop = 0;
    g_error_code = ERR_NONE;
    g_enabled = 1;
    motor_enable();
    if (g_state == STATE_ESTOP || g_state == STATE_ERROR ||
        g_state == STATE_INIT || g_state == STATE_DISABLED) {
        g_state = STATE_IDLE;
    }
    uart2_puts("OK enabled\n");
    print_prompt();
}

static void handle_disable(void)
{
    trajectory_clear();
    stepper_stop_all();
    g_homing_active = 0;
    g_enabled = 0;
    motor_disable();
    g_state = STATE_DISABLED;
    uart2_puts("OK disabled\n");
    print_prompt();
}

static void handle_clear_error(void)
{
    g_error_code = ERR_NONE;
    trajectory_cancel_staging();
    if (!ESTOP_ACTIVE()) {
        g_state = g_enabled ? STATE_IDLE : STATE_DISABLED;
    }
    uart2_puts("OK clear error\n");
    print_prompt();
}

static void handle_home(void)
{
    if (!g_enabled || ESTOP_ACTIVE() || g_error_code != ERR_NONE) {
        uart2_puts("ERR: enable first and clear estop/error\n");
        print_prompt();
        return;
    }

    g_error_code = ERR_NONE;
    trajectory_clear();
    stepper_stop_all();
    stepper_start_homing_all();
    uart2_puts("OK homing\n");
    print_prompt();
}

static void handle_home_axis(uint8_t axis)
{
    if (axis >= AXIS_COUNT) {
        uart2_puts("ERR: bad home axis\n");
        print_prompt();
        return;
    }
    if (!g_enabled || ESTOP_ACTIVE() || g_error_code != ERR_NONE) {
        uart2_puts("ERR: enable first and clear estop/error\n");
        print_prompt();
        return;
    }

    trajectory_clear();
    stepper_stop_all();
    stepper_start_homing(axis);
    uart2_puts("OK homing axis");
    uart2_put_u32((uint32_t)axis + 1);
    uart2_puts("\n");
    print_prompt();
}

static void handle_stop(void)
{
    trajectory_clear();
    stepper_stop_all();
    if (g_enabled && !ESTOP_ACTIVE() && g_error_code == ERR_NONE) {
        g_state = STATE_IDLE;
    }
    uart2_puts("OK stop\n");
    print_prompt();
}

static void handle_estop(void)
{
#if ENABLE_ESTOP_LOGIC
    g_estop = 1;
    g_enabled = 0;
    g_error_code = ERR_NONE;
    g_homing_active = 0;
    g_state = STATE_ESTOP;
    trajectory_clear();
    stepper_stop_all();
    motor_disable();
    uart2_puts("OK estop\n");
#else
    uart2_puts("OK estop ignored\n");
#endif
    print_prompt();
}

static void raw_set_dir(uint8_t axis, int8_t dir)
{
    uint8_t positive = (dir > 0) ? 1 : 0;

#if BOARD_IS_BOARD2_2
    if (axis == 0) {
        if (positive) gpio_set_pin(DIR4_PORT, DIR4_PIN);
        else gpio_clear_pin(DIR4_PORT, DIR4_PIN);
    }
#else
    if (axis == 0) {
        if (positive) gpio_set_pin(DIR1_PORT, DIR1_PIN);
        else gpio_clear_pin(DIR1_PORT, DIR1_PIN);
    }
#if AXIS_COUNT > 1
    else if (axis == 1) {
        if (positive) gpio_clear_pin(DIR2_PORT, DIR2_PIN);
        else gpio_set_pin(DIR2_PORT, DIR2_PIN);
    } else if (axis == 2) {
        if (positive) gpio_clear_pin(DIR3_PORT, DIR3_PIN);
        else gpio_set_pin(DIR3_PORT, DIR3_PIN);
    } else if (axis == 3) {
        if (positive) gpio_set_pin(DIR4_PORT, DIR4_PIN);
        else gpio_clear_pin(DIR4_PORT, DIR4_PIN);
    }
#endif
#endif
}

static void raw_step_high(uint8_t axis)
{
#if BOARD_IS_BOARD2_2
    if (axis == 0) gpio_set_pin(STEP4_PORT, STEP4_PIN);
#else
    if (axis == 0) gpio_set_pin(STEP1_PORT, STEP1_PIN);
#if AXIS_COUNT > 1
    else if (axis == 1) gpio_set_pin(STEP2_PORT, STEP2_PIN);
    else if (axis == 2) gpio_set_pin(STEP3_PORT, STEP3_PIN);
    else if (axis == 3) gpio_set_pin(STEP4_PORT, STEP4_PIN);
#endif
#endif
}

static void raw_step_low(uint8_t axis)
{
#if BOARD_IS_BOARD2_2
    if (axis == 0) gpio_clear_pin(STEP4_PORT, STEP4_PIN);
#else
    if (axis == 0) gpio_clear_pin(STEP1_PORT, STEP1_PIN);
#if AXIS_COUNT > 1
    else if (axis == 1) gpio_clear_pin(STEP2_PORT, STEP2_PIN);
    else if (axis == 2) gpio_clear_pin(STEP3_PORT, STEP3_PIN);
    else if (axis == 3) gpio_clear_pin(STEP4_PORT, STEP4_PIN);
#endif
#endif
}

static uint8_t raw_jog_stop_requested(void)
{
    if (!uart2_readable()) return 0;

    char ch = uart2_getc();
    if (lower_char(ch) == 's') {
        uart2_puts("\nSTOP raw jog\n");
        return 1;
    }
    return 0;
}

static uint8_t raw_delay_us_interruptible(uint32_t wait_us)
{
    uint32_t elapsed_us = 0;

    while (elapsed_us < wait_us) {
        uint32_t slice_us = wait_us - elapsed_us;
        if (slice_us > 10) slice_us = 10;

        if (raw_jog_stop_requested()) return 1;
        if (ESTOP_ACTIVE() || g_error_code != ERR_NONE) return 1;
        delay_us(slice_us);
        elapsed_us += slice_us;
    }
    return 0;
}

static void handle_raw_jog_command(const char *line)
{
    const char *p = &line[3];
    uint8_t axis;
    int8_t dir;
    uint32_t pulses = DEFAULT_JOG_PULSES;
    uint8_t stopped = 0;

    if (line[1] < '1' || line[1] > (char)('0' + AXIS_COUNT) ||
        (line[2] != '+' && line[2] != '-')) {
        uart2_puts("ERR: use j1+100 or j1-100\n");
        print_prompt();
        return;
    }

    if (*p >= '0' && *p <= '9') {
        if (!parse_u32(&p, &pulses)) {
            uart2_puts("ERR: bad jog pulse count\n");
            print_prompt();
            return;
        }
    }
    p = skip_spaces(p);
    if (*p != '\0') {
        uart2_puts("ERR: trailing input\n");
        print_prompt();
        return;
    }

    if (pulses == 0 || pulses > MAX_JOG_PULSES) {
        uart2_puts("ERR: jog pulse range 1..");
        uart2_put_u32(MAX_JOG_PULSES);
        uart2_puts("\n");
        print_prompt();
        return;
    }
    if (!g_enabled) {
        uart2_puts("ERR: send e first\n");
        print_prompt();
        return;
    }
    if (ESTOP_ACTIVE() || g_error_code != ERR_NONE) {
        uart2_puts("ERR: clear estop/error first\n");
        print_prompt();
        return;
    }
    if (g_homing_active || g_motion_active) {
        uart2_puts("ERR: stop homing/motion first\n");
        print_prompt();
        return;
    }

    axis = (uint8_t)(line[1] - '1');
    dir = (line[2] == '+') ? DIR_POSITIVE : DIR_NEGATIVE;
    trajectory_clear();
    raw_set_dir(axis, dir);

    uart2_puts("RAW jog axis");
    uart2_put_u32((uint32_t)axis + 1);
    uart2_putc(line[2]);
    uart2_put_u32(pulses);
    uart2_puts(" pulses, press s to stop\n");

    for (uint32_t i = 0; i < pulses; i++) {
        if (raw_jog_stop_requested()) {
            stopped = 1;
            break;
        }
        raw_step_high(axis);
        if (raw_delay_us_interruptible(JOG_STEP_HIGH_US)) {
            stopped = 1;
        }
        raw_step_low(axis);
        if (stopped || raw_delay_us_interruptible(JOG_STEP_LOW_US)) {
            stopped = 1;
            break;
        }

        __disable_irq();
        if (dir > 0) g_current_step[axis]++;
        else g_current_step[axis]--;
        g_target_step[axis] = g_current_step[axis];
        g_motion_start_step[axis] = g_current_step[axis];
        __enable_irq();
    }

    raw_step_low(axis);
    if (stopped) uart2_puts("OK raw jog stopped\n");
    else uart2_puts("OK raw jog done\n");
    print_prompt();
}

static void handle_move_command(const char *line)
{
    const char *p = &line[2];
    int32_t targets[AXIS_COUNT];
    int32_t selected_target;
    uint32_t steps = DEFAULT_MOVE_STEPS;
    uint32_t duration_ms = DEFAULT_DURATION_MS;
    uint8_t axis = (uint8_t)(line[0] - '1');
    uint8_t duration_5ms;
    uint8_t result = TRAJECTORY_STAGING_WAITING;
    int32_t delta;

    if (axis >= AXIS_COUNT) {
        uart2_puts("ERR: bad axis\n");
        print_prompt();
        return;
    }

    if (*p >= '0' && *p <= '9') {
        if (!parse_u32(&p, &steps)) {
            uart2_puts("ERR: bad step count\n");
            print_prompt();
            return;
        }
    }

    p = skip_spaces(p);
    if (*p != '\0') {
        if (!parse_u32(&p, &duration_ms)) {
            uart2_puts("ERR: bad duration\n");
            print_prompt();
            return;
        }
        p = skip_spaces(p);
        if (*p != '\0') {
            uart2_puts("ERR: trailing input\n");
            print_prompt();
            return;
        }
    }

    if (steps == 0 || steps > (uint32_t)INT32_MAX) {
        uart2_puts("ERR: step range\n");
        print_prompt();
        return;
    }
    if (duration_ms < MIN_DURATION_MS || duration_ms > MAX_DURATION_MS) {
        uart2_puts("ERR: duration range 5..1275 ms\n");
        print_prompt();
        return;
    }
    if (!motion_command_allowed()) {
        uart2_puts("ERR: need enabled, homed, no estop/error\n");
        print_prompt();
        return;
    }

    delta = (line[1] == '+') ? (int32_t)steps : -(int32_t)steps;
    if (!trajectory_resolve_target_step(axis, delta, 1, 1, &selected_target)) {
        uart2_puts("ERR: target out of range\n");
        print_prompt();
        return;
    }

    __disable_irq();
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        targets[i] = g_current_step[i];
    }
    __enable_irq();
    targets[axis] = selected_target;

    duration_5ms = (uint8_t)((duration_ms + 4) / 5);
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        result = trajectory_add_axis_command(i, targets[i], 0, duration_5ms);
        if (result == TRAJECTORY_STAGING_INVALID) {
            uart2_puts("ERR: invalid move\n");
            print_prompt();
            return;
        }
        if (result == TRAJECTORY_STAGING_QUEUE_FULL) {
            uart2_puts("ERR: queue full\n");
            print_prompt();
            return;
        }
    }

    if (result == TRAJECTORY_STAGING_COMMITTED) {
        uart2_puts("OK move axis");
        uart2_put_u32((uint32_t)axis + 1);
        uart2_putc(line[1]);
        uart2_put_u32(steps);
        uart2_puts(" duration=");
        uart2_put_u32((uint32_t)duration_5ms * 5);
        uart2_puts("ms\n");
    } else {
        uart2_puts("ERR: move not committed\n");
    }
    print_prompt();
}

static void run_command(char *line)
{
    line = (char *)skip_spaces(line);
    trim_trailing_spaces(line);

    if (line[0] == '\0') {
        print_prompt();
        return;
    }

    if ((line[0] == '?' || lower_char(line[0]) == 'h') && line[1] == '\0') {
        print_help();
        return;
    }
    if (lower_char(line[0]) == 'p' && line[1] == '\0') {
        print_status();
        return;
    }
    if (lower_char(line[0]) == 'e' && line[1] == '\0') {
        handle_enable();
        return;
    }
    if (lower_char(line[0]) == 'd' && line[1] == '\0') {
        handle_disable();
        return;
    }
    if (lower_char(line[0]) == 'c' && line[1] == '\0') {
        handle_clear_error();
        return;
    }
    if (word_equals(line, "home")) {
        handle_home();
        return;
    }
    if (lower_char(line[0]) == 'h' &&
        lower_char(line[1]) == 'o' &&
        lower_char(line[2]) == 'm' &&
        lower_char(line[3]) == 'e' &&
        line[4] >= '1' &&
        line[4] <= (char)('0' + AXIS_COUNT) &&
        line[5] == '\0') {
        handle_home_axis((uint8_t)(line[4] - '1'));
        return;
    }
    if (lower_char(line[0]) == 'j') {
        handle_raw_jog_command(line);
        return;
    }
    if (lower_char(line[0]) == 's' && line[1] == '\0') {
        handle_stop();
        return;
    }
    if (lower_char(line[0]) == 'x' && line[1] == '\0') {
        handle_estop();
        return;
    }
    if (word_equals(line, "tmc")) {
        print_tmc_status();
        return;
    }
    if (line[0] >= '1' && line[0] <= (char)('0' + AXIS_COUNT) &&
        (line[1] == '+' || line[1] == '-')) {
        handle_move_command(line);
        return;
    }

    uart2_puts("ERR: unknown command, send ?\n");
    print_prompt();
}

static void handle_rx_char(char ch)
{
    if (ch == '\n' && s_last_was_cr) {
        s_last_was_cr = 0;
        return;
    }
    s_last_was_cr = (ch == '\r') ? 1 : 0;

    if (ch == '\r' || ch == '\n') {
        uart2_puts("\n");
        s_line[s_line_len] = '\0';
        run_command(s_line);
        s_line_len = 0;
        return;
    }

    if (ch == '\b' || ch == 0x7F) {
        if (s_line_len > 0) {
            s_line_len--;
            uart2_puts("\b \b");
        }
        return;
    }

    if (ch < 0x20 || ch > 0x7E) return;

    if (s_line_len < (UART_LINE_MAX - 1)) {
        s_line[s_line_len++] = ch;
        uart2_putc(ch);
    } else {
        uart2_putc('\a');
    }
}

void uart_debug_init(uint8_t enabled)
{
    s_uart_enabled = enabled ? 1 : 0;
    if (!s_uart_enabled) return;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    gpio_af7(GPIOA, DEBUG_UART_TX_PIN);
    gpio_af7(GPIOA, DEBUG_UART_RX_PIN);
    dwt_delay_init();

    DEBUG_USART->CR1 = 0;
    DEBUG_USART->CR2 = 0;
    DEBUG_USART->CR3 = 0;
    DEBUG_USART->BRR = (DEBUG_UART_PCLK_HZ + (UART_BAUD / 2)) / UART_BAUD;
    DEBUG_USART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    s_uart_ready = 1;
}

void uart_debug_print_ready(uint8_t enabled)
{
    if (!enabled || !s_uart_enabled || !s_uart_ready) return;

    s_last_heartbeat_ms = global_tick_ms;
    uart2_puts("\n");
    uart2_puts(DEBUG_UART_NAME);
    uart2_puts(" debug ready, send ? for help\n");
    print_prompt();
}

void uart_debug_print_loop_ready(uint8_t enabled)
{
    if (!enabled || !s_uart_enabled || !s_uart_ready) return;

    uart2_puts("\ncommand loop ready\n");
    print_prompt();
}

void uart_debug_service(uint8_t enabled)
{
    if (!enabled || !s_uart_enabled || !s_uart_ready) return;

    for (uint8_t i = 0; i < UART_RX_WORK_LIMIT && uart2_readable(); i++) {
        handle_rx_char(uart2_getc());
    }

}
