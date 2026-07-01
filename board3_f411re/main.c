#include "device_driver.h"
#include "gripper_shared.h"
#include <stdint.h>
#include <stdio.h>

/* ============================================================
 * Board3 Gripper CAN Protocol v1.1 Pre-Hardware Firmware + 0x303 Position Feedback
 * Target board: STM32F411RE + MCP2515
 * ============================================================
 *
 * 목적:
 *   실제 SCS0009 서보/그리퍼를 연결하지 않은 상태에서
 *   Board3 CAN 프로토콜 로직만 테스트한다.
 *
 * 구현된 기능:
 *   - MCP2515 SPI2 기반 CAN 송수신
 *   - 0x103 gripper command 수신
 *   - 0x020 Arm Homing은 Board3에서 무시
 *   - 0x023 Gripper Home 수신 시 전체 Motor ID 0~8을 0도로 이동하는 home command 생성
 *   - Motor ID 0~8, 총 9개 frame staging
 *   - 중복 Motor ID 검사
 *   - staging timeout 검사
 *   - duration 동일성 검사
 *   - 정상 command set만 g_cmd로 전달
 *   - 가상 그리퍼 제어 처리: 실제 Feetech 호출 대신 UART debug 출력
 *   - 0x203 status 100ms 주기 송신 및 이벤트 즉시 송신
 *   - 0x303 current position feedback 20ms 주기 송신
 *     · 3개 frame으로 Motor ID 0~8 현재 위치를 int16_t 0.01도 단위 전송
 *
 * 하드웨어 연결 전 테스트 범위:
 *   - CAN frame이 정상적으로 수신되는지
 *   - 9개 frame이 하나의 command set으로 묶이는지
 *   - 잘못된 command set이 폐기되는지
 *   - 0x203 status payload가 프로토콜대로 나가는지
 *   - 0x303 position feedback이 20ms마다 3개 frame으로 나가는지
 *
 * MCP2515 wiring:
 *   SCK  -> PB13 / SPI2_SCK
 *   MISO -> PB14 / SPI2_MISO
 *   MOSI -> PB15 / SPI2_MOSI
 *   CS   -> PB12 / GPIO
 *   INT  -> PB4  / EXTI4 active-low + polling fallback
 *
 * Debug:
 *   USART2 PA2/PA3, 115200 baud
 *   LED PA5 active-high
 */

/* 공유 변수 정의: gripper_shared.h에서 extern으로 선언됨 */
volatile GripperCommand g_cmd;
volatile GripperState   g_state;

typedef struct
{
    int32_t  target_pos_001deg;
    uint16_t speed;
    uint8_t  duration_5ms;
    uint8_t  valid;
} GripperStagingSlot_t;

static GripperStagingSlot_t g_staging[GRIPPER_MOTOR_COUNT];

static volatile uint32_t g_ms_tick = 0U;
static volatile uint8_t  g_status_event = 0U;
static volatile uint8_t  g_position_event = 0U;

static uint8_t  g_staging_active = 0U;
static uint32_t g_staging_start_ms = 0U;

static void delay_loop(volatile unsigned int n)
{
    while (n--) { __NOP(); }
}

static void led_direct_init(void)
{
    /* STM32F411RE Nucleo user LED LD2 is PA5 active-high.
     * Off = low, On = high.
     */
    RCC->AHB1ENR |= (1U << 0);  /* GPIOA clock */
    (void)RCC->AHB1ENR;
    GPIOA->MODER &= ~(0x3U << (5U * 2U));
    GPIOA->MODER |=  (0x1U << (5U * 2U));
    GPIOA->OTYPER &= ~(1U << 5U);
    GPIOA->ODR &= ~(1U << 5U);  /* LED off */
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

static void Sys_Init(int baud)
{
    SCB->CPACR |= (0x3U << (10U * 2U)) | (0x3U << (11U * 2U));
    Clock_Init();
    Uart2_Init(baud);
    setvbuf(stdout, NULL, _IONBF, 0);
    LED_Init();
}

static int32_t read_i32_le(const uint8_t *d)
{
    uint32_t u = ((uint32_t)d[0]) |
                 ((uint32_t)d[1] << 8) |
                 ((uint32_t)d[2] << 16) |
                 ((uint32_t)d[3] << 24);
    return (int32_t)u;
}

static uint16_t read_u16_le(const uint8_t *d)
{
    return (uint16_t)(((uint16_t)d[0]) | ((uint16_t)d[1] << 8));
}

static int16_t clamp_i32_to_i16(int32_t v)
{
    if (v > 32767L) return 32767;
    if (v < -32768L) return (int16_t)-32768;
    return (int16_t)v;
}

static void write_i16_le(uint8_t *d, int16_t v)
{
    uint16_t u = (uint16_t)v;
    d[0] = (uint8_t)(u & 0xFFU);
    d[1] = (uint8_t)((u >> 8) & 0xFFU);
}

static uint32_t elapsed_ms(uint32_t start, uint32_t now)
{
    return (uint32_t)(now - start);
}

static uint8_t staging_count_now(void)
{
    uint8_t i;
    uint8_t count = 0U;

    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        if (g_staging[i].valid) count++;
    }

    return count;
}

static void update_staging_status(void)
{
    uint8_t count = staging_count_now();

    g_state.staging_count = count;
    g_state.buffer_free = GRIPPER_BUFFER_FREE_FROM_COUNT(count);
}

static void request_status_event(void)
{
    g_status_event = 1U;
}

static void staging_clear(void)
{
    uint8_t i;

    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        g_staging[i].target_pos_001deg = 0;
        g_staging[i].speed = 0U;
        g_staging[i].duration_5ms = 0U;
        g_staging[i].valid = 0U;
    }

    g_staging_active = 0U;
    g_staging_start_ms = 0U;
    update_staging_status();
}

static void shared_init(void)
{
    uint8_t i;

    g_cmd.duration_5ms = 0U;
    g_cmd.is_new_cmd = 0U;
    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        g_cmd.target_pos_001deg[i] = 0;
    }

    g_state.state = STATE_DISABLED;
    g_state.error_code = ERR_NONE;
    g_state.ready = 1U;  /* pre-hardware: assume logic is ready */
    g_state.staging_count = 0U;
    g_state.fault = 0U;
    g_state.buffer_free = GRIPPER_MOTOR_COUNT;
    g_state.enabled = 0U;
    g_state.fault_motor_id = GRIPPER_NO_FAULT_MOTOR;

    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        g_state.current_pos[i] = 0;
        g_state.current_pos_001deg[i] = 0;
    }

    staging_clear();
}

static void send_status(void)
{
    CAN_Frame_t st;

    update_staging_status();

    st.id = CAN_ID_BOARD3_STATUS;
    st.dlc = 8U;
    st.data[0] = g_state.state;
    st.data[1] = g_state.error_code;
    st.data[2] = g_state.ready;
    st.data[3] = g_state.staging_count;
    st.data[4] = g_state.fault;
    st.data[5] = g_state.buffer_free;
    st.data[6] = g_state.enabled;
    st.data[7] = g_state.fault_motor_id;

    (void)MCP2515_Send_Frame(&st);

    printf("TX 0x203 state=%u err=%u ready=%u stg=%u fault=%u free=%u en=%u fault_id=%u\n",
           st.data[0], st.data[1], st.data[2], st.data[3],
           st.data[4], st.data[5], st.data[6], st.data[7]);
}

static void send_position_feedback_group(uint8_t group_index, uint8_t base_motor_id)
{
    CAN_Frame_t pf;
    int16_t p0;
    int16_t p1;
    int16_t p2;

    __disable_irq();
    p0 = g_state.current_pos_001deg[base_motor_id + 0U];
    p1 = g_state.current_pos_001deg[base_motor_id + 1U];
    p2 = g_state.current_pos_001deg[base_motor_id + 2U];
    __enable_irq();

    pf.id = CAN_ID_BOARD3_POSITION;
    pf.dlc = 8U;
    pf.data[0] = group_index;
    write_i16_le(&pf.data[1], p0);
    write_i16_le(&pf.data[3], p1);
    write_i16_le(&pf.data[5], p2);
    pf.data[7] = 0U;  /* v1.1: Reserved. 향후 error/status flag로 확장 가능. */

    (void)MCP2515_Send_Frame(&pf);
}

static void send_position_feedback_0x303(void)
{
    /*
     * 20ms마다 1 cycle을 송신한다.
     * CAN ID 0x303, DLC 8, 총 3프레임:
     *   index 0x01: Motor ID 0,1,2
     *   index 0x02: Motor ID 3,4,5
     *   index 0x03: Motor ID 6,7,8
     *
     * 0x203 status와 달리 상태/에러 중심이 아니라 MoveIt2용 현재 위치 전용이다.
     */
    send_position_feedback_group(0x01U, 0U);
    send_position_feedback_group(0x02U, 3U);
    send_position_feedback_group(0x03U, 6U);
}

static void enter_error(uint8_t error_code)
{
    g_state.error_code = error_code;

    if (g_state.state != STATE_ESTOP && g_state.state != STATE_DISABLED)
    {
        g_state.state = STATE_ERROR;
    }

    staging_clear();
    g_cmd.is_new_cmd = 0U;
    request_status_event();
}

static uint8_t frame_is_exact_8_bytes(const CAN_Frame_t *frame)
{
    return (frame->dlc == 8U) ? 1U : 0U;
}

static uint8_t reserved_bytes_are_zero(const CAN_Frame_t *frame, uint8_t start_index)
{
    uint8_t i;

    if (frame->dlc > 8U)
    {
        return 0U;
    }

    for (i = start_index; i < frame->dlc; i++)
    {
        if (frame->data[i] != 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

static void emergency_stop(const CAN_Frame_t *frame)
{
    /* Final integrated protocol only:
     *   001#0100000000000000
     *   Byte0 = 1, Byte1~7 = 0. No payload Board ID is allowed.
     */
    if (!frame_is_exact_8_bytes(frame) ||
        frame->data[0] != 1U ||
        !reserved_bytes_are_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("ESTOP invalid final-format dlc=%u\n", frame->dlc);
        send_status();
        return;
    }

    __disable_irq();
    staging_clear();
    g_cmd.is_new_cmd = 0U;
    g_state.enabled = 0U;
    g_state.state = STATE_ESTOP;
    g_state.error_code = ERR_ESTOP;
    g_state.fault = 0U;
    g_state.fault_motor_id = GRIPPER_NO_FAULT_MOTOR;
    __enable_irq();

    printf("ESTOP\n");
    send_status();
}

static void handle_enable_disable(const CAN_Frame_t *frame)
{
    uint8_t cmd;

    /* Final integrated protocol only:
     *   010#0100000000000000 = Enable
     *   010#0000000000000000 = Disable
     *   Byte1~7 must be 0. No payload Board ID is allowed.
     */
    if (!frame_is_exact_8_bytes(frame) || !reserved_bytes_are_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("ENABLE invalid final-format dlc=%u\n", frame->dlc);
        send_status();
        return;
    }

    cmd = frame->data[0];

    if (cmd == 1U)
    {
        __disable_irq();
        staging_clear();
        g_cmd.is_new_cmd = 0U;
        g_state.enabled = 1U;
        g_state.error_code = ERR_NONE;
        g_state.fault = 0U;
        g_state.fault_motor_id = GRIPPER_NO_FAULT_MOTOR;
        g_state.state = STATE_IDLE;
        __enable_irq();

        printf("ENABLE\n");
    }
    else if (cmd == 0U)
    {
        __disable_irq();
        staging_clear();
        g_cmd.is_new_cmd = 0U;
        g_state.enabled = 0U;
        g_state.state = STATE_DISABLED;
        g_state.error_code = ERR_NONE;
        g_state.fault = 0U;
        g_state.fault_motor_id = GRIPPER_NO_FAULT_MOTOR;
        __enable_irq();

        printf("DISABLE\n");
    }
    else
    {
        enter_error(ERR_INVALID_CMD);
        printf("ENABLE invalid cmd=%u\n", cmd);
    }

    send_status();
}

static void handle_homing_start(const CAN_Frame_t *frame)
{
    (void)frame;
    /* FINAL protocol: 0x020 is Board1/Board2 arm homing broadcast.
     * Board3 must ignore 0x020. Use 0x023 for gripper home posture.
     */
    printf("ARM HOMING 0x020 ignored by Board3\n");
}

static void handle_gripper_home(const CAN_Frame_t *frame)
{
    uint8_t i;
    uint8_t duration_5ms;

    /* FINAL payload:
     *   023#FF00000000000000      -> default 500ms
     *   023#FF00640000000000      -> 100 * 5ms = 500ms explicit
     */
    if (frame->dlc != 8U ||
        frame->data[0] != MOTOR_ALL ||
        frame->data[1] != 0U ||
        !reserved_bytes_are_zero(frame, 3U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("GRIPPER HOME invalid payload\n");
        send_status();
        return;
    }

    if (!g_state.enabled)
    {
        g_state.error_code = ERR_DISABLED;
        g_state.state = STATE_DISABLED;
        staging_clear();
        printf("GRIPPER HOME rejected: disabled\n");
        send_status();
        return;
    }

    if (g_state.state == STATE_ESTOP)
    {
        g_state.error_code = ERR_ESTOP;
        staging_clear();
        printf("GRIPPER HOME rejected: ESTOP\n");
        send_status();
        return;
    }

    if (g_state.state == STATE_ERROR)
    {
        printf("GRIPPER HOME rejected: ERROR state err=%u\n", g_state.error_code);
        send_status();
        return;
    }

    duration_5ms = (frame->data[2] == 0U) ? GRIPPER_HOMING_DURATION_5MS : frame->data[2];

    __disable_irq();
    staging_clear();

    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        g_cmd.target_pos_001deg[i] = GRIPPER_HOME_POSITION_001DEG;
    }

    g_cmd.duration_5ms = duration_5ms;
    g_cmd.is_new_cmd = 1U;

    g_state.state = STATE_MOVING;
    g_state.error_code = ERR_NONE;
    g_state.fault = 0U;
    g_state.fault_motor_id = GRIPPER_NO_FAULT_MOTOR;
    __enable_irq();

    printf("GRIPPER HOME: home_pos_001deg=%ld duration_5ms=%u duration_ms=%u\n",
           (long)GRIPPER_HOME_POSITION_001DEG,
           duration_5ms,
           GRIPPER_DURATION_5MS_TO_MS(duration_5ms));

    send_status();
}

static void handle_clear_error(const CAN_Frame_t *frame)
{
    /* Final integrated protocol only:
     *   030#FF00000000000000
     *   Byte0 = 0xFF, Byte1~7 = 0. No payload Board ID is allowed.
     */
    if (!frame_is_exact_8_bytes(frame) ||
        frame->data[0] != MOTOR_ALL ||
        !reserved_bytes_are_zero(frame, 1U))
    {
        enter_error(ERR_INVALID_CMD);
        printf("CLEAR ERROR invalid final-format dlc=%u\n", frame->dlc);
        send_status();
        return;
    }

    __disable_irq();
    staging_clear();
    g_cmd.is_new_cmd = 0U;
    g_state.error_code = ERR_NONE;
    g_state.fault = 0U;
    g_state.fault_motor_id = GRIPPER_NO_FAULT_MOTOR;

    if (g_state.state == STATE_ERROR)
    {
        g_state.state = g_state.enabled ? STATE_IDLE : STATE_DISABLED;
    }
    /* ESTOP is released by Enable=1, not by Clear Error. */
    __enable_irq();

    printf("CLEAR ERROR state=%u enabled=%u\n", g_state.state, g_state.enabled);
    send_status();
}

static uint8_t all_durations_match(uint8_t *common_duration)
{
    uint8_t i;
    uint8_t d = g_staging[0].duration_5ms;

    for (i = 1U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        if (g_staging[i].duration_5ms != d)
        {
            return 0U;
        }
    }

    *common_duration = d;
    return 1U;
}

static void commit_staging_to_g_cmd(void)
{
    uint8_t i;
    uint8_t common_duration = 0U;

    if (!all_durations_match(&common_duration))
    {
        enter_error(ERR_DURATION_MISMATCH);
        printf("STAGING rejected: duration mismatch\n");
        send_status();
        return;
    }

    if (common_duration == 0U)
    {
        enter_error(ERR_INVALID_CMD);
        printf("STAGING rejected: duration=0\n");
        send_status();
        return;
    }

    __disable_irq();
    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        g_cmd.target_pos_001deg[i] = g_staging[i].target_pos_001deg;
    }
    g_cmd.duration_5ms = common_duration;
    g_cmd.is_new_cmd = 1U;
    g_state.state = STATE_MOVING;
    g_state.error_code = ERR_NONE;
    __enable_irq();

    printf("STAGING OK: committed 9 frames duration_5ms=%u duration_ms=%u\n",
           common_duration,
           GRIPPER_DURATION_5MS_TO_MS(common_duration));

    staging_clear();
    send_status();
}

static void handle_gripper_command(const CAN_Frame_t *frame)
{
    uint8_t byte0;
    uint8_t execute;
    uint8_t reserved;
    uint8_t motor_id;
    int32_t target_pos;
    uint16_t speed;
    uint8_t duration_5ms;
    uint8_t count;

    if (frame->dlc != 8U)
    {
        enter_error(ERR_INVALID_CMD);
        printf("CMD invalid DLC=%u\n", frame->dlc);
        send_status();
        return;
    }

    if (!g_state.enabled)
    {
        g_state.error_code = ERR_DISABLED;
        g_state.state = STATE_DISABLED;
        staging_clear();
        printf("CMD rejected: disabled\n");
        send_status();
        return;
    }

    if (g_state.state == STATE_ESTOP)
    {
        g_state.error_code = ERR_ESTOP;
        staging_clear();
        printf("CMD rejected: ESTOP\n");
        send_status();
        return;
    }

    if (g_state.state == STATE_ERROR)
    {
        printf("CMD rejected: ERROR state err=%u\n", g_state.error_code);
        send_status();
        return;
    }

    byte0 = frame->data[0];
    execute = (byte0 & GRIPPER_CMD_EXECUTE_MASK) ? 1U : 0U;
    reserved = byte0 & GRIPPER_CMD_RESERVED_MASK;
    motor_id = byte0 & GRIPPER_CMD_MOTOR_ID_MASK;

    if (!execute || reserved != 0U)
    {
        enter_error(ERR_INVALID_CMD);
        printf("CMD invalid byte0=0x%02X execute=%u reserved=0x%02X\n",
               byte0, execute, reserved);
        send_status();
        return;
    }

    if (motor_id > GRIPPER_MOTOR_ID_MAX)
    {
        enter_error(ERR_INVALID_MOTOR_ID);
        printf("CMD invalid motor_id=%u\n", motor_id);
        send_status();
        return;
    }

    if (g_staging[motor_id].valid)
    {
        enter_error(ERR_DUPLICATE_MOTOR_ID);
        printf("CMD duplicate motor_id=%u\n", motor_id);
        send_status();
        return;
    }

    target_pos = read_i32_le(&frame->data[1]);
    speed = read_u16_le(&frame->data[5]);
    duration_5ms = frame->data[7];

    if (!g_staging_active)
    {
        g_staging_active = 1U;
        g_staging_start_ms = g_ms_tick;
        g_state.state = STATE_STAGING;
        printf("STAGING START tick=%lu\n", (unsigned long)g_staging_start_ms);
    }

    g_staging[motor_id].target_pos_001deg = target_pos;
    g_staging[motor_id].speed = speed;
    g_staging[motor_id].duration_5ms = duration_5ms;
    g_staging[motor_id].valid = 1U;

    update_staging_status();

    printf("CMD staged motor=%u target_001deg=%ld speed=%u duration_5ms=%u count=%u free=%u\n",
           motor_id,
           (long)target_pos,
           speed,
           duration_5ms,
           g_state.staging_count,
           g_state.buffer_free);

    count = g_state.staging_count;

    if (count >= GRIPPER_MOTOR_COUNT)
    {
        commit_staging_to_g_cmd();
    }
    else
    {
        request_status_event();
    }
}

static void process_gripper_timeout(void)
{
    if (!g_staging_active)
    {
        return;
    }

    if (staging_count_now() >= GRIPPER_MOTOR_COUNT)
    {
        return;
    }

    if (elapsed_ms(g_staging_start_ms, g_ms_tick) > GRIPPER_STAGING_TIMEOUT_MS)
    {
        enter_error(ERR_STAGING_TIMEOUT);
        printf("STAGING TIMEOUT elapsed=%lu ms\n",
               (unsigned long)elapsed_ms(g_staging_start_ms, g_ms_tick));
        send_status();
    }
}

static void gripper_virtual_control_process(void)
{
    GripperCommand local_cmd;
    uint8_t i;

    if (!g_cmd.is_new_cmd)
    {
        return;
    }

    __disable_irq();
    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        local_cmd.target_pos_001deg[i] = g_cmd.target_pos_001deg[i];
    }
    local_cmd.duration_5ms = g_cmd.duration_5ms;
    local_cmd.is_new_cmd = g_cmd.is_new_cmd;
    g_cmd.is_new_cmd = 0U;
    __enable_irq();

    printf("VIRTUAL GRIPPER EXEC duration_5ms=%u duration_ms=%u\n",
           local_cmd.duration_5ms,
           GRIPPER_DURATION_5MS_TO_MS(local_cmd.duration_5ms));

    for (i = 0U; i < GRIPPER_MOTOR_COUNT; i++)
    {
        uint8_t servo_id = GRIPPER_MOTOR_ID_TO_SERVO_ID[i];

        /*
         * 하드웨어 연결 전:
         *   실제 Feetech_Set_Position_Time() 호출 대신 로그만 출력한다.
         *
         * 하드웨어 연결 후 이 위치에서:
         *   1. target_pos_001deg → SCS0009 position 변환
         *   2. Feetech_Set_Position_Time(servo_id, servo_pos, duration_ms) 호출
         */
        printf("  motor=%u servo=%u target_001deg=%ld\n",
               i,
               servo_id,
               (long)local_cmd.target_pos_001deg[i]);

        g_state.current_pos[i] = local_cmd.target_pos_001deg[i];
        g_state.current_pos_001deg[i] = clamp_i32_to_i16(local_cmd.target_pos_001deg[i]);
    }

    __disable_irq();
    if (g_state.state != STATE_ESTOP && g_state.state != STATE_DISABLED)
    {
        g_state.state = STATE_IDLE;
    }
    g_state.error_code = ERR_NONE;
    g_state.ready = 1U;
    g_state.fault = 0U;
    g_state.fault_motor_id = GRIPPER_NO_FAULT_MOTOR;
    __enable_irq();

    send_status();
}

static void process_can_frame(const CAN_Frame_t *frame)
{
    uint8_t i;

    printf("RX 0x%03X DLC=%u D=", frame->id, frame->dlc);
    for (i = 0U; i < frame->dlc; i++) printf("%02X", frame->data[i]);
    printf("\n");

    switch (frame->id)
    {
    case CAN_ID_BOARD3_ESTOP:
        emergency_stop(frame);
        break;

    case CAN_ID_BOARD3_ENABLE:
        handle_enable_disable(frame);
        break;

    case CAN_ID_BOARD3_HOMING:
        handle_homing_start(frame);
        break;

    case CAN_ID_BOARD3_GRIPPER_HOME:
        handle_gripper_home(frame);
        break;

    case CAN_ID_BOARD3_CLEAR_ERROR:
        handle_clear_error(frame);
        break;

    case CAN_ID_BOARD3_CMD:
        handle_gripper_command(frame);
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

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & 0x01U)
    {
        Macro_Clear_Bit(TIM3->SR, 0);
        g_ms_tick++;

        if ((g_ms_tick % GRIPPER_STATUS_PERIOD_MS) == 0U)
        {
            g_status_event = 1U;
        }

        /*
         * 0x203 status는 100ms 정각 tick, 0x303 position은 20ms 주기이되
         * 10ms offset으로 발생시켜 같은 loop에서 4개 frame이 몰리지 않게 한다.
         */
        if ((g_ms_tick % GRIPPER_POSITION_FEEDBACK_PERIOD_MS) ==
            (GRIPPER_POSITION_FEEDBACK_PERIOD_MS / 2U))
        {
            g_position_event = 1U;
        }
    }
}

void Main(void)
{
    int init_ret;

    led_direct_init();
    blink_direct(3, 800000U);

    Sys_Init(115200);

    printf("\nBoard3 Gripper CAN pre-hardware firmware - STM32F411RE + MCP2515\n");
    printf("CAN IDs: ESTOP=0x001 ENABLE=0x010 ARM_HOME_IGNORE=0x020 GRIPPER_HOME=0x023 CLEAR=0x030 CMD=0x103 STATUS=0x203 POSITION=0x303\n");
    printf("MCP2515 pins: PB13=SCK PB14=MISO PB15=MOSI PB12=CS PB4=INT\n");
    printf("Debug UART: USART2 PA2/PA3 115200, LED PA5 active-high\n");
    printf("Mode: NO REAL SERVO OUTPUT. Logic-only staging + virtual gripper execution.\n");

    shared_init();

    MCP2515_SPI_Init(64U);
    init_ret = MCP2515_Init(MCP2515_OSC_8MHZ);
    if (init_ret != 0)
    {
        g_state.state = STATE_ERROR;
        g_state.error_code = ERR_SERVO_COMM;
        printf("MCP2515 init failed: %d\n", init_ret);
        while (1)
        {
            led_direct_toggle();
            delay_loop(300000U);
        }
    }

    tim3_1ms_init();

    printf("MCP2515 init OK. Status 0x203 is sent every %u ms.\n", GRIPPER_STATUS_PERIOD_MS);
    printf("Position feedback 0x303 is sent every %u ms as 3 compressed frames.\n", GRIPPER_POSITION_FEEDBACK_PERIOD_MS);
    printf("Enable first: cansend can0 010#0100000000000000\n");
    printf("Gripper home: cansend can0 023#FF00000000000000  (home position = 0.00 deg)\n");
    printf("Then send 9 frames: 103#80... through 103#88... with same duration.\n");

    send_status();
    send_position_feedback_0x303();

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

            process_gripper_timeout();
            gripper_virtual_control_process();

            if (g_position_event)
            {
                __disable_irq();
                g_position_event = 0U;
                __enable_irq();
                send_position_feedback_0x303();
            }

            if (g_status_event)
            {
                __disable_irq();
                g_status_event = 0U;
                __enable_irq();
                send_status();
            }

            __WFI();
        }
    }
}
