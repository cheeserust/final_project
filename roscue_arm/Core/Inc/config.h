#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx.h"
#include <stdint.h>

#define SYSCLK_HZ                (96 * 1000 * 1000) // 96 MHz

#define BOARD_ID_BOARD1          1
#define BOARD_ID_BOARD2          2

#ifndef BOARD_ID
#define BOARD_ID                 BOARD_ID_BOARD1
#endif
#define ENABLE_UART 0
#ifndef ENABLE_ESTOP_LOGIC
#define ENABLE_ESTOP_LOGIC       0
#endif

#if BOARD_ID == BOARD_ID_BOARD1
#define AXIS_COUNT               4
#define BOARD_STAGING_FRAME_COUNT 4
#define BOARD_MOVE_CAN_ID        0x101
#define BOARD_STATUS_CAN_ID      0x201
#define BOARD_POSITION_CAN_ID    0x301
#elif BOARD_ID == BOARD_ID_BOARD2
#define AXIS_COUNT               1
#define BOARD_STAGING_FRAME_COUNT 1
#define BOARD_MOVE_CAN_ID        0x102
#define BOARD_STATUS_CAN_ID      0x202
#define BOARD_POSITION_CAN_ID    0x302
#else
#error "Unsupported BOARD_ID"
#endif

#define MICROSTEP                16
#define TRAJECTORY_QUEUE_SIZE    128
#define TRAJECTORY_POINT_QUEUE_SIZE (TRAJECTORY_QUEUE_SIZE / BOARD_STAGING_FRAME_COUNT)
#define MULTI_AXIS_QUEUE_SIZE    TRAJECTORY_POINT_QUEUE_SIZE
#define STAGING_TIMEOUT_MS       100
#define MIN_STEP_INTERVAL_TICKS (9 - 1) //(100rpm)
#define MOTION_MAX_STEP_RATE_SPS 1000   // duration 너무 짧으면  stepper_prepare_motion()에서 이 step/s를 넘지 않도록 duration을 늘림


#define CAN_ID_ESTOP             0x001
#define CAN_ID_ENABLE            0x010
#define CAN_ID_HOMING            0x020
#define CAN_ID_CLEAR_ERROR       0x030

#define CAN_CTRL_EXECUTE         0x80
#define CAN_CTRL_RELATIVE        0x40
#define CAN_CTRL_STEP_MODE       0x20
#define CAN_CTRL_RESERVED        0x10
#define CAN_CTRL_FLAG_MASK       0xF0
#define CAN_CTRL_MOTOR_MASK      0x0F

#define STATE_INIT               0
#define STATE_IDLE               1
#define STATE_HOMING             2
#define STATE_MOVING             3
#define STATE_ERROR              4
#define STATE_ESTOP              5
#define STATE_DISABLED           6

#define ERR_NONE                 0
#define ERR_INVALID_CMD          1
#define ERR_LIMIT_SWITCH_DETECTED 2
#define ERR_DRIVER_FAULT         3
#define ERR_HOMING_FAIL          4
#define ERR_QUEUE_FULL           5
#define ERR_RESERVED             6

#define HOMING_ALL_AXIS          255
#define LIMIT_SWITCH_ACTIVE_HIGH  1
#define LIMIT_SWITCH_DEBOUNCE_TICKS 20
#define HOMING_INTERVAL_TICKS    120

#define DIR_POSITIVE             1
#define DIR_NEGATIVE             (-1)

typedef struct {
    int32_t target_step[AXIS_COUNT];
    uint16_t speed[AXIS_COUNT];
    uint16_t duration_ms;
} TrajectoryPoint;

extern volatile uint8_t g_enabled;
extern volatile uint8_t g_estop;
extern volatile uint8_t g_state;
extern volatile uint8_t g_error_code;
extern volatile uint8_t g_queue_overflow;
extern volatile uint8_t g_motion_active;
extern volatile uint8_t g_homing_active;
extern volatile uint8_t g_homing_done_bits;
extern volatile int32_t g_current_step[AXIS_COUNT];
extern volatile int32_t g_target_step[AXIS_COUNT];
extern volatile int32_t g_motion_start_step[AXIS_COUNT];
extern volatile uint32_t global_tick_ms;

#define ESTOP_ACTIVE()           (ENABLE_ESTOP_LOGIC && g_estop)

uint8_t system_homing_done_bits(void);
uint8_t system_enabled_status(void);
uint8_t system_all_homed(void);
uint8_t system_reported_error_code(void);

#endif
