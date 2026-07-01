#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx.h"
#include <stdint.h>

#define SYSCLK_HZ                (96 * 1000 * 1000) // 96 MHz

#define AXIS_COUNT               4
#define MICROSTEP                16
#define TRAJECTORY_QUEUE_SIZE    32
#define MULTI_AXIS_QUEUE_SIZE    (TRAJECTORY_QUEUE_SIZE / AXIS_COUNT)
#define PENDING_TIMEOUT_MS       20

#define CAN_ID_ESTOP             0x001
#define CAN_ID_ENABLE            0x010
#define CAN_ID_HOMING            0x020
#define CAN_ID_CLEAR_ERROR       0x030
#define CAN_ID_BOARD1_MOVE       0x101
#define CAN_ID_BOARD1_STAT       0x201
#define CAN_ID_BOARD1_POS        0x301

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
#define LIMIT_SWITCH_ACTIVE_LEVEL 1
#define LIMIT_SWITCH_DEBOUNCE_TICKS 500
#define HOMING_INTERVAL_TICKS    300

#define DIR_POSITIVE             1
#define DIR_NEGATIVE             0

typedef struct {
    volatile int32_t current_step;
    volatile int32_t target_step;

    int32_t move_start_step;
    int32_t move_end_step;
    int32_t move_step_offset;

    uint16_t move_total_time_ms;
    uint16_t move_elapsed_time_ms;

    uint8_t moving;
    uint8_t homing;
    uint8_t homing_done;
    uint8_t enabled;
} MotorState;

// CAN에서 받은 축 1개 명령
typedef struct {
    uint8_t motor_id;
    uint8_t flags;
    int32_t target_pos;
    uint16_t speed;
    uint8_t move_duration_5ms;
} CanTrajectoryCommand;

// 축 4개 이동 명령
typedef struct {
    int32_t target_pos[AXIS_COUNT];
    uint16_t speed[AXIS_COUNT];
    uint8_t flags[AXIS_COUNT];
    uint8_t move_duration_5ms;
} MultiAxisMoveCommand;

extern MotorState axis[AXIS_COUNT];
extern volatile uint8_t global_motor_enabled;
extern volatile uint8_t global_motor_state;
extern volatile uint8_t global_motor_error;
extern volatile uint8_t global_motor_estop;
extern volatile uint32_t global_tick_ms;

void system_update_state(void);
uint8_t system_homing_done_bits(void);
uint8_t system_enabled_status(void);
uint8_t system_first_moving_axis(void);

#endif
