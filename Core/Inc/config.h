#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx.h"
#include <stdint.h>

#define SYSCLK_HZ                16000000UL
#define TIMER_TICK_US            10UL

#define AXIS_COUNT               4U
#define MOTOR_STEPS_PER_REV      200
#define MICROSTEP                16
#define TRAJECTORY_QUEUE_SIZE    32U
#define MULTI_AXIS_QUEUE_SIZE    (TRAJECTORY_QUEUE_SIZE / AXIS_COUNT)
#define STAGING_TIMEOUT_MS       20U

#define CAN_ID_ESTOP             0x001U
#define CAN_ID_ENABLE            0x010U
#define CAN_ID_HOMING            0x020U
#define CAN_ID_CLEAR_ERROR       0x030U
#define CAN_ID_BOARD1_MOVE       0x101U
#define CAN_ID_BOARD1_STAT       0x201U

#define STATE_INIT               0U
#define STATE_IDLE               1U
#define STATE_HOMING             2U
#define STATE_MOVING             3U
#define STATE_ERROR              4U
#define STATE_ESTOP              5U

#define ERR_NONE                 0U
#define ERR_INVALID_CMD          1U
#define ERR_LIMIT_DETECTED       2U
#define ERR_DRIVER_FAULT         3U
#define ERR_HOMING_FAIL          4U
#define ERR_QUEUE_FULL           5U
#define ERR_RESERVED             6U

#define HOMING_ALL_AXIS          255U
#define LIMIT_ACTIVE_LEVEL       0U
#define LIMIT_DEBOUNCE_TICKS     500U
#define HOMING_INTERVAL_TICKS    300U

#define DIR_POSITIVE             1U
#define DIR_NEGATIVE             0U

typedef struct {
    volatile int32_t current_step;
    volatile int32_t target_step;

    int32_t seg_start_step;
    int32_t seg_end_step;
    int32_t seg_delta_step;

    uint16_t seg_total_ms;
    uint16_t seg_elapsed_ms;

    uint8_t moving;
    uint8_t homing;
    uint8_t homing_done;
    uint8_t enabled;
} MotorState;

typedef struct {
    uint8_t motor_id;
    uint8_t flags;
    int32_t target_pos;
    uint16_t speed;
    uint8_t duration_5ms;
} TrajectoryPoint;

typedef struct {
    int32_t target_pos[AXIS_COUNT];
    uint16_t speed[AXIS_COUNT];
    uint8_t duration_5ms;
} MultiAxisTrajectoryPoint;

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
