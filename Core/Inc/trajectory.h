#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "config.h"

void trajectory_clear(void);
void trajectory_cancel_staging(void);
void trajectory_stop_motion(void);
void trajectory_sync_planned_to_current(void);
int32_t trajectory_get_planned_step(uint8_t axis_id);
uint8_t trajectory_add_axis_command(uint8_t motor_id, int32_t target_step, uint16_t speed, uint8_t duration_5ms);
uint8_t trajectory_handle_staging_timeout(void);
void trajectory_request_queue_overflow_clear(void);
void trajectory_cancel_queue_overflow_clear(void);
uint8_t trajectory_take_queue_overflow_clear_ack(void);
uint8_t get_free_axis_command_count(void);
void trajectory_1ms_interrupt(void);
int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw);
int32_t step_to_angle(uint8_t axis_id, int32_t step);
int32_t get_home_angle(uint8_t axis_id);
uint8_t trajectory_resolve_target_step(uint8_t axis_id, int32_t target_raw, uint8_t relative, uint8_t step_mode, int32_t *target_step);

#define TRAJECTORY_STAGING_WAITING     0
#define TRAJECTORY_STAGING_COMMITTED   1
#define TRAJECTORY_STAGING_INVALID     2
#define TRAJECTORY_STAGING_QUEUE_FULL  3

#endif
