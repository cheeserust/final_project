#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "config.h"

void trajectory_clear(void);
void trajectory_cancel_pending(void);
uint8_t trajectory_add_pending_command(const CanTrajectoryCommand *command);
uint8_t trajectory_check_pending_timeout(void);
uint8_t get_available_axis_command_count(void);
void trajectory_1ms_interrupt(void);
int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw);
int32_t step_to_angle(uint8_t axis_id, int32_t step);
int32_t get_home_angle(uint8_t axis_id);

#define TRAJECTORY_PENDING_WAITING     0
#define TRAJECTORY_PENDING_COMMITTED   1
#define TRAJECTORY_PENDING_INVALID     2
#define TRAJECTORY_PENDING_QUEUE_FULL  3

#endif
