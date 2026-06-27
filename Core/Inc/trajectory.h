#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "config.h"

void trajectory_clear(void);
void trajectory_cancel_staging(void);
uint8_t trajectory_stage_command(const TrajectoryPoint *point);
uint8_t trajectory_check_staging_timeout(void);
uint8_t trajectory_free_count(void);
void trajectory_update_1ms(void);
int32_t trajectory_angle_raw_to_step(uint8_t axis_id, int32_t angle_raw);

#define TRAJECTORY_STAGE_WAITING     0U
#define TRAJECTORY_STAGE_COMMITTED   1U
#define TRAJECTORY_STAGE_INVALID     2U
#define TRAJECTORY_STAGE_QUEUE_FULL  3U

#endif
