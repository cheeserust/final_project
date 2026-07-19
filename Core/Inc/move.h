#ifndef MOVE_H
#define MOVE_H

#include "config.h"

void move_clear(void);
uint8_t move_check_staging_timeout(uint8_t *goal_id, uint8_t *mask,
                                   uint16_t *duration_ms);
uint8_t move_stage_goal_axis(uint8_t motor_id, int32_t target_step,
                             uint8_t goal_id, uint16_t duration_ms,
                             uint8_t *received_axis_mask);
uint8_t move_start_goal(uint8_t goal_id);
uint8_t move_cancel_goal(uint8_t goal_id);
uint8_t move_goal_slot_free(void);
uint16_t move_goal_duration_ms(void);
uint8_t move_goal_mask(void);
void move_check_completion_1ms(void);
int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw);
int32_t step_to_angle(uint8_t axis_id, int32_t step);
int32_t get_home_angle(uint8_t axis_id);
uint8_t move_resolve_target_step(uint8_t axis_id, int32_t target_raw,
                                 uint8_t relative, uint8_t step_mode,
                                 int32_t *target_step);

#define GOAL_STAGE_WAITING   0
#define GOAL_STAGE_READY     1
#define GOAL_STAGE_INVALID   2
#define GOAL_STAGE_DUPLICATE 3
#define GOAL_STAGE_BUSY      4

#endif
