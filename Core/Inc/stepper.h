#ifndef STEPPER_H
#define STEPPER_H

#include "config.h"

void stepper_init(void);
void stepper_stop_axis(uint8_t id);
void stepper_stop_all(void);
void stepper_start_homing(uint8_t id);
void stepper_start_homing_all(void);
void stepper_update_10us(void);
uint8_t stepper_limit_status_bits(void);

#endif
