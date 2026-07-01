#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include "config.h"

typedef enum {
    CAN_CMD_ESTOP = 0,
    CAN_CMD_ENABLE,
    CAN_CMD_HOMING,
    CAN_CMD_CLEAR_ERROR,
    CAN_CMD_MOVE
} CanCommandType;

typedef struct {
    CanCommandType type;
    uint8_t enable;
    uint8_t target_axis;
    uint8_t homing_mode;
    CanTrajectoryCommand trajectory_command;
} CanCommand;

void can_send_status(void);
void can_send_position_feedback(uint8_t motor_id);
void can_send_position_feedback_all(void);
uint8_t can_decode_frame(uint16_t id, const uint8_t *data, uint8_t len, CanCommand *cmd);

#endif
