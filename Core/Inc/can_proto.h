#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include "config.h"

void can_send_status(void);
void can_process_frame(uint16_t id, const uint8_t *data, uint8_t len);

#endif
