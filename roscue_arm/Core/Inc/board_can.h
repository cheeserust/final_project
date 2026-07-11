#include "mcp2515.h"

#define BOARD_CAN_H

void board_can_handle_frame(const CanFrame *frame);
void board_can_queue_status(void);
void board_can_queue_position_feedback_all(void);
void board_can_service_tx(void);
void board_can_request_status_event(void);
void board_can_flush_status_event(void);
