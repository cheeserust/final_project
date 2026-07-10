#ifndef MCP2515_H
#define MCP2515_H

#include "config.h"

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame;

typedef enum {
    MCP2515_SEND_BUSY = 0,
    MCP2515_SEND_OK = 1,
    MCP2515_SEND_FAULT = 2
} Mcp2515SendResult;

extern volatile uint8_t g_mcp2515_irq_pending;

void spi2_init(void);
uint8_t mcp2515_init_500k(void);
uint8_t mcp2515_read_frame(CanFrame *frame);
Mcp2515SendResult mcp2515_send_frame(const CanFrame *frame);
uint8_t mcp2515_int_asserted(void);
uint8_t mcp2515_service(void);

#endif
