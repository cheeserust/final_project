#ifndef MCP2515_H
#define MCP2515_H

#include "config.h"

void spi2_init(void);
uint8_t mcp2515_init_500k(void);
uint8_t mcp2515_send_std(uint16_t sid, const uint8_t *data, uint8_t len);
uint8_t mcp2515_receive(uint16_t *sid, uint8_t *data, uint8_t *len);

#endif
