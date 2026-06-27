#ifndef GPIO_H
#define GPIO_H

#include "config.h"

#define STEP1_PORT GPIOA
#define STEP1_PIN  1U
#define DIR1_PORT  GPIOA
#define DIR1_PIN   0U
#define CS1_PORT   GPIOA
#define CS1_PIN    5U

#define STEP2_PORT GPIOC
#define STEP2_PIN  15U
#define DIR2_PORT  GPIOC
#define DIR2_PIN   14U
#define CS2_PORT   GPIOA
#define CS2_PIN    4U

#define STEP3_PORT GPIOB
#define STEP3_PIN  9U
#define DIR3_PORT  GPIOB
#define DIR3_PIN   8U
#define CS3_PORT   GPIOB
#define CS3_PIN    10U

#define STEP4_PORT GPIOB
#define STEP4_PIN  7U
#define DIR4_PORT  GPIOB
#define DIR4_PIN   6U
#define CS4_PORT   GPIOB
#define CS4_PIN    2U

#define TMC_MOSI_PORT GPIOB
#define TMC_MOSI_PIN  1U
#define TMC_MISO_PORT GPIOB
#define TMC_MISO_PIN  0U
#define TMC_CLK_PORT  GPIOA
#define TMC_CLK_PIN   6U

#define MOTOR_EN_PORT GPIOB
#define MOTOR_EN_PIN  3U

#define LIM1_PORT GPIOA
#define LIM1_PIN  2U
#define LIM2_PORT GPIOA
#define LIM2_PIN  3U
#define LIM3_PORT GPIOA
#define LIM3_PIN  9U
#define LIM4_PORT GPIOA
#define LIM4_PIN  10U

#define MCP_CS_PORT   GPIOB
#define MCP_CS_PIN    12U
#define MCP_INT_PORT  GPIOB
#define MCP_INT_PIN   4U
#define MCP_SCK_PORT  GPIOB
#define MCP_SCK_PIN   13U
#define MCP_MISO_PORT GPIOB
#define MCP_MISO_PIN  14U
#define MCP_MOSI_PORT GPIOB
#define MCP_MOSI_PIN  15U

#define GPIO_SET_ODR(port, pin)      ((port)->ODR |=  (1UL << (pin)))
#define GPIO_CLEAR_ODR(port, pin)    ((port)->ODR &= ~(1UL << (pin)))
#define GPIO_READ(port, pin)         (((port)->IDR >> (pin)) & 1UL)

void gpio_init(void);
void motor_enable(void);
void motor_disable(void);

#endif
