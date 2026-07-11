#ifndef GPIO_H
#define GPIO_H

#include "config.h"

#define STEP1_PORT GPIOA
#define STEP1_PIN  1
#define DIR1_PORT  GPIOA
#define DIR1_PIN   0
#define CS1_PORT   GPIOA
#define CS1_PIN    5

#define STEP2_PORT GPIOC
#define STEP2_PIN  15
#define DIR2_PORT  GPIOC
#define DIR2_PIN   14
#define CS2_PORT   GPIOA
#define CS2_PIN    4

#define STEP3_PORT GPIOB
#define STEP3_PIN  9
#define DIR3_PORT  GPIOB
#define DIR3_PIN   8
#define CS3_PORT   GPIOB
#define CS3_PIN    10

#define STEP4_PORT GPIOB
#define STEP4_PIN  7
#define DIR4_PORT  GPIOB
#define DIR4_PIN   6
#define CS4_PORT   GPIOB
#define CS4_PIN    2

#define TMC_MOSI_PORT GPIOB
#define TMC_MOSI_PIN  1
#define TMC_MISO_PORT GPIOB
#define TMC_MISO_PIN  0
#define TMC_CLK_PORT  GPIOA
#define TMC_CLK_PIN   6

#define MOTOR_EN_PORT GPIOB
#define MOTOR_EN_PIN  3

#define LIM1_PORT GPIOA
#define LIM1_PIN  7
#define LIM2_PORT GPIOA
#define LIM2_PIN  15
#define LIM3_PORT GPIOB
#define LIM3_PIN  4
#define LIM4_PORT GPIOB
#define LIM4_PIN  12

#define MCP_CS_PORT   GPIOA
#define MCP_CS_PIN    9
#define MCP_INT_PORT  GPIOA
#define MCP_INT_PIN   10
#define MCP_INT_EXTICR_PORT 0
#define MCP_INT_IRQn  EXTI15_10_IRQn
#define MCP_SCK_PORT  GPIOB
#define MCP_SCK_PIN   13
#define MCP_MISO_PORT GPIOB
#define MCP_MISO_PIN  14
#define MCP_MOSI_PORT GPIOB
#define MCP_MOSI_PIN  15

/* BSRR is a single atomic write, so an ISR changing another pin on the same
 * GPIO port cannot be overwritten by a read-modify-write of ODR. */
#define GPIO_SET_PIN(port, pin)      ((port)->BSRR = (1u << (pin)))
#define GPIO_CLEAR_PIN(port, pin)    ((port)->BSRR = (1u << ((pin) + 16u)))

void gpio_init(void);
void motor_enable(void);
void motor_disable(void);

#endif
