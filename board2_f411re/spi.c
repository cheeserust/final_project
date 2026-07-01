#include "device_driver.h"
#include <stdint.h>

/*
 * Final-board compatible MCP2515 wiring for STM32F411RE test
 *
 * MCP2515 SCK / CLK  -> PB13 : SPI2_SCK  AF5
 * MCP2515 SO  / MISO -> PB14 : SPI2_MISO AF5
 * MCP2515 SI  / MOSI -> PB15 : SPI2_MOSI AF5
 * MCP2515 CS  / NSS  -> PB12 : GPIO output
 * MCP2515 INT        -> PB4  : EXTI4, active-low falling edge
 *
 * NOTE: Function names were changed to MCP2515_SPI_* to avoid confusion
 * with the previous PB3/PB4/PB5 SPI1 test wiring.
 */

void MCP2515_SPI_Init(unsigned int div)
{
    volatile int i;
    unsigned int br_bits = 0;
    unsigned int tmp = div;

    /* Convert divider to STM32 BR field approximately.
     * div=2 -> 0, div=4 -> 1, ... div=256 -> 7
     */
    while (tmp > 2U)
    {
        tmp >>= 1;
        br_bits++;
    }
    if (br_bits > 7U) br_bits = 7U;

    Macro_Set_Bit(RCC->AHB1ENR, 1);     /* GPIOB clock */
    Macro_Set_Bit(RCC->APB1ENR, 14);    /* SPI2 clock  */
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    /* SPI2 reset */
    Macro_Set_Bit(RCC->APB1RSTR, 14);
    for (i = 0; i < 1000; i++) { __NOP(); }
    Macro_Clear_Bit(RCC->APB1RSTR, 14);

    /* PB12 = CS output, default high */
    Macro_Write_Block(GPIOB->MODER, 0x3, 0x1, 24);
    Macro_Clear_Bit(GPIOB->OTYPER, 12);
    Macro_Write_Block(GPIOB->OSPEEDR, 0x3, 0x3, 24);
    Macro_Set_Bit(GPIOB->ODR, 12);

    /* PB13/PB14/PB15 = AF5 SPI2 */
    Macro_Write_Block(GPIOB->MODER,   0x3f,  0x2a,   26);
    Macro_Write_Block(GPIOB->AFR[1],  0xfff, 0x555,  20);
    Macro_Write_Block(GPIOB->OTYPER,  0x7,   0x0,    13);
    Macro_Write_Block(GPIOB->OSPEEDR, 0x3f,  0x3f,   26);
    Macro_Write_Block(GPIOB->PUPDR,   0x3f,  0x00,   26);

    /* SPI mode 0, 8-bit, master, software NSS.
     * F411RE current project clock: PCLK1 = 48 MHz.
     * div=64 -> SPI2 SCK around 750 kHz.
     * MCP2515 is stable at this speed during bring-up.
     */
    SPI2->CR1 =
        (0U << 11) |                 /* DFF = 8-bit */
        (0U << 10) |                 /* Full duplex */
        (1U << 9)  |                 /* SSM */
        (1U << 8)  |                 /* SSI */
        (0U << 7)  |                 /* MSB first */
        (br_bits << 3) |             /* BR */
        (1U << 2)  |                 /* Master */
        (0U << 1)  |                 /* CPOL = 0 */
        (0U << 0);                   /* CPHA = 0 */

    SPI2->CR2 = 0;
    Macro_Set_Bit(SPI2->CR1, 6);     /* SPE */
}

void MCP2515_CS_Low(void)
{
    Macro_Clear_Bit(GPIOB->ODR, 12);
}

void MCP2515_CS_High(void)
{
    while (Macro_Check_Bit_Set(SPI2->SR, 7));   /* BSY wait */
    Macro_Set_Bit(GPIOB->ODR, 12);
}

uint8_t MCP2515_SPI_TxRx_Byte(uint8_t data)
{
    while (Macro_Check_Bit_Clear(SPI2->SR, 1)); /* TXE */
    *(__IO uint8_t *)&SPI2->DR = data;

    while (Macro_Check_Bit_Clear(SPI2->SR, 0)); /* RXNE */
    return *(__IO uint8_t *)&SPI2->DR;
}
