#include "device_driver.h"

/*
 * STM32F411RE / Nucleo-F411RE LED setting
 * - User LED LD2: PA5
 * - Active-high: PA5=1 -> LED ON, PA5=0 -> LED OFF
 *
 * Board3 pre-hardware firmware uses this LED only for CAN RX/activity indication.
 * This F411RE test version does not use the PC13 active-low LED mapping.
 */

void LED_Init(void)
{
    /* GPIOA clock enable */
    Macro_Set_Bit(RCC->AHB1ENR, 0);
    (void)RCC->AHB1ENR;

    /* PA5 output, push-pull, low speed is enough for LED */
    Macro_Write_Block(GPIOA->MODER,   0x3, 0x1, 10);
    Macro_Clear_Bit(GPIOA->OTYPER, 5);
    Macro_Write_Block(GPIOA->OSPEEDR, 0x3, 0x1, 10);

    LED_Off();
}

void LED_On(void)
{
    /* active-high */
    Macro_Set_Bit(GPIOA->ODR, 5);
}

void LED_Off(void)
{
    /* active-high */
    Macro_Clear_Bit(GPIOA->ODR, 5);
}

void LED_Toggle(void)
{
    Macro_Invert_Bit(GPIOA->ODR, 5);
}
