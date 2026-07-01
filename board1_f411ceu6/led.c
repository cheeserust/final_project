#include "device_driver.h"

/* STM32F411CEU6 BlackPill-style LED: PC13, active-low */

void LED_Init(void)
{
    Macro_Set_Bit(RCC->AHB1ENR, 2);     /* GPIOC clock */
    (void)RCC->AHB1ENR;

    Macro_Write_Block(GPIOC->MODER, 0x3, 0x1, 26);  /* PC13 output */
    Macro_Clear_Bit(GPIOC->OTYPER, 13);
    Macro_Set_Bit(GPIOC->ODR, 13);                  /* OFF, active-low */
}

void LED_On(void)
{
    Macro_Clear_Bit(GPIOC->ODR, 13);                /* ON, active-low */
}

void LED_Off(void)
{
    Macro_Set_Bit(GPIOC->ODR, 13);                  /* OFF, active-low */
}

void LED_Toggle(void)
{
    Macro_Invert_Bit(GPIOC->ODR, 13);
}
