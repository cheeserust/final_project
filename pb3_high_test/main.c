#include "stm32f4xx.h"

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    GPIOB->ODR |= (1UL << 3);

    GPIOB->MODER &= ~(3UL << (3U * 2U));
    GPIOB->MODER |=  (1UL << (3U * 2U));
    GPIOB->OTYPER &= ~(1UL << 3);
    GPIOB->OSPEEDR &= ~(3UL << (3U * 2U));
    GPIOB->OSPEEDR |=  (2UL << (3U * 2U));
    GPIOB->PUPDR &= ~(3UL << (3U * 2U));

    while (1) {
        GPIOB->BSRR = (1UL << 3);
    }
}
