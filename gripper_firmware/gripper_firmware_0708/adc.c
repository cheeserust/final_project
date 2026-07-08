#include "device_driver.h"

void ADC1_IN6_Init(void)
{
	Macro_Set_Bit(RCC->AHB1ENR, 0); 				// PA POWER ON
	Macro_Write_Block(GPIOA->MODER, 0x3, 0x3, 12);	// PA6(ADC-IN6) = Analog Mode

	Macro_Set_Bit(RCC->APB2ENR, 8); 				// ADC1 POWER ON
	Macro_Write_Block(ADC1->SMPR2, 0x7, 0x7, 18); 	// Clock Configuration of CH6 = 480 Cycles
	Macro_Write_Block(ADC1->SQR1, 0xF, 0x0, 20); 	// Conversion Sequence No = 1
	Macro_Write_Block(ADC1->SQR3, 0x1F, 6, 0); 		// Sequence Channel of No 1 = CH6

	Macro_Write_Block(ADC->CCR, 0x3, 0x2, 16); 		// ADC CLOCK = 16MHz(PCLK2/6)
	Macro_Set_Bit(ADC1->CR2, 0); 					// ADC ON
}

void ADC1_Start(void)
{
	Macro_Set_Bit(ADC1->CR2, 30); 					// ADC SW Start
}

void ADC1_Stop(void)
{
	Macro_Clear_Bit(ADC1->CR2, 30); 				// ADC Stop
	Macro_Clear_Bit(ADC1->CR2, 0); 					// ADC OFF
}

int ADC1_Get_Status(void)
{
	int r = Macro_Check_Bit_Set(ADC1->SR, 1);

	if(r)
	{
		Macro_Clear_Bit(ADC1->SR, 1);
		Macro_Clear_Bit(ADC1->SR, 4);
	}

	return r;
}

int ADC1_Get_Data(void)
{
	return Macro_Extract_Area(ADC1->DR, 0xFFF, 0);
}
