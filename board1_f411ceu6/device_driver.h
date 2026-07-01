#include "stm32f4xx.h"
#include "option.h"
#include "macro.h"
#include "malloc.h"
#include <stdint.h>


// Uart.c

extern void Uart2_Init(int baud);
extern void Uart2_Send_Byte(char data);
extern void Uart2_RX_Interrupt_Enable(int en);

extern void Uart1_Init(int baud);
extern void Uart1_Send_Byte(char data);
extern void Uart1_Send_String(char *pt);
extern void Uart1_Printf(char *fmt,...);
extern char Uart1_Get_Char(void);
extern char Uart1_Get_Pressed(void);

// SysTick.c

extern void SysTick_Run(unsigned int msec);
extern int SysTick_Check_Timeout(void);
extern unsigned int SysTick_Get_Time(void);
extern unsigned int SysTick_Get_Load_Time(void);
extern void SysTick_Stop(void);

// Led.c

extern void LED_Init(void);
extern void LED_On(void);
extern void LED_Off(void);

// Clock.c

extern void Clock_Init(void);

// Key.c

extern void Key_Poll_Init(void);
extern int Key_Get_Pressed(void);
extern void Key_Wait_Key_Released(void);
extern void Key_Wait_Key_Pressed(void);
extern void Key_ISR_Enable(int en);

// Timer.c

extern void TIM2_Delay(int time);
extern void TIM2_Stopwatch_Start(void);
extern unsigned int TIM2_Stopwatch_Stop(void);
extern void TIM4_Repeat(int time);
extern int TIM4_Check_Timeout(void);
extern void TIM4_Stop(void);
extern void TIM4_Change_Value(int time);
extern void TIM4_Repeat_Interrupt_Enable(int en, int time);
extern void TIM3_Out_Init(void);
extern void TIM3_Out_Freq_Generation(unsigned short freq);
extern void TIM3_Out_Stop(void);

// i2c.c

#define SC16IS752_IODIR				0x0A
#define SC16IS752_IOSTATE			0x0B

extern void I2C1_SC16IS752_Init(unsigned int freq);
extern void I2C1_SC16IS752_Write_Reg(unsigned int addr, unsigned int data);
extern void I2C1_SC16IS752_Config_GPIO(unsigned int config);
extern void I2C1_SC16IS752_Write_GPIO(unsigned int data);


// Adc.c

extern void ADC1_IN6_Init(void);
extern void ADC1_Start(void);
extern void ADC1_Stop(void);
extern int ADC1_Get_Status(void);
extern int ADC1_Get_Data(void);


//CAN

typedef struct
{
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Frame_t;

#define MCP2515_OSC_8MHZ   8
#define MCP2515_OSC_16MHZ  16

extern void LED_Toggle(void);

extern void MCP2515_SPI_Init(unsigned int div);
extern uint8_t MCP2515_SPI_TxRx_Byte(uint8_t data);
extern void MCP2515_CS_Low(void);
extern void MCP2515_CS_High(void);

extern volatile unsigned int g_mcp2515_irq;

extern int MCP2515_Init(int osc_mhz);
extern int MCP2515_Read_Frame(CAN_Frame_t *frame);
extern int MCP2515_Send_Frame(const CAN_Frame_t *frame);
extern int MCP2515_Int_Asserted(void);
extern void MCP2515_Abort_All_Tx(void);