#include "device_driver.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

void Uart2_Init(int baud)
{
  double div;
  unsigned int mant;
  unsigned int frac;

  Macro_Set_Bit(RCC->AHB1ENR, 0);                   // PA2,3
  Macro_Set_Bit(RCC->APB1ENR, 17);                   // USART2 ON
  Macro_Write_Block(GPIOA->MODER, 0xf, 0xa, 4);     // PA2,3 => ALT
  Macro_Write_Block(GPIOA->AFR[0], 0xff, 0x77, 8);  // PA2,3 => AF07
  Macro_Write_Block(GPIOA->PUPDR, 0xf, 0x5, 4);     // PA2,3 => Pull-Up  

  volatile unsigned int t = GPIOA->LCKR & 0x7FFF;
  GPIOA->LCKR = (0x1<<16)|t|(0x3<<2);                // Lock PA2, 3 Configuration
  GPIOA->LCKR = (0x0<<16)|t|(0x3<<2);
  GPIOA->LCKR = (0x1<<16)|t|(0x3<<2);
  t = GPIOA->LCKR;

  div = PCLK1/(16. * baud);
  mant = (int)div;
  frac = (int)((div - mant) * 16. + 0.5);
  mant += frac >> 4;
  frac &= 0xf;

  USART2->BRR = (mant<<4)|(frac<<0);
  USART2->CR1 = (1<<13)|(0<<12)|(0<<10)|(1<<3)|(1<<2);
  USART2->CR2 = 0<<12;
  USART2->CR3 = 0;
}

void Uart2_Send_Byte(char data)
{
  if(data == '\n')
  {
    while(!Macro_Check_Bit_Set(USART2->SR, 7));
    USART2->DR = 0x0d;
  }

  while(!Macro_Check_Bit_Set(USART2->SR, 7));
  USART2->DR = data;
}

void Uart1_Init(int baud)
{
  double div;
  unsigned int mant;
  unsigned int frac;

  Macro_Set_Bit(RCC->AHB1ENR, 0);                   // PA9,10
  Macro_Set_Bit(RCC->APB2ENR, 4);                   // USART1 ON

  // PA9만 ALT(교대기능) 모드로 설정 (PA10은 더이상 안 쓰므로 설정 제외하거나 변환)
  Macro_Write_Block(GPIOA->MODER, 0x3, 0x2, 18);    // PA9 => ALT Mode (10진수 2)
  Macro_Write_Block(GPIOA->AFR[1], 0xf, 0x7, 4);    // PA9 => AF07 (USART1_TX)
  // 중요: 내부 풀업은 끄고 Floating 상태로 설정 (물리 외장 풀업 저항 4.7k~10kΩ 사용)
  Macro_Write_Block(GPIOA->PUPDR, 0x3, 0x0, 18);    // PA9 => No Pull-up, No Pull-down

  // 밑은 PA 10도 함께 설정하므로 제외
  //Macro_Write_Block(GPIOA->MODER, 0xf, 0xa, 18);    // PA9,10 => ALT
  //Macro_Write_Block(GPIOA->AFR[1], 0xff, 0x77, 4);  // PA9,10 => AF07
  //Macro_Write_Block(GPIOA->PUPDR, 0xf, 0x5, 18);    // PA9,10 => Pull-Up
  
  volatile unsigned int t = GPIOA->LCKR & 0x7FFF;
  GPIOA->LCKR = (0x1<<16)|t|(0x1<<9);               // Lock PA9, 10 Configuration 원래는(0x3<<9) 
  GPIOA->LCKR = (0x0<<16)|t|(0x1<<9);
  GPIOA->LCKR = (0x1<<16)|t|(0x1<<9);
  t = GPIOA->LCKR;

  div = PCLK2 / (16. * baud);
  mant = (int)div;
  frac = (int)((div - mant) * 16 + 0.5);
  mant += frac >> 4;
  frac &= 0xf;
  USART1->BRR = (mant<<4)|(frac<<0);


  // 핵심 변경 Half-Duplex 레지스터 활성화 및 초기 모드 설정
  USART1->CR3 |= (1 << 3);  // HDSEL (Half-Duplex Selection) 비트 켜기 !!
  // 초기 상태는 안전하게 송신은 끄고, 수신(RE)만 켜서 대기 상태로 만듭니다.
  USART1->CR1 = (1 << 13) | (1 << 2); // UE(USART ON) | RE(Receive ON)
  USART1->CR2 = 0;

  // 기존 PA9,10
  //USART1->CR1 = (1<<13)|(0<<12)|(0<<10)|(1<<3)|(1<<2);
  //USART1->CR2 = 0 << 12;
  //USART1->CR3 = 0;
}

void Uart1_Send_Byte(char data)
{
  if(data == '\n')
  {
    while(!Macro_Check_Bit_Set(USART1->SR, 7));
    USART1->DR = 0x0d;
  }

  while(!Macro_Check_Bit_Set(USART1->SR, 7));
  USART1->DR = data;
}

void Uart1_Send_String(char *pt)
{
  while(*pt != 0)
  {
    Uart1_Send_Byte(*pt++);
  }
}

void Uart1_Printf(char *fmt,...)
{
	va_list ap;
	char string[256];

	va_start(ap,fmt);
	vsprintf(string,fmt,ap);
	Uart1_Send_String(string);
	va_end(ap);
}

char Uart1_Get_Pressed(void)
{
	if(Macro_Check_Bit_Set(USART1->SR, 5))
	{
		return (char)USART1->DR;
	}

	else
	{
		return (char)0;
	}
}

char Uart1_Get_Char(void)
{
	while(!Macro_Check_Bit_Set(USART1->SR, 5));
	return (char)USART1->DR;
}

// URT-2 로 신호 보내는 함수(오류없이)
void Uart1_Send_Binary(unsigned char data)
{
    while(!Macro_Check_Bit_Set(USART1->SR, 7));
    USART1->DR = data;
}

/**
 * @brief  USART1의 수신 버퍼에 남아있는 에코(Echo) 데이터를 비우는 함수
 * @note   URT-2 변환 보드 특성상 TX로 보낸 신호가 RX로 그대로 반사되어 들어오는 현상을 제거합니다.
 * @param  None
 * @retval None
 */

 /*
void Uart1_Flush_Rx(void)
{
    volatile char dummy;

    // 1. 송신이 완전히 완료될 때까지 대기 (SR 레지스터의 TC(Transmission Complete) 비트가 1이 될 때까지)
    while(!Macro_Check_Bit_Set(USART1->SR, 6));

    // 2. 수신 버퍼(DR)에 데이터가 차 있는 동안(RXNE == 1) 내부 데이터를 계속 읽어서 버림
    while(Macro_Check_Bit_Set(USART1->SR, 5))
    {
        dummy = (char)USART1->DR; // 읽는 순간 RXNE 플래그가 클리어됨
    }
}
*/
/**
 * @brief  무한 루프를 방지하기 위해 타임아웃 카운트가 적용된 1바이트 수신 함수
 * @param  timeout_loops: 데이터 대기를 위한 루프 카운트 수치 (1Mbps 통신 기준 30000~50000 권장)
 * @retval 수신된 데이터 (성공 시 0~255, 타임아웃 실패 시 -1)
 */
int Uart1_Get_Char_Timeout(int timeout_loops)
{
    volatile int timeout = timeout_loops;

    while(!Macro_Check_Bit_Set(USART1->SR, 5))
    {
        // [추가] ORE (Overrun Error) 플래그 비트 3 확인
        if (Macro_Check_Bit_Set(USART1->SR, 3))  
        {
            // 클리어 시퀀스: SR 레지스터를 읽고, 이어서 DR 레지스터를 읽음
            volatile uint32_t dummy = USART1->SR;
            dummy = USART1->DR;
            (void)dummy; // 컴파일러 경고 방지
            
            return -1; // 에러 발생 시 타임아웃과 동일하게 -1 반환하여 탈출
        }

        // 데이터가 안 오면 카운트를 차감하여 Blocking 방지
        if(--timeout <= 0)
        {
            return -1;
        }
    }
    
    // 수신된 데이터 반환 (0xFF 마스킹으로 깔끔한 1바이트 처리)
    return (int)(USART1->DR & 0xFF);
}

// 송신 모드로 전환 (수신 끄기 -> 에코 방지)
void Uart1_Set_Tx_Mode(void) {
    // USART1->CR1 &= ~USART_CR1_RE; // 수신 비활성화
    // USART1->CR1 |= USART_CR1_TE;  // 송신 활성화
    USART1->CR1 &= ~(1 << 2); // RE (Receive Enable) 끄기 -> 에코 차단
    USART1->CR1 |= (1 << 3);  // TE (Transmit Enable) 켜기
}

// 송신 완료 대기 후 수신 모드로 전환
void Uart1_Set_Rx_Mode(void) {
    // 마지막 데이터가 물리적으로 선을 타고 완전히 나갈 때까지 대기
    while(!Macro_Check_Bit_Set(USART1->SR, 6)); // Wait for TC (Transmission Complete)
    //USART1->CR1 &= ~USART_CR1_TE; // 송신 비활성화
    //USART1->CR1 |= USART_CR1_RE;  // 수신 활성화
    USART1->CR1 &= ~(1 << 3); // TE (Transmit Enable) 끄기
    USART1->CR1 |= (1 << 2);  // RE (Receive Enable) 켜기
}

