#include "../Inc/stm32f4xx_it.h"
#include "../Inc/config.h"
#include "../Inc/stepper.h"
#include "../Inc/trajectory.h"

static void systick_init(void)
{
    SysTick_Config(SYSCLK_HZ / 1000);  // CPU 클럭을 1000으로 나눠 1ms 주기 설정
}

static void tim2_init_10us(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // TIM2 클럭 활성화
    (void)RCC->APB1ENR;                  // 클럭 활성화 후 레지스터 반영 대기용 dummy read

    TIM2->PSC = 96 - 1;   // APB1 timer 96MHz/96 = 1MHz = 1us
    TIM2->ARR = 10 - 1;   // 10us마다 interrupt 발생
    TIM2->DIER |= TIM_DIER_UIE;       // interrupt enable
    TIM2->CR1 |= TIM_CR1_CEN;         // TIM2 카운터 시작
    /* STEP pulse timing must preempt the slower 1ms trajectory/homing work. */
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);        // TIM2 인터럽트 NVIC 활성화
}

static void tim3_init_1ms(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // TIM3 클럭 활성화
    (void)RCC->APB1ENR;                  // 클럭 활성화 후 레지스터 반영 대기용 dummy read

    TIM3->PSC = 96 - 1;       // APB1 timer 96MHz/96 = 1MHz = 1us
    TIM3->ARR = 1000 - 1;     // 1ms마다 interrupt 발생
    TIM3->DIER |= TIM_DIER_UIE; // interrupt enable
    TIM3->CR1 |= TIM_CR1_CEN;   // TIM3 카운터 시작
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_EnableIRQ(TIM3_IRQn);  // TIM3 인터럽트 NVIC 활성화
}

void interrupts_init(void)
{
    systick_init(); // 시스템 시간 1ms
    tim2_init_10us(); // 스텝 펄스용 타이머 10us
    tim3_init_1ms(); // 궤적 보간용 타이머 1ms
}

//============설정 끝=========

volatile uint32_t debug_invalid_irq = 0;
volatile uint32_t debug_invalid_icsr = 0;

void _Invalid_ISR(void)
{
    debug_invalid_icsr = SCB->ICSR;
    debug_invalid_irq = SCB->ICSR & 0x1ff;
    for(;;);
}



void SysTick_Handler(void)
{
    global_tick_ms++;  // 시스템 기준 시간(ms) 증가, 메인에서 100ms마다 상태 송신할 사용
}

// 10us
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & (1 << 0)) {
        TIM2->SR &= ~(1 << 0);
        stepper_10us_interrupt();
    }
}

// 1ms
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & (1 << 0)) {
        TIM3->SR &= ~(1 << 0);  // UIF clear: update interrupt flag 초기화

        trajectory_1ms_interrupt();  // 1ms 주기로 궤적 보간 목표 위치 갱신
        stepper_1ms_interrupt(); // 1ms주기로 리미트 스위치 판단
        stepper_homing_1ms();
    }
}
