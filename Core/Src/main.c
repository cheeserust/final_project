#include "../Inc/can_proto.h"
#include "../Inc/gpio.h"
#include "../Inc/mcp2515.h"
#include "../Inc/stepper.h"
#include "../Inc/tmc5160.h"
#include "../Inc/trajectory.h"

static void systick_init(void)
{
    SysTick_Config(SYSCLK_HZ / 1000UL);  // SysTick을 1ms 주기로 설정
}

void SysTick_Handler(void)
{
    global_tick_ms++;  // 시스템 기준 시간(ms) 증가
}

static void tim2_init_10us(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // TIM2 클럭 활성화
    (void)RCC->APB1ENR;                  // 클럭 활성화 후 레지스터 반영 대기용 dummy read

    TIM2->PSC = 16U - 1U;             // 16MHz 기준 1us 타이머 tick 생성
    TIM2->ARR = TIMER_TICK_US - 1U;   // 10us마다 update interrupt 발생
    TIM2->DIER |= TIM_DIER_UIE;       // update interrupt enable
    TIM2->CR1 |= TIM_CR1_CEN;         // TIM2 카운터 시작
    NVIC_EnableIRQ(TIM2_IRQn);        // TIM2 인터럽트 NVIC 활성화
}

static void tim3_init_1ms(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // TIM3 클럭 활성화
    (void)RCC->APB1ENR;                  // 클럭 활성화 후 레지스터 반영 대기용 dummy read

    TIM3->PSC = 16U - 1U;       // 16MHz 기준 1us 타이머 tick 생성
    TIM3->ARR = 1000U - 1U;     // 1ms마다 update interrupt 발생
    TIM3->DIER |= TIM_DIER_UIE; // update interrupt enable
    TIM3->CR1 |= TIM_CR1_CEN;   // TIM3 카운터 시작
    NVIC_EnableIRQ(TIM3_IRQn);  // TIM3 인터럽트 NVIC 활성화
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;  // update interrupt flag clear
        if (!global_motor_estop && global_motor_enabled) stepper_update_10us();  // 10us 주기로 스텝 펄스 갱신
        else stepper_stop_all();  // 비상정지/비활성 상태에서는 모든 축 정지
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;  // update interrupt flag clear
        if (!global_motor_estop && global_motor_enabled && global_motor_error == ERR_NONE) {
            trajectory_update_1ms();  // 1ms 주기로 궤적 보간 목표 위치 갱신
        }
    }
}

int main(void)
{
    uint32_t last_status_ms = 0;  // 마지막 상태 CAN 송신 시간(ms)

    __disable_irq();  // 초기화 중 인터럽트 진입 방지

    gpio_init();           // GPIO 입출력 및 대체 기능 설정
    motor_disable();       // 초기 상태에서는 모터 출력 차단
    stepper_init();        // 스텝 모터 상태 변수 초기화
    trajectory_clear();    // 궤적 큐와 목표 위치 초기화
    systick_init();        // 1ms 시스템 tick 시작
    tim2_init_10us();      // 스텝 펄스용 10us 타이머 시작
    tim3_init_1ms();       // 궤적 갱신용 1ms 타이머 시작
    spi2_init();           // MCP2515 통신용 SPI2 초기화
    tmc5160_init_all();    // 모든 TMC5160 드라이버 설정

    if (!mcp2515_init_500k()) {
        global_motor_error = ERR_DRIVER_FAULT;  // CAN 컨트롤러 초기화 실패
    }

    global_motor_state = STATE_IDLE;  // 초기화 완료 후 대기 상태로 전환
    __enable_irq();                   // 인터럽트 허용

    while (1) {
        uint16_t id;   // 수신 CAN 표준 ID
        uint8_t data[8];  // 수신 CAN 데이터
        uint8_t len;   // 수신 CAN 데이터 길이

        if (GPIO_READ(MCP_INT_PORT, MCP_INT_PIN) == 0U) {
            while (mcp2515_receive(&id, data, &len)) {
                can_process_frame(id, data, len);  // 수신한 CAN 명령 처리
            }
        }

        if (trajectory_check_staging_timeout()) {
            can_send_status();  // 다축 명령 조립 타임아웃 발생 시 상태 송신
        }

        if ((global_tick_ms - last_status_ms) >= 100UL) {
            last_status_ms = global_tick_ms;  // 상태 송신 기준 시간 갱신
            can_send_status();                // 100ms 주기 상태 프레임 송신
        }
    }
}
