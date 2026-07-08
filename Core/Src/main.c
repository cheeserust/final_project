#include "../Inc/board_can.h"
#include "../Inc/config.h"
#include "../Inc/gpio.h"
#include "../Inc/stm32f4xx_it.h"
#include "../Inc/mcp2515.h"
#include "../Inc/stepper.h"
#include "../Inc/tmc.h"
#include "../Inc/trajectory.h"

extern void uart_debug_init(uint8_t enabled);
extern void uart_debug_service(uint8_t enabled);
extern void uart_debug_print_ready(uint8_t enabled);
extern void uart_debug_print_loop_ready(uint8_t enabled);

static void clock_init_96mhz(void)
{
    RCC->CR |= (1 << 0);             // HSI ON
    while (!(RCC->CR & (1 << 1))) {}  // HSI ready

    RCC->APB1ENR |= (1 << 28);       // PWR clock enable
    (void)RCC->APB1ENR;
    PWR->CR |= (3 << 14);            // voltage scale 1

    FLASH->ACR = (1 << 10) |         // data cache
                 (1 << 9)  |         // instruction cache
                 (1 << 8)  |         // prefetch
                 (3 << 0);           // 3 wait states

    RCC->CR &= ~(1 << 24);           // PLL OFF
    while (RCC->CR & (1 << 25)) {}    // wait until PLL unlocked


    // p105:  RCC PLL configuration register
    // f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
    // f(PLL general clock output) = f(VCO clock) / PLLP
    // f(USB OTG FS, SDIO) = f(VCO clock) / PLLQ
    // usb otg 사용할떄 48mhz 맞춰야함
    RCC->PLLCFGR = (8 << 24) |       // PLLQ = 8, 384MHz / 8 = 48MHz
                   (0 << 22) |       // PLLSRC = 0 HSI 16MHz
                   (1 << 16) |       // PLLP = 4
                   (192 << 6) |      // PLLN = 192
                   (8 << 0);         // PLLM = 8, 16MHz / 8 * 192 / 4 = 96MHz

    RCC->CFGR = (0 << 13) |          // APB2 = /1
                (4 << 10) |          // APB1 = /2
                (0 << 4);            // AHB = /1

    RCC->CR |= (1 << 24);            // PLL ON
    while (!(RCC->CR & (1 << 25))) {} // PLL ready

    RCC->CFGR &= ~(0x3 << 0);
    RCC->CFGR |= (0x2 << 0);         // SYSCLK = PLL
    while (((RCC->CFGR >> 2) & 0x3) != 0x2) {}

    SystemCoreClock = SYSCLK_HZ;
}

int main(void)
{
    uint32_t last_status_ms = 0;  // 마지막 상태 CAN 송신 시간(ms)
    uint32_t last_feedback_ms = 0;  // 마지막 위치 feedback CAN 송신 시간(ms)
    uint32_t last_can_service_ms = 0;
    const uint8_t debug_uart = ENABLE_UART;
    uint8_t can_ready;

    __disable_irq();  // 초기화 중 인터럽트 진입 방지

    clock_init_96mhz();    // SYSCLK 96MHz 설정
    gpio_init();           // GPIO 초기화
    uart_debug_init(debug_uart);  // UART 디버그 콘솔
    uart_debug_print_ready(debug_uart);
    motor_disable();       // 초기 상태에서는 모터 출력 차단
    stepper_init();        // 스텝 모터 상태 변수 초기화
    trajectory_clear();    // 궤적 큐와 목표 위치 초기화
    interrupts_init();     // SysTick, TIM2, TIM3 인터럽트 시작
    spi2_init();           // MCP2515 통신용 SPI2 초기화
    tmc5160_init_all();    // 모든 TMC 드라이버 설정

    can_ready = mcp2515_init_500k();

    if (!can_ready) {
        g_error_code = ERR_DRIVER_FAULT;  // CAN 컨트롤러 초기화 실패
    }

    g_state = g_error_code == ERR_NONE ? STATE_DISABLED : STATE_ERROR;
    __enable_irq();                   // 인터럽트 허용
    last_status_ms = global_tick_ms;
    last_feedback_ms = global_tick_ms;
    last_can_service_ms = global_tick_ms;
    board_can_send_status();
    board_can_send_position_feedback_all();
    uart_debug_print_loop_ready(debug_uart);

    while (1) {
        CanFrame rx;    // [수신] CAN frame
        uint8_t rx_budget;

        uart_debug_service(debug_uart);

        // 1. MCP2515가 CAN 메시지를 받았는지 확인: EXTI쪽 flag or INT가 low인지 확인
        if (g_mcp2515_irq_pending || mcp2515_int_asserted()) {
            g_mcp2515_irq_pending = 0;

            // 2. RX backlog가 남지 않도록 한 loop에서 충분히 비우기
            rx_budget = 32;
            while (rx_budget > 0 && mcp2515_read_frame(&rx))// 읽을 프레임이 있으면 1,
            { 
                rx_budget--;
                board_can_handle_frame(&rx);
            }
        }

        // 4. 축 이동 명령이 중간에 끊겼는지 확인
        if (trajectory_handle_staging_timeout()) {
            board_can_request_status_event();  // 다축 명령 수신 타임아웃 발생 시 상태 송신
        }

        if ((global_tick_ms - last_can_service_ms) >= 1000) {
            last_can_service_ms = global_tick_ms;
            (void)mcp2515_service();
        }

        // 5. 100ms마다 현재 위치 feedback 송신
        if ((global_tick_ms - last_feedback_ms) >= 100) {
            last_feedback_ms = global_tick_ms;
            board_can_send_position_feedback_all();
        }

        // 6. 100ms마다 현재 상태를 CAN으로 송신
        if ((global_tick_ms - last_status_ms) >= 100) {
            last_status_ms = global_tick_ms;  // 상태 송신 기준 시간 갱신
            board_can_send_status();          // 100ms 주기 상태 프레임 송신
        }

        // 7. 주요 이벤트 상태 송신은 한 loop에서 모아 처리
        board_can_flush_status_event();
    }
}
