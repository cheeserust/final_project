#include "../Inc/can_proto.h"
#include "../Inc/config.h"
#include "../Inc/gpio.h"
#include "../Inc/stm32f4xx_it.h"
#include "../Inc/mcp2515.h"
#include "../Inc/stepper.h"
#include "../Inc/tmc5160.h"
#include "../Inc/trajectory.h"

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

static void set_all_axis_enabled(uint8_t enabled)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis[i].enabled = enabled;  // 모든 축의 enable 상태를 동일하게 설정
    }
}

static uint8_t process_can_command(const CanCommand *cmd)
{
    // 1. 비상정지 명령
    if (cmd->type == CAN_CMD_ESTOP) {
        global_motor_estop = 1;          // 비상정지 상태 진입
        global_motor_enabled = 0;        // 모터 구동 명령 차단
        global_motor_error = ERR_NONE;   // 비상정지는 별도 상태로 보고 에러 코드는 초기화
        set_all_axis_enabled(0);         // 모든 축 비활성화
        trajectory_clear();              // 대기 중인 이동 명령 삭제
        stepper_stop_all();              // 스텝 펄스 출력 정지
        motor_disable();                 // 모터 드라이버 출력 비활성화
        return 1;                        // 변경된 상태 즉시 송신
    }

    // 2. 모터 enable/disable 명령
    if (cmd->type == CAN_CMD_ENABLE) {
        if (cmd->enable == 1) {
            global_motor_estop = 0;         // 비상정지 상태 해제
            global_motor_error = ERR_NONE;  // 기존 에러 코드 초기화
            global_motor_enabled = 1;       // 모터 구동 허용
            set_all_axis_enabled(1);        // 모든 축 활성화
            motor_enable();                 // 모터 드라이버 enable 핀 활성화
        } else {
            global_motor_enabled = 0;  // 모터 구동 금지
            set_all_axis_enabled(0);   // 모든 축 비활성화
            trajectory_clear();        // 남아있는 이동 명령 제거
            stepper_stop_all();        // 진행 중인 스텝 출력 정지
            motor_disable();           // 모터 드라이버 출력 차단
            global_motor_state = STATE_DISABLED;  // disable 상태로 보고
        }
        return 1;  // enable 처리 결과 송신
    }

    // 3. 원점복귀 명령
    if (cmd->type == CAN_CMD_HOMING) {
        if (!global_motor_enabled || global_motor_estop) {
            global_motor_error = ERR_INVALID_CMD;  // enable 전 또는 ESTOP 중 homing은 잘못된 명령
            return 1;
        }
        if (cmd->homing_mode != 0) {
            global_motor_error = ERR_INVALID_CMD;  // 지원하지 않는 homing 모드
            return 1;                              // 에러 상태 송신
        }

        global_motor_error = ERR_NONE;      // 새 homing 명령 전 에러 초기화
        trajectory_cancel_pending();        // 수신 중이던 다축 이동 명령 취소
        stepper_start_homing_all();         // 전체 축 원점복귀 시작
        return 1;  // homing 시작 또는 에러 상태 송신
    }

    // 4. 에러 해제 명령
    if (cmd->type == CAN_CMD_CLEAR_ERROR) {
        if (cmd->target_axis != HOMING_ALL_AXIS) {
            global_motor_error = ERR_INVALID_CMD;  // 최종 프로토콜은 전체 error clear만 허용
            return 1;                              // 에러 상태 송신
        }

        global_motor_error = ERR_NONE;       // 에러 코드 초기화
        trajectory_cancel_pending();         // 에러 중 들어오던 이동 명령 수신 취소
        if (!global_motor_estop) {
            global_motor_state = global_motor_enabled ? STATE_IDLE : STATE_DISABLED;  // enable 상태에 맞게 복귀
        }
        return 1;                            // 에러 해제 결과 송신
    }

    // 5. 이동 명령
    if (cmd->type == CAN_CMD_MOVE) {
        const CanTrajectoryCommand *command = &cmd->trajectory_command;
        uint8_t is_execute_requested = (command->flags & 0x08) ? 1 : 0;  // 실행 플래그
        uint8_t pending_result;

        if (!is_execute_requested) return 0;  // 실행 플래그가 없으면 무시
        if (!global_motor_enabled || global_motor_estop) return 0;  // 모터 비활성/비상정지 중이면 무시
        if (global_motor_error != ERR_NONE) return 0;  // 에러 상태에서는 새 이동 명령 차단
        if (command->motor_id >= AXIS_COUNT) {
            trajectory_cancel_pending();          // 잘못된 프레임으로 수신 중인 명령 취소
            global_motor_error = ERR_INVALID_CMD; // 존재하지 않는 축 번호
            return 1;                             // 에러 상태 송신
        }
        if (!axis[command->motor_id].homing_done) {
            trajectory_cancel_pending();          // homing 전 이동 명령은 폐기
            global_motor_error = ERR_INVALID_CMD; // 원점복귀 전 이동 금지
            return 1;                             // 에러 상태 송신
        }
        pending_result = trajectory_add_pending_command(command);  // 축별 프레임을 다축 이동 명령에 저장

        if (pending_result == TRAJECTORY_PENDING_INVALID) {
            global_motor_error = ERR_INVALID_CMD;  // 프레임 순서/플래그/축 번호 오류
            return 1;                              // 에러 상태 송신
        }
        if (pending_result == TRAJECTORY_PENDING_QUEUE_FULL) {
            global_motor_error = ERR_QUEUE_FULL;  // 궤적 큐가 가득 참
            return 1;                             // 에러 상태 송신
        }
        return 0;
    }

    return 0;
}

int main(void)
{
    uint32_t last_status_ms = 0;  // 마지막 상태 CAN 송신 시간(ms)

    __disable_irq();  // 초기화 중 인터럽트 진입 방지

    clock_init_96mhz();    // SYSCLK 96MHz 설정
    gpio_init();           // GPIO 초기화
    motor_disable();       // 초기 상태에서는 모터 출력 차단
    stepper_init();        // 스텝 모터 상태 변수 초기화
    trajectory_clear();    // 궤적 큐와 목표 위치 초기화
    interrupts_init();     // SysTick, TIM2, TIM3 인터럽트 시작
    spi2_init();           // MCP2515 통신용 SPI2 초기화
    tmc5160_init_all();    // 모든 TMC5160 드라이버 설정

    if (!mcp2515_init_500k()) {
        global_motor_error = ERR_DRIVER_FAULT;  // CAN 컨트롤러 초기화 실패
    }

    global_motor_state = STATE_IDLE;  // 초기화 완료 후 대기 상태로 전환
    __enable_irq();                   // 인터럽트 허용

    while (1) {
        uint16_t id;   // [수신] CAN  ID
        uint8_t data[8];  // [수신] CAN 데이터
        uint8_t len;   // [수신] CAN 데이터 길이
        CanCommand cmd;  // CAN 데이터를 해석해서 만든 내부 명령

        // 1. MCP2515가 CAN 메시지를 받았는지 확인
        if ((MCP_INT_PORT->IDR & (1 << MCP_INT_PIN)) == 0) {

            // 2. MCP2515 수신 버퍼에 남아있는 메시지를 모두 읽음
            while (mcp2515_receive(&id, data, &len)) {

                // 3. CAN ID/data -> command 바꾸고 실행
                if (can_decode_frame(id, data, len, &cmd)) {
                    if (process_can_command(&cmd)) {
                        can_send_status();  // 상태 변화/에러는 즉시 송신
                    }
                }
            }
        }

        // 4. 다축 이동 명령이 중간에 끊겼는지 확인
        if (trajectory_handle_pending_timeout()) {
            can_send_status();  // 다축 명령 수신 타임아웃 발생 시 상태 송신
        }

        // 5. 100ms마다 현재 상태를 CAN으로 송신
        if ((global_tick_ms - last_status_ms) >= 100) {
            last_status_ms = global_tick_ms;  // 상태 송신 기준 시간 갱신
            can_send_status();                // 100ms 주기 상태 프레임 송신
            can_send_position_feedback_all(); // 100ms 주기 현재 위치 피드백 송신
        }
    }
}
