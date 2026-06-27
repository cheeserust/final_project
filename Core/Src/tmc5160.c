#include "../Inc/tmc5160.h"
#include "../Inc/gpio.h"

#define TMC_REG_GCONF       0x00U  // TMC5160 global configuration 레지스터
#define TMC_REG_IHOLD_IRUN  0x10U  // 모터 정지/구동 전류 설정 레지스터
#define TMC_REG_CHOPCONF    0x6CU  // chopper 및 microstep 설정 레지스터
#define TMC_REG_PWMCONF     0x70U  // stealthChop PWM 설정 레지스터

static void tmc_cs_low(uint8_t axis_id)
{
    if (axis_id == 0U) GPIO_CLEAR_ODR(CS1_PORT, CS1_PIN);       // 1번 축 TMC5160 선택
    else if (axis_id == 1U) GPIO_CLEAR_ODR(CS2_PORT, CS2_PIN);  // 2번 축 TMC5160 선택
    else if (axis_id == 2U) GPIO_CLEAR_ODR(CS3_PORT, CS3_PIN);  // 3번 축 TMC5160 선택
    else if (axis_id == 3U) GPIO_CLEAR_ODR(CS4_PORT, CS4_PIN);  // 4번 축 TMC5160 선택
}

static void tmc_cs_high(uint8_t axis_id)
{
    if (axis_id == 0U) GPIO_SET_ODR(CS1_PORT, CS1_PIN);       // 1번 축 TMC5160 선택 해제
    else if (axis_id == 1U) GPIO_SET_ODR(CS2_PORT, CS2_PIN);  // 2번 축 TMC5160 선택 해제
    else if (axis_id == 2U) GPIO_SET_ODR(CS3_PORT, CS3_PIN);  // 3번 축 TMC5160 선택 해제
    else if (axis_id == 3U) GPIO_SET_ODR(CS4_PORT, CS4_PIN);  // 4번 축 TMC5160 선택 해제
}

static void tmc5160_write(uint8_t axis_id, uint8_t addr, uint32_t data)
{
    uint8_t tx[5];  // TMC5160 write frame: 주소 1바이트 + 데이터 4바이트

    if (axis_id >= AXIS_COUNT) return;  // 잘못된 축 번호는 무시

    tx[0] = addr | 0x80U;          // write bit가 set된 레지스터 주소
    tx[1] = (uint8_t)(data >> 24); // 데이터 최상위 바이트
    tx[2] = (uint8_t)(data >> 16); // 데이터 상위 바이트
    tx[3] = (uint8_t)(data >> 8);  // 데이터 하위 바이트
    tx[4] = (uint8_t)data;         // 데이터 최하위 바이트

    tmc_cs_low(axis_id);  // 대상 축 TMC5160 선택
    for (uint8_t byte = 0; byte < 5U; byte++) {
        for (int8_t bit = 7; bit >= 0; bit--) {
            GPIO_CLEAR_ODR(TMC_CLK_PORT, TMC_CLK_PIN);  // clock low
            if (tx[byte] & (uint8_t)(1U << bit)) GPIO_SET_ODR(TMC_MOSI_PORT, TMC_MOSI_PIN);  // bit 값 1 출력
            else GPIO_CLEAR_ODR(TMC_MOSI_PORT, TMC_MOSI_PIN);  // bit 값 0 출력
            for (volatile int delay = 0; delay < 20; delay++) {}  // setup time 확보
            GPIO_SET_ODR(TMC_CLK_PORT, TMC_CLK_PIN);  // clock high로 bit 전송
            for (volatile int delay = 0; delay < 20; delay++) {}  // hold time 확보
        }
    }
    tmc_cs_high(axis_id);  // 대상 축 TMC5160 선택 해제
}

static void tmc5160_init_axis(uint8_t axis_id)
{
    tmc5160_write(axis_id, TMC_REG_GCONF, 0x00000000UL);       // 기본 global 설정
    tmc5160_write(axis_id, TMC_REG_IHOLD_IRUN, 0x00061004UL);  // hold/run 전류 설정
    tmc5160_write(axis_id, TMC_REG_CHOPCONF, 0x040100C3UL);    // chopper 및 microstep 설정
    tmc5160_write(axis_id, TMC_REG_PWMCONF, 0x00050480UL);     // PWM 모드 설정
}

void tmc5160_init_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        tmc5160_init_axis(i);  // 각 축 TMC5160 초기화
    }
}
