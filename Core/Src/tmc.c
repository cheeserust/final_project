#include "../Inc/tmc.h"
#include "../Inc/gpio.h"

#define TMC_REG_GCONF       0x00  // TMC global configuration 레지스터
#define TMC_REG_DRV_CONF    0x0A  // TMC2240 driver current range/slope 설정
#define TMC_REG_GLOBAL_SCALER 0x0B // TMC2240 global current scale 설정
#define TMC_REG_IHOLD_IRUN  0x10  // 모터 정지/구동 전류 설정 레지스터
#define TMC_REG_TPOWERDOWN  0x11  // standstill current down delay 설정
#define TMC_REG_CHOPCONF    0x6C  // chopper 및 microstep 설정 레지스터
#define TMC_REG_PWMCONF     0x70  // stealthChop PWM 설정 레지스터

#define TMC_5160     0
#define TMC_2240     1

#if BOARD_ID == BOARD_ID_BOARD1
static const uint8_t tmc_driver_type[AXIS_COUNT] = {
    TMC_5160,
    TMC_5160,
    TMC_5160,
    TMC_2240
};
#else
static const uint8_t tmc_driver_type[AXIS_COUNT] = {
    TMC_2240
};
#endif

static void tmc_cs_low(uint8_t axis_id)
{
    if (axis_id == 0) GPIO_CLEAR_PIN(CS1_PORT, CS1_PIN);       // 1번 축
    else if (axis_id == 1) GPIO_CLEAR_PIN(CS2_PORT, CS2_PIN);  // 2번 축
    else if (axis_id == 2) GPIO_CLEAR_PIN(CS3_PORT, CS3_PIN);  // 3번 축
    else if (axis_id == 3) GPIO_CLEAR_PIN(CS4_PORT, CS4_PIN);  // 4번 축
}

static void tmc_cs_high(uint8_t axis_id)
{
    if (axis_id == 0) GPIO_SET_PIN(CS1_PORT, CS1_PIN);       // 1번 축
    else if (axis_id == 1) GPIO_SET_PIN(CS2_PORT, CS2_PIN);  // 2번 축
    else if (axis_id == 2) GPIO_SET_PIN(CS3_PORT, CS3_PIN);  // 3번 축
    else if (axis_id == 3) GPIO_SET_PIN(CS4_PORT, CS4_PIN);  // 4번 축
}

static void tmc_write(uint8_t axis_id, uint8_t addr, uint32_t data)
{
    uint8_t tx[5];  //주소 1바이트 + 데이터 4바이트

    tx[0] = addr | 0x80;          // write bit가 set된 레지스터 주소
    tx[1] = (uint8_t)(data >> 24); // data MSB
    tx[2] = (uint8_t)(data >> 16); 
    tx[3] = (uint8_t)(data >> 8);  
    tx[4] = (uint8_t)data;         // data LSB

    tmc_cs_low(axis_id);  // 대상 축 TMC 선택
    for (uint8_t byte = 0; byte < 5; byte++) {
        for (int8_t bit = 7; bit >= 0; bit--) {
            GPIO_CLEAR_PIN(TMC_CLK_PORT, TMC_CLK_PIN);  // clock low
            if (tx[byte] & (uint8_t)(1 << bit)) GPIO_SET_PIN(TMC_MOSI_PORT, TMC_MOSI_PIN);  // bit 값 1 출력
            else GPIO_CLEAR_PIN(TMC_MOSI_PORT, TMC_MOSI_PIN);  // bit 값 0 출력
            for (volatile int delay = 0; delay < 20; delay++) {}  // setup time 확보
            GPIO_SET_PIN(TMC_CLK_PORT, TMC_CLK_PIN);  // clock high로 bit 전송
            for (volatile int delay = 0; delay < 20; delay++) {}  // hold time 확보
        }
    }
    tmc_cs_high(axis_id);  // 대상 축 TMC 선택 해제
}

static void tmc5160_init_axis(uint8_t axis_id)
{
    tmc_write(axis_id, TMC_REG_GCONF, 0x00000000);       // SpreadCycle, en_pwm_mode=0
    tmc_write(axis_id, TMC_REG_GLOBAL_SCALER, 0x00000000); // full scale current, 0은 256으로 처리
    if (axis_id == 0) {
        tmc_write(axis_id, TMC_REG_IHOLD_IRUN, (6 << 16) | (26 << 8) | 16);  // IHOLDDELAY=6 IRUN=24 IHOLD=17
    } else if (axis_id == 2) {
        tmc_write(axis_id, TMC_REG_IHOLD_IRUN, (6 << 16) | (16 << 8) | 4);  // IHOLDDELAY=6 IRUN=18 IHOLD=10
    } else {
        tmc_write(axis_id, TMC_REG_IHOLD_IRUN, (6 << 16) | (16 << 8) | 4);  // IHOLDDELAY=6 IRUN=16 IHOLD=4
    }
    tmc_write(axis_id, TMC_REG_TPOWERDOWN, 0x0000000A);  // standstill 전류 감소 대기
    tmc_write(axis_id, TMC_REG_CHOPCONF, 0x14010044);    // MRES=4 -> 16 microstep, intpol=1 -> 내부 256 microstep 보간, TPFD=4, TBL=2, chm=0 -> SpreadCycle, TOFF=4 
}

static void tmc2240_init_axis(uint8_t axis_id)
{
    tmc_write(axis_id, TMC_REG_GCONF, 0x00000000);        // SpreadCycle, STEP/DIR
    tmc_write(axis_id, TMC_REG_DRV_CONF, 0x00000002);     // 3A peak range
    tmc_write(axis_id, TMC_REG_GLOBAL_SCALER, 0x000000F2); // about 2.0A RMS at RREF=12k
    tmc_write(axis_id, TMC_REG_IHOLD_IRUN, (4 << 24) | (4 << 16) | (27 << 8) | 2); // IRUNDELAY=4 IHOLDDELAY=4 IRUN=27 IHOLD=2
    tmc_write(axis_id, TMC_REG_TPOWERDOWN, 0x0000000A);
    tmc_write(axis_id, TMC_REG_CHOPCONF, 0x14410155);     // 16 ustep + intpol, SpreadCycle, TOFF=5
}

void tmc5160_init_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (tmc_driver_type[i] == TMC_2240) {
            tmc2240_init_axis(i);
        } else {
            tmc5160_init_axis(i);
        }
    }
}
