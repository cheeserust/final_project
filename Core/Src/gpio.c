#include "../Inc/gpio.h"

static void gpio_output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3UL << (pin * 2U));   // 해당 핀 mode bit 초기화
    port->MODER |=  (1UL << (pin * 2U));   // general purpose output mode 설정
    port->OTYPER &= ~(1UL << pin);         // push-pull 출력 설정
    port->OSPEEDR &= ~(3UL << (pin * 2U)); // 출력 속도 bit 초기화
    port->OSPEEDR |=  (2UL << (pin * 2U)); // high speed 출력 설정
    port->PUPDR &= ~(3UL << (pin * 2U));   // pull-up/pull-down 없음
}

static void gpio_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3UL << (pin * 2U));  // input mode 설정
    port->PUPDR &= ~(3UL << (pin * 2U));  // pull 설정 bit 초기화
    port->PUPDR |=  (1UL << (pin * 2U));  // 내부 pull-up 활성화
}

static void gpio_af5(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3UL << (pin * 2U));   // 해당 핀 mode bit 초기화
    port->MODER |=  (2UL << (pin * 2U));   // alternate function mode 설정
    port->OTYPER &= ~(1UL << pin);         // push-pull 출력 설정
    port->OSPEEDR &= ~(3UL << (pin * 2U)); // 출력 속도 bit 초기화
    port->OSPEEDR |=  (3UL << (pin * 2U)); // very high speed 설정
    port->PUPDR &= ~(3UL << (pin * 2U));   // pull 설정 bit 초기화
    port->PUPDR |=  (1UL << (pin * 2U));   // 내부 pull-up 활성화
    port->AFR[pin / 8U] &= ~(0xFUL << ((pin % 8U) * 4U));  // alternate function bit 초기화
    port->AFR[pin / 8U] |=  (0x5UL << ((pin % 8U) * 4U));  // AF5(SPI2) 설정
}

void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;  // GPIOA/B/C 클럭 활성화
    (void)RCC->AHB1ENR;  // 클럭 활성화 후 레지스터 반영 대기용 dummy read

    gpio_output(DIR1_PORT, DIR1_PIN);    // 1번 축 방향 출력 핀
    gpio_output(STEP1_PORT, STEP1_PIN);  // 1번 축 step 출력 핀
    gpio_output(CS1_PORT, CS1_PIN);      // 1번 축 TMC5160 CS 핀
    gpio_output(DIR2_PORT, DIR2_PIN);    // 2번 축 방향 출력 핀
    gpio_output(STEP2_PORT, STEP2_PIN);  // 2번 축 step 출력 핀
    gpio_output(CS2_PORT, CS2_PIN);      // 2번 축 TMC5160 CS 핀
    gpio_output(DIR3_PORT, DIR3_PIN);    // 3번 축 방향 출력 핀
    gpio_output(STEP3_PORT, STEP3_PIN);  // 3번 축 step 출력 핀
    gpio_output(CS3_PORT, CS3_PIN);      // 3번 축 TMC5160 CS 핀
    gpio_output(DIR4_PORT, DIR4_PIN);    // 4번 축 방향 출력 핀
    gpio_output(STEP4_PORT, STEP4_PIN);  // 4번 축 step 출력 핀
    gpio_output(CS4_PORT, CS4_PIN);      // 4번 축 TMC5160 CS 핀
    gpio_output(TMC_MOSI_PORT, TMC_MOSI_PIN);  // TMC5160 bit-bang SPI MOSI 핀
    gpio_output(TMC_CLK_PORT, TMC_CLK_PIN);    // TMC5160 bit-bang SPI CLK 핀

    GPIO_SET_ODR(MOTOR_EN_PORT, MOTOR_EN_PIN); // output 전환 순간에도 모터 disable 상태 유지
    gpio_output(MOTOR_EN_PORT, MOTOR_EN_PIN);  // 모터 드라이버 enable 핀
    gpio_output(MCP_CS_PORT, MCP_CS_PIN);      // MCP2515 SPI CS 핀

    gpio_input_pullup(TMC_MISO_PORT, TMC_MISO_PIN);  // TMC5160 MISO 입력 핀
    gpio_input_pullup(LIM1_PORT, LIM1_PIN);          // 1번 축 리미트 스위치 입력
    gpio_input_pullup(LIM2_PORT, LIM2_PIN);          // 2번 축 리미트 스위치 입력
    gpio_input_pullup(LIM3_PORT, LIM3_PIN);          // 3번 축 리미트 스위치 입력
    gpio_input_pullup(LIM4_PORT, LIM4_PIN);          // 4번 축 리미트 스위치 입력
    gpio_input_pullup(MCP_INT_PORT, MCP_INT_PIN);    // MCP2515 interrupt 입력 핀

    gpio_af5(MCP_SCK_PORT, MCP_SCK_PIN);    // SPI2 SCK 핀 설정
    gpio_af5(MCP_MISO_PORT, MCP_MISO_PIN);  // SPI2 MISO 핀 설정
    gpio_af5(MCP_MOSI_PORT, MCP_MOSI_PIN);  // SPI2 MOSI 핀 설정

    GPIO_CLEAR_ODR(STEP1_PORT, STEP1_PIN);  // step 출력 초기값 low
    GPIO_CLEAR_ODR(STEP2_PORT, STEP2_PIN);  // step 출력 초기값 low
    GPIO_CLEAR_ODR(STEP3_PORT, STEP3_PIN);  // step 출력 초기값 low
    GPIO_CLEAR_ODR(STEP4_PORT, STEP4_PIN);  // step 출력 초기값 low
    GPIO_SET_ODR(CS1_PORT, CS1_PIN);        // TMC5160 CS 초기값 high
    GPIO_SET_ODR(CS2_PORT, CS2_PIN);        // TMC5160 CS 초기값 high
    GPIO_SET_ODR(CS3_PORT, CS3_PIN);        // TMC5160 CS 초기값 high
    GPIO_SET_ODR(CS4_PORT, CS4_PIN);        // TMC5160 CS 초기값 high
    GPIO_SET_ODR(TMC_CLK_PORT, TMC_CLK_PIN); // TMC5160 CLK idle high
    GPIO_SET_ODR(MOTOR_EN_PORT, MOTOR_EN_PIN); // 모터 enable 핀 초기값 high(disable)
    GPIO_SET_ODR(MCP_CS_PORT, MCP_CS_PIN);   // MCP2515 CS 초기값 high
}

void motor_enable(void)
{
    GPIO_CLEAR_ODR(MOTOR_EN_PORT, MOTOR_EN_PIN);  // enable 핀 active-low: 모터 출력 활성화
}

void motor_disable(void)
{
    GPIO_SET_ODR(MOTOR_EN_PORT, MOTOR_EN_PIN);  // enable 핀 high: 모터 출력 비활성화
}
