#include "../Inc/gpio.h"

// step 생성용
static void gpio_output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3 << (pin * 2));   // mode bit 초기화
    port->MODER |=  (1 << (pin * 2));   // output mode
    port->OTYPER &= ~(1 << pin);         // push-pull
    port->OSPEEDR &= ~(3 << (pin * 2)); // 출력 속도 초기화
    port->OSPEEDR |=  (2 << (pin * 2)); // 출력 속도 high speed 
    port->PUPDR &= ~(3 << (pin * 2));   // pull-up/pull-down 없음
}

static void gpio_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3 << (pin * 2));  // input mode
    port->PUPDR &= ~(3 << (pin * 2));  // pull 설정 bit 초기화
    port->PUPDR |=  (1 << (pin * 2));  // 내부 pull-up
}

// spi 핀
static void gpio_af5(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3 << (pin * 2));   //   mode bit 초기화
    port->MODER |=  (2 << (pin * 2));   // alternate function mode
    port->OTYPER &= ~(1 << pin);         // push-pull
    port->OSPEEDR &= ~(3 << (pin * 2)); // 출력 속도 bit 초기화
    port->OSPEEDR |=  (3 << (pin * 2)); // 출력 속도 very high speed
    port->PUPDR &= ~(3 << (pin * 2));   // pull 설정 bit 초기화
    port->PUPDR |=  (1 << (pin * 2));   // 내부 pull-up
    port->AFR[pin / 8] &= ~(0xF << ((pin % 8) * 4));  // alternate function bit 초기화
    port->AFR[pin / 8] |=  (0x5 << ((pin % 8) * 4));  // AF5(SPI2)
}

void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;  // GPIOA/B/C 클럭 활성화
    (void)RCC->AHB1ENR;  // 클럭 활성화 후 레지스터 반영 대기용 dummy read

    gpio_output(DIR1_PORT, DIR1_PIN);    // 1번 axis dir
    gpio_output(STEP1_PORT, STEP1_PIN);  // 1번 axis step
    gpio_output(CS1_PORT, CS1_PIN);      // 1번 axis TMC5160 CS
    gpio_output(DIR2_PORT, DIR2_PIN);    // 2번 axis dir
    gpio_output(STEP2_PORT, STEP2_PIN);  // 2번 axis step
    gpio_output(CS2_PORT, CS2_PIN);      // 2번 axis TMC5160 CS
    gpio_output(DIR3_PORT, DIR3_PIN);    // 3번 axis dir
    gpio_output(STEP3_PORT, STEP3_PIN);  // 3번 axis step
    gpio_output(CS3_PORT, CS3_PIN);      // 3번 axis TMC5160 CS
    gpio_output(DIR4_PORT, DIR4_PIN);    // 4번 axis dir
    gpio_output(STEP4_PORT, STEP4_PIN);  // 4번 axis step
    gpio_output(CS4_PORT, CS4_PIN);      // 4번 axis TMC2240 CS
    gpio_output(TMC_MOSI_PORT, TMC_MOSI_PIN);  // SPI MOSI
    gpio_output(TMC_CLK_PORT, TMC_CLK_PIN);    // SPI CLK

    GPIO_SET_PIN(MOTOR_EN_PORT, MOTOR_EN_PIN); // 모터 disable
    gpio_output(MOTOR_EN_PORT, MOTOR_EN_PIN);  // 모터 드라이버 en핀 설정
    gpio_output(MCP_CS_PORT, MCP_CS_PIN);      // MCP2515 SPI CS

    gpio_input_pullup(TMC_MISO_PORT, TMC_MISO_PIN);  // TMC MISO
    gpio_input_pullup(LIM1_PORT, LIM1_PIN);          // 1번 축 리미트 스위치 pull up
    gpio_input_pullup(LIM2_PORT, LIM2_PIN);          // 2번 축 리미트 스위치 pull up
    gpio_input_pullup(LIM3_PORT, LIM3_PIN);          // 3번 축 리미트 스위치 pull up
    gpio_input_pullup(LIM4_PORT, LIM4_PIN);          // 4번 축 리미트 스위치 pull up
    gpio_input_pullup(MCP_INT_PORT, MCP_INT_PIN);    // MCP2515 interrupt 입력

    gpio_af5(MCP_SCK_PORT, MCP_SCK_PIN);    // SPI2 SCK
    gpio_af5(MCP_MISO_PORT, MCP_MISO_PIN);  // SPI2 MISO
    gpio_af5(MCP_MOSI_PORT, MCP_MOSI_PIN);  // SPI2 MOSI

    GPIO_CLEAR_PIN(STEP1_PORT, STEP1_PIN);  // step 초기값 low
    GPIO_CLEAR_PIN(STEP2_PORT, STEP2_PIN);  // step 초기값 low
    GPIO_CLEAR_PIN(STEP3_PORT, STEP3_PIN);  // step 초기값 low
    GPIO_CLEAR_PIN(STEP4_PORT, STEP4_PIN);  // step 초기값 low
    GPIO_SET_PIN(CS1_PORT, CS1_PIN);        // TMC5160 CS 초기값 high
    GPIO_SET_PIN(CS2_PORT, CS2_PIN);        // TMC5160 CS 초기값 high
    GPIO_SET_PIN(CS3_PORT, CS3_PIN);        // TMC5160 CS 초기값 high
    GPIO_SET_PIN(CS4_PORT, CS4_PIN);        // TMC2240 CS 초기값 high
    GPIO_SET_PIN(TMC_CLK_PORT, TMC_CLK_PIN); // TMC CLK idle high
    GPIO_SET_PIN(MOTOR_EN_PORT, MOTOR_EN_PIN); // 모터 en핀 초기값 high(disable)
    GPIO_SET_PIN(MCP_CS_PORT, MCP_CS_PIN);   // MCP2515 CS 초기값 high
}

void motor_enable(void)
{
    GPIO_CLEAR_PIN(MOTOR_EN_PORT, MOTOR_EN_PIN);  // 모터en핀 active low
}

void motor_disable(void)
{
    GPIO_SET_PIN(MOTOR_EN_PORT, MOTOR_EN_PIN);  // 모터en핀 active low
}
