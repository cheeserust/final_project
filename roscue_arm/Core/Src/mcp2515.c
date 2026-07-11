#include "../Inc/mcp2515.h"
#include "../Inc/gpio.h"
#include <stdint.h>

volatile uint8_t g_mcp2515_irq_pending = 0;

/* MCP2515 SPI commands */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BITMOD      0x05
#define MCP_READ_RXB0   0x90
#define MCP_READ_RXB1   0x94
#define MCP_RTS_TX0     0x81
#define MCP_RTS_TX1     0x82
#define MCP_RTS_TX2     0x84

/* MCP2515 registers */
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCP_CANINTE     0x2B
#define MCP_CANINTF     0x2C
#define MCP_EFLG        0x2D

#define MCP_RXF0SIDH    0x00
#define MCP_RXF0SIDL    0x01
#define MCP_RXF1SIDH    0x04
#define MCP_RXF1SIDL    0x05
#define MCP_RXF2SIDH    0x08
#define MCP_RXF2SIDL    0x09
#define MCP_RXF3SIDH    0x10
#define MCP_RXF3SIDL    0x11
#define MCP_RXF4SIDH    0x14
#define MCP_RXF4SIDL    0x15
#define MCP_RXF5SIDH    0x18
#define MCP_RXF5SIDL    0x19
#define MCP_RXM0SIDH    0x20
#define MCP_RXM0SIDL    0x21
#define MCP_RXM1SIDH    0x24
#define MCP_RXM1SIDL    0x25

#define MCP_TXB0CTRL    0x30
#define MCP_TXB0SIDH    0x31
#define MCP_TXB0SIDL    0x32
#define MCP_TXB0DLC     0x35
#define MCP_TXB0D0      0x36

#define MCP_TXB1CTRL    0x40
#define MCP_TXB1SIDH    0x41
#define MCP_TXB1SIDL    0x42
#define MCP_TXB1DLC     0x45
#define MCP_TXB1D0      0x46

#define MCP_TXB2CTRL    0x50
#define MCP_TXB2SIDH    0x51
#define MCP_TXB2SIDL    0x52
#define MCP_TXB2DLC     0x55
#define MCP_TXB2D0      0x56

#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB0SIDL    0x62
#define MCP_RXB0DLC     0x65
#define MCP_RXB0D0      0x66

#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71
#define MCP_RXB1SIDL    0x72
#define MCP_RXB1DLC     0x75
#define MCP_RXB1D0      0x76

#define MCP_RX0IF       0x01
#define MCP_RX1IF       0x02
#define MCP_TX0IF       0x04
#define MCP_TX1IF       0x08
#define MCP_TX2IF       0x10
#define MCP_ERRIF       0x20
#define MCP_WAKIF       0x40
#define MCP_MERRF       0x80

#define MCP_TXREQ       0x08

#define MCP_MODE_NORMAL 0x00
#define MCP_MODE_CONFIG 0x80

#define MCP_EFLG_RX0OVR 0x01
#define MCP_EFLG_RX1OVR 0x02
#define MCP_EFLG_TXBO   0x04

#define MCP_SPI_TIMEOUT_LOOPS 100000
#define MCP_SEND_FAIL_RECOVER_THRESHOLD 10

static volatile uint8_t g_mcp2515_spi_fault;
static uint8_t g_mcp2515_send_fail_count;

static uint8_t mcp2515_recover(void);

static void mcp_delay(void)
{
    volatile int i;
    for (i = 0; i < 30000; i++) { __NOP(); }
}

static void mcp2515_cs_low(void)
{
    GPIO_CLEAR_PIN(MCP_CS_PORT, MCP_CS_PIN);
}

static void mcp2515_cs_high(void)
{
    uint32_t timeout = MCP_SPI_TIMEOUT_LOOPS;

    while (SPI2->SR & SPI_SR_BSY) {
        if (timeout-- == 0) {
            g_mcp2515_spi_fault = 1;
            break;
        }
    }
    GPIO_SET_PIN(MCP_CS_PORT, MCP_CS_PIN);
}

static uint8_t mcp2515_spi_txrx_byte(uint8_t data)
{
    uint32_t timeout = MCP_SPI_TIMEOUT_LOOPS;

    while (!(SPI2->SR & SPI_SR_TXE)) {
        if (timeout-- == 0) {
            g_mcp2515_spi_fault = 1;
            return 0;
        }
    }
    *(__IO uint8_t *)&SPI2->DR = data;

    timeout = MCP_SPI_TIMEOUT_LOOPS;
    while (!(SPI2->SR & SPI_SR_RXNE)) {
        if (timeout-- == 0) {
            g_mcp2515_spi_fault = 1;
            return 0;
        }
    }
    return *(__IO uint8_t *)&SPI2->DR;
}

static uint8_t mcp_read_reg(uint8_t addr)
{
    uint8_t v;

    mcp2515_cs_low();
    mcp2515_spi_txrx_byte(MCP_READ);
    mcp2515_spi_txrx_byte(addr);
    v = mcp2515_spi_txrx_byte(0xFF);
    mcp2515_cs_high();

    return v;
}

static void mcp_write_reg(uint8_t addr, uint8_t data)
{
    mcp2515_cs_low();
    mcp2515_spi_txrx_byte(MCP_WRITE);
    mcp2515_spi_txrx_byte(addr);
    mcp2515_spi_txrx_byte(data);
    mcp2515_cs_high();
}

static void mcp_write_standard_id(uint8_t sidh_addr, uint16_t id)
{
    mcp_write_reg(sidh_addr, (uint8_t)(id >> 3));
    mcp_write_reg((uint8_t)(sidh_addr + 1), (uint8_t)((id & 0x7) << 5));
    mcp_write_reg((uint8_t)(sidh_addr + 2), 0x00);
    mcp_write_reg((uint8_t)(sidh_addr + 3), 0x00);
}

static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    mcp2515_cs_low();
    mcp2515_spi_txrx_byte(MCP_BITMOD);
    mcp2515_spi_txrx_byte(addr);
    mcp2515_spi_txrx_byte(mask);
    mcp2515_spi_txrx_byte(data);
    mcp2515_cs_high();
}

static void mcp2515_abort_all_tx(void)
{
    /* Controller reset/recovery path only. Runtime TX busy must never call this. */
    mcp_bit_modify(MCP_CANCTRL, 0x10, 0x10);  /* ABAT=1 */
    mcp_delay();
    mcp_bit_modify(MCP_CANCTRL, 0x10, 0x00);  /* ABAT=0 */

    mcp_bit_modify(MCP_TXB0CTRL, MCP_TXREQ, 0x00);
    mcp_bit_modify(MCP_TXB1CTRL, MCP_TXREQ, 0x00);
    mcp_bit_modify(MCP_TXB2CTRL, MCP_TXREQ, 0x00);

    mcp_bit_modify(MCP_CANINTF,
                   (uint8_t)(MCP_TX0IF | MCP_TX1IF | MCP_TX2IF | MCP_ERRIF | MCP_MERRF),
                   0x00);
}

uint8_t mcp2515_int_asserted(void)
{
    /* MCP2515 INT is active-low. This is used as a level fallback when
     * the falling-edge EXTI was missed or the INT line is already low.
     */
    return ((MCP_INT_PORT->IDR & (1 << MCP_INT_PIN)) == 0) ? 1 : 0;
}

static void mcp_reset(void)
{
    mcp2515_cs_low();
    mcp2515_spi_txrx_byte(MCP_RESET);
    mcp2515_cs_high();
    mcp_delay();
}

static int mcp_set_mode(uint8_t mode)
{
    int timeout = 10000;

    mcp_bit_modify(MCP_CANCTRL, 0xE0, mode);
    if (g_mcp2515_spi_fault) return -1;

    while (timeout-- > 0) {
        if ((mcp_read_reg(MCP_CANSTAT) & 0xE0) == mode) {
            return 0;
        }
        if (g_mcp2515_spi_fault) return -1;
    }

    return -1;
}

static void mcp_set_500k_bitrate(void)
{
    /* Fosc = 8 MHz, CAN bitrate = 500 kbps */
    mcp_write_reg(MCP_CNF1, 0x00);
    mcp_write_reg(MCP_CNF2, 0x98);
    mcp_write_reg(MCP_CNF3, 0x01);
}

static void mcp_configure_rx_filters(void)
{
    mcp_write_standard_id(MCP_RXM0SIDH, 0x7FF);
    mcp_write_standard_id(MCP_RXM1SIDH, 0x7FF);

    mcp_write_standard_id(MCP_RXF0SIDH, CAN_ID_ESTOP);
    mcp_write_standard_id(MCP_RXF1SIDH, CAN_ID_ENABLE);
    mcp_write_standard_id(MCP_RXF2SIDH, CAN_ID_HOMING);
    mcp_write_standard_id(MCP_RXF3SIDH, CAN_ID_CLEAR_ERROR);
    mcp_write_standard_id(MCP_RXF4SIDH, BOARD_MOVE_CAN_ID);
    mcp_write_standard_id(MCP_RXF5SIDH, BOARD_MOVE_CAN_ID);

    mcp_write_reg(MCP_RXB0CTRL, 0x04);   /* Use filters, enable rollover to RXB1 */
    mcp_write_reg(MCP_RXB1CTRL, 0x00);   /* Use filters */
}

static void mcp_exti_init(void)
{
    /* MCP2515 INT, active-low falling edge */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    MCP_INT_PORT->MODER &= ~(0x3 << (MCP_INT_PIN * 2));
    MCP_INT_PORT->PUPDR &= ~(0x3 << (MCP_INT_PIN * 2));
    MCP_INT_PORT->PUPDR |=  (0x1 << (MCP_INT_PIN * 2));

    SYSCFG->EXTICR[MCP_INT_PIN / 4] &= ~(0xF << ((MCP_INT_PIN % 4) * 4));
    SYSCFG->EXTICR[MCP_INT_PIN / 4] |=  (MCP_INT_EXTICR_PORT << ((MCP_INT_PIN % 4) * 4));

    EXTI->FTSR |= (1 << MCP_INT_PIN);
    EXTI->RTSR &= ~(1 << MCP_INT_PIN);
    EXTI->PR = (1 << MCP_INT_PIN);
    EXTI->IMR |= (1 << MCP_INT_PIN);

// #if BOARD_ID == 1 && AXIS_COUNT > 1
//     SYSCFG->EXTICR[LIM2_PIN / 4] &= ~(0xF << ((LIM2_PIN % 4) * 4));
//     EXTI->FTSR |= (1 << LIM2_PIN);
//     EXTI->RTSR &= ~(1 << LIM2_PIN);
//     EXTI->PR = (1 << LIM2_PIN);
//     EXTI->IMR |= (1 << LIM2_PIN);
// #endif

    NVIC_SetPriority(MCP_INT_IRQn, 5);
    NVIC_EnableIRQ(MCP_INT_IRQn);
}

uint8_t mcp2515_init_500k(void)
{
    g_mcp2515_irq_pending = 0;
    g_mcp2515_spi_fault = 0;

    mcp_reset();

    if (mcp_set_mode(MCP_MODE_CONFIG) != 0) {
        return 0;
    }

    mcp_set_500k_bitrate();

    mcp_configure_rx_filters();

    mcp_write_reg(MCP_CANINTF, 0x00);
    mcp_write_reg(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF | MCP_ERRIF | MCP_MERRF);
    mcp2515_abort_all_tx();

    mcp_exti_init();

    if (mcp_set_mode(MCP_MODE_NORMAL) != 0) {
        return 0;
    }

    g_mcp2515_send_fail_count = 0;
    return 1;
}

static uint8_t mcp2515_recover(void)
{
    g_mcp2515_send_fail_count = 0;
    g_mcp2515_spi_fault = 0;

    return mcp2515_init_500k();
}

uint8_t mcp2515_service(void)
{
    uint8_t intf;
    uint8_t eflg;

    if (g_mcp2515_spi_fault) return mcp2515_recover();

    intf = mcp_read_reg(MCP_CANINTF);
    eflg = mcp_read_reg(MCP_EFLG);
    if (g_mcp2515_spi_fault) return mcp2515_recover();

    if (eflg & (MCP_EFLG_TXBO | MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR)) {
        return mcp2515_recover();
    }

    if (intf & (MCP_ERRIF | MCP_MERRF | MCP_WAKIF)) {
        mcp_bit_modify(MCP_CANINTF, (uint8_t)(MCP_ERRIF | MCP_MERRF | MCP_WAKIF), 0x00);
    }

    return g_mcp2515_spi_fault ? mcp2515_recover() : 1;
}

static void mcp_read_rx_buffer(uint8_t read_cmd, CanFrame *frame)
{
    uint8_t sidh;
    uint8_t sidl;
    uint8_t dlc;
    uint8_t i;

    mcp2515_cs_low();
    mcp2515_spi_txrx_byte(read_cmd);
    sidh = mcp2515_spi_txrx_byte(0xFF);
    sidl = mcp2515_spi_txrx_byte(0xFF);
    (void)mcp2515_spi_txrx_byte(0xFF);  /* EID8 */
    (void)mcp2515_spi_txrx_byte(0xFF);  /* EID0 */
    dlc = mcp2515_spi_txrx_byte(0xFF) & 0x0F;

    frame->id = ((uint16_t)sidh << 3) | ((uint16_t)sidl >> 5);
    if (dlc > 8) dlc = 8;
    frame->dlc = dlc;

    for (i = 0; i < 8; i++) {
        frame->data[i] = mcp2515_spi_txrx_byte(0xFF);
    }
    mcp2515_cs_high();
}

uint8_t mcp2515_read_frame(CanFrame *frame)
{
    uint8_t intf;

    if (frame == 0) return 0;
    if (g_mcp2515_spi_fault && !mcp2515_recover()) return 0;

    intf = mcp_read_reg(MCP_CANINTF);
    if (g_mcp2515_spi_fault) {
        (void)mcp2515_recover();
        return 0;
    }

    if (intf & MCP_RX0IF) {
        mcp_read_rx_buffer(MCP_READ_RXB0, frame);
        mcp_bit_modify(MCP_CANINTF, MCP_RX0IF, 0x00);
        return 1;
    }

    if (intf & MCP_RX1IF) {
        mcp_read_rx_buffer(MCP_READ_RXB1, frame);
        mcp_bit_modify(MCP_CANINTF, MCP_RX1IF, 0x00);
        return 1;
    }

    if (intf & (MCP_ERRIF | MCP_MERRF | MCP_WAKIF)) {
        mcp_bit_modify(MCP_CANINTF, (uint8_t)(MCP_ERRIF | MCP_MERRF | MCP_WAKIF), 0x00);
        (void)mcp2515_service();
    }

    return 0;
}

static int mcp_try_load_tx(uint8_t ctrl_addr,
                           uint8_t sidh_addr,
                           uint8_t sidl_addr,
                           uint8_t dlc_addr,
                           uint8_t data_addr,
                           uint8_t rts_cmd,
                           const CanFrame *frame)
{
    uint8_t i;

    uint8_t ctrl = mcp_read_reg(ctrl_addr);

    if (g_mcp2515_spi_fault) return -1;
    if ((ctrl & MCP_TXREQ) != 0) {
        return -2;  /* TX buffer busy. No runtime wait. */
    }

    mcp_write_reg(sidh_addr, (uint8_t)(frame->id >> 3));
    mcp_write_reg(sidl_addr, (uint8_t)((frame->id & 0x7) << 5));
    mcp_write_reg(dlc_addr, frame->dlc & 0x0F);

    for (i = 0; i < frame->dlc; i++) {
        mcp_write_reg((uint8_t)(data_addr + i), frame->data[i]);
    }

    mcp2515_cs_low();
    mcp2515_spi_txrx_byte(rts_cmd);
    mcp2515_cs_high();

    return g_mcp2515_spi_fault ? -1 : 0;
}

static void mcp2515_note_send_result(uint8_t ok)
{
    if (ok) {
        g_mcp2515_send_fail_count = 0;
        return;
    }

    if (g_mcp2515_send_fail_count < 255) {
        g_mcp2515_send_fail_count++;
    }

    if (g_mcp2515_send_fail_count >= MCP_SEND_FAIL_RECOVER_THRESHOLD) {
        (void)mcp2515_recover();
    }
}

Mcp2515SendResult mcp2515_send_frame(const CanFrame *frame)
{
    int ret;

    if (frame == 0 || frame->dlc > 8) return MCP2515_SEND_FAULT;
    if (!mcp2515_service()) {
        mcp2515_note_send_result(0);
        return MCP2515_SEND_FAULT;
    }

    ret = mcp_try_load_tx(MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0SIDL,
                          MCP_TXB0DLC, MCP_TXB0D0, MCP_RTS_TX0, frame);
    if (ret == 0) {
        mcp2515_note_send_result(1);
        return MCP2515_SEND_OK;
    }
    if (ret == -1) goto send_fault;

    ret = mcp_try_load_tx(MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1SIDL,
                          MCP_TXB1DLC, MCP_TXB1D0, MCP_RTS_TX1, frame);
    if (ret == 0) {
        mcp2515_note_send_result(1);
        return MCP2515_SEND_OK;
    }
    if (ret == -1) goto send_fault;

    ret = mcp_try_load_tx(MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2SIDL,
                          MCP_TXB2DLC, MCP_TXB2D0, MCP_RTS_TX2, frame);
    if (ret == 0) {
        mcp2515_note_send_result(1);
        return MCP2515_SEND_OK;
    }
    if (ret == -1) goto send_fault;

    /* All three buffers are legitimately busy. Preserve them and retry later. */
    return MCP2515_SEND_BUSY;

send_fault:
    mcp2515_note_send_result(0);
    return MCP2515_SEND_FAULT;
}

void spi2_init(void)
{
    volatile int i;
    unsigned int br_bits = 0;
    unsigned int tmp = 64;

    /* Convert divider to STM32 BR field approximately.
     * div=2 -> 0, div=4 -> 1, ... div=256 -> 7
     */
    while (tmp > 2) {
        tmp >>= 1;
        br_bits++;
    }
    if (br_bits > 7) br_bits = 7;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    /* SPI2 reset */
    RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
    for (i = 0; i < 1000; i++) { __NOP(); }
    RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

    /* MCP2515 CS output, default high */
    MCP_CS_PORT->MODER &= ~(0x3 << (MCP_CS_PIN * 2));
    MCP_CS_PORT->MODER |=  (0x1 << (MCP_CS_PIN * 2));
    MCP_CS_PORT->OTYPER &= ~(1 << MCP_CS_PIN);
    MCP_CS_PORT->OSPEEDR &= ~(0x3 << (MCP_CS_PIN * 2));
    MCP_CS_PORT->OSPEEDR |=  (0x3 << (MCP_CS_PIN * 2));
    MCP_CS_PORT->PUPDR &= ~(0x3 << (MCP_CS_PIN * 2));
    GPIO_SET_PIN(MCP_CS_PORT, MCP_CS_PIN);

    /* PB13/PB14/PB15 = AF5 SPI2 */
    MCP_SCK_PORT->MODER &= ~(0x3 << (MCP_SCK_PIN * 2));
    MCP_SCK_PORT->MODER |=  (0x2 << (MCP_SCK_PIN * 2));
    MCP_MISO_PORT->MODER &= ~(0x3 << (MCP_MISO_PIN * 2));
    MCP_MISO_PORT->MODER |=  (0x2 << (MCP_MISO_PIN * 2));
    MCP_MOSI_PORT->MODER &= ~(0x3 << (MCP_MOSI_PIN * 2));
    MCP_MOSI_PORT->MODER |=  (0x2 << (MCP_MOSI_PIN * 2));

    MCP_SCK_PORT->OTYPER &= ~(1 << MCP_SCK_PIN);
    MCP_MISO_PORT->OTYPER &= ~(1 << MCP_MISO_PIN);
    MCP_MOSI_PORT->OTYPER &= ~(1 << MCP_MOSI_PIN);

    MCP_SCK_PORT->OSPEEDR |= (0x3 << (MCP_SCK_PIN * 2));
    MCP_MISO_PORT->OSPEEDR |= (0x3 << (MCP_MISO_PIN * 2));
    MCP_MOSI_PORT->OSPEEDR |= (0x3 << (MCP_MOSI_PIN * 2));

    MCP_SCK_PORT->PUPDR &= ~(0x3 << (MCP_SCK_PIN * 2));
    MCP_MISO_PORT->PUPDR &= ~(0x3 << (MCP_MISO_PIN * 2));
    MCP_MOSI_PORT->PUPDR &= ~(0x3 << (MCP_MOSI_PIN * 2));

    MCP_SCK_PORT->AFR[MCP_SCK_PIN / 8] &= ~(0xF << ((MCP_SCK_PIN % 8) * 4));
    MCP_SCK_PORT->AFR[MCP_SCK_PIN / 8] |=  (0x5 << ((MCP_SCK_PIN % 8) * 4));
    MCP_MISO_PORT->AFR[MCP_MISO_PIN / 8] &= ~(0xF << ((MCP_MISO_PIN % 8) * 4));
    MCP_MISO_PORT->AFR[MCP_MISO_PIN / 8] |=  (0x5 << ((MCP_MISO_PIN % 8) * 4));
    MCP_MOSI_PORT->AFR[MCP_MOSI_PIN / 8] &= ~(0xF << ((MCP_MOSI_PIN % 8) * 4));
    MCP_MOSI_PORT->AFR[MCP_MOSI_PIN / 8] |=  (0x5 << ((MCP_MOSI_PIN % 8) * 4));

    /* SPI mode 0, 8-bit, master, software NSS.
     * Current project clock: PCLK1 = 48 MHz.
     * div=64 -> SPI2 SCK around 750 kHz.
     * MCP2515 is stable at this speed during bring-up.
     */
    SPI2->CR1 =
        (0 << 11) |
        (0 << 10) |
        (1 << 9)  |
        (1 << 8)  |
        (0 << 7)  |
        (br_bits << 3) |
        (1 << 2)  |
        (0 << 1)  |
        (0 << 0);

    SPI2->CR2 = 0;
    SPI2->CR1 |= SPI_CR1_SPE;
}

// 데이터가 can 모듈 들어오면 INT핀이 LOW로 떨어짐 -> exti 인터럽트 걸림
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1 << MCP_INT_PIN)) {
        EXTI->PR = (1 << MCP_INT_PIN);
        g_mcp2515_irq_pending = 1;
    }
}
