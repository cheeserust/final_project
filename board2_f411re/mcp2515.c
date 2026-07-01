#include "device_driver.h"
#include <stdint.h>

volatile unsigned int g_mcp2515_irq = 0;

/* MCP2515 SPI commands */
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BITMOD      0x05
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

static void mcp_delay(void)
{
    volatile int i;
    for (i = 0; i < 30000; i++) { __NOP(); }
}

static uint8_t mcp_read_reg(uint8_t addr)
{
    uint8_t v;

    MCP2515_CS_Low();
    MCP2515_SPI_TxRx_Byte(MCP_READ);
    MCP2515_SPI_TxRx_Byte(addr);
    v = MCP2515_SPI_TxRx_Byte(0xFF);
    MCP2515_CS_High();

    return v;
}

static void mcp_write_reg(uint8_t addr, uint8_t data)
{
    MCP2515_CS_Low();
    MCP2515_SPI_TxRx_Byte(MCP_WRITE);
    MCP2515_SPI_TxRx_Byte(addr);
    MCP2515_SPI_TxRx_Byte(data);
    MCP2515_CS_High();
}

static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    MCP2515_CS_Low();
    MCP2515_SPI_TxRx_Byte(MCP_BITMOD);
    MCP2515_SPI_TxRx_Byte(addr);
    MCP2515_SPI_TxRx_Byte(mask);
    MCP2515_SPI_TxRx_Byte(data);
    MCP2515_CS_High();
}


void MCP2515_Abort_All_Tx(void)
{
    /* Bring-up robustness:
     * If STM32 booted before Linux can0 was up, MCP2515 may keep old TXREQ
     * frames retrying forever. Abort them so the next command response can be sent.
     */
    mcp_bit_modify(MCP_CANCTRL, 0x10U, 0x10U);  /* ABAT=1 */
    mcp_delay();
    mcp_bit_modify(MCP_CANCTRL, 0x10U, 0x00U);  /* ABAT=0 */

    mcp_bit_modify(MCP_TXB0CTRL, MCP_TXREQ, 0x00U);
    mcp_bit_modify(MCP_TXB1CTRL, MCP_TXREQ, 0x00U);
    mcp_bit_modify(MCP_TXB2CTRL, MCP_TXREQ, 0x00U);

    mcp_bit_modify(MCP_CANINTF, (uint8_t)(MCP_TX0IF | MCP_TX1IF | MCP_TX2IF | MCP_ERRIF | MCP_MERRF), 0x00U);
}

int MCP2515_Int_Asserted(void)
{
    /* MCP2515 INT is active-low on PB4. This is used as a level fallback when
     * the falling-edge EXTI was missed or the INT line is already low.
     */
    return ((GPIOB->IDR & (1U << 4)) == 0U) ? 1 : 0;
}

static void mcp_reset(void)
{
    MCP2515_CS_Low();
    MCP2515_SPI_TxRx_Byte(MCP_RESET);
    MCP2515_CS_High();
    mcp_delay();
}

static int mcp_set_mode(uint8_t mode)
{
    int timeout = 10000;

    mcp_bit_modify(MCP_CANCTRL, 0xE0, mode);

    while (timeout-- > 0)
    {
        if ((mcp_read_reg(MCP_CANSTAT) & 0xE0) == mode)
        {
            return 0;
        }
    }

    return -1;
}

static void mcp_set_500k_bitrate(int osc_mhz)
{
    if (osc_mhz == MCP2515_OSC_8MHZ)
    {
        /* Fosc = 8 MHz, 500 kbps */
        mcp_write_reg(MCP_CNF1, 0x00);
        mcp_write_reg(MCP_CNF2, 0x98);
        mcp_write_reg(MCP_CNF3, 0x01);
    }
    else
    {
        /* Fosc = 16 MHz, 500 kbps */
        mcp_write_reg(MCP_CNF1, 0x00);
        mcp_write_reg(MCP_CNF2, 0xAC);
        mcp_write_reg(MCP_CNF3, 0x03);
    }
}

static void mcp_exti_pb4_init(void)
{
    /* MCP2515 INT -> PB4, active-low falling edge */
    Macro_Set_Bit(RCC->AHB1ENR, 1);     /* GPIOB clock */
    Macro_Set_Bit(RCC->APB2ENR, 14);    /* SYSCFG clock */
    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    /* PB4 input + pull-up */
    Macro_Write_Block(GPIOB->MODER, 0x3, 0x0, 8);
    Macro_Write_Block(GPIOB->PUPDR, 0x3, 0x1, 8);

    /* EXTI4 source = PB4. EXTICR[1], bits 3:0 = 1 for port B */
    Macro_Write_Block(SYSCFG->EXTICR[1], 0xF, 0x1, 0);

    Macro_Set_Bit(EXTI->FTSR, 4);
    Macro_Clear_Bit(EXTI->RTSR, 4);
    EXTI->PR = (1U << 4);
    Macro_Set_Bit(EXTI->IMR, 4);

    NVIC_SetPriority(EXTI4_IRQn, 5);
    NVIC_EnableIRQ(EXTI4_IRQn);
}

int MCP2515_Init(int osc_mhz)
{
    g_mcp2515_irq = 0;

    mcp_reset();

    if (mcp_set_mode(MCP_MODE_CONFIG) != 0)
    {
        return -1;
    }

    mcp_set_500k_bitrate(osc_mhz);

    /* Accept all standard IDs during bring-up. Filtering is done in firmware. */
    mcp_write_reg(MCP_RXB0CTRL, 0x64);   /* RXM=11, BUKT=1 */
    mcp_write_reg(MCP_RXB1CTRL, 0x60);   /* RXM=11 */

    mcp_write_reg(MCP_CANINTF, 0x00);
    mcp_write_reg(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF | MCP_ERRIF | MCP_MERRF);
    MCP2515_Abort_All_Tx();

    mcp_exti_pb4_init();

    if (mcp_set_mode(MCP_MODE_NORMAL) != 0)
    {
        return -2;
    }

    return 0;
}

static void mcp_read_rx_buffer(uint8_t sidh_addr,
                               uint8_t sidl_addr,
                               uint8_t dlc_addr,
                               uint8_t data_addr,
                               CAN_Frame_t *frame)
{
    uint8_t sidh;
    uint8_t sidl;
    uint8_t dlc;
    uint8_t i;

    sidh = mcp_read_reg(sidh_addr);
    sidl = mcp_read_reg(sidl_addr);

    frame->id = ((uint16_t)sidh << 3) | ((uint16_t)sidl >> 5);

    dlc = mcp_read_reg(dlc_addr) & 0x0F;
    if (dlc > 8U) dlc = 8U;

    frame->dlc = dlc;
    for (i = 0; i < 8U; i++) frame->data[i] = 0U;
    for (i = 0; i < dlc; i++)
    {
        frame->data[i] = mcp_read_reg((uint8_t)(data_addr + i));
    }
}

int MCP2515_Read_Frame(CAN_Frame_t *frame)
{
    uint8_t intf;

    intf = mcp_read_reg(MCP_CANINTF);

    if (intf & MCP_RX0IF)
    {
        mcp_read_rx_buffer(MCP_RXB0SIDH,
                           MCP_RXB0SIDL,
                           MCP_RXB0DLC,
                           MCP_RXB0D0,
                           frame);
        mcp_bit_modify(MCP_CANINTF, MCP_RX0IF, 0x00);
        return 1;
    }

    if (intf & MCP_RX1IF)
    {
        mcp_read_rx_buffer(MCP_RXB1SIDH,
                           MCP_RXB1SIDL,
                           MCP_RXB1DLC,
                           MCP_RXB1D0,
                           frame);
        mcp_bit_modify(MCP_CANINTF, MCP_RX1IF, 0x00);
        return 1;
    }

    if (intf & (MCP_ERRIF | MCP_MERRF | MCP_WAKIF))
    {
        mcp_bit_modify(MCP_CANINTF, (uint8_t)(MCP_ERRIF | MCP_MERRF | MCP_WAKIF), 0x00);
    }

    return 0;
}

static int mcp_try_load_tx(uint8_t ctrl_addr,
                           uint8_t sidh_addr,
                           uint8_t sidl_addr,
                           uint8_t dlc_addr,
                           uint8_t data_addr,
                           uint8_t rts_cmd,
                           const CAN_Frame_t *frame)
{
    uint8_t i;

    if ((mcp_read_reg(ctrl_addr) & MCP_TXREQ) != 0U)
    {
        return -2;  /* TX buffer busy. No runtime wait. */
    }

    mcp_write_reg(sidh_addr, (uint8_t)(frame->id >> 3));
    mcp_write_reg(sidl_addr, (uint8_t)((frame->id & 0x7U) << 5));
    mcp_write_reg(dlc_addr, frame->dlc & 0x0FU);

    for (i = 0; i < frame->dlc; i++)
    {
        mcp_write_reg((uint8_t)(data_addr + i), frame->data[i]);
    }

    MCP2515_CS_Low();
    MCP2515_SPI_TxRx_Byte(rts_cmd);
    MCP2515_CS_High();

    return 0;
}

int MCP2515_Send_Frame(const CAN_Frame_t *frame)
{
    int ret;
    uint8_t retry;

    if (frame->dlc > 8U) return -1;

    for (retry = 0U; retry < 2U; retry++)
    {
        ret = mcp_try_load_tx(MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0SIDL,
                              MCP_TXB0DLC, MCP_TXB0D0, MCP_RTS_TX0, frame);
        if (ret == 0) return 0;

        ret = mcp_try_load_tx(MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1SIDL,
                              MCP_TXB1DLC, MCP_TXB1D0, MCP_RTS_TX1, frame);
        if (ret == 0) return 0;

        ret = mcp_try_load_tx(MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2SIDL,
                              MCP_TXB2DLC, MCP_TXB2D0, MCP_RTS_TX2, frame);
        if (ret == 0) return 0;

        /* All TX buffers are busy. During bring-up this commonly happens when
         * the board tried to transmit before the USB2CAN side was up. Abort stale
         * retries once and load the current response again.
         */
        MCP2515_Abort_All_Tx();
    }

    return -2;
}

void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & (1U << 4))
    {
        EXTI->PR = (1U << 4);
        g_mcp2515_irq = 1;
    }
}
