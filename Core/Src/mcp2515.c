#include "main.h"     /* provides MCP2515_CS_Pin = GPIO_PIN_6 override */
#include "mcp2515.h"

#include <string.h>

extern SPI_HandleTypeDef MCP2515_SPI_HANDLE;

#define MCP2515_CMD_RESET        0xC0U
#define MCP2515_CMD_READ         0x03U
#define MCP2515_CMD_READ_RXBUF_0 0x90U
#define MCP2515_CMD_READ_RXBUF_1 0x94U
#define MCP2515_CMD_WRITE        0x02U
#define MCP2515_CMD_LOAD_TXB0    0x40U
#define MCP2515_CMD_LOAD_TXB1    0x42U
#define MCP2515_CMD_LOAD_TXB2    0x44U
#define MCP2515_CMD_RTS_TXB0     0x81U
#define MCP2515_CMD_RTS_TXB1     0x82U
#define MCP2515_CMD_RTS_TXB2     0x84U
#define MCP2515_CMD_BIT_MODIFY   0x05U
#define MCP2515_CMD_READ_STATUS  0xA0U
#define MCP2515_CMD_RX_STATUS    0xB0U

#define MCP2515_REG_CANSTAT      0x0EU
#define MCP2515_REG_CANCTRL      0x0FU
#define MCP2515_REG_TEC          0x1CU
#define MCP2515_REG_REC          0x1DU
#define MCP2515_REG_RXF0SIDH     0x00U
#define MCP2515_REG_RXF1SIDH     0x04U
#define MCP2515_REG_RXF2SIDH     0x08U
#define MCP2515_REG_RXF3SIDH     0x10U
#define MCP2515_REG_RXF4SIDH     0x14U
#define MCP2515_REG_RXF5SIDH     0x18U
#define MCP2515_REG_BFPCTRL      0x0CU
#define MCP2515_REG_TXRTSCTRL    0x0DU
#define MCP2515_REG_RXM0SIDH     0x20U
#define MCP2515_REG_RXM1SIDH     0x24U
#define MCP2515_REG_CNF3         0x28U
#define MCP2515_REG_CNF2         0x29U
#define MCP2515_REG_CNF1         0x2AU
#define MCP2515_REG_CANINTE      0x2BU
#define MCP2515_REG_CANINTF      0x2CU
#define MCP2515_REG_EFLG         0x2DU
#define MCP2515_REG_TXB0CTRL     0x30U
#define MCP2515_REG_TXB1CTRL     0x40U
#define MCP2515_REG_TXB2CTRL     0x50U
#define MCP2515_REG_RXB0CTRL     0x60U
#define MCP2515_REG_RXB1CTRL     0x70U

#define MCP2515_CANCTRL_REQOP_MASK 0xE0U
#define MCP2515_CANSTAT_OPMOD_MASK 0xE0U
#define MCP2515_CANINTF_RX0IF      0x01U
#define MCP2515_CANINTF_RX1IF      0x02U
#define MCP2515_CANINTE_RX0IE      0x01U
#define MCP2515_CANINTE_RX1IE      0x02U
#define MCP2515_RXB_RX_ANY         0x60U
#define MCP2515_RXB0_BUKT          0x04U
#define MCP2515_TXB_TXREQ          0x08U
#define MCP2515_TXB_EXIDE          0x08U
#define MCP2515_DLC_MASK           0x0FU
#define MCP2515_DLC_RTR            0x40U
#define MCP2515_RXBSIDL_EXIDE      0x08U

#define MCP2515_STATUS_RX0IF       0x01U
#define MCP2515_STATUS_RX1IF       0x02U
#define MCP2515_STATUS_TX0REQ      0x04U
#define MCP2515_STATUS_TX1REQ      0x10U
#define MCP2515_STATUS_TX2REQ      0x40U

#define MCP2515_MAX_SPI_XFER       16U
#define MCP2515_RX_FRAME_BYTES     13U
#define MCP2515_MODE_TIMEOUT_MS    10U

typedef struct
{
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} mcp2515_bittiming_t;

/*
 * Bit-timing tables — select with MCP2515_OSC_FREQ_HZ in mcp2515.h.
 *
 * 8 MHz crystal  (cheap breakout boards, crystal marked "8.000"):
 *   BRP+1 = prescaler.  500 kbps needs BRP=0 → 16 TQ → 8M/(2×1×16)=250k? No:
 *   formula: baud = F_OSC / (2 × (BRP+1) × N_TQ)
 *   125k: BRP=3 → 8M/(2×4×8TQ) — use BRP=1,16TQ → 8M/(2×2×16)=125k ✓
 *   250k: BRP=0, 16TQ → 8M/(2×1×16)=250k ✓
 *   500k: BRP=0,  8TQ → 8M/(2×1× 8)=500k ✓  CNF2: BTLMODE=1,PHSEG1=2,PRSEG=1→0x91, CNF3: PHSEG2=1→0x01
 *   1M:   BRP=0,  4TQ → 8M/(2×1× 4)=1M  ✓  CNF2: BTLMODE=1,PHSEG1=1,PRSEG=0→0x89, CNF3: PHSEG2=0→0x00
 *
 * 16 MHz crystal (modules marked "16.000"):
 *   125k: BRP=3,16TQ → 16M/(2×4×16)=125k ✓
 *   250k: BRP=1,16TQ → 16M/(2×2×16)=250k ✓
 *   500k: BRP=0,16TQ → 16M/(2×1×16)=500k ✓
 *   1M:   BRP=0, 8TQ → 16M/(2×1× 8)=1M  ✓
 */
#if MCP2515_OSC_FREQ_HZ == 8000000UL
static const mcp2515_bittiming_t mcp2515_bittiming_table[] =
{
    {0x01U, 0xB1U, 0x85U}, /* 125 kbps, 16 TQ, BRP=1, sample point 75% */
    {0x00U, 0xB1U, 0x85U}, /* 250 kbps, 16 TQ, BRP=0, sample point 75% */
    {0x00U, 0x90U, 0x82U}, /* 500 kbps,  8 TQ, BRP=0, sample point 75% */
    {0x00U, 0x80U, 0x80U}  /* 1 Mbps,    4 TQ, BRP=0, sample point 75% */
};
#elif MCP2515_OSC_FREQ_HZ == 16000000UL
static const mcp2515_bittiming_t mcp2515_bittiming_table[] =
{
    {0x03U, 0xB3U, 0x03U}, /* 125 kbps, 16 TQ, BRP=3, sample point 75% */
    {0x01U, 0xB3U, 0x03U}, /* 250 kbps, 16 TQ, BRP=1, sample point 75% */
    {0x00U, 0xB3U, 0x03U}, /* 500 kbps, 16 TQ, BRP=0, sample point 75% */
    {0x00U, 0x98U, 0x01U}  /* 1 Mbps,    8 TQ, BRP=0, sample point 75% */
};
#else
#error "MCP2515_OSC_FREQ_HZ must be 8000000UL or 16000000UL"
#endif

static const uint8_t mcp2515_load_txb_cmd[3] =
{
    MCP2515_CMD_LOAD_TXB0,
    MCP2515_CMD_LOAD_TXB1,
    MCP2515_CMD_LOAD_TXB2
};

static const uint8_t mcp2515_rts_cmd[3] =
{
    MCP2515_CMD_RTS_TXB0,
    MCP2515_CMD_RTS_TXB1,
    MCP2515_CMD_RTS_TXB2
};

static const uint8_t mcp2515_read_rxb_cmd[2] =
{
    MCP2515_CMD_READ_RXBUF_0,
    MCP2515_CMD_READ_RXBUF_1
};

static const uint8_t mcp2515_filter_reg[6] =
{
    MCP2515_REG_RXF0SIDH,
    MCP2515_REG_RXF1SIDH,
    MCP2515_REG_RXF2SIDH,
    MCP2515_REG_RXF3SIDH,
    MCP2515_REG_RXF4SIDH,
    MCP2515_REG_RXF5SIDH
};

static const uint8_t mcp2515_mask_reg[2] =
{
    MCP2515_REG_RXM0SIDH,
    MCP2515_REG_RXM1SIDH
};

static uint8_t mcp2515_tx_dma_buffer[MCP2515_MAX_SPI_XFER] __attribute__((aligned(4)));
static uint8_t mcp2515_rx_dma_buffer[MCP2515_MAX_SPI_XFER] __attribute__((aligned(4)));

static bool mcp2515_wait_spi_ready(uint32_t timeout_ms);
static mcp2515_result_t mcp2515_spi_transfer_internal(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);
static bool mcp2515_read_registers(uint8_t address, uint8_t *data, uint8_t length);
static bool mcp2515_write_registers(uint8_t address, const uint8_t *data, uint8_t length);
static void mcp2515_encode_id(uint32_t id, bool extended, uint8_t *buffer);
static uint32_t mcp2515_decode_id(const uint8_t *buffer, bool *extended);
static bool mcp2515_load_tx_buffer(uint8_t tx_buffer_index, const mcp2515_frame_t *frame);
static bool mcp2515_request_to_send(uint8_t tx_buffer_index);
static bool mcp2515_read_rx_buffer(uint8_t rx_buffer_index, mcp2515_frame_t *frame);

void mcp2515_select(void)
{
    HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_RESET);
}

void mcp2515_deselect(void)
{
    HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);
}

mcp2515_result_t mcp2515_spi_txrx(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    if ((tx_data == NULL) || (rx_data == NULL) || (length == 0U))
    {
        return MCP2515_ERROR;
    }

    if (!mcp2515_wait_spi_ready(MCP2515_SPI_TIMEOUT_MS))
    {
        return MCP2515_BUSY;
    }

    mcp2515_select();
    hal_status = HAL_SPI_TransmitReceive(&MCP2515_SPI_HANDLE,
                                         (uint8_t *)tx_data,
                                         rx_data,
                                         length,
                                         MCP2515_SPI_TIMEOUT_MS);
    mcp2515_deselect();

    if (hal_status == HAL_OK)
    {
        return MCP2515_OK;
    }

    if (hal_status == HAL_TIMEOUT)
    {
        return MCP2515_TIMEOUT;
    }

    return MCP2515_ERROR;
}

mcp2515_result_t mcp2515_spi_txrx_dma(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
{
    HAL_StatusTypeDef hal_status;
    uint32_t start_tick;

    if ((tx_data == NULL) || (rx_data == NULL) || (length == 0U))
    {
        return MCP2515_ERROR;
    }

    if (!mcp2515_wait_spi_ready(MCP2515_DMA_TIMEOUT_MS))
    {
        return MCP2515_BUSY;
    }

    mcp2515_select();
    hal_status = HAL_SPI_TransmitReceive_DMA(&MCP2515_SPI_HANDLE,
                                             (uint8_t *)tx_data,
                                             rx_data,
                                             length);
    if (hal_status != HAL_OK)
    {
        mcp2515_deselect();
        return (hal_status == HAL_BUSY) ? MCP2515_BUSY : MCP2515_ERROR;
    }

    start_tick = HAL_GetTick();
    while (HAL_SPI_GetState(&MCP2515_SPI_HANDLE) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - start_tick) > MCP2515_DMA_TIMEOUT_MS)
        {
            (void)HAL_SPI_Abort(&MCP2515_SPI_HANDLE);
            mcp2515_deselect();
            return MCP2515_TIMEOUT;
        }
    }

    mcp2515_deselect();

    if (HAL_SPI_GetError(&MCP2515_SPI_HANDLE) != HAL_SPI_ERROR_NONE)
    {
        return MCP2515_ERROR;
    }

    return MCP2515_OK;
}

bool mcp2515_reset(void)
{
    uint8_t tx_byte[1] = {MCP2515_CMD_RESET};
    uint8_t rx_byte[1] = {0U};

    mcp2515_deselect(); /* ensure CS is deasserted before the transaction */

    if (mcp2515_spi_txrx(tx_byte, rx_byte, sizeof(tx_byte)) != MCP2515_OK)
    {
        return false;
    }

    HAL_Delay(10U); /* 10 ms — crystal oscillator startup + chip self-init */
    return true;
}

uint8_t mcp2515_read_register(uint8_t address)
{
    uint8_t tx_buffer[3] = {MCP2515_CMD_READ, address, 0x00U};
    uint8_t rx_buffer[3] = {0U};

    if (mcp2515_spi_txrx(tx_buffer, rx_buffer, sizeof(tx_buffer)) != MCP2515_OK)
    {
        return 0U;
    }

    return rx_buffer[2];
}

bool mcp2515_write_register(uint8_t address, uint8_t value)
{
    uint8_t tx_buffer[3] = {MCP2515_CMD_WRITE, address, value};
    uint8_t rx_buffer[3] = {0U};

    return (mcp2515_spi_txrx(tx_buffer, rx_buffer, sizeof(tx_buffer)) == MCP2515_OK);
}

bool mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{
    uint8_t tx_buffer[4] = {MCP2515_CMD_BIT_MODIFY, address, mask, data};
    uint8_t rx_buffer[4] = {0U};

    return (mcp2515_spi_txrx(tx_buffer, rx_buffer, sizeof(tx_buffer)) == MCP2515_OK);
}

uint8_t mcp2515_read_status(void)
{
    uint8_t tx_buffer[2] = {MCP2515_CMD_READ_STATUS, 0x00U};
    uint8_t rx_buffer[2] = {0U};

    if (mcp2515_spi_txrx(tx_buffer, rx_buffer, sizeof(tx_buffer)) != MCP2515_OK)
    {
        return 0U;
    }

    return rx_buffer[1];
}

uint8_t mcp2515_rx_status(void)
{
    uint8_t tx_buffer[2] = {MCP2515_CMD_RX_STATUS, 0x00U};
    uint8_t rx_buffer[2] = {0U};

    if (mcp2515_spi_txrx(tx_buffer, rx_buffer, sizeof(tx_buffer)) != MCP2515_OK)
    {
        return 0U;
    }

    return rx_buffer[1];
}

bool mcp2515_set_mode(mcp2515_mode_t mode)
{
    uint32_t start_tick;

    if (!mcp2515_bit_modify(MCP2515_REG_CANCTRL, MCP2515_CANCTRL_REQOP_MASK, (uint8_t)mode))
    {
        return false;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) <= MCP2515_MODE_TIMEOUT_MS)
    {
        if ((mcp2515_read_register(MCP2515_REG_CANSTAT) & MCP2515_CANSTAT_OPMOD_MASK) == (uint8_t)mode)
        {
            return true;
        }
    }

    return false;
}

bool mcp2515_set_bitrate(mcp2515_bitrate_t bitrate)
{
    const mcp2515_bittiming_t *timing;

    if ((uint32_t)bitrate >= (sizeof(mcp2515_bittiming_table) / sizeof(mcp2515_bittiming_table[0])))
    {
        return false;
    }

    if (!mcp2515_set_mode(MCP2515_MODE_CONFIG))
    {
        return false;
    }

    timing = &mcp2515_bittiming_table[bitrate];

    if (!mcp2515_write_register(MCP2515_REG_CNF1, timing->cnf1))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_CNF2, timing->cnf2))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_CNF3, timing->cnf3))
    {
        return false;
    }

    return true;
}

bool mcp2515_init(mcp2515_bitrate_t bitrate)
{
    if (!mcp2515_reset())
    {
        return false;
    }

    if (!mcp2515_set_mode(MCP2515_MODE_CONFIG))
    {
        return false;
    }

    if (!mcp2515_set_bitrate(bitrate))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_CANINTE, MCP2515_CANINTE_RX0IE | MCP2515_CANINTE_RX1IE))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_CANINTF, 0x00U))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_TXRTSCTRL, 0x00U))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_BFPCTRL, 0x00U))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_RXB0CTRL, MCP2515_RXB_RX_ANY | MCP2515_RXB0_BUKT))
    {
        return false;
    }

    if (!mcp2515_write_register(MCP2515_REG_RXB1CTRL, MCP2515_RXB_RX_ANY))
    {
        return false;
    }

    if (!mcp2515_set_mask(0U, 0U, false))
    {
        return false;
    }

    if (!mcp2515_set_mask(1U, 0U, false))
    {
        return false;
    }

    for (uint8_t i = 0U; i < 6U; ++i)
    {
        if (!mcp2515_set_filter(i, 0U, false))
        {
            return false;
        }
    }

    return mcp2515_set_mode(MCP2515_MODE_NORMAL);
}

bool mcp2515_send_message(const mcp2515_frame_t *frame)
{
    uint8_t status;
    uint8_t tx_buffer_index;

    if (frame == NULL)
    {
        return false;
    }

    if (frame->dlc > 8U)
    {
        return false;
    }

    status = mcp2515_read_status();

    if ((status & MCP2515_STATUS_TX0REQ) == 0U)
    {
        tx_buffer_index = 0U;
    }
    else if ((status & MCP2515_STATUS_TX1REQ) == 0U)
    {
        tx_buffer_index = 1U;
    }
    else if ((status & MCP2515_STATUS_TX2REQ) == 0U)
    {
        tx_buffer_index = 2U;
    }
    else
    {
        return false;
    }

    if (!mcp2515_load_tx_buffer(tx_buffer_index, frame))
    {
        return false;
    }

    return mcp2515_request_to_send(tx_buffer_index);
}

bool mcp2515_receive_message(mcp2515_frame_t *frame)
{
    uint8_t status;

    if (frame == NULL)
    {
        return false;
    }

    status = mcp2515_read_status();

    if ((status & MCP2515_STATUS_RX0IF) != 0U)
    {
        return mcp2515_read_rx_buffer(0U, frame);
    }

    if ((status & MCP2515_STATUS_RX1IF) != 0U)
    {
        return mcp2515_read_rx_buffer(1U, frame);
    }

    return false;
}

bool mcp2515_set_mask(uint8_t mask_index, uint32_t id, bool extended)
{
    uint8_t encoded_id[4];

    if (mask_index >= 2U)
    {
        return false;
    }

    mcp2515_encode_id(id, extended, encoded_id);
    return mcp2515_write_registers(mcp2515_mask_reg[mask_index], encoded_id, sizeof(encoded_id));
}

bool mcp2515_set_filter(uint8_t filter_index, uint32_t id, bool extended)
{
    uint8_t encoded_id[4];

    if (filter_index >= 6U)
    {
        return false;
    }

    mcp2515_encode_id(id, extended, encoded_id);
    return mcp2515_write_registers(mcp2515_filter_reg[filter_index], encoded_id, sizeof(encoded_id));
}

uint8_t mcp2515_get_interrupt_flags(void)
{
    return mcp2515_read_register(MCP2515_REG_CANINTF);
}

bool mcp2515_clear_interrupt_flags(uint8_t flags)
{
    return mcp2515_bit_modify(MCP2515_REG_CANINTF, flags, 0x00U);
}

bool mcp2515_irq_pending(void)
{
    return ((mcp2515_get_interrupt_flags() & (MCP2515_CANINTF_RX0IF | MCP2515_CANINTF_RX1IF)) != 0U);
}

bool mcp2515_irq_receive(mcp2515_frame_t *frame)
{
    return mcp2515_receive_message(frame);
}

static bool mcp2515_wait_spi_ready(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (HAL_SPI_GetState(&MCP2515_SPI_HANDLE) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            return false;
        }
    }

    return true;
}

static mcp2515_result_t mcp2515_spi_transfer_internal(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
{
    if (length >= MCP2515_DMA_MIN_BYTES)
    {
        return mcp2515_spi_txrx_dma(tx_data, rx_data, length);
    }

    return mcp2515_spi_txrx(tx_data, rx_data, length);
}

static __attribute__((unused)) bool mcp2515_read_registers(uint8_t address, uint8_t *data, uint8_t length)
{
    uint16_t transfer_length;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    transfer_length = (uint16_t)length + 2U;
    if (transfer_length > MCP2515_MAX_SPI_XFER)
    {
        return false;
    }

    memset(mcp2515_tx_dma_buffer, 0, transfer_length);
    memset(mcp2515_rx_dma_buffer, 0, transfer_length);

    mcp2515_tx_dma_buffer[0] = MCP2515_CMD_READ;
    mcp2515_tx_dma_buffer[1] = address;

    if (mcp2515_spi_transfer_internal(mcp2515_tx_dma_buffer, mcp2515_rx_dma_buffer, transfer_length) != MCP2515_OK)
    {
        return false;
    }

    memcpy(data, &mcp2515_rx_dma_buffer[2], length);
    return true;
}

static bool mcp2515_write_registers(uint8_t address, const uint8_t *data, uint8_t length)
{
    uint16_t transfer_length;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    transfer_length = (uint16_t)length + 2U;
    if (transfer_length > MCP2515_MAX_SPI_XFER)
    {
        return false;
    }

    memset(mcp2515_tx_dma_buffer, 0, transfer_length);
    memset(mcp2515_rx_dma_buffer, 0, transfer_length);

    mcp2515_tx_dma_buffer[0] = MCP2515_CMD_WRITE;
    mcp2515_tx_dma_buffer[1] = address;
    memcpy(&mcp2515_tx_dma_buffer[2], data, length);

    return (mcp2515_spi_transfer_internal(mcp2515_tx_dma_buffer, mcp2515_rx_dma_buffer, transfer_length) == MCP2515_OK);
}

static void mcp2515_encode_id(uint32_t id, bool extended, uint8_t *buffer)
{
    if (extended)
    {
        buffer[0] = (uint8_t)(id >> 21);
        buffer[1] = (uint8_t)(((id >> 13) & 0xE0U) | MCP2515_TXB_EXIDE | ((id >> 16) & 0x03U));
        buffer[2] = (uint8_t)(id >> 8);
        buffer[3] = (uint8_t)id;
    }
    else
    {
        buffer[0] = (uint8_t)(id >> 3);
        buffer[1] = (uint8_t)((id & 0x07U) << 5);
        buffer[2] = 0x00U;
        buffer[3] = 0x00U;
    }
}

static uint32_t mcp2515_decode_id(const uint8_t *buffer, bool *extended)
{
    uint32_t id;

    if ((buffer[1] & MCP2515_RXBSIDL_EXIDE) != 0U)
    {
        *extended = true;
        id = ((uint32_t)buffer[0] << 21)
           | ((uint32_t)(buffer[1] & 0xE0U) << 13)
           | ((uint32_t)(buffer[1] & 0x03U) << 16)
           | ((uint32_t)buffer[2] << 8)
           | buffer[3];
    }
    else
    {
        *extended = false;
        id = ((uint32_t)buffer[0] << 3) | ((buffer[1] >> 5) & 0x07U);
    }

    return id;
}

static bool mcp2515_load_tx_buffer(uint8_t tx_buffer_index, const mcp2515_frame_t *frame)
{
    uint8_t tx_length;

    if (tx_buffer_index >= 3U)
    {
        return false;
    }

    tx_length = (uint8_t)(6U + frame->dlc);
    if (tx_length > MCP2515_MAX_SPI_XFER)
    {
        return false;
    }

    memset(mcp2515_tx_dma_buffer, 0, tx_length);
    memset(mcp2515_rx_dma_buffer, 0, tx_length);

    mcp2515_tx_dma_buffer[0] = mcp2515_load_txb_cmd[tx_buffer_index];
    mcp2515_encode_id(frame->id, frame->extended != 0U, &mcp2515_tx_dma_buffer[1]);
    mcp2515_tx_dma_buffer[5] = (uint8_t)(frame->dlc & MCP2515_DLC_MASK);

    if (frame->rtr != 0U)
    {
        mcp2515_tx_dma_buffer[5] |= MCP2515_DLC_RTR;
    }
    else if (frame->dlc > 0U)
    {
        memcpy(&mcp2515_tx_dma_buffer[6], frame->data, frame->dlc);
    }

    return (mcp2515_spi_transfer_internal(mcp2515_tx_dma_buffer, mcp2515_rx_dma_buffer, tx_length) == MCP2515_OK);
}

static bool mcp2515_request_to_send(uint8_t tx_buffer_index)
{
    uint8_t tx_buffer[1];
    uint8_t rx_buffer[1] = {0U};

    if (tx_buffer_index >= 3U)
    {
        return false;
    }

    tx_buffer[0] = mcp2515_rts_cmd[tx_buffer_index];
    return (mcp2515_spi_txrx(tx_buffer, rx_buffer, sizeof(tx_buffer)) == MCP2515_OK);
}

static bool mcp2515_read_rx_buffer(uint8_t rx_buffer_index, mcp2515_frame_t *frame)
{
    uint8_t dlc_reg;
    bool extended = false;

    if (rx_buffer_index >= 2U)
    {
        return false;
    }

    memset(mcp2515_tx_dma_buffer, 0, 1U + MCP2515_RX_FRAME_BYTES);
    memset(mcp2515_rx_dma_buffer, 0, 1U + MCP2515_RX_FRAME_BYTES);

    mcp2515_tx_dma_buffer[0] = mcp2515_read_rxb_cmd[rx_buffer_index];
    if (mcp2515_spi_transfer_internal(mcp2515_tx_dma_buffer,
                                      mcp2515_rx_dma_buffer,
                                      1U + MCP2515_RX_FRAME_BYTES) != MCP2515_OK)
    {
        return false;
    }

    frame->id = mcp2515_decode_id(&mcp2515_rx_dma_buffer[1], &extended);
    frame->extended = extended ? 1U : 0U;

    dlc_reg = mcp2515_rx_dma_buffer[5];
    frame->dlc = (uint8_t)(dlc_reg & MCP2515_DLC_MASK);
    frame->rtr = ((dlc_reg & MCP2515_DLC_RTR) != 0U) ? 1U : 0U;

    if (frame->dlc > 8U)
    {
        frame->dlc = 8U;
    }

    memset(frame->data, 0, sizeof(frame->data));
    if ((frame->rtr == 0U) && (frame->dlc > 0U))
    {
        memcpy(frame->data, &mcp2515_rx_dma_buffer[6], frame->dlc);
    }

    if (rx_buffer_index == 0U)
    {
        return mcp2515_clear_interrupt_flags(MCP2515_CANINTF_RX0IF);
    }

    return mcp2515_clear_interrupt_flags(MCP2515_CANINTF_RX1IF);
}
