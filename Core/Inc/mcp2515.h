#ifndef MCP2515_H
#define MCP2515_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Override these macros in the project settings or before including this header
 * if the MCP2515 chip-select is wired to a different GPIO.
 */
#ifndef MCP2515_CS_GPIO_Port
#define MCP2515_CS_GPIO_Port GPIOA
#endif

#ifndef MCP2515_CS_Pin
#define MCP2515_CS_Pin GPIO_PIN_4
#endif

#ifndef MCP2515_SPI_TIMEOUT_MS
#define MCP2515_SPI_TIMEOUT_MS 10U
#endif

#ifndef MCP2515_DMA_TIMEOUT_MS
#define MCP2515_DMA_TIMEOUT_MS 10U
#endif

#ifndef MCP2515_DMA_MIN_BYTES
#define MCP2515_DMA_MIN_BYTES 4U
#endif

#ifndef MCP2515_SPI_HANDLE
#define MCP2515_SPI_HANDLE hspi1
#endif

typedef enum
{
    CAN_125KBPS = 0,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
} mcp2515_bitrate_t;

typedef enum
{
    MCP2515_MODE_NORMAL = 0x00U,
    MCP2515_MODE_SLEEP = 0x20U,
    MCP2515_MODE_LOOPBACK = 0x40U,
    MCP2515_MODE_LISTENONLY = 0x60U,
    MCP2515_MODE_CONFIG = 0x80U
} mcp2515_mode_t;

typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t extended;
    uint8_t rtr;
} mcp2515_frame_t;

typedef enum
{
    MCP2515_OK = 0,
    MCP2515_ERROR,
    MCP2515_TIMEOUT,
    MCP2515_BUSY
} mcp2515_result_t;

void mcp2515_select(void);
void mcp2515_deselect(void);

mcp2515_result_t mcp2515_spi_txrx(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);
mcp2515_result_t mcp2515_spi_txrx_dma(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

bool mcp2515_reset(void);
uint8_t mcp2515_read_register(uint8_t address);
bool mcp2515_write_register(uint8_t address, uint8_t value);
bool mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data);
uint8_t mcp2515_read_status(void);
uint8_t mcp2515_rx_status(void);
bool mcp2515_set_mode(mcp2515_mode_t mode);
bool mcp2515_set_bitrate(mcp2515_bitrate_t bitrate);
bool mcp2515_init(mcp2515_bitrate_t bitrate);
bool mcp2515_send_message(const mcp2515_frame_t *frame);
bool mcp2515_receive_message(mcp2515_frame_t *frame);

bool mcp2515_set_mask(uint8_t mask_index, uint32_t id, bool extended);
bool mcp2515_set_filter(uint8_t filter_index, uint32_t id, bool extended);
uint8_t mcp2515_get_interrupt_flags(void);
bool mcp2515_clear_interrupt_flags(uint8_t flags);
bool mcp2515_irq_pending(void);
bool mcp2515_irq_receive(mcp2515_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* MCP2515_H */
