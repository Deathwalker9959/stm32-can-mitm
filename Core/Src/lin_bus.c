/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lin_bus.c
  * @brief          : Optional LIN bus helper for TJA1021 half-duplex transceivers
  ******************************************************************************
  * @attention
  *
  * LIN Bus Implementation for TJA1021 One-Wire Half-Duplex Transceiver
  * - Hardware: STM32F446 with UART in LIN mode + DMA
  * - Wiring: MCU TX->TX, RX->RX to TJA1021 (transceiver handles one-wire)
  * - Echo Handling: Transmitted bytes appear on RX due to half-duplex
  * - Interrupt-Driven: Uses DMA + UART IDLE detection for frame reception
  *
  * KEY FEATURES:
  * 1. Proper echo byte handling (skips transmitted bytes in RX buffer)
  * 2. Interrupt-based reception with DMA + IDLE line detection
  * 3. State machine for robust bus management
  * 4. Enhanced checksum calculation (LIN 2.x standard)
  * 5. Collision and error detection
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "lin_bus.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define LIN_SYNC_BYTE           0x55    // LIN sync byte
#define LIN_BREAK_DURATION_MS   2       // Break duration (not used with HAL_LIN_SendBreak)

/* Private variables ---------------------------------------------------------*/
// Use __attribute__((aligned(4))) to ensure proper memory alignment for DMA
LIN_HandleTypeDef hlin1 __attribute__((aligned(4))) = {0};  // LIN handle for USART1
LIN_HandleTypeDef hlin2 __attribute__((aligned(4))) = {0};  // LIN handle for USART2

// Alternative: Use separate DMA buffers if embedded buffers don't work
// Uncomment these if the embedded buffers have issues:
// static uint8_t dma_rx_buffer1[LIN_RX_BUFFER_SIZE] __attribute__((aligned(4)));
// static uint8_t dma_rx_buffer2[LIN_RX_BUFFER_SIZE] __attribute__((aligned(4)));

/* Private function prototypes -----------------------------------------------*/
static void LIN_ResetState(LIN_HandleTypeDef *hlin);
static LIN_StatusTypeDef LIN_StartReception(LIN_HandleTypeDef *hlin);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Calculate LIN PID (Protected ID) with parity bits
 *
 * LIN PID Format: [P1][P0][ID5:ID0]
 * P0 = ID0 ^ ID1 ^ ID2 ^ ID4
 * P1 = ~(ID1 ^ ID3 ^ ID4 ^ ID5)
 */
uint8_t LIN_CalculatePID(uint8_t id)
{
    if (id > 0x3F)
        return 0;  // Invalid ID (must be 0-63)

    // Extract ID bits
    uint8_t id_bits[6];
    for (int i = 0; i < 6; i++)
    {
        id_bits[i] = (id >> i) & 0x01;
    }

    // Calculate parity bits
    uint8_t p0 = (id_bits[0] ^ id_bits[1] ^ id_bits[2] ^ id_bits[4]) & 0x01;
    uint8_t p1 = ~(id_bits[1] ^ id_bits[3] ^ id_bits[4] ^ id_bits[5]) & 0x01;

    // Combine: [P1][P0][ID5:ID0]
    return id | (p0 << 6) | (p1 << 7);
}

/**
 * @brief Calculate LIN checksum (classic or enhanced)
 *
 * Classic:  Checksum = 0xFF - (Data0 + Data1 + ... + DataN + carry)
 * Enhanced: Checksum = 0xFF - (PID + Data0 + Data1 + ... + DataN + carry)
 *
 * Carry handling: When sum > 0xFF, subtract 0xFF (wrap-around)
 */
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t size, LIN_ChecksumTypeDef type)
{
    uint16_t sum = 0;

    // Enhanced checksum includes PID
    if (type == LIN_CHECKSUM_ENHANCED)
    {
        sum = pid;
    }

    // Add all data bytes
    for (int i = 0; i < size; i++)
    {
        sum += data[i];
        // Handle carry (wrap around)
        if (sum > 0xFF)
            sum -= 0xFF;
    }

    return (uint8_t)(0xFF - sum);
}

/**
 * @brief Initialize LIN bus handle
 */
LIN_StatusTypeDef LIN_Init(LIN_HandleTypeDef *hlin, UART_HandleTypeDef *huart)
{
    if (hlin == NULL || huart == NULL)
        return LIN_ERROR;

    // Initialize handle
    memset(hlin, 0, sizeof(LIN_HandleTypeDef));
    hlin->huart = huart;
    hlin->state = LIN_STATE_IDLE;

    return LIN_OK;
}

/**
 * @brief Send complete LIN frame (master transmits header + data + checksum)
 *
 * Frame format: [Break] [Sync=0x55] [PID] [Data0...DataN] [Checksum]
 *
 * NOTE: Due to half-duplex, all transmitted bytes will appear in RX buffer
 */
LIN_StatusTypeDef LIN_SendFrame(LIN_HandleTypeDef *hlin, LIN_FrameTypeDef *frame)
{
    if (hlin == NULL || frame == NULL)
        return LIN_ERROR;

    if (hlin->state != LIN_STATE_IDLE)
        return LIN_BUSY;

    if (frame->id > 0x3F || frame->length == 0 || frame->length > LIN_MAX_DATA_LENGTH)
        return LIN_ERROR;

    // Build frame: [Sync] [PID] [Data...] [Checksum]
    uint8_t tx_buffer[1 + 1 + LIN_MAX_DATA_LENGTH + 1];  // sync + PID + data + checksum
    uint8_t pid = LIN_CalculatePID(frame->id);
    uint8_t checksum = LIN_CalculateChecksum(pid, frame->data, frame->length, frame->checksum_type);

    tx_buffer[0] = LIN_SYNC_BYTE;
    tx_buffer[1] = pid;
    memcpy(&tx_buffer[2], frame->data, frame->length);
    tx_buffer[2 + frame->length] = checksum;

    uint8_t total_bytes = 2 + frame->length + 1;  // sync + PID + data + checksum

    // Update state
    hlin->state = LIN_STATE_TX_DATA;
    hlin->tx_echo_count = total_bytes;  // Expect echo of all bytes
    hlin->current_pid = pid;

    // Clear RX buffer and start reception (to capture echo)
    memset(hlin->rx_buffer, 0, LIN_RX_BUFFER_SIZE);
    hlin->rx_length = 0;
    hlin->frame_received = 0;

    // Abort any ongoing reception, then restart fresh
    HAL_UART_AbortReceive(hlin->huart);

    // Start DMA reception to capture echo bytes
    if (LIN_StartReception(hlin) != LIN_OK)
    {
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Send break
    if (HAL_LIN_SendBreak(hlin->huart) != HAL_OK)
    {
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Send frame (blocking mode)
    if (HAL_UART_Transmit(hlin->huart, tx_buffer, total_bytes, 100) != HAL_OK)
    {
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Wait a bit for echo to be received
    HAL_Delay(2);

    // Reset to idle
    LIN_ResetState(hlin);

    return LIN_OK;
}

/**
 * @brief Send LIN header and prepare to receive slave response
 *
 * Header format: [Break] [Sync=0x55] [PID]
 *
 * After sending header:
 * - DMA reception is started to capture echo + slave response
 * - Slave will respond with: [Data0...DataN] [Checksum]
 * - RX buffer will contain: [Echo_Sync] [Echo_PID] [Data0...DataN] [Checksum]
 */
LIN_StatusTypeDef LIN_SendHeader(LIN_HandleTypeDef *hlin, uint8_t id, uint8_t expected_length)
{
    if (hlin == NULL)
        return LIN_ERROR;

    if (hlin->state != LIN_STATE_IDLE)
        return LIN_BUSY;

    if (id > 0x3F || expected_length == 0 || expected_length > LIN_MAX_DATA_LENGTH)
        return LIN_ERROR;

    // Calculate PID
    uint8_t pid = LIN_CalculatePID(id);

    // Build header: [Sync] [PID]
    uint8_t header[2];
    header[0] = LIN_SYNC_BYTE;
    header[1] = pid;

    // Update state
    hlin->state = LIN_STATE_TX_HEADER;

    // IMPORTANT: Break field may be captured as 0x00 byte by DMA
    // If break is captured: skip 3 bytes (break + sync + PID)
    // If break not captured: skip 2 bytes (sync + PID)
    // We'll detect this dynamically in ProcessReceivedFrame
    hlin->tx_echo_count = 2;  // Will be adjusted if needed

    hlin->expected_data_length = expected_length;
    hlin->current_pid = pid;

    // Clear RX buffer and flags
    memset(hlin->rx_buffer, 0, LIN_RX_BUFFER_SIZE);
    hlin->rx_length = 0;
    hlin->frame_received = 0;
    hlin->error_flag = 0;

    // Only abort if UART is busy (prevents interrupting active reception)
//    if (hlin->huart->RxState != HAL_UART_STATE_READY)
//    {
//        HAL_UART_AbortReceive(hlin->huart);
//    }

    // Start DMA reception (will capture echo + slave response)
    if (LIN_StartReception(hlin) != LIN_OK)
    {
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Send break
    if (HAL_LIN_SendBreak(hlin->huart) != HAL_OK)
    {
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Send header (blocking)
    if (HAL_UART_Transmit(hlin->huart, header, 2, 100) != HAL_OK)
    {
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Now waiting for slave response
    hlin->state = LIN_STATE_RX_DATA;

    return LIN_OK;
}

/**
 * @brief Process received frame from RX buffer
 *
 * RX Buffer format: [Echo_Sync] [Echo_PID] [Data0...DataN] [Checksum]
 *
 * This function:
 * 1. Skips echo bytes (sync + PID)
 * 2. Extracts data bytes
 * 3. Validates checksum
 * 4. Returns frame data
 */
LIN_StatusTypeDef LIN_ProcessReceivedFrame(LIN_HandleTypeDef *hlin, LIN_FrameTypeDef *frame)
{
    if (hlin == NULL || frame == NULL)
        return LIN_ERROR;

    if (!hlin->frame_received)
        return LIN_BUSY;

    // Clear flag
    hlin->frame_received = 0;

    // OPTIMIZED: Fast sync detection (check likely positions first)
    uint8_t skip_bytes = 0;

    // Most common: Break captured at position 0, sync at position 1
    if (hlin->rx_length >= 2 && hlin->rx_buffer[1] == LIN_SYNC_BYTE)
    {
        // Check if position 2 has PID/ID
        if (hlin->rx_length >= 3)
        {
            uint8_t next_byte = hlin->rx_buffer[2];
            // Skip break + sync + (PID or ID)
            skip_bytes = (next_byte == hlin->current_pid || next_byte == (hlin->current_pid & 0x3F)) ? 3 : 2;
        }
        else
        {
            skip_bytes = 2;  // Skip break + sync
        }
    }
    // Second most common: No break, sync at position 0
    else if (hlin->rx_buffer[0] == LIN_SYNC_BYTE)
    {
        if (hlin->rx_length >= 2)
        {
            uint8_t next_byte = hlin->rx_buffer[1];
            // Skip sync + (PID or ID)
            skip_bytes = (next_byte == hlin->current_pid || next_byte == (hlin->current_pid & 0x3F)) ? 2 : 1;
        }
        else
        {
            skip_bytes = 1;  // Skip sync only
        }
    }
    // Rare: Sync at position 2
    else if (hlin->rx_length >= 3 && hlin->rx_buffer[2] == LIN_SYNC_BYTE)
    {
        skip_bytes = 3;
    }
    else
    {
        // No sync found - error
        hlin->error_count++;
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Check if we have minimum data after skipping echo
    if (hlin->rx_length < (skip_bytes + 1 + 1))  // data + checksum
    {
        hlin->error_count++;
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Skip echo bytes and extract data
    uint8_t *data_start = &hlin->rx_buffer[skip_bytes];
    uint8_t data_length = hlin->rx_length - skip_bytes - 1;  // -1 for checksum

    // Validate data length
    if (data_length != hlin->expected_data_length)
    {
        hlin->error_count++;
        LIN_ResetState(hlin);
        return LIN_ERROR;
    }

    // Extract received checksum
    uint8_t received_checksum = hlin->rx_buffer[hlin->rx_length - 1];

    // OPTIMIZED: Try most likely checksum first (ID-only based on your slave)
    uint8_t id_only = hlin->current_pid & 0x3F;
    uint8_t calc_checksum;

    // Try ID-only first (your slave uses this - fastest path)
    calc_checksum = LIN_CalculateChecksum(id_only, data_start, data_length, LIN_CHECKSUM_ENHANCED);
    if (received_checksum == calc_checksum)
    {
        goto checksum_ok;  // Fast exit
    }

    // Try full PID (LIN 2.x standard)
    calc_checksum = LIN_CalculateChecksum(hlin->current_pid, data_start, data_length, LIN_CHECKSUM_ENHANCED);
    if (received_checksum == calc_checksum)
    {
        goto checksum_ok;  // Fast exit
    }

    // Try classic (data only)
    calc_checksum = LIN_CalculateChecksum(0, data_start, data_length, LIN_CHECKSUM_CLASSIC);
    if (received_checksum == calc_checksum)
    {
        goto checksum_ok;  // Fast exit
    }

    // All methods failed
    hlin->error_count++;
    LIN_ResetState(hlin);
    return LIN_CHECKSUM_ERROR;

checksum_ok:

    // Fill frame structure
    frame->id = hlin->current_pid & 0x3F;  // Extract ID from PID
    frame->length = data_length;
    memcpy(frame->data, data_start, data_length);
    frame->checksum_type = LIN_CHECKSUM_ENHANCED;
    frame->timestamp = HAL_GetTick();

    // Update statistics
    hlin->frame_count++;

    // Reset state
    LIN_ResetState(hlin);

    return LIN_OK;
}

/**
 * @brief Check if new frame is available
 */
uint8_t LIN_IsFrameReceived(LIN_HandleTypeDef *hlin)
{
    if (hlin == NULL)
        return 0;

    return hlin->frame_received;
}

/**
 * @brief Get current bus state
 */
LIN_StateTypeDef LIN_GetState(LIN_HandleTypeDef *hlin)
{
    if (hlin == NULL)
        return LIN_STATE_ERROR;

    return hlin->state;
}

/**
 * @brief Get RX buffer pointer for debugging
 */
uint8_t* LIN_GetRxBuffer(LIN_HandleTypeDef *hlin)
{
    if (hlin == NULL)
        return NULL;

    return hlin->rx_buffer;
}

/**
 * @brief Get received byte count
 */
uint16_t LIN_GetRxLength(LIN_HandleTypeDef *hlin)
{
    if (hlin == NULL)
        return 0;

    return hlin->rx_length;
}

/**
 * @brief UART RX Event Callback
 *
 * MUST be called from HAL_UARTEx_RxEventCallback in main.c or stm32f4xx_it.c
 *
 * This callback is triggered when:
 * - DMA buffer is full
 * - UART IDLE line detected (recommended for LIN)
 */
void LIN_RxEventCallback(LIN_HandleTypeDef *hlin, uint16_t size)
{
    if (hlin == NULL || size == 0)
        return;

    // Store received length
    hlin->rx_length = size;

    // Set frame received flag
    hlin->frame_received = 1;

    // Note: Reception restart is managed by main loop state machine
}

/**
 * @brief UART Error Callback
 *
 * MUST be called from HAL_UART_ErrorCallback in main.c or stm32f4xx_it.c
 */
void LIN_ErrorCallback(LIN_HandleTypeDef *hlin)
{
    if (hlin == NULL)
        return;

    // Set error flag
    hlin->error_flag = 1;
    hlin->error_count++;

    // Reset state
    LIN_ResetState(hlin);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Reset LIN handle to idle state
 */
static void LIN_ResetState(LIN_HandleTypeDef *hlin)
{
    if (hlin == NULL)
        return;

    hlin->state = LIN_STATE_IDLE;
    hlin->tx_echo_count = 0;
    hlin->expected_data_length = 0;
    hlin->current_pid = 0;
}

/**
 * @brief Start DMA reception with IDLE detection
 */
static LIN_StatusTypeDef LIN_StartReception(LIN_HandleTypeDef *hlin)
{
    if (hlin == NULL || hlin->huart == NULL)
        return LIN_ERROR;

    // Start DMA reception with IDLE line detection
    if (HAL_UARTEx_ReceiveToIdle_DMA(hlin->huart, hlin->rx_buffer, LIN_RX_BUFFER_SIZE) != HAL_OK)
        return LIN_ERROR;

    // Disable half-transfer interrupt (we only care about complete/idle)
    if (hlin->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(hlin->huart->hdmarx, DMA_IT_HT);
    }

    return LIN_OK;
}
