/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lin_bus.h
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
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LIN_BUS_H
#define __LIN_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define LIN_MAX_DATA_LENGTH     8       // Maximum data bytes in LIN frame
#define LIN_RX_BUFFER_SIZE      32      // RX buffer size (enough for echo + response)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LIN status enumeration
 */
typedef enum {
    LIN_OK = 0,
    LIN_ERROR,
    LIN_BUSY,
    LIN_TIMEOUT,
    LIN_CHECKSUM_ERROR,
    LIN_COLLISION_ERROR
} LIN_StatusTypeDef;

/**
 * @brief LIN checksum type
 */
typedef enum {
    LIN_CHECKSUM_CLASSIC = 0,   // Checksum over data only
    LIN_CHECKSUM_ENHANCED = 1   // Checksum over PID + data (LIN 2.x standard)
} LIN_ChecksumTypeDef;

/**
 * @brief Per-bus LIN validation policy.
 *
 * Typical use:
 *   LIN_ValidationConfigTypeDef cfg = {
 *       .validate_pid_parity = 1U,
 *       .validate_checksum = 1U,
 *       .checksum_type = LIN_CHECKSUM_ENHANCED,
 *   };
 *   LIN_Init(&hlin1, &huart1);
 *   LIN_SetValidationConfig(&hlin1, &cfg);
 */
typedef struct {
    uint8_t validate_pid_parity;        // 1 = validate PID parity when present
    uint8_t validate_checksum;          // 1 = validate checksum on received frames
    LIN_ChecksumTypeDef checksum_type;  // expected checksum type for received frames
} LIN_ValidationConfigTypeDef;

/**
 * @brief LIN bus state machine
 */
typedef enum {
    LIN_STATE_IDLE = 0,         // Bus idle, ready for transmission
    LIN_STATE_BREAK,            // Sending break
    LIN_STATE_TX_HEADER,        // Transmitting sync + PID
    LIN_STATE_TX_DATA,          // Transmitting data bytes
    LIN_STATE_RX_DATA,          // Receiving data from slave
    LIN_STATE_ERROR             // Error state
} LIN_StateTypeDef;

/**
 * @brief LIN frame structure
 */
typedef struct {
    uint8_t id;                         // Frame ID (0-63)
    uint8_t data[LIN_MAX_DATA_LENGTH];  // Data bytes
    uint8_t length;                     // Data length (1-8)
    LIN_ChecksumTypeDef checksum_type;  // Checksum type
    uint32_t timestamp;                 // Reception timestamp (HAL_GetTick)
} LIN_FrameTypeDef;

/**
 * @brief LIN bus handle structure
 */
typedef struct {
    UART_HandleTypeDef *huart;          // UART handle
    LIN_StateTypeDef state;             // Current state
    LIN_ValidationConfigTypeDef validation;
    uint8_t rx_buffer[LIN_RX_BUFFER_SIZE];  // RX DMA buffer
    uint16_t rx_length;                 // Received bytes count
    uint8_t tx_echo_count;              // Number of transmitted bytes to skip
    uint8_t expected_data_length;       // Expected data bytes from slave
    uint8_t current_pid;                // Current Protected ID
    volatile uint8_t frame_received;    // Flag: new frame available
    volatile uint8_t error_flag;        // Flag: error occurred
    uint32_t error_count;               // Total error count
    uint32_t frame_count;               // Total frames received
    uint32_t timeout_count;             // Total timeouts
} LIN_HandleTypeDef;

/* Exported variables --------------------------------------------------------*/
extern LIN_HandleTypeDef hlin1;  // LIN handle for USART1
extern LIN_HandleTypeDef hlin2;  // LIN handle for USART2

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Calculate LIN PID (Protected ID) with parity bits
 * @param id: Frame ID (0-63)
 * @retval PID with parity bits [P1][P0][ID5:ID0]
 */
uint8_t LIN_CalculatePID(uint8_t id);

/**
 * @brief Calculate LIN checksum (enhanced by default)
 * @param pid: Protected ID
 * @param data: Pointer to data bytes
 * @param size: Number of data bytes
 * @param type: Checksum type (classic or enhanced)
 * @retval Checksum byte
 */
uint8_t LIN_CalculateChecksum(uint8_t pid, uint8_t *data, uint8_t size, LIN_ChecksumTypeDef type);

/**
 * @brief Initialize LIN bus handle
 * @param hlin: Pointer to LIN handle
 * @param huart: Pointer to UART handle (must be initialized with HAL_LIN_Init)
 * @retval LIN_StatusTypeDef
 */
LIN_StatusTypeDef LIN_Init(LIN_HandleTypeDef *hlin, UART_HandleTypeDef *huart);

/**
 * @brief Override the validation policy for one LIN bus handle.
 * @param hlin Pointer to LIN handle.
 * @param config Validation policy to copy into the handle.
 */
void LIN_SetValidationConfig(LIN_HandleTypeDef *hlin, const LIN_ValidationConfigTypeDef *config);

/**
 * @brief Read back the active validation policy for one LIN bus handle.
 * @param hlin Pointer to LIN handle.
 * @param config Output policy snapshot.
 */
void LIN_GetValidationConfig(const LIN_HandleTypeDef *hlin, LIN_ValidationConfigTypeDef *config);

/**
 * @brief Send complete LIN frame (master transmits header + data)
 * @param hlin: Pointer to LIN handle
 * @param frame: Pointer to frame structure
 * @retval LIN_StatusTypeDef
 *
 * NOTE: This function will receive echo bytes on RX due to half-duplex
 */
LIN_StatusTypeDef LIN_SendFrame(LIN_HandleTypeDef *hlin, LIN_FrameTypeDef *frame);

/**
 * @brief Send LIN header and prepare to receive slave response
 * @param hlin: Pointer to LIN handle
 * @param id: Frame ID (0-63)
 * @param expected_length: Expected data bytes from slave (1-8)
 * @retval LIN_StatusTypeDef
 *
 * USAGE:
 *   LIN_SendHeader(&hlin1, 0x06, 2);  // Query slave ID 0x06 for 2 bytes
 *   HAL_Delay(20);                     // Wait for slave response
 *   if (hlin1.frame_received) {
 *       LIN_ProcessReceivedFrame(&hlin1, &frame);
 *   }
 */
LIN_StatusTypeDef LIN_SendHeader(LIN_HandleTypeDef *hlin, uint8_t id, uint8_t expected_length);

/**
 * @brief Process received frame from RX buffer (call from interrupt or main loop)
 * @param hlin: Pointer to LIN handle
 * @param frame: Pointer to frame structure to store result
 * @retval LIN_StatusTypeDef
 *
 * NOTE: This function skips echo bytes automatically
 */
LIN_StatusTypeDef LIN_ProcessReceivedFrame(LIN_HandleTypeDef *hlin, LIN_FrameTypeDef *frame);

/**
 * @brief Check if new frame is available
 * @param hlin: Pointer to LIN handle
 * @retval 1 if frame received, 0 otherwise
 */
uint8_t LIN_IsFrameReceived(LIN_HandleTypeDef *hlin);

/**
 * @brief Get current bus state
 * @param hlin: Pointer to LIN handle
 * @retval LIN_StateTypeDef
 */
LIN_StateTypeDef LIN_GetState(LIN_HandleTypeDef *hlin);

/**
 * @brief Get RX buffer pointer for debugging
 * @param hlin: Pointer to LIN handle
 * @retval Pointer to RX buffer
 */
uint8_t* LIN_GetRxBuffer(LIN_HandleTypeDef *hlin);

/**
 * @brief Get received byte count
 * @param hlin: Pointer to LIN handle
 * @retval Number of bytes received
 */
uint16_t LIN_GetRxLength(LIN_HandleTypeDef *hlin);

/**
 * @brief UART RX Event Callback - MUST be called from HAL_UARTEx_RxEventCallback
 * @param hlin: Pointer to LIN handle
 * @param size: Number of bytes received
 */
void LIN_RxEventCallback(LIN_HandleTypeDef *hlin, uint16_t size);

/**
 * @brief UART Error Callback - MUST be called from HAL_UART_ErrorCallback
 * @param hlin: Pointer to LIN handle
 */
void LIN_ErrorCallback(LIN_HandleTypeDef *hlin);

#ifdef __cplusplus
}
#endif

#endif /* __LIN_BUS_H */
