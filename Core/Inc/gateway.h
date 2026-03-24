#ifndef __GATEWAY_H
#define __GATEWAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#if APP_ENABLE_LIN_BUS
#include "lin_bus.h"
#endif

#if APP_ENABLE_MCP2515
#include "mcp2515.h"
#endif

#define GATEWAY_CAN_QUEUE_SIZE 256U
#define CAN1_FILTER_BANK_START 0U
#define CAN2_FILTER_BANK_START 14U
#define ECHO_IGNORE_WINDOW_MS 4U

/* Built-in transport IDs:
 * - CAN1: standard bxCAN controller 1 on the STM32
 * - CAN2: standard bxCAN controller 2 on the STM32
 * - AUX: optional external CAN controller such as MCP2515 */

/* Bus-off events increment the busoff_count counters, then notifications
 * are re-armed.  Hardware auto-recovery is handled by AutoBusOff=ENABLE. */

/* Milliseconds of CAN bus silence before auto-sleep is requested.
 * Set to 0 to disable the idle-sleep feature entirely. */
#define GATEWAY_BUS_IDLE_TIMEOUT_MS  0U
#define GATEWAY_RX_GAP_TIMEOUT_MS    300U

/* How often (ms) the MCP2515 EFLG register is polled for bus-off/errors. */
#define GATEWAY_MCP2515_CHECK_MS     100U

typedef uint8_t GatewayBus;

#define GATEWAY_BUS_CAN1  ((GatewayBus)1U)
#define GATEWAY_BUS_CAN2  ((GatewayBus)2U)
#define GATEWAY_BUS_AUX   ((GatewayBus)3U)

typedef struct
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
} GatewayCanFrame;

typedef struct
{
  uint32_t can1_last_activity;
  uint32_t can2_last_activity;
  volatile uint32_t can1_tx_complete_count;
  volatile uint32_t can2_tx_complete_count;
  volatile uint32_t can1_tx_submit_ok_count;
  volatile uint32_t can2_tx_submit_ok_count;
  volatile uint32_t can1_tx_mailbox_full_count;
  volatile uint32_t can2_tx_mailbox_full_count;
  volatile uint32_t can1_error_flags;
  volatile uint32_t can2_error_flags;
  volatile uint32_t can1_error_count;
  volatile uint32_t can2_error_count;
  volatile uint32_t can1_rx_dropped;
  volatile uint32_t can2_rx_dropped;
  volatile uint32_t can1_tx_dropped;
  volatile uint32_t can2_tx_dropped;
  volatile uint32_t can1_tx_queued_count;
  volatile uint32_t can2_tx_queued_count;
  volatile uint32_t can1_tx_queue_peak;
  volatile uint32_t can2_tx_queue_peak;
  volatile uint32_t can1_rx_irq_count;
  volatile uint32_t can2_rx_irq_count;
  volatile uint32_t can1_rx_fifo_frames;
  volatile uint32_t can2_rx_fifo_frames;
  volatile uint32_t can1_rx_gap_count;
  volatile uint32_t can2_rx_gap_count;
  uint32_t          can1_last_error_tick;
  uint32_t          can2_last_error_tick;
  uint32_t          can1_last_busoff_tick;
  uint32_t          can2_last_busoff_tick;
  uint32_t          can1_last_rx_tick;
  uint32_t          can2_last_rx_tick;
  uint32_t          can1_last_gap_tick;
  uint32_t          can2_last_gap_tick;
  uint32_t          can1_error_flags_at_gap;
  uint32_t          can2_error_flags_at_gap;

  /* Bus management */
  volatile uint8_t  can1_busoff_pending;  /* set in IRQ, cleared by manager */
  volatile uint8_t  can2_busoff_pending;
  uint32_t          can1_busoff_count;    /* total bus-off events on CAN1   */
  uint32_t          can2_busoff_count;    /* total bus-off events on CAN2   */
  uint8_t           can1_sleeping;        /* 1 while CAN1 is in sleep mode  */
  uint8_t           can2_sleeping;        /* 1 while CAN2 is in sleep mode  */

#if APP_ENABLE_LIN_BUS
  uint32_t uart1_last_tx_time;
  uint32_t uart2_last_tx_time;
  uint8_t uart1_tx_pending_bytes;
  uint8_t uart2_tx_pending_bytes;
#endif

#if APP_ENABLE_MCP2515
  volatile uint32_t mcp2515_rx_count;
  volatile uint32_t mcp2515_tx_count;
  uint32_t          mcp2515_busoff_count; /* total bus-off events on MCP2515 */
  uint32_t          mcp2515_eflg;         /* last EFLG register snapshot     */
  uint8_t           mcp2515_sleeping;     /* 1 while MCP2515 is in sleep mode */
#endif
} GatewayStateTypeDef;

extern GatewayStateTypeDef g_gateway;
extern char g_can1_error_text[160];
extern char g_can2_error_text[160];

/**
  * @brief Reset gateway state, queues, and counters.
  * @note Call once after peripheral init and before any Gateway_* processing.
  */
void Gateway_Init(void);

/**
  * @brief Top-level gateway task.
  * @note Call from the main loop.
  * @note Execution order is:
  *       1. Gateway_ProcessCanBus()
  *       2. Gateway_ProcessLinBus()
  *       3. Gateway_ProcessSpiCanBus()
  *       4. Gateway_CAN_Service()
  */
void Gateway_Process(void);

/**
  * @brief Internal CAN dispatch entry used by Gateway_Process().
  * @note This drains pending bxCAN RX FIFOs and passes frames to App_HandleFrame().
  */
void Gateway_ProcessCanBus(void);

/**
  * @brief Developer LIN processing hook.
  * @note Default implementation is empty.
  * @note Typical usage:
  *       if (Gateway_LIN_FramePending(1U)) { inspect raw buffer; Gateway_LIN_Consume(1U); }
  */
void Gateway_ProcessLinBus(void);

/**
  * @brief Internal MCP2515 dispatch entry used by Gateway_Process().
  * @note This drains pending MCP2515 RX frames and passes them to App_HandleFrame().
  */
void Gateway_ProcessSpiCanBus(void);

/**
  * @brief Configure bxCAN filters for both CAN controllers.
  * @param phcan1 Usually &hcan1.
  * @param phcan2 Usually &hcan2.
  * @note Current setup accepts standard frames to FIFO0 and extended frames to FIFO1.
  */
void Gateway_CAN_Init(CAN_HandleTypeDef *phcan1, CAN_HandleTypeDef *phcan2);

/**
  * @brief Start bxCAN controllers and enable notifications.
  * @note Enables TX, RX0, RX1, and error notifications for CAN1 and CAN2.
  */
void Gateway_CAN_Start(void);

/**
  * @brief Pop one pending frame captured from CAN1 IRQ context.
  * @param frame Output frame buffer.
  * @retval 1 if a frame was returned, 0 if queue is empty.
  * @note Use inside Gateway_ProcessCanBus() to consume frames from CAN1.
  */
uint8_t Gateway_CAN1_Read(GatewayCanFrame *frame);

/**
  * @brief Pop one pending frame captured from CAN2 IRQ context.
  * @param frame Output frame buffer.
  * @retval 1 if a frame was returned, 0 if queue is empty.
  */
uint8_t Gateway_CAN2_Read(GatewayCanFrame *frame);

/**
  * @brief Queue a frame for transmission on a bxCAN peripheral.
  * @param to Destination peripheral, usually &hcan1 or &hcan2.
  * @param frame Frame to queue.
  * @retval 1 on success, 0 if the queue is full or the target is invalid.
  * @note This is non-blocking. Actual transmit happens in Gateway_CAN_Service().
  */
uint8_t Gateway_CAN_Send(CAN_HandleTypeDef *to, const GatewayCanFrame *frame);

/**
  * @brief Alias for Gateway_CAN_Send().
  * @param to Destination peripheral.
  * @param frame Frame to queue.
  * @retval 1 on success, 0 on failure.
  * @note Intended for clearer forwarding code in developer logic.
  */
uint8_t Gateway_CAN_Forward(CAN_HandleTypeDef *to, const GatewayCanFrame *frame);

/**
  * @brief Route a frame to a logical bus owned by the transport layer.
  * @param to_bus Destination bus identifier.
  * @param frame Frame to send.
  * @retval 1 on successful submission, 0 on failure.
  */
uint8_t Gateway_SendToBus(GatewayBus to_bus, const GatewayCanFrame *frame);

/**
  * @brief Return the current HAL tick count for app-layer timing.
  * @retval Tick value from HAL_GetTick().
  */
uint32_t Gateway_GetTick(void);

/**
  * @brief Service queued CAN transmissions.
  * @note Usually you do not call this directly because Gateway_Process() already does.
  * @note Useful if your application wants tighter control over TX scheduling.
  */
void Gateway_CAN_Service(void);

/**
  * @brief Update last-activity timestamp for a CAN bus.
  * @param can_id Use 1 for CAN1 or 2 for CAN2.
  * @note IRQ handlers already call this automatically on RX/TX activity.
  */
void Gateway_UpdateCANActivity(uint8_t can_id);

/**
  * @brief Periodic bus health manager — call from Gateway_Process().
  * @note Handles bxCAN bus-off recovery (re-arms notifications after hardware
  *       AutoBusOff recovery), MCP2515 bus-off re-init, and optional idle sleep.
  */
void Gateway_CAN_BusManagement(void);

/**
  * @brief Put a bxCAN controller into sleep mode.
  * @param can_id Use 1 for CAN1 or 2 for CAN2.
  * @note With AutoWakeUp=ENABLE the hardware wakes automatically on bus activity.
  *       Call Gateway_CAN_Wake() explicitly if AutoWakeUp=DISABLE.
  */
void Gateway_CAN_Sleep(uint8_t can_id);

/**
  * @brief Wake a bxCAN controller from sleep.
  * @param can_id Use 1 for CAN1 or 2 for CAN2.
  * @note No-op if the bus is already active.
  */
void Gateway_CAN_Wake(uint8_t can_id);

/**
  * @brief Initialize optional LIN helpers.
  * @param phuart1 Usually &huart1.
  * @param phuart2 Usually &huart2.
  * @note Safe to call even when APP_ENABLE_LIN_BUS is 0U.
  */
//void Gateway_LIN_Init(UART_HandleTypeDef *phuart1, UART_HandleTypeDef *phuart2);

/**
  * @brief Check if a raw LIN frame is waiting.
  * @param source_uart Use 1 for UART1 LIN side, 2 for UART2 LIN side.
  * @retval 1 if pending, 0 otherwise.
  */
uint8_t Gateway_LIN_FramePending(uint8_t source_uart);

/**
  * @brief Get the raw LIN DMA receive buffer.
  * @param source_uart Use 1 or 2.
  * @retval Pointer to raw buffer, or NULL when LIN is disabled or source is invalid.
  * @note Intended for developer-owned parsing logic.
  */
uint8_t *Gateway_LIN_GetRxBuffer(uint8_t source_uart);

/**
  * @brief Get the number of bytes currently stored in the raw LIN buffer.
  * @param source_uart Use 1 or 2.
  * @retval Byte count.
  */
uint16_t Gateway_LIN_GetRxLength(uint8_t source_uart);

/**
  * @brief Mark a raw LIN frame as consumed and re-arm DMA reception.
  * @param source_uart Use 1 or 2.
  * @note Call this after your custom LIN processing is complete.
  */
void Gateway_LIN_Consume(uint8_t source_uart);

/**
  * @brief Forward one raw LIN frame to the opposite UART.
  * @param source_uart Use 1 or 2.
  * @retval 1 when forwarded, 0 on invalid input or transmit failure.
  * @note This helper preserves a bare pass-through flow with break regeneration.
  * @note If you need protocol-aware remapping, do it before calling this helper
  *       or implement your own forwarding in Gateway_ProcessLinBus().
  */
uint8_t Gateway_LIN_Forward(uint8_t source_uart);

/**
  * @brief Bridge HAL UART RX event callback into the gateway.
  * @param huart UART that raised the callback.
  * @param size Number of bytes received.
  * @note Call path is already wired through gateway.c.
  */
//void Gateway_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

/**
  * @brief Bridge HAL UART error callback into the gateway.
  * @param huart UART that raised the callback.
  * @note Call path is already wired through gateway.c.
  */
//void Gateway_UART_ErrorCallback(UART_HandleTypeDef *huart);

#if APP_ENABLE_MCP2515
/**
  * @brief Initialize the optional MCP2515 SPI CAN controller.
  * @param bitrate MCP2515 bus bitrate selection.
  * @retval 1 on success, 0 on failure.
  */
uint8_t Gateway_MCP2515_Init(mcp2515_bitrate_t bitrate);

/**
  * @brief Read one frame from the MCP2515.
  * @param frame Output frame buffer.
  * @retval 1 if a frame was returned, 0 otherwise.
  */
uint8_t Gateway_MCP2515_Read(mcp2515_frame_t *frame);

/**
  * @brief Send one frame through the MCP2515.
  * @param frame Input frame.
  * @retval 1 on success, 0 on failure.
  */
uint8_t Gateway_MCP2515_Send(const mcp2515_frame_t *frame);

/**
  * @brief Convert an MCP2515 frame to bxCAN format and queue it for transmit.
  * @param to Destination bxCAN peripheral.
  * @param frame Source MCP2515 frame.
  * @retval 1 on success, 0 on failure.
  */
uint8_t Gateway_MCP2515_ForwardToCAN(CAN_HandleTypeDef *to, const mcp2515_frame_t *frame);

/**
  * @brief Convert a bxCAN frame to MCP2515 format and send it.
  * @param frame Source bxCAN frame.
  * @retval 1 on success, 0 on failure.
  */
uint8_t Gateway_CAN_ForwardToMCP2515(const GatewayCanFrame *frame);

/**
  * @brief Put the MCP2515 into sleep mode to save power.
  * @note The chip can wake on CAN bus activity if WAKFIL is set in CNF3.
  *       Call Gateway_MCP2515_Wake() to fully re-initialise after waking.
  */
void Gateway_MCP2515_Sleep(void);

/**
  * @brief Wake and fully re-initialise the MCP2515.
  * @param bitrate Bitrate to configure after wake (use same value as init).
  * @retval 1 on success, 0 if mcp2515_init() fails.
  */
uint8_t Gateway_MCP2515_Wake(mcp2515_bitrate_t bitrate);
#endif

/**
  * @brief Initialize MCP2515 if the feature is enabled in main.h.
  * @retval 1 when MCP2515 is disabled or init succeeds, 0 on init failure.
  * @note Use this from generated startup code to avoid MCP2515-specific
  *       preprocessor directives in CubeMX-owned files.
  */
uint8_t Gateway_InitOptionalMCP2515(void);

#ifdef __cplusplus
}
#endif

#endif /* __GATEWAY_H */
