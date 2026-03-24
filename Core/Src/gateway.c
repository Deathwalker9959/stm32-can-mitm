#include "gateway.h"
#include "app_logic.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;

/* CAN notification flags used by both Gateway_CAN_Start and bus-management recovery. */
static const uint32_t k_can_notifications =
    CAN_IT_TX_MAILBOX_EMPTY   |
    CAN_IT_RX_FIFO0_MSG_PENDING |
    CAN_IT_ERROR_WARNING      |
    CAN_IT_ERROR_PASSIVE      |
    CAN_IT_BUSOFF             |
    CAN_IT_LAST_ERROR_CODE    |
    CAN_IT_ERROR;

#if APP_ENABLE_MCP2515
/* MCP2515 EFLG register and relevant flag bits (not exposed in mcp2515.h). */
#define GATEWAY_MCP2515_REG_EFLG  0x2DU
#define GATEWAY_MCP2515_EFLG_TXBO 0x20U  /* TX bus-off  (error count > 255) */
#define GATEWAY_MCP2515_EFLG_TXEP 0x10U  /* TX error-passive (count >= 128) */
#define GATEWAY_MCP2515_EFLG_RXEP 0x08U  /* RX error-passive (count >= 128) */

static mcp2515_bitrate_t mcp2515_current_bitrate = CAN_250KBPS;
static uint32_t          mcp2515_last_check_tick  = 0U;
#endif

GatewayStateTypeDef g_gateway = {0};

static uint32_t g_can1_tx_last_free_tick = 0U;
static uint32_t g_can2_tx_last_free_tick = 0U;

typedef struct
{
  GatewayCanFrame frames[GATEWAY_CAN_QUEUE_SIZE];
  uint16_t head;
  uint16_t tail;
  uint16_t count;
} GatewayCanTxQueue;

static GatewayCanTxQueue g_can1_tx_queue = {0};
static GatewayCanTxQueue g_can2_tx_queue = {0};

static void Gateway_CAN_ConfigFilters(void);
static uint8_t Gateway_CAN_SendNow(CAN_HandleTypeDef *hcan, const GatewayCanFrame *frame);
static uint8_t Gateway_CAN_EnqueueTx(GatewayCanTxQueue *queue, const GatewayCanFrame *frame, volatile uint32_t *peak_counter);
static uint8_t Gateway_CAN_PeekTx(const GatewayCanTxQueue *queue, GatewayCanFrame *frame);
static void Gateway_CAN_PopTx(GatewayCanTxQueue *queue);
static void Gateway_CAN_ServiceBus(CAN_HandleTypeDef *hcan, GatewayCanTxQueue *queue);
static void Gateway_DispatchFrame(GatewayBus source_bus, const GatewayCanFrame *frame);
#if APP_ENABLE_LIN_BUS
static void Gateway_LIN_Rearm(UART_HandleTypeDef *huart, uint8_t *buffer, volatile uint8_t *frame_received);
#endif

#if APP_ENABLE_LIN_BUS
static LIN_HandleTypeDef *Gateway_LIN_HandleFromIndex(uint8_t source_uart);
static uint8_t Gateway_LIN_IsEcho(uint8_t source_uart, uint16_t length);
#endif

void Gateway_Init(void)
{
  uint32_t now;

  memset(&g_gateway, 0, sizeof(g_gateway));
  memset(&g_can1_tx_queue, 0, sizeof(g_can1_tx_queue));
  memset(&g_can2_tx_queue, 0, sizeof(g_can2_tx_queue));
  now = HAL_GetTick();
  g_gateway.can1_last_activity = now;
  g_gateway.can2_last_activity = now;
  g_gateway.can1_last_rx_tick = now;
  g_gateway.can2_last_rx_tick = now;
  g_can1_tx_last_free_tick = now;
  g_can2_tx_last_free_tick = now;
  App_Logic_Init();
}

void Gateway_Process(void)
{
  /* App-owned frame policy runs on top of the transport-owned dispatchers. */
  Gateway_ProcessCanBus();
//  Gateway_ProcessLinBus();
  Gateway_ProcessSpiCanBus();
  App_Logic_Process();
  Gateway_CAN_Service();

  /* Bus health: recover from bus-off, check MCP2515 errors, manage sleep. */
  Gateway_CAN_BusManagement();
}

void Gateway_ProcessCanBus(void)
{
  GatewayCanFrame frame;

  while (Gateway_CAN1_Read(&frame) != 0U)
  {
    Gateway_DispatchFrame(GATEWAY_BUS_CAN1, &frame);
  }

  while (Gateway_CAN2_Read(&frame) != 0U)
  {
    Gateway_DispatchFrame(GATEWAY_BUS_CAN2, &frame);
  }
}

__weak void Gateway_ProcessLinBus(void)
{
  App_ProcessLinBus();
}

void Gateway_ProcessSpiCanBus(void)
{
#if APP_ENABLE_MCP2515
  GatewayCanFrame can_frame;
  mcp2515_frame_t mcp_frame;

  while (Gateway_MCP2515_Read(&mcp_frame) != 0U)
  {
    memset(&can_frame, 0, sizeof(can_frame));
    can_frame.header.IDE = (mcp_frame.extended != 0U) ? CAN_ID_EXT : CAN_ID_STD;
    can_frame.header.RTR = (mcp_frame.rtr != 0U) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    can_frame.header.DLC = (mcp_frame.dlc <= 8U) ? mcp_frame.dlc : 8U;

    if (mcp_frame.extended != 0U)
    {
      can_frame.header.ExtId = mcp_frame.id;
    }
    else
    {
      can_frame.header.StdId = mcp_frame.id & 0x7FFU;
    }

    memcpy(can_frame.data, mcp_frame.data, can_frame.header.DLC);
    Gateway_DispatchFrame(GATEWAY_BUS_AUX, &can_frame);
  }
#endif
}

void Gateway_CAN_Init(CAN_HandleTypeDef *phcan1, CAN_HandleTypeDef *phcan2)
{
  (void)phcan1;
  (void)phcan2;
  Gateway_CAN_ConfigFilters();
}

void Gateway_CAN_Start(void)
{
#if APP_ENABLE_CAN1
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
#endif

#if APP_ENABLE_CAN2
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
#endif

#if APP_ENABLE_CAN1
  if (HAL_CAN_ActivateNotification(&hcan1, k_can_notifications) != HAL_OK)
  {
    Error_Handler();
  }
#endif

#if APP_ENABLE_CAN2
  if (HAL_CAN_ActivateNotification(&hcan2, k_can_notifications) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

uint8_t Gateway_CAN1_Read(GatewayCanFrame *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

#if APP_ENABLE_CAN1
  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &frame->header, frame->data) == HAL_OK)
    {
      Gateway_UpdateCANActivity(1U);
      return 1U;
    }
  }

  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &frame->header, frame->data) == HAL_OK)
    {
      Gateway_UpdateCANActivity(1U);
      return 1U;
    }
  }
#endif

  return 0U;
}

uint8_t Gateway_CAN2_Read(GatewayCanFrame *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

#if APP_ENABLE_CAN2
  if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &frame->header, frame->data) == HAL_OK)
    {
      Gateway_UpdateCANActivity(2U);
      return 1U;
    }
  }

  if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &frame->header, frame->data) == HAL_OK)
    {
      Gateway_UpdateCANActivity(2U);
      return 1U;
    }
  }
#endif

  return 0U;
}

uint8_t Gateway_CAN_Send(CAN_HandleTypeDef *to, const GatewayCanFrame *frame)
{
  if ((to == NULL) || (frame == NULL))
  {
    return 0U;
  }

  if (to->Instance == CAN1)
  {
#if APP_ENABLE_CAN1
    if (Gateway_CAN_SendNow(&hcan1, frame) != 0U)
    {
      g_gateway.can1_tx_submit_ok_count++;
      return 1U;
    }

    g_gateway.can1_tx_mailbox_full_count++;
    if (Gateway_CAN_EnqueueTx(&g_can1_tx_queue, frame, &g_gateway.can1_tx_queue_peak) != 0U)
    {
      g_gateway.can1_tx_queued_count++;
      return 1U;
    }

    g_gateway.can1_tx_dropped++;
#endif
    return 0U;
  }

  if (to->Instance == CAN2)
  {
#if APP_ENABLE_CAN2
    if (Gateway_CAN_SendNow(&hcan2, frame) != 0U)
    {
      g_gateway.can2_tx_submit_ok_count++;
      return 1U;
    }

    g_gateway.can2_tx_mailbox_full_count++;
    if (Gateway_CAN_EnqueueTx(&g_can2_tx_queue, frame, &g_gateway.can2_tx_queue_peak) != 0U)
    {
      g_gateway.can2_tx_queued_count++;
      return 1U;
    }

    g_gateway.can2_tx_dropped++;
#endif
    return 0U;
  }

  return 0U;
}

uint8_t Gateway_CAN_Forward(CAN_HandleTypeDef *to, const GatewayCanFrame *frame)
{
  return Gateway_CAN_Send(to, frame);
}

uint8_t Gateway_SendToBus(GatewayBus to_bus, const GatewayCanFrame *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

  switch (to_bus)
  {
    case GATEWAY_BUS_CAN1:
#if APP_ENABLE_CAN1
      return Gateway_CAN_Send(&hcan1, frame);
#else
      return 0U;
#endif

    case GATEWAY_BUS_CAN2:
#if APP_ENABLE_CAN2
      return Gateway_CAN_Send(&hcan2, frame);
#else
      return 0U;
#endif

    case GATEWAY_BUS_AUX:
#if APP_ENABLE_MCP2515
      return Gateway_CAN_ForwardToMCP2515(frame);
#else
      return 0U;
#endif

    default:
      return 0U;
  }
}

uint32_t Gateway_GetTick(void)
{
  return HAL_GetTick();
}

static void Gateway_DispatchFrame(GatewayBus source_bus, const GatewayCanFrame *frame)
{
  AppFrameContext ctx;

  if (frame == NULL)
  {
    return;
  }

  ctx.source_bus = source_bus;
  ctx.now = HAL_GetTick();
  App_HandleFrame(&ctx, frame);
}

void Gateway_CAN_Service(void)
{
#if APP_ENABLE_CAN1
  Gateway_CAN_ServiceBus(&hcan1, &g_can1_tx_queue);
#endif
#if APP_ENABLE_CAN2
  Gateway_CAN_ServiceBus(&hcan2, &g_can2_tx_queue);
#endif
}

void Gateway_UpdateCANActivity(uint8_t can_id)
{
  uint32_t now;

  now = HAL_GetTick();

  if (can_id == 1U)
  {
    g_gateway.can1_last_activity = now;
    g_gateway.can1_last_rx_tick = now;
  }
  else if (can_id == 2U)
  {
    g_gateway.can2_last_activity = now;
    g_gateway.can2_last_rx_tick = now;
  }
}

void Gateway_CAN_BusManagement(void)
{
  uint32_t now = HAL_GetTick();

  /* ---- bxCAN bus-off recovery ------------------------------------------ */
  /* AutoBusOff=ENABLE means the hardware already waits for 128×11 recessive
   * bits before re-entering error-active.  We just need to re-arm
   * notifications once the controller has recovered. */
  if (g_gateway.can1_busoff_pending != 0U)
  {
    g_gateway.can1_busoff_pending = 0U;
    g_gateway.can1_busoff_count++;
    g_gateway.can1_last_busoff_tick = now;
    (void)HAL_CAN_ActivateNotification(&hcan1, k_can_notifications);
  }

  if (g_gateway.can2_busoff_pending != 0U)
  {
    g_gateway.can2_busoff_pending = 0U;
    g_gateway.can2_busoff_count++;
    g_gateway.can2_last_busoff_tick = now;
    (void)HAL_CAN_ActivateNotification(&hcan2, k_can_notifications);
  }

  /* ---- TX mailbox activity ---------------------------------------------- */
  /* No software TX queue is used. Keep lightweight timestamps for visibility,
   * but do not abort mailboxes from software because that creates burst/pause
   * behavior under load. */
#if APP_ENABLE_CAN1
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
  {
    g_can1_tx_last_free_tick = now;
  }
#endif

#if APP_ENABLE_CAN2
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0U)
  {
    g_can2_tx_last_free_tick = now;
  }
#endif

  /* ---- Idle auto-sleep -------------------------------------------------- */
  if ((now - g_gateway.can1_last_rx_tick) > GATEWAY_RX_GAP_TIMEOUT_MS)
  {
    g_gateway.can1_rx_gap_count++;
    g_gateway.can1_last_rx_tick = now;
  }

  if ((now - g_gateway.can2_last_rx_tick) > GATEWAY_RX_GAP_TIMEOUT_MS)
  {
    g_gateway.can2_rx_gap_count++;
    g_gateway.can2_last_rx_tick = now;
  }

#if GATEWAY_BUS_IDLE_TIMEOUT_MS > 0U
  if ((g_gateway.can1_sleeping == 0U) &&
      ((now - g_gateway.can1_last_activity) > GATEWAY_BUS_IDLE_TIMEOUT_MS))
  {
    Gateway_CAN_Sleep(1U);
  }

  if ((g_gateway.can2_sleeping == 0U) &&
      ((now - g_gateway.can2_last_activity) > GATEWAY_BUS_IDLE_TIMEOUT_MS))
  {
    Gateway_CAN_Sleep(2U);
  }
#else
  (void)now;
#endif

  /* ---- MCP2515 error / bus-off recovery --------------------------------- */
#if APP_ENABLE_MCP2515
  if ((g_gateway.mcp2515_sleeping == 0U) &&
      ((now - mcp2515_last_check_tick) >= GATEWAY_MCP2515_CHECK_MS))
  {
    uint8_t eflg;

    mcp2515_last_check_tick = now;
    eflg = mcp2515_read_register(GATEWAY_MCP2515_REG_EFLG);
    g_gateway.mcp2515_eflg = eflg;

    if ((eflg & GATEWAY_MCP2515_EFLG_TXBO) != 0U)
    {
      /* Bus-off: chip stopped transmitting.  Full re-init is the only
       * recovery path on the MCP2515 (no AutoBusOff equivalent). */
      g_gateway.mcp2515_busoff_count++;
      (void)mcp2515_init(mcp2515_current_bitrate);
    }
  }
#endif
}

void Gateway_CAN_Sleep(uint8_t can_id)
{
  if (can_id == 1U)
  {
#if APP_ENABLE_CAN1
    if (g_gateway.can1_sleeping == 0U)
    {
      (void)HAL_CAN_RequestSleep(&hcan1);
      g_gateway.can1_sleeping = 1U;
    }
#endif
  }
  else if (can_id == 2U)
  {
#if APP_ENABLE_CAN2
    if (g_gateway.can2_sleeping == 0U)
    {
      (void)HAL_CAN_RequestSleep(&hcan2);
      g_gateway.can2_sleeping = 1U;
    }
#endif
  }
}

void Gateway_CAN_Wake(uint8_t can_id)
{
  if (can_id == 1U)
  {
#if APP_ENABLE_CAN1
    if (g_gateway.can1_sleeping != 0U)
    {
      (void)HAL_CAN_WakeUp(&hcan1);
      g_gateway.can1_sleeping = 0U;
      g_gateway.can1_last_activity = HAL_GetTick();
    }
#endif
  }
  else if (can_id == 2U)
  {
#if APP_ENABLE_CAN2
    if (g_gateway.can2_sleeping != 0U)
    {
      (void)HAL_CAN_WakeUp(&hcan2);
      g_gateway.can2_sleeping = 0U;
      g_gateway.can2_last_activity = HAL_GetTick();
    }
#endif
  }
}

const char *Gateway_CAN_DecodeError(uint32_t error_flags)
{
  static char buffer[160];
  size_t len = 0U;

  buffer[0] = '\0';

  if (error_flags == HAL_CAN_ERROR_NONE)
  {
    return "NONE";
  }

#define APPEND_FLAG(flag, text) \
  do { \
    if ((error_flags & (flag)) != 0U) { \
      if (len != 0U) { \
        buffer[len++] = '|'; \
      } \
      (void)strcpy(&buffer[len], (text)); \
      len += strlen(text); \
    } \
  } while (0)

  APPEND_FLAG(HAL_CAN_ERROR_EWG, "EWG");
  APPEND_FLAG(HAL_CAN_ERROR_EPV, "EPV");
  APPEND_FLAG(HAL_CAN_ERROR_BOF, "BOF");
  APPEND_FLAG(HAL_CAN_ERROR_STF, "STF");
  APPEND_FLAG(HAL_CAN_ERROR_FOR, "FOR");
  APPEND_FLAG(HAL_CAN_ERROR_ACK, "ACK");
  APPEND_FLAG(HAL_CAN_ERROR_BR,  "BR");
  APPEND_FLAG(HAL_CAN_ERROR_BD,  "BD");
  APPEND_FLAG(HAL_CAN_ERROR_CRC, "CRC");
  APPEND_FLAG(HAL_CAN_ERROR_RX_FOV0, "RX_FOV0");
  APPEND_FLAG(HAL_CAN_ERROR_RX_FOV1, "RX_FOV1");
  APPEND_FLAG(HAL_CAN_ERROR_TX_ALST0, "TX_ALST0");
  APPEND_FLAG(HAL_CAN_ERROR_TX_TERR0, "TX_TERR0");
  APPEND_FLAG(HAL_CAN_ERROR_TX_ALST1, "TX_ALST1");
  APPEND_FLAG(HAL_CAN_ERROR_TX_TERR1, "TX_TERR1");
  APPEND_FLAG(HAL_CAN_ERROR_TX_ALST2, "TX_ALST2");
  APPEND_FLAG(HAL_CAN_ERROR_TX_TERR2, "TX_TERR2");
  APPEND_FLAG(HAL_CAN_ERROR_TIMEOUT, "TIMEOUT");
  APPEND_FLAG(HAL_CAN_ERROR_NOT_INITIALIZED, "NOT_INIT");
  APPEND_FLAG(HAL_CAN_ERROR_NOT_READY, "NOT_READY");
  APPEND_FLAG(HAL_CAN_ERROR_NOT_STARTED, "NOT_STARTED");
  APPEND_FLAG(HAL_CAN_ERROR_PARAM, "PARAM");

#undef APPEND_FLAG

  if (len == 0U)
  {
    (void)strcpy(buffer, "UNKNOWN");
  }

  return buffer;
}

//void Gateway_LIN_Init(UART_HandleTypeDef *phuart1, UART_HandleTypeDef *phuart2)
//{
//#if APP_ENABLE_LIN_BUS
//  if (LIN_Init(&hlin1, phuart1) != LIN_OK)
//  {
//    Error_Handler();
//  }
//
//  if (LIN_Init(&hlin2, phuart2) != LIN_OK)
//  {
//    Error_Handler();
//  }
//
//  Gateway_LIN_Rearm(phuart1, hlin1.rx_buffer, &hlin1.frame_received);
//  Gateway_LIN_Rearm(phuart2, hlin2.rx_buffer, &hlin2.frame_received);
//#else
//  (void)phuart1;
//  (void)phuart2;
//#endif
//}

uint8_t Gateway_LIN_FramePending(uint8_t source_uart)
{
#if APP_ENABLE_LIN_BUS
  LIN_HandleTypeDef *hlin;

  hlin = Gateway_LIN_HandleFromIndex(source_uart);
  return (hlin != NULL) ? hlin->frame_received : 0U;
#else
  (void)source_uart;
  return 0U;
#endif
}

uint8_t *Gateway_LIN_GetRxBuffer(uint8_t source_uart)
{
#if APP_ENABLE_LIN_BUS
  LIN_HandleTypeDef *hlin;

  hlin = Gateway_LIN_HandleFromIndex(source_uart);
  return (hlin != NULL) ? LIN_GetRxBuffer(hlin) : NULL;
#else
  (void)source_uart;
  return NULL;
#endif
}

uint16_t Gateway_LIN_GetRxLength(uint8_t source_uart)
{
#if APP_ENABLE_LIN_BUS
  LIN_HandleTypeDef *hlin;

  hlin = Gateway_LIN_HandleFromIndex(source_uart);
  return (hlin != NULL) ? LIN_GetRxLength(hlin) : 0U;
#else
  (void)source_uart;
  return 0U;
#endif
}

void Gateway_LIN_Consume(uint8_t source_uart)
{
#if APP_ENABLE_LIN_BUS
  LIN_HandleTypeDef *hlin;

  hlin = Gateway_LIN_HandleFromIndex(source_uart);
  if (hlin != NULL)
  {
    Gateway_LIN_Rearm(hlin->huart, hlin->rx_buffer, &hlin->frame_received);
  }
#else
  (void)source_uart;
#endif
}

uint8_t Gateway_LIN_Forward(uint8_t source_uart)
{
#if APP_ENABLE_LIN_BUS
  LIN_HandleTypeDef *src_hlin;
  LIN_HandleTypeDef *dst_hlin;
  UART_HandleTypeDef *dst_huart;
  uint8_t *rx_buffer;
  uint16_t rx_length;
  uint16_t frame_length;
  uint8_t sync_pos;

  src_hlin = Gateway_LIN_HandleFromIndex(source_uart);
  if (src_hlin == NULL)
  {
    return 0U;
  }

  if (source_uart == 1U)
  {
    dst_hlin = &hlin2;
    dst_huart = hlin2.huart;
  }
  else
  {
    dst_hlin = &hlin1;
    dst_huart = hlin1.huart;
  }

  rx_buffer = LIN_GetRxBuffer(src_hlin);
  rx_length = LIN_GetRxLength(src_hlin);

  if ((rx_buffer == NULL) || (rx_length == 0U))
  {
    Gateway_LIN_Consume(source_uart);
    return 0U;
  }

  if (Gateway_LIN_IsEcho(source_uart, rx_length) != 0U)
  {
    Gateway_LIN_Consume(source_uart);
    return 0U;
  }

  sync_pos = 0U;
  while ((sync_pos < rx_length) && (sync_pos < 3U))
  {
    if (rx_buffer[sync_pos] == 0x55U)
    {
      break;
    }

    sync_pos++;
  }

  if ((sync_pos >= rx_length) || (sync_pos >= 3U))
  {
    Gateway_LIN_Consume(source_uart);
    return 0U;
  }

  frame_length = (uint16_t)(rx_length - sync_pos);
  if (frame_length == 0U)
  {
    Gateway_LIN_Consume(source_uart);
    return 0U;
  }

  if (HAL_LIN_SendBreak(dst_huart) != HAL_OK)
  {
    Gateway_LIN_Consume(source_uart);
    return 0U;
  }

  if (HAL_UART_Transmit(dst_huart, &rx_buffer[sync_pos], frame_length, 50U) != HAL_OK)
  {
    Gateway_LIN_Consume(source_uart);
    return 0U;
  }

  if (source_uart == 1U)
  {
    g_gateway.uart2_last_tx_time = HAL_GetTick();
    g_gateway.uart2_tx_pending_bytes = (uint8_t)frame_length;
  }
  else
  {
    g_gateway.uart1_last_tx_time = HAL_GetTick();
    g_gateway.uart1_tx_pending_bytes = (uint8_t)frame_length;
  }

  Gateway_LIN_Rearm(dst_hlin->huart, dst_hlin->rx_buffer, &dst_hlin->frame_received);
  Gateway_LIN_Consume(source_uart);
  return 1U;
#else
  (void)source_uart;
  return 0U;
#endif
}

//void Gateway_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
//{
//#if APP_ENABLE_LIN_BUS
//  if (huart->Instance == USART1)
//  {
//    LIN_RxEventCallback(&hlin1, size);
//  }
//  else if (huart->Instance == USART2)
//  {
//    LIN_RxEventCallback(&hlin2, size);
//  }
//#else
//  (void)huart;
//  (void)size;
//#endif
//}
//
//void Gateway_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//#if APP_ENABLE_LIN_BUS
//  if (huart->Instance == USART1)
//  {
//    LIN_ErrorCallback(&hlin1);
//    Gateway_LIN_Rearm(hlin1.huart, hlin1.rx_buffer, &hlin1.frame_received);
//  }
//  else if (huart->Instance == USART2)
//  {
//    LIN_ErrorCallback(&hlin2);
//    Gateway_LIN_Rearm(hlin2.huart, hlin2.rx_buffer, &hlin2.frame_received);
//  }
//#else
//  (void)huart;
//#endif
//}

#if APP_ENABLE_MCP2515
uint8_t Gateway_MCP2515_Init(mcp2515_bitrate_t bitrate)
{
  mcp2515_current_bitrate = bitrate;
  return (mcp2515_init(bitrate) != false) ? 1U : 0U;
}

uint8_t Gateway_MCP2515_Read(mcp2515_frame_t *frame)
{
  if ((frame == NULL) || (mcp2515_receive_message(frame) == false))
  {
    return 0U;
  }

  g_gateway.mcp2515_rx_count++;
  return 1U;
}

uint8_t Gateway_MCP2515_Send(const mcp2515_frame_t *frame)
{
  if ((frame == NULL) || (mcp2515_send_message(frame) == false))
  {
    return 0U;
  }

  g_gateway.mcp2515_tx_count++;
  return 1U;
}

uint8_t Gateway_MCP2515_ForwardToCAN(CAN_HandleTypeDef *to, const mcp2515_frame_t *frame)
{
  GatewayCanFrame can_frame;

  if ((to == NULL) || (frame == NULL))
  {
    return 0U;
  }

  memset(&can_frame, 0, sizeof(can_frame));
  can_frame.header.IDE = (frame->extended != 0U) ? CAN_ID_EXT : CAN_ID_STD;
  can_frame.header.RTR = (frame->rtr != 0U) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  can_frame.header.DLC = (frame->dlc <= 8U) ? frame->dlc : 8U;

  if (frame->extended != 0U)
  {
    can_frame.header.ExtId = frame->id;
  }
  else
  {
    can_frame.header.StdId = frame->id & 0x7FFU;
  }

  memcpy(can_frame.data, frame->data, can_frame.header.DLC);
  return Gateway_CAN_Send(to, &can_frame);
}

uint8_t Gateway_CAN_ForwardToMCP2515(const GatewayCanFrame *frame)
{
  mcp2515_frame_t mcp_frame;

  if (frame == NULL)
  {
    return 0U;
  }

  memset(&mcp_frame, 0, sizeof(mcp_frame));
  mcp_frame.extended = (frame->header.IDE == CAN_ID_EXT) ? 1U : 0U;
  mcp_frame.rtr = (frame->header.RTR == CAN_RTR_REMOTE) ? 1U : 0U;
  mcp_frame.dlc = (frame->header.DLC <= 8U) ? frame->header.DLC : 8U;
  mcp_frame.id = (mcp_frame.extended != 0U) ? frame->header.ExtId : frame->header.StdId;
  memcpy(mcp_frame.data, frame->data, mcp_frame.dlc);

  return Gateway_MCP2515_Send(&mcp_frame);
}

void Gateway_MCP2515_Sleep(void)
{
  if (g_gateway.mcp2515_sleeping == 0U)
  {
    (void)mcp2515_set_mode(MCP2515_MODE_SLEEP);
    g_gateway.mcp2515_sleeping = 1U;
  }
}

uint8_t Gateway_MCP2515_Wake(mcp2515_bitrate_t bitrate)
{
  if (g_gateway.mcp2515_sleeping != 0U)
  {
    mcp2515_current_bitrate = bitrate;
    g_gateway.mcp2515_sleeping = 0U;
    g_gateway.mcp2515_eflg = 0U;
    mcp2515_last_check_tick = HAL_GetTick();
    return (mcp2515_init(bitrate) != false) ? 1U : 0U;
  }

  return 1U;
}
#endif

uint8_t Gateway_InitOptionalMCP2515(void)
{
#if APP_ENABLE_MCP2515
  return Gateway_MCP2515_Init(CAN_250KBPS);
#else
  return 1U;
#endif
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  GatewayCanFrame frame;

  if (hcan->Instance == CAN1)
  {
#if APP_ENABLE_CAN1
    g_gateway.can1_rx_irq_count++;
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
    {
      memset(&frame, 0, sizeof(frame));
      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &frame.header, frame.data) != HAL_OK)
      {
        break;
      }

      g_gateway.can1_rx_fifo_frames++;
      Gateway_UpdateCANActivity(1U);
      Gateway_DispatchFrame(GATEWAY_BUS_CAN1, &frame);
    }
#endif
  }
  else if (hcan->Instance == CAN2)
  {
#if APP_ENABLE_CAN2
    g_gateway.can2_rx_irq_count++;
    while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0U)
    {
      memset(&frame, 0, sizeof(frame));
      if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &frame.header, frame.data) != HAL_OK)
      {
        break;
      }

      g_gateway.can2_rx_fifo_frames++;
      Gateway_UpdateCANActivity(2U);
      Gateway_DispatchFrame(GATEWAY_BUS_CAN2, &frame);
    }
#endif
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    Gateway_UpdateCANActivity(1U);
  }
  else if (hcan->Instance == CAN2)
  {
    Gateway_UpdateCANActivity(2U);
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    g_gateway.can1_tx_complete_count++;
    Gateway_UpdateCANActivity(1U);
#if APP_ENABLE_CAN1
    Gateway_CAN_ServiceBus(&hcan1, &g_can1_tx_queue);
#endif
  }
  else if (hcan->Instance == CAN2)
  {
    g_gateway.can2_tx_complete_count++;
    Gateway_UpdateCANActivity(2U);
#if APP_ENABLE_CAN2
    Gateway_CAN_ServiceBus(&hcan2, &g_can2_tx_queue);
#endif
  }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_TxMailbox0CompleteCallback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_TxMailbox0CompleteCallback(hcan);
}

void HAL_CAN_TxMailbox0AbortedCallback(CAN_HandleTypeDef *hcan)
{
  /* Frame was aborted (ACK error, arbitration lost, or bus-off).
   * Increment tx_dropped so failures are visible in the debugger. */
  if (hcan->Instance == CAN1)
  {
    g_gateway.can1_tx_dropped++;
#if APP_ENABLE_CAN1
    Gateway_CAN_ServiceBus(&hcan1, &g_can1_tx_queue);
#endif
  }
  else if (hcan->Instance == CAN2)
  {
    g_gateway.can2_tx_dropped++;
#if APP_ENABLE_CAN2
    Gateway_CAN_ServiceBus(&hcan2, &g_can2_tx_queue);
#endif
  }
}

void HAL_CAN_TxMailbox1AbortedCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_TxMailbox0AbortedCallback(hcan);
}

void HAL_CAN_TxMailbox2AbortedCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_TxMailbox0AbortedCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t err;

  if (hcan->Instance == CAN1)
  {
    err = HAL_CAN_GetError(hcan);
    g_gateway.can1_error_flags = err;
    g_gateway.can1_error_count++;
    g_gateway.can1_last_error_tick = HAL_GetTick();

    if ((err & HAL_CAN_ERROR_BOF) != 0U)
    {
      /* Hardware auto-recovers (AutoBusOff=ENABLE).  Set flag so the bus
       * manager re-arms notifications once the controller is back on-line. */
      g_gateway.can1_busoff_pending = 1U;
    }
  }
  else if (hcan->Instance == CAN2)
  {
    err = HAL_CAN_GetError(hcan);
    g_gateway.can2_error_flags = err;
    g_gateway.can2_error_count++;
    g_gateway.can2_last_error_tick = HAL_GetTick();

    if ((err & HAL_CAN_ERROR_BOF) != 0U)
    {
      g_gateway.can2_busoff_pending = 1U;
    }
  }
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
//{
//  Gateway_UART_RxEventCallback(huart, size);
//}
//
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//  Gateway_UART_ErrorCallback(huart);
//}

static void Gateway_CAN_ConfigFilters(void)
{
  CAN_FilterTypeDef f;

  /* Pass-all filter: ID=0, Mask=0 → every bit is don't-care → accepts all
   * standard and extended frames on both buses, routed to FIFO0 only. */
  memset(&f, 0, sizeof(f));
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = 0x0000U;
  f.FilterIdLow          = 0x0000U;
  f.FilterMaskIdHigh     = 0x0000U;
  f.FilterMaskIdLow      = 0x0000U;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = CAN2_FILTER_BANK_START;

#if APP_ENABLE_CAN1
  f.FilterBank = CAN1_FILTER_BANK_START;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
  {
    Error_Handler();
  }
#endif

#if APP_ENABLE_CAN2
  f.FilterBank = CAN2_FILTER_BANK_START;
  if (HAL_CAN_ConfigFilter(&hcan2, &f) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

static uint8_t Gateway_CAN_SendNow(CAN_HandleTypeDef *hcan, const GatewayCanFrame *frame)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t mailbox;

  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
  {
    return 0U;
  }

  tx_header.StdId = frame->header.StdId;
  tx_header.ExtId = frame->header.ExtId;
  tx_header.IDE = frame->header.IDE;
  tx_header.RTR = frame->header.RTR;
  tx_header.DLC = (frame->header.DLC <= 8U) ? frame->header.DLC : 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)frame->data, &mailbox) != HAL_OK)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t Gateway_CAN_EnqueueTx(GatewayCanTxQueue *queue,
                                     const GatewayCanFrame *frame,
                                     volatile uint32_t *peak_counter)
{
  uint32_t primask;
  uint16_t next_tail;

  if ((queue == NULL) || (frame == NULL))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (queue->count >= GATEWAY_CAN_QUEUE_SIZE)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }

  queue->frames[queue->tail] = *frame;
  next_tail = (uint16_t)(queue->tail + 1U);
  if (next_tail >= GATEWAY_CAN_QUEUE_SIZE)
  {
    next_tail = 0U;
  }

  queue->tail = next_tail;
  queue->count++;
  if ((peak_counter != NULL) && (queue->count > *peak_counter))
  {
    *peak_counter = queue->count;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  return 1U;
}

static uint8_t Gateway_CAN_PeekTx(const GatewayCanTxQueue *queue, GatewayCanFrame *frame)
{
  if ((queue == NULL) || (frame == NULL) || (queue->count == 0U))
  {
    return 0U;
  }

  *frame = queue->frames[queue->head];
  return 1U;
}

static void Gateway_CAN_PopTx(GatewayCanTxQueue *queue)
{
  uint32_t primask;
  uint16_t next_head;

  if ((queue == NULL) || (queue->count == 0U))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (queue->count > 0U)
  {
    next_head = (uint16_t)(queue->head + 1U);
    if (next_head >= GATEWAY_CAN_QUEUE_SIZE)
    {
      next_head = 0U;
    }

    queue->head = next_head;
    queue->count--;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void Gateway_CAN_ServiceBus(CAN_HandleTypeDef *hcan, GatewayCanTxQueue *queue)
{
  GatewayCanFrame frame;

  if ((hcan == NULL) || (queue == NULL))
  {
    return;
  }

  while ((HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0U) &&
         (Gateway_CAN_PeekTx(queue, &frame) != 0U))
  {
    if (Gateway_CAN_SendNow(hcan, &frame) == 0U)
    {
      break;
    }

    if (hcan->Instance == CAN1)
    {
      g_gateway.can1_tx_submit_ok_count++;
    }
    else if (hcan->Instance == CAN2)
    {
      g_gateway.can2_tx_submit_ok_count++;
    }

    Gateway_CAN_PopTx(queue);
  }
}

#if APP_ENABLE_LIN_BUS
static void Gateway_LIN_Rearm(UART_HandleTypeDef *huart, uint8_t *buffer, volatile uint8_t *frame_received)
{
  if ((huart == NULL) || (buffer == NULL) || (frame_received == NULL))
  {
    return;
  }

  *frame_received = 0U;

  if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, LIN_RX_BUFFER_SIZE) == HAL_OK)
  {
    if (huart->hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
  }
}
#endif

#if APP_ENABLE_LIN_BUS
static LIN_HandleTypeDef *Gateway_LIN_HandleFromIndex(uint8_t source_uart)
{
  if (source_uart == 1U)
  {
    return &hlin1;
  }

  if (source_uart == 2U)
  {
    return &hlin2;
  }

  return NULL;
}

static uint8_t Gateway_LIN_IsEcho(uint8_t source_uart, uint16_t length)
{
  uint32_t now;
  uint32_t last_tx_time;
  uint8_t pending_bytes;

  now = HAL_GetTick();

  if (source_uart == 1U)
  {
    last_tx_time = g_gateway.uart1_last_tx_time;
    pending_bytes = g_gateway.uart1_tx_pending_bytes;
  }
  else
  {
    last_tx_time = g_gateway.uart2_last_tx_time;
    pending_bytes = g_gateway.uart2_tx_pending_bytes;
  }

  if (((now - last_tx_time) <= ECHO_IGNORE_WINDOW_MS) &&
      (pending_bytes > 0U) &&
      (length <= (uint16_t)(pending_bytes + 2U)))
  {
    if (source_uart == 1U)
    {
      g_gateway.uart1_tx_pending_bytes = 0U;
    }
    else
    {
      g_gateway.uart2_tx_pending_bytes = 0U;
    }

    return 1U;
  }

  return 0U;
}
#endif
