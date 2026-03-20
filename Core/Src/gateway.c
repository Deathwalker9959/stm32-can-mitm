#include "gateway.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;

typedef struct
{
  GatewayCanFrame buffer[GATEWAY_CAN_QUEUE_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
} GatewayCanQueue;

static GatewayCanQueue gateway_can1_rx_queue = {0};
static GatewayCanQueue gateway_can2_rx_queue = {0};
static GatewayCanQueue gateway_can1_tx_queue = {0};
static GatewayCanQueue gateway_can2_tx_queue = {0};
static volatile uint8_t gateway_can1_tx_service_request = 0U;
static volatile uint8_t gateway_can2_tx_service_request = 0U;

GatewayStateTypeDef g_gateway = {0};

static HAL_StatusTypeDef Gateway_CAN_ConfigFormatFilter(CAN_HandleTypeDef *hcan,
                                                        uint32_t filter_bank,
                                                        uint32_t fifo_assignment,
                                                        uint8_t extended_only);
static void Gateway_CAN_ConfigFilters(void);
static uint8_t Gateway_CAN_QueuePush(GatewayCanQueue *queue,
                                     volatile uint32_t *dropped_counter,
                                     const GatewayCanFrame *frame);
static uint8_t Gateway_CAN_QueuePeek(const GatewayCanQueue *queue, GatewayCanFrame *frame);
static void Gateway_CAN_QueuePop(GatewayCanQueue *queue);
static void Gateway_CAN_DrainFifo(CAN_HandleTypeDef *hcan,
                                  uint32_t fifo,
                                  GatewayCanQueue *queue,
                                  volatile uint32_t *dropped_counter);
static uint8_t Gateway_CAN_SendNow(CAN_HandleTypeDef *hcan, const GatewayCanFrame *frame);
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
  memset(&gateway_can1_rx_queue, 0, sizeof(gateway_can1_rx_queue));
  memset(&gateway_can2_rx_queue, 0, sizeof(gateway_can2_rx_queue));
  memset(&gateway_can1_tx_queue, 0, sizeof(gateway_can1_tx_queue));
  memset(&gateway_can2_tx_queue, 0, sizeof(gateway_can2_tx_queue));

  now = HAL_GetTick();
  g_gateway.can1_last_activity = now;
  g_gateway.can2_last_activity = now;
  gateway_can1_tx_service_request = 1U;
  gateway_can2_tx_service_request = 1U;
}

void Gateway_Process(void)
{
  /* Developer-owned protocol logic runs first. */
  Gateway_ProcessCanBus();
  Gateway_ProcessLinBus();
  Gateway_ProcessSpiCanBus();

  /* Flush any frames queued by developer logic or prior IRQ activity. */
  Gateway_CAN_Service();
}

__weak void Gateway_ProcessCanBus(void)
{
  /*
   * Example:
   * GatewayCanFrame frame;
   * while (Gateway_CAN1_Read(&frame)) { modify frame; (void)Gateway_CAN_Forward(&hcan2, &frame); }
   */
}

__weak void Gateway_ProcessLinBus(void)
{
  /*
   * Example:
   * if (Gateway_LIN_FramePending(1U)) {
   *   uint8_t *buf = Gateway_LIN_GetRxBuffer(1U);
   *   uint16_t len = Gateway_LIN_GetRxLength(1U);
   *   inspect buf/len here;
   *   Gateway_LIN_Consume(1U);
   * }
   */
}

__weak void Gateway_ProcessSpiCanBus(void)
{
  /*
   * Example:
   * mcp2515_frame_t frame;
   * while (Gateway_MCP2515_Read(&frame)) { (void)Gateway_MCP2515_ForwardToCAN(&hcan1, &frame); }
   */
}

void Gateway_CAN_Init(CAN_HandleTypeDef *phcan1, CAN_HandleTypeDef *phcan2)
{
  (void)phcan1;
  (void)phcan2;
  Gateway_CAN_ConfigFilters();
}

void Gateway_CAN_Start(void)
{
  uint32_t notifications;

  notifications = CAN_IT_TX_MAILBOX_EMPTY |
                  CAN_IT_RX_FIFO0_MSG_PENDING |
                  CAN_IT_RX_FIFO1_MSG_PENDING |
                  CAN_IT_ERROR_WARNING |
                  CAN_IT_ERROR_PASSIVE |
                  CAN_IT_BUSOFF |
                  CAN_IT_LAST_ERROR_CODE |
                  CAN_IT_ERROR;

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, notifications) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan2, notifications) != HAL_OK)
  {
    Error_Handler();
  }
}

uint8_t Gateway_CAN1_Read(GatewayCanFrame *frame)
{
  if (Gateway_CAN_QueuePeek(&gateway_can1_rx_queue, frame) == 0U)
  {
    return 0U;
  }

  Gateway_CAN_QueuePop(&gateway_can1_rx_queue);
  return 1U;
}

uint8_t Gateway_CAN2_Read(GatewayCanFrame *frame)
{
  if (Gateway_CAN_QueuePeek(&gateway_can2_rx_queue, frame) == 0U)
  {
    return 0U;
  }

  Gateway_CAN_QueuePop(&gateway_can2_rx_queue);
  return 1U;
}

uint8_t Gateway_CAN_Send(CAN_HandleTypeDef *to, const GatewayCanFrame *frame)
{
  if ((to == NULL) || (frame == NULL))
  {
    return 0U;
  }

  if (to->Instance == CAN1)
  {
    gateway_can1_tx_service_request = 1U;
    return Gateway_CAN_QueuePush(&gateway_can1_tx_queue, &g_gateway.can1_tx_dropped, frame);
  }

  if (to->Instance == CAN2)
  {
    gateway_can2_tx_service_request = 1U;
    return Gateway_CAN_QueuePush(&gateway_can2_tx_queue, &g_gateway.can2_tx_dropped, frame);
  }

  return 0U;
}

uint8_t Gateway_CAN_Forward(CAN_HandleTypeDef *to, const GatewayCanFrame *frame)
{
  return Gateway_CAN_Send(to, frame);
}

void Gateway_CAN_Service(void)
{
  GatewayCanFrame frame;

  while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) &&
         (Gateway_CAN_QueuePeek(&gateway_can1_tx_queue, &frame) != 0U))
  {
    if (Gateway_CAN_SendNow(&hcan1, &frame) == 0U)
    {
      break;
    }

    Gateway_CAN_QueuePop(&gateway_can1_tx_queue);
  }

  while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0U) &&
         (Gateway_CAN_QueuePeek(&gateway_can2_tx_queue, &frame) != 0U))
  {
    if (Gateway_CAN_SendNow(&hcan2, &frame) == 0U)
    {
      break;
    }

    Gateway_CAN_QueuePop(&gateway_can2_tx_queue);
  }

  gateway_can1_tx_service_request = 0U;
  gateway_can2_tx_service_request = 0U;
}

void Gateway_UpdateCANActivity(uint8_t can_id)
{
  uint32_t now;

  now = HAL_GetTick();

  if (can_id == 1U)
  {
    g_gateway.can1_last_activity = now;
  }
  else if (can_id == 2U)
  {
    g_gateway.can2_last_activity = now;
  }
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
#endif

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* IRQ context: capture frames quickly and defer protocol handling to main loop. */
  if (hcan->Instance == CAN1)
  {
    Gateway_UpdateCANActivity(1U);
    Gateway_CAN_DrainFifo(hcan, CAN_RX_FIFO0, &gateway_can1_rx_queue, &g_gateway.can1_rx_dropped);
  }
  else if (hcan->Instance == CAN2)
  {
    Gateway_UpdateCANActivity(2U);
    Gateway_CAN_DrainFifo(hcan, CAN_RX_FIFO0, &gateway_can2_rx_queue, &g_gateway.can2_rx_dropped);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* FIFO1 is available for extended-frame routing or custom filter layouts. */
  if (hcan->Instance == CAN1)
  {
    Gateway_UpdateCANActivity(1U);
    Gateway_CAN_DrainFifo(hcan, CAN_RX_FIFO1, &gateway_can1_rx_queue, &g_gateway.can1_rx_dropped);
  }
  else if (hcan->Instance == CAN2)
  {
    Gateway_UpdateCANActivity(2U);
    Gateway_CAN_DrainFifo(hcan, CAN_RX_FIFO1, &gateway_can2_rx_queue, &g_gateway.can2_rx_dropped);
  }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  /* TX-complete only updates state; Gateway_CAN_Service() handles queue draining. */
  if (hcan->Instance == CAN1)
  {
    g_gateway.can1_tx_complete_count++;
    gateway_can1_tx_service_request = 1U;
    Gateway_UpdateCANActivity(1U);
  }
  else if (hcan->Instance == CAN2)
  {
    g_gateway.can2_tx_complete_count++;
    gateway_can2_tx_service_request = 1U;
    Gateway_UpdateCANActivity(2U);
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

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    g_gateway.can1_error_flags = HAL_CAN_GetError(hcan);
    gateway_can1_tx_service_request = 1U;
  }
  else if (hcan->Instance == CAN2)
  {
    g_gateway.can2_error_flags = HAL_CAN_GetError(hcan);
    gateway_can2_tx_service_request = 1U;
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

static HAL_StatusTypeDef Gateway_CAN_ConfigFormatFilter(CAN_HandleTypeDef *hcan,
                                                        uint32_t filter_bank,
                                                        uint32_t fifo_assignment,
                                                        uint8_t extended_only)
{
  CAN_FilterTypeDef filter;

  memset(&filter, 0, sizeof(filter));
  filter.FilterBank = filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = fifo_assignment;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = CAN2_FILTER_BANK_START;
  filter.FilterIdLow = (extended_only != 0U) ? 0x0004U : 0x0000U;
  filter.FilterMaskIdLow = 0x0004U;

  return HAL_CAN_ConfigFilter(hcan, &filter);
}

static void Gateway_CAN_ConfigFilters(void)
{
  if (Gateway_CAN_ConfigFormatFilter(&hcan1, CAN1_FILTER_BANK_START, CAN_FILTER_FIFO0, 0U) != HAL_OK)
  {
    Error_Handler();
  }

  if (Gateway_CAN_ConfigFormatFilter(&hcan1, CAN1_FILTER_BANK_START + 1U, CAN_FILTER_FIFO1, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  if (Gateway_CAN_ConfigFormatFilter(&hcan2, CAN2_FILTER_BANK_START, CAN_FILTER_FIFO0, 0U) != HAL_OK)
  {
    Error_Handler();
  }

  if (Gateway_CAN_ConfigFormatFilter(&hcan2, CAN2_FILTER_BANK_START + 1U, CAN_FILTER_FIFO1, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t Gateway_CAN_QueuePush(GatewayCanQueue *queue,
                                     volatile uint32_t *dropped_counter,
                                     const GatewayCanFrame *frame)
{
  uint16_t next_head;

  next_head = (uint16_t)((queue->head + 1U) % GATEWAY_CAN_QUEUE_SIZE);
  if (next_head == queue->tail)
  {
    if (dropped_counter != NULL)
    {
      (*dropped_counter)++;
    }
    return 0U;
  }

  queue->buffer[queue->head] = *frame;
  if (queue->buffer[queue->head].header.DLC > 8U)
  {
    queue->buffer[queue->head].header.DLC = 8U;
  }

  queue->head = next_head;
  return 1U;
}

static uint8_t Gateway_CAN_QueuePeek(const GatewayCanQueue *queue, GatewayCanFrame *frame)
{
  if (queue->head == queue->tail)
  {
    return 0U;
  }

  *frame = queue->buffer[queue->tail];
  return 1U;
}

static void Gateway_CAN_QueuePop(GatewayCanQueue *queue)
{
  if (queue->head != queue->tail)
  {
    queue->tail = (uint16_t)((queue->tail + 1U) % GATEWAY_CAN_QUEUE_SIZE);
  }
}

static void Gateway_CAN_DrainFifo(CAN_HandleTypeDef *hcan,
                                  uint32_t fifo,
                                  GatewayCanQueue *queue,
                                  volatile uint32_t *dropped_counter)
{
  GatewayCanFrame frame;

  memset(&frame, 0, sizeof(frame));

  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, fifo, &frame.header, frame.data) != HAL_OK)
    {
      break;
    }

    (void)Gateway_CAN_QueuePush(queue, dropped_counter, &frame);
  }
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
