#include "app_logic.h"

/*
 * Template app layer.
 *
 * Replace the generic passthrough in app_logic.c with logic shaped like this
 * in an application-specific repo:
 *
 *   - inspect ctx->source_bus
 *   - modify or drop GatewayCanFrame values
 *   - forward with Gateway_SendToBus(GATEWAY_BUS_CAN1/2/3, ...)
 *   - use Gateway_GetTick() for debounce/timers
 *   - implement App_ProcessLinBus() if the project enables APP_ENABLE_LIN_BUS
 *     and wants to consume or forward raw LIN frames through Gateway_LIN_* APIs
 */

void App_Logic_Init(void)
{
}

void App_Logic_Process(void)
{
}

void App_HandleFrame(const AppFrameContext *ctx, const GatewayCanFrame *frame)
{
  GatewayCanFrame tx_frame;

  if ((ctx == NULL) || (frame == NULL))
  {
    return;
  }

  tx_frame = *frame;

  switch (ctx->source_bus)
  {
    case GATEWAY_BUS_CAN1:
      (void)Gateway_SendToBus(GATEWAY_BUS_CAN2, &tx_frame);
      break;

    case GATEWAY_BUS_CAN2:
      (void)Gateway_SendToBus(GATEWAY_BUS_CAN1, &tx_frame);
      break;

    case GATEWAY_BUS_AUX:
    default:
      break;
  }
}

void App_ProcessLinBus(void)
{
  /*
   * Example:
   *
   * if (Gateway_LIN_FramePending(1U) != 0U)
   * {
   *   uint8_t *buf = Gateway_LIN_GetRxBuffer(1U);
   *   uint16_t len = Gateway_LIN_GetRxLength(1U);
   *
   *   (void)buf;
   *   (void)len;
   *
   *   Gateway_LIN_Consume(1U);
   * }
   *
   * if (Gateway_LIN_FramePending(2U) != 0U)
   * {
   *   (void)Gateway_LIN_Forward(2U);
   * }
   */
}
