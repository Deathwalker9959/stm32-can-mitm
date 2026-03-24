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

    case GATEWAY_BUS_CAN3:
    default:
      break;
  }
}
