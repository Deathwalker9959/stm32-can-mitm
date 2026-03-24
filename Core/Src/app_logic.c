#include "app_logic.h"

void App_Logic_Init(void)
{
}

void App_Logic_Process(void)
{
}

void App_HandleFrame(const AppFrameContext *ctx, const GatewayCanFrame *frame)
{
  if ((ctx == NULL) || (frame == NULL))
  {
    return;
  }

  switch (ctx->source_bus)
  {
    case GATEWAY_BUS_CAN1:
      (void)Gateway_SendToBus(GATEWAY_BUS_CAN2, frame);
      break;

    case GATEWAY_BUS_CAN2:
      (void)Gateway_SendToBus(GATEWAY_BUS_CAN1, frame);
      break;

    case GATEWAY_BUS_AUX:
    default:
      break;
  }
}

void App_ProcessLinBus(void)
{
}
