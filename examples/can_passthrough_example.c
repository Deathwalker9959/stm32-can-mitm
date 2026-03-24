#include "app_logic.h"

/*
 * README
 *
 * Baseline vehicle CAN gateway.
 *
 * Intended topology
 * - CAN1: upstream controller, logging tool, or development harness
 * - CAN2: real vehicle-side bus or target module bus
 *
 * What this file demonstrates
 * - A true transparent MITM for the CAN path.
 * - No frame filtering.
 * - No payload edits.
 * - No synthetic injection.
 * - No dependence on LIN or auxiliary CAN.
 *
 * Why you would start here
 * - To validate wiring, termination, ACK behavior, and timing first.
 * - To prove the board can survive real traffic before adding policy logic.
 * - To capture baseline traces from both sides before introducing remaps.
 *
 * Typical workflow
 * 1. Flash this example.
 * 2. Confirm CAN1 <-> CAN2 traffic is symmetrical.
 * 3. Save captures from both sides.
 * 4. Move to the more advanced examples once transport is proven stable.
 */

void App_Logic_Init(void)
{
}

void App_Logic_Process(void)
{
}

void App_ProcessLinBus(void)
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
