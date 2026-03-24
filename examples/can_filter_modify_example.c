/*
 * README
 *
 * CAN-only manipulation example for an automotive gateway.
 *
 * Intended topology
 * - CAN1: upstream command source, bench harness, or body controller side
 * - CAN2: downstream vehicle module or target ECU side
 *
 * Scenario
 * - A command frame on one placeholder ID is rewritten to another placeholder
 *   ID before it reaches the real ECU on CAN2.
 * - A noisy status frame on a separate placeholder ID is dropped entirely.
 *
 * Why this is realistic
 * - This mirrors a common retrofit pattern:
 *   one bus is the command source, the other is the actual ECU bus, and the
 *   gateway conditionally rewrites exactly one control path without breaking
 *   the rest of the vehicle traffic.
 *
 * What to change for a real project
 * - Replace the example IDs with your real command/status IDs.
 */

#include "app_logic.h"

#define EXAMPLE_DROP_ID           0x120U
#define EXAMPLE_REMAP_SOURCE_ID   0x140U
#define EXAMPLE_REMAP_TARGET_ID   0x141U

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
  GatewayCanFrame tx_frame;

  if ((ctx == NULL) || (frame == NULL))
  {
    return;
  }

  if ((frame->header.IDE == CAN_ID_STD) &&
      (frame->header.StdId == EXAMPLE_DROP_ID))
  {
    return;
  }

  tx_frame = *frame;

  if ((ctx->source_bus == GATEWAY_BUS_CAN1) &&
      (tx_frame.header.IDE == CAN_ID_STD) &&
      (tx_frame.header.StdId == EXAMPLE_REMAP_SOURCE_ID))
  {
    tx_frame.header.StdId = EXAMPLE_REMAP_TARGET_ID;
  }

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
