#include "app_logic.h"
#include <string.h>

/*
 * README
 *
 * Integrated dual-CAN / dual-LIN vehicle gateway example.
 *
 * Intended topology
 * - CAR <-CAN1-> MCU <-CAN2-> downstream CAN module
 * - CAR <-LIN1-> MCU <-LIN2-> downstream LIN device or switch module
 *
 * Scenario
 * - CAN1 is the original car-facing bus.
 * - CAN2 is the downstream CAN module side.
 * - LIN1 is the original car LIN side.
 * - LIN2 is connected to a separate LIN device that exposes a push button.
 * - A rising edge on that LIN2 button toggles an internal state.
 * - While the toggle state is ON, any CAN1 frame with ID 0x001 is remapped so
 *   CAN2 transmits ID 0x002 instead.
 * - When the toggle state is OFF, CAN traffic passes through unchanged.
 *
 * Why this is realistic
 * - It models a gateway that reads one human input from a LIN side device,
 *   stores internal feature state, and uses that state to modify a command on
 *   a different CAN bus without changing the rest of the traffic.
 *
 * What to change for a real project
 * - Replace the CAN IDs with your actual source/target command IDs.
 * - Replace the LIN PID and button mask with the real switch message.
 * - Replace the local LED state with any project-specific latch or feature.
 */

#define MIXED_REMAP_SOURCE_ID   0x001U
#define MIXED_REMAP_TARGET_ID   0x002U
#define MIXED_LIN_BUTTON_PID    0x92U
#define MIXED_LIN_BUTTON_MASK   0x01U

static uint8_t g_toggle_active;
static uint8_t g_lin2_button_prev;
static uint8_t g_local_led_state;

static void Mixed_SetLocalLed(uint8_t on)
{
  g_local_led_state = on ? 1U : 0U;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void App_Logic_Init(void)
{
  g_toggle_active = 0U;
  g_lin2_button_prev = 0U;
  g_local_led_state = 0U;
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

  if ((ctx->source_bus == GATEWAY_BUS_CAN1) &&
      (g_toggle_active != 0U) &&
      (tx_frame.header.IDE == CAN_ID_STD) &&
      (tx_frame.header.StdId == MIXED_REMAP_SOURCE_ID))
  {
    tx_frame.header.StdId = MIXED_REMAP_TARGET_ID;
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

void App_ProcessLinBus(void)
{
  uint8_t *buffer;
  uint16_t length;
  uint8_t button_now;

  if (Gateway_LIN_FramePending(1U) != 0U)
  {
    buffer = Gateway_LIN_GetRxBuffer(1U);
    length = Gateway_LIN_GetRxLength(1U);
    (void)buffer;
    (void)length;
    (void)Gateway_LIN_Forward(1U);
  }

  if (Gateway_LIN_FramePending(2U) != 0U)
  {
    buffer = Gateway_LIN_GetRxBuffer(2U);
    length = Gateway_LIN_GetRxLength(2U);

    if ((buffer != NULL) &&
        (length >= 5U) &&
        (buffer[2] == MIXED_LIN_BUTTON_PID))
    {
      button_now = ((buffer[3] & MIXED_LIN_BUTTON_MASK) != 0U) ? 1U : 0U;

      if ((button_now != 0U) && (g_lin2_button_prev == 0U))
      {
        g_toggle_active ^= 1U;
        Mixed_SetLocalLed(g_toggle_active);
      }

      g_lin2_button_prev = button_now;
    }

    (void)Gateway_LIN_Forward(2U);
  }
}
