#include "app_logic.h"

/*
 * README
 *
 * Simple LIN MITM example.
 *
 * Topology
 * - LIN1: original car LIN side
 * - LIN2: downstream LIN device or switch module
 *
 * Behavior
 * - Forward LIN1 -> LIN2 unchanged.
 * - Forward LIN2 -> LIN1 unchanged.
 * - While forwarding LIN2 traffic, watch one PID and one button bit.
 * - On a rising edge of that button bit, toggle a local STM32 LED.
 *
 * Why this example exists
 * - It shows the most common LIN gateway use case:
 *   observe one button or switch signal from a LIN device, keep the original
 *   bus behavior intact, and trigger one local side effect.
 *
 * What to customize
 * - Replace `LIN_EXAMPLE_BUTTON_PID` with your real protected ID.
 * - Replace `LIN_EXAMPLE_BUTTON_MASK` with the real button bit.
 * - Replace the GPIO write with your own latch, actuator, or feature state.
 *
 * Validation note
 * - Integrity checks stay in the LIN layer.
 * - Set the per-bus validation policy in App_Logic_Init() with
 *   LIN_SetValidationConfig(...), not in the button handler below.
 */

#define LIN_EXAMPLE_BUTTON_PID   0x92U
#define LIN_EXAMPLE_BUTTON_MASK  0x01U

static uint8_t g_lin_led_state;
static uint8_t g_lin_button_prev;

static void LinExample_SetLocalLed(uint8_t on)
{
  g_lin_led_state = on ? 1U : 0U;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void LinExample_HandleLin2Button(uint8_t *buffer, uint16_t length)
{
  uint8_t button_now;

  if ((buffer == NULL) || (length < 5U))
  {
    return;
  }

  if (buffer[2] != LIN_EXAMPLE_BUTTON_PID)
  {
    return;
  }

  button_now = ((buffer[3] & LIN_EXAMPLE_BUTTON_MASK) != 0U) ? 1U : 0U;

  if ((button_now != 0U) && (g_lin_button_prev == 0U))
  {
    LinExample_SetLocalLed((uint8_t)!g_lin_led_state);
  }

  g_lin_button_prev = button_now;
}

void App_Logic_Init(void)
{
  LIN_ValidationConfigTypeDef lin1_cfg = {
    .validate_pid_parity = 1U,
    .validate_checksum = 1U,
    .checksum_type = LIN_CHECKSUM_ENHANCED,
  };
  LIN_ValidationConfigTypeDef lin2_cfg = {
    .validate_pid_parity = 1U,
    .validate_checksum = 1U,
    .checksum_type = LIN_CHECKSUM_ENHANCED,
  };

  g_lin_led_state = 0U;
  g_lin_button_prev = 0U;
  LinExample_SetLocalLed(0U);
  LIN_SetValidationConfig(&hlin1, &lin1_cfg);
  LIN_SetValidationConfig(&hlin2, &lin2_cfg);
}

void App_Logic_Process(void)
{
}

void App_HandleFrame(const AppFrameContext *ctx, const GatewayCanFrame *frame)
{
  (void)ctx;
  (void)frame;
}

void App_ProcessLinBus(void)
{
  uint8_t *buffer;
  uint16_t length;

  if (Gateway_LIN_FramePending(1U) != 0U)
  {
    (void)Gateway_LIN_Forward(1U);
  }

  if (Gateway_LIN_FramePending(2U) != 0U)
  {
    buffer = Gateway_LIN_GetRxBuffer(2U);
    length = Gateway_LIN_GetRxLength(2U);
    LinExample_HandleLin2Button(buffer, length);
    (void)Gateway_LIN_Forward(2U);
  }
}
