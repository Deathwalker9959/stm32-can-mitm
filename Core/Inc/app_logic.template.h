#ifndef __APP_LOGIC_TEMPLATE_H
#define __APP_LOGIC_TEMPLATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_logic.h"

/*
 * Copy this file to app_logic.h in an application repo or adapt the public
 * app_logic.c implementation to add project-specific behavior.
 *
 * The gateway transport owns hardware RX/TX and calls:
 *   - App_Logic_Init() once during Gateway_Init()
 *   - App_HandleFrame() for every received bxCAN or auxiliary CAN frame
 *   - App_Logic_Process() once per main-loop Gateway_Process()
 *   - App_ProcessLinBus() when LIN support is enabled and the app wants to
 *     inspect or bridge raw LIN frames using Gateway_LIN_* helpers
 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_LOGIC_TEMPLATE_H */
