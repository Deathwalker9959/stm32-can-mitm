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
 *   - App_HandleFrame() for every received CAN/MCP2515 frame
 *   - App_Logic_Process() once per main-loop Gateway_Process()
 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_LOGIC_TEMPLATE_H */
