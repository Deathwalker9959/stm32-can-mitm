#ifndef __APP_LOGIC_H
#define __APP_LOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gateway.h"

typedef struct
{
  GatewayBus source_bus;
  uint32_t now;
} AppFrameContext;

void App_Logic_Init(void);
void App_Logic_Process(void);
void App_HandleFrame(const AppFrameContext *ctx, const GatewayCanFrame *frame);

#ifdef __cplusplus
}
#endif

#endif /* __APP_LOGIC_H */
