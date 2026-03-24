# stm32f4-can-gateway

STM32CubeIDE/CMake project for an STM32F4-based CAN gateway board with a generic transport layer and an application hook layer for vehicle-specific logic.

## Current default

- CAN1, CAN2, USART1, USART2, GPIO, and DMA are initialized from the CubeMX-generated project.
- `gateway.c` owns transport, counters, and bus I/O.
- `app_logic.c` is the public application hook layer and defaults to plain CAN1<->CAN2 passthrough.
- MCP2515 support is optional and disabled by default with `APP_ENABLE_MCP2515 = 0U`.
- The LIN scheduler and helper library are present in the tree but disabled by default.

## Enable LIN support

Set `APP_ENABLE_LIN_BUS` to `1U` in `Core/Inc/main.h`.

When enabled, `main.c` switches USART1 and USART2 to `HAL_LIN_Init()` and re-enables the existing LIN scheduler logic plus the `lin_bus` helper module.

## Open in STM32CubeIDE

Open the project directory directly in STM32CubeIDE, or import the existing `.project` / `.cproject` files.

The CubeMX configuration is stored in `stm32f4-can-gateway.ioc`.

## Public/private split

- This repo is intended to be the public transport base.
- Downstream private repos should own the `.ioc`, board pinout, timing, and vehicle-specific app logic.
- Avoid adding new `#if` blocks inside CubeMX-owned generated sections; prefer runtime checks or non-generated source files so downstream projects can regenerate safely.
- `Core/Inc/app_logic.template.h` and `Core/Src/app_logic.template.c` are starter templates for a private application layer.
