# stm32f4-can-gateway

STM32CubeIDE project for an STM32F446-based CAN gateway board with UART/DMA plumbing kept in place for optional LIN experiments.

## Current default

- CAN1, CAN2, USART1, USART2, GPIO, and DMA are initialized from the CubeMX-generated project.
- The LIN scheduler and helper library are present in the tree but disabled by default.
- The default build is intended to be a clean baseline for publishing and further gateway work.

## Enable LIN support

Set `APP_ENABLE_LIN_BUS` to `1U` in `Core/Inc/main.h`.

When enabled, `main.c` switches USART1 and USART2 to `HAL_LIN_Init()` and re-enables the existing LIN scheduler logic plus the `lin_bus` helper module.

## Open in STM32CubeIDE

Open the project directory directly in STM32CubeIDE, or import the existing `.project` / `.cproject` files.

The CubeMX configuration is stored in `stm32f4-can-gateway.ioc`.
