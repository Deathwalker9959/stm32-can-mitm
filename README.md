# stm32f4-can-gateway

STM32CubeIDE/CMake project for an STM32F4-based MITM gateway with a generic transport layer and an application hook layer for vehicle-specific logic.

## Current default

- CAN1 and CAN2 are the standard on-chip STM32 `bxCAN` controllers and form the default MITM pair.
- USART1, USART2, GPIO, and DMA are initialized from the CubeMX-generated project.
- `gateway.c` owns transport, counters, and bus I/O.
- `app_logic.c` is the public application hook layer and defaults to plain CAN1<->CAN2 passthrough.
- MCP2515 support is optional, disabled by default with `APP_ENABLE_MCP2515 = 0U`, and should be treated as an auxiliary external CAN interface rather than a guaranteed board feature.
- The LIN scheduler and helper library are present in the tree but disabled by default.
- LIN app ownership lives in `App_ProcessLinBus()` and uses the `Gateway_LIN_*` helpers from the transport layer.

## Transport capabilities

- `bxCAN` MITM:
  Uses the STM32's built-in `CAN1` and `CAN2` peripherals for transparent forwarding, filtering, modification, and injection.
- Optional auxiliary CAN:
  Uses an external controller such as `MCP2515` over SPI when a board actually includes it.
- LIN / USART MITM:
  Uses `USART1` and `USART2` LIN mode plus the `Gateway_LIN_*` helpers for raw LIN forwarding and protocol-aware handling.

## Enable LIN support

Set `APP_ENABLE_LIN_BUS` to `1U` in `Core/Inc/main.h`.

When enabled, `main.c` switches USART1 and USART2 to `HAL_LIN_Init()` and re-enables the existing LIN scheduler logic plus the `lin_bus` helper module.

LIN validation behavior is configured per handle in the LIN layer, not in
`app_logic.c`.

Typical setup:

```c
LIN_ValidationConfigTypeDef cfg = {
  .validate_pid_parity = 1U,
  .validate_checksum = 1U,
  .checksum_type = LIN_CHECKSUM_ENHANCED,
};

LIN_Init(&hlin1, &huart1);
LIN_SetValidationConfig(&hlin1, &cfg);
```

Use this to choose whether a given LIN bus should:
- validate PID parity
- validate checksum
- expect classic or enhanced checksum

## Open in STM32CubeIDE

Open the project directory directly in STM32CubeIDE, or import the existing `.project` / `.cproject` files.

The CubeMX configuration is stored in `stm32f4-can-gateway.ioc`.
