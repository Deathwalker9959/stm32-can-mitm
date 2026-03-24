# Examples

This folder contains realistic application-layer examples for the public
gateway base. They are meant to be read and copied into
`Core/Src/app_logic.c` in a downstream project.

These files are not part of the normal firmware build. They are reference
policies that show how to use the public transport API without editing the
transport layer itself.

## What The Examples Cover

- Plain CAN MITM passthrough
- CAN filtering, command remap, and frame injection
- LIN MITM with local side effects
- A mixed CAN + LIN integrated gateway scenario

## How To Use Them

1. Pick the example closest to your target behavior.
2. Read the README block at the top of that file first.
3. Copy only the policy logic you need into `Core/Src/app_logic.c`.
4. Leave `gateway.c` responsible for transport, buffering, and hardware I/O.

## Example Files

### `can_passthrough_example.c`

Use this when you want the cleanest possible starting point.

What it shows:
- CAN1 -> CAN2 forwarding
- CAN2 -> CAN1 forwarding
- no filtering
- no payload modification
- no injected frames

This is the right first step when validating:
- bit timing
- wiring
- ACK behavior
- basic MITM stability

### `can_filter_modify_example.c`

Use this when you want the gateway to actively manipulate one CAN path.

What it shows:
- dropping a noisy frame ID
- enabling a feature from a control frame
- remapping a command ID only when that feature is active
- injecting a periodic heartbeat frame toward the real bus

This matches a common real use case:
- a bench tool or controller talks on CAN1
- the real module sits on CAN2
- the gateway selectively changes one command path

### `lin_mitm_example.c`

Use this when you want to observe LIN traffic and trigger a local action while
still forwarding the original frame.

What it shows:
- consuming raw LIN frames from the public LIN helpers
- watching a target LIN PID and payload bit
- toggling a local STM32 LED on a LIN button press
- forwarding the original frame with break regeneration

This is useful for:
- switch-pack interception
- passive feature triggers
- retrofit indicators or local actuators

### `mixed_can_lin_example.c`

Use this when a project needs both CAN and LIN policy at the same time.

What it shows:
- shared state across CAN and LIN handling
- CAN passthrough plus optional remap
- LIN forwarding with a place for project-specific filtering
- a structure that scales better than stuffing everything into one function

This is the closest example to a real integrated vehicle gateway application.

## Public APIs Used

The examples use only the public hook surface:

- `App_Logic_Init()`
- `App_Logic_Process()`
- `App_HandleFrame(...)`
- `App_ProcessLinBus()`
- `Gateway_SendToBus(...)`
- `Gateway_GetTick()`
- `Gateway_LIN_FramePending(...)`
- `Gateway_LIN_GetRxBuffer(...)`
- `Gateway_LIN_GetRxLength(...)`
- `Gateway_LIN_Consume(...)`
- `Gateway_LIN_Forward(...)`

## Transport Naming

- `GATEWAY_BUS_CAN1` and `GATEWAY_BUS_CAN2` are the standard on-chip bxCAN buses.
- `GATEWAY_BUS_AUX` is the optional auxiliary CAN path, typically backed by an
  external controller such as MCP2515 when a board variant includes it.
