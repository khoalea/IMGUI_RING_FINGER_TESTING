# XL330 Motor Lib

Ready-to-use STM32 HAL library for ROBOTIS XL330-M288-T using Dynamixel Protocol 2.0.

## Install in STM32CubeIDE

| Step | What to do |
|---|---|
| 1 | Copy `include/dxl` into your project, for example `Core/Inc/dxl`. |
| 2 | Copy `src/*.c` into your project, for example `Core/Src/dxl`. |
| 3 | Add include path `Core/Inc/dxl` or this package `include/dxl`. |
| 4 | Make CubeIDE compile `dxl_protocol2.c`, `dxl_stm32_hal.c`, and `dxl_xl330_driver.c`. |
| 5 | Configure the UART in CubeMX. The lib does not configure pins or baud for you. |
| 6 | Include `#include "dxl/xl330_hal.h"`. |

`xl330_hal.c` is only a compatibility translation unit. It is safe to compile, but the real implementation is in `dxl_xl330_driver.c`.

## Example CubeMX IOC config

Use this as a safe starting point for the XL330 UART bus. Match the baud rate to the motor setting.

| CubeMX page | Setting | Example value | Note |
|---|---|---|---|
| Connectivity | Peripheral | `USART1` or your selected UART | The code example uses `&huart1`. |
| USART mode | Mode | `Asynchronous` | Normal UART for Dynamixel TTL bus/transceiver. |
| USART parameters | Baud Rate | `57600` default, or your motor baud | XL330 default is commonly `57600`. Use the baud stored in the motor. |
| USART parameters | Word Length | `8 Bits` | Dynamixel Protocol 2.0 uses 8N1. |
| USART parameters | Parity | `None` | 8N1. |
| USART parameters | Stop Bits | `1` | 8N1. |
| USART parameters | Hardware Flow Control | `None` | Do not use RTS/CTS. |
| USART parameters | Over Sampling | `16 Samples` | Default is fine. |
| GPIO | TX pin | Alternate Function Push Pull, High speed | Example STM32F103 USART1 TX is `PA9`. |
| GPIO | RX pin | Input floating or AF input | Example STM32F103 USART1 RX is `PA10`. |
| NVIC | USART global interrupt | Optional for this blocking driver | Enable only if your app also uses UART IRQ. |
| DMA | USART RX/TX DMA | Not required for current blocking driver | Leave disabled unless you extend transport to DMA. |
| GPIO extra | Direction pin | Optional output | Only needed for a half-duplex transceiver, then use `XL330_InitHalfDuplex()`. |

Recommended simple setup:

```text
USART1: Asynchronous
BaudRate: 57600
WordLength: 8 Bits
Parity: None
StopBits: 1
HardwareFlowControl: None
DMA: Disabled
USART1 global interrupt: Disabled or Enabled, both OK for this blocking lib
```

Wiring checklist:

| XL330 bus | STM32 / board |
|---|---|
| Data/TX-RX through proper TTL half-duplex circuit or Dynamixel adapter | Selected UART pins |
| GND | STM32 GND |
| Motor V+ | External motor supply, not STM32 3.3 V |
| Motor GND | Same ground as STM32 |

## Minimal use

```c
#include "dxl/xl330_hal.h"

XL330_HandleTypeDef dxl;

XL330_InitDefault(&dxl, &huart1);
XL330_AddMotor(&dxl, 5);
XL330_AddMotor(&dxl, 2);

XL330_SetModeAll(&dxl, XL330_MODE_POSITION);
XL330_SetTorqueAll(&dxl, XL330_ENABLE);

XL330_SetGoalPos(&dxl, 5, 2300);
XL330_SetGoalPos(&dxl, 2, 1800);
XL330_Commit(&dxl);

XL330_Update(&dxl, XL330_LAYOUT_DEFAULT, NULL, 0, NULL);
```

## More detail

Read `docs/FULL_DETAIL.md`.
