# XL330 HAL API

Compact HAL-style manager for XL330-M288-T. App code stages config/goals, then `XL330_Commit()` sends bus writes in safe order: mode, torque, goals. `XL330_Update()` reads a named layout into cached state and optional reports/buffer.

## Quick start

```c
#include "dxl/xl330_hal.h"

XL330_HandleTypeDef hxl330;

XL330_InitDefault(&hxl330, &huart1);      // CubeMX owns UART setup
XL330_AddMotor(&hxl330, 5);
XL330_AddMotor(&hxl330, 2);

XL330_SetModeAll(&hxl330, XL330_MODE_POSITION);
XL330_SetTorqueAll(&hxl330, XL330_ENABLE);
XL330_SetGoalPos(&hxl330, 5, 2300);
XL330_SetGoalPos(&hxl330, 2, 1800);
XL330_Commit(&hxl330);

XL330_Update(&hxl330, XL330_LAYOUT_DEFAULT, NULL, 0, NULL);
const XL330_State *m5 = XL330_GetCachedState(&hxl330, 5);
```

## Core surface

| Area | Calls | Notes |
|---|---|---|
| Init | `XL330_InitDefault`, `XL330_Init`, `XL330_InitEx`, `XL330_InitHalfDuplex` | Use default storage for quick start or pass `XL330_StorageConfig` for app-owned buffers. |
| Motors | `XL330_AddMotor`, `XL330_RemoveMotor`, `XL330_ClearMotors`, `XL330_Scan`, `XL330_Rescan`, `XL330_Ping` | IDs are primary. By-index variants exist for tight loops. |
| Staged config | `XL330_SetMode`, `XL330_SetModeAll`, `XL330_SetTorque`, `XL330_SetTorqueAll` | Setters only stage work. Nothing hits the bus until commit. |
| Goals | `XL330_SetGoalPos`, `XL330_SetGoalVel`, `XL330_SetGoalCur`, `XL330_SetGoalPwm` | Per-ID and by-index forms. Units follow driver names: pulse, raw velocity, mA, raw PWM. |
| Common config | `XL330_SetProfileVelocity`, `XL330_SetProfileAcceleration`, `XL330_SetProfile` | Profile writes are staged with config work. |
| Commit | `XL330_Commit` | Applies staged changes mode -> torque -> goal. Empty commit returns `XL330_NO_PENDING_CHANGES`. |
| Read layouts | `XL330_AddLayout`, `XL330_AddNamedLayout`, `XL330_Update`, `XL330_UpdateLayout` | Built-ins include `XL330_LAYOUT_DEFAULT`, `FAST`, `STATE`, `HEALTH`, `MOTION`. |
| State/report | `XL330_GetCachedState`, `XL330_GetState`, `XL330_GetStatusReport`, `XL330_GetLastReport` | Cached state is updated by `XL330_Update*`. |
| Expert raw | `XL330_ReadRaw`, `XL330_WriteRaw`, `XL330_ReadU8/U16/U32`, `XL330_WriteU8/U16/U32` | Keep raw register access out of normal control flow. |

## Staged commit example

```c
XL330_SetMode(&hxl330, 5, XL330_MODE_CURRENT);
XL330_SetTorque(&hxl330, 5, XL330_ENABLE);
XL330_SetGoalCur(&hxl330, 5, 120);

XL330_StatusTypeDef st = XL330_Commit(&hxl330);
if (st == XL330_NO_PENDING_CHANGES) {
    // No staged writes were pending.
}
```

## Update/read layout example

```c
static const XL330_Item fast_items[] = { XL330_POS, XL330_VEL, XL330_CUR };
XL330_AddLayout(&hxl330, XL330_FAST, fast_items, 3);

XL330_UpdateReport reports[XL330_MAX_MOTORS];
uint8_t report_count = 0;
XL330_Update(&hxl330, XL330_FAST, reports, XL330_MAX_MOTORS, &report_count);

uint8_t raw_rows[XL330_MAX_MOTORS * 10];
XL330_UpdateLayout(&hxl330, XL330_FAST, raw_rows, sizeof raw_rows);
```

`XL330_Update()` decodes into `XL330_UpdateReport` plus cached `XL330_State`. `XL330_UpdateLayout()` writes motor-major raw rows: each row is `XL330_GetLayoutStride(layout_id)` bytes, with items appended in layout order as little-endian control-table bytes.

## CubeIDE setup checklist

- Add `Core/Inc/dxl` to project include paths.
- Compile `Core/Src/dxl/dxl_protocol2.c`, `dxl_stm32_hal.c`, and `dxl_xl330_driver.c`. `xl330_hal.c` is only a compatibility translation unit.
- Configure the UART in CubeMX for the XL330 bus baud rate, commonly 57600 or the device-configured baud.
- For half-duplex direction control, configure the direction GPIO output and call `XL330_InitHalfDuplex()`.
- Ensure the selected UART handle, for example `&huart1`, is initialized before `XL330_Init*()`.
- Keep motor power and logic ground common with the STM32 board.
- Use `XL330_Scan()` first when IDs or baud are uncertain.
