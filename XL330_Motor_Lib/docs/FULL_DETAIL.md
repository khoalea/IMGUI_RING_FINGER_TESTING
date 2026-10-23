# XL330 Motor Lib Full Detail

## Goal

This package gives a high-level STM32 HAL API for one or many XL330-M288-T motors. App code should not manually build packets, remember register numbers, or choose single/group instructions. The manager stages commands and uses efficient bus writes when `XL330_Commit()` is called.

References:

- XL330-M288-T e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
- Dynamixel Protocol 2.0: https://emanual.robotis.com/docs/en/dxl/protocol2/

## Folder layout

| Path | Purpose |
|---|---|
| `include/dxl/xl330_hal.h` | Main include for users. |
| `include/dxl/dxl_xl330_driver.h` | Full public API and named constants. |
| `include/dxl/dxl_xl330.h` | XL330 control-table addresses and low-level helpers. |
| `include/dxl/dxl_protocol2.h` | Dynamixel Protocol 2.0 packet layer. |
| `include/dxl/dxl_stm32_hal.h` | STM32 HAL UART transport. |
| `src/dxl_xl330_driver.c` | Main high-level implementation. |
| `src/dxl_protocol2.c` | Protocol 2.0 implementation. |
| `src/dxl_stm32_hal.c` | UART transport implementation. |
| `src/xl330_hal.c` | Compatibility file only. |
| `tests/test_xl330_hal_host.c` | Host smoke test for staging and packet output. |

## Design model

| Concept | Meaning |
|---|---|
| `XL330_HandleTypeDef` | Manager object for a bus of XL330 motors. |
| Motor ID | Primary way to address a motor. IDs like `5` and `2` are stored once with `XL330_AddMotor()`. |
| Staged setters | `XL330_Set...()` stores desired work only. It does not hit the bus immediately. |
| Commit | `XL330_Commit()` sends pending work in safe order and uses group writes when useful. |
| Update layout | `XL330_Update()` / `XL330_UpdateLayout()` read state for all registered motors. |
| Cached state | `XL330_GetCachedState()` returns latest decoded state after update. |

## Core API

| Area | Calls | Notes |
|---|---|---|
| Init | `XL330_InitDefault`, `XL330_Init`, `XL330_InitEx`, `XL330_InitHalfDuplex` | CubeMX must initialize UART first. |
| Motors | `XL330_AddMotor`, `XL330_RemoveMotor`, `XL330_ClearMotors`, `XL330_Scan`, `XL330_Rescan`, `XL330_Ping` | Works with one or many motors. |
| Mode | `XL330_SetMode`, `XL330_SetModeByIndex`, `XL330_SetModeAll` | Uses named mode constants, not raw numbers. |
| Torque | `XL330_SetTorque`, `XL330_SetTorqueByIndex`, `XL330_SetTorqueAll` | Use `XL330_ENABLE` / `XL330_DISABLE`. |
| Goals | `XL330_SetGoalPos`, `XL330_SetGoalVel`, `XL330_SetGoalCur`, `XL330_SetGoalPwm` | Per-ID and by-index variants exist. |
| Profile | `XL330_SetProfileVelocity`, `XL330_SetProfileAcceleration`, `XL330_SetProfile` | Staged profile config. |
| Commit | `XL330_Commit` | Applies staged mode, torque, profile, and goals. |
| Read | `XL330_Update`, `XL330_UpdateLayout`, `XL330_GetCachedState`, `XL330_GetState` | Reads present state and status. |
| Raw expert | `XL330_ReadRaw`, `XL330_WriteRaw`, typed U8/U16/U32 helpers | Use only when high-level API is missing something. |

## Named constants

Use names from `dxl_xl330.h` and `dxl_xl330_driver.h`.

| Need | Example constants |
|---|---|
| Operating mode | `XL330_MODE_POSITION`, `XL330_MODE_VELOCITY`, `XL330_MODE_CURRENT`, `XL330_MODE_PWM`, `XL330_MODE_EXT_POSITION`, `XL330_MODE_CURRENT_POSITION` |
| Torque | `XL330_ENABLE`, `XL330_DISABLE` |
| Layout | `XL330_LAYOUT_DEFAULT`, `XL330_LAYOUT_FAST`, `XL330_LAYOUT_STATE`, `XL330_LAYOUT_HEALTH`, `XL330_LAYOUT_MOTION` |
| Read items | `XL330_POS`, `XL330_VEL`, `XL330_CUR`, `XL330_PWM`, `XL330_VOLT`, `XL330_TEMP`, `XL330_MOVING`, `XL330_HW_ERROR` |

## Typical two-motor flow

```c
#include "dxl/xl330_hal.h"

XL330_HandleTypeDef dxl;

void app_motor_init(void)
{
    XL330_InitDefault(&dxl, &huart1);
    XL330_AddMotor(&dxl, 5);
    XL330_AddMotor(&dxl, 2);

    XL330_SetModeAll(&dxl, XL330_MODE_POSITION);
    XL330_SetTorqueAll(&dxl, XL330_ENABLE);
    XL330_Commit(&dxl);
}

void app_move_position(void)
{
    XL330_SetGoalPos(&dxl, 5, 2300);
    XL330_SetGoalPos(&dxl, 2, 1800);
    XL330_Commit(&dxl);
}

void app_move_velocity(void)
{
    XL330_SetModeAll(&dxl, XL330_MODE_VELOCITY);
    XL330_SetTorqueAll(&dxl, XL330_ENABLE);
    XL330_SetGoalVel(&dxl, 5, 80);
    XL330_SetGoalVel(&dxl, 2, -80);
    XL330_Commit(&dxl);
}

void app_read_state(void)
{
    XL330_Update(&dxl, XL330_LAYOUT_DEFAULT, NULL, 0, NULL);
    const XL330_State *m5 = XL330_GetCachedState(&dxl, 5);
    const XL330_State *m2 = XL330_GetCachedState(&dxl, 2);
    (void)m5;
    (void)m2;
}
```

## CubeMX requirements

The library does not auto-configure STM32 hardware. That is CubeMX's job.

| Required setup | Why |
|---|---|
| UART TX/RX pins and baud rate | XL330 communication bus. Baud must match motor EEPROM baud setting. |
| GPIO direction pin, only for custom half-duplex transceiver | Needed by `XL330_InitHalfDuplex()`. |
| Common ground between STM32 and motor power | UART signal reference. |
| Motor power supply | STM32 logic power cannot power the motor. |

## Verification

The package was checked with:

```sh
gcc -std=c99 -Wall -Wextra -DSTM32F103xB \
  -Iinclude \
  -I../Drivers/STM32F1xx_HAL_Driver/Inc \
  -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include \
  -I../Drivers/CMSIS/Include \
  tests/test_xl330_hal_host.c src/dxl_protocol2.c src/dxl_xl330_driver.c src/xl330_hal.c
```

The full STM32CubeIDE project also built `motor_xl330-m288-t.elf` successfully.

## Notes

- `XL330_Commit()` may return `XL330_NO_PENDING_CHANGES` when nothing changed.
- Prefer high-level setters before raw register access.
- For unknown IDs, call `XL330_Scan()` or `XL330_Ping()` first.
- For another STM32 project, copy this package and configure CubeMX UART again.
