# XL330 Motor API

Minimal OOP-style wrapper for XL330-M288-T. App code stages commands, then `xl330_commit()` sends one optimized write. With multiple motors, commit uses Sync Write internally.

## Quick use

```c
XL330_Controller dxl;

xl330_begin(&dxl, &huart1);
xl330_add(&dxl, 5);
xl330_add(&dxl, 2);

xl330_mode(&dxl, XL330_MODE_POSITION);
xl330_torque(&dxl, XL330_TORQUE_ENABLE);
xl330_goal_position(&dxl, 5, 2300);
xl330_goal_position(&dxl, 2, 1800);
xl330_commit(&dxl);
xl330_update(&dxl);
```

## Core calls

| Call | Meaning |
|---|---|
| `xl330_begin(&dxl, &huart1)` | Start controller on a CubeMX UART |
| `xl330_add(&dxl, id)` | Add one motor ID |
| `xl330_mode(&dxl, mode)` | Stage mode change for all motors |
| `xl330_torque(&dxl, enable)` | Stage torque on/off for all motors |
| `xl330_goal_position(&dxl, id, pulse)` | Stage one motor position |
| `xl330_goal_velocity(&dxl, id, raw)` | Stage one motor velocity |
| `xl330_goal_current(&dxl, id, mA)` | Stage one motor current |
| `xl330_commit(&dxl)` | Apply staged mode/torque/goals |
| `xl330_update(&dxl)` | Sync-read state for all motors |
| `xl330_state(&dxl, id)` | Get cached state pointer |

## Notes

- CubeMX still configures UART pins, baud rate, clocks, and optional direction GPIO.
- Library hides packets, CRC, addresses, group/sync logic.
- Current project demo uses motors `5` and `2` in `Core/Src/main.c`.
