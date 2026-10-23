# XL330 Product API Notes

## What good smart-motor libraries do

Based on ROBOTIS Dynamixel SDK, Dynamixel Easy SDK, Dynamixel Workbench, and other smart-servo APIs, a user-friendly motor library should expose layers:

1. **Transport layer**
   - UART, half-duplex direction pin, timeouts.
   - Board-specific, hidden from app after init.

2. **Protocol layer**
   - Packet building, byte stuffing, CRC, status parsing.
   - Reusable for any Dynamixel Protocol 2.0 device.

3. **Model layer**
   - Named control-table addresses and mode names.
   - Example: `XL330_ADDR_TORQUE_ENABLE`, `XL330_MODE_CURRENT`.

4. **Product API layer**
   - Object-style motor handles.
   - Simple calls like `xl330_init`, `xl330_enable_torque`, `xl330_read_state`.
   - Multi-motor flow should be one ID array, not copy-pasted setup code.

## Current implemented direction

The app now uses only:

```c
#include "dxl/dxl_xl330_driver.h"
```

For one or many motors:

```c
#define MOTOR_COUNT 2u
uint8_t motor_ids[MOTOR_COUNT] = {5, 6};
XL330_Motor motors[MOTOR_COUNT];
XL330_State motor_states[MOTOR_COUNT];
```

Init:

```c
for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    xl330_init(&motors[i], &huart1, motor_ids[i]);
    xl330_set_current_mode(&motors[i]);
    xl330_enable_torque(&motors[i]);
}
```

Loop:

```c
for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    xl330_read_state(&motors[i], &motor_states[i]);
}
```

Scan support:

```c
uint8_t found_ids[16];
uint8_t found_count = 0;
xl330_scan(&huart1, 1, 20, found_ids, 16, &found_count);
```

## Next improvements for true product quality

- Add `XL330_Bus` so many motors share one bus object instead of each motor owning its own bus copy.
- Add `xl330_group_sync_read_state()` for fast multi-motor state reads.
- Add `xl330_group_sync_write_current()` and `xl330_group_sync_write_position()`.
- Add unit conversion helpers: raw current to mA, velocity raw to rpm, voltage raw to V.
- Add configurable timeout/retry policy.
- Add non-blocking/DMA transport later for speed.
- Add examples: single motor, multi motor, scan, current mode, position mode.
