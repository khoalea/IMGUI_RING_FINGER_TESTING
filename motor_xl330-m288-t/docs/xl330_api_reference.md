# XL330 Product API Reference

Use this layer from application code:

```c
#include "dxl/dxl_xl330_driver.h"
```

All calls return `DXL2_Status`. Treat anything other than `DXL2_OK` as a recoverable bus or device fault and log with `xl330_status_string(status)`.

## Single motor

```c
XL330_Motor motor;
XL330_State state;
DXL2_Status st;

st = xl330_init(&motor, &huart1, 5);          // UART only
// For RS-485 / half-duplex direction control, use xl330_init_half_duplex(...).
if (st == DXL2_OK) st = xl330_set_current_mode(&motor);
if (st == DXL2_OK) st = xl330_enable_torque(&motor);
if (st == DXL2_OK) st = xl330_set_goal_current(&motor, 80); // raw/current command value

while (1) {
    if (xl330_read_state(&motor, &state) == DXL2_OK) {
        // state.present_current, state.present_velocity, state.present_position
    }
    HAL_Delay(10);
}
```

Position mode is the same setup, but call `xl330_set_operating_mode(&motor, XL330_MODE_POSITION)` while torque is disabled, then `xl330_set_goal_position(&motor, pulses)`.

## Two motors, IDs 5 and 2

```c
static const uint8_t ids[] = {5, 2};
XL330_Group group;
XL330_State states[2];
XL330_StatusReport reports[2];

DXL2_Status st = xl330_group_init(&group, &huart1, ids, 2);
if (st == DXL2_OK) st = xl330_group_current_mode_all(&group); // disables torque before mode write
if (st == DXL2_OK) st = xl330_group_torque_all(&group, 1);
```

Keep arrays in the same order as `ids`: index 0 is motor 5, index 1 is motor 2.

## Scan the bus

```c
uint8_t found[16];
uint8_t found_count = 0;
DXL2_Status st = xl330_scan(&huart1, 0, 20, found, 16, &found_count);

if (st == DXL2_OK) {
    for (uint8_t i = 0; i < found_count && i < 16; ++i) {
        // found[i] is an active Dynamixel ID
    }
}
```

For a configured group, use `xl330_group_scan(&group, first, last, found, max, &found_count, reports)` when you also want per-ID status reports.

## Sync read state

```c
DXL2_Status st = xl330_group_sync_read_state(&group, states, reports);
if (st == DXL2_OK) {
    int16_t current_5 = states[0].present_current;
    int32_t pos_2 = states[1].present_position;
} else {
    for (uint8_t i = 0; i < 2; ++i) {
        // reports[i].id, reports[i].status, reports[i].error_code
    }
}
```

This reads present current, velocity, and position for all IDs in one Protocol 2.0 sync-read transaction.

## Sync write current

```c
int16_t currents[2] = {80, -80}; // motor 5, motor 2
DXL2_Status st = xl330_group_current_mode_all(&group);
if (st == DXL2_OK) st = xl330_group_torque_all(&group, 1);
if (st == DXL2_OK) st = xl330_group_sync_write_goal_current(&group, currents);
```

Use signed values. Start small and clamp in application code to your product's mechanical and thermal limits.

## Sync write position

```c
int32_t positions[2] = {2048, 1024}; // motor 5, motor 2
DXL2_Status st = xl330_group_position_mode_all(&group);
if (st == DXL2_OK) st = xl330_group_torque_all(&group, 1);
if (st == DXL2_OK) st = xl330_group_sync_write_goal_position(&group, positions);
```

Mode changes disable torque first. Re-enable torque after mode setup and before writing goals.

## CubeIDE integration checklist

- Add `Core/Inc` to include paths. Keep `Core/Inc/dxl/*.h` under the project.
- Compile these sources: `dxl_protocol2.c`, `dxl_stm32_hal.c`, `dxl_xl330_driver.c`.
- Include only `dxl/dxl_xl330_driver.h` from application files.
- Configure the UART for Dynamixel baud rate, 8-N-1, no flow control.
- Wire common ground and correct half-duplex direction pin if using `xl330_init_half_duplex`.
- Initialize HAL, clocks, GPIO, and UART before calling any XL330 API.
- Keep one `XL330_Group` per shared UART bus. Do not mix group and single-motor objects on the same bus at the same time.
- Check every `DXL2_Status`. On repeated timeout, disable product motion and rescan.
