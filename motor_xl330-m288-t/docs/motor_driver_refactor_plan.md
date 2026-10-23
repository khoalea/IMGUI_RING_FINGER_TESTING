# Motor Driver Refactor Plan: Reusable XL330 / Dynamixel Protocol 2.0 Library

Date: 2026-08-09
Project: `motor_xl330-m288-t`
Target motor: ROBOTIS Dynamixel XL330-M288-T

## Goal

Turn the current `motor_driver.c/.h` into a faster, cleaner, reusable product-style driver that can be used in this STM32CubeIDE project and copied into other embedded projects with minimal changes.

The driver should support:

- XL330-M288-T current, velocity, position, and PWM control modes.
- Fast read/write of present current, velocity, position, voltage, temperature, and error state.
- Protocol 2.0 packet correctness: header, length, instruction/status, byte stuffing, CRC, timeout, error parsing.
- Blocking mode first, then optional DMA/non-blocking mode.
- Reuse across projects by separating protocol, model-specific registers, and STM32 HAL transport.

## Research sources

Primary references:

- ROBOTIS Dynamixel Protocol 2.0 manual: <https://emanual.robotis.com/docs/en/dxl/protocol2/>
- ROBOTIS XL330-M288-T manual: <https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/>
- ROBOTIS Dynamixel SDK overview: <https://docs.robotis.com/docs/software/dynamixel_sdk/>
- ROBOTIS SDK PacketHandler docs: <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/c/c_packethandler/>
- Example STM32/LibOpenCM3 Dynamixel driver: <https://github.com/ziteh/robotis-dynamixel-motor-driver>

Important design ideas from research:

1. ROBOTIS SDK separates low-level port handling from packet handling.
2. SDK provides basic read/write and group operations like Sync Read, Sync Write, Bulk Read, and Bulk Write.
3. Protocol 2.0 needs strict packet validation and CRC checking.
4. Dynamixel bus is half-duplex, so transport must handle TX/RX direction timing safely.
5. Sync Write does not require waiting for individual response packets, so it is fast for multi-motor control.
6. Sync Read is the right direction for fast multi-motor state feedback.

## Current problems in this project driver

Current `motor_driver.h` / `motor_driver.c` works for basic testing, but it is not yet product-like.

Problems:

- `motor_driver.h` includes `main.h`, so it is tied to this STM32Cube project.
- Global HAL handles are probably used indirectly, making reuse harder.
- API returns mostly `0/1`, which hides timeout, CRC, ID mismatch, and motor error details.
- Packet layer, transport layer, and XL330 register helpers are mixed together.
- No bus/context object, so multiple buses or different UARTs are hard later.
- No clean init/deinit API.
- No Sync Read / Sync Write helpers yet.
- Blocking communication is simple but not optimal for high-rate control.
- No product-style docs/examples/tests.

## Target architecture

Recommended new file structure:

```text
Core/Inc/dxl/
  dxl_types.h
  dxl_config.h
  dxl_protocol2.h
  dxl_transport.h
  dxl_xl330.h
  dxl_stm32_hal.h

Core/Src/dxl/
  dxl_protocol2.c
  dxl_xl330.c
  dxl_stm32_hal.c
```

Optional later:

```text
docs/
  motor_driver_refactor_plan.md
  xl330_register_notes.md
examples/
  single_motor_current_mode.c
  single_motor_position_mode.c
  sync_read_two_motors.c
```

## Layer responsibilities

### 1. `dxl_types.h`

Contains shared types only. No STM32 includes.

Examples:

```c
typedef enum {
    DXL_OK = 0,
    DXL_ERR_TIMEOUT,
    DXL_ERR_CRC,
    DXL_ERR_ID_MISMATCH,
    DXL_ERR_STATUS_ERROR,
    DXL_ERR_BAD_PACKET,
    DXL_ERR_BAD_PARAM,
    DXL_ERR_TRANSPORT,
} DXL_Status;

typedef struct {
    int16_t present_current;
    int32_t present_velocity;
    int32_t present_position;
    uint16_t present_input_voltage;
    uint8_t present_temperature;
    uint8_t hardware_error_status;
} DXL_XL330_State;
```

### 2. `dxl_transport.h`

Abstract transport interface. This makes the protocol reusable on STM32 HAL, LL, bare metal, Linux, or another MCU.

```c
typedef struct {
    void *user;
    DXL_Status (*write)(void *user, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
    DXL_Status (*read)(void *user, uint8_t *data, uint16_t max_len, uint16_t *out_len, uint32_t timeout_ms);
    void (*set_tx_mode)(void *user);
    void (*set_rx_mode)(void *user);
    uint32_t (*millis)(void *user);
} DXL_Transport;
```

### 3. `dxl_protocol2.h/.c`

Pure Dynamixel Protocol 2.0 logic.

Responsibilities:

- Build instruction packets.
- Byte stuffing and de-stuffing.
- CRC-16 calculation.
- Parse status packets.
- Validate ID, length, status instruction, error byte, and CRC.
- Implement generic protocol instructions:
  - Ping
  - Read
  - Write
  - Reg Write
  - Action
  - Factory Reset
  - Reboot
  - Clear
  - Sync Read
  - Sync Write
  - Bulk Read
  - Bulk Write

Core API shape:

```c
typedef struct {
    DXL_Transport transport;
    uint8_t tx_buf[DXL_MAX_PACKET_SIZE];
    uint8_t rx_buf[DXL_MAX_PACKET_SIZE];
    uint32_t default_timeout_ms;
} DXL_Bus;

DXL_Status dxl_bus_init(DXL_Bus *bus, const DXL_Transport *transport);
DXL_Status dxl_ping(DXL_Bus *bus, uint8_t id);
DXL_Status dxl_read(DXL_Bus *bus, uint8_t id, uint16_t addr, uint16_t len, uint8_t *out);
DXL_Status dxl_write(DXL_Bus *bus, uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
DXL_Status dxl_sync_read(DXL_Bus *bus, uint16_t addr, uint16_t len, const uint8_t *ids, uint8_t count, uint8_t *out);
DXL_Status dxl_sync_write(DXL_Bus *bus, uint16_t addr, uint16_t len, const uint8_t *ids, const uint8_t *data, uint8_t count);
```

### 4. `dxl_xl330.h/.c`

Model-specific API for XL330.

Responsibilities:

- XL330 register map.
- Typed helpers for common operations.
- Unit conversions.
- Safe setup sequences.

Example register definitions:

```c
#define XL330_ADDR_ID                    7
#define XL330_ADDR_BAUD_RATE             8
#define XL330_ADDR_RETURN_DELAY_TIME     9
#define XL330_ADDR_OPERATING_MODE        11
#define XL330_ADDR_TORQUE_ENABLE         64
#define XL330_ADDR_LED                   65
#define XL330_ADDR_STATUS_RETURN_LEVEL   68
#define XL330_ADDR_HARDWARE_ERROR_STATUS 70
#define XL330_ADDR_GOAL_PWM              100
#define XL330_ADDR_GOAL_CURRENT          102
#define XL330_ADDR_GOAL_VELOCITY         104
#define XL330_ADDR_PROFILE_ACCELERATION  108
#define XL330_ADDR_PROFILE_VELOCITY      112
#define XL330_ADDR_GOAL_POSITION         116
#define XL330_ADDR_PRESENT_CURRENT       126
#define XL330_ADDR_PRESENT_VELOCITY      128
#define XL330_ADDR_PRESENT_POSITION      132
#define XL330_ADDR_PRESENT_INPUT_VOLTAGE 144
#define XL330_ADDR_PRESENT_TEMPERATURE   146
```

Typed helpers:

```c
DXL_Status xl330_set_torque_enable(DXL_Bus *bus, uint8_t id, bool enable);
DXL_Status xl330_set_operating_mode(DXL_Bus *bus, uint8_t id, uint8_t mode);
DXL_Status xl330_set_goal_current(DXL_Bus *bus, uint8_t id, int16_t current_raw);
DXL_Status xl330_set_goal_velocity(DXL_Bus *bus, uint8_t id, int32_t velocity_raw);
DXL_Status xl330_set_goal_position(DXL_Bus *bus, uint8_t id, int32_t position_raw);
DXL_Status xl330_read_state(DXL_Bus *bus, uint8_t id, DXL_XL330_State *state);
DXL_Status xl330_config_current_mode(DXL_Bus *bus, uint8_t id);
```

### 5. `dxl_stm32_hal.h/.c`

STM32 HAL transport implementation.

Responsibilities:

- Store `UART_HandleTypeDef *`.
- Set half-duplex TX/RX mode if needed.
- Use `HAL_HalfDuplex_Init` compatible logic.
- Implement blocking TX/RX first.
- Add optional DMA later.

Example:

```c
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    bool has_dir_pin;
} DXL_STM32_HAL_Port;

DXL_Status dxl_stm32_hal_transport_init(
    DXL_Transport *transport,
    DXL_STM32_HAL_Port *port,
    UART_HandleTypeDef *huart
);
```

## Speed improvement plan

### Phase 1: Reduce unnecessary bus traffic

Current main loop sometimes enables torque repeatedly. Product code should not do this.

Better:

1. Disable torque.
2. Set operating mode.
3. Enable torque once.
4. In loop, only read state or send changed command.

Example:

```c
xl330_config_current_mode(&bus, motor_id);

while (1) {
    xl330_read_state(&bus, motor_id, &state);
    HAL_Delay(10);
}
```

### Phase 2: Read contiguous blocks

For present current, velocity, and position, read once from address `126` with length `10`.

Data layout:

- Present Current: address 126, 2 bytes
- Present Velocity: address 128, 4 bytes
- Present Position: address 132, 4 bytes

One read is faster than three separate reads.

Later, optionally read address `126` length `21` to include current, velocity, position, velocity trajectory, position trajectory, voltage, and temperature.

### Phase 3: Optimize timeout

Use short timeout for 1 Mbps single motor communication.

Suggested start:

```c
bus.default_timeout_ms = 3; // for one motor at 1 Mbps
```

Then tune with oscilloscope/logic analyzer.

### Phase 4: Non-blocking/DMA transport

After stable blocking driver:

- TX with DMA.
- RX with DMA circular buffer or interrupt.
- Parser state machine consumes received bytes.
- Transaction state machine tracks pending request and timeout.

This enables control loops without blocking inside UART receive.

### Phase 5: Sync Read / Sync Write

For multiple motors:

- Use Sync Write for commands to many motors at the same address.
- Use Sync Read for present state from many motors.
- Use Bulk Read when different addresses/lengths are needed per motor.

## Product-ready features to add

### Core reliability

- Strict CRC check.
- Strict packet length validation.
- ID mismatch detection.
- Timeout detection.
- Motor hardware error parsing.
- Status Return Level support.
- Optional retry count.
- Last error storage in `DXL_Bus`.

### Safety helpers

- `xl330_safe_init_current_mode()`.
- `xl330_stop()` sets goal current or PWM to zero.
- `xl330_emergency_disable_torque()`.
- Current limit and position limit helpers.
- Temperature and voltage checks.

### Diagnostics

- `dxl_status_to_string()`.
- `xl330_error_to_string()`.
- Optional packet trace callback.
- Optional counters:
  - tx packets
  - rx packets
  - CRC errors
  - timeouts
  - status errors

### Reuse and portability

- No `main.h` in generic headers.
- No global UART inside protocol/model layers.
- One public `DXL_Bus` context.
- Compile-time config in `dxl_config.h`.
- All buffers owned by `DXL_Bus`, no malloc.
- C99 compatible.

### Documentation and examples

- `README.md` for driver usage.
- Register map notes for XL330.
- Example code for:
  - Ping motor.
  - Set ID/baud rate carefully.
  - Current mode setup.
  - Position mode setup.
  - Read present data.
  - Sync write two motors.

## Implementation roadmap

### Milestone 1: Clean current driver without changing behavior

Tasks:

- Rename current public APIs consistently.
- Add `DXL_Status` enum.
- Keep old wrappers temporarily for compatibility.
- Add comments for packet format and register map.
- Build and test with current `main.c`.

Acceptance:

- Project builds.
- Existing motor init still works.
- Existing `dxl_read_present_data()` still works.

### Milestone 2: Introduce transport abstraction

Tasks:

- Create `dxl_transport.h`.
- Create `dxl_stm32_hal.c/.h`.
- Move UART/HAL code out of protocol code.
- Remove `main.h` from `dxl_protocol2.h` and model headers.

Acceptance:

- `dxl_protocol2.c` compiles without STM32 HAL includes.
- STM32 HAL is only included by `dxl_stm32_hal.*`.
- Existing main can initialize a `DXL_Bus` and read motor state.

### Milestone 3: Split protocol and XL330 model layer

Tasks:

- Create `dxl_protocol2.c/.h` for generic packets.
- Create `dxl_xl330.c/.h` for XL330 helpers.
- Move register addresses to `dxl_xl330.h`.
- Replace packed address macro with explicit address + length.

Acceptance:

- `xl330_set_torque_enable()` works.
- `xl330_set_operating_mode()` works.
- `xl330_read_state()` reads current, velocity, position in one transaction.

### Milestone 4: Improve speed in blocking mode

Tasks:

- Remove repeated torque enable from loop.
- Read contiguous state block from address `126` length `10`.
- Tune timeout.
- Add optional retry count.
- Add Status Return Level setup.

Acceptance:

- Single motor state read loop runs reliably at target period.
- No unnecessary writes in the loop.
- Timeout and CRC errors are counted.

### Milestone 5: Add group operations

Tasks:

- Implement Sync Write.
- Implement Sync Read.
- Add examples for two or more motors.

Acceptance:

- Can command multiple motors with one Sync Write packet.
- Can read multiple motor states with Sync Read.

### Milestone 6: Add non-blocking/DMA transport

Tasks:

- TX DMA support.
- RX circular DMA support.
- Parser state machine.
- Transaction state machine.
- Optional callback on response.

Acceptance:

- Control loop does not block during UART wait.
- Driver reports transaction completion or timeout.
- Blocking API can be implemented on top of non-blocking API if desired.

## Suggested final application usage

```c
#include "dxl/dxl_types.h"
#include "dxl/dxl_transport.h"
#include "dxl/dxl_stm32_hal.h"
#include "dxl/dxl_xl330.h"

DXL_Bus dxl_bus;
DXL_Transport dxl_transport;
DXL_STM32_HAL_Port dxl_port;
DXL_XL330_State motor_state;

void app_motor_init(void) {
    dxl_stm32_hal_transport_init(&dxl_transport, &dxl_port, &huart1);
    dxl_bus_init(&dxl_bus, &dxl_transport);

    xl330_config_current_mode(&dxl_bus, 5);
}

void app_motor_loop(void) {
    DXL_Status st = xl330_read_state(&dxl_bus, 5, &motor_state);
    if (st == DXL_OK) {
        // motor_state.present_current
        // motor_state.present_velocity
        // motor_state.present_position
    }
}
```

## Notes for this exact project

Current motor-only `main.c` can stay simple while the driver is refactored.

Recommended temporary main loop after refactor:

```c
while (1) {
    xl330_read_state(&dxl_bus, motor_id, &motor_state);
    HAL_Delay(10);
}
```

Do not bring back calibration, sensor, and comm libraries until the motor driver layer is stable.

## Biggest risks

1. Half-duplex direction switching timing can cause missing status packets.
2. Too-short timeout can create false timeouts.
3. DMA RX parser can become complex, so build blocking mode first.
4. Changing motor EEPROM registers like ID and baud rate must be protected with explicit confirmation in application code.
5. Status Return Level affects whether write instructions return status packets.

## Recommended next coding step

Start with Milestone 1 and 2 only:

1. Create `dxl_types.h` with `DXL_Status` and state structs.
2. Create `DXL_Bus` and `DXL_Transport` types.
3. Move STM32 HAL UART details behind `dxl_stm32_hal.c`.
4. Keep existing working packet logic, but call it through the new bus object.
5. Build and verify with current motor-only `main.c`.

This gives reuse first. After that, optimize speed safely.

---

# Augmentation note: implementation source of truth

For the detailed code-ready instruction/API blueprint, use:

`docs/protocol2_instruction_implementation_checklist.md`

That file now contains the concrete first slice, config/type definitions, bus object, response policy table, parser acceptance criteria, manual packet test vectors, STM32 HAL transport requirements, compatibility wrapper plan, and stop points for each implementation step.

Recommended start remains small:

1. Add `Core/Inc/dxl/` and `Core/Src/dxl/`.
2. Add `dxl_config.h`, `dxl_types.h`, `dxl_transport.h`, `dxl_protocol2.h`, `dxl_protocol2.c`.
3. Move CRC and packet builder first.
4. Verify manual packet vectors.
5. Add parser and generic transaction.
6. Keep old `motor_driver` wrappers compiling until the new layer is stable.
