# Dynamixel Protocol 2.0 Instruction Details: Action-Ready Implementation Checklist

Date: 2026-08-09
Project: `motor_xl330-m288-t`
Source read: <https://emanual.robotis.com/docs/en/dxl/protocol2/#instruction-details>

## Purpose

This note converts Protocol 2.0 section **5. Instruction Details** into an implementation checklist for upgrading the current basic `motor_driver` into a full, reusable Dynamixel library.

Current state is already okay for basic packet build/transmit/receive, CRC, basic read, and basic write. The next work should add the missing instruction coverage, stricter response handling, multi-motor operations, and safe product-style APIs.

## Global Protocol 2.0 rules to support before advanced instructions

- Instruction packet format:
  - Header: `FF FF FD 00`
  - ID
  - Length low/high
  - Instruction
  - Parameters
  - CRC low/high
- Length rule:
  - `Length = parameter_count + 3`
- Status packet instruction byte must be `0x55`.
- Status packet includes an `Error` byte before returned parameters.
- Broadcast ID is `0xFE`.
- Broadcast response rule:
  - Broadcast returns status only for Ping, Sync Read, and Bulk Read.
  - Other broadcast instructions should not wait for normal status packets.
- Byte stuffing must be supported:
  - If data contains `FF FF FD`, transmit as `FF FF FD FD`.
  - Receive side must find header, validate CRC with stuffing included, then remove stuffing.
- Fast Sync Read and Fast Bulk Read status packets do **not** perform byte stuffing.
- Half-duplex direction switching must wait until the last TX bit is physically sent before RX mode.
- Byte-to-byte transmit gap must stay under 1.5 ms.
- Status error byte must be decoded:
  - bit 7: alert, hardware issue flag
  - bits 0-6: protocol error code
- Error codes to expose:
  - `0x01` Result Fail
  - `0x02` Instruction Error
  - `0x03` CRC Error
  - `0x04` Data Range Error
  - `0x05` Data Length Error
  - `0x06` Data Limit Error
  - `0x07` Access Error

## Recommended core API foundation

Before adding every instruction, create a generic transaction layer:

```c
typedef enum {
    DXL_OK = 0,
    DXL_ERR_TIMEOUT,
    DXL_ERR_CRC,
    DXL_ERR_ID_MISMATCH,
    DXL_ERR_BAD_PACKET,
    DXL_ERR_STATUS_ERROR,
    DXL_ERR_UNSUPPORTED,
    DXL_ERR_BAD_PARAM,
    DXL_ERR_NO_RESPONSE_EXPECTED,
} DXL_Status;

typedef struct {
    uint8_t id;
    uint8_t error;
    uint8_t alert;
    uint16_t param_len;
    uint8_t params[DXL_MAX_PARAM_LEN];
} DXL_StatusPacket;

DXL_Status dxl_tx_instruction(
    DXL_Bus *bus,
    uint8_t id,
    uint8_t instruction,
    const uint8_t *params,
    uint16_t param_len,
    bool expect_response,
    DXL_StatusPacket *response
);
```

Then each instruction becomes only parameter packing plus response parsing.

## Instruction implementation checklist

### 1. Ping `0x01`

Purpose:

- Check if device exists.
- Read model number and firmware version.
- Broadcast ping discovers all connected devices.

Parameters:

- None.

Response parameters:

- P1: model number LSB
- P2: model number MSB
- P3: firmware version

Required APIs:

```c
DXL_Status dxl_ping(DXL_Bus *bus, uint8_t id, DXL_DeviceInfo *info);
DXL_Status dxl_scan(DXL_Bus *bus, DXL_DeviceInfo *list, uint8_t max_count, uint8_t *found_count);
```

Implementation actions:

- Single-ID ping expects one status packet.
- Broadcast ping expects multiple status packets until timeout.
- Parse model as little-endian `uint16_t`.
- Do not treat broadcast multi-response as collision. It is expected.

Priority: high.

### 2. Read `0x02`

Purpose:

- Read data from a control table address.

Parameters:

- P1: address low
- P2: address high
- P3: length low
- P4: length high

Response parameters:

- `length` bytes of data.

Required APIs:

```c
DXL_Status dxl_read(DXL_Bus *bus, uint8_t id, uint16_t addr, uint16_t len, uint8_t *out);
DXL_Status dxl_read_u8(DXL_Bus *bus, uint8_t id, uint16_t addr, uint8_t *out);
DXL_Status dxl_read_u16(DXL_Bus *bus, uint8_t id, uint16_t addr, uint16_t *out);
DXL_Status dxl_read_u32(DXL_Bus *bus, uint8_t id, uint16_t addr, uint32_t *out);
```

Implementation actions:

- Reject broadcast ID for normal read.
- Validate response parameter length exactly equals requested length.
- If returned status has Access Error, expose it as `DXL_ERR_STATUS_ERROR` plus motor error detail.
- Parse little-endian values.

Priority: already basic, improve validation.

### 3. Write `0x03`

Purpose:

- Write data to a control table address immediately.

Parameters:

- P1: address low
- P2: address high
- P3..N: data bytes

Response:

- Status packet may be returned depending on Status Return Level and ID.
- Broadcast write usually has no response.

Required APIs:

```c
DXL_Status dxl_write(DXL_Bus *bus, uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
DXL_Status dxl_write_u8(DXL_Bus *bus, uint8_t id, uint16_t addr, uint8_t value);
DXL_Status dxl_write_u16(DXL_Bus *bus, uint8_t id, uint16_t addr, uint16_t value);
DXL_Status dxl_write_u32(DXL_Bus *bus, uint8_t id, uint16_t addr, uint32_t value);
```

Implementation actions:

- Add `expect_response` policy based on ID and configured status return behavior.
- On broadcast ID, normally send and return `DXL_OK` after TX complete.
- Decode Data Range, Data Length, Data Limit, and Access errors.
- Protect EEPROM writes at application/model layer when torque is enabled.

Priority: already basic, improve response policy and typed helpers.

### 4. Reg Write `0x04`

Purpose:

- Register a write command in the device without executing immediately.
- Later execute it using Action.
- Useful for synchronized command timing across motors.

Parameters:

- Same as Write:
  - P1: address low
  - P2: address high
  - P3..N: data bytes

Response:

- Normal status packet if expected.
- Device sets Registered Instruction control-table flag to `1`.

Required APIs:

```c
DXL_Status dxl_reg_write(DXL_Bus *bus, uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
DXL_Status dxl_reg_write_u32(DXL_Bus *bus, uint8_t id, uint16_t addr, uint32_t value);
```

Implementation actions:

- Reuse generic write parameter packer.
- Add example: reg write goal position/current on multiple motors, then action broadcast.

Priority: medium.

### 5. Action `0x05`

Purpose:

- Execute instructions previously stored with Reg Write.
- Lets multiple devices act at the same time.

Parameters:

- None.

Response:

- Status packet if expected for non-broadcast.
- Broadcast action usually no response.

Required APIs:

```c
DXL_Status dxl_action(DXL_Bus *bus, uint8_t id);
DXL_Status dxl_action_all(DXL_Bus *bus);
```

Implementation actions:

- Support broadcast action as normal use case.
- If Action without Reg Write, motor can return Instruction Error.

Priority: medium.

### 6. Factory Reset `0x06`

Purpose:

- Reset control table to factory defaults.
- Device reboots after successful reset.

Parameters:

- P1 option:
  - `0xFF`: reset all
  - `0x01`: reset all except ID
  - `0x02`: reset all except ID and baudrate

Safety rule:

- Broadcast ID with reset all `0xFF` is blocked by newer firmware as a safety feature.
- This is destructive and should require explicit app-level confirmation.

Required APIs:

```c
DXL_Status dxl_factory_reset(DXL_Bus *bus, uint8_t id, DXL_ResetOption option);
```

Implementation actions:

- Do not expose casual wrapper like `reset_all()` in application code.
- Require explicit enum option.
- For broadcast reset, block by default in library unless compile-time unsafe mode is enabled.
- After success, wait reboot time and clear bus parser state.

Priority: low for normal motor control, high for product completeness.

### 7. Reboot `0x08`

Purpose:

- Reboot the device.

Parameters:

- None.

Required APIs:

```c
DXL_Status dxl_reboot(DXL_Bus *bus, uint8_t id);
```

Implementation actions:

- Expect status before reboot if device returns one.
- After sending, wait reboot delay before next transaction.
- Clear receive buffer after reboot.

Priority: medium.

### 8. Clear `0x10`

Purpose:

- Reset certain stored information.
- For X-series common use: clear multi-turn position information.

Parameters:

- P1 `0x01` plus fixed bytes `44 58 4C 22`: clear multi-turn position.
- P1 `0x02` plus fixed bytes `45 52 43 4C`: clear registered errors, Y-series only.

Rules:

- Multi-turn clear can only execute when motor is stopped.
- If moving, motor returns Result Fail.

Required APIs:

```c
DXL_Status dxl_clear_multi_turn(DXL_Bus *bus, uint8_t id);
DXL_Status dxl_clear_error(DXL_Bus *bus, uint8_t id); // mark model-support limited
```

Implementation actions:

- Add model support flags. XL330 may not support every clear option.
- For clear multi-turn, optionally check Moving register before sending.
- Return Result Fail cleanly if motor refuses.

Priority: medium.

### 9. Control Table Backup `0x20`

Purpose:

- Store current EEPROM and selected RAM control table data to backup area.
- Restore EEPROM data from backup.

Parameters:

- P1 `0x01` plus fixed bytes `43 54 52 4C`: backup.
- P1 `0x02` plus fixed bytes `43 54 52 4C`: restore and reboot.

Rules:

- Torque Enable must be `0`.
- If torque is on, motor returns Result Fail.
- Supported by X330 firmware v46 or above, so XL330-M288-T should be checked by firmware/version.

Required APIs:

```c
DXL_Status dxl_control_table_backup(DXL_Bus *bus, uint8_t id);
DXL_Status dxl_control_table_restore(DXL_Bus *bus, uint8_t id);
```

Implementation actions:

- Read model/firmware first or document support check.
- Require torque-off helper before backup/restore.
- Restore causes reboot, so apply reboot handling.
- Treat as advanced/explicit API, not normal loop API.

Priority: low to medium.

### 10. Sync Read `0x82`

Purpose:

- Read same address and same length from multiple devices using one instruction packet.

Parameters:

- P1: address low
- P2: address high
- P3: length low
- P4: length high
- P5..N: device IDs

Rules:

- Packet ID must be broadcast `0xFE`.
- Each device returns its own status packet.
- Responses arrive in input ID order.
- If data is not continuous, use Indirect Address.

Required APIs:

```c
DXL_Status dxl_sync_read(
    DXL_Bus *bus,
    uint16_t addr,
    uint16_t len,
    const uint8_t *ids,
    uint8_t id_count,
    DXL_SyncReadResult *results
);
```

Implementation actions:

- Send one broadcast packet.
- Receive `id_count` status packets or until timeout.
- Match each response by ID, not only order, to be robust.
- Store per-ID status and error.
- Add XL330 helper for present current/velocity/position across many motors.

Priority: high for multi-motor and fast feedback.

### 11. Sync Write `0x83`

Purpose:

- Write same address and same length to multiple devices using one packet.

Parameters:

- P1: address low
- P2: address high
- P3: data length low
- P4: data length high
- Then for each device:
  - ID
  - data bytes of fixed length

Rules:

- Packet ID must be broadcast `0xFE`.
- No normal status packet expected.

Required APIs:

```c
DXL_Status dxl_sync_write(
    DXL_Bus *bus,
    uint16_t addr,
    uint16_t len,
    const uint8_t *ids,
    const uint8_t *data_by_id,
    uint8_t id_count
);
```

Implementation actions:

- Use broadcast ID.
- Do not wait for status packet.
- Validate buffer size: `4 + id_count * (1 + len)` params.
- Add helpers:
  - `xl330_sync_set_goal_current()`
  - `xl330_sync_set_goal_position()`
  - `xl330_sync_set_torque_enable()`

Priority: high for product-ready multi-motor control.

### 12. Fast Sync Read `0x8A`

Purpose:

- Faster alternative to Sync Read.
- One combined status packet is built across all chained motors.

Parameters:

- Same as Sync Read:
  - address low/high
  - length low/high
  - ID list

Response:

- One packet using broadcast-like ID format.
- Data chunk pattern includes per-device ID, data, and internal CRC bytes.
- Main controller checks only final status-packet CRC.
- No byte stuffing in the Fast Sync Read status packet.

Support:

- X330 firmware v46 or above.
- X430/540 firmware v45 or above.
- P series v12 or above.

Required APIs:

```c
DXL_Status dxl_fast_sync_read(
    DXL_Bus *bus,
    uint16_t addr,
    uint16_t len,
    const uint8_t *ids,
    uint8_t id_count,
    DXL_SyncReadResult *results
);
```

Implementation actions:

- Feature-gate by model/firmware capability.
- Parser must support fast-read response layout.
- Parser must skip byte destuffing for this response type.
- Validate final packet CRC.
- Parse internal per-device CRC fields but do not require main MCU to validate them unless desired.

Priority: medium after normal Sync Read works.

### 13. Bulk Read `0x92`

Purpose:

- Read different addresses and different lengths from multiple devices using one instruction packet.

Parameters per device:

- ID
- address low
- address high
- length low
- length high

Rules:

- Packet ID must be broadcast `0xFE`.
- Same ID cannot appear multiple times.
- Each device returns one status packet.
- Responses arrive in input ID order.
- Use Indirect Address if non-contiguous data is needed from same device.

Required APIs:

```c
typedef struct {
    uint8_t id;
    uint16_t addr;
    uint16_t len;
} DXL_BulkReadItem;

DXL_Status dxl_bulk_read(
    DXL_Bus *bus,
    const DXL_BulkReadItem *items,
    uint8_t item_count,
    DXL_BulkReadResult *results
);
```

Implementation actions:

- Validate no duplicate IDs.
- Send broadcast packet.
- Receive one status packet per item.
- Store variable-length result per ID.
- Add timeout behavior for partial results.

Priority: medium.

### 14. Bulk Write `0x93`

Purpose:

- Write different addresses and different lengths to multiple devices using one instruction packet.

Parameters per device:

- ID
- address low
- address high
- length low
- length high
- data bytes

Rules:

- Packet ID must be broadcast `0xFE`.
- Same ID cannot appear multiple times.
- No normal status packet expected.

Required APIs:

```c
typedef struct {
    uint8_t id;
    uint16_t addr;
    uint16_t len;
    const uint8_t *data;
} DXL_BulkWriteItem;

DXL_Status dxl_bulk_write(DXL_Bus *bus, const DXL_BulkWriteItem *items, uint8_t item_count);
```

Implementation actions:

- Validate no duplicate IDs.
- Validate total param length fits TX buffer.
- Do not wait for status packet.
- Useful for setting different goal currents/positions/registers in one packet.

Priority: medium.

### 15. Fast Bulk Read `0x9A`

Purpose:

- Faster alternative to Bulk Read.
- Reads different addresses/lengths from multiple devices.
- Returns one combined status packet.

Parameters:

- Same as Bulk Read.

Response:

- Combined response with per-device data sections and internal CRC bytes.
- Main controller checks final packet CRC only.
- No byte stuffing in Fast Bulk Read status packet.

Support:

- X330 firmware v46 or above.
- X430/540 firmware v45 or above.
- P series v12 or above.

Required APIs:

```c
DXL_Status dxl_fast_bulk_read(
    DXL_Bus *bus,
    const DXL_BulkReadItem *items,
    uint8_t item_count,
    DXL_BulkReadResult *results
);
```

Implementation actions:

- Reuse Bulk Read parameter builder.
- Add special combined-response parser.
- Feature-gate by model/firmware capability.
- Skip destuffing for fast status packet.

Priority: low to medium after normal Bulk Read works.

## Suggested implementation order

### Stage A: Make current basic driver production-safe

1. Add `DXL_Status` enum.
2. Decode status packet error byte.
3. Add `DXL_StatusPacket` output from generic transaction.
4. Improve response/no-response policy for broadcast ID and Status Return Level.
5. Make byte stuffing and de-stuffing complete and tested.
6. Add strict parser validation:
   - header
   - reserved byte
   - packet ID
   - length
   - instruction `0x55` for status
   - CRC
   - expected param length

### Stage B: Finish single-device instructions

Implement:

1. `dxl_ping()` and `dxl_scan()`
2. improved `dxl_read()`
3. improved `dxl_write()`
4. `dxl_reg_write()`
5. `dxl_action()`
6. `dxl_reboot()`
7. protected `dxl_factory_reset()`
8. `dxl_clear_multi_turn()`
9. `dxl_control_table_backup()` / `restore()`

### Stage C: Add multi-device group operations

Implement:

1. `dxl_sync_write()`
2. `dxl_sync_read()`
3. `dxl_bulk_write()`
4. `dxl_bulk_read()`

### Stage D: Add fast read operations

Implement:

1. `dxl_fast_sync_read()`
2. `dxl_fast_bulk_read()`

Only do this after the normal group operations are stable.

### Stage E: Add XL330 product layer

Implement typed XL330 helpers on top of generic instructions:

```c
xl330_ping()
xl330_config_current_mode()
xl330_config_position_mode()
xl330_set_torque_enable()
xl330_set_goal_current()
xl330_set_goal_velocity()
xl330_set_goal_position()
xl330_read_present_state()
xl330_read_health()
xl330_sync_read_present_state()
xl330_sync_set_goal_current()
xl330_bulk_read_health_and_state()
```

## Validation plan

### Parser/packet tests

- Build known packets from manual and compare bytes/CRC.
- Parse known status packets from manual.
- Test byte stuffing with data containing `FF FF FD`.
- Test CRC fail path.
- Test error byte decode.

### Hardware tests, one motor

- Ping ID 5.
- Read model number and firmware.
- Read present position.
- Write LED on/off.
- Torque off, set current mode, torque on.
- Read present current/velocity/position.
- Reboot and reconnect.

### Hardware tests, multiple motors later

- Broadcast ping scan.
- Sync Read same address/length.
- Sync Write torque enable or LED.
- Bulk Read voltage from one motor and temperature from another.
- Fast Sync Read only after firmware support is confirmed.

## What not to do yet

- Do not start with DMA parser first.
- Do not implement factory reset as an easy one-line public app call.
- Do not depend on `main.h` in protocol or model headers.
- Do not hide motor status errors behind only `0/1` return values.
- Do not assume fast read status packets use the same parser path as normal packets.

## Immediate next coding action when ready

Start by creating the protocol instruction enum and status/error decode layer:

```c
typedef enum {
    DXL_INST_PING                 = 0x01,
    DXL_INST_READ                 = 0x02,
    DXL_INST_WRITE                = 0x03,
    DXL_INST_REG_WRITE            = 0x04,
    DXL_INST_ACTION               = 0x05,
    DXL_INST_FACTORY_RESET        = 0x06,
    DXL_INST_REBOOT               = 0x08,
    DXL_INST_CLEAR                = 0x10,
    DXL_INST_CONTROL_TABLE_BACKUP = 0x20,
    DXL_INST_STATUS               = 0x55,
    DXL_INST_SYNC_READ            = 0x82,
    DXL_INST_SYNC_WRITE           = 0x83,
    DXL_INST_FAST_SYNC_READ       = 0x8A,
    DXL_INST_BULK_READ            = 0x92,
    DXL_INST_BULK_WRITE           = 0x93,
    DXL_INST_FAST_BULK_READ       = 0x9A,
} DXL_Instruction;
```

Then add a generic function to send any instruction and parse a normal status packet. After that, each instruction is simple and safe to add one by one.

---

# Augmentation: implementation-ready blueprint

This section tightens the previous checklist into a direct build plan. Use it as the source of truth when starting code.

## Implementation principle

Do **not** implement all instructions as independent copy-paste functions. Implement one reliable generic packet transaction core, then thin wrappers for each instruction.

Target dependency direction:

```text
application main.c
  -> xl330 product API
    -> protocol2 instruction API
      -> packet encode/decode + CRC + stuffing
        -> transport interface
          -> STM32 HAL UART implementation
```

Rules:

- `dxl_protocol2.*` must not include `main.h` or STM32 HAL headers.
- `dxl_xl330.*` must not include `main.h` or STM32 HAL headers.
- Only `dxl_stm32_hal.*` may include STM32 HAL headers.
- No malloc. All buffers are static or owned by `DXL_Bus`.
- Public APIs return `DXL_Status`, not `uint8_t` success/fail.
- Preserve old API wrappers temporarily so current `main.c` still builds during migration.

## Exact first implementation slice

Start with this minimal slice before touching advanced instructions:

### Files to create

```text
Core/Inc/dxl/dxl_config.h
Core/Inc/dxl/dxl_types.h
Core/Inc/dxl/dxl_transport.h
Core/Inc/dxl/dxl_protocol2.h
Core/Src/dxl/dxl_protocol2.c
```

### Keep existing files at first

```text
Core/Inc/motor_driver.h
Core/Src/motor_driver.c
```

Use existing `motor_driver` as compatibility wrapper until the new `dxl` layer is validated.

## Concrete `dxl_config.h`

```c
#ifndef DXL_CONFIG_H
#define DXL_CONFIG_H

#define DXL_MAX_PACKET_SIZE        256u
#define DXL_MAX_PARAM_LEN          220u
#define DXL_MAX_DEVICES            16u
#define DXL_DEFAULT_TIMEOUT_MS     10u
#define DXL_DEFAULT_RETRY_COUNT    0u
#define DXL_ENABLE_BYTE_STUFFING   1u
#define DXL_ENABLE_FAST_READ       1u
#define DXL_ENABLE_DESTRUCTIVE_API 0u

#endif
```

Rationale:

- 256 bytes is enough for XL330 normal use and keeps RAM small on STM32F103C8.
- Destructive APIs like Factory Reset should compile but be guarded by config or explicit confirmation flag.

## Concrete type model

Use three layers of error state:

1. Transport/protocol status returned by function.
2. Status packet error byte returned by motor.
3. Diagnostics counters stored in bus.

```c
typedef enum {
    DXL_OK = 0,
    DXL_ERR_BAD_ARG,
    DXL_ERR_TIMEOUT,
    DXL_ERR_TX,
    DXL_ERR_RX,
    DXL_ERR_CRC,
    DXL_ERR_BYTE_STUFFING,
    DXL_ERR_BAD_HEADER,
    DXL_ERR_BAD_LENGTH,
    DXL_ERR_ID_MISMATCH,
    DXL_ERR_STATUS_INSTRUCTION,
    DXL_ERR_STATUS_PACKET,
    DXL_ERR_UNSUPPORTED,
    DXL_ERR_BUFFER_OVERFLOW,
    DXL_ERR_NO_RESPONSE_EXPECTED,
} DXL_Status;

typedef enum {
    DXL_DEVICE_ERR_NONE              = 0x00,
    DXL_DEVICE_ERR_RESULT_FAIL       = 0x01,
    DXL_DEVICE_ERR_INSTRUCTION       = 0x02,
    DXL_DEVICE_ERR_CRC               = 0x03,
    DXL_DEVICE_ERR_DATA_RANGE        = 0x04,
    DXL_DEVICE_ERR_DATA_LENGTH       = 0x05,
    DXL_DEVICE_ERR_DATA_LIMIT        = 0x06,
    DXL_DEVICE_ERR_ACCESS            = 0x07,
} DXL_DeviceError;

typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t timeouts;
    uint32_t crc_errors;
    uint32_t status_errors;
    uint32_t malformed_packets;
} DXL_Diagnostics;
```

## Concrete bus object

```c
typedef struct {
    DXL_Transport transport;
    uint8_t tx_buf[DXL_MAX_PACKET_SIZE];
    uint8_t rx_buf[DXL_MAX_PACKET_SIZE];
    uint32_t timeout_ms;
    uint8_t retry_count;
    DXL_Status last_status;
    uint8_t last_device_error;
    uint8_t last_alert;
    DXL_Diagnostics diag;
    void (*trace_cb)(const char *event, const uint8_t *bytes, uint16_t len, void *user);
    void *trace_user;
} DXL_Bus;
```

## Packet core functions to implement first

Implement these internal helpers before instruction APIs:

```c
uint16_t dxl_crc_update(uint16_t crc, const uint8_t *data, uint16_t size);
DXL_Status dxl_build_packet(DXL_Bus *bus, uint8_t id, uint8_t inst,
                            const uint8_t *params, uint16_t param_len,
                            uint8_t *out, uint16_t *out_len);
DXL_Status dxl_apply_byte_stuffing(const uint8_t *in, uint16_t in_len,
                                   uint8_t *out, uint16_t out_cap,
                                   uint16_t *out_len);
DXL_Status dxl_remove_byte_stuffing(uint8_t *packet, uint16_t *packet_len);
DXL_Status dxl_parse_status_packet(DXL_Bus *bus, const uint8_t *packet,
                                   uint16_t packet_len, DXL_StatusPacket *out);
```

Then implement one generic transaction:

```c
DXL_Status dxl_tx_rx(DXL_Bus *bus,
                     uint8_t id,
                     uint8_t inst,
                     const uint8_t *params,
                     uint16_t param_len,
                     DXL_ResponsePolicy policy,
                     DXL_StatusPacket *response);
```

`DXL_ResponsePolicy` should be explicit:

```c
typedef enum {
    DXL_RESPONSE_NONE,
    DXL_RESPONSE_ONE,
    DXL_RESPONSE_MANY,
    DXL_RESPONSE_AUTO,
} DXL_ResponsePolicy;
```

Why this matters:

- Sync Write and Bulk Write should not block waiting for replies.
- Broadcast Ping, Sync Read, and Bulk Read need many replies.
- Single read needs exactly one reply.

## Response policy table

| Instruction | ID | Expected response policy |
|---|---:|---|
| Ping | normal ID | one status packet |
| Ping | broadcast | many status packets until timeout |
| Read | normal ID | one status packet |
| Read | broadcast | invalid, return bad arg |
| Write | normal ID | auto, depends on Status Return Level |
| Write | broadcast | none |
| Reg Write | normal ID | auto |
| Reg Write | broadcast | none or avoid unless intentionally used |
| Action | normal ID | auto |
| Action | broadcast | none |
| Factory Reset | normal ID | one or auto, then reboot delay |
| Factory Reset | broadcast | blocked unless explicitly enabled |
| Reboot | normal ID | one or auto, then reboot delay |
| Clear | normal ID | one status packet |
| Control Table Backup | normal ID | one status packet, restore reboots |
| Sync Read | broadcast only | many status packets, count = id_count |
| Sync Write | broadcast only | none |
| Fast Sync Read | broadcast only | one combined status packet |
| Bulk Read | broadcast only | many status packets, count = item_count |
| Bulk Write | broadcast only | none |
| Fast Bulk Read | broadcast only | one combined status packet |

## Parser acceptance criteria

The parser is ready only when these pass:

- Rejects packet shorter than 10 bytes.
- Rejects bad header.
- Rejects reserved byte not `0x00`.
- Rejects length that does not match actual received bytes.
- Rejects status packet instruction not `0x55`.
- Rejects CRC mismatch.
- Correctly separates alert bit and device error number.
- Correctly returns `param_len = length - 4` for normal status packets.
- Correctly de-stuffs normal status packets after CRC validation.
- Does **not** de-stuff Fast Sync Read / Fast Bulk Read combined responses.

## Manual packet test vectors to add

Use packets from Protocol 2.0 manual as hardcoded tests or debug self-test comments.

Minimum vectors:

- Ping ID 1 instruction:
  - `FF FF FD 00 01 03 00 01 19 4E`
- Ping ID 1 response:
  - `FF FF FD 00 01 07 00 55 00 06 04 26 65 5D`
- Read Present Position instruction:
  - `FF FF FD 00 01 07 00 02 84 00 04 00 1D 15`
- Write Goal Position instruction:
  - `FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89`
- Sync Read instruction for IDs 1 and 2:
  - `FF FF FD 00 FE 09 00 82 84 00 04 00 01 02 CE FA`
- Sync Write instruction for IDs 1 and 2:
  - `FF FF FD 00 FE 11 00 83 74 00 04 00 01 96 00 00 00 02 AA 00 00 00 82 87`

If generated bytes differ, stop and fix packet length, endian, stuffing, or CRC before adding more APIs.

## Concrete implementation sequence with stop points

### Step 1: Protocol constants and enums

Add instruction enum, device error enum, status enum, broadcast ID, header constants.

Stop point:

- Project builds.
- Existing old `motor_driver` API still builds.

### Step 2: CRC and packet builder self-check

Move or duplicate current CRC into `dxl_protocol2.c`.

Stop point:

- Build known manual packets exactly.
- No hardware needed.

### Step 3: Status parser

Implement parser against manual response packets.

Stop point:

- Parse Ping response into model `0x0406`, firmware `0x26`.
- Parse error byte correctly.

### Step 4: Generic transaction

Implement `dxl_tx_rx()` with blocking transport.

Stop point:

- Existing `dxl_ping()` works on motor ID 5.
- Timeout gives `DXL_ERR_TIMEOUT`, not silent fail.

### Step 5: Basic wrappers

Implement typed read/write wrappers.

Stop point:

- Read present position.
- Write LED on/off.
- Set torque enable.

### Step 6: XL330 layer

Create XL330 helpers and register map.

Stop point:

- `xl330_config_current_mode()` works.
- `xl330_read_present_state()` reads current, velocity, position in one contiguous read from address 126 length 10.

### Step 7: Single-device advanced instructions

Add Reg Write, Action, Reboot, Clear, Backup/Restore, guarded Factory Reset.

Stop point:

- Reg Write + Action works with LED or goal current.
- Reboot returns and recovers after delay.
- Destructive functions require explicit guard.

### Step 8: Group operations

Add Sync Write, Sync Read, Bulk Write, Bulk Read.

Stop point:

- For one motor, Sync Read should still work with one ID.
- Sync Write should not wait for status.
- Bulk Read with one item should work before testing many devices.

### Step 9: Fast operations

Add Fast Sync Read and Fast Bulk Read last.

Stop point:

- Only enable if firmware capability confirmed by Ping/model/firmware.
- Combined parser validated.

## XL330-specific product API surface

Keep app code away from raw addresses most of the time.

Recommended product layer:

```c
DXL_Status xl330_ping(DXL_Bus *bus, uint8_t id, DXL_DeviceInfo *info);
DXL_Status xl330_set_torque(DXL_Bus *bus, uint8_t id, bool enable);
DXL_Status xl330_set_led(DXL_Bus *bus, uint8_t id, bool enable);
DXL_Status xl330_set_status_return_level(DXL_Bus *bus, uint8_t id, uint8_t level);
DXL_Status xl330_set_operating_mode(DXL_Bus *bus, uint8_t id, uint8_t mode);
DXL_Status xl330_config_current_mode(DXL_Bus *bus, uint8_t id);
DXL_Status xl330_config_position_mode(DXL_Bus *bus, uint8_t id);
DXL_Status xl330_set_goal_current_raw(DXL_Bus *bus, uint8_t id, int16_t current);
DXL_Status xl330_set_goal_velocity_raw(DXL_Bus *bus, uint8_t id, int32_t velocity);
DXL_Status xl330_set_goal_position_raw(DXL_Bus *bus, uint8_t id, int32_t position);
DXL_Status xl330_read_present_state(DXL_Bus *bus, uint8_t id, DXL_XL330_State *state);
DXL_Status xl330_read_health(DXL_Bus *bus, uint8_t id, DXL_XL330_Health *health);
DXL_Status xl330_stop(DXL_Bus *bus, uint8_t id);
DXL_Status xl330_emergency_torque_off(DXL_Bus *bus, uint8_t id);
```

## Safety rules for XL330 layer

- Changing Operating Mode should do:
  1. torque off
  2. write operating mode
  3. optionally verify by readback
  4. torque on only if requested
- EEPROM writes should require torque off.
- Factory reset should not be called by normal application loop.
- Goal current should be range-checked before writing.
- If status packet alert bit is set, read Hardware Error Status register and store in health struct.

## Performance choices for this STM32F103 + XL330 setup

Recommended initial settings:

- UART: USART1 half-duplex at 1 Mbps.
- Blocking transport first.
- One motor read loop: `xl330_read_present_state()` every 10 ms.
- Do not call torque enable repeatedly in loop.
- Set Status Return Level to a known value during init.
- Use contiguous read address 126 length 10 for current, velocity, position.
- Timeout start value: 5 ms for single motor. Tune lower after stable.

For multiple motors later:

- Commands: Sync Write.
- Feedback: Sync Read first.
- Faster feedback: Fast Sync Read only after normal Sync Read is stable and firmware support is confirmed.

## STM32 HAL transport requirements

Blocking transport must:

- Clear stale RX bytes before TX.
- Switch to TX mode before sending.
- Transmit complete packet without >1.5 ms byte gaps.
- Wait for UART transmission complete, not just TX buffer empty.
- Switch to RX mode immediately after final bit is sent.
- Receive until full packet length is known, then CRC validates, or timeout occurs.

For STM32 HAL half-duplex UART, prefer:

- `HAL_HalfDuplex_EnableTransmitter()` before transmit.
- `HAL_UART_Transmit()` or DMA TX.
- Wait for `UART_FLAG_TC` after TX.
- `HAL_HalfDuplex_EnableReceiver()` before receive.

If current code uses only `HAL_HalfDuplex_Init()` and blocking receive, verify the direction switching is explicit enough.

## Compatibility wrapper plan

Keep these current old names temporarily:

```c
uint8_t dxl_set_torque(uint8_t id, uint8_t enable);
uint8_t dxl_set_operating_mode(uint8_t id, uint8_t mode);
DXL_Response dxl_read_present_data(uint8_t id, DXL_RealTimeData *out_data);
uint8_t dxl_write(uint8_t id, uint32_t addr_info, uint32_t value);
```

But internally call new APIs:

```c
uint8_t dxl_set_torque(uint8_t id, uint8_t enable) {
    return xl330_set_torque(&default_bus, id, enable) == DXL_OK;
}
```

This lets current `main.c` keep working while the new reusable library is built.

## Definition of done for "ready to implement"

This plan is ready to code when the first implementation task is:

1. Create `Core/Inc/dxl/` and `Core/Src/dxl/`.
2. Add config/types/transport/protocol files.
3. Move CRC and packet build logic into `dxl_protocol2.c`.
4. Add manual packet vector checks in comments or a debug self-test function.
5. Keep old wrappers compiling.
6. Build after every small step.

Do **not** start by rewriting all of `motor_driver.c` at once.
