# motor_xl330_usb_sine — giao thức USB

STM32F103C8T6, USB FS (PA11/PA12) hiện ra thành `/dev/ttyACM0`, VID:PID `0483:5740`.
Baud rate do PC đặt không có ý nghĩa. Bus Dynamixel: USART1 PA9, half-duplex.
Baud bus và ID **tự dò lúc boot**: broadcast ping lần lượt ở 1M, 57600, 115200,
2M, 3M, 4M, 9600; baud đầu tiên có trả lời thì quét ID 0..252 (~5 s). PONG báo
baud tìm được. Firmware không ghi EEPROM motor để đổi baud.

Build: `make` (cần `arm-none-eabi-gcc`), nạp: `make flash` (ST-Link + OpenOCD).
Test host: `./run_host_tests.sh`.

## Khung gói

Giống bản USART2: `0xAA 0x55 │ TYPE │ LEN │ PAYLOAD │ CS │ 0x03`,
`CS = (TYPE + LEN + Σpayload) & 0xFF`, little-endian, `LEN ≤ 128`.
Khung sai checksum bị bỏ im lặng.

## Lệnh PC → STM32

| TYPE | Tên | LEN | Payload |
|---|---|---|---|
| `0x01` | PING | 0 | — |
| `0x10` | TLM_RATE | 2 | `u16` chu kỳ ms, 0 = tắt |
| `0x30` | REG_READ | 4 | `u8` id, `u16` addr, `u8` len (1/2/4) |
| `0x31` | REG_WRITE | 8 | `u8` id, `u16` addr, `u8` len, `u32` value |
| `0x50` | MOVE | 20 | `u8` id, `u8` enable, `i32` center, `i32` amp, `u32` freq_mHz, `u32` prof_vel, `u16` goal_cur |

`goal = center + amp·sin(2π·f·t)`, đơn vị pulse. Motor chạy ở **Current-based
Position (mode 5)**:

- `prof_vel` → Profile Velocity (addr 112), raw 0.229 rpm/LSB: trần tốc độ.
  0 hoặc lớn hơn Velocity Limit (addr 44) thì bị kẹp về Velocity Limit.
- `goal_cur` → Goal Current (addr 102), raw: trần mô-men. Kẹp về Current Limit (addr 38).
- `enable = 0` tắt torque. Bật từ trạng thái tắt thì pha về 0 (bắt đầu ở `center`).
  Đang chạy mà đổi amp/freq thì pha giữ nguyên, quỹ đạo không giật.
- `freq ≤ 10000 mHz`, `amp ≥ 0`, id phải trả lời ping lúc khởi động, sai → `ERR PARAM`.

Lúc khởi động mọi motor ở mode 5, **torque tắt**, center = vị trí hiện tại.

Chú ý: nếu `prof_vel` nhỏ hơn tốc độ đỉnh của hình sin (`2π·f·amp` pulse/s),
motor không theo kịp và hình sin bị bẹt thành tam giác, biên độ thật nhỏ hơn
`amp`. TLM có cả `goal` và `pos` để thấy ngay chuyện này.

## Trả lời STM32 → PC

| TYPE | Tên | LEN | Payload |
|---|---|---|---|
| `0x81` | PONG | 6+n | `u8` ver (=3), `u8` n, `u8` id[n], `u32` baud bus (0 = không thấy motor) |
| `0x90` | TLM | 5+15n | `u32` t_ms, `u8` n, n × {`u8` id, `i32` goal, `i32` pos, `i32` vel, `i16` cur} |
| `0xB0` | REG_VALUE | 8 | `u8` id, `u16` addr, `u8` len, `u32` value |
| `0xD0` | MOVE | 20 | như lệnh MOVE, nhưng là bản **đã kẹp** |
| `0xEE` | ERR | 2 | `u8` code, `u8` echo_type — 1 BAD_LEN, 2 UNKNOWN_CMD, 3 BUS, 5 PARAM |

## Biến để xem bằng Live Expressions

`bus_err`, `tick_overrun`, `pclink_tx_drop`, `pclink_rx_overrun`, `motion[]`.
PC13 (LED) THẤP trong lúc một tick đang nói chuyện với bus — đo trên scope.
