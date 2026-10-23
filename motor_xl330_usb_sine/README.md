# motor_xl330_usb_sine

Firmware cho **STM32F103C8T6 (Blue Pill)**: điều khiển Dynamixel **XL330** chạy vị trí theo
hình sin (biên độ, tần số), đặt được trần **velocity** và **torque**, nhận lệnh và đẩy telemetry
qua **USB CDC**. Kèm CLI Python và GUI ImGui C++ cho PC.

```
PC ──USB FS (CDC, /dev/ttyACM*)──► STM32F103C8T6 ──USART1 PA9 half-duplex──► XL330 ×N
   pclink_cli.py / sine_gui          72 MHz, bare-metal                      Protocol 2.0
```

## Kiến trúc

### Firmware (`Core/`, `USB_DEVICE/`)

| Khối | File | Vai trò |
|---|---|---|
| Khởi động | `Core/Src/main.c` | Clock HSE 8 MHz × 9 = 72 MHz, USB = 72/1.5 = 48 MHz. Kéo D+ (PA12) xuống 20 ms để PC enumerate lại sau mỗi lần reset |
| Dò bus | `main.c: dxl_discover()` | Broadcast ping lần lượt ở 1M, 57600, 115200, 2M, 3M, 4M, 9600 baud. Baud đầu tiên có trả lời thì quét ID 0..252 (~5 s) rồi `AddMotor` mọi con trả lời. **Không ghi baud/ID vào motor** |
| Cấu hình motor | `main.c: motion_init()` | Torque OFF → ghi EEPROM **chỉ khi khác**: Return Delay Time = 0, Operating Mode = 5 (Current-based Position). Đọc Current Limit (38), Velocity Limit (44); center = vị trí hiện tại |
| Vòng điều khiển | `main.c: motion_tick()` | Mỗi 10 ms: nếu PC đổi lệnh thì ghi Profile Velocity (112) → Goal Current (102) → Torque Enable; ghi Goal Position theo sin; sync read trạng thái. PC13 THẤP trong lúc dùng bus (chân trace cho scope) |
| Sin | `Core/Src/User_code/motion.c` | `goal = center + amp·sin(phase)`, phase tăng theo thời gian thật (đúng pha kể cả khi tick trễ). Bật lại từ OFF → phase = 0; đổi amp/freq lúc đang chạy → giữ phase |
| Giao thức PC | `Core/Src/User_code/pc_link.c` | ISR USB chép byte vào ring buffer, vòng lặp chính giải khung. Lệnh MOVE bị kẹp theo Limit của motor rồi echo bản đã kẹp. TX bỏ khung sau 2 ms nếu USB bận (không treo) |
| USB glue | `USB_DEVICE/App/usbd_cdc_if.c` | `CDC_Receive_FS → PcLink_RxBytes`, `PcLink_UsbSend` |
| Driver XL330 | `Core/*/User_code/dxl/` | Thư viện Protocol 2.0 dùng lại từ `motor_xl330-m288-t`, polling blocking |

Không RTOS, không malloc, không `HAL_Delay` ngoài init, mọi vòng chờ có timeout.

### Giao thức USB

Khung `0xAA 0x55 | TYPE | LEN | PAYLOAD | CS | 0x03`, `CS = (TYPE+LEN+Σpayload) & 0xFF`,
little-endian. Chi tiết: [`docs/protocol.md`](docs/protocol.md).

| Lệnh PC → MCU | | Trả lời MCU → PC | |
|---|---|---|---|
| `0x01` PING | | `0x81` PONG | ver=3, ids, baud bus |
| `0x10` TLM_RATE u16 ms | | `0x90` TLM | t_ms + {id, goal, pos, vel, cur} mỗi motor |
| `0x30/0x31` REG_READ/WRITE | | `0xB0` REG_VALUE | |
| `0x50` MOVE (20 B) | id, enable, center, amp, freq_mHz, prof_vel, goal_cur | `0xD0` MOVE | bản đã kẹp |
| | | `0xEE` ERR | 1 BAD_LEN, 2 UNKNOWN_CMD, 3 BUS, 5 PARAM |

### PC tools (`tools/`)

- `pclink_cli.py` — CLI: `ping`, `move`, `tlm` (ra CSV), `read`, `write` (cảnh báo khi ghi EEPROM).
- `sine_gui/` — operator console ImGui + ImPlot:
  - GUI = main thread trên một core, I/O serial = thread khác; trao đổi qua queue lock-free (`rt.hpp`), không mutex.
  - Panel mỗi motor: ô nhập số center (nút `= pos`), amp, freq, vel, torque; **Start** cần xác nhận; **STOP ALL** (phím Space); đếm số chu kỳ sin đã hoàn thành (tích phân f·dt theo đồng hồ MCU, về 0 khi Start).
  - Plot goal/pos và Present Current; cảnh báo khi `prof_vel` thấp hơn tốc độ đỉnh của sin (sin sẽ bẹt).
  - An toàn: khoá nút khi > 250 ms không có TLM; bỏ lệnh xếp hàng lúc mất link; đóng GUI = tắt torque mọi motor; giữ cổng độc quyền (`TIOCEXCL`).
  - `proto.hpp` + `test_proto.cpp`: framing thuần, test bằng ctest.

## Nối dây

| Blue Pill | Nối tới |
|---|---|
| GND | GND nguồn motor **và** GND motor (chung mass, nối trước tiên) |
| PA9 | DATA motor. Kéo lên 5 V qua điện trở (đang dùng 10 kΩ, chạy được ở 1 Mbps trong lần đo 2026-10-23; nếu `bus_err` tăng thì hạ xuống ~1 kΩ). PA9 cấu hình open-drain |
| PA10 | không dùng |
| PA11/PA12 | micro-USB tới PC (cáp có dây data) |
| PA13/PA14, GND, 3.3V | ST-Link (SWDIO/SWCLK), BOOT0 = 0 |
| — | V+ motor từ nguồn ngoài, **không** lấy từ Blue Pill |

Cấp nguồn motor **trước** khi cắm USB: firmware chỉ dò bus một lần lúc boot.

## Build và nạp firmware

Cần `arm-none-eabi-gcc`, `make`, `openocd` (hoặc `st-flash`).

```bash
make              # build/motor_xl330_usb_sine.{elf,bin,hex}
st-info --probe   # phải ra chipid 0x0410 (F1xx Medium-density), KHÔNG phải board khác
make flash        # openocd + ST-Link, kỳ vọng "** Verified OK **"
./run_host_tests.sh   # test pc_link + motion trên PC (gcc), kỳ vọng "TẤT CẢ PASS"
```

`main.c`, `pc_link.c`, `motion.c` build với `-Wall -Wextra -Werror`.

## Chạy

### CLI

```bash
pip install pyserial
python3 tools/pclink_cli.py ping                       # PONG proto=3 motors=[5] baud=1000000
python3 tools/pclink_cli.py tlm --seconds 2 --csv log.csv
python3 tools/pclink_cli.py read --id 5 --addr 132 --len 4
python3 tools/pclink_cli.py move --id 5 --center 2486 --amp 300 --freq 0.5   # MOTOR QUAY
python3 tools/pclink_cli.py move --id 5 --off                                # tắt torque
```

`--vel` = Profile Velocity raw (0.229 rpm/LSB, 0 = Velocity Limit), `--cur` = Goal Current raw.
Firmware kẹp cả hai theo Limit đọc từ motor.

### GUI

Cần `cmake ≥ 3.20`, `g++` C++20, OpenGL dev và X11/Wayland dev (để build GLFW 3.4 qua FetchContent).

```bash
cd tools/sine_gui
cmake -S . -B build && cmake --build build -j
ctest --test-dir build                     # test_proto
./build/sine_gui --port /dev/ttyACM0       # tuỳ chọn: --gui-core N --io-core M
```

Có sẵn source imgui/implot ở máy thì thêm
`-DFETCHCONTENT_SOURCE_DIR_IMGUI=<dir> -DFETCHCONTENT_SOURCE_DIR_IMPLOT=<dir>` để khỏi tải.
Bản không cửa sổ (SSH/CI): `cmake -S . -B build-h -DGUI_HEADLESS=ON`, chạy `./build-h/sine_gui --frames 120`.

### Lỗi thường gặp

| Triệu chứng | Nguyên nhân |
|---|---|
| `lsusb` không thấy `0483:5740`, `dmesg` im lặng | Cáp micro-USB chỉ có dây nguồn, hoặc jack bong hàn |
| `dmesg` báo `error -71` | Điện trở kéo lên D+ (R10) sai giá trị trên Blue Pill |
| PING không trả lời, cổng mở được | Tiến trình khác đang giữ cổng (`fuser /dev/ttyACM0`), vd. `haptic_hand_node` dùng cùng VID:PID |
| GUI báo `Device or resource busy` | Một `sine_gui` khác đang mở cổng |
| `motors=[]`, `baud=0` | Motor chưa có nguồn lúc boot, dây PA9/GND, hoặc motor đang Protocol 1.0 (addr 13) |
| Sin bị bẹt thành tam giác | `prof_vel` < tốc độ đỉnh `2π·f·amp` (GUI có cảnh báo) |

## Số đã đo (2026-10-23)

| Đại lượng | Kết quả |
|---|---|
| USB enumerate | `0483:5740` → `/dev/ttyACM0` |
| PONG | `proto=3 motors=[5] baud=1000000` |
| Telemetry | 200 khung / 2 s, Δt_ms min/max = 10/10, 0 byte hỏng |
| Motor ID 5 sau init | RDT = 0, Mode = 5, Current Limit 1750, Velocity Limit 445, Protocol 2, Baud code 3 |

**Chưa đo**: chạy sin trên motor thật (biên độ/pha pos so với goal), thời gian PC13 THẤP mỗi tick,
`bus_err`/`pclink_tx_drop` sau 10 phút chạy liên tục.

## Còn treo

- Đơn vị Goal Current của XL330 (mA/LSB) chưa tra e-Manual; giao thức truyền raw.
- Firmware không có deadman: GUI bị kill -9 thì motor giữ lệnh cuối. Thêm watchdog ~500 ms trong `pc_link.c` nếu cần.
- PA9 có chịu được 5 V (FT) hay không: kiểm lại trong datasheet DS5319.
