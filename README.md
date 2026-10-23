# IMGUI_RING_FINGER_TESTING

Firmware STM32 và công cụ PC để thử động cơ **Dynamixel XL330-M288-T** cho ngón áp út của haptic hand.
Một thư viện Protocol 2.0 dùng chung, hai project firmware cho Blue Pill, CLI Python và GUI Dear ImGui.

```
PC ──USB CDC──► STM32F103C8T6 (Blue Pill) ──USART1 half-duplex──► XL330 ×N
 pclink_cli.py        72 MHz, bare-metal               Protocol 2.0
 sine_gui (ImGui)
```

## Nội dung

| Thư mục | Là gì | Trạng thái |
|---|---|---|
| [`XL330_Motor_Lib/`](XL330_Motor_Lib/) | Thư viện STM32 HAL cho XL330, Dynamixel Protocol 2.0. Chép vào project nào cũng được | Có host test |
| [`motor_xl330_usb_sine/`](motor_xl330_usb_sine/) | Firmware chạy motor theo hình sin, điều khiển và telemetry qua USB. Kèm `pclink_cli.py` và GUI `sine_gui` | Đã nạp, đã enumerate USB, đã đọc telemetry. **Chưa đo khi chạy sin trên motor** |
| [`motor_xl330-m288-t/`](motor_xl330-m288-t/) | Project STM32CubeIDE gốc: driver motor, đọc ADC, hiệu chuẩn, bài test lực bằng tạ treo | Dùng để tham khảo, phát triển |

## Bắt đầu nhanh

```bash
# Firmware (cần arm-none-eabi-gcc, make, openocd + ST-Link)
cd motor_xl330_usb_sine
make && make flash                      # kỳ vọng "** Verified OK **"

# Kiểm tra từ PC
python3 tools/pclink_cli.py ping        # PONG proto=3 motors=[5] baud=1000000

# GUI (cần cmake ≥ 3.20, g++ C++20, OpenGL/X11 dev)
cd tools/sine_gui
cmake -S . -B build && cmake --build build -j
./build/sine_gui
```

Nối dây, giao thức, các lỗi thường gặp: [`motor_xl330_usb_sine/README.md`](motor_xl330_usb_sine/README.md).

## Test trên PC (không cần board)

```bash
cd motor_xl330_usb_sine && ./run_host_tests.sh
cd motor_xl330-m288-t   && ./run_host_tests.sh
```

## Số đã đo — 2026-10-23

| Đại lượng | Kết quả |
|---|---|
| USB enumerate | `0483:5740` → `/dev/ttyACM0` |
| Dò bus | Motor ID 5 @ 1 Mbps |
| Telemetry | 200 khung / 2 s, Δt = 10 ms, 0 byte hỏng |

**Chưa đo:** chạy sin trên motor thật, thời gian bus mỗi tick (PC13), `bus_err` sau 10 phút.

## Tài liệu

- Giao thức USB: [`motor_xl330_usb_sine/docs/protocol.md`](motor_xl330_usb_sine/docs/protocol.md)
- API thư viện: [`XL330_Motor_Lib/docs/`](XL330_Motor_Lib/docs/)
- Bài test lực: [`motor_xl330-m288-t/docs/force_test_guide.md`](motor_xl330-m288-t/docs/force_test_guide.md)
