`# Giao thức PC ↔ STM32 (pc_link)

Đường nói chuyện giữa tool calib trên PC và firmware, chạy trên **USART2, 115200 8N1**
(chân PA2 = TX, PA3 = RX).

- Firmware: [`Core/Src/User_code/pc_link.c`](../Core/Src/User_code/pc_link.c),
  [`Core/Inc/User_code/pc_link.h`](../Core/Inc/User_code/pc_link.h)
- GUI: [`tools/calib.html`](../tools/calib.html)
- Test: [`test_pc_link_host.c`](../test_pc_link_host.c), chạy bằng `./run_host_tests.sh`

## Chạy tool

```sh
cd tools
python3 -m http.server 8000
```

Mở `http://localhost:8000/calib.html` bằng **Chrome hoặc Edge**.

Ba điều kiện phải đủ cả, thiếu một cái là ô chọn cổng hiện ra trống hoặc nút Kết nối
bị khoá:

1. **Chrome hoặc Edge trên desktop.** Firefox và Safari không có Web Serial API. Trên
   Ubuntu, Firefox là trình duyệt mặc định — đây là nguyên nhân hay gặp nhất.
2. **URL phải là `localhost` hoặc `127.0.0.1`.** Web Serial đòi secure context.
   `file://` không dùng được, và mở bằng IP LAN (`http://192.168.x.x:8000`) cũng không
   — dù server có listen ở `0.0.0.0`.
3. **Tài khoản thuộc nhóm `dialout`** mới mở được `/dev/ttyUSB*`:
   ```sh
   sudo usermod -aG dialout $USER     # đăng xuất rồi vào lại
   ```

Trang tự phát hiện và báo rõ vướng ở điều kiện nào.

## Kiểm tra không cần trình duyệt

[`tools/pclink_cli.py`](../tools/pclink_cli.py) nói cùng giao thức, dùng để tách bạch
lỗi trình duyệt với lỗi phần cứng/firmware:

```sh
./pclink_cli.py ping                                  # PONG proto=1 motor_id=5
./pclink_cli.py tlm --rate 100 --seconds 5
./pclink_cli.py read  --id 5 --addr 132 --len 4        # present position
./pclink_cli.py write --id 5 --addr 116 --len 4 --value 2048
./pclink_cli.py force tare
./pclink_cli.py force measure --weight 100
./pclink_cli.py cal
```

`ping` chạy được nghĩa là firmware, USART2, dây TX/RX và giao thức đều ổn — lúc đó
mọi vấn đề còn lại đều thuộc về trình duyệt. Cần `pip install pyserial`.

## Khung gói

Giữ đúng kiểu đóng gói của `comm_protocol.c` cũ, thêm `TYPE` và `LEN`:

```
0xAA 0x55 │ TYPE │ LEN │ PAYLOAD[LEN] │ CS │ 0x03
```

- `CS = (TYPE + LEN + tổng các byte payload) & 0xFF` — cộng dồn 8 bit
- Mọi số nhiều byte trong payload là **little-endian**
- `LEN` tối đa 64. Gói khai báo dài hơn bị vứt ngay ở byte `LEN`.

Khung sai checksum hoặc sai footer bị bỏ im lặng, không trả lời. Bộ nhận tự đồng bộ
lại ở byte `0xAA` kế tiếp, kể cả khi gặp `0xAA 0xAA` liên tiếp.

## Lệnh PC → STM32

| TYPE | Tên | LEN | Payload |
|---|---|---|---|
| `0x01` | PING | 0 | — |
| `0x10` | TLM_RATE | 2 | `u16` chu kỳ ms, 0 = tắt stream |
| `0x20` | FT_CMD | 5 | `u8` cmd, `i32` weight_g |
| `0x21` | FT_GET | 0 | — |
| `0x30` | REG_READ | 4 | `u8` id, `u16` addr, `u8` len |
| `0x31` | REG_WRITE | 8 | `u8` id, `u16` addr, `u8` len, `u32` value |
| `0x40` | CAL_GET | 0 | — |
| `0x41` | CAL_SET | 48 | `CalibrationData`, chỉ sửa bản trong RAM |
| `0x42` | CAL_SAVE | 0 | đẩy bản RAM xuống flash |

`FT_CMD.cmd`: `1` = đo, `2` = tare, `3` = xoá bảng log — trùng với `FTEST_CMD_*` trong
`main.c`. `REG_*.len` chỉ nhận 1, 2 hoặc 4; khác đi trả `ERR_PARAM`.

## Trả lời STM32 → PC

| TYPE | Tên | LEN | Payload |
|---|---|---|---|
| `0x81` | PONG | 2 | `u8` proto_ver, `u8` motor_id |
| `0x90` | TLM | 13 | `i32` pos, `i32` vel, `i32` cur, `u8` bus_status |
| `0xA0` | FT_STATE | 56 | xem bảng dưới |
| `0xB0` | REG_VALUE | 8 | `u8` id, `u16` addr, `u8` len, `u32` value |
| `0xC0` | CAL_DATA | 48 | `CalibrationData` |
| `0xEE` | ERR | 2 | `u8` code, `u8` echo_type |

Mã lỗi: `1` BAD_LEN, `2` UNKNOWN_CMD, `3` BUS, `4` FLASH, `5` PARAM.

### FT_STATE — 56 byte

| Offset | Kiểu | Trường |
|---|---|---|
| 0 | u8 | `busy` |
| 1 | u8 | `result_ready` |
| 2 | u8 | `tare_valid` |
| 3 | u8 | `res_hit_ceiling` |
| 4 | i32 | `tare_ma` |
| 8 | i32 | `res_weight_g` |
| 12 | i32 | `res_cur_avg_ma` |
| 16 | i32 | `res_cur_min_ma` |
| 20 | i32 | `res_cur_max_ma` |
| 24 | i32 | `res_cur_net_ma` |
| 28 | i32 | `res_load_torque_unm` |
| 32 | i32 | `res_kt_unm_per_ma` |
| 36 | i32 | `res_sag_pulse` |
| 40 | i32 | `res_break_ma` |
| 44 | u8 | `log_count` (45–47 đệm) |
| 48 | u32 | `samples` |
| 52 | u32 | `bus_errors` |

Ý nghĩa từng trường xem [force_test_guide.md](force_test_guide.md).

### CalibrationData — 48 byte

| Offset | Kiểu | Trường |
|---|---|---|
| 0 | 4×f32 | `joints[0]`: zero, scale, min_angle, max_angle |
| 16 | 4×f32 | `joints[1]`: zero, scale, min_angle, max_angle |
| 32 | f32 | `motor.torque_rate` |
| 36 | f32 | `motor.torque_gain` |
| 40 | f32 | `motor.torque_limit` |
| 44 | u32 | `crc` — firmware tự tính, PC gửi gì cũng bị ghi đè |

## Cách bắt nhịp phép đo lực

`FT_CMD` **không** chặn: nó chỉ đặt lệnh rồi trả về `FT_STATE` ngay. Phép đo thật chạy
trong `ftest_poll()` của vòng lặp chính, mất từ 1.6s (HOLD) tới vài chục giây (RAMP).

Firmware xoá `result_ready` ngay lúc nhận lệnh, nên PC không bị nhầm kết quả cũ là mới.
GUI gọi `FT_GET` mỗi 250ms tới khi thấy `busy == 0 && result_ready == 1`.

Trong lúc đo, các vòng chờ của bài test gọi `PcLink_Task()` liên tục
(`ftest_wait_ms()`), nên link vẫn sống: telemetry vẫn chảy và lệnh của PC không bị mất.
Đây là lý do không dùng `dbh_DelayMS()` trong khối test — buffer DMA 256 byte ở 115200
baud đầy chỉ sau ~22ms.

## Ghi EEPROM của motor

Theo phương án đã chốt, firmware **không chặn địa chỉ nào**. Phần bảo vệ nằm ở GUI:
ghi vào `addr ≤ 63` phải bấm nút hai lần trong vòng 10 giây.

Hai điều cần nhớ khi chạm vùng này:

- Ghi nhầm **ID** hoặc **Baud Rate** làm mất liên lạc với motor, phải cắm U2D2 để cứu.
- XL330 chỉ cho ghi EEPROM khi **Torque Enable = 0**. Ghi lúc torque đang bật sẽ trả
  về `ERR_BUS`. Tắt torque (addr 64 = 0) trước, ghi xong bật lại.

## Ghi chú triển khai

- **TX chặn.** `link_send()` dùng `HAL_UART_Transmit` chứ không phải DMA. Gói dài nhất
  54 byte ≈ 4.7ms ở 115200. Đổi lại không phải nuôi hàng đợi TX và không mất gói vì
  DMA đang bận. Telemetry 20Hz tốn khoảng 6% CPU — nếu sau này cần nhanh hơn thì mới
  tính tới DMA TX.
- **RX dùng DMA vòng**, rút byte trong `PcLink_Task()` — giữ nguyên pattern của
  `Comm_Task()` cũ.
- **`comm_protocol.c` không còn được gọi.** Nó vẫn nằm trong build nhưng bị
  `--gc-sections` loại bỏ. Kênh ASCII `"CALIB#"` cũ đã bị thay hẳn bằng message có
  type. Xoá file đi được nếu bạn không còn dùng cho việc khác.
