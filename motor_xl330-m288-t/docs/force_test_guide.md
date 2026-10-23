# Bài test lực XL330 bằng tạ treo

Đo quan hệ giữa moment tải và dòng điện của XL330-M288-T, dùng tạ 100g / 200g / 500g.
Toàn bộ code nằm trong `Core/Src/main.c`, bật/tắt bằng `FTEST_ENABLE`.

## 1. Cấu hình trước khi nạp

Sửa các macro ở đầu `main.c` (khối `USER CODE BEGIN PTD`):

| Macro | Mặc định | Ý nghĩa |
|---|---|---|
| `FTEST_ENABLE` | `1` | `0` = quay lại demo 2 motor cũ |
| `FTEST_ARM_MM` | `50` | **Cần điền**: chiều dài cánh tay đòn, từ tâm trục tới điểm treo tạ (mm) |
| `FTEST_HOLD_POS` | `2048` | **Cần kiểm tra**: vị trí giữ, phải sao cho cánh tay đòn nằm ngang |
| `FTEST_MODE` | `FTEST_MODE_HOLD` | đổi sang `FTEST_MODE_RAMP` cho phép đo breakaway |
| `FTEST_ID` | `MOTOR_5_ID` | động cơ đem test |
| `FTEST_RAMP_DIR` | `+1` | chỉ dùng ở mode RAMP; đổi `-1` nếu motor quay ngược chiều cần nhấc |
| `FTEST_RAMP_MAX_MA` | `900` | trần dòng an toàn ở mode RAMP |

Cánh tay đòn phải **nằm ngang** ở vị trí giữ, vì moment tải tỉ lệ với cos của góc lệch —
lệch 30° là sai số đã 13%.

## 2. Xem số liệu

Nạp firmware, bấm **Debug** trong STM32CubeIDE, cho chạy (F8), rồi:

```
Window > Show View > Live Expressions   →  Add new expression:  ftest
```

Bung cây `ftest` ra là thấy toàn bộ. Các trường cập nhật liên tục ở 20 Hz:

| Trường | Ý nghĩa |
|---|---|
| `cur_ma` | dòng tức thời |
| `pos`, `vel` | vị trí (pulse), vận tốc (raw) |
| `samples` | số lần đọc bus thành công — phải tăng đều, nếu đứng im là bus chết |
| `bus_errors`, `bus_status` | đếm lỗi và mã lỗi lần đọc gần nhất |

## 3. Quy trình đo (mode HOLD)

**Bước 1 — tare (bắt buộc, làm một lần):** tháo hết tạ, để cánh tay đòn trơn.
Trong Live Expressions ghi:

```
ftest.cmd = 2
```

LED PC13 sáng ~2.1s trong lúc đo. Xong: `tare_valid = 1`, `tare_ma` = dòng để giữ
riêng cánh tay đòn. Bước này trừ đi khối lượng cánh tay đòn và ma sát, không có nó
thì số đo lệch một lượng cố định.

**Bước 2 — đo từng quả tạ:** treo tạ vào, rồi ghi hai giá trị:

```
ftest.weight_g = 100     ← khối lượng đang treo
ftest.cmd      = 1       ← chạy đo
```

Firmware bỏ 10 mẫu đầu cho ổn định, gom 32 mẫu tiếp, rồi ghi kết quả. `cmd` tự về 0.

Lặp lại với 200g và 500g.

## 4. Đọc kết quả

Sau mỗi phép đo, `result_ready = 1` và:

| Trường | Ý nghĩa |
|---|---|
| `res_cur_avg_ma` | dòng giữ trung bình (đã bao gồm cánh tay đòn) |
| `res_cur_net_ma` | **= avg − tare**, phần dòng do riêng quả tạ gây ra |
| `res_cur_min_ma` / `res_cur_max_ma` | biên độ dao động — chênh lệch lớn nghĩa là rung hoặc nhiễu |
| `res_load_torque_unm` | moment tải m·g·L tính từ `weight_g` và `ARM_MM`, đơn vị µN·m |
| `res_kt_unm_per_ma` | **moment ÷ dòng thực** — con số để so sánh |
| `res_sag_pulse` | độ võng so với vị trí đặt; càng nặng càng võng nhiều |

`res_kt_unm_per_ma` là chỉ số quan trọng nhất: nếu động cơ tuyến tính thì **cả ba quả
tạ phải cho ra giá trị xấp xỉ nhau**. Lệch nhiều nghĩa là có ma sát lớn, cánh tay đòn
không nằm ngang, hoặc đã chạm giới hạn dòng.

Bảng `log_*[]` giữ lại cả 3 lần đo để nhìn một lượt:
`log_weight_g[]`, `log_cur_ma[]` (dòng thực), `log_torque_unm[]`, `log_kt[]`.
Xoá bảng bằng `ftest.cmd = 3`.

## 5. Mode RAMP

Đổi `FTEST_MODE` sang `FTEST_MODE_RAMP` rồi nạp lại. Cách dùng y hệt (`cmd = 2` để
tare, `cmd = 1` để đo), nhưng firmware chuyển motor sang current mode và tăng goal
current mỗi `FTEST_RAMP_STEP_MA` (10mA) sau mỗi `FTEST_RAMP_STEP_MS` (300ms) tới khi
vị trí lệch quá `FTEST_RAMP_MOVE_PULSE` (30 pulse).

Kết quả chính là `res_break_ma` — dòng tối thiểu để thắng tải.

Nếu `res_hit_ceiling = 1` nghĩa là đã chạm `FTEST_RAMP_MAX_MA` mà vẫn chưa nhấc nổi:
kiểm tra `FTEST_RAMP_DIR` có đúng chiều không, rồi mới nghĩ tới việc nâng trần.
Khi đó `result_ready` giữ nguyên 0 và không có dòng nào được ghi vào log.

Kết thúc mỗi lần ramp, firmware luôn nhả dòng về 0 rồi quay lại POSITION mode giữ
`FTEST_HOLD_POS`, để tạ không rơi tự do.

## 6. Lưu ý về đơn vị

Driver truyền thẳng giá trị raw của control table, không scale. Theo datasheet
XL330-M288-T thì đơn vị Present Current / Goal Current là **1.0 mA mỗi LSB**, nên raw
chính là mA. Nếu bạn đo bằng nguồn ngoài mà thấy lệch hệ số cố định thì kiểm tra lại
điểm này trước — mọi con số dẫn xuất (`res_kt_unm_per_ma`) đều lệch theo.

Phép tính moment dùng số nguyên, không kéo float vào:

```
res_load_torque_unm = weight_g * ARM_MM * 981 / 100     [µN·m]
```

Ví dụ 500g trên cánh tay 50mm → 500 × 50 × 981 / 100 = 245250 µN·m = 0.245 N·m.
