#!/usr/bin/env python3
"""
CLI nói chuyện với firmware qua giao thức pc_link — dùng để kiểm tra board
độc lập với trình duyệt, và làm phương án dự phòng khi Web Serial không chạy.
ping
    ./pclink_cli.py tlm --rate 100
    ./pclink_cli.py read  --id 5 --addr 132 --len 4
    ./pclink_cli.py write --id 5 --addr 116 --len 4 --value 2048
    ./pclink_cli.py force tare
    ./pclink_cli.py force measure --weight 100
    ./pclink_cli.py cal

    ./pclink_cli.py 
Giao thức: xem docs/pc_link_protocol.md
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Thiếu pyserial:  pip install pyserial")

HDR1, HDR2, FTR = 0xAA, 0x55, 0x03

CMD_PING, CMD_TLM_RATE = 0x01, 0x10
CMD_FT_CMD, CMD_FT_GET = 0x20, 0x21
CMD_REG_READ, CMD_REG_WRITE = 0x30, 0x31
CMD_CAL_GET, CMD_CAL_SET, CMD_CAL_SAVE = 0x40, 0x41, 0x42

RSP = {0x81: "PONG", 0x90: "TLM", 0xA0: "FT_STATE",
       0xB0: "REG_VALUE", 0xC0: "CAL_DATA", 0xEE: "ERR"}
ERRNAME = {1: "BAD_LEN", 2: "UNKNOWN_CMD", 3: "BUS", 4: "FLASH", 5: "PARAM"}

FT_MEASURE, FT_TARE, FT_CLEAR = 1, 2, 3
FT_HOLD_ALL, FT_RELAX_ALL = 4, 5      # bật / nhả torque cho cả dàn


def build(type_, payload=b""):
    cs = (type_ + len(payload) + sum(payload)) & 0xFF
    return bytes([HDR1, HDR2, type_, len(payload)]) + payload + bytes([cs, FTR])


class Decoder:
    """FSM giải khung, gương của process_byte() trong pc_link.c."""

    H1, H2, TYPE, LEN, PAY, CS, FT = range(7)

    def __init__(self):
        self.st = self.H1
        self.buf = bytearray()

    def feed(self, data):
        out = []
        for b in data:
            if self.st == self.H1:
                if b == HDR1:
                    self.st = self.H2
            elif self.st == self.H2:
                self.st = self.TYPE if b == HDR2 else (self.H2 if b == HDR1 else self.H1)
            elif self.st == self.TYPE:
                self.type = b
                self.st = self.LEN
            elif self.st == self.LEN:
                if b > 64:
                    self.st = self.H1
                    continue
                self.len = b
                self.cs = (self.type + b) & 0xFF
                self.buf = bytearray()
                self.st = self.CS if b == 0 else self.PAY
            elif self.st == self.PAY:
                self.buf.append(b)
                self.cs = (self.cs + b) & 0xFF
                if len(self.buf) >= self.len:
                    self.st = self.CS
            elif self.st == self.CS:
                self.st = self.FT if b == self.cs else self.H1
            elif self.st == self.FT:
                if b == FTR:
                    out.append((self.type, bytes(self.buf)))
                self.st = self.H1
        return out


class Link:
    def __init__(self, port, baud=115200, timeout=1.0):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.dec = Decoder()
        self.timeout = timeout

    def send(self, type_, payload=b""):
        self.ser.write(build(type_, payload))

    def poll(self, seconds):
        """Đọc trong khoảng thời gian cho trước, trả về mọi khung nhận được."""
        frames, t0 = [], time.time()
        while time.time() - t0 < seconds:
            data = self.ser.read(256)
            if data:
                frames += self.dec.feed(data)
        return frames

    def request(self, type_, payload=b"", want=None):
        """Gửi rồi chờ khung trả lời đầu tiên (hoặc đúng loại `want`)."""
        self.send(type_, payload)
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            for t, p in self.dec.feed(self.ser.read(256)):
                if want is None or t == want or t == 0xEE:
                    return t, p
        return None, None


def show(t, p):
    name = RSP.get(t, f"0x{t:02X}")
    if t == 0x81:
        return f"PONG   proto={p[0]} motor_id={p[1]}"
    if t == 0x90:
        pos, vel, cur = struct.unpack_from("<iii", p, 0)
        return f"TLM    pos={pos:6d}  vel={vel:6d}  cur={cur:5d} mA  bus={p[12]}"
    if t == 0xB0:
        mid, addr, ln = p[0], struct.unpack_from("<H", p, 1)[0], p[3]
        val = struct.unpack_from("<I", p, 4)[0]
        return f"REG    id={mid} addr={addr} len={ln} value={val}"
    if t == 0xEE:
        return f"ERR    {ERRNAME.get(p[0], p[0])} (lệnh 0x{p[1]:02X})"
    if t == 0xA0:
        f = struct.unpack_from("<10i", p, 4)
        return (f"FT     busy={p[0]} ready={p[1]} tare_valid={p[2]} hit_ceiling={p[3]}\n"
                f"       tare={f[0]}mA weight={f[1]}g avg={f[2]}mA "
                f"min={f[3]} max={f[4]} net={f[5]}mA\n"
                f"       torque={f[6]}uNm kt={f[7]} sag={f[8]} break={f[9]}mA")
    if t == 0xC0:
        j = struct.unpack_from("<8f", p, 0)
        m = struct.unpack_from("<3f", p, 32)
        crc = struct.unpack_from("<I", p, 44)[0]
        return (f"CAL    joint0={j[0:4]}\n       joint1={j[4:8]}\n"
                f"       motor rate={m[0]} gain={m[1]} limit={m[2]}  crc=0x{crc:08X}")
    return f"{name}  {p.hex(' ')}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")

    t = sub.add_parser("tlm");   t.add_argument("--rate", type=int, default=100)
    t.add_argument("--seconds", type=float, default=5.0)

    r = sub.add_parser("read")
    r.add_argument("--id", type=int, default=5)
    r.add_argument("--addr", type=int, required=True)
    r.add_argument("--len", type=int, default=4, choices=[1, 2, 4])

    w = sub.add_parser("write")
    w.add_argument("--id", type=int, default=5)
    w.add_argument("--addr", type=int, required=True)
    w.add_argument("--len", type=int, default=4, choices=[1, 2, 4])
    w.add_argument("--value", type=int, required=True)

    f = sub.add_parser("force")
    f.add_argument("action",
                   choices=["tare", "measure", "get", "clear", "hold", "relax"])
    f.add_argument("--weight", type=int, default=0)

    sub.add_parser("cal")

    a = ap.parse_args()

    try:
        link = Link(a.port, a.baud)
    except serial.SerialException as e:
        sys.exit(f"Không mở được {a.port}: {e}")

    if a.cmd == "ping":
        t_, p = link.request(CMD_PING, want=0x81)
        if t_ is None:
            sys.exit("KHÔNG CÓ TRẢ LỜI. Board chưa nạp firmware mới, sai chân TX/RX,\n"
                     "hoặc USART2 chưa được khởi tạo.")
        print(show(t_, p))

    elif a.cmd == "tlm":
        link.send(CMD_TLM_RATE, struct.pack("<H", a.rate))
        for t_, p in link.poll(a.seconds):
            print(show(t_, p))
        link.send(CMD_TLM_RATE, struct.pack("<H", 0))

    elif a.cmd == "read":
        t_, p = link.request(CMD_REG_READ, struct.pack("<BHB", a.id, a.addr, a.len))
        print(show(t_, p) if t_ else "KHÔNG CÓ TRẢ LỜI")

    elif a.cmd == "write":
        if a.addr <= 63:
            print(f"CẢNH BÁO: addr {a.addr} nằm trong EEPROM. Ghi nhầm ID hoặc Baud "
                  "sẽ mất liên lạc với motor.\nXL330 cũng chỉ cho ghi EEPROM khi "
                  "Torque Enable = 0.")
            if input("Gõ 'yes' để tiếp tục: ").strip() != "yes":
                sys.exit("Đã huỷ.")
        t_, p = link.request(CMD_REG_WRITE,
                             struct.pack("<BHBI", a.id, a.addr, a.len, a.value))
        print(show(t_, p) if t_ else "KHÔNG CÓ TRẢ LỜI")

    elif a.cmd == "force":
        act = {"tare": FT_TARE, "measure": FT_MEASURE, "clear": FT_CLEAR,
               "hold": FT_HOLD_ALL, "relax": FT_RELAX_ALL}
        if a.action == "get":
            t_, p = link.request(CMD_FT_GET, want=0xA0)
            print(show(t_, p) if t_ else "KHÔNG CÓ TRẢ LỜI")
        elif a.action in ("clear", "hold", "relax"):
            # Ba lệnh này làm xong ngay, không sinh result_ready. Đi qua vòng
            # chờ ở dưới là đứng đủ 60s rồi báo "hết giờ" — 'clear' vốn đang
            # dính đúng lỗi đó.
            link.send(CMD_FT_CMD, struct.pack("<Bi", act[a.action], 0))
            time.sleep(0.3)
            t_, p = link.request(CMD_FT_GET, want=0xA0)
            print(show(t_, p) if t_ else "Đã gửi lệnh.")
        else:
            link.send(CMD_FT_CMD, struct.pack("<Bi", act[a.action], a.weight))
            print("Đã gửi lệnh, chờ phép đo xong…")
            for _ in range(240):                       # tối đa 60s
                t_, p = link.request(CMD_FT_GET, want=0xA0)
                if t_ == 0xA0 and p[0] == 0 and p[1] == 1:
                    print(show(t_, p))
                    break
                time.sleep(0.25)
            else:
                print("Hết giờ chờ kết quả.")

    elif a.cmd == "cal":
        t_, p = link.request(CMD_CAL_GET, want=0xC0)
        print(show(t_, p) if t_ else "KHÔNG CÓ TRẢ LỜI")


if __name__ == "__main__":
    main()
