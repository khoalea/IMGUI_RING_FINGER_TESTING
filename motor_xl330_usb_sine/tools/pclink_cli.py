#!/usr/bin/env python3
"""
CLI cho firmware motor_xl330_usb_sine qua USB CDC (/dev/ttyACM0).

    ./pclink_cli.py ping
    ./pclink_cli.py move --id 2 --center 2048 --amp 300 --freq 0.5 --vel 200 --cur 150
    ./pclink_cli.py move --id 2 --off                    # tắt torque
    ./pclink_cli.py tlm --rate 10 --seconds 5 --csv log.csv
    ./pclink_cli.py read  --id 2 --addr 132 --len 4
    ./pclink_cli.py write --id 2 --addr 84 --len 2 --value 800

--vel = Profile Velocity raw (0.229 rpm/LSB), 0 = lấy Velocity Limit của motor.
--cur = Goal Current raw (trần mô-men ở mode 5). Firmware kẹp cả hai theo
        Limit đọc từ motor, lệnh trả về in ra bản ĐÃ kẹp.
Giao thức: xem docs/protocol.md
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
CMD_REG_READ, CMD_REG_WRITE = 0x30, 0x31
CMD_MOVE = 0x50
MOVE_FMT = "<BBiiIIH"       # id, enable, center, amp, freq_mhz, prof_vel, goal_cur

RSP = {0x81: "PONG", 0x90: "TLM", 0xB0: "REG_VALUE", 0xD0: "MOVE", 0xEE: "ERR"}
ERRNAME = {1: "BAD_LEN", 2: "UNKNOWN_CMD", 3: "BUS", 5: "PARAM"}


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
                if b > 128:
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
    def __init__(self, port, baud=115200, timeout=1.0):   # baud vô nghĩa với USB CDC
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


def tlm_rows(p):
    """TLM -> [(t_ms, id, goal, pos, vel, cur), ...]"""
    t_ms, n = struct.unpack_from("<IB", p, 0)
    return [(t_ms,) + struct.unpack_from("<Biiih", p, 5 + 15 * i) for i in range(n)]


def show(t, p):
    name = RSP.get(t, f"0x{t:02X}")
    if t == 0x81:
        baud = struct.unpack_from("<I", p, 2 + p[1])[0] if len(p) >= 6 + p[1] else None
        return f"PONG   proto={p[0]} motors={list(p[2:2 + p[1]])} baud={baud}"
    if t == 0x90:
        return "\n".join(f"TLM    t={r[0]:8d}ms id={r[1]} goal={r[2]:6d} pos={r[3]:6d} "
                         f"vel={r[4]:5d} cur={r[5]:5d}" for r in tlm_rows(p))
    if t == 0xD0:
        mid, en, c, a, f, v, cur = struct.unpack(MOVE_FMT, p)
        return (f"MOVE   id={mid} enable={en} center={c} amp={a} freq={f / 1000:.3f}Hz "
                f"prof_vel={v} goal_cur={cur}")
    if t == 0xB0:
        mid, addr, ln = p[0], struct.unpack_from("<H", p, 1)[0], p[3]
        val = struct.unpack_from("<I", p, 4)[0]
        return f"REG    id={mid} addr={addr} len={ln} value={val}"
    if t == 0xEE:
        return f"ERR    {ERRNAME.get(p[0], p[0])} (lệnh 0x{p[1]:02X})"
    return f"{name}  {p.hex(' ')}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping")

    t = sub.add_parser("tlm");   t.add_argument("--rate", type=int, default=10)
    t.add_argument("--seconds", type=float, default=5.0)
    t.add_argument("--csv", help="ghi t_ms,id,goal,pos,vel,cur ra file thay vì in")

    m = sub.add_parser("move")
    m.add_argument("--id", type=int, default=2)
    m.add_argument("--center", type=int, default=2048, help="pulse")
    m.add_argument("--amp", type=int, default=0, help="pulse, 0 = đứng ở center")
    m.add_argument("--freq", type=float, default=0.0, help="Hz, tối đa 10")
    m.add_argument("--vel", type=int, default=0, help="Profile Velocity raw")
    m.add_argument("--cur", type=int, default=0xFFFF, help="Goal Current raw, mặc định = Current Limit")
    m.add_argument("--off", action="store_true", help="tắt torque")

    r = sub.add_parser("read")
    r.add_argument("--id", type=int, default=2)
    r.add_argument("--addr", type=int, required=True)
    r.add_argument("--len", type=int, default=4, choices=[1, 2, 4])

    w = sub.add_parser("write")
    w.add_argument("--id", type=int, default=2)
    w.add_argument("--addr", type=int, required=True)
    w.add_argument("--len", type=int, default=4, choices=[1, 2, 4])
    w.add_argument("--value", type=int, required=True)


    a = ap.parse_args()

    try:
        link = Link(a.port, a.baud)
    except serial.SerialException as e:
        sys.exit(f"Không mở được {a.port}: {e}")

    if a.cmd == "ping":
        t_, p = link.request(CMD_PING, want=0x81)
        if t_ is None:
            sys.exit("KHÔNG CÓ TRẢ LỜI. Board chưa nạp firmware mới, hoặc nhầm cổng\n"
                     "(xem `ls /dev/ttyACM*`, `dmesg | tail`).")
        print(show(t_, p))

    elif a.cmd == "tlm":
        link.send(CMD_TLM_RATE, struct.pack("<H", a.rate))
        frames = link.poll(a.seconds)
        link.send(CMD_TLM_RATE, struct.pack("<H", 0))
        if a.csv:
            with open(a.csv, "w") as fh:
                fh.write("t_ms,id,goal,pos,vel,cur\n")
                for t_, p in frames:
                    if t_ == 0x90:
                        fh.writelines(",".join(map(str, r)) + "\n" for r in tlm_rows(p))
            print(f"{sum(t_ == 0x90 for t_, _ in frames)} khung TLM -> {a.csv}")
        else:
            for t_, p in frames:
                print(show(t_, p))

    elif a.cmd == "move":
        pay = struct.pack(MOVE_FMT, a.id, 0 if a.off else 1, a.center, a.amp,
                          round(a.freq * 1000), a.vel, a.cur)
        t_, p = link.request(CMD_MOVE, pay)
        print(show(t_, p) if t_ else "KHÔNG CÓ TRẢ LỜI")

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


if __name__ == "__main__":
    main()
