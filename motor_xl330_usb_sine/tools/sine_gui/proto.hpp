// pclink framing (docs/protocol.md): 0xAA 0x55 | TYPE | LEN | PAYLOAD | CS | 0x03, little-endian.
// Header thuần, không I/O: test_proto.cpp kiểm riêng được.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace pcl {

enum : std::uint8_t {
  CMD_PING = 0x01, CMD_TLM_RATE = 0x10, CMD_MOVE = 0x50,
  RSP_PONG = 0x81, RSP_TLM = 0x90, RSP_REG = 0xB0, RSP_MOVE = 0xD0, RSP_ERR = 0xEE,
};
inline constexpr std::size_t kMaxPayload = 128, kMoveLen = 20, kTlmRec = 15;

struct Move {                       // = payload MOVE, cả lệnh lẫn echo đã kẹp
  std::uint8_t id, enable; std::int32_t center, amp; std::uint32_t freq_mhz, prof_vel; std::uint16_t goal_cur;
};
struct TlmRec { std::uint8_t id; std::int32_t goal, pos, vel; std::int16_t cur; };

template <class T> inline T rd(const std::uint8_t* p) { T v; std::memcpy(&v, p, sizeof v); return v; }  // host LE
template <class T> inline std::uint8_t* wr(std::uint8_t* p, T v) { std::memcpy(p, &v, sizeof v); return p + sizeof v; }

// Trả về số byte đã ghi vào out (cần >= len + 6).
inline std::size_t build(std::uint8_t* out, std::uint8_t type, const std::uint8_t* pay, std::uint8_t len) {
  std::uint8_t cs = static_cast<std::uint8_t>(type + len);
  out[0] = 0xAA; out[1] = 0x55; out[2] = type; out[3] = len;
  for (std::size_t i = 0; i < len; ++i) { out[4 + i] = pay[i]; cs = static_cast<std::uint8_t>(cs + pay[i]); }
  out[4 + len] = cs; out[5 + len] = 0x03;
  return len + 6u;
}

inline void pack_move(std::uint8_t* p, const Move& m) {
  p = wr(p, m.id); p = wr(p, m.enable); p = wr(p, m.center); p = wr(p, m.amp);
  p = wr(p, m.freq_mhz); p = wr(p, m.prof_vel); wr(p, m.goal_cur);
}
inline Move unpack_move(const std::uint8_t* p) {
  return {p[0], p[1], rd<std::int32_t>(p + 2), rd<std::int32_t>(p + 6),
          rd<std::uint32_t>(p + 10), rd<std::uint32_t>(p + 14), rd<std::uint16_t>(p + 18)};
}
inline TlmRec unpack_tlm(const std::uint8_t* p) {   // p trỏ vào bản ghi thứ i: payload + 5 + 15*i
  return {p[0], rd<std::int32_t>(p + 1), rd<std::int32_t>(p + 5), rd<std::int32_t>(p + 9), rd<std::int16_t>(p + 13)};
}

// FSM gương của process_byte() trong pc_link.c. feed() trả true khi vừa xong một khung hợp lệ.
struct Decoder {
  enum St : std::uint8_t { H1, H2, TYPE, LEN, PAY, CS, FT } st = H1;
  std::uint8_t type = 0, len = 0, n = 0, cs = 0, pay[kMaxPayload]{};
  std::uint32_t bad = 0;            // khung bị bỏ vì sai checksum/footer/len

  bool feed(std::uint8_t b) {
    switch (st) {
      case H1: if (b == 0xAA) st = H2; break;
      case H2: st = b == 0x55 ? TYPE : (b == 0xAA ? H2 : H1); break;
      case TYPE: type = b; st = LEN; break;
      case LEN:
        if (b > kMaxPayload) { ++bad; st = H1; break; }
        len = b; n = 0; cs = static_cast<std::uint8_t>(type + b); st = b ? PAY : CS; break;
      case PAY: pay[n++] = b; cs = static_cast<std::uint8_t>(cs + b); if (n >= len) st = CS; break;
      case CS: if (b == cs) st = FT; else { ++bad; st = H1; } break;
      case FT: st = H1; if (b == 0x03) return true; ++bad; break;
    }
    return false;
  }
};

} // namespace pcl
