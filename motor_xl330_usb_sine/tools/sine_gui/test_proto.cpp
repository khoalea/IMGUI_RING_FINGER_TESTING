// Kiểm proto.hpp khớp layout firmware (cùng số liệu với test_host.c). Chạy: ctest hoặc ./test_proto
#include "proto.hpp"
#include <cstdio>

static int fails;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

int main() {
  using namespace pcl;
  std::uint8_t pay[kMoveLen], f[64];
  const Move m{2, 1, -1000, 50, 500, 200, 300};
  pack_move(pay, m);
  const std::size_t n = build(f, CMD_MOVE, pay, kMoveLen);
  CHECK(n == 26 && f[2] == CMD_MOVE && f[3] == 20 && f[25] == 0x03);
  CHECK(pay[18] == (300 & 0xFF) && pay[19] == (300 >> 8));          // goal_cur ở offset 18, như test_host.c

  Decoder d; int got = 0;
  for (std::size_t i = 0; i < n; ++i) got += d.feed(f[i]);           // một lèo
  CHECK(got == 1 && d.type == CMD_MOVE && d.len == 20);
  const Move e = unpack_move(d.pay);
  CHECK(e.id == 2 && e.enable == 1 && e.center == -1000 && e.amp == 50 && e.freq_mhz == 500 &&
        e.prof_vel == 200 && e.goal_cur == 300);

  f[n - 2] ^= 0xFF; got = 0;                                         // sai checksum -> bỏ, đếm bad
  for (std::size_t i = 0; i < n; ++i) got += d.feed(f[i]);
  CHECK(got == 0 && d.bad == 1);

  // TLM 2 motor, rác 0xAA phía trước, ghép từng byte
  std::uint8_t t[5 + 2 * kTlmRec]{}, *p = t;
  p = wr(p, std::uint32_t{1234}); p = wr(p, std::uint8_t{2});
  p = wr(p, std::uint8_t{2}); p = wr(p, std::int32_t{2048}); p = wr(p, std::int32_t{2050});
  p = wr(p, std::int32_t{-3}); p = wr(p, std::int16_t{-40});
  p = wr(p, std::uint8_t{5}); p = wr(p, std::int32_t{-1234}); p = wr(p, std::int32_t{0});
  p = wr(p, std::int32_t{0}); wr(p, std::int16_t{0});
  std::uint8_t g[80] = {0xAA, 0xAA};
  const std::size_t gn = 2 + build(g + 2, RSP_TLM, t, sizeof t);
  got = 0;
  for (std::size_t i = 0; i < gn; ++i) got += d.feed(g[i]);
  CHECK(got == 1 && d.type == RSP_TLM && d.len == 35);
  const TlmRec a = unpack_tlm(d.pay + 5), b = unpack_tlm(d.pay + 5 + kTlmRec);
  CHECK(rd<std::uint32_t>(d.pay) == 1234 && d.pay[4] == 2);
  CHECK(a.id == 2 && a.goal == 2048 && a.pos == 2050 && a.vel == -3 && a.cur == -40);
  CHECK(b.id == 5 && b.goal == -1234);

  std::printf("%s (%d lỗi)\n", fails ? "CÓ TEST HỎNG" : "TẤT CẢ PASS", fails);
  return fails != 0;
}
