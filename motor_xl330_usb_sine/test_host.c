/*
 * Host test cho pc_link.c + motion.c: FSM khung gói, lệnh MOVE, kẹp trần,
 * pha sin, vòng đệm RX, bỏ khung khi USB bận. HAL/USB/XL330 là bản giả.
 *
 * Chạy: ./run_host_tests.sh
 */

#include <stdio.h>
#include <string.h>

#include "User_code/pc_link.h"
#include "User_code/motion.h"

static int fails;
static void check(int cond, const char *what)
{
    printf("%s %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) fails++;
}

/* ==================== HAL / USB giả ==================== */
static uint32_t fake_tick;
uint32_t dxl_baud = 57600u;
uint32_t HAL_GetTick(void) { return fake_tick; }

static uint8_t  tx_last[256];
static uint16_t tx_last_len;
static int      tx_count;
static int      usb_busy;         /* 1 = giả gói trước chưa bay xong */

uint8_t PcLink_UsbSend(const uint8_t *buf, uint16_t len)
{
    if (usb_busy) { fake_tick++; return 1u; }   /* thời gian vẫn trôi khi chờ */
    memcpy(tx_last, buf, len);
    tx_last_len = len;
    tx_count++;
    return 0u;
}

/* ==================== XL330 giả ==================== */
XL330_StatusTypeDef XL330_ReadU8(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint8_t *v)
{ (void)h; (void)id; (void)a; *v = 0; return XL330_OK; }
XL330_StatusTypeDef XL330_ReadU16(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint16_t *v)
{ (void)h; (void)id; (void)a; *v = 0; return XL330_OK; }
XL330_StatusTypeDef XL330_ReadU32(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint32_t *v)
{ (void)h; (void)id; (void)a; *v = 0; return XL330_OK; }
XL330_StatusTypeDef XL330_WriteU8(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint8_t v)
{ (void)h; (void)id; (void)a; (void)v; return XL330_OK; }
XL330_StatusTypeDef XL330_WriteU16(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint16_t v)
{ (void)h; (void)id; (void)a; (void)v; return XL330_OK; }
XL330_StatusTypeDef XL330_WriteU32(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint32_t v)
{ (void)h; (void)id; (void)a; (void)v; return XL330_OK; }

/* ==================== tiện ích ==================== */
static XL330_HandleTypeDef dxl;

static uint16_t frame(uint8_t *out, uint8_t type, const uint8_t *p, uint8_t len)
{
    uint8_t cs = (uint8_t)(type + len);
    out[0] = 0xAA; out[1] = 0x55; out[2] = type; out[3] = len;
    for (uint8_t i = 0; i < len; i++) { out[4 + i] = p[i]; cs = (uint8_t)(cs + p[i]); }
    out[4 + len] = cs; out[5 + len] = 0x03;
    return (uint16_t)(6 + len);
}

static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }

static void send_move(uint8_t id, uint8_t en, int32_t center, int32_t amp,
                      uint32_t fmhz, uint32_t vel, uint16_t cur)
{
    uint8_t p[PCLINK_MOVE_LEN], f[64];
    p[0] = id; p[1] = en;
    put32(&p[2], (uint32_t)center); put32(&p[6], (uint32_t)amp);
    put32(&p[10], fmhz); put32(&p[14], vel);
    p[18] = (uint8_t)cur; p[19] = (uint8_t)(cur >> 8);
    uint16_t n = frame(f, PCLINK_CMD_MOVE, p, PCLINK_MOVE_LEN);
    /* Cắt làm hai mảnh như hai gói USB rời. */
    PcLink_RxBytes(f, 7);
    PcLink_Task();
    PcLink_RxBytes(f + 7, n - 7u);
    PcLink_Task();
}

int main(void)
{
    /* ---- Motion_Step: 1 Hz, biên độ 100 quanh 2048 ---- */
    MotionCh c = { .center = 2048, .amp = 100, .freq_mhz = 1000 };
    check(Motion_Step(&c, 250) == 2148, "sin 1/4 chu kỳ = center + amp");
    check(Motion_Step(&c, 250) == 2048, "sin 1/2 chu kỳ = center");
    check(Motion_Step(&c, 250) == 1948, "sin 3/4 chu kỳ = center - amp");
    MotionCh d = { .center = 0, .amp = 100, .freq_mhz = 1000 };
    check(Motion_Step(&d, 10250) == 100, "dt lớn (10.25 chu kỳ) vẫn đúng pha");
    check(d.phase >= 0.0f && d.phase < 6.2832f, "pha luôn nằm trong [0, 2pi)");

    /* ---- Dàn giả: 2 motor ID 2 và 5 ---- */
    static uint8_t ids[XL330_MAX_MOTORS];
    static XL330_State states[XL330_MAX_MOTORS];
    dxl.storage.ids = ids; dxl.storage.states = states;
    dxl.motor_count = 2;
    dxl.storage.ids[0] = 2; dxl.storage.ids[1] = 5;
    motion[0].cur_lim = 300; motion[0].vel_lim = 200;
    PcLink_Init(&dxl);

    /* MOVE hợp lệ, cur vượt trần, vel = 0 (vô hạn) */
    tx_count = 0;
    send_move(2, 1, 1000, 50, 500, 0, 999);
    check(tx_count == 1 && tx_last[2] == PCLINK_RSP_MOVE, "MOVE cắt hai mảnh -> trả RSP_MOVE");
    check(motion[0].enable == 1 && motion[0].center == 1000 && motion[0].amp == 50 &&
          motion[0].freq_mhz == 500 && motion[0].dirty == 1, "MOVE ghi đúng bảng motion[0]");
    check(motion[0].goal_cur == 300, "goal_cur kẹp về Current Limit");
    check(motion[0].prof_vel == 200, "prof_vel 0 (vô hạn) kẹp về Velocity Limit");
    check(tx_last[4 + 18] == (300 & 0xFF) && tx_last[4 + 19] == (300 >> 8), "trả về bản ĐÃ kẹp");

    /* Đang chạy, đổi biên độ: giữ pha. Tắt rồi bật lại: pha về 0. */
    motion[0].phase = 1.0f;
    send_move(2, 1, 1000, 80, 500, 100, 100);
    check(motion[0].phase == 1.0f, "đổi amp lúc đang chạy giữ nguyên pha");
    send_move(2, 0, 1000, 80, 500, 100, 100);
    send_move(2, 1, 1000, 80, 500, 100, 100);
    check(motion[0].phase == 0.0f, "bật lại từ trạng thái tắt -> pha 0");

    /* Lệnh sai */
    send_move(9, 1, 0, 10, 100, 0, 0);
    check(tx_last[2] == PCLINK_RSP_ERR && tx_last[4] == PCLINK_ERR_PARAM, "ID không có trên bus -> ERR_PARAM");
    send_move(5, 1, 0, 10, MOTION_MAX_FREQ_MHZ + 1, 0, 0);
    check(tx_last[2] == PCLINK_RSP_ERR && motion[1].enable == 0, "freq vượt trần -> ERR, không ghi bảng");
    send_move(5, 1, 0, -10, 100, 0, 0);
    check(tx_last[2] == PCLINK_RSP_ERR, "amp âm -> ERR");

    /* Checksum sai bị bỏ im lặng */
    {
        uint8_t f[16];
        uint16_t n = frame(f, PCLINK_CMD_PING, NULL, 0);
        f[n - 2] ^= 0xFF;
        tx_count = 0;
        PcLink_RxBytes(f, n); PcLink_Task();
        check(tx_count == 0, "sai checksum -> không trả lời");
        n = frame(f, PCLINK_CMD_PING, NULL, 0);
        PcLink_RxBytes(f, n); PcLink_Task();
        check(tx_count == 1 && tx_last[2] == PCLINK_RSP_PONG && tx_last[4] == PCLINK_PROTO_VER &&
              tx_last[5] == 2 && tx_last[6] == 2 && tx_last[7] == 5, "PING -> PONG ver, n, ids");
        check(tx_last[3] == 8 && (tx_last[8] | (tx_last[9] << 8) | (tx_last[10] << 16)) == 57600,
              "PONG kèm baud u32 sau danh sách id");
    }

    /* Telemetry: 2 motor = 5 + 2*15 = 35 byte payload */
    {
        uint8_t f[16], p[2] = { 10, 0 };
        uint16_t n = frame(f, PCLINK_CMD_TLM_RATE, p, 2);
        PcLink_RxBytes(f, n); PcLink_Task();
        motion[1].goal = -1234;
        fake_tick += 10; tx_count = 0;
        PcLink_Task();
        check(tx_count == 1 && tx_last[2] == PCLINK_RSP_TLM && tx_last[3] == 35, "TLM cả dàn trong 1 khung");
        int32_t g = (int32_t)(tx_last[4 + 5 + 15 + 1] | (tx_last[4 + 5 + 15 + 2] << 8) |
                              (tx_last[4 + 5 + 15 + 3] << 16) | ((uint32_t)tx_last[4 + 5 + 15 + 4] << 24));
        check(tx_last[4 + 5 + 15] == 5 && g == -1234, "TLM motor thứ 2: id và goal âm đúng");
        p[0] = 0;
        n = frame(f, PCLINK_CMD_TLM_RATE, p, 2);
        PcLink_RxBytes(f, n); PcLink_Task();
    }

    /* USB bận mãi: bỏ khung sau timeout, không treo */
    {
        uint8_t f[16];
        uint16_t n = frame(f, PCLINK_CMD_PING, NULL, 0);
        usb_busy = 1;
        PcLink_RxBytes(f, n); PcLink_Task();
        usb_busy = 0;
        check(pclink_tx_drop == 1, "USB bận quá timeout -> bỏ khung, đếm tx_drop");
    }

    /* Vòng đệm đầy: đếm byte bị vứt, không ghi đè */
    {
        static uint8_t junk[PCLINK_RX_BUF_SIZE + 10];
        memset(junk, 0x11, sizeof junk);
        PcLink_RxBytes(junk, sizeof junk);
        check(pclink_rx_overrun == 11, "RX đầy -> đếm overrun (ring giữ được SIZE-1)");
        PcLink_Task();
    }

    printf("\n%s (%d lỗi)\n", fails ? "CÓ TEST HỎNG" : "TẤT CẢ PASS", fails);
    return fails ? 1 : 0;
}
