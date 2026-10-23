/*
 * Host test cho pc_link.c — chạy FSM khung gói và bộ điều phối lệnh thật,
 * với HAL và driver XL330 thay bằng bản giả.
 *
 * Biên dịch bằng run_host_tests.sh.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "User_code/pc_link.h"
#include "User_code/calibration.h"

/* ==================== Bộ đếm test ==================== */
static int fails;
static void check(int cond, const char *what)
{
    printf("%s %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) fails++;
}

/* ==================== HAL giả ==================== */

/* Buffer DMA vòng mà pc_link.c đọc: ta điều khiển CNDTR để giả con trỏ ghi. */
static DMA_Channel_TypeDef fake_dma_ch;
static DMA_HandleTypeDef   fake_dma;
static UART_HandleTypeDef  fake_uart;

static uint8_t *dma_target;      /* buffer pc_link đưa cho HAL_UART_Receive_DMA */
static uint16_t dma_size;

/* Các khung pc_link đã phát ra */
#define TXCAP 32
static uint8_t  tx_buf[TXCAP][80];
static uint16_t tx_len[TXCAP];
static int      tx_count;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *p,
                                    uint16_t size, uint32_t timeout)
{
    (void)h; (void)timeout;
    if (tx_count < TXCAP && size <= sizeof(tx_buf[0])) {
        memcpy(tx_buf[tx_count], p, size);
        tx_len[tx_count] = size;
        tx_count++;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t size)
{
    (void)h;
    dma_target = p;
    dma_size   = size;
    fake_dma_ch.CNDTR = size;   /* chưa nhận byte nào */
    return HAL_OK;
}

static uint32_t fake_tick;
uint32_t HAL_GetTick(void) { return fake_tick; }

/* ==================== calibration giả ==================== */
static CalibrationData flash_image;
static int flash_writes;
static int flash_should_fail;

uint8_t Calib_Read(CalibrationData *dest) { *dest = flash_image; return 1u; }
uint8_t Calib_Write(CalibrationData *src)
{
    if (flash_should_fail) return 0u;
    src->crc = 0xC0FFEEu;
    flash_image = *src;
    flash_writes++;
    return 1u;
}

/* ==================== driver XL330 giả ==================== */
static uint16_t last_addr;
static uint8_t  last_id;
static uint32_t reg_value = 1234u;
static XL330_StatusTypeDef reg_status = XL330_OK;
static int      write_count;

XL330_StatusTypeDef XL330_ReadU8(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint8_t *v)
{ (void)h; last_id=id; last_addr=a; *v=(uint8_t)reg_value; return reg_status; }
XL330_StatusTypeDef XL330_ReadU16(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint16_t *v)
{ (void)h; last_id=id; last_addr=a; *v=(uint16_t)reg_value; return reg_status; }
XL330_StatusTypeDef XL330_ReadU32(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint32_t *v)
{ (void)h; last_id=id; last_addr=a; *v=reg_value; return reg_status; }
XL330_StatusTypeDef XL330_WriteU8(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint8_t v)
{ (void)h;(void)id;(void)a; reg_value=v; write_count++; return reg_status; }
XL330_StatusTypeDef XL330_WriteU16(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint16_t v)
{ (void)h;(void)id;(void)a; reg_value=v; write_count++; return reg_status; }
XL330_StatusTypeDef XL330_WriteU32(XL330_HandleTypeDef *h, uint8_t id, uint16_t a, uint32_t v)
{ (void)h;(void)id;(void)a; reg_value=v; write_count++; return reg_status; }

static XL330_State fake_state;
const XL330_State *XL330_GetCachedState(const XL330_HandleTypeDef *h, uint8_t id)
{ (void)h; (void)id; return &fake_state; }

/* ==================== móc nối bài test lực giả ==================== */
static uint8_t force_cmd_seen;
static int32_t force_weight_seen;
static int     force_cmd_calls;

void PcLink_ForceCommand(uint8_t cmd, int32_t weight_g)
{ force_cmd_seen = cmd; force_weight_seen = weight_g; force_cmd_calls++; }

uint8_t PcLink_ForceStatePack(uint8_t *buf)
{
    memset(buf, 0, 56);
    buf[0] = 1u;                       /* busy      */
    buf[1] = 1u;                       /* ready     */
    buf[12] = 0x2Au;                   /* avg = 42  */
    return 56u;
}

/* ==================== Tiện ích ==================== */

/* Đẩy byte vào buffer DMA giả rồi cho pc_link rút ra. */
static void feed(const uint8_t *bytes, uint16_t n)
{
    static uint16_t wr;
    for (uint16_t i = 0; i < n; ++i) {
        dma_target[wr] = bytes[i];
        wr = (uint16_t)((wr + 1u) % dma_size);
        fake_dma_ch.CNDTR = (uint32_t)(dma_size - wr);
    }
    PcLink_Task();
}

static uint8_t checksum(uint8_t type, const uint8_t *p, uint8_t len)
{
    uint8_t cs = (uint8_t)(type + len);
    for (uint8_t i = 0; i < len; ++i) cs = (uint8_t)(cs + p[i]);
    return cs;
}

/* Gửi một khung hợp lệ vào pc_link. */
static void send_frame(uint8_t type, const uint8_t *p, uint8_t len)
{
    uint8_t f[80];
    f[0]=PCLINK_HDR1; f[1]=PCLINK_HDR2; f[2]=type; f[3]=len;
    if (len) memcpy(&f[4], p, len);
    f[4+len] = checksum(type, p, len);
    f[5+len] = PCLINK_FTR;
    tx_count = 0;
    feed(f, (uint16_t)(6u + len));
}

/* Khung phát ra thứ i có đúng type và độ dài payload không? */
static int tx_is(int i, uint8_t type, uint8_t len)
{
    if (i >= tx_count) return 0;
    if (tx_len[i] != (uint16_t)(6u + len)) return 0;
    if (tx_buf[i][0] != PCLINK_HDR1 || tx_buf[i][1] != PCLINK_HDR2) return 0;
    if (tx_buf[i][2] != type || tx_buf[i][3] != len) return 0;
    if (tx_buf[i][4+len] != checksum(type, &tx_buf[i][4], len)) return 0;
    if (tx_buf[i][5+len] != PCLINK_FTR) return 0;
    return 1;
}

static uint32_t tx_u32(int i, int off)
{
    const uint8_t *p = &tx_buf[i][4 + off];
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}

/* ==================== Kịch bản ==================== */

int main(void)
{
    XL330_HandleTypeDef dxl;
    memset(&dxl, 0, sizeof(dxl));

    fake_dma.Instance = &fake_dma_ch;
    fake_uart.hdmarx  = &fake_dma;

    PcLink_Init(&fake_uart, &dxl);
    check(dma_target != NULL, "Init xin DMA nhận vòng");

    /* ---- PING ---- */
    send_frame(PCLINK_CMD_PING, NULL, 0);
    check(tx_count == 1 && tx_is(0, PCLINK_RSP_PONG, 2), "PING tra ve PONG");
    check(tx_buf[0][4] == PCLINK_PROTO_VER, "PONG mang so hieu giao thuc");

    /* ---- checksum sai thì phải im lặng ---- */
    {
        uint8_t bad[] = {PCLINK_HDR1, PCLINK_HDR2, PCLINK_CMD_PING, 0, 0xFF, PCLINK_FTR};
        tx_count = 0;
        feed(bad, sizeof(bad));
        check(tx_count == 0, "Checksum sai bi bo, khong tra loi");
    }

    /* ---- footer sai thì phải im lặng ---- */
    {
        uint8_t bad[] = {PCLINK_HDR1, PCLINK_HDR2, PCLINK_CMD_PING, 0,
                         (uint8_t)(PCLINK_CMD_PING + 0), 0x99};
        tx_count = 0;
        feed(bad, sizeof(bad));
        check(tx_count == 0, "Footer sai bi bo");
    }

    /* ---- khung tốt ngay sau khung rác vẫn nhận được ---- */
    send_frame(PCLINK_CMD_PING, NULL, 0);
    check(tx_count == 1 && tx_is(0, PCLINK_RSP_PONG, 2), "Dong bo lai duoc sau khung rac");

    /* ---- byte rác chen trước header ---- */
    {
        uint8_t junk[] = {0x00, 0xFF, 0xAA, 0xAA};   /* 0xAA 0xAA rồi mới tới khung */
        tx_count = 0;
        feed(junk, sizeof(junk));
        uint8_t rest[] = {PCLINK_HDR2, PCLINK_CMD_PING, 0,
                          (uint8_t)PCLINK_CMD_PING, PCLINK_FTR};
        feed(rest, sizeof(rest));
        check(tx_count == 1 && tx_is(0, PCLINK_RSP_PONG, 2), "0xAA 0xAA lien tiep van bat duoc header");
    }

    /* ---- LEN sai cho lệnh có độ dài cố định ---- */
    {
        uint8_t p[3] = {1,2,3};
        send_frame(PCLINK_CMD_REG_READ, p, 3);
        check(tx_is(0, PCLINK_RSP_ERR, 2) && tx_buf[0][4] == PCLINK_ERR_BAD_LEN,
              "REG_READ sai LEN tra ve ERR_BAD_LEN");
    }

    /* ---- TYPE lạ ---- */
    send_frame(0x77, NULL, 0);
    check(tx_is(0, PCLINK_RSP_ERR, 2) && tx_buf[0][4] == PCLINK_ERR_UNKNOWN_CMD,
          "TYPE la tra ve ERR_UNKNOWN_CMD");

    /* ---- REG_READ ---- */
    {
        uint8_t p[4] = {5u, 132u & 0xFF, 0u, 4u};   /* id 5, addr 132, len 4 */
        reg_value = 2039u; reg_status = XL330_OK;
        send_frame(PCLINK_CMD_REG_READ, p, 4);
        check(tx_is(0, PCLINK_RSP_REG_VALUE, 8), "REG_READ tra ve REG_VALUE");
        check(last_id == 5u && last_addr == 132u, "REG_READ chuyen dung id va dia chi");
        check(tx_u32(0, 4) == 2039u, "REG_VALUE mang dung gia tri");
    }

    /* ---- REG_READ độ dài không hợp lệ ---- */
    {
        uint8_t p[4] = {5u, 132u, 0u, 3u};          /* len 3 không hợp lệ */
        send_frame(PCLINK_CMD_REG_READ, p, 4);
        check(tx_is(0, PCLINK_RSP_ERR, 2) && tx_buf[0][4] == PCLINK_ERR_PARAM,
              "REG_READ len=3 tra ve ERR_PARAM");
    }

    /* ---- lỗi bus ---- */
    {
        uint8_t p[4] = {5u, 132u, 0u, 4u};
        reg_status = XL330_BUS_ERROR;
        send_frame(PCLINK_CMD_REG_READ, p, 4);
        check(tx_is(0, PCLINK_RSP_ERR, 2) && tx_buf[0][4] == PCLINK_ERR_BUS,
              "Loi bus tra ve ERR_BUS");
        reg_status = XL330_OK;
    }

    /* ---- REG_WRITE: ghi xong đọc lại ---- */
    {
        uint8_t p[8] = {5u, 116u, 0u, 4u, 0x00, 0x08, 0x00, 0x00}; /* addr 116 = 2048 */
        write_count = 0;
        send_frame(PCLINK_CMD_REG_WRITE, p, 8);
        check(write_count == 1, "REG_WRITE goi driver dung mot lan");
        check(tx_is(0, PCLINK_RSP_REG_VALUE, 8), "REG_WRITE tra ve gia tri doc lai");
        check(tx_u32(0, 4) == 2048u, "Gia tri doc lai dung");
    }

    /* ---- FT_CMD ---- */
    {
        uint8_t p[5] = {2u, 0xF4, 0x01, 0x00, 0x00};  /* TARE, weight 500 */
        force_cmd_calls = 0;
        send_frame(PCLINK_CMD_FT_CMD, p, 5);
        check(force_cmd_calls == 1 && force_cmd_seen == 2u && force_weight_seen == 500,
              "FT_CMD chuyen dung lenh va khoi luong");
        check(tx_is(0, PCLINK_RSP_FT_STATE, 56), "FT_CMD tra ve FT_STATE 56 byte");
    }

    /* ---- FT_GET ---- */
    send_frame(PCLINK_CMD_FT_GET, NULL, 0);
    check(tx_is(0, PCLINK_RSP_FT_STATE, 56), "FT_GET tra ve FT_STATE");
    check(tx_u32(0, 12) == 42u, "FT_STATE giu dung offset truong");

    /* ---- CalibrationData đúng 48 byte như GUI giả định ---- */
    check(sizeof(CalibrationData) == 48u, "CalibrationData dung 48 byte");

    /* ---- CAL_GET / CAL_SET / CAL_SAVE ---- */
    send_frame(PCLINK_CMD_CAL_GET, NULL, 0);
    check(tx_is(0, PCLINK_RSP_CAL_DATA, 48), "CAL_GET tra ve 48 byte");

    {
        uint8_t p[48];
        float rate = 347.5f;
        memset(p, 0, sizeof(p));
        memcpy(&p[32], &rate, 4);                  /* motor.torque_rate */
        flash_writes = 0;
        send_frame(PCLINK_CMD_CAL_SET, p, 48);
        check(tx_is(0, PCLINK_RSP_CAL_DATA, 48), "CAL_SET tra ve ban RAM");
        check(flash_writes == 0, "CAL_SET khong dong vao flash");

        send_frame(PCLINK_CMD_CAL_SAVE, NULL, 0);
        check(flash_writes == 1, "CAL_SAVE ghi flash dung mot lan");
        check(tx_is(0, PCLINK_RSP_CAL_DATA, 48), "CAL_SAVE tra ve ban doc lai");
        float back;
        memcpy(&back, &tx_buf[0][4+32], 4);
        check(back == rate, "Gia tri torque_rate di tron ven qua CAL_SET/SAVE");
    }

    /* ---- ghi flash hỏng ---- */
    flash_should_fail = 1;
    send_frame(PCLINK_CMD_CAL_SAVE, NULL, 0);
    check(tx_is(0, PCLINK_RSP_ERR, 2) && tx_buf[0][4] == PCLINK_ERR_FLASH,
          "Ghi flash hong tra ve ERR_FLASH");
    flash_should_fail = 0;

    /* ---- telemetry theo chu kỳ ---- */
    {
        uint8_t p[2] = {50u, 0u};                  /* 50 ms */
        fake_state.present_position = 2039;
        fake_state.present_velocity = -7;
        fake_state.present_current  = 87;
        dxl.motor_count = 1u;
        dxl.storage.motors = (XL330_MotorSlot *)calloc(1, sizeof(XL330_MotorSlot));
        dxl.storage.motors[0].id = 5u;

        fake_tick = 1000u;
        send_frame(PCLINK_CMD_TLM_RATE, p, 2);
        check(tx_is(0, PCLINK_RSP_PONG, 2), "TLM_RATE duoc xac nhan");

        tx_count = 0;
        fake_tick = 1020u;  PcLink_Task();
        check(tx_count == 0, "Chua toi chu ky thi chua gui telemetry");

        fake_tick = 1060u;  PcLink_Task();
        check(tx_count == 1 && tx_is(0, PCLINK_RSP_TLM, 13), "Toi chu ky thi gui TLM 13 byte");
        check(tx_u32(0, 0) == 2039u, "TLM mang dung vi tri");
        check((int32_t)tx_u32(0, 4) == -7, "TLM mang dung van toc am");
        check(tx_u32(0, 8) == 87u, "TLM mang dung dong dien");

        /* tắt stream */
        p[0] = 0u;
        send_frame(PCLINK_CMD_TLM_RATE, p, 2);
        tx_count = 0;
        fake_tick += 5000u; PcLink_Task();
        check(tx_count == 0, "TLM_RATE = 0 thi ngung stream");
    }

    printf(fails ? "\n%d test HONG\n" : "\nTat ca test PASS\n", fails);
    return fails ? 1 : 0;
}
