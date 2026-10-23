/**
 ******************************************************************************
 * @file    pc_link.c
 * @brief   Giao thức nhị phân PC <-> STM32 qua USB CDC.
 *
 * RX: ISR USB chép byte vào vòng đệm, PcLink_Task() rút ra trong vòng lặp
 * chính — thay cho DMA vòng của bản USART2, FSM khung gói giữ nguyên.
 *
 * TX: PcLink_UsbSend() chỉ nhận khi gói trước đã bay xong. Thử lại tối đa
 * PCLINK_TX_TIMEOUT_MS rồi bỏ khung, đếm vào pclink_tx_drop. Cổng chưa mở
 * phía PC thì host không lấy gói, nên không được chờ vô hạn.
 ******************************************************************************
 */

#include "User_code/pc_link.h"
#include "User_code/motion.h"
#include <string.h>

/* ---------------- Trạng thái module ---------------- */

static XL330_HandleTypeDef *link_dxl;

/* Vòng đệm RX. ISR chỉ ghi rx_head, vòng lặp chỉ ghi rx_tail: một bên ghi
 * một biến 16-bit, không cần khoá trên M3. */
static uint8_t           rx_ring[PCLINK_RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
volatile uint32_t        pclink_rx_overrun;
uint32_t                 pclink_tx_drop;

typedef enum {
    ST_H1 = 0,
    ST_H2,
    ST_TYPE,
    ST_LEN,
    ST_PAYLOAD,
    ST_CS,
    ST_FTR
} RxState_t;

static RxState_t rx_state;
static uint8_t   rx_type;
static uint8_t   rx_len;
static uint8_t   rx_idx;
static uint8_t   rx_cs;
static uint8_t   rx_payload[PCLINK_MAX_PAYLOAD];

static uint8_t   tx_frame[6u + PCLINK_MAX_PAYLOAD];

static uint16_t  tlm_period_ms;   /* 0 = tắt stream */
static uint32_t  tlm_last_tick;

/* ---------------- Đóng gói số little-endian ---------------- */

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* ---------------- Gửi khung ---------------- */

static void link_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t  cs;
    uint8_t  i;
    uint32_t t0;

    if (len > PCLINK_MAX_PAYLOAD) {
        return;
    }

    tx_frame[0] = PCLINK_HDR1;
    tx_frame[1] = PCLINK_HDR2;
    tx_frame[2] = type;
    tx_frame[3] = len;
    if ((len > 0u) && (payload != NULL)) {
        memcpy(&tx_frame[4], payload, len);
    }

    cs = (uint8_t)(type + len);
    for (i = 0u; i < len; ++i) {
        cs = (uint8_t)(cs + tx_frame[4u + i]);
    }
    tx_frame[4u + len] = cs;
    tx_frame[5u + len] = PCLINK_FTR;

    t0 = HAL_GetTick();
    while (PcLink_UsbSend(tx_frame, (uint16_t)(6u + len)) != 0u) {
        if ((HAL_GetTick() - t0) > PCLINK_TX_TIMEOUT_MS) {
            pclink_tx_drop++;
            return;
        }
    }
}

static void link_send_err(uint8_t code, uint8_t echo_type)
{
    uint8_t p[2];
    p[0] = code;
    p[1] = echo_type;
    link_send(PCLINK_RSP_ERR, p, 2u);
}

/* ---------------- Các lệnh ---------------- */

static void send_pong(void)
{
    uint8_t p[2u + XL330_MAX_MOTORS + 4u];
    uint8_t i;
    uint8_t n = (link_dxl != NULL) ? link_dxl->motor_count : 0u;

    p[0] = PCLINK_PROTO_VER;
    p[1] = n;
    for (i = 0u; i < n; ++i) {
        p[2u + i] = link_dxl->storage.ids[i];
    }
    put_u32(&p[2u + n], dxl_baud);
    link_send(PCLINK_RSP_PONG, p, (uint8_t)(6u + n));
}

/* Cả dàn trong một khung, để PC vẽ được goal và pos trên cùng trục thời gian. */
static void send_telemetry(void)
{
    uint8_t p[5u + 15u * XL330_MAX_MOTORS];
    uint8_t i;
    uint8_t n = link_dxl->motor_count;
    uint8_t *q = &p[5];

    put_u32(&p[0], HAL_GetTick());
    p[4] = n;
    for (i = 0u; i < n; ++i) {
        const XL330_State *st = &link_dxl->storage.states[i];
        q[0] = link_dxl->storage.ids[i];
        put_u32(&q[1], (uint32_t)motion[i].goal);
        put_u32(&q[5], (uint32_t)st->present_position);
        put_u32(&q[9], (uint32_t)st->present_velocity);
        put_u16(&q[13], (uint16_t)st->present_current);
        q += 15;
    }
    link_send(PCLINK_RSP_TLM, p, (uint8_t)(5u + 15u * n));
}

static void send_move(uint8_t idx)
{
    const MotionCh *c = &motion[idx];
    uint8_t p[PCLINK_MOVE_LEN];

    p[0] = link_dxl->storage.ids[idx];
    p[1] = c->enable;
    put_u32(&p[2],  (uint32_t)c->center);
    put_u32(&p[6],  (uint32_t)c->amp);
    put_u32(&p[10], c->freq_mhz);
    put_u32(&p[14], c->prof_vel);
    put_u16(&p[18], c->goal_cur);
    link_send(PCLINK_RSP_MOVE, p, PCLINK_MOVE_LEN);
}

static int8_t find_idx(uint8_t id)
{
    uint8_t i;
    for (i = 0u; i < link_dxl->motor_count; ++i) {
        if (link_dxl->storage.ids[i] == id) {
            return (int8_t)i;
        }
    }
    return -1;
}

/*
 * Chỉ ghi vào bảng motion[]; main.c đẩy xuống bus ở tick kế tiếp.
 * Trả về bản ĐÃ KẸP để PC thấy đúng thứ motor sẽ nhận.
 */
static void handle_move(const uint8_t *p)
{
    int8_t   ix   = find_idx(p[0]);
    int32_t  amp  = (int32_t)get_u32(&p[6]);
    uint32_t freq = get_u32(&p[10]);
    MotionCh *c;

    if ((ix < 0) || (amp < 0) || (freq > MOTION_MAX_FREQ_MHZ)) {
        link_send_err(PCLINK_ERR_PARAM, PCLINK_CMD_MOVE);
        return;
    }
    c = &motion[ix];

    /* Bật từ trạng thái tắt thì chạy lại từ pha 0, tức là từ center.
     * Đang chạy mà chỉ đổi amp/freq thì giữ pha để quỹ đạo không giật. */
    if (p[1] && !c->enable) {
        c->phase = 0.0f;
    }
    c->enable   = p[1] ? 1u : 0u;
    c->center   = (int32_t)get_u32(&p[2]);
    c->amp      = amp;
    c->freq_mhz = freq;
    c->prof_vel = get_u32(&p[14]);
    c->goal_cur = get_u16(&p[18]);

    /* Vượt Limit thì motor trả Data Range Error và BỎ cả lệnh, nên kẹp ở đây. */
    if ((c->vel_lim != 0u) && ((c->prof_vel == 0u) || (c->prof_vel > c->vel_lim))) {
        c->prof_vel = c->vel_lim;
    }
    if ((c->cur_lim != 0u) && (c->goal_cur > c->cur_lim)) {
        c->goal_cur = c->cur_lim;
    }
    c->dirty = 1u;

    send_move((uint8_t)ix);
}

/*
 * Đọc control table của XL330. Không chặn địa chỉ nào — giống bản USART2,
 * phần cảnh báo vùng EEPROM nằm ở phía PC.
 */
static void handle_reg_read(const uint8_t *payload)
{
    uint8_t  id   = payload[0];
    uint16_t addr = get_u16(&payload[1]);
    uint8_t  len  = payload[3];
    uint32_t value = 0u;
    XL330_StatusTypeDef s;
    uint8_t p[8];

    switch (len) {
        case 1u: {
            uint8_t v = 0u;
            s = XL330_ReadU8(link_dxl, id, addr, &v);
            value = v;
            break;
        }
        case 2u: {
            uint16_t v = 0u;
            s = XL330_ReadU16(link_dxl, id, addr, &v);
            value = v;
            break;
        }
        case 4u:
            s = XL330_ReadU32(link_dxl, id, addr, &value);
            break;
        default:
            link_send_err(PCLINK_ERR_PARAM, PCLINK_CMD_REG_READ);
            return;
    }

    if (s != XL330_OK) {
        link_send_err(PCLINK_ERR_BUS, PCLINK_CMD_REG_READ);
        return;
    }

    p[0] = id;
    put_u16(&p[1], addr);
    p[3] = len;
    put_u32(&p[4], value);
    link_send(PCLINK_RSP_REG_VALUE, p, 8u);
}

static void handle_reg_write(const uint8_t *payload)
{
    uint8_t  id    = payload[0];
    uint16_t addr  = get_u16(&payload[1]);
    uint8_t  len   = payload[3];
    uint32_t value = get_u32(&payload[4]);
    XL330_StatusTypeDef s;

    switch (len) {
        case 1u: s = XL330_WriteU8(link_dxl, id, addr, (uint8_t)value);   break;
        case 2u: s = XL330_WriteU16(link_dxl, id, addr, (uint16_t)value); break;
        case 4u: s = XL330_WriteU32(link_dxl, id, addr, value);           break;
        default:
            link_send_err(PCLINK_ERR_PARAM, PCLINK_CMD_REG_WRITE);
            return;
    }

    if (s != XL330_OK) {
        link_send_err(PCLINK_ERR_BUS, PCLINK_CMD_REG_WRITE);
        return;
    }

    /* Đọc lại để PC thấy đúng thứ motor thực sự nhận. */
    handle_reg_read(payload);
}

static void handle_frame(void)
{
    switch (rx_type) {

        case PCLINK_CMD_PING:
            send_pong();
            break;

        case PCLINK_CMD_TLM_RATE:
            if (rx_len != 2u) {
                link_send_err(PCLINK_ERR_BAD_LEN, rx_type);
                break;
            }
            tlm_period_ms = get_u16(rx_payload);
            tlm_last_tick = HAL_GetTick();
            send_pong();
            break;

        case PCLINK_CMD_REG_READ:
            if (rx_len != 4u) {
                link_send_err(PCLINK_ERR_BAD_LEN, rx_type);
                break;
            }
            handle_reg_read(rx_payload);
            break;

        case PCLINK_CMD_REG_WRITE:
            if (rx_len != 8u) {
                link_send_err(PCLINK_ERR_BAD_LEN, rx_type);
                break;
            }
            handle_reg_write(rx_payload);
            break;

        case PCLINK_CMD_MOVE:
            if (rx_len != PCLINK_MOVE_LEN) {
                link_send_err(PCLINK_ERR_BAD_LEN, rx_type);
                break;
            }
            handle_move(rx_payload);
            break;

        default:
            link_send_err(PCLINK_ERR_UNKNOWN_CMD, rx_type);
            break;
    }
}

/* ---------------- FSM khung gói ---------------- */

static void process_byte(uint8_t b)
{
    switch (rx_state) {

        case ST_H1:
            if (b == PCLINK_HDR1) {
                rx_state = ST_H2;
            }
            break;

        case ST_H2:
            /* 0xAA 0xAA: byte thứ hai có thể là header của khung kế tiếp. */
            rx_state = (b == PCLINK_HDR2) ? ST_TYPE
                     : ((b == PCLINK_HDR1) ? ST_H2 : ST_H1);
            break;

        case ST_TYPE:
            rx_type  = b;
            rx_state = ST_LEN;
            break;

        case ST_LEN:
            if (b > PCLINK_MAX_PAYLOAD) {
                rx_state = ST_H1;   /* gói rác, bỏ */
                break;
            }
            rx_len   = b;
            rx_idx   = 0u;
            rx_cs    = (uint8_t)(rx_type + rx_len);
            rx_state = (rx_len == 0u) ? ST_CS : ST_PAYLOAD;
            break;

        case ST_PAYLOAD:
            rx_payload[rx_idx++] = b;
            rx_cs = (uint8_t)(rx_cs + b);
            if (rx_idx >= rx_len) {
                rx_state = ST_CS;
            }
            break;

        case ST_CS:
            rx_state = (b == rx_cs) ? ST_FTR : ST_H1;
            break;

        case ST_FTR:
            if (b == PCLINK_FTR) {
                handle_frame();
            }
            rx_state = ST_H1;
            break;

        default:
            rx_state = ST_H1;
            break;
    }
}

/* ---------------- API ---------------- */

void PcLink_RxBytes(const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint16_t next;

    for (i = 0u; i < len; ++i) {
        next = (uint16_t)((rx_head + 1u) % PCLINK_RX_BUF_SIZE);
        if (next == rx_tail) {
            pclink_rx_overrun += len - i;
            return;
        }
        rx_ring[rx_head] = buf[i];
        rx_head = next;
    }
}

void PcLink_Init(XL330_HandleTypeDef *hxl)
{
    link_dxl      = hxl;
    rx_state      = ST_H1;
    tlm_period_ms = 0u;
    tlm_last_tick = HAL_GetTick();
}

void PcLink_Task(void)
{
    if (link_dxl == NULL) {
        return;
    }

    while (rx_tail != rx_head) {
        process_byte(rx_ring[rx_tail]);
        rx_tail = (uint16_t)((rx_tail + 1u) % PCLINK_RX_BUF_SIZE);
    }

    if (tlm_period_ms != 0u) {
        uint32_t now = HAL_GetTick();
        if ((now - tlm_last_tick) >= tlm_period_ms) {
            tlm_last_tick = now;
            send_telemetry();
        }
    }
}
