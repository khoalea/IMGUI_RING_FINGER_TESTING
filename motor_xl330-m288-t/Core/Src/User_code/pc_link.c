/**
 ******************************************************************************
 * @file    pc_link.c
 * @brief   Giao thức nhị phân PC <-> STM32 cho tool calib.
 *
 * RX dùng DMA vòng rồi rút byte trong PcLink_Task(), đúng pattern của
 * comm_protocol.c cũ vì nó chạy tốt.
 *
 * TX dùng HAL_UART_Transmit chặn thay vì DMA: gói dài nhất 54 byte, ở
 * 115200 baud mất ~4.7ms. Đổi lại không phải nuôi hàng đợi TX và không bao
 * giờ mất gói vì DMA đang bận. Telemetry 20Hz tốn khoảng 6% CPU.
 ******************************************************************************
 */

#include "User_code/pc_link.h"
#include "User_code/calibration.h"
#include <string.h>

/* ---------------- Trạng thái module ---------------- */

static UART_HandleTypeDef  *link_uart;
static XL330_HandleTypeDef *link_dxl;

static uint8_t  rx_dma_buf[PCLINK_RX_BUF_SIZE];
static uint16_t rx_read_pos;

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

/* Bản làm việc của dữ liệu calib. CAL_SET sửa bản này, CAL_SAVE mới ghi flash. */
static CalibrationData cal_ram;

/* ---------------- Đóng gói số little-endian ---------------- */

static void put_u8(uint8_t *p, uint8_t v)   { p[0] = v; }
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
    uint8_t cs;
    uint8_t i;

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

    HAL_UART_Transmit(link_uart, tx_frame, (uint16_t)(6u + len), 100u);
}

static void link_send_err(uint8_t code, uint8_t echo_type)
{
    uint8_t p[2];
    p[0] = code;
    p[1] = echo_type;
    link_send(PCLINK_RSP_ERR, p, 2u);
}

/* ---------------- Các lệnh ---------------- */

static void send_telemetry(void)
{
    const XL330_State *st;
    uint8_t p[13];
    uint8_t id = 0u;

    if ((link_dxl == NULL) || (link_dxl->motor_count == 0u)) {
        return;
    }
    id = link_dxl->storage.motors[0].id;
    st = XL330_GetCachedState(link_dxl, id);
    if (st == NULL) {
        return;
    }

    put_u32(&p[0], (uint32_t)st->present_position);
    put_u32(&p[4], (uint32_t)st->present_velocity);
    put_u32(&p[8], (uint32_t)(int32_t)st->present_current);
    put_u8(&p[12], (uint8_t)link_dxl->last_report.status);
    link_send(PCLINK_RSP_TLM, p, 13u);
}

static void send_pong(void)
{
    uint8_t p[2];
    p[0] = PCLINK_PROTO_VER;
    p[1] = ((link_dxl != NULL) && (link_dxl->motor_count > 0u))
           ? link_dxl->storage.motors[0].id
           : 0u;
    link_send(PCLINK_RSP_PONG, p, 2u);
}

static void send_force_state(void)
{
    uint8_t p[PCLINK_MAX_PAYLOAD];
    uint8_t len = PcLink_ForceStatePack(p);

    if (len > 0u) {
        link_send(PCLINK_RSP_FT_STATE, p, len);
    }
}

static void send_cal_data(void)
{
    link_send(PCLINK_RSP_CAL_DATA, (const uint8_t *)&cal_ram, (uint8_t)sizeof(cal_ram));
}

/*
 * Đọc control table của XL330. Không chặn địa chỉ nào — theo thiết kế đã
 * chốt, phần cảnh báo vùng EEPROM do GUI đảm nhiệm (bắt xác nhận hai lần).
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

    put_u8(&p[0], id);
    put_u16(&p[1], addr);
    put_u8(&p[3], len);
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

        case PCLINK_CMD_FT_CMD:
            if (rx_len != 5u) {
                link_send_err(PCLINK_ERR_BAD_LEN, rx_type);
                break;
            }
            PcLink_ForceCommand(rx_payload[0], (int32_t)get_u32(&rx_payload[1]));
            send_force_state();
            break;

        case PCLINK_CMD_FT_GET:
            send_force_state();
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

        case PCLINK_CMD_CAL_GET:
            send_cal_data();
            break;

        case PCLINK_CMD_CAL_SET:
            if (rx_len != (uint8_t)sizeof(cal_ram)) {
                link_send_err(PCLINK_ERR_BAD_LEN, rx_type);
                break;
            }
            memcpy(&cal_ram, rx_payload, sizeof(cal_ram));
            send_cal_data();
            break;

        case PCLINK_CMD_CAL_SAVE:
            if (Calib_Write(&cal_ram) == 0u) {
                link_send_err(PCLINK_ERR_FLASH, rx_type);
                break;
            }
            /* Đọc lại từ flash để PC thấy đúng thứ đã nằm trên board. */
            (void)Calib_Read(&cal_ram);
            send_cal_data();
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

void PcLink_Init(UART_HandleTypeDef *huart, XL330_HandleTypeDef *hxl)
{
    link_uart = huart;
    link_dxl  = hxl;

    rx_read_pos   = 0u;
    rx_state      = ST_H1;
    tlm_period_ms = 0u;
    tlm_last_tick = HAL_GetTick();

    /* CRC hỏng thì Calib_Read đã tự nạp giá trị mặc định an toàn. */
    (void)Calib_Read(&cal_ram);

    HAL_UART_Receive_DMA(link_uart, rx_dma_buf, PCLINK_RX_BUF_SIZE);
}

void PcLink_Task(void)
{
    uint16_t head;

    if (link_uart == NULL) {
        return;
    }

    head = (uint16_t)(PCLINK_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(link_uart->hdmarx));
    while (rx_read_pos != head) {
        process_byte(rx_dma_buf[rx_read_pos]);
        rx_read_pos++;
        if (rx_read_pos >= PCLINK_RX_BUF_SIZE) {
            rx_read_pos = 0u;
        }
    }

    if (tlm_period_ms != 0u) {
        uint32_t now = HAL_GetTick();
        if ((now - tlm_last_tick) >= tlm_period_ms) {
            tlm_last_tick = now;
            send_telemetry();
        }
    }
}
