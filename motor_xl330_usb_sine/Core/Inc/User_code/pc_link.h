/**
 ******************************************************************************
 * @file    pc_link.h
 * @brief   Giao thức nhị phân PC <-> STM32 qua USB CDC (/dev/ttyACM*).
 *
 * Cùng khung gói với bản USART2 của motor_xl330-m288-t:
 *
 *     0xAA 0x55 | TYPE | LEN | PAYLOAD[LEN] | CS | 0x03
 *
 * CS = (TYPE + LEN + tổng các byte payload) & 0xFF  — cộng dồn 8 bit.
 * Mọi số nhiều byte trong payload là little-endian.
 ******************************************************************************
 */

#ifndef INC_PC_LINK_H_
#define INC_PC_LINK_H_

#include "main.h"
#include "User_code/dxl/dxl_xl330_driver.h"
#include <stdint.h>

/* ---------------- Khung gói ---------------- */
#define PCLINK_HDR1             0xAAu
#define PCLINK_HDR2             0x55u
#define PCLINK_FTR              0x03u
#define PCLINK_PROTO_VER        3u     /* 2 = bản USB + MOVE, 3 = PONG có baud */

#define PCLINK_RX_BUF_SIZE      256u   /* vòng đệm ISR USB -> vòng lặp    */
#define PCLINK_MAX_PAYLOAD      128u   /* TLM 8 motor = 5 + 8*15 = 125    */
#define PCLINK_TX_TIMEOUT_MS    2u     /* chờ gói USB trước bay xong      */

/* ---------------- PC -> STM32 ---------------- */
#define PCLINK_CMD_PING         0x01u  /* len 0                            */
#define PCLINK_CMD_TLM_RATE     0x10u  /* u16 period_ms, 0 = tắt stream    */
#define PCLINK_CMD_REG_READ     0x30u  /* u8 id, u16 addr, u8 len          */
#define PCLINK_CMD_REG_WRITE    0x31u  /* u8 id, u16 addr, u8 len, u32 val */
#define PCLINK_CMD_MOVE         0x50u  /* xem PCLINK_MOVE_LEN              */

/* MOVE: u8 id, u8 enable, i32 center, i32 amp, u32 freq_mhz,
 *       u32 prof_vel, u16 goal_cur                         = 20 byte */
#define PCLINK_MOVE_LEN         20u

/* ---------------- STM32 -> PC ---------------- */
#define PCLINK_RSP_PONG         0x81u  /* u8 ver, u8 n, u8 id[n], u32 baud */
#define PCLINK_RSP_TLM          0x90u  /* u32 t_ms, u8 n, n x {u8 id,
                                        * i32 goal, i32 pos, i32 vel,
                                        * i16 cur}                        */
#define PCLINK_RSP_REG_VALUE    0xB0u  /* u8 id, u16 addr, u8 len, u32 val */
#define PCLINK_RSP_MOVE         0xD0u  /* MOVE sau khi kẹp, cùng layout    */
#define PCLINK_RSP_ERR          0xEEu  /* u8 code, u8 echo_type            */

/* ---------------- Mã lỗi trong PCLINK_RSP_ERR ---------------- */
#define PCLINK_ERR_BAD_LEN      1u
#define PCLINK_ERR_UNKNOWN_CMD  2u
#define PCLINK_ERR_BUS          3u
#define PCLINK_ERR_PARAM        5u     /* id lạ, freq quá trần, len sai   */

/* ---------------- API ---------------- */
void PcLink_Init(XL330_HandleTypeDef *hxl);

/* Gọi trong vòng lặp chính: rút byte, chạy FSM, gửi telemetry định kỳ. */
void PcLink_Task(void);

/* Gọi từ CDC_Receive_FS (ngữ cảnh ISR USB). Chỉ chép vào vòng đệm. */
void PcLink_RxBytes(const uint8_t *buf, uint32_t len);

/* Byte bị vứt vì vòng đệm RX đầy / khung TX bị bỏ vì USB bận. */
extern volatile uint32_t pclink_rx_overrun;
extern uint32_t          pclink_tx_drop;
extern uint32_t          dxl_baud;          /* main.c: baud bus dò được lúc boot, 0 = không thấy motor */

/* Chép khung vào buffer TX của USB rồi phát. Cài trong usbd_cdc_if.c, thay
 * bằng bản giả khi test host. 0 = đã nhận, 1 = gói trước chưa bay xong
 * hoặc chưa enumerate. */
extern uint8_t PcLink_UsbSend(const uint8_t *buf, uint16_t len);

#endif /* INC_PC_LINK_H_ */
