/**
 ******************************************************************************
 * @file    pc_link.h
 * @brief   Giao thức nhị phân giữa PC và STM32 cho tool calib (USART2).
 *
 * Khung gói giữ đúng kiểu của comm_protocol.c cũ, thêm TYPE + LEN để một
 * đường truyền chở được nhiều loại lệnh:
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
#define PCLINK_PROTO_VER        1u

#define PCLINK_RX_BUF_SIZE      256u   /* buffer DMA vòng                  */
#define PCLINK_MAX_PAYLOAD      64u    /* payload dài nhất đang dùng là 48 */

/* ---------------- PC -> STM32 ---------------- */
#define PCLINK_CMD_PING         0x01u  /* len 0                            */
#define PCLINK_CMD_TLM_RATE     0x10u  /* u16 period_ms, 0 = tắt stream    */
#define PCLINK_CMD_FT_CMD       0x20u  /* u8 cmd, i32 weight_g             */
#define PCLINK_CMD_FT_GET       0x21u  /* len 0                            */
#define PCLINK_CMD_REG_READ     0x30u  /* u8 id, u16 addr, u8 len          */
#define PCLINK_CMD_REG_WRITE    0x31u  /* u8 id, u16 addr, u8 len, u32 val */
#define PCLINK_CMD_CAL_GET      0x40u  /* len 0                            */
#define PCLINK_CMD_CAL_SET      0x41u  /* CalibrationData, chỉ ghi vào RAM */
#define PCLINK_CMD_CAL_SAVE     0x42u  /* len 0, đẩy bản RAM xuống flash   */

/* ---------------- STM32 -> PC ---------------- */
#define PCLINK_RSP_PONG         0x81u  /* u8 proto_ver, u8 motor_id        */
#define PCLINK_RSP_TLM          0x90u  /* i32 pos, i32 vel, i32 cur, u8 st */
#define PCLINK_RSP_FT_STATE     0xA0u  /* khối kết quả bài test lực        */
#define PCLINK_RSP_REG_VALUE    0xB0u  /* u8 id, u16 addr, u8 len, u32 val */
#define PCLINK_RSP_CAL_DATA     0xC0u  /* CalibrationData                  */
#define PCLINK_RSP_ERR          0xEEu  /* u8 code, u8 echo_type            */

/* ---------------- Mã lỗi trong PCLINK_RSP_ERR ---------------- */
#define PCLINK_ERR_BAD_LEN      1u     /* LEN không khớp loại lệnh         */
#define PCLINK_ERR_UNKNOWN_CMD  2u     /* TYPE lạ                          */
#define PCLINK_ERR_BUS          3u     /* lỗi bus Dynamixel                */
#define PCLINK_ERR_FLASH        4u     /* ghi flash thất bại               */
#define PCLINK_ERR_PARAM        5u     /* tham số sai (vd reg len != 1/2/4)*/

/* ---------------- API ---------------- */
void PcLink_Init(UART_HandleTypeDef *huart, XL330_HandleTypeDef *hxl);

/*
 * Gọi càng thường xuyên càng tốt. Rút byte từ buffer DMA vòng, chạy FSM
 * khung gói, và gửi telemetry định kỳ nếu PC đã bật stream.
 * An toàn khi gọi lồng trong vòng chờ của bài test lực.
 */
void PcLink_Task(void);

/*
 * Hai móc nối tới bài test lực, cài đặt trong main.c (nơi có biến ftest).
 * Khi FTEST_ENABLE = 0 thì main.c cài bản rỗng.
 */
extern void    PcLink_ForceCommand(uint8_t cmd, int32_t weight_g);
extern uint8_t PcLink_ForceStatePack(uint8_t *buf);  /* trả về số byte đã ghi */

#endif /* INC_PC_LINK_H_ */
