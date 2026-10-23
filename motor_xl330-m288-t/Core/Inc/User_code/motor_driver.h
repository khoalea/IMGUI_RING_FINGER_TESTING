/*
 * motor_driver.h
 *
 *  Created on: Feb 5, 2026
 *      Author: vinbui
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include <stdint.h>
#include <string.h>
#include "main.h"

// ---- Protocol 2.0 Definitions ----
#define MOTOR_READ   				0x02
#define MOTOR_WRITE  				0x03
#define MOTOR_PING   				0x01
#define MOTOR_REG_WRITE				0x04
#define MOTOR_ACTIOM                0x05
#define MOTOR_FACTORY_RESET			0x06
#define MOTOR_REBOOT				0x08
#define MOTOR_CLEAR					0x10
#define MOTOR_CONTROL_TABL_BACKUP	0x20
#define MOTOR_STATUS				0x55
#define MOTOR_SYNC_READ				0x82
#define MOTOR_SYNC_WRITE			0x83
#define MOTOR_FAST_SYNC_READ		0x8A
#define MOTOR_BULK_READ				0x92
#define MOTOR_BULK_WRITE			0x93
#define MOTOR_FAST_BULK_READ		0x9A
// ---- Operating Modes (Address 11) ----
#define DXL_MODE_CURRENT                0
#define DXL_MODE_VELOCITY               1
#define DXL_MODE_POSITION               3
#define DXL_MODE_EXT_POSITION           4
#define DXL_MODE_CURRENT_BASED_POS      5
#define DXL_MODE_PWM                    16

#define DXL_PACK(addr, len)  (((uint32_t)(len) << 16) | (uint16_t)(addr))

// Torque Control
#define DXL_ADDR_TORQUE_ENABLE          DXL_PACK(64, 1)
#define DXL_ADDR_LED                    DXL_PACK(65, 1)
#define DXL_ADDR_STATUS_RETURN_LEVEL    DXL_PACK(68, 1)
#define DXL_ADDR_HARDWARE_ERROR_STATUS  DXL_PACK(70, 1)

// PID & Feedforward Gains
#define DXL_ADDR_VELOCITY_I_GAIN        DXL_PACK(76, 2)
#define DXL_ADDR_VELOCITY_P_GAIN        DXL_PACK(78, 2)
#define DXL_ADDR_POSITION_D_GAIN        DXL_PACK(80, 2)
#define DXL_ADDR_POSITION_I_GAIN        DXL_PACK(82, 2)
#define DXL_ADDR_POSITION_P_GAIN        DXL_PACK(84, 2)
#define DXL_ADDR_FEEDFORWARD_2ND_GAIN   DXL_PACK(88, 2)
#define DXL_ADDR_FEEDFORWARD_1ST_GAIN   DXL_PACK(90, 2)

// Bus Watchdog
#define DXL_ADDR_BUS_WATCHDOG           DXL_PACK(98, 1)

// Goals
#define DXL_ADDR_GOAL_PWM               DXL_PACK(100, 2)
#define DXL_ADDR_GOAL_CURRENT           DXL_PACK(102, 2)
#define DXL_ADDR_GOAL_VELOCITY          DXL_PACK(104, 4)
#define DXL_ADDR_PROFILE_ACCELERATION   DXL_PACK(108, 4)
#define DXL_ADDR_PROFILE_VELOCITY       DXL_PACK(112, 4)
#define DXL_ADDR_GOAL_POSITION          DXL_PACK(116, 4)

// Real-time Info (Read-only usually, but defining size helps)
#define DXL_ADDR_REALTIME_TICK          DXL_PACK(120, 2)
#define DXL_ADDR_MOVING                 DXL_PACK(122, 1)
#define DXL_ADDR_MOVING_STATUS          DXL_PACK(123, 1)
#define DXL_ADDR_PRESENT_PWM            DXL_PACK(124, 2)
#define DXL_ADDR_PRESENT_CURRENT        DXL_PACK(126, 2)
#define DXL_ADDR_PRESENT_VELOCITY       DXL_PACK(128, 4)
#define DXL_ADDR_PRESENT_POSITION       DXL_PACK(132, 4)
#define DXL_ADDR_VELOCITY_TRAJECTORY    DXL_PACK(136, 4)
#define DXL_ADDR_POSITION_TRAJECTORY    DXL_PACK(140, 4)
#define DXL_ADDR_PRESENT_INPUT_VOLTAGE  DXL_PACK(144, 2)
#define DXL_ADDR_PRESENT_TEMPERATURE    DXL_PACK(146, 1)
// ---- Data Structures ----

// The structure containing the parsed data from the motor
typedef struct {
    uint8_t id;
    uint8_t error;
    uint8_t param_len;      // Number of data bytes received
    uint8_t params[48];     // Buffer to hold the data (Max typical size)
} DXL_StatusPacket;

// The wrapper structure containing the success flag and the data
typedef struct {
    uint8_t success;        // 1 = Success, 0 = Timeout/CRC Error
    DXL_StatusPacket data;  // The actual packet content
} DXL_Response;

typedef struct {
    int16_t present_current;   // Address 126 (2 bytes), Unit: 1mA (XL330)
    int32_t present_velocity;  // Address 128 (4 bytes), Unit: 0.229 rpm (XL330)
    int32_t present_position;  // Address 132 (4 bytes), Unit: 1 pulse
} DXL_RealTimeData;
// ---- Function Prototypes ----

// Core utility functions
uint16_t dxl_crc_update(uint16_t crc, uint8_t *data, uint16_t size);
uint16_t dxl_byte_stuff(uint8_t *src, uint16_t len, uint8_t *dst);
uint16_t dxl_destuff(uint8_t *src, uint16_t src_len, uint8_t *dst, uint16_t dst_max);
uint16_t dxl_build_packet(uint8_t *out,
                          uint8_t id,
                          uint8_t instruction,
                          const uint8_t *params,
                          uint16_t param_len);
// High-level transaction function (Send CMD -> Wait -> Parse -> Return Struct)
uint8_t dxl_motor_init(UART_HandleTypeDef *huart);
DXL_Response dxl_transmit_receive(uint8_t id, uint8_t instruction, uint8_t *params, uint16_t param_len, uint16_t timeout_ms);

// Helper to set current (example usage of the high-level function)
uint8_t dxl_set_torque(uint8_t id, uint8_t enable);
uint8_t dxl_set_operating_mode(uint8_t id, uint8_t mode);
DXL_Response dxl_read_present_data(uint8_t id, DXL_RealTimeData *out_data);

// Professional Generic Write Function
uint8_t dxl_write(uint8_t id, uint32_t addr_info, uint32_t value);
#endif /* INC_MOTOR_DRIVER_H_ */
