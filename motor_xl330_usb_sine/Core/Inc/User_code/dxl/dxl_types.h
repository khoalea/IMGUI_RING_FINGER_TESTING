#ifndef DXL_TYPES_H_
#define DXL_TYPES_H_

#include <stdbool.h>
#include <stdint.h>
#include "dxl_config.h"

#define DXL_HEADER_0 0xFFu
#define DXL_HEADER_1 0xFFu
#define DXL_HEADER_2 0xFDu
#define DXL_RESERVED 0x00u
#define DXL_BROADCAST_ID 0xFEu
#define DXL_STATUS_INSTRUCTION 0x55u

#define DXL_INST_PING 0x01u
#define DXL_INST_READ 0x02u
#define DXL_INST_WRITE 0x03u
#define DXL_INST_REG_WRITE 0x04u
#define DXL_INST_ACTION 0x05u
#define DXL_INST_FACTORY_RESET 0x06u
#define DXL_INST_REBOOT 0x08u
#define DXL_INST_CLEAR 0x10u
#define DXL_INST_CONTROL_TABLE_BACKUP 0x20u
#define DXL_INST_SYNC_READ 0x82u
#define DXL_INST_SYNC_WRITE 0x83u
#define DXL_INST_FAST_SYNC_READ 0x8Au
#define DXL_INST_BULK_READ 0x92u
#define DXL_INST_BULK_WRITE 0x93u
#define DXL_INST_FAST_BULK_READ 0x9Au

#define DXL_ERRBIT_ALERT 0x80u
#define DXL_ERRCODE_MASK 0x7Fu
#define DXL_ERRCODE_RESULT_FAIL 0x01u
#define DXL_ERRCODE_INSTRUCTION 0x02u
#define DXL_ERRCODE_CRC 0x03u
#define DXL_ERRCODE_DATA_RANGE 0x04u
#define DXL_ERRCODE_DATA_LENGTH 0x05u
#define DXL_ERRCODE_DATA_LIMIT 0x06u
#define DXL_ERRCODE_ACCESS 0x07u

typedef enum {
    DXL2_OK = 0,
    DXL2_ERR_TIMEOUT,
    DXL2_ERR_CRC,
    DXL2_ERR_ID_MISMATCH,
    DXL2_ERR_BAD_PACKET,
    DXL2_ERR_STATUS_ERROR,
    DXL2_ERR_UNSUPPORTED,
    DXL2_ERR_BAD_PARAM,
    DXL2_ERR_NO_RESPONSE_EXPECTED,
    DXL2_ERR_TRANSPORT,
    DXL2_ERR_BUFFER_TOO_SMALL
} DXL2_Status;

typedef struct {
    uint8_t id;
    uint8_t error;
    uint8_t alert;
    uint8_t error_code;
    uint16_t param_len;
    uint8_t params[DXL_MAX_PARAM_LEN];
} DXL2_StatusPacket;

typedef struct {
    uint8_t id;
    uint16_t model_number;
    uint8_t firmware_version;
} DXL2_DeviceInfo;

#endif
