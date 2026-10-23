#ifndef DXL_PROTOCOL2_H_
#define DXL_PROTOCOL2_H_

#include "dxl_transport.h"

uint16_t dxl2_crc_update(uint16_t crc, const uint8_t *data, uint16_t size);
DXL2_Status dxl2_byte_stuff(const uint8_t *src, uint16_t len, uint8_t *dst, uint16_t dst_max, uint16_t *out_len);
DXL2_Status dxl2_destuff(const uint8_t *src, uint16_t len, uint8_t *dst, uint16_t dst_max, uint16_t *out_len);
DXL2_Status dxl2_build_packet(uint8_t *out, uint16_t out_max, uint8_t id, uint8_t instruction, const uint8_t *params, uint16_t param_len, uint16_t *out_len);
DXL2_Status dxl2_parse_status_packet(const uint8_t *raw, uint16_t raw_len, uint8_t expected_id, bool stuffed, DXL2_StatusPacket *packet);
DXL2_Status dxl2_bus_init(DXL2_Bus *bus, const DXL2_Transport *transport);
DXL2_Status dxl2_tx_instruction(DXL2_Bus *bus, uint8_t id, uint8_t instruction, const uint8_t *params, uint16_t param_len, bool expect_response, DXL2_StatusPacket *response);
DXL2_Status dxl2_ping(DXL2_Bus *bus, uint8_t id, DXL2_DeviceInfo *info);
DXL2_Status dxl2_read(DXL2_Bus *bus, uint8_t id, uint16_t addr, uint16_t len, uint8_t *out);
DXL2_Status dxl2_write(DXL2_Bus *bus, uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
DXL2_Status dxl2_write_u8(DXL2_Bus *bus, uint8_t id, uint16_t addr, uint8_t value);
DXL2_Status dxl2_write_u16(DXL2_Bus *bus, uint8_t id, uint16_t addr, uint16_t value);
DXL2_Status dxl2_write_u32(DXL2_Bus *bus, uint8_t id, uint16_t addr, uint32_t value);
DXL2_Status dxl2_read_u8(DXL2_Bus *bus, uint8_t id, uint16_t addr, uint8_t *out);
DXL2_Status dxl2_read_u16(DXL2_Bus *bus, uint8_t id, uint16_t addr, uint16_t *out);
DXL2_Status dxl2_read_u32(DXL2_Bus *bus, uint8_t id, uint16_t addr, uint32_t *out);
DXL2_Status dxl2_reg_write(DXL2_Bus *bus, uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);
DXL2_Status dxl2_action(DXL2_Bus *bus, uint8_t id);

typedef struct {
    uint8_t id;
    const uint8_t *data;
} DXL2_SyncWriteItem;

typedef struct {
    uint8_t id;
    uint16_t addr;
    uint16_t len;
    uint8_t *data;
} DXL2_BulkReadItem;

typedef struct {
    uint8_t id;
    uint16_t addr;
    uint16_t len;
    const uint8_t *data;
} DXL2_BulkWriteItem;

typedef struct {
    uint8_t id;
    uint8_t error;
    uint8_t alert;
    uint8_t error_code;
    uint16_t param_len;
    uint8_t params[DXL_MAX_PARAM_LEN];
} DXL2_GroupResult;

DXL2_Status dxl2_sync_read(DXL2_Bus *bus, uint16_t addr, uint16_t len, const uint8_t *ids, uint8_t id_count, uint8_t *out, uint16_t out_stride, DXL2_GroupResult *results);
DXL2_Status dxl2_sync_write(DXL2_Bus *bus, uint16_t addr, uint16_t len, const DXL2_SyncWriteItem *items, uint8_t item_count);
DXL2_Status dxl2_bulk_read(DXL2_Bus *bus, DXL2_BulkReadItem *items, uint8_t item_count, DXL2_GroupResult *results);
DXL2_Status dxl2_bulk_write(DXL2_Bus *bus, const DXL2_BulkWriteItem *items, uint8_t item_count);
DXL2_Status dxl2_fast_sync_read(DXL2_Bus *bus, uint16_t addr, uint16_t len, const uint8_t *ids, uint8_t id_count, uint8_t *out, uint16_t out_stride, DXL2_GroupResult *result);
DXL2_Status dxl2_fast_bulk_read(DXL2_Bus *bus, DXL2_BulkReadItem *items, uint8_t item_count, DXL2_GroupResult *result);

#if DXL_ENABLE_SHORT_API_ALIASES
#define DXL_Status DXL2_Status
#define DXL_StatusPacket DXL2_StatusPacket
#define DXL_Bus DXL2_Bus
#define DXL_SyncWriteItem DXL2_SyncWriteItem
#define DXL_BulkReadItem DXL2_BulkReadItem
#define DXL_BulkWriteItem DXL2_BulkWriteItem
#define DXL_GroupResult DXL2_GroupResult
#define dxl_tx_instruction dxl2_tx_instruction
#define dxl_ping dxl2_ping
#define dxl_read dxl2_read
#define dxl_write dxl2_write
#define dxl_sync_read dxl2_sync_read
#define dxl_sync_write dxl2_sync_write
#define dxl_bulk_read dxl2_bulk_read
#define dxl_bulk_write dxl2_bulk_write
#define dxl_fast_sync_read dxl2_fast_sync_read
#define dxl_fast_bulk_read dxl2_fast_bulk_read
#endif

#endif
