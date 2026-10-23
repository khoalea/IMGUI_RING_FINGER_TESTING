#ifndef DXL_TRANSPORT_H_
#define DXL_TRANSPORT_H_

#include <stdint.h>
#include "dxl_types.h"

typedef struct {
    void *user;
    DXL2_Status (*write)(void *user, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
    DXL2_Status (*read_byte)(void *user, uint8_t *byte, uint32_t timeout_ms);
    void (*set_tx_mode)(void *user);
    void (*set_rx_mode)(void *user);
    uint32_t (*millis)(void *user);
} DXL2_Transport;

typedef struct {
    DXL2_Transport transport;
    uint8_t tx_buf[DXL_MAX_PACKET_SIZE];
    uint8_t rx_buf[DXL_MAX_PACKET_SIZE];
    uint32_t default_timeout_ms;
    uint8_t status_return_level;
} DXL2_Bus;

#endif
