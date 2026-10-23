/*
 * motor_driver.c
 *
 * Compatibility wrappers for the reusable Dynamixel Protocol 2.0 library.
 */

#include "User_code/motor_driver.h"
#include "User_code/dxl/dxl_protocol2.h"
#include "User_code/dxl/dxl_stm32_hal.h"
#include <stddef.h>

extern UART_HandleTypeDef huart1;

static DXL2_Bus motor_bus;
static DXL2_STM32HAL_Context motor_hal_ctx;
static uint8_t motor_bus_ready = 0u;

static uint16_t dxl_addr(uint32_t addr_info)
{
    return (uint16_t)(addr_info & 0xFFFFu);
}

static uint8_t dxl_len(uint32_t addr_info)
{
    return (uint8_t)((addr_info >> 16) & 0xFFu);
}

static DXL2_Status motor_bus_init_if_needed(void)
{
    if (motor_bus_ready) {
        return DXL2_OK;
    }

    DXL2_Transport transport;
    DXL2_Status status = dxl2_stm32hal_transport_init(&transport, &motor_hal_ctx, &huart1);
    if (status != DXL2_OK) {
        return status;
    }

    status = dxl2_bus_init(&motor_bus, &transport);
    if (status == DXL2_OK) {
        motor_bus_ready = 1u;
    }
    return status;
}

uint8_t dxl_motor_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return 0u;
    }

    DXL2_Transport transport;
    DXL2_Status status = dxl2_stm32hal_transport_init(&transport, &motor_hal_ctx, huart);
    if (status != DXL2_OK) {
        motor_bus_ready = 0u;
        return 0u;
    }

    status = dxl2_bus_init(&motor_bus, &transport);
    motor_bus_ready = (status == DXL2_OK) ? 1u : 0u;
    return motor_bus_ready;
}

uint16_t dxl_crc_update(uint16_t crc, uint8_t *data, uint16_t size)
{
    return dxl2_crc_update(crc, data, size);
}

uint16_t dxl_byte_stuff(uint8_t *src, uint16_t len, uint8_t *dst)
{
    uint16_t out_len = 0u;
    if (dxl2_byte_stuff(src, len, dst, DXL_MAX_PACKET_SIZE, &out_len) != DXL2_OK) {
        return 0u;
    }
    return out_len;
}

uint16_t dxl_destuff(uint8_t *src, uint16_t src_len, uint8_t *dst, uint16_t dst_max)
{
    uint16_t out_len = 0u;
    if (dxl2_destuff(src, src_len, dst, dst_max, &out_len) != DXL2_OK) {
        return 0u;
    }
    return out_len;
}

uint16_t dxl_build_packet(uint8_t *out,
                          uint8_t id,
                          uint8_t instruction,
                          const uint8_t *params,
                          uint16_t param_len)
{
    uint16_t out_len = 0u;
    if (dxl2_build_packet(out, DXL_MAX_PACKET_SIZE, id, instruction, params, param_len, &out_len) != DXL2_OK) {
        return 0u;
    }
    return out_len;
}

DXL_Response dxl_transmit_receive(uint8_t id,
                                  uint8_t instruction,
                                  uint8_t *params,
                                  uint16_t param_len,
                                  uint16_t timeout_ms)
{
    DXL_Response response = {0};
    if (motor_bus_init_if_needed() != DXL2_OK) {
        return response;
    }

    uint32_t old_timeout = motor_bus.default_timeout_ms;
    motor_bus.default_timeout_ms = timeout_ms;

    DXL2_StatusPacket packet;
    DXL2_Status status = dxl2_tx_instruction(&motor_bus, id, instruction, params, param_len, true, &packet);

    motor_bus.default_timeout_ms = old_timeout;

    if (status == DXL2_OK || status == DXL2_ERR_STATUS_ERROR) {
        response.success = 1u;
        response.data.id = packet.id;
        response.data.error = packet.error;
        response.data.param_len = (packet.param_len > sizeof(response.data.params)) ?
                                  sizeof(response.data.params) : (uint8_t)packet.param_len;
        memcpy(response.data.params, packet.params, response.data.param_len);
    }

    return response;
}

uint8_t dxl_set_torque(uint8_t id, uint8_t enable)
{
    if (motor_bus_init_if_needed() != DXL2_OK) {
        return 0u;
    }
    DXL2_Status status = dxl2_write_u8(&motor_bus, id, 64u, enable ? 1u : 0u);
    return (status == DXL2_OK) ? 1u : 0u;
}

uint8_t dxl_set_operating_mode(uint8_t id, uint8_t mode)
{
    if (motor_bus_init_if_needed() != DXL2_OK) {
        return 0u;
    }

    if (!dxl_set_torque(id, 0u)) {
        return 0u;
    }

    if (dxl2_write_u8(&motor_bus, id, 11u, mode) != DXL2_OK) {
        return 0u;
    }

    uint8_t readback = 0u;
    if (dxl2_read_u8(&motor_bus, id, 11u, &readback) != DXL2_OK) {
        return 0u;
    }

    return (readback == mode) ? 1u : 0u;
}

DXL_Response dxl_read_present_data(uint8_t id, DXL_RealTimeData *out_data)
{
    DXL_Response response = {0};
    if (motor_bus_init_if_needed() != DXL2_OK || out_data == NULL) {
        return response;
    }

    uint8_t raw[10];
    DXL2_Status status = dxl2_read(&motor_bus, id, 126u, sizeof(raw), raw);
    if (status != DXL2_OK) {
        return response;
    }

    response.success = 1u;
    response.data.id = id;
    response.data.error = 0u;
    response.data.param_len = sizeof(raw);
    memcpy(response.data.params, raw, sizeof(raw));

    out_data->present_current = (int16_t)(raw[0] | ((uint16_t)raw[1] << 8));
    out_data->present_velocity = (int32_t)((uint32_t)raw[2] |
                                           ((uint32_t)raw[3] << 8) |
                                           ((uint32_t)raw[4] << 16) |
                                           ((uint32_t)raw[5] << 24));
    out_data->present_position = (int32_t)((uint32_t)raw[6] |
                                           ((uint32_t)raw[7] << 8) |
                                           ((uint32_t)raw[8] << 16) |
                                           ((uint32_t)raw[9] << 24));
    return response;
}

uint8_t dxl_write(uint8_t id, uint32_t addr_info, uint32_t value)
{
    if (motor_bus_init_if_needed() != DXL2_OK) {
        return 0u;
    }

    uint16_t addr = dxl_addr(addr_info);
    uint8_t len = dxl_len(addr_info);
    DXL2_Status status;

    if (len == 1u) {
        status = dxl2_write_u8(&motor_bus, id, addr, (uint8_t)value);
    } else if (len == 2u) {
        status = dxl2_write_u16(&motor_bus, id, addr, (uint16_t)value);
    } else if (len == 4u) {
        status = dxl2_write_u32(&motor_bus, id, addr, value);
    } else {
        return 0u;
    }

    return (status == DXL2_OK) ? 1u : 0u;
}
