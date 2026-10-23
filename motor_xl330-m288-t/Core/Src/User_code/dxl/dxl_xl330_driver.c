#include "User_code/dxl/dxl_xl330_driver.h"

#include <string.h>

#define XL330_LAYOUT_READ_MAX_SPAN 32u

static const XL330_Item xl330_default_items[] = { XL330_CUR, XL330_VEL, XL330_POS };
static const XL330_Item xl330_fast_items[] = { XL330_POS, XL330_VEL, XL330_CUR };
static const XL330_Item xl330_state_items[] = { XL330_POS, XL330_VEL, XL330_CUR };
static const XL330_Item xl330_health_items[] = { XL330_TEMP, XL330_VOLT, XL330_HW_ERROR };
static const XL330_Item xl330_motion_items[] = { XL330_MOVING, XL330_MOVING_STATUS, XL330_POS, XL330_VEL };

static uint16_t u16_from_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t u32_from_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void u16_to_le(uint16_t value, uint8_t *data)
{
    data[0] = (uint8_t)(value & 0xffu);
    data[1] = (uint8_t)(value >> 8);
}

static void u32_to_le(uint32_t value, uint8_t *data)
{
    data[0] = (uint8_t)(value & 0xffu);
    data[1] = (uint8_t)((value >> 8) & 0xffu);
    data[2] = (uint8_t)((value >> 16) & 0xffu);
    data[3] = (uint8_t)((value >> 24) & 0xffu);
}

static DXL2_Status xl330_store_status(XL330_Motor *motor, DXL2_Status status)
{
    if (motor != 0) {
        motor->last_status = status;
    }
    return status;
}

static DXL2_Status xl330_group_store_status(XL330_Group *group, DXL2_Status status)
{
    if (group != 0) {
        group->last_status = status;
    }
    return status;
}

static DXL2_Status xl330_group_validate(XL330_Group *group)
{
    if (group == 0 || group->ids == 0 || group->count == 0u) {
        return DXL2_ERR_BAD_PARAM;
    }
    return DXL2_OK;
}

static DXL2_Status xl330_controller_store_status(XL330_Controller *ctl, DXL2_Status status)
{
    if (ctl != 0) {
        ctl->last_status = status;
    }
    return status;
}

static int8_t xl330_controller_index(const XL330_Controller *ctl, uint8_t id)
{
    if (ctl == 0) {
        return -1;
    }
    for (uint8_t i = 0; i < ctl->count; ++i) {
        if (ctl->ids[i] == id) {
            return (int8_t)i;
        }
    }
    return -1;
}

#define XL330_PENDING_MODE(i)   (1ul << (i))
#define XL330_PENDING_TORQUE(i) (1ul << (8u + (i)))
#define XL330_PENDING_CONFIG(i) (XL330_PENDING_MODE(i) | XL330_PENDING_TORQUE(i))
#define XL330_PENDING_GOAL(i)   (1ul << (i))
#define XL330_LAYOUT_MAX_SPAN   32u

static uint8_t xl330_default_ids[XL330_MAX_MOTORS];
static XL330_MotorSlot xl330_default_motors[XL330_MAX_MOTORS];
static XL330_State xl330_default_states[XL330_MAX_MOTORS];
static XL330_StatusReport xl330_default_reports[XL330_MAX_MOTORS];
static XL330_LayoutDef xl330_default_layouts[XL330_MAX_LAYOUTS];
static XL330_Item xl330_default_layout_items[XL330_MAX_LAYOUTS * XL330_MAX_LAYOUT_ITEMS];

static const XL330_Item xl330_builtin_default_items[] = { XL330_CUR, XL330_VEL, XL330_POS };
static const XL330_Item xl330_builtin_fast_items[] = { XL330_POS, XL330_VEL, XL330_CUR };
static const XL330_Item xl330_builtin_state_items[] = { XL330_CUR, XL330_VEL, XL330_POS, XL330_MOVING, XL330_HW_ERROR };
static const XL330_Item xl330_builtin_health_items[] = { XL330_VOLT, XL330_TEMP, XL330_HW_ERROR };
static const XL330_Item xl330_builtin_motion_items[] = { XL330_MOVING, XL330_MOVING_STATUS, XL330_TICK };

static XL330_StatusTypeDef xl330_map_status(DXL2_Status status)
{
    switch (status) {
    case DXL2_OK: return XL330_OK;
    case DXL2_ERR_TIMEOUT: return XL330_TIMEOUT;
    case DXL2_ERR_BAD_PARAM: return XL330_INVALID_PARAM;
    case DXL2_ERR_BUFFER_TOO_SMALL: return XL330_STORAGE_FULL;
    case DXL2_ERR_UNSUPPORTED: return XL330_UNSUPPORTED;
    case DXL2_ERR_STATUS_ERROR: return XL330_DEVICE_ERROR;
    case DXL2_ERR_TRANSPORT: return XL330_BUS_ERROR;
    default: return XL330_ERROR;
    }
}

static XL330_StatusTypeDef xl330_hal_store(XL330_HandleTypeDef *h, DXL2_Status ps, uint8_t id, uint8_t index, uint16_t addr)
{
    XL330_StatusTypeDef s = xl330_map_status(ps);
    if (h != 0) {
        h->last_status = ps;
        h->last_report.status = s;
        h->last_report.protocol_status = ps;
        h->last_report.id = id;
        h->last_report.index = index;
        h->last_report.address = addr;
    }
    return s;
}

static int8_t xl330_find_index_h(const XL330_HandleTypeDef *h, uint8_t id)
{
    if (h == 0) return -1;
    for (uint8_t i = 0; i < h->motor_count; ++i) if (h->storage.motors[i].present && h->storage.motors[i].id == id) return (int8_t)i;
    return -1;
}

static uint16_t xl330_item_addr(XL330_Item item)
{
    switch (item) {
    case XL330_POS: return XL330_ADDR_PRESENT_POSITION;
    case XL330_VEL: return XL330_ADDR_PRESENT_VELOCITY;
    case XL330_CUR: return XL330_ADDR_PRESENT_CURRENT;
    case XL330_PWM: return XL330_ADDR_PRESENT_PWM;
    case XL330_VOLT: return XL330_ADDR_PRESENT_INPUT_VOLTAGE;
    case XL330_TEMP: return XL330_ADDR_PRESENT_TEMPERATURE;
    case XL330_MOVING: return XL330_ADDR_MOVING;
    case XL330_MOVING_STATUS: return XL330_ADDR_MOVING_STATUS;
    case XL330_HW_ERROR: return XL330_ADDR_HARDWARE_ERROR_STATUS;
    case XL330_TICK: return XL330_ADDR_REALTIME_TICK;
    case XL330_GOAL_POS_ITEM: return XL330_ADDR_GOAL_POSITION;
    case XL330_GOAL_VEL_ITEM: return XL330_ADDR_GOAL_VELOCITY;
    case XL330_GOAL_CUR_ITEM: return XL330_ADDR_GOAL_CURRENT;
    case XL330_GOAL_PWM_ITEM: return XL330_ADDR_GOAL_PWM;
    default: return 0xffffu;
    }
}

static uint16_t xl330_item_len(XL330_Item item)
{
    switch (item) {
    case XL330_POS:
    case XL330_VEL:
    case XL330_GOAL_POS_ITEM:
    case XL330_GOAL_VEL_ITEM: return XL330_LEN_U32;
    case XL330_CUR:
    case XL330_PWM:
    case XL330_VOLT:
    case XL330_TICK:
    case XL330_GOAL_CUR_ITEM:
    case XL330_GOAL_PWM_ITEM: return XL330_LEN_U16;
    case XL330_TEMP:
    case XL330_MOVING:
    case XL330_MOVING_STATUS:
    case XL330_HW_ERROR: return XL330_LEN_U8;
    default: return 0u;
    }
}

static int32_t xl330_read_le_signed(const uint8_t *p, uint16_t len)
{
    if (len == 1u) return (int8_t)p[0];
    if (len == 2u) return (int16_t)u16_from_le(p);
    return (int32_t)u32_from_le(p);
}

static void xl330_cache_item(XL330_HandleTypeDef *h, uint8_t idx, XL330_Item item, const uint8_t *p, uint16_t len)
{
    int32_t v = xl330_read_le_signed(p, len);
    if (item == XL330_CUR) h->storage.states[idx].present_current = (int16_t)v;
    else if (item == XL330_VEL) h->storage.states[idx].present_velocity = v;
    else if (item == XL330_POS) h->storage.states[idx].present_position = v;
    else if (item == XL330_HW_ERROR) h->storage.reports[idx].error_code = (uint8_t)v;
}

static XL330_StatusTypeDef xl330_add_layout_internal(XL330_HandleTypeDef *h, XL330_LayoutId id, const char *name, const XL330_Item *items, uint8_t count)
{
    if (h == 0 || items == 0 || count == 0u || count > h->storage.max_layout_items) return XL330_INVALID_PARAM;
    if (h->layout_count >= h->storage.max_layouts) return XL330_STORAGE_FULL;
    XL330_LayoutDef *l = &h->storage.layouts[h->layout_count];
    XL330_Item *dst = &h->storage.layout_items[(uint16_t)h->layout_count * h->storage.max_layout_items];
    for (uint8_t i = 0; i < count; ++i) { if (xl330_item_len(items[i]) == 0u) return XL330_INVALID_PARAM; dst[i] = items[i]; }
    l->id = id; l->name = name; l->items = dst; l->item_count = count; h->layout_count++;
    return XL330_OK;
}

static const XL330_LayoutDef *xl330_find_layout(const XL330_HandleTypeDef *h, XL330_LayoutId id)
{
    if (h == 0) return 0;
    for (uint8_t i = 0; i < h->layout_count; ++i) if (h->storage.layouts[i].id == id) return &h->storage.layouts[i];
    return 0;
}

DXL2_Status xl330_init(XL330_Motor *motor, UART_HandleTypeDef *huart, uint8_t id)
{
    return xl330_init_half_duplex(motor, huart, id, 0, 0u, GPIO_PIN_SET);
}

DXL2_Status xl330_init_half_duplex(XL330_Motor *motor,
                                   UART_HandleTypeDef *huart,
                                   uint8_t id,
                                   GPIO_TypeDef *dir_port,
                                   uint16_t dir_pin,
                                   GPIO_PinState tx_state)
{
    if (motor == 0 || huart == 0) {
        return xl330_store_status(motor, DXL2_ERR_BAD_PARAM);
    }

    motor->id = id;
    motor->last_status = DXL2_OK;

    DXL2_Status status = dxl2_stm32hal_transport_init_half_duplex(&motor->transport,
                                                                  &motor->hal,
                                                                  huart,
                                                                  dir_port,
                                                                  dir_pin,
                                                                  tx_state);
    if (status != DXL2_OK) {
        return xl330_store_status(motor, status);
    }

    status = dxl2_bus_init(&motor->bus, &motor->transport);
    return xl330_store_status(motor, status);
}

DXL2_Status xl330_ping(XL330_Motor *motor, DXL2_DeviceInfo *info)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor, dxl2_ping(&motor->bus, motor->id, info));
}

DXL2_Status xl330_scan(UART_HandleTypeDef *huart,
                       uint8_t first_id,
                       uint8_t last_id,
                       uint8_t *found_ids,
                       uint8_t max_found,
                       uint8_t *found_count)
{
    if (huart == 0 || found_ids == 0 || found_count == 0 || first_id > last_id) {
        return DXL2_ERR_BAD_PARAM;
    }

    *found_count = 0;
    XL330_Motor probe;
    DXL2_Status init_status = xl330_init(&probe, huart, first_id);
    if (init_status != DXL2_OK) {
        return init_status;
    }

    for (uint16_t id = first_id; id <= last_id; ++id) {
        probe.id = (uint8_t)id;
        DXL2_DeviceInfo info;
        DXL2_Status ping_status = xl330_ping(&probe, &info);
        if (ping_status == DXL2_OK && *found_count < max_found) {
            found_ids[*found_count] = (uint8_t)id;
            *found_count = (uint8_t)(*found_count + 1u);
        }
    }

    return (*found_count > 0u) ? DXL2_OK : DXL2_ERR_TIMEOUT;
}

DXL2_Status xl330_set_torque(XL330_Motor *motor, uint8_t enable)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor,
                              dxl2_write_u8(&motor->bus,
                                            motor->id,
                                            XL330_ADDR_TORQUE_ENABLE,
                                            enable ? XL330_TORQUE_ENABLE : XL330_TORQUE_DISABLE));
}

DXL2_Status xl330_enable_torque(XL330_Motor *motor)
{
    return xl330_set_torque(motor, XL330_TORQUE_ENABLE);
}

DXL2_Status xl330_disable_torque(XL330_Motor *motor)
{
    return xl330_set_torque(motor, XL330_TORQUE_DISABLE);
}

DXL2_Status xl330_set_operating_mode(XL330_Motor *motor, uint8_t mode)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }

    DXL2_Status status = xl330_disable_torque(motor);
    if (status != DXL2_OK) {
        return status;
    }

    status = dxl2_write_u8(&motor->bus, motor->id, XL330_ADDR_OPERATING_MODE, mode);
    return xl330_store_status(motor, status);
}

DXL2_Status xl330_set_current_mode(XL330_Motor *motor)
{
    return xl330_set_operating_mode(motor, XL330_MODE_CURRENT);
}

DXL2_Status xl330_set_goal_current(XL330_Motor *motor, int16_t current_ma)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor,
                              dxl2_write_u16(&motor->bus,
                                             motor->id,
                                             XL330_ADDR_GOAL_CURRENT,
	                                             (uint16_t)current_ma));
}

DXL2_Status xl330_set_goal_velocity(XL330_Motor *motor, int32_t velocity_raw)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor,
                              dxl2_write_u32(&motor->bus,
                                             motor->id,
                                             XL330_ADDR_GOAL_VELOCITY,
                                             (uint32_t)velocity_raw));
}

DXL2_Status xl330_set_goal_position(XL330_Motor *motor, int32_t position_pulse)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor,
                              dxl2_write_u32(&motor->bus,
                                             motor->id,
                                             XL330_ADDR_GOAL_POSITION,
                                             (uint32_t)position_pulse));
}

DXL2_Status xl330_write_u8(XL330_Motor *motor, uint16_t address, uint8_t value)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor, dxl2_write_u8(&motor->bus, motor->id, address, value));
}

DXL2_Status xl330_write_u16(XL330_Motor *motor, uint16_t address, uint16_t value)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor, dxl2_write_u16(&motor->bus, motor->id, address, value));
}

DXL2_Status xl330_write_u32(XL330_Motor *motor, uint16_t address, uint32_t value)
{
    if (motor == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor, dxl2_write_u32(&motor->bus, motor->id, address, value));
}

DXL2_Status xl330_read_u8(XL330_Motor *motor, uint16_t address, uint8_t *value)
{
    if (motor == 0 || value == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor, dxl2_read_u8(&motor->bus, motor->id, address, value));
}

DXL2_Status xl330_read_u16(XL330_Motor *motor, uint16_t address, uint16_t *value)
{
    if (motor == 0 || value == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor, dxl2_read_u16(&motor->bus, motor->id, address, value));
}

DXL2_Status xl330_read_u32(XL330_Motor *motor, uint16_t address, uint32_t *value)
{
    if (motor == 0 || value == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return xl330_store_status(motor, dxl2_read_u32(&motor->bus, motor->id, address, value));
}

DXL2_Status xl330_read_state(XL330_Motor *motor, XL330_State *state)
{
    if (motor == 0 || state == 0) {
        return DXL2_ERR_BAD_PARAM;
    }

    uint8_t data[XL330_LEN_PRESENT_CURRENT_VELOCITY_POSITION];
    DXL2_Status status = dxl2_read(&motor->bus,
                                   motor->id,
                                   XL330_ADDR_PRESENT_CURRENT,
                                   sizeof(data),
                                   data);
    if (status == DXL2_OK) {
        state->present_current = (int16_t)u16_from_le(&data[0]);
        state->present_velocity = (int32_t)u32_from_le(&data[2]);
        state->present_position = (int32_t)u32_from_le(&data[6]);
    }
    return xl330_store_status(motor, status);
}

DXL2_Status xl330_read_present_current(XL330_Motor *motor, int16_t *current_ma)
{
    if (motor == 0 || current_ma == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    uint16_t value = 0;
    DXL2_Status status = dxl2_read_u16(&motor->bus, motor->id, XL330_ADDR_PRESENT_CURRENT, &value);
    if (status == DXL2_OK) {
        *current_ma = (int16_t)value;
    }
    return xl330_store_status(motor, status);
}

DXL2_Status xl330_read_present_velocity(XL330_Motor *motor, int32_t *velocity_raw)
{
    if (motor == 0 || velocity_raw == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    uint32_t value = 0;
    DXL2_Status status = dxl2_read_u32(&motor->bus, motor->id, XL330_ADDR_PRESENT_VELOCITY, &value);
    if (status == DXL2_OK) {
        *velocity_raw = (int32_t)value;
    }
    return xl330_store_status(motor, status);
}

DXL2_Status xl330_read_present_position(XL330_Motor *motor, int32_t *position_pulse)
{
    if (motor == 0 || position_pulse == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    uint32_t value = 0;
    DXL2_Status status = dxl2_read_u32(&motor->bus, motor->id, XL330_ADDR_PRESENT_POSITION, &value);
    if (status == DXL2_OK) {
        *position_pulse = (int32_t)value;
    }
    return xl330_store_status(motor, status);
}

DXL2_Status xl330_group_init(XL330_Group *group, UART_HandleTypeDef *huart, const uint8_t *ids, uint8_t count)
{
    return xl330_group_init_half_duplex(group, huart, ids, count, 0, 0u, GPIO_PIN_SET);
}

DXL2_Status xl330_group_init_half_duplex(XL330_Group *group, UART_HandleTypeDef *huart, const uint8_t *ids, uint8_t count, GPIO_TypeDef *dir_port, uint16_t dir_pin, GPIO_PinState tx_state)
{
    if (group == 0 || huart == 0 || ids == 0 || count == 0u) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    group->ids = ids;
    group->count = count;
    group->last_status = DXL2_OK;
    DXL2_Status status = dxl2_stm32hal_transport_init_half_duplex(&group->transport, &group->hal, huart, dir_port, dir_pin, tx_state);
    if (status != DXL2_OK) return xl330_group_store_status(group, status);
    return xl330_group_store_status(group, dxl2_bus_init(&group->bus, &group->transport));
}

DXL2_Status xl330_group_safe_status(const XL330_Group *group, DXL2_Status bus_status, const DXL2_GroupResult *results, XL330_StatusReport *reports)
{
    if (group == 0 || group->ids == 0 || group->count == 0u) return DXL2_ERR_BAD_PARAM;
    DXL2_Status summary = bus_status;
    for (uint8_t i = 0; reports != 0 && i < group->count; ++i) {
        reports[i].id = group->ids[i];
        reports[i].status = bus_status;
        reports[i].error = 0u;
        reports[i].alert = 0u;
        reports[i].error_code = 0u;
        if (results != 0) {
            reports[i].id = results[i].id;
            reports[i].error = results[i].error;
            reports[i].alert = results[i].alert;
            reports[i].error_code = results[i].error_code;
            reports[i].status = (results[i].error != 0u) ? DXL2_ERR_STATUS_ERROR : DXL2_OK;
        }
        if (summary == DXL2_OK && reports[i].status != DXL2_OK) summary = reports[i].status;
    }
    return summary;
}

DXL2_Status xl330_group_scan(XL330_Group *group, uint8_t first_id, uint8_t last_id, uint8_t *found_ids, uint8_t max_found, uint8_t *found_count, XL330_StatusReport *reports)
{
    if (xl330_group_validate(group) != DXL2_OK || found_ids == 0 || found_count == 0 || first_id > last_id) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    *found_count = 0;
    uint8_t report_index = 0u;
    for (uint16_t id = first_id; id <= last_id; ++id) {
        DXL2_DeviceInfo info;
        DXL2_Status status = dxl2_ping(&group->bus, (uint8_t)id, &info);
        if (status == DXL2_OK) {
            if (*found_count < max_found) found_ids[*found_count] = (uint8_t)id;
            *found_count = (uint8_t)(*found_count + 1u);
        }
        if (reports != 0 && report_index < group->count) {
            reports[report_index].id = (uint8_t)id;
            reports[report_index].status = status;
            reports[report_index].error = 0u;
            reports[report_index].alert = 0u;
            reports[report_index].error_code = 0u;
            ++report_index;
        }
    }
    return xl330_group_store_status(group, (*found_count > 0u) ? DXL2_OK : DXL2_ERR_TIMEOUT);
}

DXL2_Status xl330_group_sync_read_state(XL330_Group *group, XL330_State *states, XL330_StatusReport *reports)
{
    if (xl330_group_validate(group) != DXL2_OK || states == 0) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    uint8_t raw[XL330_LEN_PRESENT_CURRENT_VELOCITY_POSITION * group->count];
    DXL2_GroupResult results[group->count];
    DXL2_Status status = dxl2_sync_read(&group->bus, XL330_ADDR_PRESENT_CURRENT, XL330_LEN_PRESENT_CURRENT_VELOCITY_POSITION, group->ids, group->count, raw, XL330_LEN_PRESENT_CURRENT_VELOCITY_POSITION, results);
    if (status == DXL2_OK || status == DXL2_ERR_STATUS_ERROR) {
        for (uint8_t i = 0; i < group->count; ++i) {
            const uint8_t *data = &raw[(uint16_t)i * XL330_LEN_PRESENT_CURRENT_VELOCITY_POSITION];
            states[i].present_current = (int16_t)u16_from_le(&data[0]);
            states[i].present_velocity = (int32_t)u32_from_le(&data[2]);
            states[i].present_position = (int32_t)u32_from_le(&data[6]);
        }
    }
    const DXL2_GroupResult *safe_results = (status == DXL2_OK || status == DXL2_ERR_STATUS_ERROR) ? results : 0;
    return xl330_group_store_status(group, xl330_group_safe_status(group, status, safe_results, reports));
}

DXL2_Status xl330_group_sync_write_goal_current(XL330_Group *group, const int16_t *currents_ma)
{
    if (xl330_group_validate(group) != DXL2_OK || currents_ma == 0) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    uint8_t data[group->count][XL330_LEN_U16];
    DXL2_SyncWriteItem items[group->count];
    for (uint8_t i = 0; i < group->count; ++i) { u16_to_le((uint16_t)currents_ma[i], data[i]); items[i].id = group->ids[i]; items[i].data = data[i]; }
    return xl330_group_store_status(group, dxl2_sync_write(&group->bus, XL330_ADDR_GOAL_CURRENT, XL330_LEN_U16, items, group->count));
}

DXL2_Status xl330_group_sync_write_goal_velocity(XL330_Group *group, const int32_t *velocities_raw)
{
    if (xl330_group_validate(group) != DXL2_OK || velocities_raw == 0) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    uint8_t data[group->count][XL330_LEN_U32];
    DXL2_SyncWriteItem items[group->count];
    for (uint8_t i = 0; i < group->count; ++i) { u32_to_le((uint32_t)velocities_raw[i], data[i]); items[i].id = group->ids[i]; items[i].data = data[i]; }
    return xl330_group_store_status(group, dxl2_sync_write(&group->bus, XL330_ADDR_GOAL_VELOCITY, XL330_LEN_U32, items, group->count));
}

DXL2_Status xl330_group_sync_write_goal_position(XL330_Group *group, const int32_t *positions_pulse)
{
    if (xl330_group_validate(group) != DXL2_OK || positions_pulse == 0) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    uint8_t data[group->count][XL330_LEN_U32];
    DXL2_SyncWriteItem items[group->count];
    for (uint8_t i = 0; i < group->count; ++i) { u32_to_le((uint32_t)positions_pulse[i], data[i]); items[i].id = group->ids[i]; items[i].data = data[i]; }
    return xl330_group_store_status(group, dxl2_sync_write(&group->bus, XL330_ADDR_GOAL_POSITION, XL330_LEN_U32, items, group->count));
}

DXL2_Status xl330_group_torque_all(XL330_Group *group, uint8_t enable)
{
    if (xl330_group_validate(group) != DXL2_OK) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    uint8_t value = enable ? XL330_TORQUE_ENABLE : XL330_TORQUE_DISABLE;
    DXL2_SyncWriteItem items[group->count];
    for (uint8_t i = 0; i < group->count; ++i) { items[i].id = group->ids[i]; items[i].data = &value; }
    return xl330_group_store_status(group, dxl2_sync_write(&group->bus, XL330_ADDR_TORQUE_ENABLE, XL330_LEN_U8, items, group->count));
}

DXL2_Status xl330_group_mode_all(XL330_Group *group, uint8_t mode)
{
    if (xl330_group_validate(group) != DXL2_OK) return xl330_group_store_status(group, DXL2_ERR_BAD_PARAM);
    DXL2_Status status = xl330_group_torque_all(group, XL330_TORQUE_DISABLE);
    if (status != DXL2_OK) return status;
    DXL2_SyncWriteItem items[group->count];
    for (uint8_t i = 0; i < group->count; ++i) { items[i].id = group->ids[i]; items[i].data = &mode; }
    return xl330_group_store_status(group, dxl2_sync_write(&group->bus, XL330_ADDR_OPERATING_MODE, XL330_LEN_U8, items, group->count));
}

DXL2_Status xl330_group_current_mode_all(XL330_Group *group) { return xl330_group_mode_all(group, XL330_MODE_CURRENT); }
DXL2_Status xl330_group_velocity_mode_all(XL330_Group *group) { return xl330_group_mode_all(group, XL330_MODE_VELOCITY); }
DXL2_Status xl330_group_position_mode_all(XL330_Group *group) { return xl330_group_mode_all(group, XL330_MODE_POSITION); }

const char *xl330_status_string(DXL2_Status status)
{
    switch (status) {
    case DXL2_OK: return "OK";
    case DXL2_ERR_TIMEOUT: return "timeout";
    case DXL2_ERR_CRC: return "crc";
    case DXL2_ERR_ID_MISMATCH: return "id_mismatch";
    case DXL2_ERR_BAD_PACKET: return "bad_packet";
    case DXL2_ERR_STATUS_ERROR: return "status_error";
    case DXL2_ERR_UNSUPPORTED: return "unsupported";
    case DXL2_ERR_BAD_PARAM: return "bad_param";
    case DXL2_ERR_NO_RESPONSE_EXPECTED: return "no_response_expected";
    case DXL2_ERR_TRANSPORT: return "transport";
    case DXL2_ERR_BUFFER_TOO_SMALL: return "buffer_too_small";
    default: return "unknown";
    }
}

DXL2_Status xl330_begin(XL330_Controller *ctl, UART_HandleTypeDef *huart)
{
    return xl330_begin_half_duplex(ctl, huart, 0, 0u, GPIO_PIN_SET);
}

DXL2_Status xl330_begin_half_duplex(XL330_Controller *ctl,
                                    UART_HandleTypeDef *huart,
                                    GPIO_TypeDef *dir_port,
                                    uint16_t dir_pin,
                                    GPIO_PinState tx_state)
{
    if (ctl == 0 || huart == 0) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }

    for (uint8_t i = 0; i < XL330_MAX_MOTORS; ++i) {
        ctl->ids[i] = 0u;
        ctl->goal_current[i] = 0;
        ctl->goal_velocity[i] = 0;
        ctl->goal_position[i] = 0;
        ctl->states[i].present_current = 0;
        ctl->states[i].present_velocity = 0;
        ctl->states[i].present_position = 0;
        ctl->reports[i].id = 0u;
        ctl->reports[i].status = DXL2_OK;
        ctl->reports[i].error = 0u;
        ctl->reports[i].alert = 0u;
        ctl->reports[i].error_code = 0u;
    }

    ctl->count = 0u;
    ctl->mode = XL330_MODE_POSITION;
    ctl->mode_dirty = 0u;
    ctl->torque_enable = XL330_TORQUE_DISABLE;
    ctl->torque_dirty = 0u;
    ctl->goal_kind = XL330_GOAL_NONE;
    ctl->last_status = DXL2_OK;

    DXL2_Status status = xl330_group_init_half_duplex(&ctl->group,
                                                      huart,
                                                      ctl->ids,
                                                      1u,
                                                      dir_port,
                                                      dir_pin,
                                                      tx_state);
    ctl->group.count = 0u;
    return xl330_controller_store_status(ctl, status);
}

DXL2_Status xl330_add(XL330_Controller *ctl, uint8_t id)
{
    if (ctl == 0 || ctl->count >= XL330_MAX_MOTORS || xl330_controller_index(ctl, id) >= 0) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }
    ctl->ids[ctl->count] = id;
    ctl->reports[ctl->count].id = id;
    ctl->count++;
    ctl->group.ids = ctl->ids;
    ctl->group.count = ctl->count;
    return xl330_controller_store_status(ctl, DXL2_OK);
}

DXL2_Status xl330_mode(XL330_Controller *ctl, uint8_t mode)
{
    if (ctl == 0) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }
    ctl->mode = mode;
    ctl->mode_dirty = 1u;
    return xl330_controller_store_status(ctl, DXL2_OK);
}

DXL2_Status xl330_torque(XL330_Controller *ctl, uint8_t enable)
{
    if (ctl == 0) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }
    ctl->torque_enable = enable ? XL330_TORQUE_ENABLE : XL330_TORQUE_DISABLE;
    ctl->torque_dirty = 1u;
    return xl330_controller_store_status(ctl, DXL2_OK);
}

DXL2_Status xl330_goal_current(XL330_Controller *ctl, uint8_t id, int16_t current_ma)
{
    int8_t index = xl330_controller_index(ctl, id);
    if (index < 0) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }
    ctl->goal_current[index] = current_ma;
    ctl->goal_kind = XL330_GOAL_CURRENT;
    return xl330_controller_store_status(ctl, DXL2_OK);
}

DXL2_Status xl330_goal_velocity(XL330_Controller *ctl, uint8_t id, int32_t velocity_raw)
{
    int8_t index = xl330_controller_index(ctl, id);
    if (index < 0) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }
    ctl->goal_velocity[index] = velocity_raw;
    ctl->goal_kind = XL330_GOAL_VELOCITY;
    return xl330_controller_store_status(ctl, DXL2_OK);
}

DXL2_Status xl330_goal_position(XL330_Controller *ctl, uint8_t id, int32_t position_pulse)
{
    int8_t index = xl330_controller_index(ctl, id);
    if (index < 0) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }
    ctl->goal_position[index] = position_pulse;
    ctl->goal_kind = XL330_GOAL_POSITION;
    return xl330_controller_store_status(ctl, DXL2_OK);
}

DXL2_Status xl330_commit(XL330_Controller *ctl)
{
    if (ctl == 0 || ctl->count == 0u) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }

    DXL2_Status status = DXL2_OK;
    if (ctl->mode_dirty != 0u) {
        status = xl330_group_mode_all(&ctl->group, ctl->mode);
        if (status != DXL2_OK) return xl330_controller_store_status(ctl, status);
        ctl->mode_dirty = 0u;
        ctl->torque_dirty = 1u;
    }
    if (ctl->torque_dirty != 0u) {
        status = xl330_group_torque_all(&ctl->group, ctl->torque_enable);
        if (status != DXL2_OK) return xl330_controller_store_status(ctl, status);
        ctl->torque_dirty = 0u;
    }

    switch (ctl->goal_kind) {
    case XL330_GOAL_CURRENT:
        status = xl330_group_sync_write_goal_current(&ctl->group, ctl->goal_current);
        break;
    case XL330_GOAL_VELOCITY:
        status = xl330_group_sync_write_goal_velocity(&ctl->group, ctl->goal_velocity);
        break;
    case XL330_GOAL_POSITION:
        status = xl330_group_sync_write_goal_position(&ctl->group, ctl->goal_position);
        break;
    case XL330_GOAL_NONE:
    default:
        status = DXL2_OK;
        break;
    }
    ctl->goal_kind = XL330_GOAL_NONE;
    return xl330_controller_store_status(ctl, status);
}

DXL2_Status xl330_update(XL330_Controller *ctl)
{
    if (ctl == 0 || ctl->count == 0u) {
        return xl330_controller_store_status(ctl, DXL2_ERR_BAD_PARAM);
    }
    return xl330_controller_store_status(ctl,
                                         xl330_group_sync_read_state(&ctl->group,
                                                                    ctl->states,
                                                                    ctl->reports));
}

const XL330_State *xl330_state(const XL330_Controller *ctl, uint8_t id)
{
    int8_t index = xl330_controller_index(ctl, id);
    return (index < 0) ? 0 : &ctl->states[index];
}

const XL330_StatusReport *xl330_report(const XL330_Controller *ctl, uint8_t id)
{
    int8_t index = xl330_controller_index(ctl, id);
    return (index < 0) ? 0 : &ctl->reports[index];
}

static XL330_StatusTypeDef xl330_status_from_bus(DXL2_Status status)
{
    switch (status) {
    case DXL2_OK: return XL330_OK;
    case DXL2_ERR_TIMEOUT: return XL330_TIMEOUT;
    case DXL2_ERR_BAD_PARAM:
    case DXL2_ERR_BUFFER_TOO_SMALL: return XL330_INVALID_PARAM;
    default: return XL330_BUS_ERROR;
    }
}

static const XL330_Item *xl330_builtin_layout_items(XL330_LayoutId id, uint8_t *count)
{
    switch (id) {
    case XL330_LAYOUT_DEFAULT: *count = 3u; return xl330_default_items;
    case XL330_LAYOUT_FAST: *count = 3u; return xl330_fast_items;
    case XL330_LAYOUT_STATE: *count = 3u; return xl330_state_items;
    case XL330_LAYOUT_HEALTH: *count = 3u; return xl330_health_items;
    case XL330_LAYOUT_MOTION: *count = 4u; return xl330_motion_items;
    default: *count = 0u; return 0;
    }
}

static const XL330_Item *xl330_find_layout_items(const XL330_HandleTypeDef *hxl330,
                                                 XL330_LayoutId layout_id,
                                                 uint8_t *item_count)
{
    const XL330_Item *items = xl330_builtin_layout_items(layout_id, item_count);
    if (items != 0) return items;
    if (hxl330 == 0 || hxl330->storage.layouts == 0) return 0;
    for (uint8_t i = 0; i < hxl330->layout_count; ++i) {
        if (hxl330->storage.layouts[i].id == layout_id) {
            *item_count = hxl330->storage.layouts[i].item_count;
            return hxl330->storage.layouts[i].items;
        }
    }
    return 0;
}

static uint16_t xl330_layout_stride_from_items(const XL330_Item *items, uint8_t count)
{
    uint16_t stride = 0u;
    if (items == 0 || count == 0u || count > XL330_MAX_LAYOUT_ITEMS) return 0u;
    for (uint8_t i = 0; i < count; ++i) {
        uint16_t len = xl330_item_len(items[i]);
        if (len == 0u || xl330_item_addr(items[i]) == 0xffffu) return 0u;
        stride = (uint16_t)(stride + len);
    }
    return stride;
}

static void xl330_copy_item_to_outputs(XL330_HandleTypeDef *hxl330,
                                        uint8_t motor_index,
                                        XL330_Item item,
                                        const uint8_t *src,
                                        uint16_t len,
                                        uint8_t *user_buf,
                                        uint16_t out_off)
{
    if (user_buf != 0) memcpy(&user_buf[out_off], src, len);
    if (hxl330->storage.states != 0) {
        if (item == XL330_CUR) hxl330->storage.states[motor_index].present_current = (int16_t)u16_from_le(src);
        else if (item == XL330_VEL) hxl330->storage.states[motor_index].present_velocity = (int32_t)u32_from_le(src);
        else if (item == XL330_POS) hxl330->storage.states[motor_index].present_position = (int32_t)u32_from_le(src);
    }
    if (hxl330->storage.reports != 0 && item == XL330_HW_ERROR) hxl330->storage.reports[motor_index].error_code = src[0];
}

static XL330_StatusTypeDef xl330_register_layout(XL330_HandleTypeDef *hxl330,
                                                  XL330_LayoutId layout_id,
                                                  const char *name,
                                                  const XL330_Item *items,
                                                  uint8_t item_count)
{
    if (hxl330 == 0 || items == 0 || item_count == 0u ||
        item_count > hxl330->storage.max_layout_items ||
        hxl330->storage.layouts == 0 || hxl330->storage.layout_items == 0 ||
        layout_id < XL330_LAYOUT_CUSTOM_BASE ||
        xl330_layout_stride_from_items(items, item_count) == 0u) {
        return XL330_INVALID_PARAM;
    }

    uint8_t slot = hxl330->layout_count;
    for (uint8_t i = 0; i < hxl330->layout_count; ++i) {
        if (hxl330->storage.layouts[i].id == layout_id) { slot = i; break; }
    }
    if (slot == hxl330->layout_count && hxl330->layout_count >= hxl330->storage.max_layouts) return XL330_STORAGE_FULL;

    XL330_Item *copy = &hxl330->storage.layout_items[(uint16_t)slot * hxl330->storage.max_layout_items];
    memcpy(copy, items, (size_t)item_count * sizeof(XL330_Item));
    hxl330->storage.layouts[slot].id = layout_id;
    hxl330->storage.layouts[slot].name = name;
    hxl330->storage.layouts[slot].items = copy;
    hxl330->storage.layouts[slot].item_count = item_count;
    if (slot == hxl330->layout_count) hxl330->layout_count++;
    return XL330_OK;
}

uint16_t XL330_GetLayoutStride(XL330_LayoutId layout_id)
{
    uint8_t count = 0u;
    const XL330_Item *items = xl330_builtin_layout_items(layout_id, &count);
    return xl330_layout_stride_from_items(items, count);
}

XL330_StatusTypeDef XL330_AddLayout(XL330_HandleTypeDef *hxl330,
                                    XL330_LayoutId layout_id,
                                    const XL330_Item *items,
                                    uint8_t item_count)
{
    return xl330_register_layout(hxl330, layout_id, 0, items, item_count);
}

XL330_StatusTypeDef XL330_AddNamedLayout(XL330_HandleTypeDef *hxl330,
                                         XL330_LayoutId layout_id,
                                         const char *name,
                                         const XL330_Item *items,
                                         uint8_t item_count)
{
    return xl330_register_layout(hxl330, layout_id, name, items, item_count);
}

XL330_StatusTypeDef XL330_RemoveLayout(XL330_HandleTypeDef *hxl330, XL330_LayoutId layout_id)
{
    if (hxl330 == 0 || hxl330->storage.layouts == 0 || layout_id < XL330_LAYOUT_CUSTOM_BASE) return XL330_INVALID_PARAM;
    for (uint8_t i = 0; i < hxl330->layout_count; ++i) {
        if (hxl330->storage.layouts[i].id == layout_id) {
            for (uint8_t j = i; (uint8_t)(j + 1u) < hxl330->layout_count; ++j) hxl330->storage.layouts[j] = hxl330->storage.layouts[j + 1u];
            hxl330->layout_count--;
            return XL330_OK;
        }
    }
    return XL330_LAYOUT_NOT_FOUND;
}

XL330_StatusTypeDef XL330_UpdateLayout(XL330_HandleTypeDef *hxl330,
                                       XL330_LayoutId layout_id,
                                       uint8_t *user_buf,
                                       uint16_t user_buf_len)
{
    uint8_t count = 0u;
    const XL330_Item *items = xl330_find_layout_items(hxl330, layout_id, &count);
    if (hxl330 == 0 || items == 0 || hxl330->motor_count == 0u || hxl330->storage.ids == 0) return XL330_INVALID_PARAM;

    uint16_t stride = xl330_layout_stride_from_items(items, count);
    if (stride == 0u) return XL330_INVALID_PARAM;
    if (user_buf != 0 && user_buf_len < (uint16_t)(stride * hxl330->motor_count)) return XL330_INVALID_PARAM;

    uint16_t min_addr = 0xffffu, max_addr = 0u;
    for (uint8_t i = 0; i < count; ++i) {
        uint16_t addr = xl330_item_addr(items[i]);
        uint16_t end = (uint16_t)(addr + xl330_item_len(items[i]));
        if (addr < min_addr) min_addr = addr;
        if (end > max_addr) max_addr = end;
    }
    uint16_t span = (uint16_t)(max_addr - min_addr);
    DXL2_Status bus_status = DXL2_OK;

    if (span != 0u && span <= XL330_LAYOUT_READ_MAX_SPAN) {
        uint8_t raw[XL330_MAX_MOTORS][XL330_LAYOUT_READ_MAX_SPAN];
        DXL2_GroupResult results[XL330_MAX_MOTORS];
        memset(results, 0, sizeof(results));

        if (hxl330->motor_count == 1u) {
            bus_status = dxl2_read(&hxl330->bus, hxl330->storage.ids[0], min_addr, span, raw[0]);
            if (hxl330->storage.reports != 0) {
                hxl330->storage.reports[0].id = hxl330->storage.ids[0];
                hxl330->storage.reports[0].status = bus_status;
                hxl330->storage.reports[0].error = 0u;
                hxl330->storage.reports[0].alert = 0u;
            }
        } else {
            DXL2_GroupResult fast_result;
            bus_status = dxl2_fast_sync_read(&hxl330->bus, min_addr, span, hxl330->storage.ids, hxl330->motor_count, &raw[0][0], XL330_LAYOUT_READ_MAX_SPAN, &fast_result);
            if (bus_status != DXL2_OK && bus_status != DXL2_ERR_STATUS_ERROR) {
                bus_status = dxl2_sync_read(&hxl330->bus, min_addr, span, hxl330->storage.ids, hxl330->motor_count, &raw[0][0], XL330_LAYOUT_READ_MAX_SPAN, results);
                if (hxl330->storage.reports != 0) {
                    XL330_Group temp_group;
                    temp_group.ids = hxl330->storage.ids;
                    temp_group.count = hxl330->motor_count;
                    (void)xl330_group_safe_status(&temp_group, bus_status, results, hxl330->storage.reports);
                }
            } else if (hxl330->storage.reports != 0) {
                for (uint8_t m = 0; m < hxl330->motor_count; ++m) {
                    hxl330->storage.reports[m].id = hxl330->storage.ids[m];
                    hxl330->storage.reports[m].status = bus_status;
                    hxl330->storage.reports[m].error = fast_result.error;
                    hxl330->storage.reports[m].alert = fast_result.alert;
                    hxl330->storage.reports[m].error_code = fast_result.error_code;
                }
            }
        }
        if (bus_status != DXL2_OK && bus_status != DXL2_ERR_STATUS_ERROR) return xl330_hal_store(hxl330, bus_status, 0u, 0u, min_addr);

        for (uint8_t m = 0; m < hxl330->motor_count; ++m) {
            uint16_t off = (uint16_t)(m * stride);
            for (uint8_t i = 0; i < count; ++i) {
                uint16_t addr = xl330_item_addr(items[i]);
                uint16_t len = xl330_item_len(items[i]);
                xl330_copy_item_to_outputs(hxl330, m, items[i], &raw[m][addr - min_addr], len, user_buf, off);
                off = (uint16_t)(off + len);
            }
        }
        return xl330_hal_store(hxl330, bus_status, 0u, 0u, min_addr);
    }

    uint8_t item_raw[XL330_MAX_MOTORS][XL330_MAX_LAYOUT_ITEMS][XL330_LEN_U32];
    for (uint8_t m = 0; m < hxl330->motor_count; ++m) {
        DXL2_BulkReadItem br[XL330_MAX_LAYOUT_ITEMS];
        DXL2_GroupResult result;
        for (uint8_t i = 0; i < count; ++i) {
            br[i].id = hxl330->storage.ids[m];
            br[i].addr = xl330_item_addr(items[i]);
            br[i].len = xl330_item_len(items[i]);
            br[i].data = item_raw[m][i];
        }
        bus_status = dxl2_fast_bulk_read(&hxl330->bus, br, count, &result);
        if (bus_status != DXL2_OK && bus_status != DXL2_ERR_STATUS_ERROR) bus_status = dxl2_bulk_read(&hxl330->bus, br, count, &result);
        if (bus_status != DXL2_OK && bus_status != DXL2_ERR_STATUS_ERROR) return xl330_hal_store(hxl330, bus_status, hxl330->storage.ids[m], m, 0u);
        if (hxl330->storage.reports != 0) {
            hxl330->storage.reports[m].id = hxl330->storage.ids[m];
            hxl330->storage.reports[m].status = bus_status;
            hxl330->storage.reports[m].error = result.error;
            hxl330->storage.reports[m].alert = result.alert;
            hxl330->storage.reports[m].error_code = result.error_code;
        }
        uint16_t off = (uint16_t)(m * stride);
        for (uint8_t i = 0; i < count; ++i) {
            uint16_t len = xl330_item_len(items[i]);
            xl330_copy_item_to_outputs(hxl330, m, items[i], item_raw[m][i], len, user_buf, off);
            off = (uint16_t)(off + len);
        }
    }
    return xl330_hal_store(hxl330, bus_status, 0u, 0u, 0u);
}

/* Minimal HAL-style lifecycle/staging core completed after layout engine. */
const XL330_StorageConfig *XL330_DefaultStorageConfig(void)
{
    static const XL330_StorageConfig cfg = { xl330_default_ids, XL330_MAX_MOTORS, xl330_default_motors, xl330_default_states, xl330_default_reports, xl330_default_layouts, XL330_MAX_LAYOUTS, xl330_default_layout_items, XL330_MAX_LAYOUT_ITEMS };
    return &cfg;
}
static XL330_StatusTypeDef xl330_init_storage2(XL330_HandleTypeDef *h, const XL330_StorageConfig *s)
{
    if (!h || !s || !s->ids || !s->motors || !s->states || !s->reports || !s->layouts || !s->layout_items) return XL330_INVALID_PARAM;
    h->storage = *s; h->motor_count = 0u; h->layout_count = 0u; h->pending_config_mask = 0u; h->pending_goal_mask = 0u; h->last_status = DXL2_OK; h->async_state = XL330_ASYNC_IDLE;
    h->last_report.status = XL330_OK; h->last_report.protocol_status = DXL2_OK;
    for (uint8_t i=0;i<s->max_motors;++i) { s->ids[i]=0u; s->motors[i].id=0u; s->motors[i].index=i; s->motors[i].present=0u; s->motors[i].operating_mode=XL330_MODE_POSITION; s->motors[i].torque_enable=XL330_TORQUE_DISABLE; s->states[i].present_current=0; s->states[i].present_velocity=0; s->states[i].present_position=0; s->reports[i].id=0u; s->reports[i].status=DXL2_OK; s->reports[i].error=0u; s->reports[i].alert=0u; s->reports[i].error_code=0u; h->staged_mode[i]=XL330_MODE_POSITION; h->staged_torque[i]=XL330_TORQUE_DISABLE; h->staged_goal_kind[i]=XL330_GOAL_NONE; }
    return XL330_OK;
}
XL330_StatusTypeDef XL330_InitDefault(XL330_HandleTypeDef *h, UART_HandleTypeDef *huart) { return XL330_Init(h, huart, XL330_DefaultStorageConfig()); }
XL330_StatusTypeDef XL330_Init(XL330_HandleTypeDef *h, UART_HandleTypeDef *huart, const XL330_StorageConfig *storage) { XL330_InitTypeDef i = { huart, storage, 0u }; return XL330_InitEx(h, &i); }
XL330_StatusTypeDef XL330_InitEx(XL330_HandleTypeDef *h, const XL330_InitTypeDef *init) { if (!init) return XL330_INVALID_PARAM; XL330_HalfDuplexInitTypeDef hd = { init->huart, 0, 0u, GPIO_PIN_SET, init->storage, init->timeout_ms }; return XL330_InitHalfDuplex(h, &hd); }
XL330_StatusTypeDef XL330_InitHalfDuplex(XL330_HandleTypeDef *h, const XL330_HalfDuplexInitTypeDef *init) { if (!h || !init || !init->huart) return xl330_hal_store(h, DXL2_ERR_BAD_PARAM, 0u, 0u, 0u); XL330_StatusTypeDef r=xl330_init_storage2(h, init->storage?init->storage:XL330_DefaultStorageConfig()); if(r!=XL330_OK)return r; DXL2_Status ps=dxl2_stm32hal_transport_init_half_duplex(&h->transport,&h->hal,init->huart,init->dir_port,init->dir_pin,init->tx_state); if(ps==DXL2_OK)ps=dxl2_bus_init(&h->bus,&h->transport); return xl330_hal_store(h,ps,0u,0u,0u); }
XL330_StatusTypeDef XL330_DeInit(XL330_HandleTypeDef *h) { if(!h)return XL330_INVALID_PARAM; h->motor_count=0u; h->pending_config_mask=h->pending_goal_mask=0u; return XL330_OK; }
XL330_StatusTypeDef XL330_AddMotor(XL330_HandleTypeDef *h, uint8_t id) { if(!h||h->motor_count>=h->storage.max_motors||xl330_find_index_h(h,id)>=0)return xl330_hal_store(h,DXL2_ERR_BAD_PARAM,id,0u,0u); uint8_t i=h->motor_count++; h->storage.ids[i]=id; h->storage.motors[i].id=id; h->storage.motors[i].index=i; h->storage.motors[i].present=1u; h->storage.reports[i].id=id; return xl330_hal_store(h,DXL2_OK,id,i,0u); }
XL330_StatusTypeDef XL330_RemoveMotor(XL330_HandleTypeDef *h,uint8_t id){int8_t ix=xl330_find_index_h(h,id); if(ix<0)return XL330_NOT_FOUND; for(uint8_t i=(uint8_t)ix;i+1u<h->motor_count;++i){h->storage.ids[i]=h->storage.ids[i+1u];h->storage.motors[i]=h->storage.motors[i+1u];h->storage.motors[i].index=i;h->storage.states[i]=h->storage.states[i+1u];h->storage.reports[i]=h->storage.reports[i+1u];} h->motor_count--; return XL330_OK;}
XL330_StatusTypeDef XL330_ClearMotors(XL330_HandleTypeDef *h){if(!h)return XL330_INVALID_PARAM;h->motor_count=0u;h->pending_config_mask=h->pending_goal_mask=0u;return XL330_OK;}
static XL330_StatusTypeDef xl330_mode_i2(XL330_HandleTypeDef*h,uint8_t i,uint8_t m){if(!h||i>=h->motor_count)return XL330_INVALID_PARAM;h->staged_mode[i]=m;h->pending_config_mask|=XL330_PENDING_MODE(i);return XL330_OK;}
static XL330_StatusTypeDef xl330_torque_i2(XL330_HandleTypeDef*h,uint8_t i,uint8_t e){if(!h||i>=h->motor_count)return XL330_INVALID_PARAM;h->staged_torque[i]=e?XL330_TORQUE_ENABLE:XL330_TORQUE_DISABLE;h->pending_config_mask|=XL330_PENDING_TORQUE(i);return XL330_OK;}
XL330_StatusTypeDef XL330_SetMode(XL330_HandleTypeDef*h,uint8_t id,uint8_t m){int8_t i=xl330_find_index_h(h,id);return i<0?XL330_NOT_FOUND:xl330_mode_i2(h,(uint8_t)i,m);} XL330_StatusTypeDef XL330_SetModeByIndex(XL330_HandleTypeDef*h,uint8_t i,uint8_t m){return xl330_mode_i2(h,i,m);} XL330_StatusTypeDef XL330_SetModeAll(XL330_HandleTypeDef*h,uint8_t m){if(!h)return XL330_INVALID_PARAM;for(uint8_t i=0;i<h->motor_count;++i)(void)xl330_mode_i2(h,i,m);return XL330_OK;}
XL330_StatusTypeDef XL330_SetTorque(XL330_HandleTypeDef*h,uint8_t id,uint8_t e){int8_t i=xl330_find_index_h(h,id);return i<0?XL330_NOT_FOUND:xl330_torque_i2(h,(uint8_t)i,e);} XL330_StatusTypeDef XL330_SetTorqueByIndex(XL330_HandleTypeDef*h,uint8_t i,uint8_t e){return xl330_torque_i2(h,i,e);} XL330_StatusTypeDef XL330_SetTorqueAll(XL330_HandleTypeDef*h,uint8_t e){if(!h)return XL330_INVALID_PARAM;for(uint8_t i=0;i<h->motor_count;++i)(void)xl330_torque_i2(h,i,e);return XL330_OK;}
static XL330_StatusTypeDef xl330_goal_i2(XL330_HandleTypeDef*h,uint8_t i,XL330_GoalKind k,int32_t v){if(!h||i>=h->motor_count)return XL330_INVALID_PARAM;h->staged_goal_kind[i]=k;if(k==XL330_GOAL_CURRENT)h->staged_goal_current[i]=(int16_t)v;else if(k==XL330_GOAL_VELOCITY)h->staged_goal_velocity[i]=v;else if(k==XL330_GOAL_POSITION)h->staged_goal_position[i]=v;else if(k==XL330_GOAL_PWM)h->staged_goal_pwm[i]=(int16_t)v;h->pending_goal_mask|=XL330_PENDING_GOAL(i);return XL330_OK;}
XL330_StatusTypeDef XL330_SetGoalPos(XL330_HandleTypeDef*h,uint8_t id,int32_t v){int8_t i=xl330_find_index_h(h,id);return i<0?XL330_NOT_FOUND:xl330_goal_i2(h,(uint8_t)i,XL330_GOAL_POSITION,v);} XL330_StatusTypeDef XL330_SetGoalVel(XL330_HandleTypeDef*h,uint8_t id,int32_t v){int8_t i=xl330_find_index_h(h,id);return i<0?XL330_NOT_FOUND:xl330_goal_i2(h,(uint8_t)i,XL330_GOAL_VELOCITY,v);} XL330_StatusTypeDef XL330_SetGoalCur(XL330_HandleTypeDef*h,uint8_t id,int16_t v){int8_t i=xl330_find_index_h(h,id);return i<0?XL330_NOT_FOUND:xl330_goal_i2(h,(uint8_t)i,XL330_GOAL_CURRENT,v);} XL330_StatusTypeDef XL330_SetGoalPwm(XL330_HandleTypeDef*h,uint8_t id,int16_t v){int8_t i=xl330_find_index_h(h,id);return i<0?XL330_NOT_FOUND:xl330_goal_i2(h,(uint8_t)i,XL330_GOAL_PWM,v);}
XL330_StatusTypeDef XL330_SetGoalPosByIndex(XL330_HandleTypeDef*h,uint8_t i,int32_t v){return xl330_goal_i2(h,i,XL330_GOAL_POSITION,v);} XL330_StatusTypeDef XL330_SetGoalVelByIndex(XL330_HandleTypeDef*h,uint8_t i,int32_t v){return xl330_goal_i2(h,i,XL330_GOAL_VELOCITY,v);} XL330_StatusTypeDef XL330_SetGoalCurByIndex(XL330_HandleTypeDef*h,uint8_t i,int16_t v){return xl330_goal_i2(h,i,XL330_GOAL_CURRENT,v);} XL330_StatusTypeDef XL330_SetGoalPwmByIndex(XL330_HandleTypeDef*h,uint8_t i,int16_t v){return xl330_goal_i2(h,i,XL330_GOAL_PWM,v);}
XL330_StatusTypeDef XL330_SetProfileVelocity(XL330_HandleTypeDef*h,uint8_t id,uint32_t v){return XL330_WriteU32(h,id,XL330_ADDR_PROFILE_VELOCITY,v);} XL330_StatusTypeDef XL330_SetProfileAcceleration(XL330_HandleTypeDef*h,uint8_t id,uint32_t v){return XL330_WriteU32(h,id,XL330_ADDR_PROFILE_ACCELERATION,v);} XL330_StatusTypeDef XL330_SetProfile(XL330_HandleTypeDef*h,uint8_t id,uint32_t v,uint32_t a){XL330_StatusTypeDef s=XL330_SetProfileVelocity(h,id,v);return s==XL330_OK?XL330_SetProfileAcceleration(h,id,a):s;}
static DXL2_Status xl330_commit_goal2(XL330_HandleTypeDef*h,uint8_t i){uint8_t id=h->storage.ids[i];switch(h->staged_goal_kind[i]){case XL330_GOAL_CURRENT:return dxl2_write_u16(&h->bus,id,XL330_ADDR_GOAL_CURRENT,(uint16_t)h->staged_goal_current[i]);case XL330_GOAL_VELOCITY:return dxl2_write_u32(&h->bus,id,XL330_ADDR_GOAL_VELOCITY,(uint32_t)h->staged_goal_velocity[i]);case XL330_GOAL_POSITION:return dxl2_write_u32(&h->bus,id,XL330_ADDR_GOAL_POSITION,(uint32_t)h->staged_goal_position[i]);case XL330_GOAL_PWM:return dxl2_write_u16(&h->bus,id,XL330_ADDR_GOAL_PWM,(uint16_t)h->staged_goal_pwm[i]);default:return DXL2_OK;}}
XL330_StatusTypeDef XL330_Commit(XL330_HandleTypeDef*h){if(!h||h->motor_count==0u)return xl330_hal_store(h,DXL2_ERR_BAD_PARAM,0u,0u,0u);if(h->pending_config_mask==0u&&h->pending_goal_mask==0u){h->last_report.status=XL330_NO_PENDING_CHANGES;h->last_status=DXL2_OK;return XL330_NO_PENDING_CHANGES;}DXL2_Status first=DXL2_OK;for(uint8_t i=0;i<h->motor_count;++i)if(h->pending_config_mask&XL330_PENDING_MODE(i)){(void)dxl2_write_u8(&h->bus,h->storage.ids[i],XL330_ADDR_TORQUE_ENABLE,XL330_TORQUE_DISABLE);DXL2_Status s=dxl2_write_u8(&h->bus,h->storage.ids[i],XL330_ADDR_OPERATING_MODE,h->staged_mode[i]);if(s==DXL2_OK)h->storage.motors[i].operating_mode=h->staged_mode[i];else if(first==DXL2_OK)first=s;}for(uint8_t i=0;i<h->motor_count;++i)if(h->pending_config_mask&XL330_PENDING_TORQUE(i)){DXL2_Status s=dxl2_write_u8(&h->bus,h->storage.ids[i],XL330_ADDR_TORQUE_ENABLE,h->staged_torque[i]);if(s==DXL2_OK)h->storage.motors[i].torque_enable=h->staged_torque[i];else if(first==DXL2_OK)first=s;}for(uint8_t i=0;i<h->motor_count;++i)if(h->pending_goal_mask&XL330_PENDING_GOAL(i)){DXL2_Status s=xl330_commit_goal2(h,i);if(s!=DXL2_OK&&first==DXL2_OK)first=s;}if(first==DXL2_OK){h->pending_config_mask=0u;h->pending_goal_mask=0u;for(uint8_t i=0;i<h->motor_count;++i)h->staged_goal_kind[i]=XL330_GOAL_NONE;}return xl330_hal_store(h,first,0u,0u,0u);}

XL330_StatusTypeDef XL330_Update(XL330_HandleTypeDef *h,
                                 XL330_LayoutId layout_id,
                                 XL330_UpdateReport *out_reports,
                                 uint8_t max_reports,
                                 uint8_t *report_count)
{
    if (!h) return XL330_INVALID_PARAM;
    XL330_StatusTypeDef status = XL330_UpdateLayout(h, layout_id, 0, 0u);
    if (report_count) *report_count = 0u;
    if (status != XL330_OK && status != XL330_DEVICE_ERROR) return status;

    uint8_t item_count = 0u;
    const XL330_Item *items = xl330_find_layout_items(h, layout_id, &item_count);
    if (!items) return XL330_LAYOUT_NOT_FOUND;

    uint8_t count = h->motor_count;
    if (out_reports && max_reports < count) count = max_reports;
    for (uint8_t m = 0; out_reports && m < count; ++m) {
        out_reports[m].id = h->storage.ids[m];
        out_reports[m].item_count = item_count;
        out_reports[m].report = h->storage.reports[m];
        for (uint8_t i = 0; i < item_count && i < XL330_MAX_LAYOUT_ITEMS; ++i) {
            out_reports[m].items[i].item = items[i];
            out_reports[m].items[i].valid = 1u;
            switch (items[i]) {
            case XL330_CUR: out_reports[m].items[i].value = h->storage.states[m].present_current; break;
            case XL330_VEL: out_reports[m].items[i].value = h->storage.states[m].present_velocity; break;
            case XL330_POS: out_reports[m].items[i].value = h->storage.states[m].present_position; break;
            default: out_reports[m].items[i].valid = 0u; out_reports[m].items[i].value = 0; break;
            }
        }
    }
    if (report_count) *report_count = count;
    return status;
}

XL330_StatusTypeDef XL330_GetState(const XL330_HandleTypeDef*h,uint8_t id,XL330_State*s){int8_t i=xl330_find_index_h(h,id);if(i<0||!s)return XL330_NOT_FOUND;*s=h->storage.states[i];return XL330_OK;} const XL330_State *XL330_GetCachedState(const XL330_HandleTypeDef*h,uint8_t id){int8_t i=xl330_find_index_h(h,id);return i<0?0:&h->storage.states[i];} const XL330_StatusReport *XL330_GetStatusReport(const XL330_HandleTypeDef*h,uint8_t id){int8_t i=xl330_find_index_h(h,id);return i<0?0:&h->storage.reports[i];} const XL330_Report *XL330_GetLastReport(const XL330_HandleTypeDef*h){return h?&h->last_report:0;}
XL330_StatusTypeDef XL330_ReadRaw(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,uint16_t l,uint8_t*d){
		return(h&&d)?xl330_hal_store(h,dxl2_read(&h->bus,id,a,l,d),id,0u,a):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_WriteRaw(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,const uint8_t*d,uint16_t l){return(h&&d)?xl330_hal_store(h,dxl2_write(&h->bus,id,a,d,l),id,0u,a):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_ReadU8(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,uint8_t*v){return(h&&v)?xl330_hal_store(h,dxl2_read_u8(&h->bus,id,a,v),id,0u,a):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_ReadU16(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,uint16_t*v){return(h&&v)?xl330_hal_store(h,dxl2_read_u16(&h->bus,id,a,v),id,0u,a):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_ReadU32(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,uint32_t*v){return(h&&v)?xl330_hal_store(h,dxl2_read_u32(&h->bus,id,a,v),id,0u,a):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_WriteU8(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,uint8_t v){return h?xl330_hal_store(h,dxl2_write_u8(&h->bus,id,a,v),id,0u,a):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_WriteU16(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,uint16_t v){return h?xl330_hal_store(h,dxl2_write_u16(&h->bus,id,a,v),id,0u,a):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_WriteU32(XL330_HandleTypeDef*h,uint8_t id,uint16_t a,uint32_t v){return h?xl330_hal_store(h,dxl2_write_u32(&h->bus,id,a,v),id,0u,a):XL330_INVALID_PARAM;}
XL330_StatusTypeDef XL330_Ping(XL330_HandleTypeDef*h,uint8_t id,DXL2_DeviceInfo*info){return h?xl330_hal_store(h,dxl2_ping(&h->bus,id,info),id,0u,0u):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_Rescan(XL330_HandleTypeDef*h,uint8_t f,uint8_t l,XL330_DeviceReport*d,uint8_t max,uint8_t*c){if(!h||!c||f>l)return XL330_INVALID_PARAM;*c=0u;for(uint16_t id=f;id<=l;++id){DXL2_DeviceInfo info;DXL2_Status ps=dxl2_ping(&h->bus,(uint8_t)id,&info);if(ps==DXL2_OK){if(*c<max&&d){d[*c].id=(uint8_t)id;d[*c].info=info;}*c=(uint8_t)(*c+1u);}}return *c?XL330_OK:XL330_NOT_FOUND;} XL330_StatusTypeDef XL330_ScanHalfDuplex(const XL330_HalfDuplexInitTypeDef*init,uint8_t f,uint8_t l,XL330_DeviceReport*d,uint8_t max,uint8_t*c){XL330_HandleTypeDef h;XL330_StatusTypeDef s=XL330_InitHalfDuplex(&h,init);return s==XL330_OK?XL330_Rescan(&h,f,l,d,max,c):s;} XL330_StatusTypeDef XL330_Scan(UART_HandleTypeDef*huart,uint8_t f,uint8_t l,XL330_DeviceReport*d,uint8_t max,uint8_t*c){XL330_HalfDuplexInitTypeDef init={huart,0,0u,GPIO_PIN_SET,XL330_DefaultStorageConfig(),0u};return XL330_ScanHalfDuplex(&init,f,l,d,max,c);}
XL330_StatusTypeDef XL330_CommitAsyncStart(XL330_HandleTypeDef*h){if(!h)return XL330_INVALID_PARAM;h->async_state=XL330_ASYNC_BUSY;XL330_StatusTypeDef s=XL330_Commit(h);h->async_state=(s==XL330_OK||s==XL330_NO_PENDING_CHANGES)?XL330_ASYNC_DONE:XL330_ASYNC_ERROR;return s;} XL330_StatusTypeDef XL330_UpdateAsyncStart(XL330_HandleTypeDef*h,XL330_LayoutId id){if(!h)return XL330_INVALID_PARAM;h->async_state=XL330_ASYNC_BUSY;XL330_StatusTypeDef s=XL330_Update(h,id,0,0,0);h->async_state=s==XL330_OK?XL330_ASYNC_DONE:XL330_ASYNC_ERROR;return s;} XL330_StatusTypeDef XL330_AsyncPoll(XL330_HandleTypeDef*h){return h?(h->async_state==XL330_ASYNC_ERROR?XL330_ERROR:XL330_OK):XL330_INVALID_PARAM;} XL330_StatusTypeDef XL330_AsyncCancel(XL330_HandleTypeDef*h){if(!h)return XL330_INVALID_PARAM;h->async_state=XL330_ASYNC_IDLE;return XL330_OK;} XL330_AsyncState XL330_GetAsyncState(const XL330_HandleTypeDef*h){return h?h->async_state:XL330_ASYNC_ERROR;} const char *XL330_GetStatusString(XL330_StatusTypeDef s){switch(s){case XL330_OK:return"OK";case XL330_TIMEOUT:return"timeout";case XL330_INVALID_PARAM:return"invalid_param";case XL330_NOT_FOUND:return"not_found";case XL330_NO_PENDING_CHANGES:return"no_pending_changes";case XL330_STORAGE_FULL:return"storage_full";case XL330_LAYOUT_NOT_FOUND:return"layout_not_found";case XL330_UNSUPPORTED:return"unsupported";case XL330_BUS_ERROR:return"bus_error";case XL330_DEVICE_ERROR:return"device_error";default:return"error";}}
