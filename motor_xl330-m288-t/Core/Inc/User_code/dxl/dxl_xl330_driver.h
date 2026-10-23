#ifndef DXL_XL330_DRIVER_H_
#define DXL_XL330_DRIVER_H_

#include <stddef.h>
#include <stdint.h>

#include "dxl_protocol2.h"
#include "dxl_stm32_hal.h"
#include "dxl_xl330.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef XL330_MAX_MOTORS
#define XL330_MAX_MOTORS 8u
#endif

#ifndef XL330_MAX_LAYOUTS
#define XL330_MAX_LAYOUTS 4u
#endif

#ifndef XL330_MAX_LAYOUT_ITEMS
#define XL330_MAX_LAYOUT_ITEMS 8u
#endif

#ifndef XL330_ENABLE
#define XL330_ENABLE 1u
#endif

#ifndef XL330_DISABLE
#define XL330_DISABLE 0u
#endif

/* -------------------------------------------------------------------------- */
/* HAL-style public status model                                               */
/* -------------------------------------------------------------------------- */

typedef enum {
    XL330_OK = 0,
    XL330_ERROR,
    XL330_BUSY,
    XL330_TIMEOUT,
    XL330_INVALID_PARAM,
    XL330_INVALID_STATE,
    XL330_NOT_FOUND,
    XL330_NO_PENDING_CHANGES,
    XL330_STORAGE_FULL,
    XL330_LAYOUT_NOT_FOUND,
    XL330_UNSUPPORTED,
    XL330_BUS_ERROR,
    XL330_DEVICE_ERROR
} XL330_StatusTypeDef;

typedef enum {
    XL330_LAYOUT_DEFAULT = 0,
    XL330_LAYOUT_FAST = 1,
    XL330_LAYOUT_STATE = 2,
    XL330_LAYOUT_HEALTH = 3,
    XL330_LAYOUT_MOTION = 4,
    XL330_LAYOUT_CUSTOM_BASE = 32
} XL330_LayoutId;

/* Backwards-friendly short preset aliases from the spec examples. */
#define XL330_DEFAULT XL330_LAYOUT_DEFAULT
#define XL330_FAST    XL330_LAYOUT_FAST
#define XL330_STATE   XL330_LAYOUT_STATE
#define XL330_HEALTH  XL330_LAYOUT_HEALTH
#define XL330_MOTION  XL330_LAYOUT_MOTION

typedef enum {
    XL330_ITEM_NONE = 0,
    XL330_POS,
    XL330_VEL,
    XL330_CUR,
    XL330_PWM,
    XL330_VOLT,
    XL330_TEMP,
    XL330_MOVING,
    XL330_MOVING_STATUS,
    XL330_HW_ERROR,
    XL330_TICK,
    XL330_GOAL_POS_ITEM,
    XL330_GOAL_VEL_ITEM,
    XL330_GOAL_CUR_ITEM,
    XL330_GOAL_PWM_ITEM
} XL330_Item;

typedef enum {
    XL330_GOAL_NONE = 0,
    XL330_GOAL_CURRENT,
    XL330_GOAL_VELOCITY,
    XL330_GOAL_POSITION,
    XL330_GOAL_PWM
} XL330_GoalKind;

typedef enum {
    XL330_ASYNC_IDLE = 0,
    XL330_ASYNC_BUSY,
    XL330_ASYNC_DONE,
    XL330_ASYNC_ERROR
} XL330_AsyncState;

typedef struct XL330_HandleTypeDef XL330_HandleTypeDef;
typedef XL330_HandleTypeDef XL330_Handle;

typedef struct {
    XL330_StatusTypeDef status;
    DXL2_Status protocol_status;
    uint8_t protocol_error;
    uint8_t id;
    uint8_t index;
    uint8_t alert;
    uint8_t hardware_error;
    uint16_t address;
} XL330_Report;

typedef struct {
    uint8_t id;
    DXL2_Status status;
    uint8_t error;
    uint8_t alert;
    uint8_t error_code;
} XL330_StatusReport;

typedef struct {
    int16_t present_current;
    int32_t present_velocity;
    int32_t present_position;
} XL330_State;

/* -------------------------------------------------------------------------- */
/* Storage/configuration                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t id;
    uint8_t index;
    uint8_t present;
    uint8_t operating_mode;
    uint8_t torque_enable;
    uint32_t goal_capability_mask;
} XL330_MotorSlot;

typedef struct {
    XL330_LayoutId id;
    const char *name;
    const XL330_Item *items;
    uint8_t item_count;
} XL330_LayoutDef;

typedef struct {
    uint8_t *ids;
    uint8_t max_motors;
    XL330_MotorSlot *motors;
    XL330_State *states;
    XL330_StatusReport *reports;
    XL330_LayoutDef *layouts;
    uint8_t max_layouts;
    XL330_Item *layout_items;
    uint8_t max_layout_items;
} XL330_StorageConfig;

typedef struct {
    UART_HandleTypeDef *huart;
    const XL330_StorageConfig *storage;
    uint32_t timeout_ms;
} XL330_InitTypeDef;

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_PinState tx_state;
    const XL330_StorageConfig *storage;
    uint32_t timeout_ms;
} XL330_HalfDuplexInitTypeDef;

typedef struct {
    XL330_Item item;
    int32_t value;
    uint8_t valid;
} XL330_ItemValue;

typedef struct {
    uint8_t id;
    uint8_t item_count;
    XL330_ItemValue items[XL330_MAX_LAYOUT_ITEMS];
    XL330_StatusReport report;
} XL330_UpdateReport;

typedef struct {
    uint8_t id;
    DXL2_DeviceInfo info;
    XL330_StatusReport report;
} XL330_DeviceReport;

struct XL330_HandleTypeDef {
    DXL2_Transport transport;
    DXL2_STM32HAL_Context hal;
    DXL2_Bus bus;
    XL330_StorageConfig storage;
    uint8_t motor_count;
    uint8_t layout_count;
    uint32_t pending_config_mask;
    uint32_t pending_goal_mask;
    uint8_t staged_mode[XL330_MAX_MOTORS];
    uint8_t staged_torque[XL330_MAX_MOTORS];
    XL330_GoalKind staged_goal_kind[XL330_MAX_MOTORS];
    int16_t staged_goal_current[XL330_MAX_MOTORS];
    int32_t staged_goal_velocity[XL330_MAX_MOTORS];
    int32_t staged_goal_position[XL330_MAX_MOTORS];
    int16_t staged_goal_pwm[XL330_MAX_MOTORS];
    XL330_Report last_report;
    DXL2_Status last_status;
    XL330_AsyncState async_state;
    void *user_context;
};

/* -------------------------------------------------------------------------- */
/* Compatibility handle types                                                  */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t id;
    DXL2_Transport transport;
    DXL2_STM32HAL_Context hal;
    DXL2_Bus bus;
    DXL2_Status last_status;
} XL330_Motor;

typedef struct {
    const uint8_t *ids;
    uint8_t count;
    DXL2_Transport transport;
    DXL2_STM32HAL_Context hal;
    DXL2_Bus bus;
    DXL2_Status last_status;
} XL330_Group;

typedef struct {
    XL330_Group group;
    uint8_t ids[XL330_MAX_MOTORS];
    uint8_t count;
    uint8_t mode;
    uint8_t mode_dirty;
    uint8_t torque_enable;
    uint8_t torque_dirty;
    XL330_GoalKind goal_kind;
    int16_t goal_current[XL330_MAX_MOTORS];
    int32_t goal_velocity[XL330_MAX_MOTORS];
    int32_t goal_position[XL330_MAX_MOTORS];
    XL330_State states[XL330_MAX_MOTORS];
    XL330_StatusReport reports[XL330_MAX_MOTORS];
    DXL2_Status last_status;
} XL330_Controller;

/* -------------------------------------------------------------------------- */
/* HAL-style lifecycle and discovery                                           */
/* -------------------------------------------------------------------------- */

XL330_StatusTypeDef XL330_Init(XL330_HandleTypeDef *hxl330,
                               UART_HandleTypeDef *huart,
                               const XL330_StorageConfig *storage);
XL330_StatusTypeDef XL330_InitDefault(XL330_HandleTypeDef *hxl330, UART_HandleTypeDef *huart);
const XL330_StorageConfig *XL330_DefaultStorageConfig(void);
XL330_StatusTypeDef XL330_InitEx(XL330_HandleTypeDef *hxl330,
                                 const XL330_InitTypeDef *init);
XL330_StatusTypeDef XL330_InitHalfDuplex(XL330_HandleTypeDef *hxl330,
                                         const XL330_HalfDuplexInitTypeDef *init);
XL330_StatusTypeDef XL330_DeInit(XL330_HandleTypeDef *hxl330);
XL330_StatusTypeDef XL330_AddMotor(XL330_HandleTypeDef *hxl330, uint8_t id);
XL330_StatusTypeDef XL330_RemoveMotor(XL330_HandleTypeDef *hxl330, uint8_t id);
XL330_StatusTypeDef XL330_ClearMotors(XL330_HandleTypeDef *hxl330);
XL330_StatusTypeDef XL330_Scan(UART_HandleTypeDef *huart,
                               uint8_t first_id,
                               uint8_t last_id,
                               XL330_DeviceReport *devices,
                               uint8_t max_devices,
                               uint8_t *device_count);
XL330_StatusTypeDef XL330_ScanHalfDuplex(const XL330_HalfDuplexInitTypeDef *init,
                                         uint8_t first_id,
                                         uint8_t last_id,
                                         XL330_DeviceReport *devices,
                                         uint8_t max_devices,
                                         uint8_t *device_count);
XL330_StatusTypeDef XL330_Rescan(XL330_HandleTypeDef *hxl330,
                                 uint8_t first_id,
                                 uint8_t last_id,
                                 XL330_DeviceReport *devices,
                                 uint8_t max_devices,
                                 uint8_t *device_count);
XL330_StatusTypeDef XL330_Ping(XL330_HandleTypeDef *hxl330, uint8_t id, DXL2_DeviceInfo *info);

/* -------------------------------------------------------------------------- */
/* Layout registration and staged configuration                                */
/* -------------------------------------------------------------------------- */

XL330_StatusTypeDef XL330_AddLayout(XL330_HandleTypeDef *hxl330,
                                    XL330_LayoutId layout_id,
                                    const XL330_Item *items,
                                    uint8_t item_count);
XL330_StatusTypeDef XL330_AddNamedLayout(XL330_HandleTypeDef *hxl330,
                                         XL330_LayoutId layout_id,
                                         const char *name,
                                         const XL330_Item *items,
                                         uint8_t item_count);
XL330_StatusTypeDef XL330_RemoveLayout(XL330_HandleTypeDef *hxl330, XL330_LayoutId layout_id);
XL330_StatusTypeDef XL330_SetMode(XL330_HandleTypeDef *hxl330, uint8_t id, uint8_t mode);
XL330_StatusTypeDef XL330_SetModeByIndex(XL330_HandleTypeDef *hxl330, uint8_t index, uint8_t mode);
XL330_StatusTypeDef XL330_SetModeAll(XL330_HandleTypeDef *hxl330, uint8_t mode);
XL330_StatusTypeDef XL330_SetTorque(XL330_HandleTypeDef *hxl330, uint8_t id, uint8_t enable);
XL330_StatusTypeDef XL330_SetTorqueByIndex(XL330_HandleTypeDef *hxl330, uint8_t index, uint8_t enable);
XL330_StatusTypeDef XL330_SetTorqueAll(XL330_HandleTypeDef *hxl330, uint8_t enable);
XL330_StatusTypeDef XL330_SetProfileVelocity(XL330_HandleTypeDef *hxl330, uint8_t id, uint32_t velocity);
XL330_StatusTypeDef XL330_SetProfileAcceleration(XL330_HandleTypeDef *hxl330, uint8_t id, uint32_t acceleration);
XL330_StatusTypeDef XL330_SetProfile(XL330_HandleTypeDef *hxl330,
                                     uint8_t id,
                                     uint32_t velocity,
                                     uint32_t acceleration);

/* -------------------------------------------------------------------------- */
/* Staged realtime goals and smart blocking I/O                                */
/* -------------------------------------------------------------------------- */

XL330_StatusTypeDef XL330_SetGoalPos(XL330_HandleTypeDef *hxl330, uint8_t id, int32_t position_pulse);
XL330_StatusTypeDef XL330_SetGoalVel(XL330_HandleTypeDef *hxl330, uint8_t id, int32_t velocity_raw);
XL330_StatusTypeDef XL330_SetGoalCur(XL330_HandleTypeDef *hxl330, uint8_t id, int16_t current_ma);
XL330_StatusTypeDef XL330_SetGoalPwm(XL330_HandleTypeDef *hxl330, uint8_t id, int16_t pwm_raw);
XL330_StatusTypeDef XL330_SetGoalPosByIndex(XL330_HandleTypeDef *hxl330, uint8_t index, int32_t position_pulse);
XL330_StatusTypeDef XL330_SetGoalVelByIndex(XL330_HandleTypeDef *hxl330, uint8_t index, int32_t velocity_raw);
XL330_StatusTypeDef XL330_SetGoalCurByIndex(XL330_HandleTypeDef *hxl330, uint8_t index, int16_t current_ma);
XL330_StatusTypeDef XL330_SetGoalPwmByIndex(XL330_HandleTypeDef *hxl330, uint8_t index, int16_t pwm_raw);
XL330_StatusTypeDef XL330_Commit(XL330_HandleTypeDef *hxl330);
XL330_StatusTypeDef XL330_Update(XL330_HandleTypeDef *hxl330,
                                 XL330_LayoutId layout_id,
                                 XL330_UpdateReport *out_reports,
                                 uint8_t max_reports,
                                 uint8_t *report_count);
/*
 * Simple read-layout buffer API. Rows are motor-major in AddMotor order.
 * Each row is XL330_GetLayoutStride(layout_id) bytes. Item values are appended
 * in layout order as raw little-endian control-table bytes with no header.
 */
XL330_StatusTypeDef XL330_UpdateLayout(XL330_HandleTypeDef *hxl330,
                                       XL330_LayoutId layout_id,
                                       uint8_t *user_buf,
                                       uint16_t user_buf_len);
uint16_t XL330_GetLayoutStride(XL330_LayoutId layout_id);
XL330_StatusTypeDef XL330_GetState(const XL330_HandleTypeDef *hxl330, uint8_t id, XL330_State *state);
const XL330_State *XL330_GetCachedState(const XL330_HandleTypeDef *hxl330, uint8_t id);
const XL330_StatusReport *XL330_GetStatusReport(const XL330_HandleTypeDef *hxl330, uint8_t id);
const XL330_Report *XL330_GetLastReport(const XL330_HandleTypeDef *hxl330);
const char *XL330_GetStatusString(XL330_StatusTypeDef status);

/* -------------------------------------------------------------------------- */
/* Async-ready placeholders. Non-blocking implementation may be phased in.      */
/* -------------------------------------------------------------------------- */

XL330_StatusTypeDef XL330_CommitAsyncStart(XL330_HandleTypeDef *hxl330);
XL330_StatusTypeDef XL330_UpdateAsyncStart(XL330_HandleTypeDef *hxl330, XL330_LayoutId layout_id);
XL330_StatusTypeDef XL330_AsyncPoll(XL330_HandleTypeDef *hxl330);
XL330_StatusTypeDef XL330_AsyncCancel(XL330_HandleTypeDef *hxl330);
XL330_AsyncState XL330_GetAsyncState(const XL330_HandleTypeDef *hxl330);

/* -------------------------------------------------------------------------- */
/* Expert raw-access API. Prefer staged config/goal APIs for normal use.        */
/* -------------------------------------------------------------------------- */

XL330_StatusTypeDef XL330_ReadRaw(XL330_HandleTypeDef *hxl330,
                                  uint8_t id,
                                  uint16_t address,
                                  uint16_t length,
                                  uint8_t *data);
XL330_StatusTypeDef XL330_WriteRaw(XL330_HandleTypeDef *hxl330,
                                   uint8_t id,
                                   uint16_t address,
                                   const uint8_t *data,
                                   uint16_t length);
XL330_StatusTypeDef XL330_ReadU8(XL330_HandleTypeDef *hxl330, uint8_t id, uint16_t address, uint8_t *value);
XL330_StatusTypeDef XL330_ReadU16(XL330_HandleTypeDef *hxl330, uint8_t id, uint16_t address, uint16_t *value);
XL330_StatusTypeDef XL330_ReadU32(XL330_HandleTypeDef *hxl330, uint8_t id, uint16_t address, uint32_t *value);
XL330_StatusTypeDef XL330_WriteU8(XL330_HandleTypeDef *hxl330, uint8_t id, uint16_t address, uint8_t value);
XL330_StatusTypeDef XL330_WriteU16(XL330_HandleTypeDef *hxl330, uint8_t id, uint16_t address, uint16_t value);
XL330_StatusTypeDef XL330_WriteU32(XL330_HandleTypeDef *hxl330, uint8_t id, uint16_t address, uint32_t value);

/* -------------------------------------------------------------------------- */
/* Legacy API compatibility declarations                                       */
/* -------------------------------------------------------------------------- */

DXL2_Status xl330_init(XL330_Motor *motor, UART_HandleTypeDef *huart, uint8_t id);
DXL2_Status xl330_init_half_duplex(XL330_Motor *motor,
                                   UART_HandleTypeDef *huart,
                                   uint8_t id,
                                   GPIO_TypeDef *dir_port,
                                   uint16_t dir_pin,
                                   GPIO_PinState tx_state);
DXL2_Status xl330_ping(XL330_Motor *motor, DXL2_DeviceInfo *info);
DXL2_Status xl330_scan(UART_HandleTypeDef *huart,
                       uint8_t first_id,
                       uint8_t last_id,
                       uint8_t *found_ids,
                       uint8_t max_found,
                       uint8_t *found_count);
DXL2_Status xl330_set_torque(XL330_Motor *motor, uint8_t enable);
DXL2_Status xl330_enable_torque(XL330_Motor *motor);
DXL2_Status xl330_disable_torque(XL330_Motor *motor);
DXL2_Status xl330_set_operating_mode(XL330_Motor *motor, uint8_t mode);
DXL2_Status xl330_set_current_mode(XL330_Motor *motor);
DXL2_Status xl330_set_goal_current(XL330_Motor *motor, int16_t current_ma);
DXL2_Status xl330_set_goal_velocity(XL330_Motor *motor, int32_t velocity_raw);
DXL2_Status xl330_set_goal_position(XL330_Motor *motor, int32_t position_pulse);
DXL2_Status xl330_write_u8(XL330_Motor *motor, uint16_t address, uint8_t value);
DXL2_Status xl330_write_u16(XL330_Motor *motor, uint16_t address, uint16_t value);
DXL2_Status xl330_write_u32(XL330_Motor *motor, uint16_t address, uint32_t value);
DXL2_Status xl330_read_u8(XL330_Motor *motor, uint16_t address, uint8_t *value);
DXL2_Status xl330_read_u16(XL330_Motor *motor, uint16_t address, uint16_t *value);
DXL2_Status xl330_read_u32(XL330_Motor *motor, uint16_t address, uint32_t *value);
DXL2_Status xl330_read_state(XL330_Motor *motor, XL330_State *state);
DXL2_Status xl330_read_present_current(XL330_Motor *motor, int16_t *current_ma);
DXL2_Status xl330_read_present_velocity(XL330_Motor *motor, int32_t *velocity_raw);
DXL2_Status xl330_read_present_position(XL330_Motor *motor, int32_t *position_pulse);

DXL2_Status xl330_group_init(XL330_Group *group,
                             UART_HandleTypeDef *huart,
                             const uint8_t *ids,
                             uint8_t count);
DXL2_Status xl330_group_init_half_duplex(XL330_Group *group,
                                         UART_HandleTypeDef *huart,
                                         const uint8_t *ids,
                                         uint8_t count,
                                         GPIO_TypeDef *dir_port,
                                         uint16_t dir_pin,
                                         GPIO_PinState tx_state);
DXL2_Status xl330_group_scan(XL330_Group *group,
                             uint8_t first_id,
                             uint8_t last_id,
                             uint8_t *found_ids,
                             uint8_t max_found,
                             uint8_t *found_count,
                             XL330_StatusReport *reports);
DXL2_Status xl330_group_sync_read_state(XL330_Group *group,
                                        XL330_State *states,
                                        XL330_StatusReport *reports);
DXL2_Status xl330_group_sync_write_goal_current(XL330_Group *group,
                                                const int16_t *currents_ma);
DXL2_Status xl330_group_sync_write_goal_velocity(XL330_Group *group,
                                                 const int32_t *velocities_raw);
DXL2_Status xl330_group_sync_write_goal_position(XL330_Group *group,
                                                 const int32_t *positions_pulse);
DXL2_Status xl330_group_torque_all(XL330_Group *group, uint8_t enable);
DXL2_Status xl330_group_mode_all(XL330_Group *group, uint8_t mode);
DXL2_Status xl330_group_current_mode_all(XL330_Group *group);
DXL2_Status xl330_group_velocity_mode_all(XL330_Group *group);
DXL2_Status xl330_group_position_mode_all(XL330_Group *group);
DXL2_Status xl330_group_safe_status(const XL330_Group *group,
                                    DXL2_Status bus_status,
                                    const DXL2_GroupResult *results,
                                    XL330_StatusReport *reports);
const char *xl330_status_string(DXL2_Status status);

DXL2_Status xl330_begin(XL330_Controller *ctl, UART_HandleTypeDef *huart);
DXL2_Status xl330_begin_half_duplex(XL330_Controller *ctl,
                                    UART_HandleTypeDef *huart,
                                    GPIO_TypeDef *dir_port,
                                    uint16_t dir_pin,
                                    GPIO_PinState tx_state);
DXL2_Status xl330_add(XL330_Controller *ctl, uint8_t id);
DXL2_Status xl330_mode(XL330_Controller *ctl, uint8_t mode);
DXL2_Status xl330_torque(XL330_Controller *ctl, uint8_t enable);
DXL2_Status xl330_goal_current(XL330_Controller *ctl, uint8_t id, int16_t current_ma);
DXL2_Status xl330_goal_velocity(XL330_Controller *ctl, uint8_t id, int32_t velocity_raw);
DXL2_Status xl330_goal_position(XL330_Controller *ctl, uint8_t id, int32_t position_pulse);
DXL2_Status xl330_commit(XL330_Controller *ctl);
DXL2_Status xl330_update(XL330_Controller *ctl);
const XL330_State *xl330_state(const XL330_Controller *ctl, uint8_t id);
const XL330_StatusReport *xl330_report(const XL330_Controller *ctl, uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* DXL_XL330_DRIVER_H_ */
