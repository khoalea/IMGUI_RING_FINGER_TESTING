#ifndef DXL_XL330_H_
#define DXL_XL330_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ROBOTIS Dynamixel XL330-M288-T control table names.
 * Keep model-specific names here so app code does not use magic numbers.
 */

/* EEPROM area */
#define XL330_ADDR_MODEL_NUMBER                 0u
#define XL330_ADDR_MODEL_INFORMATION            2u
#define XL330_ADDR_FIRMWARE_VERSION             6u
#define XL330_ADDR_ID                           7u
#define XL330_ADDR_BAUD_RATE                    8u
#define XL330_ADDR_RETURN_DELAY_TIME            9u
#define XL330_ADDR_DRIVE_MODE                   10u
#define XL330_ADDR_OPERATING_MODE               11u
#define XL330_ADDR_SECONDARY_ID                 12u
#define XL330_ADDR_PROTOCOL_TYPE                13u
#define XL330_ADDR_HOMING_OFFSET                20u
#define XL330_ADDR_MOVING_THRESHOLD             24u
#define XL330_ADDR_TEMPERATURE_LIMIT            31u
#define XL330_ADDR_MAX_VOLTAGE_LIMIT            32u
#define XL330_ADDR_MIN_VOLTAGE_LIMIT            34u
#define XL330_ADDR_PWM_LIMIT                    36u
#define XL330_ADDR_CURRENT_LIMIT                38u
#define XL330_ADDR_VELOCITY_LIMIT               44u
#define XL330_ADDR_MAX_POSITION_LIMIT           48u
#define XL330_ADDR_MIN_POSITION_LIMIT           52u
#define XL330_ADDR_SHUTDOWN                     63u

/* RAM area */
#define XL330_ADDR_TORQUE_ENABLE                64u
#define XL330_ADDR_LED                          65u
#define XL330_ADDR_STATUS_RETURN_LEVEL          68u
#define XL330_ADDR_REGISTERED_INSTRUCTION       69u
#define XL330_ADDR_HARDWARE_ERROR_STATUS        70u
#define XL330_ADDR_VELOCITY_I_GAIN              76u
#define XL330_ADDR_VELOCITY_P_GAIN              78u
#define XL330_ADDR_POSITION_D_GAIN              80u
#define XL330_ADDR_POSITION_I_GAIN              82u
#define XL330_ADDR_POSITION_P_GAIN              84u
#define XL330_ADDR_FEEDFORWARD_2ND_GAIN         88u
#define XL330_ADDR_FEEDFORWARD_1ST_GAIN         90u
#define XL330_ADDR_BUS_WATCHDOG                 98u
#define XL330_ADDR_GOAL_PWM                     100u
#define XL330_ADDR_GOAL_CURRENT                 102u
#define XL330_ADDR_GOAL_VELOCITY                104u
#define XL330_ADDR_PROFILE_ACCELERATION         108u
#define XL330_ADDR_PROFILE_VELOCITY             112u
#define XL330_ADDR_GOAL_POSITION                116u
#define XL330_ADDR_REALTIME_TICK                120u
#define XL330_ADDR_MOVING                       122u
#define XL330_ADDR_MOVING_STATUS                123u
#define XL330_ADDR_PRESENT_PWM                  124u
#define XL330_ADDR_PRESENT_CURRENT              126u
#define XL330_ADDR_PRESENT_VELOCITY             128u
#define XL330_ADDR_PRESENT_POSITION             132u
#define XL330_ADDR_VELOCITY_TRAJECTORY          136u
#define XL330_ADDR_POSITION_TRAJECTORY          140u
#define XL330_ADDR_PRESENT_INPUT_VOLTAGE        144u
#define XL330_ADDR_PRESENT_TEMPERATURE          146u
#define XL330_ADDR_BACKUP_READY                 147u

/* Common data lengths */
#define XL330_LEN_U8                            1u
#define XL330_LEN_U16                           2u
#define XL330_LEN_U32                           4u
#define XL330_LEN_PRESENT_CURRENT_VELOCITY_POSITION 10u

/* Operating Mode values, address XL330_ADDR_OPERATING_MODE */
#define XL330_MODE_CURRENT                      0u
#define XL330_MODE_VELOCITY                     1u
#define XL330_MODE_POSITION                     3u
#define XL330_MODE_EXTENDED_POSITION            4u
#define XL330_MODE_CURRENT_BASED_POSITION       5u
#define XL330_MODE_PWM                          16u

/* Boolean-like register values */
#define XL330_TORQUE_DISABLE                    0u
#define XL330_TORQUE_ENABLE                     1u
#define XL330_LED_OFF                           0u
#define XL330_LED_ON                            1u

/* Status Return Level values */
#define XL330_STATUS_RETURN_PING_ONLY           0u
#define XL330_STATUS_RETURN_READ_ONLY           1u
#define XL330_STATUS_RETURN_ALL                 2u

/* Baud Rate register values used by X-series models */
#define XL330_BAUD_9600                         0u
#define XL330_BAUD_57600                        1u
#define XL330_BAUD_115200                       2u
#define XL330_BAUD_1000000                      3u
#define XL330_BAUD_2000000                      4u
#define XL330_BAUD_3000000                      5u
#define XL330_BAUD_4000000                      6u
#define XL330_BAUD_4500000                      7u

#ifdef __cplusplus
}
#endif

#endif /* DXL_XL330_H_ */
