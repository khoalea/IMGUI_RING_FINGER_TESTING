/*
 * calibration.h
 *
 *  Created on: Feb 10, 2026
 *      Author: vinbui
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
float zero;
float scale;
float min_angle;
float max_angle;
} JointParams;

typedef struct {
float torque_rate;
float torque_gain;
float torque_limit;
} MotorParams;

typedef struct {
JointParams joints[2];
MotorParams motor;
uint32_t crc;
} CalibrationData;

// Link to the address in Flash
extern const CalibrationData flash_calib_data;

uint8_t Calib_Read(CalibrationData* dest);
uint8_t Calib_Write(CalibrationData* src);



#endif /* INC_CALIBRATION_H_ */
