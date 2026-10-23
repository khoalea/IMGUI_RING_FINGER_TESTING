/*
 * adc_sensors.h
 *
 *  Created on: Feb 6, 2026
 *      Author: vinbui
 */

#ifndef INC_ADC_SENSORS_H_
#define INC_ADC_SENSORS_H_

#include "main.h" // Ensures HAL types (ADC_HandleTypeDef, etc.) are available

// ---- Configuration ----
#define ADC_CHANNELS      3      // Number of converted channels (Rank 1 + Rank 2)
#define SENSOR_LPF_ALPHA  0.1f   // Smoothing factor (0.1 = Very Smooth, 1.0 = No Filter)

// ---- Data Structures ----

/**
 * @brief Represents a single Potentiometer's state
 */
typedef struct {
    // Outputs
    float position;      // Calculated Position (e.g., 0-360 degrees)
    float velocity;      // Calculated Velocity (degrees/sec)
    uint16_t raw_adc;    // Raw ADC value (0-4095)

    // Internal History (for Physics Calc)
    float _prev_position;
    uint32_t _last_timestamp_ms;
} Potentiometer_t;

/**
 * @brief Container for all system sensors
 */
typedef struct {
    Potentiometer_t pot1; // Corresponds to ADC Channel Rank 1
    Potentiometer_t pot2; // Corresponds to ADC Channel Rank 2
    uint16_t vref_raw;
} SystemSensors_t;

// ---- Function Prototypes ----

/**
 * @brief Initializes the ADC DMA and Sensor structs.
 * @param hadc Pointer to the ADC Handle (e.g., &hadc1)
 */
void Sensors_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief Polling function to get latest data.
 * Inside, it calculates the latest physics based on current DMA buffer.
 * @return Pointer to the global SystemSensors_t struct.
 */
SystemSensors_t* Sensors_GetData(void);

#endif /* INC_ADC_SENSORS_H_ */
