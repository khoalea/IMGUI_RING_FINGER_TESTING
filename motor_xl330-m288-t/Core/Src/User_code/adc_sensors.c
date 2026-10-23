/*
 * adc_sensors.c
 *
 * Description: Reads 2 Potentiometers + VREF via ADC DMA (Circular Mode)
 * and calculates Position & Velocity with supply compensation.
 */

#include "User_code/adc_sensors.h"
#include "User_code/calibration.h"
#include <math.h>

extern CalibrationData myConfig;

// ---- Internal Variables ----
static ADC_HandleTypeDef *sensor_hadc;

volatile uint16_t adc_dma_buffer[ADC_CHANNELS];

static SystemSensors_t sensors = {0};


// ---- Physics Calculation ----
static void CalculatePhysics(Potentiometer_t *sensor,
                             JointParams *param,
                             uint16_t new_raw,
                             uint16_t vref_raw)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - sensor->_last_timestamp_ms) / 1000.0f;

    // ---- VREF Compensation ----
    float corrected_raw = (float)new_raw;

    if (vref_raw > 0)
    {
        corrected_raw = corrected_raw * (4095.0f / (float)vref_raw);
    }

    sensor->raw_adc = (uint16_t)corrected_raw;

    // ---- Position ----
    float new_pos = (corrected_raw - param->zero) * param->scale;

    if (new_pos < param->min_angle) new_pos = param->min_angle;
    if (new_pos > param->max_angle) new_pos = param->max_angle;

    sensor->position = new_pos;

    // ---- Velocity ----
    if (dt > 0.0f)
    {
        float instantaneous_vel = (new_pos - sensor->_prev_position) / dt;

        sensor->velocity = (SENSOR_LPF_ALPHA * instantaneous_vel) +
                           ((1.0f - SENSOR_LPF_ALPHA) * sensor->velocity);

        sensor->_prev_position = new_pos;
        sensor->_last_timestamp_ms = now;
    }
}


// ---- Initialization ----
void Sensors_Init(ADC_HandleTypeDef *hadc)
{
    sensor_hadc = hadc;

    uint32_t now = HAL_GetTick();
    sensors.pot1._last_timestamp_ms = now;
    sensors.pot2._last_timestamp_ms = now;

    if (HAL_ADC_Start_DMA(sensor_hadc, (uint32_t*)adc_dma_buffer, ADC_CHANNELS) != HAL_OK)
    {
        // optional: error handling
    }
}


// ---- Update from DMA ----
static void Sensors_Update(void)
{
    sensors.vref_raw = adc_dma_buffer[2];

    // Then update sensors using it
    CalculatePhysics(&sensors.pot1, &myConfig.joints[0], adc_dma_buffer[0], sensors.vref_raw);
    CalculatePhysics(&sensors.pot2, &myConfig.joints[1], adc_dma_buffer[1], sensors.vref_raw);
}


// ---- Public Access ----
SystemSensors_t* Sensors_GetData(void)
{
    Sensors_Update();
    return &sensors;
}
