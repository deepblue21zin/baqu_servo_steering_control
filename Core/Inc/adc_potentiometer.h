/**
 * @file adc_potentiometer.h
 * @brief ADC Potentiometer module for absolute position
 * @author Person C
 * @date 2025-01-06
 */

#ifndef ADC_POTENTIOMETER_H
#define ADC_POTENTIOMETER_H

#include "main.h"
#include <stdint.h>

/* ========== Configuration ========== */

// ADC handle (extern from main.c)
extern ADC_HandleTypeDef hadc1;

/* ========== Structures ========== */

typedef struct {
    ADC_HandleTypeDef *hadc;  // ADC handle
    uint32_t channel;         // ADC channel
    float min_angle;          // Minimum angle (deg)
    float max_angle;          // Maximum angle (deg)
    uint16_t min_raw;         // Raw value at min angle
    uint16_t max_raw;         // Raw value at max angle
} ADC_PotConfig_t;

/* ========== Functions ========== */

/**
 * @brief Initialize ADC potentiometer
 * @param config Configuration struct
 * @return 0 on success, -1 on error
 */
int ADC_Pot_Init(ADC_PotConfig_t *config);

/**
 * @brief Get raw ADC value (0-4095)
 * @return Raw ADC value
 */
uint16_t ADC_Pot_GetRaw(void);

/**
 * @brief Get voltage (0-3.3V)
 * @return Voltage in volts
 */
float ADC_Pot_GetVoltage(void);

/**
 * @brief Get angle in degrees
 * @return Angle in degrees
 */
float ADC_Pot_GetAngle(void);

/**
 * @brief Calibrate potentiometer
 * @param min_angle Minimum angle (deg)
 * @param max_angle Maximum angle (deg)
 * 
 * Calibration steps:
 * 1. Move motor to minimum position
 * 2. Call: ADC_Pot_Calibrate(min_angle, max_angle)
 * 3. Move motor to maximum position
 * 4. Calibration complete
 */
void ADC_Pot_Calibrate(float min_angle, float max_angle);

#endif /* ADC_POTENTIOMETER_H */