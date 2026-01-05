/**
 * @file adc_potentiometer.c
 * @brief ADC Potentiometer implementation
 * @author Person C
 */

#include "main.h"
#include "adc_potentiometer.h"

/* ========== Private Variables ========== */

static ADC_PotConfig_t pot_config = {
    .hadc = NULL,
    .channel = ADC_CHANNEL_4,  // PA4
    .min_angle = -90.0f,
    .max_angle = 90.0f,
    .min_raw = 0,
    .max_raw = 4095
};

/* ========== Public Functions ========== */

/**
 * @brief Initialize ADC potentiometer
 */
int ADC_Pot_Init(ADC_PotConfig_t *config)
{
    if (config != NULL) {
        pot_config = *config;
    }
    
    // Use default hadc1 if not provided
    if (pot_config.hadc == NULL) {
        pot_config.hadc = &hadc1;
    }
    
    // Start ADC
    if (HAL_ADC_Start(pot_config.hadc) != HAL_OK) {
        return -1;
    }
    
    return 0;
}

/**
 * @brief Get raw ADC value
 */
uint16_t ADC_Pot_GetRaw(void)
{
    // Start conversion
    HAL_ADC_Start(pot_config.hadc);
    
    // Wait for conversion
    if (HAL_ADC_PollForConversion(pot_config.hadc, 100) == HAL_OK) {
        return (uint16_t)HAL_ADC_GetValue(pot_config.hadc);
    }
    
    return 0;
}

/**
 * @brief Get voltage
 */
float ADC_Pot_GetVoltage(void)
{
    uint16_t raw = ADC_Pot_GetRaw();
    
    // Convert to voltage (0-3.3V for 12-bit ADC)
    float voltage = (float)raw * 3.3f / 4095.0f;
    
    return voltage;
}

/**
 * @brief Get angle in degrees
 */
float ADC_Pot_GetAngle(void)
{
    uint16_t raw = ADC_Pot_GetRaw();
    
    // Map raw value to angle
    float angle = pot_config.min_angle + 
                  (float)(raw - pot_config.min_raw) * 
                  (pot_config.max_angle - pot_config.min_angle) / 
                  (float)(pot_config.max_raw - pot_config.min_raw);
    
    return angle;
}

/**
 * @brief Calibrate potentiometer
 */
void ADC_Pot_Calibrate(float min_angle, float max_angle)
{
    // Read current position as minimum
    pot_config.min_raw = ADC_Pot_GetRaw();
    pot_config.min_angle = min_angle;
    
    // Wait for user to move to maximum
    HAL_Delay(3000);  // 3 seconds
    
    // Read maximum position
    pot_config.max_raw = ADC_Pot_GetRaw();
    pot_config.max_angle = max_angle;
}