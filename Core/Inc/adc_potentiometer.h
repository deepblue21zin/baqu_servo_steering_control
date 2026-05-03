/**
 * @file adc_potentiometer.h
 * @brief ADC Potentiometer module for absolute position
 */

#ifndef ADC_POTENTIOMETER_H
#define ADC_POTENTIOMETER_H

#include <stdint.h>

#include "main.h"

typedef enum {
    ADC_POT_VALID = 0x00,
    ADC_POT_INVALID_NOT_INIT = 0x01,
    ADC_POT_INVALID_DISCONNECT = 0x02,
    ADC_POT_INVALID_TIMEOUT = 0x04,
    ADC_POT_INVALID_STUCK = 0x08,
    ADC_POT_INVALID_JUMP = 0x10,
    ADC_POT_INVALID_RANGE = 0x20
} ADC_PotValidity_t;

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t channel;
    float min_angle;
    float max_angle;
    uint16_t min_raw;
    uint16_t max_raw;
} ADC_PotConfig_t;

typedef struct {
    uint16_t raw;
    float voltage;
    float calibrated_angle_deg;
    uint32_t sample_tick_ms;
    uint32_t age_ms;
    uint32_t validity;
} ADC_PotSample_t; /* MODIFIED(Codex): calibrated sample + validity bits. */

typedef struct {
    uint32_t version;
    uint32_t checksum;
    uint16_t min_raw;
    uint16_t max_raw;
    float min_angle;
    float max_angle;
} ADC_PotCalibration_t;

int ADC_Pot_Init(ADC_PotConfig_t *config);
void ADC_Pot_Service(void);
uint16_t ADC_Pot_GetRaw(void);
float ADC_Pot_GetVoltage(void);
float ADC_Pot_GetAngle(void);
uint8_t ADC_Pot_GetSample(ADC_PotSample_t *out_sample);
uint8_t ADC_Pot_GetCalibration(ADC_PotCalibration_t *out_calibration);
void ADC_Pot_Calibrate(float min_angle, float max_angle);

#endif /* ADC_POTENTIOMETER_H */
