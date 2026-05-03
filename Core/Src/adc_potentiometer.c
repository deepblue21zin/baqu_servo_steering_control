/**
 * @file adc_potentiometer.c
 * @brief ADC Potentiometer implementation
 */

#include "adc_potentiometer.h"

#include "adc.h"
#include "constants.h"
#include "project_params.h"

#include <stdint.h>
#include <stdio.h>

typedef struct {
    uint16_t prev_raw;
    uint16_t same_count;
    uint16_t jump_threshold_raw;
    uint16_t stuck_threshold_count;
    uint16_t disconnect_low_raw;
    uint16_t disconnect_high_raw;
    uint16_t valid_min_raw;
    uint16_t valid_max_raw;
    uint32_t last_sample_tick_ms;
    uint8_t initialized;
} ADC_PotDiag_t; /* MODIFIED(Codex): local ADC diagnostics state used for validity flags. */

static ADC_PotConfig_t pot_config = {
    .hadc = NULL,
    .channel = ADC_CHANNEL_4,
    .min_angle = POT_MIN_ANGLE_DEG,
    .max_angle = POT_MAX_ANGLE_DEG,
    .min_raw = 0U,
    .max_raw = (uint16_t)ADC_MAX_COUNT
};

static ADC_PotDiag_t g_pot_diag = {
    .prev_raw = 0U,
    .same_count = 0U,
    .jump_threshold_raw = 128U,
    .stuck_threshold_count = 50U,
    .disconnect_low_raw = 1U,
    .disconnect_high_raw = 4094U,
    .valid_min_raw = 0U,
    .valid_max_raw = (uint16_t)ADC_MAX_COUNT,
    .last_sample_tick_ms = 0U,
    .initialized = 0U
};

static uint8_t ADC_Pot_ReadRaw(uint16_t *out_raw, uint32_t timeout_ms)
{
    if ((out_raw == NULL) || (pot_config.hadc == NULL)) {
        return 0U;
    }

    if (HAL_ADC_Start(pot_config.hadc) != HAL_OK) {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(pot_config.hadc, timeout_ms) != HAL_OK) {
        return 0U;
    }

    *out_raw = (uint16_t)HAL_ADC_GetValue(pot_config.hadc);
    return 1U;
}

static float ADC_Pot_ConvertRawToAngle(uint16_t raw)
{
    if (pot_config.max_raw <= pot_config.min_raw) {
        return pot_config.min_angle;
    }

    return pot_config.min_angle +
           ((float)(raw - pot_config.min_raw) *
            (pot_config.max_angle - pot_config.min_angle) /
            (float)(pot_config.max_raw - pot_config.min_raw));
}

static uint32_t ADC_Pot_GetCalibrationChecksum(void)
{
    int32_t min_angle_x1000 = (int32_t)(pot_config.min_angle * 1000.0f);
    int32_t max_angle_x1000 = (int32_t)(pot_config.max_angle * 1000.0f);

    return 0x41504341UL ^
           ADC_POT_CALIBRATION_VERSION ^
           (uint32_t)pot_config.min_raw ^
           ((uint32_t)pot_config.max_raw << 16) ^
           (uint32_t)min_angle_x1000 ^
           (uint32_t)max_angle_x1000;
}

static uint32_t ADC_Pot_EvaluateValidity(uint16_t raw, uint8_t conversion_timeout, uint32_t sample_tick_ms)
{
    uint32_t flags = ADC_POT_VALID;
    uint16_t diff = (raw >= g_pot_diag.prev_raw) ? (raw - g_pot_diag.prev_raw) :
                    (g_pot_diag.prev_raw - raw);

    /* MODIFIED(Codex): surface disconnect/range/jump/stuck information to callers via validity bits. */
    if (g_pot_diag.initialized == 0U) {
        flags |= ADC_POT_INVALID_NOT_INIT;
    }
    if (conversion_timeout != 0U) {
        flags |= ADC_POT_INVALID_TIMEOUT;
        return flags;
    }
    if ((raw <= g_pot_diag.disconnect_low_raw) || (raw >= g_pot_diag.disconnect_high_raw)) {
        flags |= ADC_POT_INVALID_DISCONNECT;
    } else if ((raw < g_pot_diag.valid_min_raw) || (raw > g_pot_diag.valid_max_raw)) {
        flags |= ADC_POT_INVALID_RANGE;
    }
    if (diff > g_pot_diag.jump_threshold_raw) {
        flags |= ADC_POT_INVALID_JUMP;
    }
    if (raw == g_pot_diag.prev_raw) {
        g_pot_diag.same_count++;
        if (g_pot_diag.same_count >= g_pot_diag.stuck_threshold_count) {
            flags |= ADC_POT_INVALID_STUCK;
        }
    } else {
        g_pot_diag.same_count = 0U;
    }

    g_pot_diag.prev_raw = raw;
    g_pot_diag.last_sample_tick_ms = sample_tick_ms;
    return flags;
}

int ADC_Pot_Init(ADC_PotConfig_t *config)
{
    if (config != NULL) {
        pot_config = *config;
    }

    if (pot_config.hadc == NULL) {
        pot_config.hadc = &hadc1;
    }

    g_pot_diag.valid_min_raw = pot_config.min_raw;
    g_pot_diag.valid_max_raw = pot_config.max_raw;

    if (HAL_ADC_Start(pot_config.hadc) != HAL_OK) {
        return -1;
    }

    g_pot_diag.initialized = 1U;
    return 0;
}

void ADC_Pot_Service(void)
{
    /* ADC conversion is sampled on demand by ADC_Pot_GetSample(). */
}

uint16_t ADC_Pot_GetRaw(void)
{
    uint16_t raw = 0U;

    if (ADC_Pot_ReadRaw(&raw, ADC_POT_CONVERSION_TIMEOUT_MS) != 0U) {
        return raw;
    }

    return 0U;
}

float ADC_Pot_GetVoltage(void)
{
    uint16_t raw = ADC_Pot_GetRaw();
    return ((float)raw * ADC_VREF) / ADC_MAX_COUNT;
}

float ADC_Pot_GetAngle(void)
{
    return ADC_Pot_ConvertRawToAngle(ADC_Pot_GetRaw());
}

uint8_t ADC_Pot_GetSample(ADC_PotSample_t *out_sample)
{
    uint16_t raw = 0U;
    uint32_t now_ms = 0U;
    uint8_t conversion_timeout = 0U;

    if (out_sample == NULL) {
        return 0U;
    }

    if (ADC_Pot_ReadRaw(&raw, ADC_POT_CONVERSION_TIMEOUT_MS) == 0U) {
        raw = g_pot_diag.prev_raw;
        conversion_timeout = 1U;
    }
    now_ms = HAL_GetTick();

    out_sample->raw = raw;
    out_sample->voltage = ((float)raw * ADC_VREF) / ADC_MAX_COUNT;
    out_sample->calibrated_angle_deg = ADC_Pot_ConvertRawToAngle(raw);
    out_sample->sample_tick_ms = now_ms;
    out_sample->age_ms = (conversion_timeout == 0U) ? 0U : (now_ms - g_pot_diag.last_sample_tick_ms);
    out_sample->validity = ADC_Pot_EvaluateValidity(raw, conversion_timeout, now_ms);
    return 1U;
}

uint8_t ADC_Pot_GetCalibration(ADC_PotCalibration_t *out_calibration)
{
    if (out_calibration == NULL) {
        return 0U;
    }

    out_calibration->version = ADC_POT_CALIBRATION_VERSION;
    out_calibration->checksum = ADC_Pot_GetCalibrationChecksum();
    out_calibration->min_raw = pot_config.min_raw;
    out_calibration->max_raw = pot_config.max_raw;
    out_calibration->min_angle = pot_config.min_angle;
    out_calibration->max_angle = pot_config.max_angle;
    return 1U;
}

void ADC_Pot_Calibrate(float min_angle, float max_angle)
{
    pot_config.min_raw = ADC_Pot_GetRaw();
    pot_config.min_angle = min_angle;

    HAL_Delay(3000U);

    pot_config.max_raw = ADC_Pot_GetRaw();
    pot_config.max_angle = max_angle;
    g_pot_diag.valid_min_raw = pot_config.min_raw;
    g_pot_diag.valid_max_raw = pot_config.max_raw;
}
