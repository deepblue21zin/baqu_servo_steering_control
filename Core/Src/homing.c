/**
 * @file homing.c
 * @brief Homing implementation
 */

#include "homing.h"

#include "adc_potentiometer.h"
#include "constants.h"
#include "encoder_reader.h"
#include "project_params.h"

#include <math.h>
#include <stdio.h>

static HomingStatus_t homing_status = HOMING_STATUS_NOT_STARTED;
static float g_homing_last_crosscheck_error_deg = 0.0f;
static const char *g_homing_last_failure_reason = "none";

static int32_t Homing_RoundToCount(float count_value)
{
    if (count_value >= 0.0f) {
        return (int32_t)(count_value + 0.5f);
    }

    return (int32_t)(count_value - 0.5f);
}

static uint8_t Homing_WaitForAdcSample(ADC_PotSample_t *out_sample, uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    if (out_sample == NULL) {
        return 0U;
    }

    while ((HAL_GetTick() - start_ms) <= timeout_ms) {
        ADC_Pot_Service();
        if (ADC_Pot_GetSample(out_sample) != 0U) {
            return 1U;
        }
    }

    return 0U;
}

int Homing_Init(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    g_homing_last_crosscheck_error_deg = 0.0f;
    g_homing_last_failure_reason = "none";
    printf("[Homing] Initialized\r\n");
    return 0;
}

int Homing_FindZero(void)
{
    ADC_PotSample_t adc_sample = {0};
    EncoderSample_t encoder_sample = {0};
    float steering_deg = 0.0f;
    float motor_deg = 0.0f;
    int32_t offset_count = 0;

    homing_status = HOMING_STATUS_IN_PROGRESS;
    g_homing_last_crosscheck_error_deg = 0.0f;
    g_homing_last_failure_reason = "none";

    if (ADC_Pot_IsInitialized() == 0U) {
        g_homing_last_failure_reason = "adc_not_initialized";
        homing_status = HOMING_STATUS_ERROR;
        return -1;
    }

    if (Homing_WaitForAdcSample(&adc_sample, ADC_POT_SAMPLE_STALE_FAULT_MS) == 0U) {
        g_homing_last_failure_reason = "adc_sample_timeout";
        homing_status = HOMING_STATUS_ERROR;
        return -1;
    }

    if (adc_sample.validity != ADC_POT_VALID) {
        g_homing_last_failure_reason = "adc_sample_invalid";
        homing_status = HOMING_STATUS_ERROR;
        printf("[Homing] Reject ADC sample: validity=0x%02lX age=%lu raw=%u\r\n",
               (unsigned long)adc_sample.validity,
               (unsigned long)adc_sample.age_ms,
               (unsigned int)adc_sample.raw);
        return -1;
    }

    steering_deg = adc_sample.calibrated_angle_deg;
    motor_deg = SteeringDegToMotorDeg(steering_deg);

    EncoderReader_Reset();
    EncoderReader_Service();

    /* After reset the logical angle is count-offset, so the offset must be the negative of the measured position. */
    offset_count = -Homing_RoundToCount(motor_deg / ENCODER_DEG_PER_COUNT);
    EncoderReader_SetOffset(offset_count);
    EncoderReader_Service();

    if (EncoderReader_GetSample(&encoder_sample) == 0U) {
        g_homing_last_failure_reason = "encoder_sample_unavailable";
        homing_status = HOMING_STATUS_ERROR;
        return -1;
    }

    g_homing_last_crosscheck_error_deg = encoder_sample.steering_deg - steering_deg;
    if (fabsf(g_homing_last_crosscheck_error_deg) > SENSOR_HOMING_CROSSCHECK_FAULT_DEG) {
        g_homing_last_failure_reason = "homing_crosscheck_mismatch";
        homing_status = HOMING_STATUS_ERROR;
        printf("[Homing] Cross-check failed: adc=%.3f enc=%.3f err=%.3f deg\r\n",
               steering_deg,
               encoder_sample.steering_deg,
               g_homing_last_crosscheck_error_deg);
        return -1;
    }

    homing_status = HOMING_STATUS_COMPLETE;
    printf("[Homing] Complete: adc=%.3f deg motor=%.3f deg offset=%ld enc=%.3f deg err=%.3f deg\r\n",
           steering_deg,
           motor_deg,
           (long)offset_count,
           encoder_sample.steering_deg,
           g_homing_last_crosscheck_error_deg);

    return 0;
}

uint8_t Homing_IsComplete(void)
{
    return (homing_status == HOMING_STATUS_COMPLETE) ? 1U : 0U;
}

HomingStatus_t Homing_GetStatus(void)
{
    return homing_status;
}

const char* Homing_GetStatusString(void)
{
    switch (homing_status) {
    case HOMING_STATUS_NOT_STARTED:
        return "Not Started";
    case HOMING_STATUS_IN_PROGRESS:
        return "In Progress";
    case HOMING_STATUS_COMPLETE:
        return "Complete";
    case HOMING_STATUS_ERROR:
        return "Error";
    default:
        return "Unknown";
    }
}

float Homing_GetLastCrosscheckErrorDeg(void)
{
    return g_homing_last_crosscheck_error_deg;
}

const char* Homing_GetLastFailureReason(void)
{
    return g_homing_last_failure_reason;
}

void Homing_Reset(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    g_homing_last_crosscheck_error_deg = 0.0f;
    g_homing_last_failure_reason = "none";
    printf("[Homing] Reset\r\n");
}
