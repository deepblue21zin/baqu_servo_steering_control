/**
 * @file homing.c
 * @brief Homing implementation
 */

#include "homing.h"

#include "adc_potentiometer.h"
#include "constants.h"
#include "encoder_reader.h"
#include "project_params.h"

#include <stdio.h>

static HomingStatus_t homing_status = HOMING_STATUS_NOT_STARTED;
static const char *g_homing_last_failure_reason = "none";
static float g_homing_last_crosscheck_error_deg = 0.0f;

static int32_t Homing_RoundToCount(float count_f)
{
    if (count_f >= 0.0f) {
        return (int32_t)(count_f + 0.5f);
    }

    return (int32_t)(count_f - 0.5f);
}

static float Homing_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

int Homing_Init(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    g_homing_last_failure_reason = "none";
    g_homing_last_crosscheck_error_deg = 0.0f;
    printf("[Homing] Initialized\n");
    return 0;
}

int Homing_FindZero(void)
{
    ADC_PotSample_t adc_sample = {0};
    EncoderSample_t encoder_sample = {0};
    float steering_deg = 0.0f;
    float motor_deg = 0.0f;
    int32_t desired_count = 0;

    homing_status = HOMING_STATUS_IN_PROGRESS;
    g_homing_last_failure_reason = "none";
    g_homing_last_crosscheck_error_deg = 0.0f;

    /* MODIFIED(Codex): treat ADC as steering-axis absolute position, then convert to motor-side encoder offset. */
    if (ADC_Pot_GetSample(&adc_sample) == 0U) {
        homing_status = HOMING_STATUS_ERROR;
        g_homing_last_failure_reason = "adc_sample_unavailable";
        return -1;
    }

    if ((adc_sample.validity & (ADC_POT_INVALID_NOT_INIT |
                                ADC_POT_INVALID_DISCONNECT |
                                ADC_POT_INVALID_TIMEOUT |
                                ADC_POT_INVALID_RANGE)) != 0U) {
        homing_status = HOMING_STATUS_ERROR;
        g_homing_last_failure_reason = "adc_invalid";
        return -1;
    }

    steering_deg = adc_sample.calibrated_angle_deg;
    motor_deg = SteeringDegToMotorDeg(steering_deg);
    printf("[Homing] ADC Steering: %.2f deg, Motor: %.2f deg\n", steering_deg, motor_deg);

    EncoderReader_Reset();

    desired_count = Homing_RoundToCount(motor_deg / ENCODER_DEG_PER_COUNT);
    EncoderReader_SetOffset(-desired_count);

    if (EncoderReader_GetSample(&encoder_sample) == 0U) {
        homing_status = HOMING_STATUS_ERROR;
        g_homing_last_failure_reason = "encoder_sample_unavailable";
        return -1;
    }

    g_homing_last_crosscheck_error_deg = encoder_sample.steering_deg - steering_deg;
    if (Homing_AbsFloat(g_homing_last_crosscheck_error_deg) > SENSOR_HOMING_CROSSCHECK_FAULT_DEG) {
        homing_status = HOMING_STATUS_ERROR;
        g_homing_last_failure_reason = "homing_crosscheck";
        printf("[Homing] Crosscheck failed: enc=%.2f adc=%.2f err=%.3f deg\r\n",
               encoder_sample.steering_deg,
               steering_deg,
               g_homing_last_crosscheck_error_deg);
        return -1;
    }

    homing_status = HOMING_STATUS_COMPLETE;
    printf("[Homing] Complete at steering %.2f deg\n", steering_deg);

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

const char* Homing_GetLastFailureReason(void)
{
    return g_homing_last_failure_reason;
}

float Homing_GetLastCrosscheckErrorDeg(void)
{
    return g_homing_last_crosscheck_error_deg;
}

void Homing_Reset(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    g_homing_last_failure_reason = "none";
    g_homing_last_crosscheck_error_deg = 0.0f;
    printf("[Homing] Reset\n");
}
