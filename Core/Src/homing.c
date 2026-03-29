/**
 * @file homing.c
 * @brief Homing implementation
 */

#include "homing.h"

#include "adc_potentiometer.h"
#include "constants.h"
#include "encoder_reader.h"

#include <stdio.h>

static HomingStatus_t homing_status = HOMING_STATUS_NOT_STARTED;

int Homing_Init(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    printf("[Homing] Initialized\n");
    return 0;
}

int Homing_FindZero(void)
{
    ADC_PotSample_t adc_sample = {0};
    float steering_deg = 0.0f;
    float motor_deg = 0.0f;
    int32_t offset_count = 0;

    homing_status = HOMING_STATUS_IN_PROGRESS;

    /* MODIFIED(Codex): treat ADC as steering-axis absolute position, then convert to motor-side encoder offset. */
    if (ADC_Pot_GetSample(&adc_sample) == 0U) {
        homing_status = HOMING_STATUS_ERROR;
        return -1;
    }

    steering_deg = adc_sample.calibrated_angle_deg;
    motor_deg = SteeringDegToMotorDeg(steering_deg);
    printf("[Homing] ADC Steering: %.2f deg, Motor: %.2f deg\n", steering_deg, motor_deg);

    EncoderReader_Reset();

    offset_count = (int32_t)(motor_deg / ENCODER_DEG_PER_COUNT);
    EncoderReader_SetOffset(offset_count);

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

void Homing_Reset(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    printf("[Homing] Reset\n");
}
