/**
 * @file encoder_diag.c
 * @brief Encoder diagnostic validity checks.
 */

#include "encoder_diag.h"

#include "constants.h"
#include "project_params.h"

static float last_velocity_steering_dps = 0.0f;

void EncoderDiag_Reset(void)
{
    last_velocity_steering_dps = 0.0f;
}

uint32_t EncoderDiag_EvaluateInit(uint8_t initialized)
{
    return (initialized == 0U) ? ENCODER_INVALID_NOT_INIT : ENCODER_VALID;
}

uint32_t EncoderDiag_EvaluateStale(uint32_t age_ms)
{
    uint32_t flags = ENCODER_VALID;

    if (age_ms >= ENCODER_SAMPLE_STALE_WARN_MS) {
        flags |= ENCODER_WARN_STALE;
    }
    if (age_ms >= ENCODER_SAMPLE_STALE_FAULT_MS) {
        flags |= ENCODER_FAULT_STALE;
    }

    return flags;
}

uint32_t EncoderDiag_EvaluateMotion(int32_t delta_count, uint32_t interval_ms)
{
    uint32_t flags = ENCODER_VALID;
    float dt_s = 0.001f;
    float steering_delta_deg = 0.0f;
    float velocity_steering_dps = 0.0f;
    float accel_steering_dps2 = 0.0f;

    if (interval_ms > 0U) {
        dt_s = ((float)interval_ms) * 0.001f;
    }

    steering_delta_deg = MotorDegToSteeringDeg((float)delta_count * ENCODER_DEG_PER_COUNT);
    velocity_steering_dps = steering_delta_deg / dt_s;
    if (velocity_steering_dps < 0.0f) {
        velocity_steering_dps = -velocity_steering_dps;
    }

    if (velocity_steering_dps >= ENCODER_VELOCITY_WARN_STEERING_DPS) {
        flags |= ENCODER_WARN_VELOCITY;
    }
    if (velocity_steering_dps >= ENCODER_VELOCITY_FAULT_STEERING_DPS) {
        flags |= ENCODER_FAULT_VELOCITY;
    }

    accel_steering_dps2 = (velocity_steering_dps - last_velocity_steering_dps) / dt_s;
    if (accel_steering_dps2 < 0.0f) {
        accel_steering_dps2 = -accel_steering_dps2;
    }
    last_velocity_steering_dps = velocity_steering_dps;

    if (accel_steering_dps2 >= ENCODER_ACCEL_WARN_STEERING_DPS2) {
        flags |= ENCODER_WARN_ACCEL;
    }
    if (accel_steering_dps2 >= ENCODER_ACCEL_FAULT_STEERING_DPS2) {
        flags |= ENCODER_FAULT_ACCEL;
    }

    return flags;
}
