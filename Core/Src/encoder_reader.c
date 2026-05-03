/**
 * @file encoder_reader.c
 * @brief Encoder reader implementation
 */

#include "encoder_reader.h"

#include "constants.h"
#include "project_params.h"
#include "tim.h"

#include <stdio.h>

#define ENCODER_TIMER htim2
#define ENCODER_COUNTER_CENTER 32768UL

typedef struct {
    uint16_t raw_count;
    uint16_t prev_raw_count;
    int32_t delta_count;
    int64_t accum_count;
    int32_t offset_count;
    uint32_t sample_tick_ms;
    uint8_t virtual_feedback_enabled;
    uint16_t virtual_raw_count;
    int32_t virtual_delta_count;
    int64_t virtual_accum_count;
    uint32_t virtual_sample_tick_ms;
    float last_diag_velocity_steering_dps;
    uint8_t initialized;
} EncoderReader_State_t; /* MODIFIED(Codex): keep raw/delta/accum/timestamp in one state block. */

static EncoderReader_State_t g_encoder = {0};

static int64_t EncoderReader_UpdateCount(void);

static uint32_t EncoderReader_EvaluateValidity(const EncoderSample_t *sample)
{
    uint32_t flags = ENCODER_VALID;
    float dt_s = 0.001f;
    float steering_delta_deg = 0.0f;
    float velocity_steering_dps = 0.0f;
    float accel_steering_dps2 = 0.0f;

    if (g_encoder.initialized == 0U) {
        flags |= ENCODER_INVALID_NOT_INIT;
    }

    if (sample->age_ms >= ENCODER_SAMPLE_STALE_WARN_MS) {
        flags |= ENCODER_WARN_STALE;
    }
    if (sample->age_ms >= ENCODER_SAMPLE_STALE_FAULT_MS) {
        flags |= ENCODER_FAULT_STALE;
    }

    if (sample->age_ms > 0U) {
        dt_s = ((float)sample->age_ms) * 0.001f;
    }

    steering_delta_deg = MotorDegToSteeringDeg((float)sample->delta_count * ENCODER_DEG_PER_COUNT);
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

    accel_steering_dps2 = (velocity_steering_dps - g_encoder.last_diag_velocity_steering_dps) / dt_s;
    if (accel_steering_dps2 < 0.0f) {
        accel_steering_dps2 = -accel_steering_dps2;
    }
    g_encoder.last_diag_velocity_steering_dps = velocity_steering_dps;

    if (accel_steering_dps2 >= ENCODER_ACCEL_WARN_STEERING_DPS2) {
        flags |= ENCODER_WARN_ACCEL;
    }
    if (accel_steering_dps2 >= ENCODER_ACCEL_FAULT_STEERING_DPS2) {
        flags |= ENCODER_FAULT_ACCEL;
    }

    return flags;
}

static void EncoderReader_FillSample(EncoderSample_t *out_sample,
                                     uint16_t raw_count,
                                     int32_t delta_count,
                                     int64_t accum_count,
                                     uint32_t sample_tick_ms,
                                     uint32_t now_ms)
{
    out_sample->raw_count = raw_count;
    out_sample->delta_count = delta_count;
    out_sample->accum_count = accum_count - (int64_t)g_encoder.offset_count;
    out_sample->motor_deg = (float)out_sample->accum_count * ENCODER_DEG_PER_COUNT;
    out_sample->steering_deg = MotorDegToSteeringDeg(out_sample->motor_deg);
    out_sample->sample_tick_ms = sample_tick_ms;
    out_sample->age_ms = now_ms - sample_tick_ms;
    out_sample->validity = EncoderReader_EvaluateValidity(out_sample);
}

static uint16_t EncoderReader_GetVirtualRawCount(void)
{
    int32_t raw32 = (int32_t)ENCODER_COUNTER_CENTER + (int32_t)g_encoder.virtual_accum_count;
    return (uint16_t)raw32;
}

static int64_t EncoderReader_GetActiveCount(void)
{
    if (g_encoder.virtual_feedback_enabled != 0U) {
        return g_encoder.virtual_accum_count;
    }

    return EncoderReader_UpdateCount();
}

static int64_t EncoderReader_UpdateCount(void)
{
    uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    int16_t signed_delta = (int16_t)(raw - g_encoder.prev_raw_count);
    int32_t adjusted_delta = (ENCODER_COUNT_POLARITY < 0) ?
                             -(int32_t)signed_delta :
                             (int32_t)signed_delta;

    /* Unwrap the 16-bit hardware counter, then apply the configured encoder polarity. */
    g_encoder.raw_count = raw;
    g_encoder.delta_count = adjusted_delta;
    g_encoder.accum_count += adjusted_delta;
    g_encoder.prev_raw_count = raw;
    g_encoder.sample_tick_ms = HAL_GetTick();

    return g_encoder.accum_count;
}

int EncoderReader_Init(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    g_encoder.raw_count = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    g_encoder.prev_raw_count = g_encoder.raw_count;
    g_encoder.delta_count = 0;
    g_encoder.accum_count = 0;
    g_encoder.offset_count = 0;
    g_encoder.sample_tick_ms = HAL_GetTick();
    g_encoder.virtual_feedback_enabled = 0U;
    g_encoder.virtual_raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.virtual_delta_count = 0;
    g_encoder.virtual_accum_count = 0;
    g_encoder.virtual_sample_tick_ms = g_encoder.sample_tick_ms;
    g_encoder.last_diag_velocity_steering_dps = 0.0f;
    g_encoder.initialized = 1U;

    printf("[Encoder] Initialized\n");
    return 0;
}

void EncoderReader_Service(void)
{
    /* Sampling remains demand-driven through EncoderReader_GetSample(). */
}

float EncoderReader_GetAngleDeg(void)
{
    return EncoderReader_GetMotorDeg();
}

float EncoderReader_GetMotorDeg(void)
{
    int64_t adjusted_count = EncoderReader_GetActiveCount() - (int64_t)g_encoder.offset_count;
    return (float)adjusted_count * ENCODER_DEG_PER_COUNT;
}

int32_t EncoderReader_GetCount(void)
{
    return (int32_t)(EncoderReader_GetActiveCount() - (int64_t)g_encoder.offset_count);
}

int32_t EncoderReader_GetDeltaCount(void)
{
    if (g_encoder.virtual_feedback_enabled != 0U) {
        return g_encoder.virtual_delta_count;
    }

    EncoderReader_UpdateCount();
    return g_encoder.delta_count;
}

uint8_t EncoderReader_GetSample(EncoderSample_t *out_sample)
{
    uint32_t now_ms = 0U;

    if ((out_sample == NULL) || (g_encoder.initialized == 0U)) {
        return 0U;
    }

    if (g_encoder.virtual_feedback_enabled != 0U) {
        now_ms = HAL_GetTick();
        EncoderReader_FillSample(out_sample,
                                 g_encoder.virtual_raw_count,
                                 g_encoder.virtual_delta_count,
                                 g_encoder.virtual_accum_count,
                                 g_encoder.virtual_sample_tick_ms,
                                 now_ms);
        return 1U;
    }

    EncoderReader_UpdateCount();
    now_ms = HAL_GetTick();

    EncoderReader_FillSample(out_sample,
                             g_encoder.raw_count,
                             g_encoder.delta_count,
                             g_encoder.accum_count,
                             g_encoder.sample_tick_ms,
                             now_ms);
    return 1U;
}

uint32_t EncoderReader_GetRawCounter(void)
{
    if (g_encoder.initialized == 0U) {
        return (uint32_t)((uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER));
    }

    if (g_encoder.virtual_feedback_enabled != 0U) {
        return (uint32_t)g_encoder.virtual_raw_count;
    }

    EncoderReader_UpdateCount();
    return (uint32_t)g_encoder.raw_count;
}

void EncoderReader_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    g_encoder.raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.prev_raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.delta_count = 0;
    g_encoder.accum_count = 0;
    g_encoder.offset_count = 0;
    g_encoder.sample_tick_ms = HAL_GetTick();
    g_encoder.virtual_raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.virtual_delta_count = 0;
    g_encoder.virtual_accum_count = 0;
    g_encoder.virtual_sample_tick_ms = g_encoder.sample_tick_ms;
    g_encoder.last_diag_velocity_steering_dps = 0.0f;

    printf("[Encoder] Reset\n");
}

void EncoderReader_SetOffset(int32_t offset)
{
    g_encoder.offset_count = offset;
    printf("[Encoder] Offset set: %ld\n", offset);
}

void EncoderReader_EnableVirtualFeedback(uint8_t enable)
{
    g_encoder.virtual_feedback_enabled = (enable != 0U) ? 1U : 0U;
    g_encoder.virtual_delta_count = 0;
    g_encoder.virtual_sample_tick_ms = HAL_GetTick();
}

void EncoderReader_SetVirtualFeedbackCount(int64_t accum_count)
{
    int64_t prev_count = g_encoder.virtual_accum_count;

    g_encoder.virtual_accum_count = accum_count;
    g_encoder.virtual_delta_count = (int32_t)(accum_count - prev_count);
    g_encoder.virtual_raw_count = EncoderReader_GetVirtualRawCount();
    g_encoder.virtual_sample_tick_ms = HAL_GetTick();
}

uint8_t EncoderReader_IsInitialized(void)
{
    return g_encoder.initialized;
}
