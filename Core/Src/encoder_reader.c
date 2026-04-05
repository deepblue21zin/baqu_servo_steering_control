/**
 * @file encoder_reader.c
 * @brief Encoder reader implementation
 */

#include "encoder_reader.h"

#include "constants.h"
#include "project_params.h"
#include "tim.h"

#include <math.h>
#include <stdio.h>

#define ENCODER_TIMER htim2
#define ENCODER_COUNTER_CENTER 32768UL

typedef struct {
    uint16_t raw_count;
    uint16_t prev_raw_count;
    int32_t delta_count;
    int64_t accum_count;
    float motor_velocity_deg_per_s;
    float motor_accel_deg_per_s2;
    uint32_t sample_tick_ms;
} EncoderReader_Channel_t;

typedef struct {
    EncoderReader_Channel_t real_channel;
    EncoderReader_Channel_t virtual_channel;
    int32_t offset_count;
    uint8_t virtual_feedback_enabled;
    uint8_t initialized;
} EncoderReader_State_t; /* Keep real/virtual sample caches separate while exposing one active contract. */

static EncoderReader_State_t g_encoder = {0};

static const EncoderReader_Channel_t* EncoderReader_GetActiveChannelConst(void)
{
    return (g_encoder.virtual_feedback_enabled != 0U) ?
        &g_encoder.virtual_channel :
        &g_encoder.real_channel;
}

static uint16_t EncoderReader_GetVirtualRawCount(int64_t accum_count)
{
    int32_t raw32 = (int32_t)ENCODER_COUNTER_CENTER + (int32_t)accum_count;
    return (uint16_t)raw32;
}

static void EncoderReader_UpdateDerived(EncoderReader_Channel_t *channel,
                                        uint16_t raw_count,
                                        int32_t delta_count,
                                        int64_t accum_count,
                                        uint32_t now_ms)
{
    float dt_s = 0.001f;
    float prev_velocity_deg_per_s = 0.0f;
    float motor_delta_deg = 0.0f;
    float motor_velocity_deg_per_s = 0.0f;
    float motor_accel_deg_per_s2 = 0.0f;

    if (channel == NULL) {
        return;
    }

    if (channel->sample_tick_ms != 0U) {
        uint32_t dt_ms = now_ms - channel->sample_tick_ms;

        if (dt_ms == 0U) {
            dt_ms = 1U;
        } else if (dt_ms > 100U) {
            dt_ms = 100U;
        }
        dt_s = (float)dt_ms * 0.001f;
    }

    prev_velocity_deg_per_s = channel->motor_velocity_deg_per_s;
    motor_delta_deg = (float)delta_count * ENCODER_DEG_PER_COUNT;
    motor_velocity_deg_per_s = motor_delta_deg / dt_s;
    motor_accel_deg_per_s2 = (motor_velocity_deg_per_s - prev_velocity_deg_per_s) / dt_s;

    channel->raw_count = raw_count;
    channel->delta_count = delta_count;
    channel->accum_count = accum_count;
    channel->motor_velocity_deg_per_s = motor_velocity_deg_per_s;
    channel->motor_accel_deg_per_s2 = motor_accel_deg_per_s2;
    channel->sample_tick_ms = now_ms;
}

static uint32_t EncoderReader_BuildValidity(const EncoderReader_Channel_t *channel,
                                            uint32_t age_ms)
{
    uint32_t validity = ENCODER_VALID;
    float steering_velocity_deg_per_s = 0.0f;
    float steering_accel_deg_per_s2 = 0.0f;

    if ((g_encoder.initialized == 0U) || (channel == NULL)) {
        return ENCODER_INVALID_NOT_INIT;
    }

    if (age_ms >= ENCODER_SAMPLE_STALE_WARN_MS) {
        validity |= ENCODER_WARN_STALE;
    }
    if (age_ms >= ENCODER_SAMPLE_STALE_FAULT_MS) {
        validity |= ENCODER_FAULT_STALE;
    }

    steering_velocity_deg_per_s = MotorDegToSteeringDeg(channel->motor_velocity_deg_per_s);
    steering_accel_deg_per_s2 = MotorDegToSteeringDeg(channel->motor_accel_deg_per_s2);

    if (fabsf(steering_velocity_deg_per_s) >= ENCODER_VELOCITY_WARN_STEERING_DPS) {
        validity |= ENCODER_WARN_VELOCITY;
    }
    if (fabsf(steering_velocity_deg_per_s) >= ENCODER_VELOCITY_FAULT_STEERING_DPS) {
        validity |= ENCODER_FAULT_VELOCITY;
    }
    if (fabsf(steering_accel_deg_per_s2) >= ENCODER_ACCEL_WARN_STEERING_DPS2) {
        validity |= ENCODER_WARN_ACCEL;
    }
    if (fabsf(steering_accel_deg_per_s2) >= ENCODER_ACCEL_FAULT_STEERING_DPS2) {
        validity |= ENCODER_FAULT_ACCEL;
    }

    return validity;
}

static void EncoderReader_FillSample(const EncoderReader_Channel_t *channel,
                                     EncoderSample_t *out_sample,
                                     uint32_t now_ms)
{
    int64_t adjusted_count = 0;
    uint32_t age_ms = 0U;

    if ((channel == NULL) || (out_sample == NULL)) {
        return;
    }

    adjusted_count = channel->accum_count - (int64_t)g_encoder.offset_count;
    age_ms = now_ms - channel->sample_tick_ms;

    out_sample->raw_count = channel->raw_count;
    out_sample->delta_count = channel->delta_count;
    out_sample->accum_count = adjusted_count;
    out_sample->motor_deg = (float)adjusted_count * ENCODER_DEG_PER_COUNT;
    out_sample->steering_deg = MotorDegToSteeringDeg(out_sample->motor_deg);
    out_sample->motor_velocity_deg_per_s = channel->motor_velocity_deg_per_s;
    out_sample->steering_velocity_deg_per_s = MotorDegToSteeringDeg(channel->motor_velocity_deg_per_s);
    out_sample->motor_accel_deg_per_s2 = channel->motor_accel_deg_per_s2;
    out_sample->steering_accel_deg_per_s2 = MotorDegToSteeringDeg(channel->motor_accel_deg_per_s2);
    out_sample->sample_tick_ms = channel->sample_tick_ms;
    out_sample->age_ms = age_ms;
    out_sample->validity = EncoderReader_BuildValidity(channel, age_ms);
}

static void EncoderReader_SeedRealCounterState(void)
{
    uint16_t raw_count = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    uint32_t now_ms = HAL_GetTick();

    g_encoder.real_channel.raw_count = raw_count;
    g_encoder.real_channel.prev_raw_count = raw_count;
    g_encoder.real_channel.delta_count = 0;
    g_encoder.real_channel.accum_count = 0;
    g_encoder.real_channel.motor_velocity_deg_per_s = 0.0f;
    g_encoder.real_channel.motor_accel_deg_per_s2 = 0.0f;
    g_encoder.real_channel.sample_tick_ms = now_ms;
}

int EncoderReader_Init(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    EncoderReader_SeedRealCounterState();

    g_encoder.virtual_channel.raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.virtual_channel.prev_raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.virtual_channel.delta_count = 0;
    g_encoder.virtual_channel.accum_count = 0;
    g_encoder.virtual_channel.motor_velocity_deg_per_s = 0.0f;
    g_encoder.virtual_channel.motor_accel_deg_per_s2 = 0.0f;
    g_encoder.virtual_channel.sample_tick_ms = g_encoder.real_channel.sample_tick_ms;
    g_encoder.offset_count = 0;
    g_encoder.virtual_feedback_enabled = 0U;
    g_encoder.initialized = 1U;

    printf("[Encoder] Initialized: +count=%s, +motor_deg=%s\r\n",
           (ENCODER_COUNT_POLARITY >= 0) ? "+CW" : "+CCW",
           (SENSOR_POSITIVE_MOTOR_IS_CW != 0) ? "+CW" : "+CCW");
    return 0;
}

void EncoderReader_Service(void)
{
    uint16_t raw_count = 0U;
    int16_t signed_delta = 0;
    int32_t delta_count = 0;
    int64_t accum_count = 0;
    uint32_t now_ms = 0U;

    if ((g_encoder.initialized == 0U) || (g_encoder.virtual_feedback_enabled != 0U)) {
        return;
    }

    raw_count = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    signed_delta = (int16_t)(raw_count - g_encoder.real_channel.prev_raw_count);
    delta_count = (int32_t)signed_delta * (int32_t)ENCODER_COUNT_POLARITY;
    accum_count = g_encoder.real_channel.accum_count + (int64_t)delta_count;
    now_ms = HAL_GetTick();

    g_encoder.real_channel.prev_raw_count = raw_count;
    EncoderReader_UpdateDerived(&g_encoder.real_channel,
                                raw_count,
                                delta_count,
                                accum_count,
                                now_ms);
}

float EncoderReader_GetAngleDeg(void)
{
    return EncoderReader_GetMotorDeg();
}

float EncoderReader_GetMotorDeg(void)
{
    const EncoderReader_Channel_t *channel = EncoderReader_GetActiveChannelConst();
    int64_t adjusted_count = 0;

    if ((g_encoder.initialized == 0U) || (channel == NULL)) {
        return 0.0f;
    }

    adjusted_count = channel->accum_count - (int64_t)g_encoder.offset_count;
    return (float)adjusted_count * ENCODER_DEG_PER_COUNT;
}

int32_t EncoderReader_GetCount(void)
{
    const EncoderReader_Channel_t *channel = EncoderReader_GetActiveChannelConst();
    int64_t adjusted_count = 0;

    if ((g_encoder.initialized == 0U) || (channel == NULL)) {
        return 0;
    }

    adjusted_count = channel->accum_count - (int64_t)g_encoder.offset_count;
    return (int32_t)adjusted_count;
}

int32_t EncoderReader_GetDeltaCount(void)
{
    const EncoderReader_Channel_t *channel = EncoderReader_GetActiveChannelConst();

    if ((g_encoder.initialized == 0U) || (channel == NULL)) {
        return 0;
    }

    return channel->delta_count;
}

uint8_t EncoderReader_GetSample(EncoderSample_t *out_sample)
{
    const EncoderReader_Channel_t *channel = EncoderReader_GetActiveChannelConst();

    if ((out_sample == NULL) || (channel == NULL)) {
        return 0U;
    }

    EncoderReader_FillSample(channel, out_sample, HAL_GetTick());
    return (g_encoder.initialized != 0U) ? 1U : 0U;
}

uint32_t EncoderReader_GetRawCounter(void)
{
    const EncoderReader_Channel_t *channel = EncoderReader_GetActiveChannelConst();

    if (channel == NULL) {
        return (uint32_t)((uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER));
    }

    return (uint32_t)channel->raw_count;
}

void EncoderReader_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    EncoderReader_SeedRealCounterState();

    g_encoder.virtual_channel.raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.virtual_channel.prev_raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.virtual_channel.delta_count = 0;
    g_encoder.virtual_channel.accum_count = 0;
    g_encoder.virtual_channel.motor_velocity_deg_per_s = 0.0f;
    g_encoder.virtual_channel.motor_accel_deg_per_s2 = 0.0f;
    g_encoder.virtual_channel.sample_tick_ms = g_encoder.real_channel.sample_tick_ms;
    g_encoder.offset_count = 0;

    printf("[Encoder] Reset\r\n");
}

void EncoderReader_SetOffset(int32_t offset)
{
    g_encoder.offset_count = offset;
    printf("[Encoder] Offset set: %ld counts\r\n", (long)offset);
}

void EncoderReader_EnableVirtualFeedback(uint8_t enable)
{
    g_encoder.virtual_feedback_enabled = (enable != 0U) ? 1U : 0U;
    g_encoder.virtual_channel.delta_count = 0;
    g_encoder.virtual_channel.motor_velocity_deg_per_s = 0.0f;
    g_encoder.virtual_channel.motor_accel_deg_per_s2 = 0.0f;
    g_encoder.virtual_channel.sample_tick_ms = HAL_GetTick();

    if (g_encoder.virtual_feedback_enabled == 0U) {
        EncoderReader_SeedRealCounterState();
    }
}

void EncoderReader_SetVirtualFeedbackCount(int64_t accum_count)
{
    int64_t prev_accum_count = g_encoder.virtual_channel.accum_count;
    int32_t delta_count = (int32_t)(accum_count - prev_accum_count);

    EncoderReader_UpdateDerived(&g_encoder.virtual_channel,
                                EncoderReader_GetVirtualRawCount(accum_count),
                                delta_count,
                                accum_count,
                                HAL_GetTick());
}

uint8_t EncoderReader_IsInitialized(void)
{
    return g_encoder.initialized;
}
