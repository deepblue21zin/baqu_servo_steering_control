/**
 * @file encoder_reader.c
 * @brief TIM2 encoder reader implementation.
 */

#include "encoder_reader.h"

#include "constants.h"
#include "project_params.h"
#include "tim.h"

#include <stdint.h>

#define ENCODER_TIMER htim2
#define ENCODER_COUNTER_CENTER 0x80000000UL

typedef struct {
    uint32_t raw_count;
    uint32_t prev_raw_count;
    int32_t delta_count;
    int64_t accum_count;
    int64_t offset_count;
    uint32_t sample_tick_ms;
    EncoderSample_t last_sample;
    uint8_t initialized;
} EncoderReader_State_t;

static EncoderReader_State_t g_encoder = {0};

static int32_t EncoderReader_ApplyCountPolarity(int32_t count)
{
#if ENCODER_COUNT_POLARITY < 0
    return -count;
#else
    return count;
#endif
}

static int64_t EncoderReader_MotorDegToCount(float motor_deg)
{
    float count_f = motor_deg / ENCODER_DEG_PER_COUNT;

    if (count_f >= 0.0f) {
        return (int64_t)(count_f + 0.5f);
    }
    return (int64_t)(count_f - 0.5f);
}

static void EncoderReader_FillLastSample(uint32_t raw_count,
                                         int32_t delta_count,
                                         int64_t accum_count,
                                         uint32_t sample_tick_ms,
                                         uint32_t interval_ms)
{
    EncoderSample_t sample = {0};

    sample.raw_count = raw_count;
    sample.delta_count = delta_count;
    sample.accum_count = accum_count - g_encoder.offset_count;
    sample.motor_deg = (float)sample.accum_count * ENCODER_DEG_PER_COUNT;
    sample.steering_deg = MotorDegToSteeringDeg(sample.motor_deg);
    sample.age_ms = 0U;
    sample.interval_ms = interval_ms;
    sample.sample_tick_ms = sample_tick_ms;
    sample.validity = EncoderDiag_EvaluateInit(g_encoder.initialized);
    sample.validity |= EncoderDiag_EvaluateMotion(delta_count, interval_ms);

    g_encoder.raw_count = raw_count;
    g_encoder.delta_count = delta_count;
    g_encoder.accum_count = accum_count;
    g_encoder.sample_tick_ms = sample_tick_ms;
    g_encoder.last_sample = sample;
}

static uint8_t EncoderReader_CopyLastSample(EncoderSample_t *out_sample)
{
    uint32_t primask = 0U;

    if ((out_sample == NULL) || (g_encoder.initialized == 0U)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out_sample = g_encoder.last_sample;
    if (primask == 0U) {
        __enable_irq();
    }

    out_sample->age_ms = HAL_GetTick() - out_sample->sample_tick_ms;
    out_sample->validity |= EncoderDiag_EvaluateStale(out_sample->age_ms);

    return 1U;
}

int EncoderReader_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    g_encoder.raw_count = (uint32_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    g_encoder.prev_raw_count = g_encoder.raw_count;
    g_encoder.delta_count = 0;
    g_encoder.accum_count = 0;
    g_encoder.offset_count = 0;
    g_encoder.sample_tick_ms = now_ms;
    g_encoder.initialized = 1U;
    EncoderDiag_Reset();

    EncoderReader_FillLastSample(g_encoder.raw_count, 0, g_encoder.accum_count, now_ms, 0U);

    return 0;
}

int EncoderReader_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t interval_ms = 0U;
    uint32_t raw = 0U;
    uint32_t raw_delta = 0U;
    int32_t signed_delta = 0;
    int32_t adjusted_delta = 0;
    int64_t accum_count = 0;

    if (g_encoder.initialized == 0U) {
        return 0;
    }

    interval_ms = now_ms - g_encoder.sample_tick_ms;
    raw = (uint32_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    raw_delta = raw - g_encoder.prev_raw_count;
    signed_delta = (int32_t)raw_delta;
    adjusted_delta = EncoderReader_ApplyCountPolarity(signed_delta);
    accum_count = g_encoder.accum_count + (int64_t)adjusted_delta;

    g_encoder.prev_raw_count = raw;
    EncoderReader_FillLastSample(raw, adjusted_delta, accum_count, now_ms, interval_ms);

    return 1;
}

void EncoderReader_Service(void)
{
    (void)EncoderReader_Update();
}

float EncoderReader_GetMotorDeg(void)
{
    EncoderSample_t sample = {0};

    if (EncoderReader_CopyLastSample(&sample) == 0U) {
        return 0.0f;
    }
    return sample.motor_deg;
}

float EncoderReader_GetSteeringDeg(void)
{
    EncoderSample_t sample = {0};

    if (EncoderReader_CopyLastSample(&sample) == 0U) {
        return 0.0f;
    }
    return sample.steering_deg;
}

int32_t EncoderReader_GetCount(void)
{
    EncoderSample_t sample = {0};

    if (EncoderReader_CopyLastSample(&sample) == 0U) {
        return 0;
    }
    return (int32_t)sample.accum_count;
}

int32_t EncoderReader_GetDeltaCount(void)
{
    EncoderSample_t sample = {0};

    if (EncoderReader_CopyLastSample(&sample) == 0U) {
        return 0;
    }
    return sample.delta_count;
}

uint8_t EncoderReader_GetLastSample(EncoderSample_t *out_sample)
{
    return EncoderReader_CopyLastSample(out_sample);
}

uint32_t EncoderReader_GetRawCounter(void)
{
    EncoderSample_t sample = {0};

    if (EncoderReader_CopyLastSample(&sample) == 0U) {
        return (uint32_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    }
    return sample.raw_count;
}

void EncoderReader_Reset(void)
{
    uint32_t now_ms = HAL_GetTick();

    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    g_encoder.raw_count = ENCODER_COUNTER_CENTER;
    g_encoder.prev_raw_count = ENCODER_COUNTER_CENTER;
    g_encoder.delta_count = 0;
    g_encoder.accum_count = 0;
    g_encoder.offset_count = 0;
    g_encoder.sample_tick_ms = now_ms;
    EncoderDiag_Reset();

    EncoderReader_FillLastSample(g_encoder.raw_count, 0, g_encoder.accum_count, now_ms, 0U);
}

void EncoderReader_SetOffset(int32_t offset)
{
    g_encoder.offset_count = (int64_t)offset;
    EncoderDiag_Reset();
    EncoderReader_FillLastSample(g_encoder.raw_count,
                                 0,
                                 g_encoder.accum_count,
                                 HAL_GetTick(),
                                 0U);
}

void EncoderReader_SetCurrentAsZero(void)
{
    g_encoder.offset_count = g_encoder.accum_count;
    EncoderDiag_Reset();
    EncoderReader_FillLastSample(g_encoder.raw_count,
                                 0,
                                 g_encoder.accum_count,
                                 HAL_GetTick(),
                                 0U);
}

void EncoderReader_SetCurrentAsMotorDeg(float motor_deg)
{
    int64_t desired_count = EncoderReader_MotorDegToCount(motor_deg);

    g_encoder.offset_count = g_encoder.accum_count - desired_count;
    EncoderDiag_Reset();
    EncoderReader_FillLastSample(g_encoder.raw_count,
                                 0,
                                 g_encoder.accum_count,
                                 HAL_GetTick(),
                                 0U);
}

uint8_t EncoderReader_IsInitialized(void)
{
    return g_encoder.initialized;
}
