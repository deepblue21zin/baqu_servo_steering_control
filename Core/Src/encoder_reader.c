/**
 * @file encoder_reader.c
 * @brief Encoder reader implementation
 */

#include "encoder_reader.h"

#include "constants.h"
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
    uint8_t initialized;
} EncoderReader_State_t; /* MODIFIED(Codex): keep raw/delta/accum/timestamp in one state block. */

static EncoderReader_State_t g_encoder = {0};

static int64_t EncoderReader_UpdateCount(void)
{
    uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    int16_t signed_delta = (int16_t)(raw - g_encoder.prev_raw_count);

    /* MODIFIED(Codex): unwrap the 16-bit hardware counter by interpreting the delta as int16_t. */
    g_encoder.raw_count = raw;
    g_encoder.delta_count = (int32_t)signed_delta;
    g_encoder.accum_count += (int32_t)signed_delta;
    g_encoder.prev_raw_count = raw;
    g_encoder.sample_tick_ms = HAL_GetTick();

    return g_encoder.accum_count;
}

int EncoderReader_Init(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    g_encoder.raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.prev_raw_count = (uint16_t)ENCODER_COUNTER_CENTER;
    g_encoder.delta_count = 0;
    g_encoder.accum_count = 0;
    g_encoder.offset_count = 0;
    g_encoder.sample_tick_ms = HAL_GetTick();
    g_encoder.initialized = 1U;

    printf("[Encoder] Initialized\n");
    return 0;
}

float EncoderReader_GetAngleDeg(void)
{
    return EncoderReader_GetMotorDeg();
}

float EncoderReader_GetMotorDeg(void)
{
    int64_t adjusted_count = EncoderReader_UpdateCount() - (int64_t)g_encoder.offset_count;
    return (float)adjusted_count * ENCODER_DEG_PER_COUNT;
}

int32_t EncoderReader_GetCount(void)
{
    return (int32_t)(EncoderReader_UpdateCount() - (int64_t)g_encoder.offset_count);
}

int32_t EncoderReader_GetDeltaCount(void)
{
    EncoderReader_UpdateCount();
    return g_encoder.delta_count;
}

uint8_t EncoderReader_GetSample(EncoderSample_t *out_sample)
{
    uint32_t now_ms = 0U;

    if ((out_sample == NULL) || (g_encoder.initialized == 0U)) {
        return 0U;
    }

    EncoderReader_UpdateCount();
    now_ms = HAL_GetTick();

    out_sample->raw_count = g_encoder.raw_count;
    out_sample->delta_count = g_encoder.delta_count;
    out_sample->accum_count = g_encoder.accum_count - (int64_t)g_encoder.offset_count;
    out_sample->motor_deg = (float)out_sample->accum_count * ENCODER_DEG_PER_COUNT;
    out_sample->sample_tick_ms = g_encoder.sample_tick_ms;
    out_sample->age_ms = now_ms - g_encoder.sample_tick_ms;
    return 1U;
}

uint32_t EncoderReader_GetRawCounter(void)
{
    if (g_encoder.initialized == 0U) {
        return (uint32_t)((uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER));
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

    printf("[Encoder] Reset\n");
}

void EncoderReader_SetOffset(int32_t offset)
{
    g_encoder.offset_count = offset;
    printf("[Encoder] Offset set: %ld\n", offset);
}

uint8_t EncoderReader_IsInitialized(void)
{
    return g_encoder.initialized;
}
