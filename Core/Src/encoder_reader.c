/**
 * @file encoder_reader.c
 * @brief Encoder reader implementation
 */

#include "encoder_reader.h"
#include "tim.h"
#include <stdio.h>

#define ENCODER_TIMER htim2
#define ENCODER_COUNTER_CENTER 32768UL

/* Encoder configuration (XML-FBL04AMK1) */
#define PULSE_PER_REV 12000
#define QUADRATURE    4
#define COUNT_PER_REV (PULSE_PER_REV * QUADRATURE)
#define DEG_PER_COUNT (360.0f / COUNT_PER_REV)

static int32_t encoder_count = 0;
static int32_t encoder_offset = 0;
static int32_t encoder_delta = 0;
static uint32_t encoder_last_raw = ENCODER_COUNTER_CENTER;
static uint8_t initialized = 0;

static int32_t EncoderReader_UpdateCount(void)
{
    uint32_t raw = __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    int32_t delta = (int32_t)(raw - encoder_last_raw);

    encoder_delta = delta;
    encoder_count += encoder_delta;
    encoder_last_raw = raw;

    return encoder_count;
}

int EncoderReader_Init(void)
{
    encoder_count = 0;
    encoder_offset = 0;
    encoder_delta = 0;
    encoder_last_raw = ENCODER_COUNTER_CENTER;
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);
    initialized = 1;
    printf("[Encoder] Initialized\n");
    return 0;
}

float EncoderReader_GetAngleDeg(void)
{
    int32_t adjusted_count = EncoderReader_UpdateCount() - encoder_offset;
    return (float)adjusted_count * DEG_PER_COUNT;
}

int32_t EncoderReader_GetCount(void)
{
    return EncoderReader_UpdateCount() - encoder_offset;
}

uint32_t EncoderReader_GetRawCounter(void)
{
    return __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
}

void EncoderReader_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);
    encoder_count = 0;
    encoder_offset = 0;
    encoder_delta = 0;
    encoder_last_raw = ENCODER_COUNTER_CENTER;
    printf("[Encoder] Reset\n");
}

void EncoderReader_SetOffset(int32_t offset)
{
    encoder_offset = offset;
    printf("[Encoder] Offset set: %ld\n", offset);
}

uint8_t EncoderReader_IsInitialized(void)
{
    return initialized;
}
