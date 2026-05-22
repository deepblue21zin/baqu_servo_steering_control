/**
 * @file encoder_reader.h
 * @brief Encoder reader module for position feedback
 */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "encoder_diag.h"

#include <stdint.h>

typedef struct {
    uint32_t raw_count;
    int32_t delta_count;
    int64_t accum_count;
    float motor_deg;
    float steering_deg;
    uint32_t age_ms;
    uint32_t interval_ms;
    uint32_t sample_tick_ms;
    uint32_t validity;
} EncoderSample_t; /* MODIFIED(Codex): richer encoder snapshot for diagnostics. */

int EncoderReader_Init(void);
int EncoderReader_Update(void);
void EncoderReader_Service(void);
float EncoderReader_GetMotorDeg(void);
float EncoderReader_GetSteeringDeg(void);
int32_t EncoderReader_GetCount(void);
int32_t EncoderReader_GetDeltaCount(void);
uint8_t EncoderReader_GetLastSample(EncoderSample_t *out_sample);
uint32_t EncoderReader_GetRawCounter(void);
void EncoderReader_Reset(void);
void EncoderReader_SetOffset(int32_t offset);
void EncoderReader_SetCurrentAsZero(void);
void EncoderReader_SetCurrentAsMotorDeg(float motor_deg);
uint8_t EncoderReader_IsInitialized(void);

#endif /* ENCODER_READER_H */
