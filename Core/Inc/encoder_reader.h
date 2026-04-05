/**
 * @file encoder_reader.h
 * @brief Encoder reader module for position feedback
 */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <stdint.h>

typedef enum {
    ENCODER_VALID = 0x00,
    ENCODER_WARN_STALE = 0x01,
    ENCODER_FAULT_STALE = 0x02,
    ENCODER_WARN_VELOCITY = 0x04,
    ENCODER_FAULT_VELOCITY = 0x08,
    ENCODER_WARN_ACCEL = 0x10,
    ENCODER_FAULT_ACCEL = 0x20,
    ENCODER_INVALID_NOT_INIT = 0x40
} EncoderValidity_t;

typedef struct {
    uint16_t raw_count;
    int32_t delta_count;
    int64_t accum_count;
    float motor_deg;
    float steering_deg;
    float motor_velocity_deg_per_s;
    float steering_velocity_deg_per_s;
    float motor_accel_deg_per_s2;
    float steering_accel_deg_per_s2;
    uint32_t age_ms;
    uint32_t sample_tick_ms;
    uint32_t validity;
} EncoderSample_t; /* MODIFIED(Codex): richer encoder snapshot for diagnostics. */

int EncoderReader_Init(void);
void EncoderReader_Service(void);
float EncoderReader_GetAngleDeg(void);
float EncoderReader_GetMotorDeg(void);
int32_t EncoderReader_GetCount(void);
int32_t EncoderReader_GetDeltaCount(void);
uint8_t EncoderReader_GetSample(EncoderSample_t *out_sample);
uint32_t EncoderReader_GetRawCounter(void);
void EncoderReader_Reset(void);
void EncoderReader_SetOffset(int32_t offset);
void EncoderReader_EnableVirtualFeedback(uint8_t enable);
void EncoderReader_SetVirtualFeedbackCount(int64_t accum_count);
uint8_t EncoderReader_IsInitialized(void);

#endif /* ENCODER_READER_H */
