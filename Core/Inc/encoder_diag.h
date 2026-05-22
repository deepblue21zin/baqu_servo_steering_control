/**
 * @file encoder_diag.h
 * @brief Encoder diagnostic validity checks.
 */

#ifndef ENCODER_DIAG_H
#define ENCODER_DIAG_H

#include <stdint.h>

typedef enum {
    ENCODER_VALID = 0x00,
    ENCODER_INVALID_NOT_INIT = 0x01,
    ENCODER_WARN_STALE = 0x02,
    ENCODER_FAULT_STALE = 0x04,
    ENCODER_WARN_VELOCITY = 0x08,
    ENCODER_FAULT_VELOCITY = 0x10,
    ENCODER_WARN_ACCEL = 0x20,
    ENCODER_FAULT_ACCEL = 0x40
} EncoderValidity_t;

void EncoderDiag_Reset(void);
uint32_t EncoderDiag_EvaluateInit(uint8_t initialized);
uint32_t EncoderDiag_EvaluateStale(uint32_t age_ms);
uint32_t EncoderDiag_EvaluateMotion(int32_t delta_count, uint32_t interval_ms);

#endif /* ENCODER_DIAG_H */
