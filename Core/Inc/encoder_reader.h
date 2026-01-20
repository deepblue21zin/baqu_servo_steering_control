/**
 * @file encoder_reader.h
 * @brief Encoder reader module for position feedback
 */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "main.h"
#include <stdint.h>

/* ========== Functions ========== */

/**
 * @brief Initialize encoder reader
 * @return 0 on success, -1 on error
 */
int EncoderReader_Init(void);

/**
 * @brief Get current angle in degrees
 * @return Angle in degrees
 */
float EncoderReader_GetAngleDeg(void);

/**
 * @brief Get raw encoder count
 * @return Encoder count
 */
int32_t EncoderReader_GetCount(void);

/**
 * @brief Reset encoder count to zero
 */
void EncoderReader_Reset(void);

/**
 * @brief Set encoder offset (for homing)
 * @param offset Offset value
 */
void EncoderReader_SetOffset(int32_t offset);

/**
 * @brief Check if encoder is initialized
 * @return 1 if initialized, 0 otherwise
 */
uint8_t EncoderReader_IsInitialized(void);

#endif /* ENCODER_READER_H */
