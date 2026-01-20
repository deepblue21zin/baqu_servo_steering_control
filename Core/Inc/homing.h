/**
 * @file homing.h
 * @brief Homing (원점 찾기) module
 * @author Person B
 * @date 2025-01-06
 */

#ifndef HOMING_H
#define HOMING_H

#include "main.h"
#include <stdint.h>

/* ========== Status ========== */

typedef enum {
    HOMING_STATUS_NOT_STARTED = 0,
    HOMING_STATUS_IN_PROGRESS,
    HOMING_STATUS_COMPLETE,
    HOMING_STATUS_ERROR
} HomingStatus_t;

/* ========== Functions ========== */

/**
 * @brief Initialize homing module
 * @return 0 on success, -1 on error
 */
int Homing_Init(void);

/**
 * @brief Perform homing sequence
 * @return 0 on success, -1 on error
 * 
 * Steps:
 * 1. Read absolute position from ADC potentiometer
 * 2. Reset encoder count
 * 3. Set encoder offset to match potentiometer angle
 */
int Homing_FindZero(void);

/**
 * @brief Check if homing is complete
 * @return 1 if complete, 0 otherwise
 */
uint8_t Homing_IsComplete(void);

/**
 * @brief Get homing status
 * @return Current status
 */
HomingStatus_t Homing_GetStatus(void);

/**
 * @brief Get status string
 * @return Status description
 */
const char* Homing_GetStatusString(void);

/**
 * @brief Reset homing state
 */
void Homing_Reset(void);

#endif /* HOMING_H */