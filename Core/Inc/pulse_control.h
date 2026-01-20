/**
 * @file pulse_control.h
 * @brief Pulse/PWM control for servo motor driver
 */

#ifndef PULSE_CONTROL_H
#define PULSE_CONTROL_H

#include "main.h"
#include <stdint.h>

/* ========== Functions ========== */

/**
 * @brief Initialize pulse control module
 */
void PulseControl_Init(void);

/**
 * @brief Set pulse frequency (Hz)
 * @param freq_hz Frequency in Hz (positive=CW, negative=CCW, 0=stop)
 */
void PulseControl_SetFrequency(int32_t freq_hz);

/**
 * @brief Stop pulse output
 */
void PulseControl_Stop(void);

/**
 * @brief Get current pulse frequency
 * @return Current frequency in Hz
 */
int32_t PulseControl_GetCurrentFreq(void);

/**
 * @brief Check if pulse module is initialized
 * @return 1 if initialized, 0 otherwise
 */
uint8_t PulseControl_IsInitialized(void);

/**
 * @brief Output pulses in forward direction (CW)
 * @param count Number of pulses to output
 */
void pulse_forward(uint32_t count);

/**
 * @brief Output pulses in reverse direction (CCW)
 * @param count Number of pulses to output
 */
void pulse_reverse(uint32_t count);

#endif /* PULSE_CONTROL_H */
