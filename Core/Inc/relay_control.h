/**
 * @file relay_control.h
 * @brief Relay control for servo motor ON/OFF and emergency stop
 */

#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include "main.h"

/* ========== Pin Definitions (Define in main.h) ========== */
// #define SVON_PORT GPIOA
// #define SVON_PIN GPIO_PIN_0
// #define EMG_PORT GPIOA
// #define EMG_PIN GPIO_PIN_1

/* ========== Functions ========== */

/**
 * @brief Initialize relay control
 */
void Relay_Init(void);

/**
 * @brief Turn servo ON
 */
void Relay_ServoOn(void);

/**
 * @brief Turn servo OFF
 */
void Relay_ServoOff(void);

/**
 * @brief Emergency stop
 */
void Relay_Emergency(void);

/**
 * @brief Release emergency
 */
void Relay_EmergencyRelease(void);

#endif /* RELAY_CONTROL_H */
