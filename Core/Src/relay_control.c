/**
 * @file relay_control.c
 * @brief Basic relay control implementation
 */

#include "main.h"
#include "relay_control.h"

/* ========== Public Functions ========== */

/**
 * @brief Initialize relay control
 */
void Relay_Init(void)
{
    // Default state: All OFF
    HAL_GPIO_WritePin(SVON_PORT, SVON_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMG_PORT, EMG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Turn servo ON
 */
void Relay_ServoOn(void)
{
    HAL_GPIO_WritePin(SVON_PORT, SVON_PIN, GPIO_PIN_SET);
}

/**
 * @brief Turn servo OFF
 */
void Relay_ServoOff(void)
{
    HAL_GPIO_WritePin(SVON_PORT, SVON_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Emergency stop
 */
void Relay_Emergency(void)
{
    HAL_GPIO_WritePin(EMG_PORT, EMG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Release emergency
 */
void Relay_EmergencyRelease(void)
{
    HAL_GPIO_WritePin(EMG_PORT, EMG_PIN, GPIO_PIN_SET);
}