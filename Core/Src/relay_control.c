/**
 * @file relay_control.c
 * @brief Basic relay control implementation
 */

#include "main.h"
#include "relay_control.h"

/* ========== Public Functions ========== */

/**
 * @brief Initialize relay control
 * SVON: Active LOW (LOW=ON, HIGH=OFF)
 * EMG: Active LOW, B접점 (LOW=비상정지, HIGH=정상)
 */
void Relay_Init(void)
{
    // Default state: Servo OFF, EMG released (정상 상태)
    HAL_GPIO_WritePin(SVON_PORT, SVON_PIN_CTRL, GPIO_PIN_SET);   // SVON OFF (Active LOW)
    HAL_GPIO_WritePin(EMG_PORT, EMG_PIN_CTRL, GPIO_PIN_SET);     // EMG 해제 (정상)
}

/**
 * @brief Turn servo ON (Active LOW: LOW=ON)
 */
void Relay_ServoOn(void)
{
    HAL_GPIO_WritePin(SVON_PORT, SVON_PIN_CTRL, GPIO_PIN_RESET); // LOW = Servo ON
}

/**
 * @brief Turn servo OFF (Active LOW: HIGH=OFF)
 */
void Relay_ServoOff(void)
{
    HAL_GPIO_WritePin(SVON_PORT, SVON_PIN_CTRL, GPIO_PIN_SET);   // HIGH = Servo OFF
}

/**
 * @brief Emergency stop (Active LOW: LOW=정지)
 */
void Relay_Emergency(void)
{
    HAL_GPIO_WritePin(EMG_PORT, EMG_PIN_CTRL, GPIO_PIN_RESET);   // LOW = 비상정지
}

/**
 * @brief Release emergency (Active LOW: HIGH=정상)
 */
void Relay_EmergencyRelease(void)
{
    HAL_GPIO_WritePin(EMG_PORT, EMG_PIN_CTRL, GPIO_PIN_SET);     // HIGH = 정상
}