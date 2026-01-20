/**
 * @file homing.c
 * @brief Homing (원점 찾기) implementation
 */

#include "homing.h"
#include "constants.h"
#include "adc_potentiometer.h"
#include "encoder_reader.h"
#include <stdio.h>

/* ========== Private Variables ========== */

static HomingStatus_t homing_status = HOMING_STATUS_NOT_STARTED;

/* ========== Public Functions ========== */

/**
 * @brief Initialize homing module
 */
int Homing_Init(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    printf("[Homing] Initialized\n");
    return 0;
}

/**
 * @brief Perform homing sequence
 */
int Homing_FindZero(void)
{
    homing_status = HOMING_STATUS_IN_PROGRESS;

    // 1. Read absolute position from ADC potentiometer
    float pot_angle = ADC_Pot_GetAngle();
    printf("[Homing] ADC Angle: %.2f deg\n", pot_angle);

    // 2. Reset encoder count
    EncoderReader_Reset();

    // 3. Set encoder offset to match potentiometer angle
    // Convert angle to encoder count
    int32_t offset_count = (int32_t)(pot_angle * ENCODER_PPR / FULL_ROTATION_DEG);
    EncoderReader_SetOffset(offset_count);

    homing_status = HOMING_STATUS_COMPLETE;
    printf("[Homing] Complete at %.2f deg\n", pot_angle);

    return 0;
}

/**
 * @brief Check if homing is complete
 */
uint8_t Homing_IsComplete(void)
{
    return (homing_status == HOMING_STATUS_COMPLETE) ? 1 : 0;
}

/**
 * @brief Get homing status
 */
HomingStatus_t Homing_GetStatus(void)
{
    return homing_status;
}

/**
 * @brief Get status string
 */
const char* Homing_GetStatusString(void)
{
    switch (homing_status) {
        case HOMING_STATUS_NOT_STARTED:
            return "Not Started";
        case HOMING_STATUS_IN_PROGRESS:
            return "In Progress";
        case HOMING_STATUS_COMPLETE:
            return "Complete";
        case HOMING_STATUS_ERROR:
            return "Error";
        default:
            return "Unknown";
    }
}

/**
 * @brief Reset homing state
 */
void Homing_Reset(void)
{
    homing_status = HOMING_STATUS_NOT_STARTED;
    printf("[Homing] Reset\n");
}