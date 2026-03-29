#ifndef POSITION_CONTROL_SAFETY_H
#define POSITION_CONTROL_SAFETY_H

#include "position_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool is_safe;
    uint8_t fault_flag;
    PosCtrl_Error_t error;
    CommandResult_t result;
} PositionControlSafetyResult_t;

/* Initialize the safety module with default or caller-provided limits. */
void PositionControlSafety_Init(const SafetyLimits_t* initial_limits);

/* Update the active safety limits used by the evaluator. */
void PositionControlSafety_SetLimits(const SafetyLimits_t* limits);

/* Return the latest applied safety-limit snapshot. */
SafetyLimits_t PositionControlSafety_GetLimits(void);

/* Evaluate angle, tracking, and velocity limits without actuating hardware. */
PositionControlSafetyResult_t PositionControlSafety_Evaluate(float current_angle,
                                                             float tracking_error,
                                                             float measured_velocity_deg_per_s);

#endif /* POSITION_CONTROL_SAFETY_H */
