#include "position_control_safety.h"

#include "constants.h"

#include <math.h>

#define POSITION_SAFETY_ANGLE_MARGIN_DEG 5.0f

static SafetyLimits_t position_control_safety_limits = {
    .max_error_allowed = MAX_TRACKING_ERROR_DEG,
    .max_velocity = 0.0f,
    .watchdog_timeout_ms = 0U
};

/* Build a passing result when all safety limits are satisfied. */
static PositionControlSafetyResult_t PositionControlSafety_Ok(void)
{
    PositionControlSafetyResult_t result = {
        .is_safe = true,
        .fault_flag = 0U,
        .error = POS_CTRL_OK,
        .result = CMD_RESULT_NONE
    };

    return result;
}

/* Build a failing result without triggering any actuator-side effect. */
static PositionControlSafetyResult_t PositionControlSafety_Trip(uint8_t fault_flag,
                                                                PosCtrl_Error_t error,
                                                                CommandResult_t result_code)
{
    PositionControlSafetyResult_t result = {
        .is_safe = false,
        .fault_flag = fault_flag,
        .error = error,
        .result = result_code
    };

    return result;
}

/* Normalize externally supplied limits before storing them. */
static SafetyLimits_t PositionControlSafety_NormalizeLimits(const SafetyLimits_t* limits)
{
    SafetyLimits_t normalized = position_control_safety_limits;

    if (limits == NULL) {
        return normalized;
    }

    if (limits->max_error_allowed > 0.0f) {
        normalized.max_error_allowed = fabsf(limits->max_error_allowed);
    }
    normalized.max_velocity = fabsf(limits->max_velocity);
    normalized.watchdog_timeout_ms = limits->watchdog_timeout_ms;

    return normalized;
}

/* Initialize the standalone safety evaluator state. */
void PositionControlSafety_Init(const SafetyLimits_t* initial_limits)
{
    position_control_safety_limits = (SafetyLimits_t){
        .max_error_allowed = MAX_TRACKING_ERROR_DEG,
        .max_velocity = 0.0f,
        .watchdog_timeout_ms = 0U
    };

    if (initial_limits != NULL) {
        position_control_safety_limits = PositionControlSafety_NormalizeLimits(initial_limits);
    }
}

/* Replace the active safety-limit snapshot used by the evaluator. */
void PositionControlSafety_SetLimits(const SafetyLimits_t* limits)
{
    if (limits == NULL) {
        return;
    }

    position_control_safety_limits = PositionControlSafety_NormalizeLimits(limits);
}

/* Return a copy of the latest safety limits for logging and lifecycle sync. */
SafetyLimits_t PositionControlSafety_GetLimits(void)
{
    return position_control_safety_limits;
}

/* Evaluate all local safety criteria without directly stopping the actuator. */
PositionControlSafetyResult_t PositionControlSafety_Evaluate(float current_angle,
                                                             float tracking_error,
                                                             float measured_velocity_deg_per_s)
{
    if (current_angle > MAX_ANGLE_DEG + POSITION_SAFETY_ANGLE_MARGIN_DEG ||
        current_angle < MIN_ANGLE_DEG - POSITION_SAFETY_ANGLE_MARGIN_DEG) {
        return PositionControlSafety_Trip(1U,
                                          POS_CTRL_ERR_OVER_LIMIT,
                                          CMD_RESULT_FAULT_LIMIT);
    }

    if ((position_control_safety_limits.max_error_allowed > 0.0f) &&
        (fabsf(tracking_error) > position_control_safety_limits.max_error_allowed)) {
        return PositionControlSafety_Trip(2U,
                                          POS_CTRL_ERR_SAFETY,
                                          CMD_RESULT_FAULT_TRACKING);
    }

    if ((position_control_safety_limits.max_velocity > 0.0f) &&
        (fabsf(measured_velocity_deg_per_s) > position_control_safety_limits.max_velocity)) {
        return PositionControlSafety_Trip(4U,
                                          POS_CTRL_ERR_VELOCITY,
                                          CMD_RESULT_FAULT_VELOCITY);
    }

    return PositionControlSafety_Ok();
}
