#include "position_control_safety.h"

#include "constants.h"

#include <math.h>

static SafetyLimits_t position_control_safety_limits = {
    .max_error_motor_deg = MAX_TRACKING_ERROR_MOTOR_DEG,
    .max_velocity_motor_deg_per_s = 0.0f,
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
#if POSITION_CONTROL_SAFETY_ENABLE
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
#endif

/* Normalize externally supplied limits before storing them. */
static SafetyLimits_t PositionControlSafety_NormalizeLimits(const SafetyLimits_t* limits)
{
    SafetyLimits_t normalized = position_control_safety_limits;

    if (limits == NULL) {
        return normalized;
    }

    if (limits->max_error_motor_deg > 0.0f) {
        normalized.max_error_motor_deg = fabsf(limits->max_error_motor_deg);
    }
    normalized.max_velocity_motor_deg_per_s = fabsf(limits->max_velocity_motor_deg_per_s);
    normalized.watchdog_timeout_ms = limits->watchdog_timeout_ms;

    return normalized;
}

/* Initialize the standalone safety evaluator state. */
void PositionControlSafety_Init(const SafetyLimits_t* initial_limits)
{
    position_control_safety_limits = (SafetyLimits_t){
        .max_error_motor_deg = MAX_TRACKING_ERROR_MOTOR_DEG,
        .max_velocity_motor_deg_per_s = 0.0f,
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
PositionControlSafetyResult_t PositionControlSafety_Evaluate(float current_motor_deg,
                                                             float tracking_error_motor_deg,
                                                             float measured_velocity_motor_deg_per_s)
{
#if !POSITION_CONTROL_SAFETY_ENABLE
    (void)current_motor_deg;
    (void)tracking_error_motor_deg;
    (void)measured_velocity_motor_deg_per_s;
    return PositionControlSafety_Ok();
#else
    if (current_motor_deg > MAX_MOTOR_ANGLE_DEG + POSITION_SAFETY_ANGLE_MARGIN_DEG ||
        current_motor_deg < MIN_MOTOR_ANGLE_DEG - POSITION_SAFETY_ANGLE_MARGIN_DEG) {
        return PositionControlSafety_Trip(1U,
                                          POS_CTRL_ERR_OVER_LIMIT,
                                          CMD_RESULT_FAULT_LIMIT);
    }

    if ((position_control_safety_limits.max_error_motor_deg > 0.0f) &&
        (fabsf(tracking_error_motor_deg) > position_control_safety_limits.max_error_motor_deg)) {
        return PositionControlSafety_Trip(2U,
                                          POS_CTRL_ERR_SAFETY,
                                          CMD_RESULT_FAULT_TRACKING);
    }

    if ((position_control_safety_limits.max_velocity_motor_deg_per_s > 0.0f) &&
        (fabsf(measured_velocity_motor_deg_per_s) > position_control_safety_limits.max_velocity_motor_deg_per_s)) {
        return PositionControlSafety_Trip(4U,
                                          POS_CTRL_ERR_VELOCITY,
                                          CMD_RESULT_FAULT_VELOCITY);
    }

    return PositionControlSafety_Ok();
#endif
}

PositionControlSafetyResult_t PositionControlSafety_EvaluateCommandTimeout(uint32_t start_ms,
                                                                           uint32_t timeout_ms,
                                                                           uint32_t now_ms)
{
#if !POSITION_CONTROL_SAFETY_ENABLE
    (void)start_ms;
    (void)timeout_ms;
    (void)now_ms;
    return PositionControlSafety_Ok();
#else
    if ((timeout_ms > 0U) && ((now_ms - start_ms) > timeout_ms)) {
        return PositionControlSafety_Trip(3U,
                                          POS_CTRL_ERR_TIMEOUT,
                                          CMD_RESULT_TIMEOUT);
    }

    return PositionControlSafety_Ok();
#endif
}
