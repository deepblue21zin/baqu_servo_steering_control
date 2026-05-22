#include "position_control_diag.h"

#include "encoder_reader.h"
#include "debug_vars.h"

#include <stddef.h>
#include <stdint.h>

/* Convert a floating-point degree value into milli-degree debug units. */
static int32_t PositionControlDiag_DegToMilliDeg(float deg)
{
    float scaled = deg * 1000.0f;

    if (scaled > (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < (float)INT32_MIN) {
        return INT32_MIN;
    }

    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }
    return (int32_t)(scaled - 0.5f);
}

/* Clamp the pulse command into the exported debug field width. */
static int32_t PositionControlDiag_OutputToDebugCmd(float output)
{
    if (output > (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (output < (float)INT32_MIN) {
        return INT32_MIN;
    }

    if (output >= 0.0f) {
        return (int32_t)(output + 0.5f);
    }
    return (int32_t)(output - 0.5f);
}

/* Return a short label for the command completion reason. */
const char* PositionControlDiag_CommandResultString(CommandResult_t result)
{
    switch (result) {
    case CMD_RESULT_REACHED:
        return "REACHED";
    case CMD_RESULT_TIMEOUT:
        return "TIMEOUT";
    case CMD_RESULT_ESTOP:
        return "ESTOP";
    case CMD_RESULT_DISABLED:
        return "DISABLED";
    case CMD_RESULT_REPLACED:
        return "REPLACED";
    case CMD_RESULT_FAULT_LIMIT:
        return "FAULT_LIMIT";
    case CMD_RESULT_FAULT_TRACKING:
        return "FAULT_TRACKING";
    case CMD_RESULT_FAULT_VELOCITY:
        return "FAULT_VELOCITY";
    case CMD_RESULT_NONE:
    default:
        return "NONE";
    }
}

/* Return a short label for the command lifecycle state. */
const char* PositionControlDiag_CommandStateString(CommandState_t state_value)
{
    switch (state_value) {
    case CMD_ACTIVE:
        return "ACTIVE";
    case CMD_REACHED:
        return "REACHED";
    case CMD_TIMEOUT:
        return "TIMEOUT";
    case CMD_ABORTED:
        return "ABORTED";
    case CMD_FAULTED:
        return "FAULTED";
    case CMD_IDLE:
    default:
        return "IDLE";
    }
}

/* Push the latest controller snapshot into the globally shared debug variables. */
void PositionControlDiag_UpdateDebugVars(const PositionControl_State_t* state)
{
    if (state == NULL) {
        return;
    }

    dbg_enc_raw = EncoderReader_GetRawCounter();
    dbg_pos_mdeg = PositionControlDiag_DegToMilliDeg(state->current_steering_deg);
    dbg_target_mdeg = PositionControlDiag_DegToMilliDeg(state->target_steering_deg);
    dbg_err_mdeg = PositionControlDiag_DegToMilliDeg(state->error_steering_deg);
    dbg_pwm_cmd = PositionControlDiag_OutputToDebugCmd(state->output_hz);
}

/* Map a position-control error code to a readable string. */
const char* PositionControlDiag_ErrorString(PosCtrl_Error_t error)
{
    switch (error) {
    case POS_CTRL_OK:
        return "No Error";
    case POS_CTRL_ERR_NOT_INIT:
        return "Not Initialized";
    case POS_CTRL_ERR_DISABLED:
        return "Control Disabled";
    case POS_CTRL_ERR_OVER_LIMIT:
        return "Target Out of Range";
    case POS_CTRL_ERR_ENCODER:
        return "Encoder Error";
    case POS_CTRL_ERR_TIMEOUT:
        return "Timeout Error";
    case POS_CTRL_ERR_SAFETY:
        return "Safety Violation";
    case POS_CTRL_ERR_VELOCITY:
        return "Velocity Limit Exceeded";
    default:
        return "Unknown Error";
    }
}

/* Preserve the legacy public API while routing string lookup through diag helpers. */
const char* PositionControl_GetErrorString(PosCtrl_Error_t error)
{
    return PositionControlDiag_ErrorString(error);
}
