#include "position_control_diag.h"

#include "constants.h"
#include "encoder_reader.h"
#include "debug_vars.h"

#include <stdio.h>
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
static int16_t PositionControlDiag_OutputToDebugCmd(float output)
{
    if (output > (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (output < (float)INT16_MIN) {
        return INT16_MIN;
    }

    if (output >= 0.0f) {
        return (int16_t)(output + 0.5f);
    }
    return (int16_t)(output - 0.5f);
}

/* Encode the current controller condition into shared debug fault bits. */
static uint32_t PositionControlDiag_BuildDebugFaultFlags(bool control_enabled,
                                                         ControlMode_t control_mode,
                                                         uint8_t fault_flag)
{
    uint32_t flags = 0U;

    if (fault_flag == 1U) {
        flags |= DBG_FAULT_POS_LIMIT;
    }
    if (fault_flag == 2U) {
        flags |= DBG_FAULT_TRACKING;
    }
    if (fault_flag == 3U) {
        flags |= DBG_FAULT_TIMEOUT;
    }
    if (fault_flag == 4U) {
        flags |= DBG_FAULT_VELOCITY;
    }
    if (!control_enabled) {
        flags |= DBG_FAULT_DISABLED;
    }
    if (control_mode == CTRL_MODE_EMERGENCY) {
        flags |= DBG_FAULT_EMERGENCY;
    }

    return flags;
}

/* Return a short label for the command source that triggered the motion. */
const char* PositionControlDiag_CommandSourceString(CommandSource_t source)
{
    switch (source) {
    case CMD_SRC_UDP:
        return "UDP";
    case CMD_SRC_KEYBOARD:
        return "KEYBOARD";
    case CMD_SRC_SERVICE:
        return "SERVICE";
    case CMD_SRC_LOCALTEST:
        return "LOCALTEST";
    case CMD_SRC_NONE:
    default:
        return "NONE";
    }
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
void PositionControlDiag_UpdateDebugVars(const PositionControl_State_t* state,
                                         bool control_enabled,
                                         ControlMode_t control_mode,
                                         uint8_t fault_flag)
{
    if (state == NULL) {
        return;
    }

    dbg_enc_raw = (int32_t)EncoderReader_GetRawCounter();
    dbg_pos_mdeg = PositionControlDiag_DegToMilliDeg(MotorDegToSteeringDeg(state->current_angle));
    dbg_target_mdeg = PositionControlDiag_DegToMilliDeg(MotorDegToSteeringDeg(state->target_angle));
    dbg_err_mdeg = PositionControlDiag_DegToMilliDeg(MotorDegToSteeringDeg(state->error));
    dbg_pwm_cmd = PositionControlDiag_OutputToDebugCmd(state->output);
    dbg_fault_flags = PositionControlDiag_BuildDebugFaultFlags(control_enabled,
                                                               control_mode,
                                                               fault_flag);
}

/* Print a compact runtime snapshot for bench-side inspection. */
void PositionControlDiag_PrintStateSummary(const PositionControl_State_t* state,
                                           bool control_enabled,
                                           uint8_t fault_flag,
                                           const CommandLifecycle_t* command_lifecycle)
{
    if (state == NULL || command_lifecycle == NULL) {
        return;
    }

    printf("[PosCtrl] EN:%d FLT:%d CMD:%lu/%s/%s Target:%.2f Current:%.2f Error:%.2f Out:%.0f %s\r\n",
           (int)control_enabled,
           (int)fault_flag,
           (unsigned long)command_lifecycle->command_id,
           PositionControlDiag_CommandStateString(command_lifecycle->state),
           PositionControlDiag_CommandResultString(command_lifecycle->result),
           state->target_angle,
           state->current_angle,
           state->error,
           state->output,
           state->is_stable ? "STABLE" : "");
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
