#ifndef POSITION_CONTROL_DIAG_H
#define POSITION_CONTROL_DIAG_H

#include "position_control.h"

/* Return a readable label for the command source. */
const char* PositionControlDiag_CommandSourceString(CommandSource_t source);
/* Return a readable label for the command result. */
const char* PositionControlDiag_CommandResultString(CommandResult_t result);
/* Return a readable label for the command lifecycle state. */
const char* PositionControlDiag_CommandStateString(CommandState_t state_value);
/* Mirror the latest controller snapshot into shared debug variables. */
void PositionControlDiag_UpdateDebugVars(const PositionControl_State_t* state,
                                         bool control_enabled,
                                         ControlMode_t control_mode,
                                         uint8_t fault_flag);
/* Print a compact controller state summary for bench debugging. */
void PositionControlDiag_PrintStateSummary(const PositionControl_State_t* state,
                                           bool control_enabled,
                                           uint8_t fault_flag,
                                           const CommandLifecycle_t* command_lifecycle);
/* Return a readable label for a position-control error code. */
const char* PositionControlDiag_ErrorString(PosCtrl_Error_t error);

#endif /* POSITION_CONTROL_DIAG_H */
