#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "project_params.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    POS_CTRL_OK = 0,
    POS_CTRL_ERR_NOT_INIT = -1,
    POS_CTRL_ERR_DISABLED = -2,
    POS_CTRL_ERR_OVER_LIMIT = -3,
    POS_CTRL_ERR_ENCODER = -4,
    POS_CTRL_ERR_TIMEOUT = -5,
    POS_CTRL_ERR_SAFETY = -6,
    POS_CTRL_ERR_VELOCITY = -7
} PosCtrl_Error_t;

typedef enum {
    DEBUG_NONE = 0,
    DEBUG_ERROR = 1,
    DEBUG_WARNING = 2,
    DEBUG_INFO = 3,
    DEBUG_VERBOSE = 4
} DebugLevel_t;

typedef enum {
    CTRL_MODE_IDLE = 0,
    CTRL_MODE_POSITION = 1,
    CTRL_MODE_MANUAL = 2,
    CTRL_MODE_EMERGENCY = 99
} ControlMode_t;

typedef enum {
    CMD_IDLE = 0,
    CMD_ACTIVE,
    CMD_REACHED,
    CMD_TIMEOUT,
    CMD_ABORTED,
    CMD_FAULTED
} CommandState_t;

typedef enum {
    CMD_SRC_NONE = 0,
    CMD_SRC_UDP,
    CMD_SRC_KEYBOARD,
    CMD_SRC_SERVICE,
    CMD_SRC_LOCALTEST
} CommandSource_t;

typedef enum {
    CMD_RESULT_NONE = 0,
    CMD_RESULT_REACHED,
    CMD_RESULT_TIMEOUT,
    CMD_RESULT_ESTOP,
    CMD_RESULT_DISABLED,
    CMD_RESULT_REPLACED,
    CMD_RESULT_FAULT_LIMIT,
    CMD_RESULT_FAULT_TRACKING,
    CMD_RESULT_FAULT_VELOCITY
} CommandResult_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral_limit;
    float output_limit;
} PID_Params_t;

typedef struct {
    float target_angle;
    float current_angle;
    float error;
    float output;
    bool is_stable;
    uint32_t stable_time_ms;
    ControlMode_t mode;
    PosCtrl_Error_t last_error;
} PositionControl_State_t;

typedef struct {
    uint32_t command_id;
    CommandState_t state;
    CommandSource_t source;
    CommandResult_t result;
    float target_steering_deg;
    float target_motor_deg;
    float start_steering_deg;
    float final_steering_deg;
    float final_error_deg;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t timeout_ms;
} CommandLifecycle_t;

typedef struct {
    float max_error;
    float avg_error;
    uint32_t update_count;
    uint32_t overshoot_count;
    uint32_t max_settle_time_ms;
} PosCtrl_Stats_t;

typedef struct {
    float max_error_allowed;
    float max_velocity;
    uint32_t watchdog_timeout_ms;
} SafetyLimits_t;

typedef void (*PosCtrl_ErrorCallback_t)(PosCtrl_Error_t error);
typedef void (*PosCtrl_StableCallback_t)(void);

int PositionControl_Init(void);
void PositionControl_Update(void);

int PositionControl_SetTarget(float target_deg);
int PositionControl_SetTargetWithSource(float target_deg, CommandSource_t source);
float PositionControl_GetTarget(void);

PositionControl_State_t PositionControl_GetState(void);
CommandLifecycle_t PositionControl_GetCommandLifecycle(void);
float PositionControl_GetCurrentAngle(void);
float PositionControl_GetError(void);
bool PositionControl_IsStable(void);

void PositionControl_SetPID(float Kp, float Ki, float Kd);
void PositionControl_GetPID(PID_Params_t* params);

void PositionControl_SetMode(ControlMode_t mode);
ControlMode_t PositionControl_GetMode(void);
int PositionControl_Enable(void);
void PositionControl_Disable(void);
void PositionControl_Reset(void);

void PositionControl_SetSafetyLimits(SafetyLimits_t* limits);
bool PositionControl_IsSafe(void);
bool PositionControl_CheckSafety(void);
void PositionControl_EmergencyStop(void);
void PositionControl_AbortCommand(CommandResult_t reason);

PosCtrl_Stats_t PositionControl_GetStats(void);
void PositionControl_ResetStats(void);

void PositionControl_RegisterErrorCallback(PosCtrl_ErrorCallback_t callback);
void PositionControl_RegisterStableCallback(PosCtrl_StableCallback_t callback);

void PositionControl_SetDebugLevel(DebugLevel_t level);
void PositionControl_PrintStatus(void);
const char* PositionControl_GetErrorString(PosCtrl_Error_t error);

#endif
