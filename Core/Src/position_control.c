#include "position_control.h"
#include "position_control_diag.h"
#include "position_control_safety.h"
#include "encoder_reader.h"
#include "pulse_control.h"
#include "relay_control.h"
#include "constants.h"
#include "main.h"
#include "latency_profiler.h"

#include <math.h>
#include <stdio.h>

static volatile uint8_t fault_flag = 0U;
static uint32_t command_next_id = 1U;
static CommandSource_t pending_command_source = CMD_SRC_NONE;

static PID_Params_t pid_params = {
    .Kp = DEFAULT_KP,
    .Ki = DEFAULT_KI,
    .Kd = DEFAULT_KD,
    .integral_limit = DEFAULT_INTEGRAL_LIMIT,
    .output_limit = DEFAULT_OUTPUT_LIMIT
};

static struct {
    float prev_error;
    float integral;
    uint32_t last_time_ms;
} pid_state = {0};

static PositionControl_State_t state = {
    .target_angle = 0.0f,
    .current_angle = 0.0f,
    .error = 0.0f,
    .output = 0.0f,
    .is_stable = false,
    .stable_time_ms = 0U,
    .mode = CTRL_MODE_IDLE,
    .last_error = POS_CTRL_OK
};

static volatile bool control_enabled = false;
static volatile ControlMode_t control_mode = CTRL_MODE_IDLE;
static CommandLifecycle_t command_lifecycle = {
    .command_id = 0U,
    .state = CMD_IDLE,
    .source = CMD_SRC_NONE,
    .result = CMD_RESULT_NONE,
    .timeout_ms = POSITION_COMMAND_TIMEOUT_MS
};

static PosCtrl_Stats_t controller_stats = {0};
static PosCtrl_ErrorCallback_t error_callback = NULL;
static PosCtrl_StableCallback_t stable_callback = NULL;
static PosCtrl_Error_t last_notified_error = POS_CTRL_OK;
static DebugLevel_t debug_level = DEBUG_INFO;
static float measured_velocity_deg_per_s = 0.0f;
static float last_velocity_angle = 0.0f;
static float stats_prev_error = 0.0f;
static bool stats_prev_error_valid = false;

#ifdef DBG_LOOP_Pin
#define DBG_LOOP_SET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_SET)
#define DBG_LOOP_RESET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_RESET)
#else
#define DBG_LOOP_SET() ((void)0)
#define DBG_LOOP_RESET() ((void)0)
#endif

#define POSCTRL_LOG(level, ...) \
    do { \
        if ((uint8_t)debug_level >= (uint8_t)(level)) { \
            printf(__VA_ARGS__); \
        } \
    } while (0)

static void PositionControl_ReportError(PosCtrl_Error_t error)
{
    state.last_error = error;

    if (error == POS_CTRL_OK) {
        last_notified_error = POS_CTRL_OK;
        return;
    }

    if ((error_callback != NULL) && (error != last_notified_error)) {
        error_callback(error);
    }

    last_notified_error = error;
}

static void PositionControl_ResetStatsTracking(void)
{
    stats_prev_error = 0.0f;
    stats_prev_error_valid = false;
}

static void PositionControl_ClearStats(void)
{
    controller_stats = (PosCtrl_Stats_t){0};
    PositionControl_ResetStatsTracking();
}

static void PositionControl_RecordStatsSample(float error)
{
    float abs_error = fabsf(error);
    uint32_t next_count = controller_stats.update_count + 1U;

    controller_stats.update_count = next_count;

    if (abs_error > controller_stats.max_error) {
        controller_stats.max_error = abs_error;
    }

    if (next_count == 1U) {
        controller_stats.avg_error = abs_error;
    } else {
        controller_stats.avg_error += (abs_error - controller_stats.avg_error) / (float)next_count;
    }

    if (stats_prev_error_valid) {
        bool prev_outside = fabsf(stats_prev_error) > STABLE_ERROR_THRESHOLD;
        bool curr_outside = abs_error > STABLE_ERROR_THRESHOLD;
        bool crossed_zero = ((stats_prev_error > 0.0f) && (error < 0.0f)) ||
                            ((stats_prev_error < 0.0f) && (error > 0.0f));

        if (prev_outside && curr_outside && crossed_zero) {
            controller_stats.overshoot_count++;
        }
    }

    stats_prev_error = error;
    stats_prev_error_valid = true;
}

static void PositionControl_SyncDiagState(void)
{
    PositionControlDiag_UpdateDebugVars(&state, control_enabled, control_mode, fault_flag);
}

/* Mirror the latest safety evaluation into controller-local fault state. */
static bool PositionControl_ApplySafetyResult(const PositionControlSafetyResult_t* safety_result)
{
    if (safety_result == NULL) {
        fault_flag = 0U;
        PositionControl_ReportError(POS_CTRL_OK);
        return true;
    }

    fault_flag = safety_result->fault_flag;
    PositionControl_ReportError(safety_result->error);
    return safety_result->is_safe;
}

static bool PositionControl_CommandReadyForStart(void)
{
    if (!control_enabled) {
        return false;
    }
    if (control_mode == CTRL_MODE_EMERGENCY) {
        return false;
    }
    if (EncoderReader_IsInitialized() == 0U) {
        return false;
    }
    return true;
}

static void PositionControl_CommandStart(CommandSource_t source)
{
    uint32_t now_ms = HAL_GetTick();
    SafetyLimits_t active_limits = PositionControlSafety_GetLimits();

    command_lifecycle.command_id = command_next_id++;
    command_lifecycle.state = CMD_ACTIVE;
    command_lifecycle.source = source;
    command_lifecycle.result = CMD_RESULT_NONE;
    command_lifecycle.target_steering_deg = MotorDegToSteeringDeg(state.target_angle);
    command_lifecycle.target_motor_deg = state.target_angle;
    command_lifecycle.start_steering_deg = MotorDegToSteeringDeg(state.current_angle);
    command_lifecycle.final_steering_deg = command_lifecycle.start_steering_deg;
    command_lifecycle.final_error_deg = MotorDegToSteeringDeg(state.error);
    command_lifecycle.start_ms = now_ms;
    command_lifecycle.end_ms = 0U;
    command_lifecycle.timeout_ms = active_limits.watchdog_timeout_ms;
    pending_command_source = CMD_SRC_NONE;
    PositionControl_ResetStatsTracking();

    POSCTRL_LOG(DEBUG_INFO,
                "CMD_START,id=%lu,src=%s,target_deg=%.3f,target_motor_deg=%.3f,start_ms=%lu,start_deg=%.3f,start_error_deg=%.3f\r\n",
                (unsigned long)command_lifecycle.command_id,
                PositionControlDiag_CommandSourceString(command_lifecycle.source),
                command_lifecycle.target_steering_deg,
                command_lifecycle.target_motor_deg,
                (unsigned long)command_lifecycle.start_ms,
                command_lifecycle.start_steering_deg,
                command_lifecycle.final_error_deg);
}

static void PositionControl_CommandFinish(CommandState_t end_state, CommandResult_t result, uint32_t now_ms)
{
    if (command_lifecycle.state != CMD_ACTIVE) {
        return;
    }

    pid_state.integral = 0.0f;
    command_lifecycle.state = end_state;
    command_lifecycle.result = result;
    command_lifecycle.end_ms = now_ms;
    command_lifecycle.final_steering_deg = MotorDegToSteeringDeg(state.current_angle);
    command_lifecycle.final_error_deg = MotorDegToSteeringDeg(state.error);

    switch (end_state) {
    case CMD_REACHED:
    {
        uint32_t settling_ms = command_lifecycle.end_ms - command_lifecycle.start_ms;
        if (settling_ms > controller_stats.max_settle_time_ms) {
            controller_stats.max_settle_time_ms = settling_ms;
        }
        POSCTRL_LOG(DEBUG_INFO,
                    "CMD_REACHED,id=%lu,end_ms=%lu,settling_ms=%lu,final_deg=%.3f,final_error_deg=%.3f\r\n",
                    (unsigned long)command_lifecycle.command_id,
                    (unsigned long)command_lifecycle.end_ms,
                    (unsigned long)settling_ms,
                    command_lifecycle.final_steering_deg,
                    command_lifecycle.final_error_deg);
        break;
    }

    case CMD_TIMEOUT:
        POSCTRL_LOG(DEBUG_ERROR,
                    "CMD_TIMEOUT,id=%lu,end_ms=%lu,elapsed_ms=%lu,error_deg=%.3f\r\n",
                    (unsigned long)command_lifecycle.command_id,
                    (unsigned long)command_lifecycle.end_ms,
                    (unsigned long)(command_lifecycle.end_ms - command_lifecycle.start_ms),
                    command_lifecycle.final_error_deg);
        break;

    case CMD_ABORTED:
        POSCTRL_LOG(DEBUG_WARNING,
                    "CMD_ABORT,id=%lu,reason=%s,end_ms=%lu,error_deg=%.3f\r\n",
                    (unsigned long)command_lifecycle.command_id,
                    PositionControlDiag_CommandResultString(result),
                    (unsigned long)command_lifecycle.end_ms,
                    command_lifecycle.final_error_deg);
        break;

    case CMD_FAULTED:
        POSCTRL_LOG(DEBUG_ERROR,
                    "CMD_FAULT,id=%lu,reason=%s,end_ms=%lu,error_deg=%.3f\r\n",
                    (unsigned long)command_lifecycle.command_id,
                    PositionControlDiag_CommandResultString(result),
                    (unsigned long)command_lifecycle.end_ms,
                    command_lifecycle.final_error_deg);
        break;

    case CMD_IDLE:
    case CMD_ACTIVE:
    default:
        break;
    }
}

static float PID_Calculate(float error, float dt)
{
    float p_term = pid_params.Kp * error;
    float i_term = 0.0f;
    float derivative = 0.0f;
    float d_term = 0.0f;
    float output = 0.0f;

    pid_state.integral += error * dt;

    if (pid_state.integral > pid_params.integral_limit) {
        pid_state.integral = pid_params.integral_limit;
    } else if (pid_state.integral < -pid_params.integral_limit) {
        pid_state.integral = -pid_params.integral_limit;
    }

    i_term = pid_params.Ki * pid_state.integral;
    derivative = (error - pid_state.prev_error) / dt;
    d_term = pid_params.Kd * derivative;
    pid_state.prev_error = error;

    output = p_term + i_term + d_term;

    if (output > pid_params.output_limit) {
        output = pid_params.output_limit;
    } else if (output < -pid_params.output_limit) {
        output = -pid_params.output_limit;
    }

    return output;
}

int PositionControl_Init(void)
{
    PositionControlSafety_Init(&(SafetyLimits_t){
        .max_error_allowed = MAX_TRACKING_ERROR_DEG,
        .max_velocity = 0.0f,
        .watchdog_timeout_ms = POSITION_COMMAND_TIMEOUT_MS
    });

    pid_state.prev_error = 0.0f;
    pid_state.integral = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();

    state.target_angle = 0.0f;
    state.current_angle = 0.0f;
    state.error = 0.0f;
    state.output = 0.0f;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    state.mode = CTRL_MODE_IDLE;
    measured_velocity_deg_per_s = 0.0f;
    last_velocity_angle = 0.0f;
    control_enabled = false;
    control_mode = CTRL_MODE_IDLE;
    PositionControl_ReportError(POS_CTRL_OK);
    fault_flag = 0U;
    command_next_id = 1U;
    pending_command_source = CMD_SRC_NONE;
    command_lifecycle.command_id = 0U;
    command_lifecycle.state = CMD_IDLE;
    command_lifecycle.source = CMD_SRC_NONE;
    command_lifecycle.result = CMD_RESULT_NONE;
    command_lifecycle.target_steering_deg = 0.0f;
    command_lifecycle.target_motor_deg = 0.0f;
    command_lifecycle.start_steering_deg = 0.0f;
    command_lifecycle.final_steering_deg = 0.0f;
    command_lifecycle.final_error_deg = 0.0f;
    command_lifecycle.start_ms = 0U;
    command_lifecycle.end_ms = 0U;
    command_lifecycle.timeout_ms = PositionControlSafety_GetLimits().watchdog_timeout_ms;
    PositionControl_ClearStats();
    PositionControl_SyncDiagState();

    POSCTRL_LOG(DEBUG_INFO, "[PosCtrl] Initialized\r\n");
    return POS_CTRL_OK;
}

void PositionControl_Update(void)
{
    bool was_stable = state.is_stable;
    uint32_t current_time = 0U;
    float dt = 0.001f;
    PositionControlSafetyResult_t safety_result = {0};

    DBG_LOOP_SET();

    if (!control_enabled) {
        state.current_angle = EncoderReader_GetAngleDeg();
        state.error = state.target_angle - state.current_angle;
        state.output = 0.0f;
        measured_velocity_deg_per_s = 0.0f;
        last_velocity_angle = state.current_angle;
        PulseControl_Stop();
        PositionControl_SyncDiagState();
        DBG_LOOP_RESET();
        return;
    }

    LAT_BEGIN(LAT_STAGE_SENSE);
    state.current_angle = EncoderReader_GetAngleDeg();
    current_time = HAL_GetTick();
    dt = (current_time - pid_state.last_time_ms) / 1000.0f;
    if (dt <= 0.0f) {
        dt = 0.001f;
    } else if (dt > 0.1f) {
        dt = 0.1f;
    }
    measured_velocity_deg_per_s = (state.current_angle - last_velocity_angle) / dt;
    last_velocity_angle = state.current_angle;
    pid_state.last_time_ms = current_time;
    state.error = state.target_angle - state.current_angle;
    PositionControl_RecordStatsSample(state.error);
    LAT_END(LAT_STAGE_SENSE);

    LAT_BEGIN(LAT_STAGE_CONTROL);
    if ((command_lifecycle.state == CMD_ACTIVE) &&
        (command_lifecycle.timeout_ms > 0U)) {
        uint32_t elapsed_ms = current_time - command_lifecycle.start_ms;
        if (elapsed_ms > command_lifecycle.timeout_ms) {
            fault_flag = 3U;
            PositionControl_ReportError(POS_CTRL_ERR_TIMEOUT);
            state.output = 0.0f;
            PulseControl_Stop();
            control_enabled = false;
            control_mode = CTRL_MODE_EMERGENCY;
            state.mode = CTRL_MODE_EMERGENCY;
            pid_state.integral = 0.0f;
            Relay_Emergency();
            PositionControl_CommandFinish(CMD_TIMEOUT, CMD_RESULT_TIMEOUT, current_time);
            PositionControl_SyncDiagState();
            LAT_END(LAT_STAGE_CONTROL);
            DBG_LOOP_RESET();
            return;
        }
    }

    safety_result = PositionControlSafety_Evaluate(state.current_angle,
                                                   state.error,
                                                   measured_velocity_deg_per_s);
    if (!PositionControl_ApplySafetyResult(&safety_result)) {

        state.output = 0.0f;
        if (command_lifecycle.state == CMD_ACTIVE) {
            PositionControl_CommandFinish(CMD_FAULTED, safety_result.result, current_time);
        }
        PositionControl_SyncDiagState();
        LAT_END(LAT_STAGE_CONTROL);
        PositionControl_EmergencyStop();
        DBG_LOOP_RESET();
        return;
    }

    if (command_lifecycle.state == CMD_REACHED) {
        state.output = 0.0f;
        PulseControl_Stop();
        PositionControl_SyncDiagState();
        LAT_END(LAT_STAGE_CONTROL);
        DBG_LOOP_RESET();
        return;
    }

    state.output = PID_Calculate(state.error, dt);
    LAT_END(LAT_STAGE_CONTROL);

    LAT_BEGIN(LAT_STAGE_ACTUATE);
    PulseControl_SetFrequency((int32_t)state.output);
    LAT_END(LAT_STAGE_ACTUATE);

    if (fabsf(state.error) < STABLE_ERROR_THRESHOLD) {
        state.stable_time_ms += (uint32_t)(dt * 1000.0f);
        if (state.stable_time_ms >= STABLE_TIME_MS) {
            state.is_stable = true;
            if (!was_stable && (stable_callback != NULL)) {
                stable_callback();
            }
            if (command_lifecycle.state == CMD_ACTIVE) {
                PositionControl_CommandFinish(CMD_REACHED, CMD_RESULT_REACHED, current_time);
                state.output = 0.0f;
                PulseControl_Stop();
            }
        }
    } else {
        state.stable_time_ms = 0U;
        state.is_stable = false;
    }

    PositionControl_SyncDiagState();
    DBG_LOOP_RESET();
}

int PositionControl_SetTarget(float target_deg)
{
    return PositionControl_SetTargetWithSource(target_deg, CMD_SRC_NONE);
}

int PositionControl_SetTargetWithSource(float target_deg, CommandSource_t source)
{
    if (target_deg > MAX_ANGLE_DEG || target_deg < MIN_ANGLE_DEG) {
        PositionControl_ReportError(POS_CTRL_ERR_OVER_LIMIT);
        POSCTRL_LOG(DEBUG_ERROR, "[PosCtrl] Reject target %.3f deg (out of range)\r\n", target_deg);
        return POS_CTRL_ERR_OVER_LIMIT;
    }

    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_REPLACED, HAL_GetTick());
    }

    __disable_irq();
    state.target_angle = target_deg;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    pending_command_source = source;
    PositionControl_ResetStatsTracking();
    __enable_irq();

    state.current_angle = EncoderReader_GetAngleDeg();
    state.error = state.target_angle - state.current_angle;
    last_velocity_angle = state.current_angle;

    if (PositionControl_CommandReadyForStart()) {
        PositionControl_CommandStart(source);
    }

    PositionControl_SyncDiagState();
    return POS_CTRL_OK;
}

float PositionControl_GetTarget(void)
{
    return state.target_angle;
}

PositionControl_State_t PositionControl_GetState(void)
{
    return state;
}

CommandLifecycle_t PositionControl_GetCommandLifecycle(void)
{
    return command_lifecycle;
}

float PositionControl_GetCurrentAngle(void)
{
    return state.current_angle;
}

float PositionControl_GetError(void)
{
    return state.error;
}

bool PositionControl_IsStable(void)
{
    return state.is_stable;
}

void PositionControl_SetPID(float Kp, float Ki, float Kd)
{
    pid_params.Kp = Kp;
    pid_params.Ki = Ki;
    pid_params.Kd = Kd;
    pid_state.integral = 0.0f;

    POSCTRL_LOG(DEBUG_INFO,
                "[PosCtrl] PID updated: Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
                Kp,
                Ki,
                Kd);
}

void PositionControl_GetPID(PID_Params_t* params)
{
    if (params != NULL) {
        *params = pid_params;
    }
}

void PositionControl_SetMode(ControlMode_t mode)
{
    control_mode = mode;
    state.mode = mode;
}

ControlMode_t PositionControl_GetMode(void)
{
    return control_mode;
}

int PositionControl_Enable(void)
{
    control_enabled = true;
    fault_flag = 0U;
    control_mode = CTRL_MODE_POSITION;
    state.mode = CTRL_MODE_POSITION;
    PositionControl_ReportError(POS_CTRL_OK);
    Relay_EmergencyRelease();

    state.current_angle = EncoderReader_GetAngleDeg();
    state.error = state.target_angle - state.current_angle;
    pid_state.prev_error = state.error;
    pid_state.integral = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();
    measured_velocity_deg_per_s = 0.0f;
    last_velocity_angle = state.current_angle;
    PositionControl_ResetStatsTracking();

    if ((command_lifecycle.state != CMD_ACTIVE) &&
        (fabsf(state.error) > STABLE_ERROR_THRESHOLD)) {
        PositionControl_CommandStart((pending_command_source != CMD_SRC_NONE) ?
                                     pending_command_source :
                                     CMD_SRC_LOCALTEST);
    }

    POSCTRL_LOG(DEBUG_INFO, "[PosCtrl] Enabled (angle=%.2f)\r\n", state.current_angle);
    PositionControl_SyncDiagState();
    return POS_CTRL_OK;
}

void PositionControl_Disable(void)
{
    if (!control_enabled) {
        return;
    }

    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_DISABLED, HAL_GetTick());
    }

    control_enabled = false;
    control_mode = CTRL_MODE_IDLE;
    state.mode = CTRL_MODE_IDLE;
    state.output = 0.0f;
    measured_velocity_deg_per_s = 0.0f;
    last_velocity_angle = state.current_angle;
    PositionControl_ResetStatsTracking();
    PulseControl_Stop();

    POSCTRL_LOG(DEBUG_INFO, "[PosCtrl] Disabled\r\n");
    PositionControl_SyncDiagState();
}

void PositionControl_Reset(void)
{
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_DISABLED, HAL_GetTick());
    }

    pid_state.integral = 0.0f;
    pid_state.prev_error = 0.0f;
    state.target_angle = 0.0f;
    state.output = 0.0f;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    measured_velocity_deg_per_s = 0.0f;
    last_velocity_angle = state.current_angle;
    PositionControl_ResetStatsTracking();

    POSCTRL_LOG(DEBUG_INFO, "[PosCtrl] Reset\r\n");
    PositionControl_SyncDiagState();
}

void PositionControl_SetSafetyLimits(SafetyLimits_t* limits)
{
    SafetyLimits_t applied_limits = {0};

    if (limits == NULL) {
        return;
    }

    __disable_irq();
    PositionControlSafety_SetLimits(limits);
    applied_limits = PositionControlSafety_GetLimits();
    command_lifecycle.timeout_ms = applied_limits.watchdog_timeout_ms;
    __enable_irq();

    POSCTRL_LOG(DEBUG_INFO,
                "[PosCtrl] Safety limits updated: max_error=%.2f deg, max_velocity=%.2f deg/s, timeout=%lu ms\r\n",
                applied_limits.max_error_allowed,
                applied_limits.max_velocity,
                (unsigned long)applied_limits.watchdog_timeout_ms);
}

bool PositionControl_CheckSafety(void)
{
    PositionControlSafetyResult_t safety_result = PositionControlSafety_Evaluate(state.current_angle,
                                                                                 state.error,
                                                                                 measured_velocity_deg_per_s);

    return PositionControl_ApplySafetyResult(&safety_result);
}

bool PositionControl_IsSafe(void)
{
    return PositionControl_CheckSafety();
}

void PositionControl_EmergencyStop(void)
{
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_ESTOP, HAL_GetTick());
    }

    control_enabled = false;
    control_mode = CTRL_MODE_EMERGENCY;
    state.mode = CTRL_MODE_EMERGENCY;
    state.output = 0.0f;
    PulseControl_Stop();
    pid_state.integral = 0.0f;
    Relay_Emergency();

    POSCTRL_LOG(DEBUG_ERROR,
                "[PosCtrl] EMERGENCY STOP! FLT=%d Ang:%.1f Err:%.1f Vel:%.1f\r\n",
                (int)fault_flag,
                state.current_angle,
                state.error,
                measured_velocity_deg_per_s);

    measured_velocity_deg_per_s = 0.0f;
    PositionControl_SyncDiagState();
}

void PositionControl_AbortCommand(CommandResult_t reason)
{
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, reason, HAL_GetTick());
    }
}

PosCtrl_Stats_t PositionControl_GetStats(void)
{
    PosCtrl_Stats_t snapshot;

    __disable_irq();
    snapshot = controller_stats;
    __enable_irq();

    return snapshot;
}

void PositionControl_ResetStats(void)
{
    __disable_irq();
    PositionControl_ClearStats();
    __enable_irq();
}

void PositionControl_RegisterErrorCallback(PosCtrl_ErrorCallback_t callback)
{
    __disable_irq();
    error_callback = callback;
    last_notified_error = POS_CTRL_OK;
    __enable_irq();
}

void PositionControl_RegisterStableCallback(PosCtrl_StableCallback_t callback)
{
    __disable_irq();
    stable_callback = callback;
    __enable_irq();
}

void PositionControl_SetDebugLevel(DebugLevel_t level)
{
    if ((uint8_t)level > (uint8_t)DEBUG_VERBOSE) {
        level = DEBUG_VERBOSE;
    }
    debug_level = level;
}

void PositionControl_PrintStatus(void)
{
    PositionControlDiag_PrintStateSummary(&state,
                                          control_enabled,
                                          fault_flag,
                                          &command_lifecycle);
}
