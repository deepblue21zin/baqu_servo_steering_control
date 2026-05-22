#include "position_control.h"
#include "position_control_diag.h"
#include "position_control_safety.h"
#include "encoder_reader.h"
#include "pulse_control.h"
#include "constants.h"
#include "main.h"
#include "latency_profiler.h"

#include <math.h>

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
    float derivative_filtered;
    float last_output_hz;
    uint32_t last_time_ms;
} pid_state = {0};

static PositionControl_State_t state = {
    .target_steering_deg = 0.0f,
    .current_steering_deg = 0.0f,
    .error_steering_deg = 0.0f,
    .target_motor_deg = 0.0f,
    .current_motor_deg = 0.0f,
    .error_motor_deg = 0.0f,
    .output_hz = 0.0f,
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

static float stable_error_motor_deg = STABLE_ERROR_MOTOR_DEG;
static uint32_t stable_time_setting_ms = STABLE_TIME_MS;
static float measured_velocity_motor_deg_per_s = 0.0f;
static float last_velocity_motor_deg = 0.0f;

#ifdef DBG_LOOP_Pin
#define DBG_LOOP_SET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_SET)
#define DBG_LOOP_RESET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_RESET)
#else
#define DBG_LOOP_SET() ((void)0)
#define DBG_LOOP_RESET() ((void)0)
#endif

static SafetyLimits_t PositionControl_GetConfiguredSafetyLimits(void)
{
    SafetyLimits_t limits = {
        .max_error_motor_deg = 0.0f,
        .max_velocity_motor_deg_per_s = 0.0f,
        .watchdog_timeout_ms = 0U
    };

#if POSITION_FAILSAFE_EXTRA_ENABLE
#if POSITION_FAILSAFE_PROFILE == POSITION_FAILSAFE_PROFILE_PARAM_TEST
    limits.max_error_motor_deg = POSITION_FAILSAFE_PARAM_TEST_MAX_ERROR_MOTOR_DEG;
    limits.max_velocity_motor_deg_per_s = POSITION_FAILSAFE_PARAM_TEST_MAX_VELOCITY_MOTOR_DEG_PER_S;
    limits.watchdog_timeout_ms = POSITION_FAILSAFE_PARAM_TEST_TIMEOUT_MS;
#elif POSITION_FAILSAFE_PROFILE == POSITION_FAILSAFE_PROFILE_VEHICLE_TEST
    limits.max_error_motor_deg = POSITION_FAILSAFE_VEHICLE_TEST_MAX_ERROR_MOTOR_DEG;
    limits.max_velocity_motor_deg_per_s = POSITION_FAILSAFE_VEHICLE_TEST_MAX_VELOCITY_MOTOR_DEG_PER_S;
    limits.watchdog_timeout_ms = POSITION_FAILSAFE_VEHICLE_TEST_TIMEOUT_MS;
#endif
#endif

    return limits;
}

static void PositionControl_ReportError(PosCtrl_Error_t error)
{
    state.last_error = error;
}

static void PositionControl_UpdateSteeringSnapshot(void)
{
    state.target_steering_deg = MotorDegToSteeringDeg(state.target_motor_deg);
    state.current_steering_deg = MotorDegToSteeringDeg(state.current_motor_deg);
    state.error_steering_deg = MotorDegToSteeringDeg(state.error_motor_deg);
}

static void PositionControl_SyncDiagState(void)
{
    PositionControl_UpdateSteeringSnapshot();
    PositionControlDiag_UpdateDebugVars(&state);
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

    PositionControl_UpdateSteeringSnapshot();

    command_lifecycle.command_id = command_next_id++;
    command_lifecycle.state = CMD_ACTIVE;
    command_lifecycle.source = source;
    command_lifecycle.result = CMD_RESULT_NONE;
    command_lifecycle.target_steering_deg = state.target_steering_deg;
    command_lifecycle.target_motor_deg = state.target_motor_deg;
    command_lifecycle.start_steering_deg = state.current_steering_deg;
    command_lifecycle.final_steering_deg = command_lifecycle.start_steering_deg;
    command_lifecycle.final_error_steering_deg = state.error_steering_deg;
    command_lifecycle.start_ms = now_ms;
    command_lifecycle.end_ms = 0U;
    command_lifecycle.timeout_ms = active_limits.watchdog_timeout_ms;
    pending_command_source = CMD_SRC_NONE;
}

static void PositionControl_CommandFinish(CommandState_t end_state, CommandResult_t result, uint32_t now_ms)
{
    if (command_lifecycle.state != CMD_ACTIVE) {
        return;
    }

    PositionControl_UpdateSteeringSnapshot();

    pid_state.integral = 0.0f;
    command_lifecycle.state = end_state;
    command_lifecycle.result = result;
    command_lifecycle.end_ms = now_ms;
    command_lifecycle.final_steering_deg = state.current_steering_deg;
    command_lifecycle.final_error_steering_deg = state.error_steering_deg;
}

float PositionControl_GetStableErrorMotorDeg(void)
{
    return stable_error_motor_deg;
}

uint32_t PositionControl_GetStableTimeMs(void)
{
    return stable_time_setting_ms;
}

void PositionControl_SetStableCriteriaMotorDeg(float error_threshold_motor_deg, uint32_t stable_time_ms)
{
    if (error_threshold_motor_deg <= 0.0f) {
        error_threshold_motor_deg = STABLE_ERROR_MOTOR_DEG;
    }
    if (error_threshold_motor_deg > 100.0f) {
        error_threshold_motor_deg = 100.0f;
    }

    if (stable_time_ms == 0U) {
        stable_time_ms = STABLE_TIME_MS;
    }
    if (stable_time_ms > 10000U) {
        stable_time_ms = 10000U;
    }

    stable_error_motor_deg = error_threshold_motor_deg;
    stable_time_setting_ms = stable_time_ms;
}

static float PID_Calculate(float error, float dt)
{
    float p_term = pid_params.Kp * error;
    float i_term = 0.0f;
    float derivative_raw = 0.0f;
    float derivative = 0.0f;
    float d_term = 0.0f;
    float output = 0.0f;
    float alpha = DEFAULT_D_FILTER_ALPHA;

    pid_state.integral += error * dt;

    if (pid_state.integral > pid_params.integral_limit) {
        pid_state.integral = pid_params.integral_limit;
    } else if (pid_state.integral < -pid_params.integral_limit) {
        pid_state.integral = -pid_params.integral_limit;
    }

    i_term = pid_params.Ki * pid_state.integral;
    derivative_raw = (error - pid_state.prev_error) / dt;
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 0.99f) {
        alpha = 0.99f;
    }
    derivative = (alpha * pid_state.derivative_filtered) + ((1.0f - alpha) * derivative_raw);
    pid_state.derivative_filtered = derivative;
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

static float PositionControl_ApplyOutputShaping(float requested_output_hz, float dt)
{
    if (POSITION_OUTPUT_SLEW_HZ_PER_S > 0.0f) {
        float max_delta_hz = POSITION_OUTPUT_SLEW_HZ_PER_S * dt;
        float delta_hz = requested_output_hz - pid_state.last_output_hz;

        if (delta_hz > max_delta_hz) {
            requested_output_hz = pid_state.last_output_hz + max_delta_hz;
        } else if (delta_hz < -max_delta_hz) {
            requested_output_hz = pid_state.last_output_hz - max_delta_hz;
        }
    }

    pid_state.last_output_hz = requested_output_hz;
    return requested_output_hz;
}

static void PositionControl_ResetPidDynamicState(float current_error_motor_deg)
{
    pid_state.prev_error = current_error_motor_deg;
    pid_state.integral = 0.0f;
    pid_state.derivative_filtered = 0.0f;
    pid_state.last_output_hz = 0.0f;
}

static bool PositionControl_ShouldHoldWithoutRecontrol(void)
{
    return fabsf(state.error_motor_deg) <= POSITION_HOLD_REARM_ERROR_MOTOR_DEG;
}

static void PositionControl_RearmHoldControl(uint32_t now_ms)
{
    command_lifecycle.state = CMD_ACTIVE;
    command_lifecycle.result = CMD_RESULT_NONE;
    command_lifecycle.start_ms = now_ms;
    command_lifecycle.end_ms = 0U;
    command_lifecycle.timeout_ms = PositionControlSafety_GetLimits().watchdog_timeout_ms;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    PositionControl_ResetPidDynamicState(state.error_motor_deg);
}

int PositionControl_Init(void)
{
    SafetyLimits_t configured_limits = PositionControl_GetConfiguredSafetyLimits();

    PositionControlSafety_Init(&configured_limits);

    pid_state.prev_error = 0.0f;
    pid_state.integral = 0.0f;
    pid_state.derivative_filtered = 0.0f;
    pid_state.last_output_hz = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();

    state.target_motor_deg = 0.0f;
    state.current_motor_deg = 0.0f;
    state.error_motor_deg = 0.0f;
    PositionControl_UpdateSteeringSnapshot();
    state.output_hz = 0.0f;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    state.mode = CTRL_MODE_IDLE;
    measured_velocity_motor_deg_per_s = 0.0f;
    last_velocity_motor_deg = 0.0f;
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
    command_lifecycle.final_error_steering_deg = 0.0f;
    command_lifecycle.start_ms = 0U;
    command_lifecycle.end_ms = 0U;
    command_lifecycle.timeout_ms = PositionControlSafety_GetLimits().watchdog_timeout_ms;
    PositionControl_SyncDiagState();

    return POS_CTRL_OK;
}

void PositionControl_UpdateWithCurrentMotorDeg(float current_motor_deg)
{
    uint32_t current_time = 0U;
    float dt = 0.001f;
    PositionControlSafetyResult_t safety_result = {0};
    PositionControlSafetyResult_t timeout_result = {0};

    DBG_LOOP_SET();

    if (!control_enabled) {
        state.current_motor_deg = current_motor_deg;
        state.error_motor_deg = state.target_motor_deg - state.current_motor_deg;
        state.output_hz = 0.0f;
        measured_velocity_motor_deg_per_s = 0.0f;
        last_velocity_motor_deg = state.current_motor_deg;
        pid_state.last_output_hz = 0.0f;
        PulseControl_Stop();
        PositionControl_SyncDiagState();
        DBG_LOOP_RESET();
        return;
    }

    LAT_BEGIN(LAT_STAGE_SENSE);
    state.current_motor_deg = current_motor_deg;
    current_time = HAL_GetTick();
    dt = (current_time - pid_state.last_time_ms) / 1000.0f;
    if (dt <= 0.0f) {
        dt = 0.001f;
    } else if (dt > 0.1f) {
        dt = 0.1f;
    }
    measured_velocity_motor_deg_per_s = (state.current_motor_deg - last_velocity_motor_deg) / dt;
    last_velocity_motor_deg = state.current_motor_deg;
    pid_state.last_time_ms = current_time;
    state.error_motor_deg = state.target_motor_deg - state.current_motor_deg;
    LAT_END(LAT_STAGE_SENSE);

    LAT_BEGIN(LAT_STAGE_CONTROL);
    if (command_lifecycle.state == CMD_ACTIVE) {
        timeout_result = PositionControlSafety_EvaluateCommandTimeout(command_lifecycle.start_ms,
                                                                      command_lifecycle.timeout_ms,
                                                                      current_time);
        if (!PositionControl_ApplySafetyResult(&timeout_result)) {
            state.output_hz = 0.0f;
            pid_state.last_output_hz = 0.0f;
            PulseControl_Stop();
            control_enabled = false;
            control_mode = CTRL_MODE_EMERGENCY;
            state.mode = CTRL_MODE_EMERGENCY;
            PositionControl_ResetPidDynamicState(state.error_motor_deg);
            PositionControl_CommandFinish(CMD_TIMEOUT, CMD_RESULT_TIMEOUT, current_time);
            PositionControl_SyncDiagState();
            LAT_END(LAT_STAGE_CONTROL);
            DBG_LOOP_RESET();
            return;
        }
    }

    safety_result = PositionControlSafety_Evaluate(state.current_motor_deg,
                                                   state.error_motor_deg,
                                                   measured_velocity_motor_deg_per_s);
    if (!PositionControl_ApplySafetyResult(&safety_result)) {

        state.output_hz = 0.0f;
        pid_state.last_output_hz = 0.0f;
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
        if (PositionControl_ShouldHoldWithoutRecontrol()) {
            state.output_hz = 0.0f;
            pid_state.last_output_hz = 0.0f;
            PulseControl_Stop();
            PositionControl_SyncDiagState();
            LAT_END(LAT_STAGE_CONTROL);
            DBG_LOOP_RESET();
            return;
        }

        PositionControl_RearmHoldControl(current_time);
    }

    state.output_hz = PositionControl_ApplyOutputShaping(PID_Calculate(state.error_motor_deg, dt), dt);
    LAT_END(LAT_STAGE_CONTROL);

    LAT_BEGIN(LAT_STAGE_ACTUATE);
    PulseControl_SetFrequency((int32_t)state.output_hz);
    LAT_END(LAT_STAGE_ACTUATE);

    if (fabsf(state.error_motor_deg) < PositionControl_GetStableErrorMotorDeg()) {
        state.stable_time_ms += (uint32_t)(dt * 1000.0f);
        if (state.stable_time_ms >= PositionControl_GetStableTimeMs()) {
            state.is_stable = true;
            if (command_lifecycle.state == CMD_ACTIVE) {
                PositionControl_CommandFinish(CMD_REACHED, CMD_RESULT_REACHED, current_time);
                state.output_hz = 0.0f;
                pid_state.last_output_hz = 0.0f;
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

void PositionControl_Update(void)
{
    PositionControl_UpdateWithCurrentMotorDeg(EncoderReader_GetMotorDeg());
}

int PositionControl_SetTargetMotorDeg(float target_motor_deg)
{
    return PositionControl_SetTargetMotorDegWithSource(target_motor_deg, CMD_SRC_NONE);
}

int PositionControl_SetTargetMotorDegWithSource(float target_motor_deg, CommandSource_t source)
{
    if (target_motor_deg > MAX_MOTOR_ANGLE_DEG || target_motor_deg < MIN_MOTOR_ANGLE_DEG) {
        PositionControl_ReportError(POS_CTRL_ERR_OVER_LIMIT);
        return POS_CTRL_ERR_OVER_LIMIT;
    }

    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_REPLACED, HAL_GetTick());
    }

    __disable_irq();
    state.target_motor_deg = target_motor_deg;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    pending_command_source = source;
    __enable_irq();

    state.current_motor_deg = EncoderReader_GetMotorDeg();
    state.error_motor_deg = state.target_motor_deg - state.current_motor_deg;
    last_velocity_motor_deg = state.current_motor_deg;
    PositionControl_ResetPidDynamicState(state.error_motor_deg);

    if (PositionControl_CommandReadyForStart()) {
        PositionControl_CommandStart(source);
    }

    PositionControl_SyncDiagState();
    return POS_CTRL_OK;
}

int PositionControl_SetTargetSteeringDeg(float target_steering_deg)
{
    return PositionControl_SetTargetSteeringDegWithSource(target_steering_deg, CMD_SRC_NONE);
}

int PositionControl_SetTargetSteeringDegWithSource(float target_steering_deg, CommandSource_t source)
{
    return PositionControl_SetTargetMotorDegWithSource(SteeringDegToMotorDeg(target_steering_deg), source);
}

float PositionControl_GetTargetMotorDeg(void)
{
    return state.target_motor_deg;
}

PositionControl_State_t PositionControl_GetState(void)
{
    PositionControl_UpdateSteeringSnapshot();
    return state;
}

CommandLifecycle_t PositionControl_GetCommandLifecycle(void)
{
    return command_lifecycle;
}

float PositionControl_GetCurrentMotorDeg(void)
{
    return state.current_motor_deg;
}

float PositionControl_GetErrorMotorDeg(void)
{
    return state.error_motor_deg;
}

bool PositionControl_IsStable(void)
{
    return state.is_stable;
}

void PositionControl_SetPID(float Kp, float Ki, float Kd)
{
    PID_Params_t params = pid_params;

    params.Kp = Kp;
    params.Ki = Ki;
    params.Kd = Kd;
    PositionControl_SetPIDParams(&params);
}

void PositionControl_SetPIDParams(const PID_Params_t* params)
{
    if (params == NULL) {
        return;
    }

    pid_params = *params;
    PositionControl_ResetPidDynamicState(state.error_motor_deg);
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

    state.current_motor_deg = EncoderReader_GetMotorDeg();
    state.error_motor_deg = state.target_motor_deg - state.current_motor_deg;
    PositionControl_ResetPidDynamicState(state.error_motor_deg);
    pid_state.last_time_ms = HAL_GetTick();
    measured_velocity_motor_deg_per_s = 0.0f;
    last_velocity_motor_deg = state.current_motor_deg;

    if ((command_lifecycle.state != CMD_ACTIVE) &&
        (fabsf(state.error_motor_deg) > PositionControl_GetStableErrorMotorDeg())) {
        PositionControl_CommandStart((pending_command_source != CMD_SRC_NONE) ?
                                     pending_command_source :
                                     CMD_SRC_LOCALTEST);
    }

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
    state.output_hz = 0.0f;
    measured_velocity_motor_deg_per_s = 0.0f;
    last_velocity_motor_deg = state.current_motor_deg;
    PositionControl_ResetPidDynamicState(state.error_motor_deg);
    PulseControl_Stop();

    PositionControl_SyncDiagState();
}

void PositionControl_Reset(void)
{
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_DISABLED, HAL_GetTick());
    }

    state.target_motor_deg = 0.0f;
    state.error_motor_deg = state.target_motor_deg - state.current_motor_deg;
    state.output_hz = 0.0f;
    state.is_stable = false;
    state.stable_time_ms = 0U;
    measured_velocity_motor_deg_per_s = 0.0f;
    last_velocity_motor_deg = state.current_motor_deg;
    PositionControl_ResetPidDynamicState(state.error_motor_deg);

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
}

bool PositionControl_CheckSafety(void)
{
    PositionControlSafetyResult_t safety_result = PositionControlSafety_Evaluate(state.current_motor_deg,
                                                                                 state.error_motor_deg,
                                                                                 measured_velocity_motor_deg_per_s);

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
#if APP_RUNTIME_EMERGENCY_LATCH_ENABLE
    control_mode = CTRL_MODE_EMERGENCY;
    state.mode = CTRL_MODE_EMERGENCY;
#else
    control_mode = CTRL_MODE_IDLE;
    state.mode = CTRL_MODE_IDLE;
#endif
    state.output_hz = 0.0f;
    PulseControl_Stop();
    PositionControl_ResetPidDynamicState(state.error_motor_deg);

    measured_velocity_motor_deg_per_s = 0.0f;
    PositionControl_SyncDiagState();
}

void PositionControl_AbortCommand(CommandResult_t reason)
{
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, reason, HAL_GetTick());
    }
}
