#include "position_control.h"
#include "encoder_reader.h"      // 엔코더 읽기
#include "pulse_control.h"       // 펄스 출력
#include "relay_control.h"       // 릴레이 제어 (EmergencyStop에서 사용)
#include "constants.h"
#include "debug_vars.h"
#include <stdio.h>
#include <stdint.h>              // uint32_t, int32_t
#include <math.h>                // fabsf
#include "main.h"
#include "latency_profiler.h"

//의존성

// 내부 변수 지정
static volatile uint8_t debug_enabled = 0; // 디버그 메시지 출력 여부
static volatile uint8_t fault_flag = 0; // 안전 관련 플래그
static uint32_t command_next_id = 1U;
static CommandSource_t pending_command_source = CMD_SRC_NONE;

#define POSITION_COMMAND_TIMEOUT_MS 3000U
#define POSITION_COMMAND_TIMEOUT_FAILSAFE_ENABLE 0U

static PID_Params_t pid_params = {
    .Kp = 50.0f,           // 초기값 (실험으로 튜닝 필요)
    .Ki = 5.0f,
    .Kd = 20.0f,
    .integral_limit = 1000.0f,
    .output_limit = 10000.0f   // 최대 펄스 주파수 (Hz)
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
    .stable_time_ms = 0
};

static volatile bool control_enabled = false;
static volatile ControlMode_t control_mode = CTRL_MODE_IDLE;
static PosCtrl_Stats_t stats = {0};
static CommandLifecycle_t command_lifecycle = {
    .command_id = 0U,
    .state = CMD_IDLE,
    .source = CMD_SRC_NONE,
    .result = CMD_RESULT_NONE,
    .timeout_ms = POSITION_COMMAND_TIMEOUT_MS
};

int PositionControl_SetTargetWithSource(float target_deg, CommandSource_t source);

#ifdef DBG_LOOP_Pin
#define DBG_LOOP_SET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_SET)
#define DBG_LOOP_RESET() HAL_GPIO_WritePin(DBG_LOOP_GPIO_Port, DBG_LOOP_Pin, GPIO_PIN_RESET)
#else
#define DBG_LOOP_SET() ((void)0)
#define DBG_LOOP_RESET() ((void)0)
#endif

static int32_t PositionControl_DegToMilliDeg(float deg)
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

static int16_t PositionControl_OutputToDebugCmd(float output)
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

static const char* PositionControl_CommandSourceString(CommandSource_t source)
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

static const char* PositionControl_CommandResultString(CommandResult_t result)
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
    case CMD_RESULT_NONE:
    default:
        return "NONE";
    }
}

static const char* PositionControl_CommandStateString(CommandState_t state_value)
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
    command_lifecycle.timeout_ms = POSITION_COMMAND_TIMEOUT_MS;
    pending_command_source = CMD_SRC_NONE;

    printf("CMD_START,id=%lu,src=%s,target_deg=%.3f,target_motor_deg=%.3f,start_ms=%lu,start_deg=%.3f,start_error_deg=%.3f\r\n",
           (unsigned long)command_lifecycle.command_id,
           PositionControl_CommandSourceString(command_lifecycle.source),
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
        printf("CMD_REACHED,id=%lu,end_ms=%lu,settling_ms=%lu,final_deg=%.3f,final_error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               (unsigned long)command_lifecycle.end_ms,
               (unsigned long)(command_lifecycle.end_ms - command_lifecycle.start_ms),
               command_lifecycle.final_steering_deg,
               command_lifecycle.final_error_deg);
        break;

    case CMD_TIMEOUT:
        printf("CMD_TIMEOUT,id=%lu,end_ms=%lu,elapsed_ms=%lu,error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               (unsigned long)command_lifecycle.end_ms,
               (unsigned long)(command_lifecycle.end_ms - command_lifecycle.start_ms),
               command_lifecycle.final_error_deg);
        break;

    case CMD_ABORTED:
        printf("CMD_ABORT,id=%lu,reason=%s,end_ms=%lu,error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               PositionControl_CommandResultString(result),
               (unsigned long)command_lifecycle.end_ms,
               command_lifecycle.final_error_deg);
        break;

    case CMD_FAULTED:
        printf("CMD_FAULT,id=%lu,reason=%s,end_ms=%lu,error_deg=%.3f\r\n",
               (unsigned long)command_lifecycle.command_id,
               PositionControl_CommandResultString(result),
               (unsigned long)command_lifecycle.end_ms,
               command_lifecycle.final_error_deg);
        break;

    case CMD_IDLE:
    case CMD_ACTIVE:
    default:
        break;
    }
}

static uint32_t PositionControl_BuildDebugFaultFlags(void)
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
    if (!control_enabled) {
        flags |= DBG_FAULT_DISABLED;
    }
    if (control_mode == CTRL_MODE_EMERGENCY) {
        flags |= DBG_FAULT_EMERGENCY;
    }

    return flags;
}

static void PositionControl_UpdateDebugVars(void)
{
    dbg_enc_raw = (int32_t)EncoderReader_GetRawCounter();
    dbg_pos_mdeg = PositionControl_DegToMilliDeg(MotorDegToSteeringDeg(state.current_angle));
    dbg_target_mdeg = PositionControl_DegToMilliDeg(MotorDegToSteeringDeg(state.target_angle));
    dbg_err_mdeg = PositionControl_DegToMilliDeg(MotorDegToSteeringDeg(state.error));
    dbg_pwm_cmd = PositionControl_OutputToDebugCmd(state.output);
    dbg_fault_flags = PositionControl_BuildDebugFaultFlags();
}

// ========== 초기화 ==========
// Init()함수를 호출 안하는 프로그램의 경우 선언시 초기화만 해도 된다.
//런타임에 재초기화가 필요한 경우

int PositionControl_Init(void) {
    pid_state.prev_error = 0.0f;  //D항 계산: (현재오차 - 이전오차)/dt, 초기화 안하면 폭주 가능
    pid_state.integral = 0.0f;   //적분 누적값 초기화
    pid_state.last_time_ms = HAL_GetTick();  //HAL_GetTick():STM32HAL함수, 시스템 시작후 경과 시간 반환(dt계산을 위한 기준점 설정)
    
    state.target_angle = 0.0f;
    state.current_angle = 0.0f;
    control_enabled = false; //제어 비활성화 상태로 시작, 안전장치, Enable() 함수를 명시적으로 호출해야 제어 시작
    control_mode = CTRL_MODE_IDLE;
    state.mode = CTRL_MODE_IDLE;
    state.last_error = POS_CTRL_OK;
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
    command_lifecycle.timeout_ms = POSITION_COMMAND_TIMEOUT_MS;

    dbg_enc_raw = 0;
    dbg_pos_mdeg = 0;
    dbg_target_mdeg = 0;
    dbg_err_mdeg = 0;
    dbg_pwm_cmd = 0;
    dbg_fault_flags = DBG_FAULT_DISABLED;

#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Initialized\n");
#endif
    return POS_CTRL_OK;  // 성공
}

// ========== PID 계산 ==========
static float PID_Calculate(float error, float dt) {
    // P항
    float p_term = pid_params.Kp * error;
    
    // I항 (적분 와인드업 방지)
    pid_state.integral += error * dt;
    
    // 적분 제한(최대치 제한)
    if (pid_state.integral > pid_params.integral_limit) {
        pid_state.integral = pid_params.integral_limit;
    } else if (pid_state.integral < -pid_params.integral_limit) {
        pid_state.integral = -pid_params.integral_limit;
    }
    
    float i_term = pid_params.Ki * pid_state.integral;
    
    // D항 (미분)
    float derivative = (error - pid_state.prev_error) / dt;
    float d_term = pid_params.Kd * derivative;
    
    pid_state.prev_error = error;
    
    // 출력 계산
    float output = p_term + i_term + d_term;
    
    // 출력 제한
    if (output > pid_params.output_limit) {
        output = pid_params.output_limit;
    } else if (output < -pid_params.output_limit) {
        output = -pid_params.output_limit;
    }
    
    return output;
}

// ========== 메인 제어 루프 (1ms마다 호출!) ==========
void PositionControl_Update(void) {
    DBG_LOOP_SET();

    if (!control_enabled) {
        state.current_angle = EncoderReader_GetAngleDeg();
        state.error = state.target_angle - state.current_angle;
        state.output = 0.0f;
        PulseControl_Stop();
        PositionControl_UpdateDebugVars();
        DBG_LOOP_RESET();
        return;
    }

    LAT_BEGIN(LAT_STAGE_SENSE);
    // 1. 현재 각도 읽기
    state.current_angle = EncoderReader_GetAngleDeg();

    // 2. 오차 계산 (안전 체크보다 먼저!)
    // [BUG FIX] 기존 코드: 안전 체크가 오차 계산보다 앞에 있어서
    // CheckSafety()가 이전 루프의 state.error를 참조하는 1-step 지연 버그 존재.
    // 수정: 오차를 먼저 계산하고 안전 체크 수행.
    state.error = state.target_angle - state.current_angle;
    LAT_END(LAT_STAGE_SENSE);

    // 3. 안전 체크 (현재 오차 기준으로 판단)
    LAT_BEGIN(LAT_STAGE_CONTROL);
    if (!PositionControl_CheckSafety()) {
        CommandResult_t fault_result = CMD_RESULT_NONE;

        if (fault_flag == 1U) {
            fault_result = CMD_RESULT_FAULT_LIMIT;
        } else if (fault_flag == 2U) {
            fault_result = CMD_RESULT_FAULT_TRACKING;
        }

        state.output = 0.0f;
        if (command_lifecycle.state == CMD_ACTIVE) {
            PositionControl_CommandFinish(CMD_FAULTED, fault_result, HAL_GetTick());
        }
        PositionControl_UpdateDebugVars();
        LAT_END(LAT_STAGE_CONTROL);
        PositionControl_EmergencyStop();
        DBG_LOOP_RESET();
        return;
    }
    
    // 4. 시간 계산
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid_state.last_time_ms) / 1000.0f;  // ms → s
    //dt가 0 이되면 0으로 나누는게 되버림(D항 계산에서 폭주 가능), dt가 너무 크면 제어 성능 저하, 0.001s(1ms)보다 너무 크면 제어 성능 저하, 0.001s보다 너무 작으면 D항 계산에서 노이즈 영향 커짐
    if (dt <= 0.0f) {
        dt = 0.001f;  // 최소 dt 보장 (1ms)
        
    }else if (dt > 0.1f) {
        dt = 0.1f;    // 최대 dt 제한 (100ms)
    }

    pid_state.last_time_ms = current_time;

    if (POSITION_COMMAND_TIMEOUT_FAILSAFE_ENABLE != 0U &&
        command_lifecycle.state == CMD_ACTIVE) {
        uint32_t elapsed_ms = current_time - command_lifecycle.start_ms;
        if (elapsed_ms > command_lifecycle.timeout_ms) {
            fault_flag = 3U;
            state.last_error = POS_CTRL_ERR_TIMEOUT;
            state.output = 0.0f;
            PulseControl_Stop();
            control_enabled = false;
            control_mode = CTRL_MODE_EMERGENCY;
            state.mode = CTRL_MODE_EMERGENCY;
            pid_state.integral = 0.0f;
            Relay_Emergency();
            PositionControl_CommandFinish(CMD_TIMEOUT, CMD_RESULT_TIMEOUT, current_time);
            PositionControl_UpdateDebugVars();
            LAT_END(LAT_STAGE_CONTROL);
            DBG_LOOP_RESET();
            return;
        }
    }

    if (command_lifecycle.state == CMD_REACHED) {
        state.output = 0.0f;
        PulseControl_Stop();
        PositionControl_UpdateDebugVars();
        LAT_END(LAT_STAGE_CONTROL);
        DBG_LOOP_RESET();
        return;
    }
    
    // 5. PID 계산
    state.output = PID_Calculate(state.error, dt);
    LAT_END(LAT_STAGE_CONTROL);
    
    // 6. 펄스 출력
    LAT_BEGIN(LAT_STAGE_ACTUATE);
    PulseControl_SetFrequency((int32_t)state.output);
    LAT_END(LAT_STAGE_ACTUATE);
    
    // 7. 안정화 판단
    if (fabsf(state.error) < POSITION_TOLERANCE) {
        state.stable_time_ms += (uint32_t)(dt * 1000.0f); // 안정 유지 시간 누적
        if (state.stable_time_ms > 100) {  // 100ms 이상 안정
            state.is_stable = true;
            if (command_lifecycle.state == CMD_ACTIVE) {
                PositionControl_CommandFinish(CMD_REACHED, CMD_RESULT_REACHED, current_time);
                state.output = 0.0f;
                PulseControl_Stop();
            }
        }
    } else {
        state.stable_time_ms = 0;
        state.is_stable = false;
    }

    PositionControl_UpdateDebugVars();
    DBG_LOOP_RESET();
}

// ========== 목표 설정 ==========
int PositionControl_SetTarget(float target_deg) {
    return PositionControl_SetTargetWithSource(target_deg, CMD_SRC_NONE);
}

int PositionControl_SetTargetWithSource(float target_deg, CommandSource_t source) {
    // 범위 체크
    if (target_deg > MAX_ANGLE_DEG || target_deg < MIN_ANGLE_DEG) {
        return POS_CTRL_ERR_OVER_LIMIT;
    }

    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_REPLACED, HAL_GetTick());
    }

    __disable_irq();
    state.target_angle = target_deg;
    state.is_stable = false;
    state.stable_time_ms = 0;
    pending_command_source = source;
    __enable_irq();

    state.current_angle = EncoderReader_GetAngleDeg();
    state.error = state.target_angle - state.current_angle;

    if (PositionControl_CommandReadyForStart()) {
        PositionControl_CommandStart(source);
    }

    PositionControl_UpdateDebugVars();

    return POS_CTRL_OK;
}

// ========== 상태 읽기 ==========
float PositionControl_GetTarget(void) {
    return state.target_angle;
}
float PositionControl_GetError(void) {
    return state.error;
}
void PositionControl_GetPID(PID_Params_t* params) {
    if (params != NULL) {
        *params = pid_params;
    }
}
PositionControl_State_t PositionControl_GetState(void) {
    return state;
}

CommandLifecycle_t PositionControl_GetCommandLifecycle(void) {
    return command_lifecycle;
}

float PositionControl_GetCurrentAngle(void) {
    return state.current_angle;
}

bool PositionControl_IsStable(void) {
    return state.is_stable;
}

// ========== PID 게인 설정 ==========
void PositionControl_SetPID(float Kp, float Ki, float Kd) {
    pid_params.Kp = Kp;
    pid_params.Ki = Ki;
    pid_params.Kd = Kd;
    
    // 적분 리셋
    pid_state.integral = 0.0f;
    
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] PID updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
           Kp, Ki, Kd);
#endif
}

// ========== 제어 모드 ==========
void PositionControl_SetMode(ControlMode_t mode) {
    control_mode = mode;
    state.mode = mode;
}
ControlMode_t PositionControl_GetMode(void) {
    return control_mode;
}

int PositionControl_Enable(void) {
    control_enabled = true;
    fault_flag = 0;              // EmergencyStop 후 재활성화 시 fault 초기화
    control_mode = CTRL_MODE_POSITION;
    state.mode = CTRL_MODE_POSITION;
    state.last_error = POS_CTRL_OK;
    Relay_EmergencyRelease();

    // [BUG FIX] D항 킥 방지 (Derivative Kick Prevention)
    // 기존: prev_error=0으로 초기화 → 첫 호출 시 D=(error-0)/0.001=10000 → d_term=200000 → MAX cap
    // 수정: Enable 시점의 현재 오차로 prev_error 초기화 → 첫 D항 = (error-error)/dt = 0
    state.current_angle = EncoderReader_GetAngleDeg();
    pid_state.prev_error = state.target_angle - state.current_angle;
    pid_state.integral = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();
    state.error = state.target_angle - state.current_angle;

    if (command_lifecycle.state != CMD_ACTIVE && fabsf(state.error) > POSITION_TOLERANCE) {
        PositionControl_CommandStart(
            (pending_command_source != CMD_SRC_NONE) ? pending_command_source : CMD_SRC_LOCALTEST
        );
    }
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Enabled (FLT cleared, angle=%.2f)\r\n", state.current_angle);
#endif
    PositionControl_UpdateDebugVars();
    return POS_CTRL_OK;
}

void PositionControl_Disable(void) {
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
    PulseControl_Stop();
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Disabled\n");
#endif
    PositionControl_UpdateDebugVars();
}

void PositionControl_Reset(void) {
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, CMD_RESULT_DISABLED, HAL_GetTick());
    }
    pid_state.integral = 0.0f;
    pid_state.prev_error = 0.0f;
    state.target_angle = 0.0f;
    state.output = 0.0f;
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] Reset\n");
#endif
    PositionControl_UpdateDebugVars();
}

// ========== 안전 기능 ==========

void PositionControl_SetSafetyLimits(SafetyLimits_t* limits) {
    // 현재는 미구현
    (void)limits;
}

bool PositionControl_CheckSafety(void) {
    // 각도 범위 체크 (하드웨어 물리 한계)
    if (state.current_angle > MAX_ANGLE_DEG + 5.0f ||
        state.current_angle < MIN_ANGLE_DEG - 5.0f) {
        fault_flag = 1;
        state.last_error = POS_CTRL_ERR_OVER_LIMIT;
        return false;
    }

    // 다회전 운용(기어비 적용) 기준 추종 오차 한계
    // 참고: 이 체크는 state.error 계산 후에 호출해야 의미 있음
    if (fabsf(state.error) > MAX_TRACKING_ERROR_DEG) {
        fault_flag = 2;
        state.last_error = POS_CTRL_ERR_SAFETY;
        return false;
    }

    state.last_error = POS_CTRL_OK;
    return true;
}
bool PositionControl_IsSafe(void) {
    return PositionControl_CheckSafety();
}

void PositionControl_EmergencyStop(void) {
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
    // [수정] fault 상세 원인 출력 (fault_flag: 1=각도범위초과, 2=오차초과)
#if LATENCY_LOG_ENABLE
    printf("[PosCtrl] EMERGENCY STOP! FLT=%d Ang:%.1f Err:%.1f\r\n",
           (int)fault_flag, state.current_angle, state.error);
#endif
    PositionControl_UpdateDebugVars();
    // 현재는 소프트 정지 + EMG 릴레이 정지를 함께 수행.
    // 재활성화 시에는 상위 모드 전이에서 Relay_EmergencyRelease() 이후
    // PositionControl_Enable()이 호출되어 제어를 재개한다.
}

void PositionControl_AbortCommand(CommandResult_t reason)
{
    if (command_lifecycle.state == CMD_ACTIVE) {
        PositionControl_CommandFinish(CMD_ABORTED, reason, HAL_GetTick());
    }
}
// ========== 성능 모니터링 ==========
PosCtrl_Stats_t PositionControl_GetStats(void) {
    return stats;
}
void PositionControl_ResetStats(void) {
    // 현재 미구현
}
// ========== 콜백 함수 등록 ==========
void PositionControl_RegisterErrorCallback(PosCtrl_ErrorCallback_t callback) {
    // 현재 미구현
    (void)callback;
}
void PositionControl_RegisterStableCallback(PosCtrl_StableCallback_t callback) {
    // 현재 미구현
    (void)callback;
}


// ========== 디버깅 ==========
void PositionControl_SetDebugLevel(DebugLevel_t level) {
    // 현재 미구현
    (void)level;
}
const char* PositionControl_GetErrorString(PosCtrl_Error_t error) {
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
        default:
            return "Unknown Error";
    }
}
       

void PositionControl_PrintStatus(void) {
    printf("[PosCtrl] EN:%d FLT:%d CMD:%lu/%s/%s Target:%.2f Current:%.2f Error:%.2f Out:%.0f %s\r\n",
           (int)control_enabled,
           (int)fault_flag,
           (unsigned long)command_lifecycle.command_id,
           PositionControl_CommandStateString(command_lifecycle.state),
           PositionControl_CommandResultString(command_lifecycle.result),
           state.target_angle,
           state.current_angle,
           state.error,
           state.output,
           state.is_stable ? "STABLE" : "");
}
