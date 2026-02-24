#include "position_control.h"
#include "encoder_reader.h"      // 엔코더 읽기
#include "pulse_control.h"       // 펄스 출력
#include "relay_control.h"       // 릴레이 제어 (EmergencyStop에서 사용)
#include <stdio.h>
#include <stdint.h>              // uint32_t, int32_t
#include <math.h>                // fabsf
#include "main.h"

//의존성

// 내부 변수 지정
static volatile uint8_t debug_enabled = 0; // 디버그 메시지 출력 여부
static volatile uint8_t fault_flag = 0; // 안전 관련 플래그

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
static PosCtrl_Stats_t stats = {0};

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

    printf("[PosCtrl] Initialized\n");
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
    if (!control_enabled) {
        PulseControl_Stop();
        return;
    }
    
    // 1. 현재 각도 읽기
    state.current_angle = EncoderReader_GetAngleDeg();

    // 2. 오차 계산 (안전 체크보다 먼저!)
    // [BUG FIX] 기존 코드: 안전 체크가 오차 계산보다 앞에 있어서
    // CheckSafety()가 이전 루프의 state.error를 참조하는 1-step 지연 버그 존재.
    // 수정: 오차를 먼저 계산하고 안전 체크 수행.
    state.error = state.target_angle - state.current_angle;

    // 3. 안전 체크 (현재 오차 기준으로 판단)
    if (!PositionControl_CheckSafety()) {
        PositionControl_EmergencyStop();
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
    
    // 5. PID 계산
    state.output = PID_Calculate(state.error, dt);
    
    // 6. 펄스 출력
    PulseControl_SetFrequency((int32_t)state.output);
    
    // 7. 안정화 판단
    if (fabsf(state.error) < POSITION_TOLERANCE) {
        state.stable_time_ms += (uint32_t)(dt * 1000.0f); // 안정 유지 시간 누적
        if (state.stable_time_ms > 100) {  // 100ms 이상 안정
            state.is_stable = true;
        }
    } else {
        state.stable_time_ms = 0;
        state.is_stable = false;
    }
}

// ========== 목표 설정 ==========
int PositionControl_SetTarget(float target_deg) {
    // 범위 체크
    if (target_deg > MAX_ANGLE_DEG || target_deg < MIN_ANGLE_DEG) {
        return POS_CTRL_ERR_OVER_LIMIT;
    }

    __disable_irq();
    state.target_angle = target_deg;
    state.is_stable = false;
    state.stable_time_ms = 0;
    __enable_irq();

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
    
    printf("[PosCtrl] PID updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
           Kp, Ki, Kd);
}

// ========== 제어 모드 ==========
void PositionControl_SetMode(ControlMode_t mode) {
    // 현재는 단일 모드만 지원
    (void)mode; // 미사용 변수 경고 방지
}
ControlMode_t PositionControl_GetMode(void) {
    return CTRL_MODE_POSITION;
}

int PositionControl_Enable(void) {
    control_enabled = true;
    fault_flag = 0;              // EmergencyStop 후 재활성화 시 fault 초기화

    // [BUG FIX] D항 킥 방지 (Derivative Kick Prevention)
    // 기존: prev_error=0으로 초기화 → 첫 호출 시 D=(error-0)/0.001=10000 → d_term=200000 → MAX cap
    // 수정: Enable 시점의 현재 오차로 prev_error 초기화 → 첫 D항 = (error-error)/dt = 0
    state.current_angle = EncoderReader_GetAngleDeg();
    pid_state.prev_error = state.target_angle - state.current_angle;
    pid_state.integral = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();
    printf("[PosCtrl] Enabled (FLT cleared, angle=%.2f)\r\n", state.current_angle);
    return POS_CTRL_OK;
}

void PositionControl_Disable(void) {
    control_enabled = false;
    PulseControl_Stop();
    printf("[PosCtrl] Disabled\n");
}

void PositionControl_Reset(void) {
    pid_state.integral = 0.0f;
    pid_state.prev_error = 0.0f;
    state.target_angle = 0.0f;
    printf("[PosCtrl] Reset\n");
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
        return false;
    }

    // [수정] 오차 임계값: 60° → 150°로 완화 (테스트 시 PID 오버슈트/진동 허용)
    // 참고: 이 체크는 state.error 계산 후에 호출해야 의미 있음
    if (fabsf(state.error) > 150.0f) {
        fault_flag = 2;
        return false;
    }

    return true;
}
bool PositionControl_IsSafe(void) {
    return PositionControl_CheckSafety();
}

void PositionControl_EmergencyStop(void) {
    control_enabled = false;
    PulseControl_Stop();
    pid_state.integral = 0.0f;
    // [수정] fault 상세 원인 출력 (fault_flag: 1=각도범위초과, 2=오차초과)
    printf("[PosCtrl] EMERGENCY STOP! FLT=%d Ang:%.1f Err:%.1f\r\n",
           (int)fault_flag, state.current_angle, state.error);
    // EMG 릴레이 24V 미연결 상태이므로 Relay_Emergency() 호출 안 함.
    // P2-02 EMG 기능 비활성화 상태이므로 드라이브에도 영향 없음.
    // 재활성화: PositionControl_Enable() 재호출로 복구 가능.
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
    printf("[PosCtrl] EN:%d FLT:%d Target:%.2f Current:%.2f Error:%.2f Out:%.0f %s\r\n",
           (int)control_enabled,
           (int)fault_flag,
           state.target_angle,
           state.current_angle,
           state.error,
           state.output,
           state.is_stable ? "STABLE" : "");
}
