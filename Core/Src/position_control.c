#include "position_control.h"
#include "encoder_reader.h"      // 엔코더 읽기
#include "pulse_control.h"       // 펄스 출력
#include <stdio.h>
#include <stdint.h>              // uint32_t, int32_t
#include <stdbool.h>             // bool
#include <math.h>                // fabsf
//의존성

// 내부 변수 지정
static PID_Params_t pid_params = {
    .Kp = 500.0f,           // 초기값 (실험으로 튜닝 필요)
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

static bool control_enabled = false;

// ========== 초기화 ==========
int PositionControl_Init(void) {
    pid_state.prev_error = 0.0f;
    pid_state.integral = 0.0f;
    pid_state.last_time_ms = HAL_GetTick();
    
    state.target_angle = 0.0f;
    state.current_angle = 0.0f;
    control_enabled = false;

    printf("[PosCtrl] Initialized\n");
    return POS_CTRL_OK;  // 성공
}

// ========== PID 계산 ==========
static float PID_Calculate(float error, float dt) {
    // P항
    float p_term = pid_params.Kp * error;
    
    // I항 (적분 와인드업 방지)
    pid_state.integral += error * dt;
    
    // 적분 제한
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
    
    // 2. 안전 체크
    if (!PositionControl_CheckSafety()) {
        PositionControl_EmergencyStop();
        return;
    }
    
    // 3. 오차 계산
    state.error = state.target_angle - state.current_angle;
    
    // 4. 시간 계산
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid_state.last_time_ms) / 1000.0f;  // ms → s
    pid_state.last_time_ms = current_time;
    
    // 5. PID 계산
    state.output = PID_Calculate(state.error, dt);
    
    // 6. 펄스 출력
    PulseControl_SetFrequency((int32_t)state.output);
    
    // 7. 안정화 판단
    if (fabsf(state.error) < POSITION_TOLERANCE) {
        state.stable_time_ms++;
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
        printf("[PosCtrl] ERROR: Target out of range: %.2f\n", target_deg);
        return POS_CTRL_ERR_OVER_LIMIT;  // 범위 초과 에러
    }

    state.target_angle = target_deg;
    state.is_stable = false;
    state.stable_time_ms = 0;

    printf("[PosCtrl] Target set: %.2f deg\n", target_deg);
    return POS_CTRL_OK;  // 성공
}

// ========== 상태 읽기 ==========
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
int PositionControl_Enable(void) {
    control_enabled = true;
    pid_state.integral = 0.0f;  // 적분 리셋
    printf("[PosCtrl] Enabled\n");
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
bool PositionControl_CheckSafety(void) {
    // 각도 범위 체크
    if (state.current_angle > MAX_ANGLE_DEG + 5.0f || 
        state.current_angle < MIN_ANGLE_DEG - 5.0f) {
        printf("[PosCtrl] ERROR: Angle out of range! %.2f\n", 
               state.current_angle);
        return false;
    }
    
    // 오차가 너무 큰 경우
    if (fabsf(state.error) > 60.0f) {
        printf("[PosCtrl] ERROR: Error too large! %.2f\n", 
               state.error);
        return false;
    }
    
    return true;
}

void PositionControl_EmergencyStop(void) {
    control_enabled = false;
    PulseControl_Stop();
    pid_state.integral = 0.0f;
    printf("[PosCtrl] EMERGENCY STOP!\n");
}

// ========== 디버깅 ==========
void PositionControl_PrintStatus(void) {
    printf("[PosCtrl] Target:%.2f Current:%.2f Error:%.2f Out:%.0f %s\n",
           state.target_angle,
           state.current_angle,
           state.error,
           state.output,
           state.is_stable ? "STABLE" : "");
}
