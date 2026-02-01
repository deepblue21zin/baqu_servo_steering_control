#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include "main.h"
#include <stdint.h>   // uint32_t, int32_t 등
#include <stdbool.h>  // bool, true, false

#include "encoder_reader.h"
#include "pulse_control.h"
#include "relay_control.h"
#include "homing.h"

// ========== 상수 정의 ==========//
#define CONTROL_PERIOD_MS       1       // 제어 주기 1ms
#define MAX_ANGLE_DEG          45.0f    // 최대 조향 각도
#define MIN_ANGLE_DEG         -45.0f    // 최소 조향 각도
#define POSITION_TOLERANCE     0.5f     // 위치 허용 오차 (도)

//PID 기본값
#define DEFAULT_KP             500.0f
#define DEFAULT_KI             5.0f
#define DEFAULT_KD             20.0f
#define DEFAULT_INTEGRAL_LIMIT 1000.0f
#define DEFAULT_OUTPUT_LIMIT   10000.0f  //최대 출력 (펄스 주파수) PWM 듀티 사이클%

//안정화 판정
#define STABLE_ERROR_THRESHOLD  0.5f   //오차 허용 범위 (도)
#define STABLE_TIME_MS        100    //안정화 판정 시간 (ms)

// ========== 에러 코드 ==========
typedef enum {
    POS_CTRL_OK = 0,
    POS_CTRL_ERR_NOT_INIT = -1,    // 초기화 안됨
    POS_CTRL_ERR_DISABLED = -2,    // 제어 비활성화
    POS_CTRL_ERR_OVER_LIMIT = -3,  // 각도 범위 초과
    POS_CTRL_ERR_ENCODER = -4,     // 엔코더 이상
    POS_CTRL_ERR_TIMEOUT = -5,     // 제어 시간 초과
    POS_CTRL_ERR_SAFETY = -6       // 안전 검사 실패
} PosCtrl_Error_t;

// ========== 디버그 설정 ==========
typedef enum {
    DEBUG_NONE = 0,      // 디버그 출력 없음
    DEBUG_ERROR = 1,     // 에러만 출력
    DEBUG_WARNING = 2,   // 경고 포함
    DEBUG_INFO = 3,      // 정보 포함
    DEBUG_VERBOSE = 4    // 상세 정보 포함
} DebugLevel_t;

// ========== 제어 모드 ==========
typedef enum {
    CTRL_MODE_IDLE = 0,        // 대기 모드 (모터 OFF)
    CTRL_MODE_POSITION = 1,    // 위치 제어 모드
    CTRL_MODE_MANUAL = 2,      // 수동 제어 모드
    CTRL_MODE_EMERGENCY = 99   // 비상정지
} ControlMode_t;

// ========== 데이터 구조 ==========
typedef struct {
    float Kp;                   // 비례 게인
    float Ki;                   // 적분 게인
    float Kd;                   // 미분 게인
    float integral_limit;       // 적분 와인드업 방지
    float output_limit;         // 출력 제한
} PID_Params_t;

typedef struct {
    float target_angle;           // 목표 각도
    float current_angle;          // 현재 각도
    float error;                  // 오차
    float output;                 // 제어 출력
    bool is_stable;               // 안정화 여부
    uint32_t stable_time_ms;      // 안정 유지 시간
    ControlMode_t mode;           // 현재 모드
    PosCtrl_Error_t last_error;   // 마지막 에러 코드
} PositionControl_State_t;


// ========== 성능 모니터링 ==========
typedef struct {
    float max_error;                // 최대 오차
    float avg_error;                // 평균 오차
    uint32_t update_count;          // 업데이트 횟수
    uint32_t overshoot_count;       // 오버슈트 횟수
    uint32_t max_settle_time_ms;    // 최대 안정화 시간
} PosCtrl_Stats_t;

// ========== 안전 한계값 ==========
typedef struct {
    float max_error_allowed;        // 허용 최대 오차 (도)
    float max_velocity;             // 최대 각속도 (도/초)
    uint32_t watchdog_timeout_ms;   // 워치독 타임아웃 (ms)
} SafetyLimits_t;

// ========== 콜백 함수 타입 ==========
typedef void (*PosCtrl_ErrorCallback_t)(PosCtrl_Error_t error);
typedef void (*PosCtrl_StableCallback_t)(void);

// ========================================
// 함수 선언
// ========================================

// ========== 초기화 ==========
int PositionControl_Init(void);

// ========== 메인 제어 루프 (1ms 타이머 인터럽트에서 호출) ==========
void PositionControl_Update(void);

// ========== 목표 설정 (외부에서 호출) ==========
int PositionControl_SetTarget(float target_deg);
float PositionControl_GetTarget(void);

// ========== 상태 읽기 ==========
PositionControl_State_t PositionControl_GetState(void);
float PositionControl_GetCurrentAngle(void);
float PositionControl_GetError(void);
bool PositionControl_IsStable(void);

// ========== PID 게인 설정 (튜닝용) ==========
void PositionControl_SetPID(float Kp, float Ki, float Kd);
void PositionControl_GetPID(PID_Params_t* params);

// ========== 제어 모드 ==========
void PositionControl_SetMode(ControlMode_t mode);
ControlMode_t PositionControl_GetMode(void);
int PositionControl_Enable(void);
void PositionControl_Disable(void);
void PositionControl_Reset(void);  // PID 적분 리셋

// ========== 안전 기능 ==========
void PositionControl_SetSafetyLimits(SafetyLimits_t* limits);
bool PositionControl_IsSafe(void);
bool PositionControl_CheckSafety(void);
void PositionControl_EmergencyStop(void);

// ========== 성능 모니터링 ==========
PosCtrl_Stats_t PositionControl_GetStats(void);
void PositionControl_ResetStats(void);

// ========== 콜백 등록 ==========
void PositionControl_RegisterErrorCallback(PosCtrl_ErrorCallback_t callback);
void PositionControl_RegisterStableCallback(PosCtrl_StableCallback_t callback);

// ========== 디버깅 ==========
void PositionControl_SetDebugLevel(DebugLevel_t level);
void PositionControl_PrintStatus(void);
const char* PositionControl_GetErrorString(PosCtrl_Error_t error);

#endif