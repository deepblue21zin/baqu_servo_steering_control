/**
 * @file position_control.c
 * @brief Position control implementation
 * @author 팀장
 */

#include "main.h"
#include "position_control.h"
#include "pulse_control.h"      // Person A
#include "relay_control.h"      // Person B
#include "encoder_reader.h"     // Person C
#include "homing.h"             // Person B
#include <stdio.h>
#include <math.h>

/* ========== Private Variables ========== */

static PosCtrlState_t state = {
    .current_angle = 0.0f,
    .target_angle = 0.0f,
    .error = 0.0f,
    .status = POS_CTRL_STATUS_IDLE
};

/* ========== Private Functions ========== */

/**
 * @brief Calculate pulses needed
 */
static uint32_t calculate_pulses(float angle_deg)
{
    // 전자기어비: P4-01/P4-05 = 524288/12000 = 43.69
    // 1회전 = 12000 펄스 (엔코더)
    // 360도 = 12000 펄스
    // 1도 = 33.33 펄스
    
    float pulses_per_degree = 12000.0f / 360.0f;
    uint32_t pulses = (uint32_t)fabsf(angle_deg * pulses_per_degree);
    
    return pulses;
}

/* ========== Public Functions ========== */

/**
 * @brief Initialize position control
 */
int PosCtrl_Init(void)
{
    printf("[PosCtrl] Initializing...\r\n");
    
    // Initialize subsystems
    Pulse_Init();
    Relay_Init();
    Encoder_Init();
    Homing_Init();
    
    // Perform homing
    printf("[PosCtrl] Performing homing...\r\n");
    Relay_ServoOn();
    HAL_Delay(100);
    
    if (Homing_FindZero() != 0) {
        printf("[PosCtrl] ERROR: Homing failed!\r\n");
        return -1;
    }
    
    // Update current position
    state.current_angle = Encoder_GetAngle();
    state.target_angle = state.current_angle;
    state.error = 0.0f;
    state.status = POS_CTRL_STATUS_IDLE;
    
    printf("[PosCtrl] Initialized at %.2f deg\r\n", state.current_angle);
    
    return 0;
}

/**
 * @brief Move to target angle
 */
int PosCtrl_MoveTo(float target_angle)
{
    // Limit check
    if (target_angle > POSITION_CONTROL_MAX_ANGLE || 
        target_angle < POSITION_CONTROL_MIN_ANGLE) {
        printf("[PosCtrl] ERROR: Target out of range: %.2f deg\r\n", target_angle);
        return -1;
    }
    
    state.target_angle = target_angle;
    state.status = POS_CTRL_STATUS_MOVING;
    
    printf("[PosCtrl] Moving to %.2f deg\r\n", target_angle);
    
    return 0;
}

/**
 * @brief Move relative
 */
int PosCtrl_MoveRelative(float delta_angle)
{
    float new_target = state.current_angle + delta_angle;
    return PosCtrl_MoveTo(new_target);
}

/**
 * @brief Update control loop
 */
int PosCtrl_Update(void)
{
    // Update current position
    state.current_angle = Encoder_GetAngle();
    state.error = state.target_angle - state.current_angle;
    
    // Check if idle
    if (state.status == POS_CTRL_STATUS_IDLE) {
        return 0;
    }
    
    // Check if reached
    if (fabsf(state.error) < POSITION_CONTROL_TOLERANCE) {
        state.status = POS_CTRL_STATUS_REACHED;
        printf("[PosCtrl] Target reached! Current: %.2f deg\r\n", state.current_angle);
        return 0;
    }
    
    // Calculate pulses
    uint32_t pulses = calculate_pulses(state.error);
    uint8_t direction = (state.error > 0) ? 1 : 0;  // 1=forward, 0=reverse
    
    // Generate pulses
    printf("[PosCtrl] Generating %lu pulses, dir=%d\r\n", pulses, direction);
    Pulse_Generate(pulses, direction);
    
    // Update position
    state.current_angle = Encoder_GetAngle();
    state.error = state.target_angle - state.current_angle;
    
    printf("[PosCtrl] Current: %.2f, Error: %.2f deg\r\n", 
           state.current_angle, state.error);
    
    return 0;
}

/**
 * @brief Stop movement
 */
void PosCtrl_Stop(void)
{
    Pulse_Stop();
    state.status = POS_CTRL_STATUS_IDLE;
    state.target_angle = state.current_angle;
    state.error = 0.0f;
    
    printf("[PosCtrl] Stopped at %.2f deg\r\n", state.current_angle);
}

/**
 * @brief Get current state
 */
const PosCtrlState_t* PosCtrl_GetState(void)
{
    return &state;
}

/**
 * @brief Check if target reached
 */
uint8_t PosCtrl_IsTargetReached(void)
{
    return (state.status == POS_CTRL_STATUS_REACHED) ? 1 : 0;
}

/**
 * @brief Get current position
 */
float PosCtrl_GetPosition(void)
{
    return state.current_angle;
}

/**
 * @brief Get position error
 */
float PosCtrl_GetError(void)
{
    return state.error;
}