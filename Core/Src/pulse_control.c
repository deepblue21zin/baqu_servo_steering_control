/*
 * pulse_control.c
 *
 * Created on: 2026.01.19.
 * Author: 고진성
 * * [Hardware Pin Map]
 * Pulse Output : PE9  (TIM1_CH1) -> SN75176 -> L7(Pin 9, PF+)
 * Dir Output   : PE11 (GPIO_OUT) -> SN75176 -> L7(Pin 11, PR+)
 * * [Logic]
 * TIM1 PWM Interrupt를 사용하여 정확한 개수의 펄스를 내보냅니다.
 */

#include "pulse_control.h"

// =========================================================
// [하드웨어 설정] 회로도 기반 확정 (PE11)
// =========================================================
#define DIR_GPIO_PORT   GPIOE
#define DIR_PIN         GPIO_PIN_11
// =========================================================

static TIM_HandleTypeDef *p_htim1;
static volatile uint32_t remaining_steps = 0;
static volatile uint8_t is_busy = 0;

extern TIM_HandleTypeDef htim1;
/**
  * @brief 펄스 제어 초기화
  */
void PulseControl_Init(void) {
	p_htim1 = &htim1;
    is_busy = 0;

    // 방향 핀 초기 상태 설정 (Safety)
    // CubeMX(main.c)에서 PE11을 GPIO_Output으로 설정했는지 꼭 확인하세요.
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);
}
/**
  * @brief 정방향 펄스 전송 (main.c 호환용)
  */
void pulse_forward(uint32_t count) {
    // DIR_CW 또는 DIR_CCW는 pulse_control.h에 정의된 enum 값을 따르세요.
    // 보통 CW를 정방향으로 사용합니다.
    PulseControl_SendSteps(count, DIR_CW);
}

/**
  * @brief 역방향 펄스 전송 (main.c 호환용)
  */
void pulse_reverse(uint32_t count) {
    PulseControl_SendSteps(count, DIR_CCW);
}

/**
  * @brief 주파수(속도) 설정 (main.c/position_control.c 호환용)
  */
void PulseControl_SetFrequency(int32_t freq_hz) {
    if (freq_hz < 0) freq_hz = -freq_hz; // 음수 처리
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
        return;
    }

    // 주파수 계산: 주파수 = 타이머클럭 / ((PSC+1) * (ARR+1))
    // 간단히 Period(ARR) 값만 변경하여 속도를 조절하는 예시입니다.
    uint32_t timer_clk = 180000000; // STM32F429 TIM1 클럭 (예시: 180MHz)
    uint32_t psc = p_htim1->Instance->PSC;
    uint32_t arr = (timer_clk / ((psc + 1) * freq_hz)) - 1;

    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2); // Duty 50%
}

/**
  * @brief 펄스 및 방향 신호 전송 시작
  */
void PulseControl_SendSteps(uint32_t steps, MotorDirection dir) {
    if (steps == 0 || is_busy) return; // 방어 코드

    is_busy = 1;
    remaining_steps = steps;

    // [방향 제어]
    // PE11 핀의 High/Low 상태로 SN75176을 통해 L7 드라이브의 방향을 결정
    if (dir == DIR_CW) {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);   // CW
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET); // CCW
    }

    // [펄스 발사]
    // TIM1 PWM 시작 및 인터럽트 활성화
    // Prescaler: 215, Period: 9 (약 100kHz 설정 가정)
    HAL_TIM_PWM_Start_IT(p_htim1, TIM_CHANNEL_1);
}

/**
  * @brief PWM 펄스 카운팅 콜백 (인터럽트 핸들러)
  * 펄스가 1개 나갈 때마다 호출되어 remaining_steps를 줄입니다.
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    // TIM1 인스턴스인지 확인
    if (htim->Instance == TIM1) {
        if (remaining_steps > 0) {
            remaining_steps--; // 남은 펄스 수 차감
        }

        if (remaining_steps == 0) {
            // 목표 펄스 도달 시 PWM 정지
            HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
            is_busy = 0;
        }
    }
}

/**
  * @brief 강제 정지
  */
void PulseControl_Stop(void) {
    HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
    remaining_steps = 0;
    is_busy = 0;
}

/**
  * @brief 상태 확인
  */
uint8_t PulseControl_IsBusy(void) {
    return is_busy;
}
