/*
 * pulse_control.c
 *
 * Created on: 2026.01.19.
 * Author: 고진성 (Modified by Baqu for PID Support)
 * * [Hardware Pin Map]
 * Pulse Output : PE9  (TIM1_CH1) -> SN75176 -> L7(Pin 9, PF+)
 * Dir Output   : PE11 (GPIO_OUT) -> SN75176 -> L7(Pin 11, PR+)
 * * [Logic]
 * 1. Step Mode: TIM1 PWM Interrupt (정해진 거리 이동)
 * 2. Speed Mode: TIM1 Frequency Change (PID 제어용 연속 이동)
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
  * @brief 정방향 펄스 전송 (main.c 호환용 - 테스트용)
  */
void pulse_forward(uint32_t count) {
    PulseControl_SendSteps(count, DIR_CW);
}

/**
  * @brief 역방향 펄스 전송 (main.c 호환용 - 테스트용)
  */
void pulse_reverse(uint32_t count) {
    PulseControl_SendSteps(count, DIR_CCW);
}

/**
 * @brief PID 제어용 주파수 및 방향 설정 함수 (수정됨)
 * @param freq_hz : PID 계산 결과값 (양수: 정방향, 음수: 역방향, 크기: 속도)
 * * [수정 사항 설명]
 * 기존 코드에는 방향 제어(DIR_PIN) 로직이 빠져 있어, PID가 역방향 명령을 내려도
 * 모터가 한쪽으로만 도는 문제가 있었습니다. 이를 해결하기 위해 부호(+, -)에 따라
 * DIR 핀을 High/Low로 바꿔주는 로직을 추가했습니다.
 */
void PulseControl_SetFrequency(int32_t freq_hz) {
    // 1. 정지 신호 처리
    if (freq_hz == 0) {
        // 인터럽트 방식이 아닌 일반 Stop 사용 (PID 제어는 연속적이므로)
        HAL_TIM_PWM_Stop(p_htim1, TIM_CHANNEL_1); 
        return;
    }

    // 2. 방향 제어 (가장 중요!)
    // freq_hz가 양수면 CW, 음수면 CCW (하드웨어 연결에 따라 반대일 수 있음)
    if (freq_hz > 0) {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);   // 정방향
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET); // 역방향
        freq_hz = -freq_hz; // 주파수 계산을 위해 양수로 변환
    }

    // 3. 최고 속도 제한 (안전장치, 예: 100kHz)
    if (freq_hz > 100000) freq_hz = 100000;
    // 너무 느린 속도 방지 (타이머 분주비 한계 고려)
    if (freq_hz < 10) freq_hz = 10;

    // 4. 주파수(ARR) 계산
    // 공식: TimerClock / ((PSC+1) * TargetFreq) - 1
    // MCU의 TIM1 클럭이 180MHz이고 PSC가 215라고 가정 (CubeMX 설정 확인 필)
    uint32_t timer_clk = 180000000; 
    uint32_t psc = p_htim1->Instance->PSC;
    
    // 0으로 나누기 방지
    uint32_t arr = (timer_clk / ((psc + 1) * freq_hz)) - 1;

    // 5. 레지스터 업데이트 (주파수 및 듀티비 50% 설정)
    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);

    // 6. PWM 시작
    // PID 제어 중에는 인터럽트(_IT)를 쓰지 않고 계속 출력만 내보냅니다.
    HAL_TIM_PWM_Start(p_htim1, TIM_CHANNEL_1);
}

/**
  * @brief 펄스 및 방향 신호 전송 시작 (스텝 모드)
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
    HAL_TIM_PWM_Stop(p_htim1, TIM_CHANNEL_1); // IT가 아닌 것도 확실히 끔
    remaining_steps = 0;
    is_busy = 0;
}

/**
  * @brief 상태 확인
  */
uint8_t PulseControl_IsBusy(void) {
    return is_busy;
}