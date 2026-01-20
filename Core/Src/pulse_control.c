#include "pulse_control.h"
#include "tim.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>

/* ========== Private Variables ========== */

static uint8_t initialized = 0;
static int32_t current_freq = 0;

/* ========== Public Functions ========== */

// TODO: 실제 구현 필요
void PulseControl_Init(void) {
    initialized = 1;
    current_freq = 0;
    printf("[PulseCtrl] Init\n");
}

void PulseControl_SetFrequency(int32_t freq_hz) {
    current_freq = freq_hz;
    printf("[PulseCtrl] SetFreq: %ld Hz\n", freq_hz);
}

void PulseControl_Stop(void) {
    current_freq = 0;
    printf("[PulseCtrl] Stop\n");
}

int32_t PulseControl_GetCurrentFreq(void) {
    return current_freq;
}

uint8_t PulseControl_IsInitialized(void) {
    return initialized;
}

void pulse_forward(uint32_t count) {
    // SIGN 핀 HIGH = 정방향 (CH2)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);  // SIGN LOW (or adjust based on driver)

    // 펄스 출력 (간단한 딜레이 방식 - 테스트용)
    for (uint32_t i = 0; i < count; i++) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 5);  // Pulse HIGH
        HAL_Delay(1);  // 1ms delay (약 500Hz)
    }

    printf("[PulseCtrl] Forward %lu pulses\n", count);
}

void pulse_reverse(uint32_t count) {
    // SIGN 핀 HIGH = 역방향 (CH2)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 5);  // SIGN HIGH

    // 펄스 출력 (간단한 딜레이 방식 - 테스트용)
    for (uint32_t i = 0; i < count; i++) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 5);  // Pulse HIGH
        HAL_Delay(1);  // 1ms delay
    }

    printf("[PulseCtrl] Reverse %lu pulses\n", count);
}