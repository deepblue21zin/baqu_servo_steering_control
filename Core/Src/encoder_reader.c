/**
 * @file encoder_reader.c
 * @brief Encoder reader implementation (stub)
 */

#include "encoder_reader.h"
#include "tim.h"
#include <stdio.h>

static int32_t encoder_count = 0;
static int32_t encoder_offset = 0;
static uint8_t initialized = 0;

// 엔코더 설정값 (XML-FBL04AMK1)
#define PULSE_PER_REV 12000      // 12000 PPR (4체배 시 48000 count/rev)
#define QUADRATURE    4          // TIM2 Encoder Mode = 4체배
#define COUNT_PER_REV (PULSE_PER_REV * QUADRATURE)  // 48000
#define DEG_PER_COUNT (360.0f / COUNT_PER_REV)      // 0.0075도/카운트

int EncoderReader_Init(void) {
    encoder_count = 0;
    encoder_offset = 0;
    initialized = 1;
    printf("[Encoder] Initialized\n");
    return 0;
}

float EncoderReader_GetAngleDeg(void) {
    // 먼저 카운터 값을 읽어서 업데이트
    encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t adjusted_count = encoder_count - encoder_offset;
    return (float)adjusted_count * DEG_PER_COUNT;
}

int32_t EncoderReader_GetCount(void) {
    // TIM2 하드웨어 카운터에서 직접 읽기
    encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    return encoder_count;
}

void EncoderReader_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // TIM2 카운터 리셋
    encoder_count = 0;
    encoder_offset = 0;
    printf("[Encoder] Reset\n");
}

void EncoderReader_SetOffset(int32_t offset) {
    encoder_offset = offset;
    printf("[Encoder] Offset set: %ld\n", offset);
}

uint8_t EncoderReader_IsInitialized(void) {
    return initialized;
}

// TODO: 타이머 인터럽트에서 호출할 함수 (실제 하드웨어 연결 시)
// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//     if (htim->Instance == TIM2) {
//         // A상 펄스 감지
//         if (HAL_GPIO_ReadPin(ENC_B_PORT, ENC_B_PIN)) {
//             encoder_count++;  // CW
//         } else {
//             encoder_count--;  // CCW
//         }
//     }
// }