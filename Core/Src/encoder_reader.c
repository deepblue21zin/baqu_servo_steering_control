/**
 * @file encoder_reader.c
 * @brief Encoder reader implementation (stub)
 */

#include "encoder_reader.h"
#include "tim.h"
#include <stdio.h>

#define ENCODER_TIMER htim4
#define ENCODER_MAX_COUNT 65535   // 16비트

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
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, 32768);  // 중간값에서 시작 (오버플로우 여유)
    initialized = 1;
    printf("[Encoder] Initialized\n");
    return 0;
}

float EncoderReader_GetAngleDeg(void) {
    // 먼저 카운터 값을 읽어서 업데이트
    uint16_t raw = __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    encoder_count = (int32_t)raw - 32768;  // 16비트 카운터 값 읽기 (0~65535)
    int32_t adjusted_count = encoder_count - encoder_offset;
    return (float)adjusted_count * DEG_PER_COUNT;
}
//중요 : TIM4는 16비트이므로 TIM2(32비트)에서 사용하던 방식으로 카운터 값을 읽으면 오버플로우 문제가 발생할 수 있습니다. 따라서 TIM4의 카운터 값을 읽을 때는 16비트 범위 내에서 처리해야 합니다. 위 코드에서는 32768을 빼서 -32768 ~ +32767 범위로 조정한 후, 오프셋을 적용하여 각도를 계산합니다.
int32_t EncoderReader_GetCount(void) {
    // TIM4 하드웨어 카운터에서 직접 읽기
    uint16_t raw = __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    encoder_count = (int32_t)raw - 32768;
    return encoder_count;
}

void EncoderReader_Reset(void) {
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, 32768);  // TIM4 카운터 중간값으로 리셋
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