/*
 * pulse_control.h
 *
 * Created on: 2026.01.19.
 * Author: 고진성
 * Function: STM32 to L7 Servo Drive Pulse Generation
 * Hardware: NUCLEO-F767ZI -> SN75176 -> L7 Servo Drive
 */

#ifndef INC_PULSE_CONTROL_H_
#define INC_PULSE_CONTROL_H_

#include "stm32f7xx_hal.h"

// 서보 모터 방향 정의
typedef enum {
    DIR_CW = 0,   // 시계 방향 (Clockwise)
    DIR_CCW = 1   // 반시계 방향 (Counter-Clockwise)
} MotorDirection;

/**
  * @brief 펄스 제어 모듈 초기화
  * @param htim TIM1 핸들러 주소
  */
void PulseControl_Init(TIM_HandleTypeDef *htim);

/**
  * @brief 특정 개수만큼의 펄스를 전송 (위치 제어 핵심 함수)
  * @param steps 전송할 펄스 개수 (Distance)
  * @param dir 회전 방향 (Direction)
  */
void PulseControl_SendSteps(uint32_t steps, MotorDirection dir);

/**
  * @brief 펄스 전송 즉시 중지 (비상 정지용)
  */
void PulseControl_Stop(void);

/**
  * @brief 현재 모터가 동작 중인지 확인
  * @return 1: 동작 중, 0: 대기 중
  */
uint8_t PulseControl_IsBusy(void);

#endif /* INC_PULSE_CONTROL_H_ */