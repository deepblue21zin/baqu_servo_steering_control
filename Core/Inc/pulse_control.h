/*
 * pulse_control.h
 *
 * STM32 to L7 servo drive pulse/direction generation
 *   - PE9(TIM1_CH1) -> pulse line driver input -> PF+/PF-
 *   - PE10(GPIO)    -> direction line driver input -> PR+/PR-
 */

#ifndef INC_PULSE_CONTROL_H_
#define INC_PULSE_CONTROL_H_

#include <stdint.h>

#ifndef DIR_ACTIVE_HIGH_FOR_CW
#define DIR_ACTIVE_HIGH_FOR_CW 1
#endif

typedef enum {
    DIR_CW = 0,
    DIR_CCW = 1
} MotorDirection;

typedef struct {
    int32_t requested_frequency_hz;
    uint32_t applied_frequency_hz;
    uint32_t autoreload;
    uint32_t compare;
    MotorDirection direction;
    uint8_t output_active;
    uint8_t line_driver_enabled;
    uint8_t reverse_guard_active;
    uint8_t busy;
} PulseControl_Status_t;

void PulseControl_Init(void);
void PulseControl_SendSteps(uint32_t steps, MotorDirection dir);
void PulseControl_Stop(void);
uint8_t PulseControl_IsBusy(void);
void pulse_forward(uint32_t count);
void pulse_reverse(uint32_t count);
void PulseControl_SetFrequency(int32_t freq_hz);
PulseControl_Status_t PulseControl_GetStatus(void);

#endif /* INC_PULSE_CONTROL_H_ */
