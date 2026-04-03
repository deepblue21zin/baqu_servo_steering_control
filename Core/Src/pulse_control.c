/*
 * pulse_control.c
 *
 * Pulse Output : PE9  (TIM1_CH1) -> pulse line driver input -> PF+/PF-
 * Dir Output   : PE10 (GPIO_OUT) -> direction line driver input -> PR+/PR-
 */

#include "pulse_control.h"
#include "tim.h"

typedef enum {
    PULSE_REVERSE_IDLE = 0U,
    PULSE_REVERSE_WAIT_STOP = 1U,
    PULSE_REVERSE_WAIT_DIR_SETTLE = 2U
} PulseReverseState_t;

static TIM_HandleTypeDef *p_htim1;
static volatile uint32_t remaining_steps = 0U;
static volatile uint8_t is_busy = 0U;
static volatile uint8_t line_drivers_enabled = 0U;
static volatile uint8_t output_active = 0U;
static volatile int32_t requested_frequency_hz = 0;
static volatile uint32_t applied_frequency_hz = 0U;
static volatile MotorDirection current_direction = DIR_CCW;
static volatile MotorDirection pending_direction = DIR_CCW;
static volatile uint32_t pending_frequency_hz = 0U;
static volatile uint32_t reverse_guard_deadline_ms = 0U;
static volatile PulseReverseState_t reverse_state = PULSE_REVERSE_IDLE;

extern TIM_HandleTypeDef htim1;

static uint32_t PulseControl_GetTimerClockHz(void)
{
    RCC_ClkInitTypeDef clk_init = {0};
    uint32_t flash_latency = 0U;
    uint32_t apb2_clock_hz = HAL_RCC_GetPCLK2Freq();

    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);
    if (clk_init.APB2CLKDivider == RCC_HCLK_DIV1) {
        return apb2_clock_hz;
    }

    return apb2_clock_hz * 2U;
}

static uint32_t PulseControl_ClampFrequencyHz(uint32_t freq_hz)
{
    if (freq_hz < PULSECONTROL_MIN_FREQ_HZ) {
        return PULSECONTROL_MIN_FREQ_HZ;
    }
    if (freq_hz > PULSECONTROL_MAX_FREQ_HZ) {
        return PULSECONTROL_MAX_FREQ_HZ;
    }
    return freq_hz;
}

static uint8_t PulseControl_DeadlineExpired(uint32_t deadline_ms)
{
    return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0) ? 1U : 0U;
}

static void PulseControl_EnableSharedLineDrivers(void)
{
    if (line_drivers_enabled != 0U) {
        return;
    }

    HAL_GPIO_WritePin(LINE_DRIVER_DE_GPIO_Port, LINE_DRIVER_DE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LINE_DRIVER_REN_GPIO_Port, LINE_DRIVER_REN_Pin, GPIO_PIN_SET);
    line_drivers_enabled = 1U;
}

static void PulseControl_ApplyDirection(MotorDirection dir)
{
    GPIO_PinState pin_state = GPIO_PIN_RESET;

    if (dir == DIR_CW) {
        pin_state = (DIR_ACTIVE_HIGH_FOR_CW != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else {
        pin_state = (DIR_ACTIVE_HIGH_FOR_CW != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(PR_TX_GPIO_Port, PR_TX_Pin, pin_state);
    current_direction = dir;
}

static uint32_t PulseControl_CalculateAppliedFrequencyHz(uint32_t period_counts)
{
    uint32_t timer_clock_hz = PulseControl_GetTimerClockHz();
    uint32_t prescaler = p_htim1->Instance->PSC + 1U;
    uint64_t denominator = (uint64_t)prescaler * (uint64_t)period_counts;

    if (denominator == 0U) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)timer_clock_hz + (denominator / 2U)) / denominator);
}

static void PulseControl_StopOutputInternal(void)
{
    __HAL_TIM_DISABLE_IT(p_htim1, TIM_IT_CC1);
    HAL_TIM_PWM_Stop(p_htim1, TIM_CHANNEL_1);
    remaining_steps = 0U;
    is_busy = 0U;
    output_active = 0U;
    applied_frequency_hz = 0U;
}

static void PulseControl_ApplyPwmFrequency(uint32_t freq_hz)
{
    uint32_t timer_clock_hz = PulseControl_GetTimerClockHz();
    uint32_t prescaler = p_htim1->Instance->PSC + 1U;
    uint64_t denominator = (uint64_t)prescaler * (uint64_t)freq_hz;
    uint64_t period_counts = 0U;
    uint32_t autoreload = 0U;
    uint32_t compare = 0U;

    if (denominator == 0U) {
        return;
    }

    period_counts = ((uint64_t)timer_clock_hz + (denominator / 2U)) / denominator;
    if (period_counts < 2U) {
        period_counts = 2U;
    }
    if (period_counts > 65536U) {
        period_counts = 65536U;
    }

    autoreload = (uint32_t)(period_counts - 1U);
    compare = (uint32_t)(period_counts / 2U);
    if (compare == 0U) {
        compare = 1U;
    }
    if (compare > autoreload) {
        compare = autoreload;
    }

    __HAL_TIM_SET_AUTORELOAD(p_htim1, autoreload);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, compare);
    applied_frequency_hz = PulseControl_CalculateAppliedFrequencyHz((uint32_t)period_counts);
}

static void PulseControl_StartContinuousOutput(uint32_t freq_hz)
{
    PulseControl_ApplyPwmFrequency(freq_hz);

    if (output_active == 0U) {
        if (HAL_TIM_PWM_Start(p_htim1, TIM_CHANNEL_1) == HAL_OK) {
            output_active = 1U;
        } else {
            applied_frequency_hz = 0U;
        }
    }
}

static void PulseControl_BeginReverseGuard(MotorDirection dir, uint32_t freq_hz)
{
    pending_direction = dir;
    pending_frequency_hz = freq_hz;
    PulseControl_StopOutputInternal();
    reverse_guard_deadline_ms = HAL_GetTick() + PULSECONTROL_DIRECTION_GUARD_MS;
    reverse_state = PULSE_REVERSE_WAIT_STOP;
}

static void PulseControl_ServiceReverseGuard(void)
{
    if (reverse_state == PULSE_REVERSE_WAIT_STOP) {
        if (PulseControl_DeadlineExpired(reverse_guard_deadline_ms) == 0U) {
            return;
        }

        PulseControl_ApplyDirection(pending_direction);
        reverse_guard_deadline_ms = HAL_GetTick() + PULSECONTROL_DIRECTION_GUARD_MS;
        reverse_state = PULSE_REVERSE_WAIT_DIR_SETTLE;
        return;
    }

    if (reverse_state == PULSE_REVERSE_WAIT_DIR_SETTLE) {
        if (PulseControl_DeadlineExpired(reverse_guard_deadline_ms) == 0U) {
            return;
        }

        reverse_state = PULSE_REVERSE_IDLE;
        if (pending_frequency_hz > 0U) {
            PulseControl_StartContinuousOutput(pending_frequency_hz);
        }
    }
}

void PulseControl_Init(void)
{
    p_htim1 = &htim1;
    remaining_steps = 0U;
    is_busy = 0U;
    line_drivers_enabled = 0U;
    output_active = 0U;
    requested_frequency_hz = 0;
    applied_frequency_hz = 0U;
    current_direction = DIR_CW;
    pending_direction = DIR_CCW;
    pending_frequency_hz = 0U;
    reverse_guard_deadline_ms = 0U;
    reverse_state = PULSE_REVERSE_IDLE;

    PulseControl_EnableSharedLineDrivers();
    PulseControl_ApplyDirection(DIR_CCW);
}

void pulse_forward(uint32_t count)
{
    PulseControl_SendSteps(count, DIR_CW);
}

void pulse_reverse(uint32_t count)
{
    PulseControl_SendSteps(count, DIR_CCW);
}

void PulseControl_SetFrequency(int32_t freq_hz)
{
    MotorDirection target_direction = DIR_CCW;
    uint32_t target_frequency_hz = 0U;

    PulseControl_EnableSharedLineDrivers();
    requested_frequency_hz = freq_hz;
    PulseControl_ServiceReverseGuard();

    if (freq_hz == 0) {
        pending_frequency_hz = 0U;
        reverse_state = PULSE_REVERSE_IDLE;
        PulseControl_StopOutputInternal();
        return;
    }

    if (freq_hz > 0) {
        target_direction = DIR_CW;
        target_frequency_hz = PulseControl_ClampFrequencyHz((uint32_t)freq_hz);
    } else {
        target_direction = DIR_CCW;
        target_frequency_hz = PulseControl_ClampFrequencyHz((uint32_t)(-freq_hz));
    }

    if (reverse_state != PULSE_REVERSE_IDLE) {
        pending_direction = target_direction;
        pending_frequency_hz = target_frequency_hz;
        return;
    }

    if (target_direction != current_direction) {
        PulseControl_BeginReverseGuard(target_direction, target_frequency_hz);
        return;
    }

    PulseControl_StartContinuousOutput(target_frequency_hz);
}

void PulseControl_SendSteps(uint32_t steps, MotorDirection dir)
{
    if ((steps == 0U) || (is_busy != 0U)) {
        return;
    }

    PulseControl_EnableSharedLineDrivers();
    requested_frequency_hz = 0;
    pending_frequency_hz = 0U;
    reverse_state = PULSE_REVERSE_IDLE;
    PulseControl_StopOutputInternal();

    is_busy = 1U;
    remaining_steps = steps;
    PulseControl_ApplyDirection(dir);

    if (HAL_TIM_PWM_Start_IT(p_htim1, TIM_CHANNEL_1) == HAL_OK) {
        uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(p_htim1) + 1U;

        output_active = 1U;
        applied_frequency_hz = PulseControl_CalculateAppliedFrequencyHz(period_counts);
    } else {
        remaining_steps = 0U;
        is_busy = 0U;
        applied_frequency_hz = 0U;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) {
        return;
    }

    if (remaining_steps > 0U) {
        remaining_steps--;
    }

    if (remaining_steps == 0U) {
        PulseControl_StopOutputInternal();
    }
}

void PulseControl_Stop(void)
{
    requested_frequency_hz = 0;
    pending_frequency_hz = 0U;
    reverse_state = PULSE_REVERSE_IDLE;
    PulseControl_StopOutputInternal();
}

uint8_t PulseControl_IsBusy(void)
{
    return is_busy;
}

PulseControl_Status_t PulseControl_GetStatus(void)
{
    PulseControl_Status_t status;

    status.requested_frequency_hz = requested_frequency_hz;
    status.applied_frequency_hz = applied_frequency_hz;
    status.autoreload = __HAL_TIM_GET_AUTORELOAD(p_htim1);
    status.compare = __HAL_TIM_GET_COMPARE(p_htim1, TIM_CHANNEL_1);
    status.direction = current_direction;
    status.output_active = output_active;
    status.line_driver_enabled = line_drivers_enabled;
    status.reverse_guard_active = (reverse_state != PULSE_REVERSE_IDLE) ? 1U : 0U;
    status.busy = is_busy;

    return status;
}
