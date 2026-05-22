/*
 * pulse_control.c
 *
 * Pulse Output : PE9  (TIM1_CH1) -> pulse line driver input -> PF+/PF-
 * Dir Output   : PE10 (GPIO_OUT) -> direction line driver input -> PR+/PR-
 */

#include "pulse_control.h"

#include "constants.h"
#include "tim.h"

typedef enum {
    PULSE_REVERSE_IDLE = 0U,
    PULSE_REVERSE_WAIT_STOP = 1U,
    PULSE_REVERSE_WAIT_DIR_SETTLE = 2U
} PulseReverseState_t;

static TIM_HandleTypeDef *p_htim1;
static volatile uint8_t initialized = 0U;
static volatile uint8_t line_drivers_enabled = 0U;
static volatile uint8_t output_active = 0U;
static volatile int32_t requested_frequency_hz = 0;
static volatile uint32_t target_frequency_hz = 0U;
static volatile uint32_t commanded_frequency_hz = 0U;
static volatile uint32_t applied_frequency_hz = 0U;
static volatile MotorDirection current_direction = DIR_CCW;
static volatile MotorDirection pending_direction = DIR_CCW;
static volatile uint32_t pending_frequency_hz = 0U;
static volatile uint32_t reverse_guard_deadline_ms = 0U;
static volatile uint32_t last_service_ms = 0U;
static volatile PulseReverseState_t reverse_state = PULSE_REVERSE_IDLE;

extern TIM_HandleTypeDef htim1;

static uint8_t PulseControl_IsReady(void)
{
    return ((initialized != 0U) && (p_htim1 != NULL) && (p_htim1->Instance != NULL)) ? 1U : 0U;
}

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
    uint32_t min_freq_hz = PULSECONTROL_MIN_FREQ_HZ;
    uint32_t max_freq_hz = PULSECONTROL_MAX_FREQ_HZ;

    if (max_freq_hz > MAX_PULSE_FREQ) {
        max_freq_hz = MAX_PULSE_FREQ;
    }
    if (min_freq_hz > max_freq_hz) {
        min_freq_hz = max_freq_hz;
    }

    if (freq_hz < min_freq_hz) {
        return min_freq_hz;
    }
    if (freq_hz > max_freq_hz) {
        return max_freq_hz;
    }
    return freq_hz;
}

static uint8_t PulseControl_DeadlineExpired(uint32_t deadline_ms)
{
    return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0) ? 1U : 0U;
}

static uint8_t PulseControl_ReadLineDriverEnablePins(void)
{
    GPIO_PinState de_state = HAL_GPIO_ReadPin(LINE_DRIVER_DE_GPIO_Port, LINE_DRIVER_DE_Pin);
    GPIO_PinState ren_state = HAL_GPIO_ReadPin(LINE_DRIVER_REN_GPIO_Port, LINE_DRIVER_REN_Pin);

    return ((de_state == GPIO_PIN_SET) && (ren_state == GPIO_PIN_SET)) ? 1U : 0U;
}

static void PulseControl_EnableSharedLineDrivers(void)
{
    HAL_GPIO_WritePin(LINE_DRIVER_DE_GPIO_Port, LINE_DRIVER_DE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LINE_DRIVER_REN_GPIO_Port, LINE_DRIVER_REN_Pin, GPIO_PIN_SET);
    line_drivers_enabled = PulseControl_ReadLineDriverEnablePins();
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
    if ((p_htim1 != NULL) && (p_htim1->Instance != NULL)) {
        HAL_TIM_PWM_Stop(p_htim1, TIM_CHANNEL_1);
    }

    output_active = 0U;
    commanded_frequency_hz = 0U;
    applied_frequency_hz = 0U;
}

static void PulseControl_ApplyPwmFrequency(uint32_t freq_hz)
{
    uint32_t timer_clock_hz = PulseControl_GetTimerClockHz();
    uint32_t prescaler = 1U;
    uint64_t denominator = (uint64_t)prescaler * (uint64_t)freq_hz;
    uint64_t period_counts = 0U;
    uint32_t autoreload = 0U;
    uint32_t compare = 0U;

    if ((freq_hz == 0U) || (timer_clock_hz == 0U) || (PulseControl_IsReady() == 0U)) {
        return;
    }

    prescaler = (uint32_t)(((uint64_t)timer_clock_hz +
                            (((uint64_t)freq_hz * 65536ULL) - 1ULL)) /
                           ((uint64_t)freq_hz * 65536ULL));
    if (prescaler == 0U) {
        prescaler = 1U;
    }
    if (prescaler > 65536U) {
        prescaler = 65536U;
    }

    denominator = (uint64_t)prescaler * (uint64_t)freq_hz;
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

    __HAL_TIM_SET_PRESCALER(p_htim1, prescaler - 1U);
    __HAL_TIM_SET_AUTORELOAD(p_htim1, autoreload);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, compare);
    __HAL_TIM_SET_COUNTER(p_htim1, 0U);
    p_htim1->Instance->EGR = TIM_EGR_UG;

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

static uint32_t PulseControl_StepRamp(uint32_t current_hz, uint32_t target_hz, uint32_t elapsed_ms)
{
#if PULSECONTROL_RAMP_HZ_PER_S > 0U
    uint64_t max_delta = ((uint64_t)PULSECONTROL_RAMP_HZ_PER_S * (uint64_t)elapsed_ms) / 1000ULL;

    if (elapsed_ms == 0U) {
        return current_hz;
    }
    if (max_delta == 0U) {
        max_delta = 1U;
    }

    if (target_hz > current_hz) {
        uint32_t delta = target_hz - current_hz;
        if ((uint64_t)delta > max_delta) {
            return current_hz + (uint32_t)max_delta;
        }
        return target_hz;
    }

    if (current_hz > target_hz) {
        uint32_t delta = current_hz - target_hz;
        if ((uint64_t)delta > max_delta) {
            return current_hz - (uint32_t)max_delta;
        }
        return target_hz;
    }
#else
    (void)elapsed_ms;
#endif

    return target_hz;
}

static void PulseControl_ServiceRamp(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - last_service_ms;
    uint32_t next_frequency_hz = 0U;

    if (target_frequency_hz == 0U) {
        if (output_active != 0U) {
            PulseControl_StopOutputInternal();
        }
        last_service_ms = now_ms;
        return;
    }

    next_frequency_hz = PulseControl_StepRamp(commanded_frequency_hz,
                                             target_frequency_hz,
                                             elapsed_ms);
    if ((next_frequency_hz != 0U) && (next_frequency_hz < PULSECONTROL_MIN_FREQ_HZ)) {
        next_frequency_hz = PULSECONTROL_MIN_FREQ_HZ;
    }

    if (next_frequency_hz != commanded_frequency_hz) {
        commanded_frequency_hz = next_frequency_hz;
        PulseControl_StartContinuousOutput(commanded_frequency_hz);
    } else if ((output_active == 0U) && (commanded_frequency_hz > 0U)) {
        PulseControl_StartContinuousOutput(commanded_frequency_hz);
    }

    last_service_ms = now_ms;
}

static void PulseControl_BeginReverseGuard(MotorDirection dir, uint32_t freq_hz)
{
    pending_direction = dir;
    pending_frequency_hz = freq_hz;
    target_frequency_hz = 0U;
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
        target_frequency_hz = pending_frequency_hz;
        commanded_frequency_hz = 0U;
        last_service_ms = HAL_GetTick();
    }
}

void PulseControl_Init(void)
{
    p_htim1 = &htim1;
    line_drivers_enabled = 0U;
    output_active = 0U;
    requested_frequency_hz = 0;
    target_frequency_hz = 0U;
    commanded_frequency_hz = 0U;
    applied_frequency_hz = 0U;
    current_direction = DIR_CW;
    pending_direction = DIR_CCW;
    pending_frequency_hz = 0U;
    reverse_guard_deadline_ms = 0U;
    last_service_ms = HAL_GetTick();
    reverse_state = PULSE_REVERSE_IDLE;
    initialized = 1U;

    PulseControl_EnableSharedLineDrivers();
    PulseControl_ApplyDirection(DIR_CCW);
}

void PulseControl_SetFrequency(int32_t freq_hz)
{
    MotorDirection target_direction = DIR_CCW;
    uint32_t target_hz = 0U;

    if (PulseControl_IsReady() == 0U) {
        return;
    }

    PulseControl_EnableSharedLineDrivers();
    requested_frequency_hz = freq_hz;

    if (freq_hz == 0) {
        pending_frequency_hz = 0U;
        target_frequency_hz = 0U;
        reverse_state = PULSE_REVERSE_IDLE;
        PulseControl_StopOutputInternal();
        return;
    }

    if (freq_hz > 0) {
        target_direction = DIR_CW;
        target_hz = PulseControl_ClampFrequencyHz((uint32_t)freq_hz);
    } else {
        target_direction = DIR_CCW;
        target_hz = PulseControl_ClampFrequencyHz((uint32_t)(-freq_hz));
    }

    if (reverse_state != PULSE_REVERSE_IDLE) {
        pending_direction = target_direction;
        pending_frequency_hz = target_hz;
        return;
    }

    if (target_direction != current_direction) {
        PulseControl_BeginReverseGuard(target_direction, target_hz);
        return;
    }

    target_frequency_hz = target_hz;
}

void PulseControl_Service(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (PulseControl_IsReady() == 0U) {
        return;
    }

    PulseControl_ServiceReverseGuard();
    if (reverse_state != PULSE_REVERSE_IDLE) {
        last_service_ms = now_ms;
        return;
    }

    PulseControl_ServiceRamp(now_ms);
}

void PulseControl_Stop(void)
{
    requested_frequency_hz = 0;
    pending_frequency_hz = 0U;
    target_frequency_hz = 0U;
    reverse_state = PULSE_REVERSE_IDLE;
    PulseControl_StopOutputInternal();
}

PulseControl_Status_t PulseControl_GetStatus(void)
{
    PulseControl_Status_t status = {0};

    status.requested_frequency_hz = requested_frequency_hz;
    status.applied_frequency_hz = applied_frequency_hz;
    status.direction = current_direction;
    status.output_active = output_active;
    status.line_driver_enabled = line_drivers_enabled;
    status.reverse_guard_active = (reverse_state != PULSE_REVERSE_IDLE) ? 1U : 0U;
    status.initialized = initialized;

    return status;
}
