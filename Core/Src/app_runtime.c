#include "app_runtime.h"

#include "app_runtime_live_debug.h"
#include "app_runtime_teleplot.h"
#include "main.h"
#include "gpio.h"
#include "iwdg.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"

#include "constants.h"
#include "debug_vars.h"
#include "encoder_reader.h"
#include "ethernet_communication.h"
#include "latency_profiler.h"
#include "project_params.h"
#include "position_control.h"
#include "position_control_diag.h"
#include "pulse_control.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint8_t interrupt_flag;

#if LATENCY_AUTO_REPORT_ENABLE
static uint32_t g_latency_report_seq = 0U;
#endif
static uint32_t g_debug_print_divider = 0U;
#if APP_RUNTIME_KEYBOARD_TEST_MODE
static float g_keyboard_target_steer_deg = 0.0f;
static char g_keyboard_line_buf[32] = {0};
static uint8_t g_keyboard_line_len = 0U;
#if APP_RUNTIME_KEYBOARD_SCENARIO_ENABLE
typedef struct {
    uint8_t active;
    uint8_t step_index;
    uint32_t test_id;
    uint32_t step_start_ms;
    uint32_t dwell_until_ms;
    uint32_t step_command_id;
} AppRuntime_KeyboardScenario_t;

static AppRuntime_KeyboardScenario_t g_keyboard_scenario = {0};
static uint32_t g_keyboard_scenario_next_id = 1U;
static const float g_keyboard_scenario_targets_deg[] = {
    0.0f, 5.0f, 0.0f, -5.0f, 0.0f, 20.0f, 30.0f, 40.0f
};
#endif
#else
static SteerMode_t g_prev_mode = STEER_MODE_NONE;
static SteerMode_t g_current_mode = STEER_MODE_NONE;
#endif
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
static uint8_t g_periodic_csv_enabled = APP_RUNTIME_PERIODIC_CSV_LOG_DEFAULT_ENABLE;
#endif

static int32_t AppRuntime_GetDisplayEncoderCount(void)
{
    return EncoderReader_GetCount();
}

static uint32_t AppRuntime_GetDisplayEncoderRaw(void)
{
    return EncoderReader_GetRawCounter();
}

static void AppRuntime_SetKeyboardTargetFromLiveDebug(float target_steering_deg)
{
#if APP_RUNTIME_KEYBOARD_TEST_MODE
    g_keyboard_target_steer_deg = target_steering_deg;
#else
    (void)target_steering_deg;
#endif
}

static void AppRuntime_PrintKinematicContract(void)
{
    printf("[KINCFG] motor_to_steering=%s motor_deg_per_steer_deg=%.3f\r\n",
           (MOTOR_TO_STEERING_POLARITY >= 0) ? "same" : "inverted",
           MOTOR_DEG_PER_STEERING_DEG);
}

static void AppRuntime_RequestControlEnable(const char *source)
{
    (void)source;
    PositionControl_Enable();
}

#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
/* Print the CSV schema once so bench logs remain self-describing. */
static void AppRuntime_PrintPeriodicCsvHeader(void)
{
    printf("CSV_HEADER,ms,mode,target_steering_deg,current_steering_deg,error_steering_deg,output_hz,dir,enc_cnt,enc_raw,req_hz,applied_hz,out_active,rev_guard,cmd_id,cmd_state,cmd_result\r\n");
}

static void AppRuntime_SetPeriodicCsvEnabled(uint8_t enabled, uint8_t print_header)
{
    g_periodic_csv_enabled = (enabled != 0U) ? 1U : 0U;
    dbg_csv_log_enable = g_periodic_csv_enabled;

    if ((g_periodic_csv_enabled != 0U) && (print_header != 0U)) {
        AppRuntime_PrintPeriodicCsvHeader();
    }
}

static uint8_t AppRuntime_GetPeriodicCsvEnabled(void)
{
    return g_periodic_csv_enabled;
}

/* Emit a throttled CSV telemetry row for offline log analysis. */
static void AppRuntime_ServicePeriodicCsv(void)
{
    static uint32_t last_ms = 0U;
    uint32_t now_ms = HAL_GetTick();

    if (g_periodic_csv_enabled == 0U) {
        return;
    }

    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    int32_t enc_count = AppRuntime_GetDisplayEncoderCount();
    uint32_t enc_raw = AppRuntime_GetDisplayEncoderRaw();

    if ((uint32_t)(now_ms - last_ms) < APP_RUNTIME_PERIODIC_CSV_LOG_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    printf("CSV,%lu,%d,%.3f,%.3f,%.3f,%.0f,%d,%ld,%lu,%ld,%lu,%u,%u,%lu,%d,%d\r\n",
           (unsigned long)now_ms,
           (int)PositionControl_GetMode(),
           s.target_steering_deg,
           s.current_steering_deg,
           s.error_steering_deg,
           s.output_hz,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           (long)pulse_status.requested_frequency_hz,
           (unsigned long)pulse_status.applied_frequency_hz,
           (unsigned int)pulse_status.output_active,
           (unsigned int)pulse_status.reverse_guard_active,
           (unsigned long)cmd.command_id,
           (int)cmd.state,
           (int)cmd.result);
}
#else
#define AppRuntime_PrintPeriodicCsvHeader() ((void)0)
#define AppRuntime_SetPeriodicCsvEnabled(enabled, print_header) ((void)0)
#define AppRuntime_GetPeriodicCsvEnabled() (0U)
#define AppRuntime_ServicePeriodicCsv() ((void)0)
#endif

static void AppRuntime_RequestControlEnableHook(const char *source)
{
    (void)source;
    AppRuntime_RequestControlEnable(source);
}

static void AppRuntime_SetPeriodicCsvEnabledHook(uint8_t enabled, uint8_t print_header)
{
    (void)enabled;
    (void)print_header;
    AppRuntime_SetPeriodicCsvEnabled(enabled, print_header);
}

static uint8_t AppRuntime_GetPeriodicCsvEnabledHook(void)
{
    return (uint8_t)AppRuntime_GetPeriodicCsvEnabled();
}

static const AppRuntimeLiveDebug_Hooks_t g_live_debug_hooks = {
    .request_control_enable = AppRuntime_RequestControlEnableHook,
    .set_keyboard_target_deg = AppRuntime_SetKeyboardTargetFromLiveDebug,
    .set_periodic_csv_enabled = AppRuntime_SetPeriodicCsvEnabledHook,
    .get_periodic_csv_enabled = AppRuntime_GetPeriodicCsvEnabledHook
};

/* Emit the latency-profiler batch report once enough samples have been collected. */
static void AppRuntime_TryLatencyAutoReport(void)
{
#if LATENCY_AUTO_REPORT_ENABLE
    static uint32_t check_div = 0U;
    uint32_t i = 0U;
    uint32_t miss_count = 0U;
    LatencyStageStats_t stats = {0};

    if (++check_div < 10U) {
        return;
    }
    check_div = 0U;

    for (i = 0U; i < (uint32_t)LAT_STAGE_COUNT; i++) {
        if (LatencyProfiler_GetStageSampleCount((LatencyStage_t)i) < LATENCY_AUTO_REPORT_SAMPLES) {
            return;
        }
    }

    miss_count = LatencyProfiler_GetDeadlineMissCount();
    printf("LATENCY_BATCH_BEGIN,seq=%lu,samples=%u,core_hz=%lu,deadline_miss=%lu\r\n",
           (unsigned long)g_latency_report_seq,
           (unsigned int)LATENCY_AUTO_REPORT_SAMPLES,
           (unsigned long)SystemCoreClock,
           (unsigned long)miss_count);

    for (i = 0U; i < (uint32_t)LAT_STAGE_COUNT; i++) {
        if (LatencyProfiler_GetStageStats((LatencyStage_t)i, &stats)) {
            printf("LATENCY_STAGE,seq=%lu,name=%s,count=%lu,avg_cycles=%lu,p99_cycles=%lu,max_cycles=%lu,avg_us=%.3f,p99_us=%.3f,max_us=%.3f\r\n",
                   (unsigned long)g_latency_report_seq,
                   LatencyProfiler_StageName((LatencyStage_t)i),
                   (unsigned long)stats.sample_count,
                   (unsigned long)stats.avg_cycles,
                   (unsigned long)stats.p99_cycles,
                   (unsigned long)stats.max_cycles,
                   stats.avg_us,
                   stats.p99_us,
                   stats.max_us);
        }
    }

    printf("LATENCY_BATCH_END,seq=%lu\r\n", (unsigned long)g_latency_report_seq);

    g_latency_report_seq++;
    LatencyProfiler_Reset();
#endif
}

/* Print live TIM2 encoder register details when bench diagnostics are enabled. */
static void AppRuntime_ServiceEncoderRuntimeDiag(void)
{
#if APP_RUNTIME_ENCODER_DIAG_ENABLE
    static uint32_t last_ms = 0U;
    static uint32_t prev_cnt = 32768UL;
    uint32_t now_ms = HAL_GetTick();
    uint32_t cnt = 0U;
    uint32_t delta_u32 = 0U;
    int32_t delta = 0;
    uint32_t cr1 = 0U;
    uint32_t smcr = 0U;
    uint32_t ccmr1 = 0U;
    uint32_t ccer = 0U;
    uint32_t cen = 0U;
    uint32_t sms = 0U;
    uint32_t cc1s = 0U;
    uint32_t cc2s = 0U;
    uint32_t cc1e = 0U;
    uint32_t cc2e = 0U;
    GPIO_PinState enc_a_state = GPIO_PIN_RESET;
    GPIO_PinState enc_b_state = GPIO_PIN_RESET;
    EncoderSample_t encoder_sample = {0};

    if (dbg_encoder_diag_enable == 0U) {
        return;
    }

    if ((uint32_t)(now_ms - last_ms) < APP_RUNTIME_ENCODER_DIAG_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    cnt = __HAL_TIM_GET_COUNTER(&htim2);
    delta_u32 = (uint32_t)(cnt - prev_cnt);
    delta = (int32_t)delta_u32;
    cr1 = htim2.Instance->CR1;
    smcr = htim2.Instance->SMCR;
    ccmr1 = htim2.Instance->CCMR1;
    ccer = htim2.Instance->CCER;
    cen = ((cr1 & TIM_CR1_CEN) != 0U) ? 1U : 0U;
    sms = (smcr & TIM_SMCR_SMS) >> TIM_SMCR_SMS_Pos;
    cc1s = (ccmr1 & TIM_CCMR1_CC1S) >> TIM_CCMR1_CC1S_Pos;
    cc2s = (ccmr1 & TIM_CCMR1_CC2S) >> TIM_CCMR1_CC2S_Pos;
    cc1e = ((ccer & TIM_CCER_CC1E) != 0U) ? 1U : 0U;
    cc2e = ((ccer & TIM_CCER_CC2E) != 0U) ? 1U : 0U;
    enc_a_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    enc_b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    (void)EncoderReader_GetLastSample(&encoder_sample);

    printf("[ENCDBG] ms=%lu cnt=%lu prev=%lu raw_delta=%ld signed_delta=%ld steer=%.3f motor=%.3f dir_pin=%d enc_valid=0x%02lX A=%d B=%d CEN=%lu SMS=%lu CC1S=%lu CC2S=%lu CC1E=%lu CC2E=%lu CR1=0x%04lX SMCR=0x%04lX CCMR1=0x%04lX CCER=0x%04lX\r\n",
           (unsigned long)now_ms,
           (unsigned long)cnt,
           (unsigned long)prev_cnt,
           (long)delta,
           (long)encoder_sample.delta_count,
           encoder_sample.steering_deg,
           encoder_sample.motor_deg,
           (int)HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin),
           (unsigned long)encoder_sample.validity,
           (int)enc_a_state,
           (int)enc_b_state,
           (unsigned long)cen,
           (unsigned long)sms,
           (unsigned long)cc1s,
           (unsigned long)cc2s,
           (unsigned long)cc1e,
           (unsigned long)cc2e,
           (unsigned long)cr1,
           (unsigned long)smcr,
           (unsigned long)ccmr1,
           (unsigned long)ccer);

    prev_cnt = cnt;
#endif
}

#if APP_RUNTIME_KEYBOARD_TEST_MODE
/* Clamp keyboard-entered steering angles to the allowed steering envelope. */
static float AppRuntime_KeyboardClampSteeringDeg(float steering_deg)
{
    if (steering_deg > MAX_STEERING_ANGLE) {
        return MAX_STEERING_ANGLE;
    }
    if (steering_deg < MIN_STEERING_ANGLE) {
        return MIN_STEERING_ANGLE;
    }
    return steering_deg;
}

/* Clear the buffered keyboard command line. */
static void AppRuntime_KeyboardClearLine(void)
{
    g_keyboard_line_len = 0U;
    g_keyboard_line_buf[0] = '\0';
}

/* Print a human-readable control snapshot for keyboard bench testing. */
static void AppRuntime_KeyboardPrintControlSnapshot(const char *reason)
{
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    int32_t enc_count = AppRuntime_GetDisplayEncoderCount();
    uint32_t enc_raw = AppRuntime_GetDisplayEncoderRaw();
    printf("[KB][%s] T=%.2fdeg C=%.2fdeg E=%.2fdeg O=%.0f DIR=%d ENC=%ld RAW=%lu REQ=%ld AP=%lu RUN=%u REV=%u CMD=%lu/%s/%s\r\n",
           reason,
           s.target_steering_deg,
           s.current_steering_deg,
           s.error_steering_deg,
           s.output_hz,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           (long)pulse_status.requested_frequency_hz,
           (unsigned long)pulse_status.applied_frequency_hz,
           (unsigned int)pulse_status.output_active,
           (unsigned int)pulse_status.reverse_guard_active,
           (unsigned long)cmd.command_id,
           PositionControlDiag_CommandStateString(cmd.state),
           PositionControlDiag_CommandResultString(cmd.result));
}

/* Apply the current keyboard target to the position controller. */
static void AppRuntime_KeyboardApplyTarget(void)
{
    float motor_target_deg = SteeringDegToMotorDeg(g_keyboard_target_steer_deg);
    int ret = PositionControl_SetTargetSteeringDegWithSource(g_keyboard_target_steer_deg, CMD_SRC_KEYBOARD);

#if APP_RUNTIME_KEYBOARD_AUTO_ENABLE_ON_TARGET
    if (ret == POS_CTRL_OK) {
        AppRuntime_RequestControlEnable("keyboard_target");
    }
#endif

    printf("[KB] target steer=%.1f deg motor=%.1f deg ret=%d\r\n",
           g_keyboard_target_steer_deg,
           motor_target_deg,
           ret);
    AppRuntime_KeyboardPrintControlSnapshot("target");
}

#if APP_RUNTIME_KEYBOARD_SCENARIO_ENABLE
static uint8_t AppRuntime_KeyboardScenarioStepCount(void)
{
    return (uint8_t)(sizeof(g_keyboard_scenario_targets_deg) /
                     sizeof(g_keyboard_scenario_targets_deg[0]));
}

static const char* AppRuntime_KeyboardScenarioStepLevel(const char *result)
{
    if (result == NULL) {
        return "FAIL";
    }
    if ((strcmp(result, "REACHED") == 0) ||
        (strcmp(result, "ALREADY_IN_BAND") == 0)) {
        return "OK";
    }
    return "FAIL";
}

static void AppRuntime_KeyboardScenarioStop(const char *reason)
{
    if (g_keyboard_scenario.active == 0U) {
        return;
    }

    printf("TEST_END,id=%lu,scenario=keyboard_baseline,result=ABORT,reason=%s,step=%u\r\n",
           (unsigned long)g_keyboard_scenario.test_id,
           (reason != NULL) ? reason : "manual",
           (unsigned int)g_keyboard_scenario.step_index);
    printf("[SCN] #%lu ABORT reason=%s step=%u\r\n",
           (unsigned long)g_keyboard_scenario.test_id,
           (reason != NULL) ? reason : "manual",
           (unsigned int)g_keyboard_scenario.step_index);
    g_keyboard_scenario.active = 0U;
    g_keyboard_scenario.dwell_until_ms = 0U;
}

static void AppRuntime_KeyboardScenarioStartStep(void)
{
    uint8_t step_count = AppRuntime_KeyboardScenarioStepCount();
    float target_steering_deg = 0.0f;
    float motor_target_deg = 0.0f;
    int ret = POS_CTRL_OK;
    CommandLifecycle_t cmd = {0};
    PositionControl_State_t s = {0};

    if ((g_keyboard_scenario.active == 0U) ||
        (g_keyboard_scenario.step_index >= step_count)) {
        return;
    }

    target_steering_deg = g_keyboard_scenario_targets_deg[g_keyboard_scenario.step_index];
    g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(target_steering_deg);
    motor_target_deg = SteeringDegToMotorDeg(g_keyboard_target_steer_deg);
    ret = PositionControl_SetTargetSteeringDegWithSource(g_keyboard_target_steer_deg, CMD_SRC_KEYBOARD);
    if (ret == POS_CTRL_OK) {
        AppRuntime_RequestControlEnable("keyboard_scenario");
    }

    cmd = PositionControl_GetCommandLifecycle();
    g_keyboard_scenario.step_start_ms = HAL_GetTick();
    g_keyboard_scenario.step_command_id = cmd.command_id;
    g_keyboard_scenario.dwell_until_ms = 0U;

    printf("STEP_BEGIN,test_id=%lu,idx=%u,total=%u,target_steering_deg=%.3f,target_motor_deg=%.3f,cmd=%lu,ret=%d\r\n",
           (unsigned long)g_keyboard_scenario.test_id,
           (unsigned int)(g_keyboard_scenario.step_index + 1U),
           (unsigned int)step_count,
           g_keyboard_target_steer_deg,
           motor_target_deg,
           (unsigned long)g_keyboard_scenario.step_command_id,
           ret);
    s = PositionControl_GetState();
    printf("[SCN] #%lu STEP %u/%u target_steering=%.1fdeg current_steering=%.2fdeg error_steering=%.2fdeg ret=%d\r\n",
           (unsigned long)g_keyboard_scenario.test_id,
           (unsigned int)(g_keyboard_scenario.step_index + 1U),
           (unsigned int)step_count,
           g_keyboard_target_steer_deg,
           s.current_steering_deg,
           s.error_steering_deg,
           ret);
    AppRuntime_KeyboardPrintControlSnapshot("scenario_step");

    if (ret != POS_CTRL_OK) {
        printf("TEST_END,id=%lu,scenario=keyboard_baseline,result=FAIL,reason=set_target_failed,step=%u\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               (unsigned int)(g_keyboard_scenario.step_index + 1U));
        printf("[SCN] #%lu COMPLETE FAIL reason=set_target_failed step=%u\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               (unsigned int)(g_keyboard_scenario.step_index + 1U));
        g_keyboard_scenario.active = 0U;
    }
}

static void AppRuntime_KeyboardScenarioFinishStep(const char *result,
                                                  const CommandLifecycle_t *cmd)
{
    PositionControl_State_t s = PositionControl_GetState();
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - g_keyboard_scenario.step_start_ms;
    float final_steering_deg = s.current_steering_deg;
    float final_error_steering_deg = s.error_steering_deg;
    uint32_t cmd_id = g_keyboard_scenario.step_command_id;

    if (cmd != NULL) {
        cmd_id = cmd->command_id;
        final_steering_deg = cmd->final_steering_deg;
        final_error_steering_deg = cmd->final_error_steering_deg;
        if (cmd->end_ms >= cmd->start_ms) {
            elapsed_ms = cmd->end_ms - cmd->start_ms;
        }
    }

    printf("STEP_END,test_id=%lu,idx=%u,result=%s,cmd=%lu,elapsed_ms=%lu,target_steering_deg=%.3f,final_steering_deg=%.3f,final_error_steering_deg=%.3f\r\n",
           (unsigned long)g_keyboard_scenario.test_id,
           (unsigned int)(g_keyboard_scenario.step_index + 1U),
           (result != NULL) ? result : "UNKNOWN",
           (unsigned long)cmd_id,
           (unsigned long)elapsed_ms,
           g_keyboard_scenario_targets_deg[g_keyboard_scenario.step_index],
           final_steering_deg,
           final_error_steering_deg);
    printf("[SCN] #%lu STEP %u %s result=%s target_steering=%.1fdeg final_steering=%.2fdeg err_steering=%.2fdeg time=%lums\r\n",
           (unsigned long)g_keyboard_scenario.test_id,
           (unsigned int)(g_keyboard_scenario.step_index + 1U),
           AppRuntime_KeyboardScenarioStepLevel(result),
           (result != NULL) ? result : "UNKNOWN",
           g_keyboard_scenario_targets_deg[g_keyboard_scenario.step_index],
           final_steering_deg,
           final_error_steering_deg,
           (unsigned long)elapsed_ms);

    g_keyboard_scenario.dwell_until_ms = now_ms + APP_RUNTIME_KEYBOARD_SCENARIO_DWELL_MS;
}

static void AppRuntime_KeyboardScenarioStart(void)
{
    AppRuntime_KeyboardScenarioStop("restart");
    memset(&g_keyboard_scenario, 0, sizeof(g_keyboard_scenario));
    g_keyboard_scenario.active = 1U;
    g_keyboard_scenario.test_id = g_keyboard_scenario_next_id++;

    printf("TEST_BEGIN,id=%lu,scenario=keyboard_baseline,steps=%u,targets=0,5,0,-5,0,20,30,40,note=press_Z_before_1_for_bench_zero\r\n",
           (unsigned long)g_keyboard_scenario.test_id,
           (unsigned int)AppRuntime_KeyboardScenarioStepCount());
    printf("[SCN] #%lu START baseline targets: 0 -> 5 -> 0 -> -5 -> 0 -> 20 -> 30 -> 40 deg\r\n",
           (unsigned long)g_keyboard_scenario.test_id);
    AppRuntime_KeyboardScenarioStartStep();
}

static void AppRuntime_KeyboardScenarioService(void)
{
    CommandLifecycle_t cmd = {0};
    PositionControl_State_t s = {0};
    uint32_t now_ms = HAL_GetTick();
    uint8_t step_count = AppRuntime_KeyboardScenarioStepCount();

    if (g_keyboard_scenario.active == 0U) {
        return;
    }

    if (g_keyboard_scenario.dwell_until_ms != 0U) {
        if ((int32_t)(now_ms - g_keyboard_scenario.dwell_until_ms) < 0) {
            return;
        }

        g_keyboard_scenario.step_index++;
        if (g_keyboard_scenario.step_index >= step_count) {
            printf("TEST_END,id=%lu,scenario=keyboard_baseline,result=PASS,steps=%u\r\n",
                   (unsigned long)g_keyboard_scenario.test_id,
                   (unsigned int)step_count);
            printf("[SCN] #%lu COMPLETE PASS steps=%u\r\n",
                   (unsigned long)g_keyboard_scenario.test_id,
                   (unsigned int)step_count);
            g_keyboard_scenario.active = 0U;
            g_keyboard_scenario.dwell_until_ms = 0U;
            return;
        }

        AppRuntime_KeyboardScenarioStartStep();
        return;
    }

    cmd = PositionControl_GetCommandLifecycle();
    s = PositionControl_GetState();

    if ((cmd.command_id == g_keyboard_scenario.step_command_id) &&
        (cmd.state == CMD_REACHED)) {
        AppRuntime_KeyboardScenarioFinishStep("REACHED", &cmd);
        return;
    }

    if ((cmd.command_id == g_keyboard_scenario.step_command_id) &&
        ((cmd.state == CMD_ABORTED) ||
         (cmd.state == CMD_TIMEOUT) ||
         (cmd.state == CMD_FAULTED))) {
        AppRuntime_KeyboardScenarioFinishStep(PositionControlDiag_CommandStateString(cmd.state), &cmd);
        printf("TEST_END,id=%lu,scenario=keyboard_baseline,result=FAIL,reason=%s,step=%u\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               PositionControlDiag_CommandResultString(cmd.result),
               (unsigned int)(g_keyboard_scenario.step_index + 1U));
        printf("[SCN] #%lu COMPLETE FAIL reason=%s step=%u\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               PositionControlDiag_CommandResultString(cmd.result),
               (unsigned int)(g_keyboard_scenario.step_index + 1U));
        g_keyboard_scenario.active = 0U;
        return;
    }

    if (!((cmd.command_id == g_keyboard_scenario.step_command_id) &&
          (cmd.state == CMD_ACTIVE)) &&
        (fabsf(s.error_motor_deg) < PositionControl_GetStableErrorMotorDeg())) {
        AppRuntime_KeyboardScenarioFinishStep("ALREADY_IN_BAND", NULL);
        return;
    }

    if ((now_ms - g_keyboard_scenario.step_start_ms) >
        APP_RUNTIME_KEYBOARD_SCENARIO_STEP_TIMEOUT_MS) {
        printf("STEP_END,test_id=%lu,idx=%u,result=SCENARIO_TIMEOUT,cmd=%lu,elapsed_ms=%lu,target_steering_deg=%.3f,current_steering_deg=%.3f,error_steering_deg=%.3f\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               (unsigned int)(g_keyboard_scenario.step_index + 1U),
               (unsigned long)g_keyboard_scenario.step_command_id,
               (unsigned long)(now_ms - g_keyboard_scenario.step_start_ms),
               g_keyboard_scenario_targets_deg[g_keyboard_scenario.step_index],
               s.current_steering_deg,
               s.error_steering_deg);
        printf("[SCN] #%lu STEP %u FAIL result=TIMEOUT target_steering=%.1fdeg current_steering=%.2fdeg error_steering=%.2fdeg time=%lums\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               (unsigned int)(g_keyboard_scenario.step_index + 1U),
               g_keyboard_scenario_targets_deg[g_keyboard_scenario.step_index],
               s.current_steering_deg,
               s.error_steering_deg,
               (unsigned long)(now_ms - g_keyboard_scenario.step_start_ms));
        PositionControl_Disable();
        printf("TEST_END,id=%lu,scenario=keyboard_baseline,result=FAIL,reason=step_timeout,step=%u\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               (unsigned int)(g_keyboard_scenario.step_index + 1U));
        printf("[SCN] #%lu COMPLETE FAIL reason=step_timeout step=%u\r\n",
               (unsigned long)g_keyboard_scenario.test_id,
               (unsigned int)(g_keyboard_scenario.step_index + 1U));
        g_keyboard_scenario.active = 0U;
    }
}
#else
#define AppRuntime_KeyboardScenarioStop(reason) ((void)0)
#define AppRuntime_KeyboardScenarioService() ((void)0)
#endif

/* Define the current bench position as zero for the active feedback reference. */
static void AppRuntime_KeyboardZeroCurrentPosition(void)
{
    PositionControl_Disable();
    PulseControl_Stop();
    EncoderReader_Reset();

    PositionControl_Reset();
    g_keyboard_target_steer_deg = 0.0f;
    (void)PositionControl_SetTargetSteeringDegWithSource(0.0f, CMD_SRC_KEYBOARD);

    printf("[KB] zero set tim2=reset\r\n");
    AppRuntime_KeyboardPrintControlSnapshot("zero");
}

/* Print the interactive keyboard bench-test help text. */
static void AppRuntime_KeyboardPrintHelp(void)
{
    printf("[KB] 1:scenario A:left D:right S:center Z:zero E:enable Q:disable X:estop P:print T:teleplot H:help step=%.1f deg\r\n",
           APP_RUNTIME_KEYBOARD_STEP_DEG);
    printf("[KB] direction test: press Z then 1. Numeric target also works, ex) 5, -3.5, 0.\r\n");
}

/* Parse the buffered numeric steering target and apply it. */
static void AppRuntime_KeyboardApplyTypedTarget(void)
{
    char *end_ptr = NULL;
    float typed_target_deg = 0.0f;

    g_keyboard_line_buf[g_keyboard_line_len] = '\0';
    typed_target_deg = strtof(g_keyboard_line_buf, &end_ptr);

    if (end_ptr == g_keyboard_line_buf || *end_ptr != '\0') {
        printf("[KB] invalid target \"%s\"\r\n", g_keyboard_line_buf);
        AppRuntime_KeyboardClearLine();
        return;
    }

    g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(typed_target_deg);
    AppRuntime_KeyboardApplyTarget();
    AppRuntime_KeyboardClearLine();
}

/* Service the UART-driven keyboard bench-test interface. */
static void AppRuntime_KeyboardProcessInput(void)
{
    uint8_t ch = 0U;

    if (HAL_UART_Receive(&huart3, &ch, 1, 0U) != HAL_OK) {
        return;
    }

    switch (ch) {
    case '\r':
    case '\n':
        if (g_keyboard_line_len > 0U) {
            AppRuntime_KeyboardApplyTypedTarget();
        }
        break;

    case '\b':
    case 0x7FU:
        if (g_keyboard_line_len > 0U) {
            g_keyboard_line_len--;
            g_keyboard_line_buf[g_keyboard_line_len] = '\0';
        }
        break;

    case 'a':
    case 'A':
        AppRuntime_KeyboardScenarioStop("manual_a");
        AppRuntime_KeyboardClearLine();
        g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(
            g_keyboard_target_steer_deg - APP_RUNTIME_KEYBOARD_STEP_DEG);
        AppRuntime_KeyboardApplyTarget();
        break;

    case 'd':
    case 'D':
        AppRuntime_KeyboardScenarioStop("manual_d");
        AppRuntime_KeyboardClearLine();
        g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(
            g_keyboard_target_steer_deg + APP_RUNTIME_KEYBOARD_STEP_DEG);
        AppRuntime_KeyboardApplyTarget();
        break;

    case 's':
    case 'S':
        AppRuntime_KeyboardScenarioStop("manual_s");
        AppRuntime_KeyboardClearLine();
        g_keyboard_target_steer_deg = 0.0f;
        AppRuntime_KeyboardApplyTarget();
        break;

    case 'z':
    case 'Z':
        AppRuntime_KeyboardScenarioStop("manual_z");
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardZeroCurrentPosition();
        break;

    case 'e':
    case 'E':
        AppRuntime_KeyboardScenarioStop("manual_e");
        AppRuntime_KeyboardClearLine();
        AppRuntime_RequestControlEnable("keyboard");
        break;

    case 'q':
    case 'Q':
        AppRuntime_KeyboardScenarioStop("manual_q");
        AppRuntime_KeyboardClearLine();
        PositionControl_Disable();
        printf("[KB] control disabled\r\n");
        break;

    case 'x':
    case 'X':
        AppRuntime_KeyboardScenarioStop("manual_x");
        AppRuntime_KeyboardClearLine();
        PositionControl_EmergencyStop();
        printf("[KB] emergency stop\r\n");
        break;

    case 'p':
    case 'P':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardPrintControlSnapshot("snapshot");
        break;

    case 'l':
    case 'L':
        AppRuntime_KeyboardClearLine();
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
        AppRuntime_SetPeriodicCsvEnabled((uint8_t)(g_periodic_csv_enabled == 0U ? 1U : 0U), 0U);
        printf("[KB] csv log %s\r\n", (g_periodic_csv_enabled != 0U) ? "enabled" : "disabled");
        if (g_periodic_csv_enabled != 0U) {
            AppRuntime_PrintPeriodicCsvHeader();
        }
#else
        printf("[KB] csv log feature disabled at build time\r\n");
#endif
        break;

    case 't':
    case 'T':
        AppRuntime_KeyboardClearLine();
#if APP_RUNTIME_TELEPLOT_ENABLE
        dbg_teleplot_enable = (uint8_t)(dbg_teleplot_enable == 0U ? 1U : 0U);
        printf("[KB] teleplot %s\r\n", (dbg_teleplot_enable != 0U) ? "enabled" : "disabled");
#else
        printf("[KB] teleplot feature disabled at build time\r\n");
#endif
        break;

    case 'g':
    case 'G':
        AppRuntime_KeyboardClearLine();
#if APP_RUNTIME_ENCODER_DIAG_ENABLE
        dbg_encoder_diag_enable = (uint8_t)(dbg_encoder_diag_enable == 0U ? 1U : 0U);
        printf("[KB] encoder diag %s\r\n", (dbg_encoder_diag_enable != 0U) ? "enabled" : "disabled");
#else
        printf("[KB] encoder diag feature disabled at build time\r\n");
#endif
        break;

    case 'h':
    case 'H':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardPrintHelp();
        break;

#if APP_RUNTIME_KEYBOARD_SCENARIO_ENABLE
    case '1':
        if (g_keyboard_line_len == 0U) {
            AppRuntime_KeyboardClearLine();
            AppRuntime_KeyboardScenarioStart();
        } else if (g_keyboard_line_len < (uint8_t)(sizeof(g_keyboard_line_buf) - 1U)) {
            g_keyboard_line_buf[g_keyboard_line_len++] = (char)ch;
            g_keyboard_line_buf[g_keyboard_line_len] = '\0';
        } else {
            printf("[KB] input too long\r\n");
            AppRuntime_KeyboardClearLine();
        }
        break;
#endif

    default:
        if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.') {
            AppRuntime_KeyboardScenarioStop("manual_target");
            if (g_keyboard_line_len < (uint8_t)(sizeof(g_keyboard_line_buf) - 1U)) {
                g_keyboard_line_buf[g_keyboard_line_len++] = (char)ch;
                g_keyboard_line_buf[g_keyboard_line_len] = '\0';
            } else {
                printf("[KB] input too long\r\n");
                AppRuntime_KeyboardClearLine();
            }
        } else {
            printf("[KB] unknown key '%c' (0x%02X)\r\n", (char)ch, (unsigned int)ch);
            AppRuntime_KeyboardClearLine();
        }
        break;
    }
}
#endif

/* Configure the direction line-driver GPIO that CubeMX leaves to user code. */
static void AppRuntime_ConfigureDirectionPin(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = GPIO_PIN_10;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio_init);
}

#if !APP_RUNTIME_KEYBOARD_TEST_MODE
/* Process UDP mode changes and map received packets into controller commands. */
static void AppRuntime_ServiceUdpComms(void)
{
    SteerMode_t mode = STEER_MODE_NONE;
    uint32_t now_ms = 0U;
    uint32_t last_rx_ms = 0U;

    LAT_BEGIN(LAT_STAGE_COMMS);
    MX_LWIP_Process();

    mode = EthComm_GetCurrentMode();
    now_ms = HAL_GetTick();
    last_rx_ms = EthComm_GetLastRxTick();

    if ((mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) &&
        ((now_ms - last_rx_ms) > ETHCOMM_RX_TIMEOUT_MS)) {
        EthComm_ForceMode(STEER_MODE_ESTOP);
        mode = STEER_MODE_ESTOP;
    }

    if (mode == STEER_MODE_ESTOP) {
        if (g_prev_mode != STEER_MODE_ESTOP) {
            PositionControl_EmergencyStop();
        }
    } else if (mode == STEER_MODE_NONE) {
        if (g_prev_mode != STEER_MODE_NONE) {
            PositionControl_Disable();
        }
    } else if ((mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) &&
               (g_prev_mode == STEER_MODE_NONE || g_prev_mode == STEER_MODE_ESTOP)) {
        AppRuntime_RequestControlEnable("udp_mode");
    }

    if (EthComm_ConsumeEmergencyRequest()) {
        PositionControl_EmergencyStop();
        mode = STEER_MODE_ESTOP;
    }

    if (EthComm_HasNewData()) {
        AutoDrive_Packet_t pkt = EthComm_GetLatestData();
        if (mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) {
            PositionControl_SetTargetSteeringDegWithSource(pkt.steering_angle, CMD_SRC_UDP);
        }
    }

    LAT_END(LAT_STAGE_COMMS);

    g_current_mode = mode;
    g_prev_mode = mode;
}
#endif

/* Emit the slower human-readable diagnostic snapshot used during bring-up. */
static void AppRuntime_PrintPeriodicDiag(void)
{
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    int32_t enc_count = AppRuntime_GetDisplayEncoderCount();
    uint32_t enc_raw = AppRuntime_GetDisplayEncoderRaw();
    float target_steer_deg = s.target_steering_deg;
    float current_steer_deg = s.current_steering_deg;
    float error_steer_deg = s.error_steering_deg;

#if LATENCY_LOG_ENABLE
    printf("[DIAG] MODE:%d CMD:%lu/%s/%s Tst:%.2f Cst:%.2f Est:%.2f O:%.0f DIR:%d ENC:%ld RAW:%lu REQ:%ld AP:%lu RUN:%u REV:%u\r\n",
           (int)PositionControl_GetMode(),
           (unsigned long)cmd.command_id,
           PositionControlDiag_CommandStateString(cmd.state),
           PositionControlDiag_CommandResultString(cmd.result),
           target_steer_deg,
           current_steer_deg,
           error_steer_deg,
           s.output_hz,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           (long)pulse_status.requested_frequency_hz,
           (unsigned long)pulse_status.applied_frequency_hz,
           (unsigned int)pulse_status.output_active,
           (unsigned int)pulse_status.reverse_guard_active);
#else
    (void)pulse_status;
    (void)enc_count;
    (void)enc_raw;
    (void)dir_state;
    (void)s;
    (void)cmd;
    (void)target_steer_deg;
    (void)current_steer_deg;
    (void)error_steer_deg;
#endif
}

/* Service the 1 ms application path that runs from the timer interrupt flag. */
static void AppRuntime_ServiceFastTick(void)
{
    (void)EncoderReader_Update();
#if APP_RUNTIME_AUTO_FIXED_PULSE_TEST
#if !APP_RUNTIME_KEYBOARD_TEST_MODE
    if (g_current_mode == STEER_MODE_AUTO) {
        PulseControl_SetFrequency(APP_RUNTIME_AUTO_FIXED_PULSE_HZ);
    } else {
        PulseControl_Stop();
    }
#else
    PulseControl_Stop();
#endif
#else
    PositionControl_Update();
#endif
    PulseControl_Service();

#if APP_RUNTIME_KEYBOARD_TEST_MODE
    AppRuntime_KeyboardScenarioService();
#endif

    AppRuntime_ServiceEncoderRuntimeDiag();

    if (++g_debug_print_divider < APP_RUNTIME_PERIODIC_DIAG_DIVIDER) {
        return;
    }

    g_debug_print_divider = 0U;
    AppRuntime_PrintPeriodicDiag();
}

/* Initialize the application-specific runtime after CubeMX peripherals are ready. */
void AppRuntime_Init(void)
{
    LatencyProfiler_Init(SystemCoreClock);
    AppRuntime_ConfigureDirectionPin();

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    PulseControl_Init();
    EncoderReader_Init();
    PositionControl_Init();

    HAL_Delay(500);

    {
        char msg[] = "Servo Start!\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
    }

#if APP_RUNTIME_RESET_ENCODER_ON_BOOT
    EncoderReader_Reset();
    printf("[BOOT_ZERO] mode=current_position_as_zero tim2=reset target_steering_deg=0.000 auto_start=%u\r\n",
           (unsigned int)APP_RUNTIME_AUTO_START_CONTROL_ENABLE);
#endif
    AppRuntime_PrintKinematicContract();

    EncoderReader_Service();
    PositionControl_Reset();
    PositionControl_SetTargetSteeringDegWithSource(0.0f, CMD_SRC_LOCALTEST);
#if APP_RUNTIME_KEYBOARD_TEST_MODE
    g_keyboard_target_steer_deg = 0.0f;
#else
    g_prev_mode = STEER_MODE_NONE;
    g_current_mode = STEER_MODE_NONE;
#endif
    g_debug_print_divider = 0U;
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
    AppRuntime_SetPeriodicCsvEnabled(APP_RUNTIME_PERIODIC_CSV_LOG_DEFAULT_ENABLE, 0U);
#else
    dbg_csv_log_enable = 0U;
#endif
#if APP_RUNTIME_TELEPLOT_ENABLE
    dbg_teleplot_enable = APP_RUNTIME_TELEPLOT_DEFAULT_ENABLE;
#else
    dbg_teleplot_enable = 0U;
#endif
#if APP_RUNTIME_ENCODER_DIAG_ENABLE
    dbg_encoder_diag_enable = APP_RUNTIME_ENCODER_DIAG_DEFAULT_ENABLE;
#else
    dbg_encoder_diag_enable = 0U;
#endif
#if APP_RUNTIME_AUTO_START_CONTROL_ENABLE
    AppRuntime_RequestControlEnable("boot_auto");
#endif

#if APP_RUNTIME_KEYBOARD_TEST_MODE
    AppRuntime_KeyboardPrintHelp();
    if (dbg_csv_log_enable != 0U) {
        AppRuntime_PrintPeriodicCsvHeader();
    }
#else
    EthComm_UDP_Init();
#endif
}

/* Run one application super-loop iteration on top of the CubeMX main loop. */
void AppRuntime_RunIteration(void)
{
#if APP_RUNTIME_KEYBOARD_TEST_MODE
    LAT_BEGIN(LAT_STAGE_COMMS);
    AppRuntime_KeyboardProcessInput();
    LAT_END(LAT_STAGE_COMMS);
#else
    AppRuntime_ServiceUdpComms();
#endif
    AppRuntimeLiveDebug_Service(&g_live_debug_hooks);

    if (interrupt_flag != 0U) {
        interrupt_flag = 0U;
        AppRuntime_ServiceFastTick();
    }

    AppRuntimeTeleplot_Service();
    AppRuntime_ServicePeriodicCsv();
    AppRuntime_TryLatencyAutoReport();
#if APP_RUNTIME_IWDG_ENABLE
    HAL_IWDG_Refresh(&hiwdg);
#endif
}
