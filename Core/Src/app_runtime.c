#include "app_runtime.h"

#include "main.h"
#include "gpio.h"
#include "iwdg.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"

#include "adc_potentiometer.h"
#include "can_encoder_bridge.h"
#include "constants.h"
#include "encoder_reader.h"
#include "ethernet_communication.h"
#include "homing.h"
#include "latency_profiler.h"
#include "project_params.h"
#include "position_control.h"
#include "position_control_diag.h"
#include "pulse_control.h"
#include "relay_control.h"
#include "rs422_encoder_uart.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint8_t interrupt_flag;

static uint32_t g_latency_report_seq = 0U;
static uint32_t g_debug_print_divider = 0U;
#if APP_RUNTIME_KEYBOARD_TEST_MODE
static float g_keyboard_target_steer_deg = 0.0f;
static char g_keyboard_line_buf[32] = {0};
static uint8_t g_keyboard_line_len = 0U;
#else
static SteerMode_t g_prev_mode = STEER_MODE_NONE;
static SteerMode_t g_current_mode = STEER_MODE_NONE;
#endif
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
static uint8_t g_periodic_csv_enabled = 1U;
#endif

#if APP_RUNTIME_SENSOR_DIAG_ENABLE
#define APP_SENSOR_WARN_ENCODER_STALE        (1UL << 0)
#define APP_SENSOR_WARN_ADC_STALE            (1UL << 1)
#define APP_SENSOR_WARN_CROSSCHECK           (1UL << 2)
#define APP_SENSOR_WARN_VELOCITY             (1UL << 3)
#define APP_SENSOR_WARN_ACCEL                (1UL << 4)
#define APP_SENSOR_WARN_ADC_JUMP             (1UL << 5)
#define APP_SENSOR_WARN_ADC_STUCK            (1UL << 6)
#define APP_SENSOR_WARN_DIR_MISMATCH         (1UL << 7)

#define APP_SENSOR_FAULT_ENCODER_STALE       (1UL << 16)
#define APP_SENSOR_FAULT_ADC_STALE           (1UL << 17)
#define APP_SENSOR_FAULT_ADC_INVALID         (1UL << 18)
#define APP_SENSOR_FAULT_CROSSCHECK          (1UL << 19)
#define APP_SENSOR_FAULT_VELOCITY            (1UL << 20)
#define APP_SENSOR_FAULT_ACCEL               (1UL << 21)
#define APP_SENSOR_FAULT_DIR_MISMATCH        (1UL << 22)

typedef struct {
    EncoderSample_t encoder;
    ADC_PotSample_t adc;
    ADC_PotCalibration_t adc_calibration;
    float crosscheck_error_deg;
    uint32_t warn_flags;
    uint32_t fault_flags;
    uint32_t crosscheck_condition_start_ms;
    uint32_t direction_condition_start_ms;
    const char *last_reason;
} AppRuntime_SensorDiag_t;

static AppRuntime_SensorDiag_t g_sensor_diag = {
    .last_reason = "none"
};
#endif

#if APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE
typedef struct {
    int64_t accum_count;
    float count_residual;
} AppRuntime_VirtualEncoder_t; /* Putty-only encoder estimator from applied pulse output. */

static AppRuntime_VirtualEncoder_t g_virtual_encoder = {0};

/* Reset the bench-only display encoder derived from pulse output. */
static void AppRuntime_ResetVirtualEncoder(void)
{
    g_virtual_encoder.accum_count = 0;
    g_virtual_encoder.count_residual = 0.0f;
    EncoderReader_SetVirtualFeedbackCount(0);
}

/* Approximate encoder motion from the currently applied pulse frequency. */
static void AppRuntime_UpdateVirtualEncoder(void)
{
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    float delta_pulses = 0.0f;
    float delta_counts = 0.0f;
    float total_counts = 0.0f;
    int32_t whole_counts = 0;

    if ((pulse_status.output_active == 0U) || (pulse_status.applied_frequency_hz == 0U)) {
        EncoderReader_SetVirtualFeedbackCount(g_virtual_encoder.accum_count);
        return;
    }

    delta_pulses = ((float)pulse_status.applied_frequency_hz) * 0.001f;
    if (pulse_status.direction == DIR_CCW) {
        delta_pulses = -delta_pulses;
    }

    delta_counts = delta_pulses * (DEG_PER_PULSE / ENCODER_DEG_PER_COUNT);
    total_counts = g_virtual_encoder.count_residual + delta_counts;
    whole_counts = (int32_t)total_counts;

    g_virtual_encoder.accum_count += (int64_t)whole_counts;
    g_virtual_encoder.count_residual = total_counts - (float)whole_counts;
    EncoderReader_SetVirtualFeedbackCount(g_virtual_encoder.accum_count);
}

/* Return the Putty display count derived from the commanded motion. */
static int32_t AppRuntime_GetDisplayEncoderCount(void)
{
    return (int32_t)g_virtual_encoder.accum_count;
}

/* Return a timer-like raw counter value for Putty display only. */
static uint32_t AppRuntime_GetDisplayEncoderRaw(void)
{
    int32_t raw32 = 32768 + (int32_t)g_virtual_encoder.accum_count;
    return (uint32_t)((uint16_t)raw32);
}
#else
#define AppRuntime_ResetVirtualEncoder() ((void)0)
#define AppRuntime_UpdateVirtualEncoder() ((void)0)
static int32_t AppRuntime_GetDisplayEncoderCount(void)
{
    return EncoderReader_GetCount();
}

static uint32_t AppRuntime_GetDisplayEncoderRaw(void)
{
    return EncoderReader_GetRawCounter();
}
#endif

/* Convert a steering-angle target into the equivalent motor-angle target. */
static float AppRuntime_TargetSteeringDegToMotorDeg(float steering_deg)
{
    return SteeringDegToMotorDeg(steering_deg);
}

/* Convert a motor-angle reading back into steering-angle units. */
static float AppRuntime_TargetMotorDegToSteeringDeg(float motor_deg)
{
    return MotorDegToSteeringDeg(motor_deg);
}

#if APP_RUNTIME_SENSOR_DIAG_ENABLE
static const char* AppRuntime_SensorReasonString(uint32_t warn_flags, uint32_t fault_flags)
{
    if ((fault_flags & APP_SENSOR_FAULT_ENCODER_STALE) != 0U) {
        return "encoder_stale";
    }
    if ((fault_flags & APP_SENSOR_FAULT_ADC_STALE) != 0U) {
        return "adc_stale";
    }
    if ((fault_flags & APP_SENSOR_FAULT_ADC_INVALID) != 0U) {
        return "adc_invalid";
    }
    if ((fault_flags & APP_SENSOR_FAULT_CROSSCHECK) != 0U) {
        return "sensor_crosscheck";
    }
    if ((fault_flags & APP_SENSOR_FAULT_VELOCITY) != 0U) {
        return "encoder_velocity";
    }
    if ((fault_flags & APP_SENSOR_FAULT_ACCEL) != 0U) {
        return "encoder_accel";
    }
    if ((fault_flags & APP_SENSOR_FAULT_DIR_MISMATCH) != 0U) {
        return "direction_mismatch";
    }

    if ((warn_flags & APP_SENSOR_WARN_ENCODER_STALE) != 0U) {
        return "encoder_stale_warn";
    }
    if ((warn_flags & APP_SENSOR_WARN_ADC_STALE) != 0U) {
        return "adc_stale_warn";
    }
    if ((warn_flags & APP_SENSOR_WARN_CROSSCHECK) != 0U) {
        return "sensor_crosscheck_warn";
    }
    if ((warn_flags & APP_SENSOR_WARN_VELOCITY) != 0U) {
        return "encoder_velocity_warn";
    }
    if ((warn_flags & APP_SENSOR_WARN_ACCEL) != 0U) {
        return "encoder_accel_warn";
    }
    if ((warn_flags & APP_SENSOR_WARN_ADC_JUMP) != 0U) {
        return "adc_jump_warn";
    }
    if ((warn_flags & APP_SENSOR_WARN_ADC_STUCK) != 0U) {
        return "adc_stuck_warn";
    }
    if ((warn_flags & APP_SENSOR_WARN_DIR_MISMATCH) != 0U) {
        return "direction_mismatch_warn";
    }

    return "none";
}

static void AppRuntime_ResetSensorDiag(void)
{
    memset(&g_sensor_diag, 0, sizeof(g_sensor_diag));
    g_sensor_diag.last_reason = "none";
#if APP_RUNTIME_ADC_POT_ENABLE
    (void)ADC_Pot_GetCalibration(&g_sensor_diag.adc_calibration);
#else
    g_sensor_diag.adc.validity = ADC_POT_VALID;
#endif
}

static void AppRuntime_PrintSensorContract(void)
{
#if APP_RUNTIME_ADC_POT_ENABLE
    printf("[SENCFG] +steering=%s +motor=%s +encoder_count=%s DIR_PIN_1=%s adc_raw_inc=%s\r\n",
           (SENSOR_POSITIVE_STEERING_IS_CW != 0) ? "CW" : "CCW",
           (SENSOR_POSITIVE_MOTOR_IS_CW != 0) ? "CW" : "CCW",
           (ENCODER_COUNT_POLARITY >= 0) ? "CW" : "CCW",
           (SENSOR_DIR_PIN_ONE_IS_CW != 0) ? "CW" : "CCW",
           (ADC_POT_STEERING_POLARITY >= 0) ? "+steering" : "-steering");
#else
    printf("[SENCFG] +steering=%s +motor=%s +encoder_count=%s DIR_PIN_1=%s adc_pot=disabled\r\n",
           (SENSOR_POSITIVE_STEERING_IS_CW != 0) ? "CW" : "CCW",
           (SENSOR_POSITIVE_MOTOR_IS_CW != 0) ? "CW" : "CCW",
           (ENCODER_COUNT_POLARITY >= 0) ? "CW" : "CCW",
           (SENSOR_DIR_PIN_ONE_IS_CW != 0) ? "CW" : "CCW");
#endif
}

static void AppRuntime_LogSensorState(const char *level,
                                      uint32_t warn_flags,
                                      uint32_t fault_flags)
{
    printf("[SENSOR][%s] reason=%s warn=0x%08lX fault=0x%08lX enc=%.3f adc=%.3f xerr=%.3f enc_age=%lu adc_age=%lu enc_valid=0x%02lX adc_valid=0x%02lX calib_v=%lu calib_crc=0x%08lX\r\n",
           level,
           g_sensor_diag.last_reason,
           (unsigned long)warn_flags,
           (unsigned long)fault_flags,
           g_sensor_diag.encoder.steering_deg,
           g_sensor_diag.adc.calibrated_angle_deg,
           g_sensor_diag.crosscheck_error_deg,
           (unsigned long)g_sensor_diag.encoder.age_ms,
           (unsigned long)g_sensor_diag.adc.age_ms,
           (unsigned long)g_sensor_diag.encoder.validity,
           (unsigned long)g_sensor_diag.adc.validity,
           (unsigned long)g_sensor_diag.adc_calibration.version,
           (unsigned long)g_sensor_diag.adc_calibration.checksum);
}

static uint8_t AppRuntime_SensorsReadyForControl(void)
{
    if ((APP_RUNTIME_ADC_POT_ENABLE != 0) &&
        (APP_RUNTIME_AUTO_HOME_ON_BOOT != 0) &&
        (Homing_IsComplete() == 0U)) {
        g_sensor_diag.last_reason = Homing_GetLastFailureReason();
        return 0U;
    }

    if ((APP_RUNTIME_EMERGENCY_LATCH_ENABLE != 0) && (g_sensor_diag.fault_flags != 0U)) {
        return 0U;
    }

    if ((g_sensor_diag.encoder.validity & (ENCODER_FAULT_STALE | ENCODER_INVALID_NOT_INIT)) != 0U) {
        g_sensor_diag.last_reason = "encoder_not_ready";
        return 0U;
    }

    if (APP_RUNTIME_ADC_POT_ENABLE != 0) {
        if ((g_sensor_diag.adc.validity & (ADC_POT_INVALID_NOT_INIT |
                                           ADC_POT_INVALID_DISCONNECT |
                                           ADC_POT_INVALID_TIMEOUT |
                                           ADC_POT_INVALID_RANGE)) != 0U) {
            g_sensor_diag.last_reason = "adc_not_ready";
            return 0U;
        }
    }

    return 1U;
}

static void AppRuntime_RequestControlEnable(const char *source)
{
    if (AppRuntime_SensorsReadyForControl() == 0U) {
        printf("[SENSOR][BLOCK] enable=%s reason=%s warn=0x%08lX fault=0x%08lX homing=%u\r\n",
               source,
               g_sensor_diag.last_reason,
               (unsigned long)g_sensor_diag.warn_flags,
               (unsigned long)g_sensor_diag.fault_flags,
               (unsigned int)Homing_IsComplete());
        return;
    }

    PositionControl_Enable();
}

static void AppRuntime_ServiceSensorSupervisor(void)
{
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    uint32_t now_ms = HAL_GetTick();
    uint32_t prev_warn_flags = g_sensor_diag.warn_flags;
    uint32_t prev_fault_flags = g_sensor_diag.fault_flags;
    uint32_t warn_flags = 0U;
    uint32_t fault_flags = 0U;
    float delta_steer_deg = 0.0f;
    float abs_crosscheck_error_deg = 0.0f;
    uint8_t encoder_ok = 0U;
    uint8_t adc_ok = 0U;

    EncoderReader_Service();
#if APP_RUNTIME_ADC_POT_ENABLE
    ADC_Pot_Service();
#endif

    encoder_ok = EncoderReader_GetSample(&g_sensor_diag.encoder);
#if APP_RUNTIME_ADC_POT_ENABLE
    adc_ok = ADC_Pot_GetSample(&g_sensor_diag.adc);
    (void)ADC_Pot_GetCalibration(&g_sensor_diag.adc_calibration);
#else
    g_sensor_diag.adc.validity = ADC_POT_VALID;
    g_sensor_diag.adc.age_ms = 0U;
#endif

    if ((encoder_ok == 0U) || ((g_sensor_diag.encoder.validity & ENCODER_INVALID_NOT_INIT) != 0U)) {
        fault_flags |= APP_SENSOR_FAULT_ENCODER_STALE;
    }
    if ((g_sensor_diag.encoder.validity & ENCODER_WARN_STALE) != 0U) {
        warn_flags |= APP_SENSOR_WARN_ENCODER_STALE;
    }
    if ((g_sensor_diag.encoder.validity & ENCODER_FAULT_STALE) != 0U) {
        fault_flags |= APP_SENSOR_FAULT_ENCODER_STALE;
    }
    if ((g_sensor_diag.encoder.validity & ENCODER_WARN_VELOCITY) != 0U) {
        warn_flags |= APP_SENSOR_WARN_VELOCITY;
    }
    if ((g_sensor_diag.encoder.validity & ENCODER_FAULT_VELOCITY) != 0U) {
        fault_flags |= APP_SENSOR_FAULT_VELOCITY;
    }
    if ((g_sensor_diag.encoder.validity & ENCODER_WARN_ACCEL) != 0U) {
        warn_flags |= APP_SENSOR_WARN_ACCEL;
    }
    if ((g_sensor_diag.encoder.validity & ENCODER_FAULT_ACCEL) != 0U) {
        fault_flags |= APP_SENSOR_FAULT_ACCEL;
    }

    if (APP_RUNTIME_ADC_POT_ENABLE != 0) {
        if ((adc_ok == 0U) || ((g_sensor_diag.adc.validity & ADC_POT_INVALID_NOT_INIT) != 0U)) {
            fault_flags |= APP_SENSOR_FAULT_ADC_STALE;
        }
        if (g_sensor_diag.adc.age_ms >= ADC_POT_SAMPLE_STALE_WARN_MS) {
            warn_flags |= APP_SENSOR_WARN_ADC_STALE;
        }
        if ((g_sensor_diag.adc.age_ms >= ADC_POT_SAMPLE_STALE_FAULT_MS) ||
            ((g_sensor_diag.adc.validity & ADC_POT_INVALID_TIMEOUT) != 0U)) {
            fault_flags |= APP_SENSOR_FAULT_ADC_STALE;
        }
        if ((g_sensor_diag.adc.validity & (ADC_POT_INVALID_DISCONNECT | ADC_POT_INVALID_RANGE)) != 0U) {
            fault_flags |= APP_SENSOR_FAULT_ADC_INVALID;
        }
        if ((g_sensor_diag.adc.validity & ADC_POT_INVALID_JUMP) != 0U) {
            warn_flags |= APP_SENSOR_WARN_ADC_JUMP;
        }
        if ((g_sensor_diag.adc.validity & ADC_POT_INVALID_STUCK) != 0U) {
            warn_flags |= APP_SENSOR_WARN_ADC_STUCK;
        }
    }

    g_sensor_diag.crosscheck_error_deg = 0.0f;
    if ((APP_RUNTIME_ADC_POT_ENABLE != 0) &&
        (encoder_ok != 0U) && (adc_ok != 0U) &&
        ((fault_flags & (APP_SENSOR_FAULT_ENCODER_STALE |
                         APP_SENSOR_FAULT_ADC_STALE |
                         APP_SENSOR_FAULT_ADC_INVALID)) == 0U)) {
        g_sensor_diag.crosscheck_error_deg =
            g_sensor_diag.encoder.steering_deg - g_sensor_diag.adc.calibrated_angle_deg;
        abs_crosscheck_error_deg = fabsf(g_sensor_diag.crosscheck_error_deg);

        if (abs_crosscheck_error_deg >= SENSOR_CROSSCHECK_WARN_STEERING_DEG) {
            if (g_sensor_diag.crosscheck_condition_start_ms == 0U) {
                g_sensor_diag.crosscheck_condition_start_ms = now_ms;
            }
            if ((now_ms - g_sensor_diag.crosscheck_condition_start_ms) >= SENSOR_CROSSCHECK_WARN_PERSIST_MS) {
                warn_flags |= APP_SENSOR_WARN_CROSSCHECK;
            }
            if ((abs_crosscheck_error_deg >= SENSOR_CROSSCHECK_FAULT_STEERING_DEG) &&
                ((now_ms - g_sensor_diag.crosscheck_condition_start_ms) >= SENSOR_CROSSCHECK_FAULT_PERSIST_MS)) {
                fault_flags |= APP_SENSOR_FAULT_CROSSCHECK;
            }
        } else {
            g_sensor_diag.crosscheck_condition_start_ms = 0U;
        }
    } else {
        g_sensor_diag.crosscheck_condition_start_ms = 0U;
    }

    if ((SENSOR_DIRECTION_PLAUSIBILITY_ENABLE != 0) &&
        (pulse_status.output_active != 0U) &&
        (pulse_status.applied_frequency_hz >= SENSOR_DIRECTION_MIN_APPLIED_HZ)) {
        int32_t expected_sign = (pulse_status.direction == DIR_CW) ? 1 : -1;
        int32_t actual_sign = 0;

        delta_steer_deg = MotorDegToSteeringDeg((float)g_sensor_diag.encoder.delta_count * ENCODER_DEG_PER_COUNT);
        if (delta_steer_deg > SENSOR_DIRECTION_MIN_STEERING_DELTA_DEG) {
            actual_sign = 1;
        } else if (delta_steer_deg < -SENSOR_DIRECTION_MIN_STEERING_DELTA_DEG) {
            actual_sign = -1;
        }

        if ((actual_sign != 0) && (actual_sign != expected_sign)) {
            if (g_sensor_diag.direction_condition_start_ms == 0U) {
                g_sensor_diag.direction_condition_start_ms = now_ms;
            }
            if ((now_ms - g_sensor_diag.direction_condition_start_ms) >= SENSOR_DIRECTION_WARN_PERSIST_MS) {
                warn_flags |= APP_SENSOR_WARN_DIR_MISMATCH;
            }
            if ((now_ms - g_sensor_diag.direction_condition_start_ms) >= SENSOR_DIRECTION_FAULT_PERSIST_MS) {
                fault_flags |= APP_SENSOR_FAULT_DIR_MISMATCH;
            }
        } else {
            g_sensor_diag.direction_condition_start_ms = 0U;
        }
    } else {
        g_sensor_diag.direction_condition_start_ms = 0U;
    }

    g_sensor_diag.warn_flags = warn_flags;
    g_sensor_diag.fault_flags = fault_flags;
    g_sensor_diag.last_reason = AppRuntime_SensorReasonString(warn_flags, fault_flags);

    if ((fault_flags != prev_fault_flags) || (warn_flags != prev_warn_flags)) {
        if (fault_flags != 0U) {
            AppRuntime_LogSensorState("FAULT", warn_flags, fault_flags);
        } else if (warn_flags != 0U) {
            AppRuntime_LogSensorState("WARN", warn_flags, fault_flags);
        } else {
            AppRuntime_LogSensorState("CLEAR", warn_flags, fault_flags);
        }
    }

    if ((APP_RUNTIME_EMERGENCY_LATCH_ENABLE != 0) &&
        (fault_flags != 0U) &&
        (prev_fault_flags == 0U)) {
        PositionControl_EmergencyStop();
    }
}
#else
#define AppRuntime_ResetSensorDiag() ((void)0)
#define AppRuntime_PrintSensorContract() ((void)0)
#define AppRuntime_SensorsReadyForControl() (1U)
#define AppRuntime_RequestControlEnable(source) PositionControl_Enable()
#define AppRuntime_ServiceSensorSupervisor() ((void)0)
#endif

#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
/* Print the CSV schema once so bench logs remain self-describing. */
static void AppRuntime_PrintPeriodicCsvHeader(void)
{
    printf("CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc_cnt,enc_raw,enc_deg,adc_deg,xerr_deg,enc_age_ms,adc_age_ms,enc_valid,adc_valid,sen_warn,sen_fault,sen_reason,req_hz,applied_hz,out_active,rev_guard,cmd_id,cmd_state,cmd_result\r\n");
}

/* Emit a throttled CSV telemetry row for offline log analysis. */
static void AppRuntime_ServicePeriodicCsv(void)
{
    static uint32_t last_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    PulseControl_Status_t pulse_status = PulseControl_GetStatus();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    int32_t enc_count = AppRuntime_GetDisplayEncoderCount();
    uint32_t enc_raw = AppRuntime_GetDisplayEncoderRaw();
#if APP_RUNTIME_SENSOR_DIAG_ENABLE
    float enc_steering_deg = g_sensor_diag.encoder.steering_deg;
    float adc_steering_deg = g_sensor_diag.adc.calibrated_angle_deg;
    float crosscheck_error_deg = g_sensor_diag.crosscheck_error_deg;
    uint32_t enc_age_ms = g_sensor_diag.encoder.age_ms;
    uint32_t adc_age_ms = g_sensor_diag.adc.age_ms;
    uint32_t enc_validity = g_sensor_diag.encoder.validity;
    uint32_t adc_validity = g_sensor_diag.adc.validity;
    uint32_t sensor_warn_flags = g_sensor_diag.warn_flags;
    uint32_t sensor_fault_flags = g_sensor_diag.fault_flags;
    const char *sensor_reason = g_sensor_diag.last_reason;
#else
    float enc_steering_deg = 0.0f;
    float adc_steering_deg = 0.0f;
    float crosscheck_error_deg = 0.0f;
    uint32_t enc_age_ms = 0U;
    uint32_t adc_age_ms = 0U;
    uint32_t enc_validity = 0U;
    uint32_t adc_validity = 0U;
    uint32_t sensor_warn_flags = 0U;
    uint32_t sensor_fault_flags = 0U;
    const char *sensor_reason = "none";
#endif

    if (g_periodic_csv_enabled == 0U) {
        return;
    }

    if ((uint32_t)(now_ms - last_ms) < APP_RUNTIME_PERIODIC_CSV_LOG_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    printf("CSV,%lu,%d,%.3f,%.3f,%.3f,%.0f,%d,%ld,%lu,%.3f,%.3f,%.3f,%lu,%lu,0x%02lX,0x%02lX,0x%08lX,0x%08lX,%s,%ld,%lu,%u,%u,%lu,%d,%d\r\n",
           (unsigned long)now_ms,
           (int)PositionControl_GetMode(),
           AppRuntime_TargetMotorDegToSteeringDeg(s.target_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.current_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.error),
           s.output,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           enc_steering_deg,
           adc_steering_deg,
           crosscheck_error_deg,
           (unsigned long)enc_age_ms,
           (unsigned long)adc_age_ms,
           (unsigned long)enc_validity,
           (unsigned long)adc_validity,
           (unsigned long)sensor_warn_flags,
           (unsigned long)sensor_fault_flags,
           sensor_reason,
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
#define AppRuntime_ServicePeriodicCsv() ((void)0)
#endif

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
    (void)EncoderReader_GetSample(&encoder_sample);

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
#if APP_RUNTIME_SENSOR_DIAG_ENABLE
    float adc_steering_deg = g_sensor_diag.adc.calibrated_angle_deg;
    float crosscheck_error_deg = g_sensor_diag.crosscheck_error_deg;
    uint32_t encoder_validity = g_sensor_diag.encoder.validity;
    uint32_t adc_validity = g_sensor_diag.adc.validity;
    uint32_t sensor_warn_flags = g_sensor_diag.warn_flags;
    uint32_t sensor_fault_flags = g_sensor_diag.fault_flags;
    const char *sensor_reason = g_sensor_diag.last_reason;
#else
    float adc_steering_deg = 0.0f;
    float crosscheck_error_deg = 0.0f;
    uint32_t encoder_validity = 0U;
    uint32_t adc_validity = 0U;
    uint32_t sensor_warn_flags = 0U;
    uint32_t sensor_fault_flags = 0U;
    const char *sensor_reason = "none";
#endif

    printf("[KB][%s] T=%.2fdeg C=%.2fdeg E=%.2fdeg O=%.0f DIR=%d ENC=%ld RAW=%lu ADC=%.2f XERR=%.2f EV=0x%02lX AV=0x%02lX SW=0x%08lX SF=0x%08lX SR=%s REQ=%ld AP=%lu RUN=%u REV=%u CMD=%lu/%s/%s\r\n",
           reason,
           AppRuntime_TargetMotorDegToSteeringDeg(s.target_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.current_angle),
           AppRuntime_TargetMotorDegToSteeringDeg(s.error),
           s.output,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           adc_steering_deg,
           crosscheck_error_deg,
           (unsigned long)encoder_validity,
           (unsigned long)adc_validity,
           (unsigned long)sensor_warn_flags,
           (unsigned long)sensor_fault_flags,
           sensor_reason,
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
    int ret = PositionControl_SetTargetWithSource(motor_target_deg, CMD_SRC_KEYBOARD);

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

/* Define the current bench position as zero for both TIM2 and RS422 references. */
static void AppRuntime_KeyboardZeroCurrentPosition(void)
{
    Rs422Encoder_Status_t rs422_status = {0};
    uint8_t rs422_has_frame = Rs422Encoder_GetLatest(&rs422_status);
    uint8_t rs422_zero_ok = 0U;

    PositionControl_Disable();
    PulseControl_Stop();
    EncoderReader_Reset();

#if RS422_ENCODER_READER_ENABLE
    if (rs422_has_frame != 0U) {
        rs422_zero_ok = Rs422Encoder_SetZeroCurrent();
    }
#endif

    PositionControl_Reset();
    g_keyboard_target_steer_deg = 0.0f;
    (void)PositionControl_SetTargetWithSource(0.0f, CMD_SRC_KEYBOARD);

    printf("[KB] zero set tim2=reset rs422=%s raw=%ld zero=%ld frames=%lu\r\n",
           (rs422_zero_ok != 0U) ? "set" : ((rs422_has_frame != 0U) ? "failed" : "no_frame"),
           (long)rs422_status.last_count,
           (long)rs422_status.zero_count,
           (unsigned long)rs422_status.frames);
    AppRuntime_KeyboardPrintControlSnapshot("zero");
}

/* Print the interactive keyboard bench-test help text. */
static void AppRuntime_KeyboardPrintHelp(void)
{
    printf("[KB] A:left D:right S:center Z:zero E:enable Q:disable X:estop P:print L:csv H:help step=%.1f deg\r\n",
           APP_RUNTIME_KEYBOARD_STEP_DEG);
    printf("[KB] numeric target: type steering deg then Enter. ex) 5, -3.5, 0\r\n");
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
        AppRuntime_KeyboardClearLine();
        g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(
            g_keyboard_target_steer_deg - APP_RUNTIME_KEYBOARD_STEP_DEG);
        AppRuntime_KeyboardApplyTarget();
        break;

    case 'd':
    case 'D':
        AppRuntime_KeyboardClearLine();
        g_keyboard_target_steer_deg = AppRuntime_KeyboardClampSteeringDeg(
            g_keyboard_target_steer_deg + APP_RUNTIME_KEYBOARD_STEP_DEG);
        AppRuntime_KeyboardApplyTarget();
        break;

    case 's':
    case 'S':
        AppRuntime_KeyboardClearLine();
        g_keyboard_target_steer_deg = 0.0f;
        AppRuntime_KeyboardApplyTarget();
        break;

    case 'z':
    case 'Z':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardZeroCurrentPosition();
        break;

    case 'e':
    case 'E':
        AppRuntime_KeyboardClearLine();
        AppRuntime_RequestControlEnable("keyboard");
        break;

    case 'q':
    case 'Q':
        AppRuntime_KeyboardClearLine();
        PositionControl_Disable();
        printf("[KB] control disabled\r\n");
        break;

    case 'x':
    case 'X':
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
        g_periodic_csv_enabled = (uint8_t)(g_periodic_csv_enabled == 0U ? 1U : 0U);
        printf("[KB] csv log %s\r\n", (g_periodic_csv_enabled != 0U) ? "enabled" : "disabled");
        if (g_periodic_csv_enabled != 0U) {
            AppRuntime_PrintPeriodicCsvHeader();
        }
#else
        printf("[KB] csv log feature disabled at build time\r\n");
#endif
        break;

    case 'h':
    case 'H':
        AppRuntime_KeyboardClearLine();
        AppRuntime_KeyboardPrintHelp();
        break;

    default:
        if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.') {
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
            PositionControl_SetTargetWithSource(AppRuntime_TargetSteeringDegToMotorDeg(pkt.steering_angle),
                                                CMD_SRC_UDP);
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
    float target_steer_deg = AppRuntime_TargetMotorDegToSteeringDeg(s.target_angle);
    float current_steer_deg = AppRuntime_TargetMotorDegToSteeringDeg(s.current_angle);
    float error_steer_deg = AppRuntime_TargetMotorDegToSteeringDeg(s.error);
#if APP_RUNTIME_SENSOR_DIAG_ENABLE
    float adc_steering_deg = g_sensor_diag.adc.calibrated_angle_deg;
    float crosscheck_error_deg = g_sensor_diag.crosscheck_error_deg;
    uint32_t encoder_validity = g_sensor_diag.encoder.validity;
    uint32_t adc_validity = g_sensor_diag.adc.validity;
    uint32_t sensor_warn_flags = g_sensor_diag.warn_flags;
    uint32_t sensor_fault_flags = g_sensor_diag.fault_flags;
    const char *sensor_reason = g_sensor_diag.last_reason;
#else
    float adc_steering_deg = 0.0f;
    float crosscheck_error_deg = 0.0f;
    uint32_t encoder_validity = 0U;
    uint32_t adc_validity = 0U;
    uint32_t sensor_warn_flags = 0U;
    uint32_t sensor_fault_flags = 0U;
    const char *sensor_reason = "none";
#endif

#if LATENCY_LOG_ENABLE
    printf("[DIAG] MODE:%d CMD:%lu/%s/%s Tst:%.2f Cst:%.2f Est:%.2f O:%.0f DIR:%d ENC:%ld RAW:%lu ADC:%.2f XERR:%.2f EV:0x%02lX AV:0x%02lX SW:0x%08lX SF:0x%08lX SR:%s REQ:%ld AP:%lu ARR:%lu CCR:%lu RUN:%u REV:%u\r\n",
           (int)PositionControl_GetMode(),
           (unsigned long)cmd.command_id,
           PositionControlDiag_CommandStateString(cmd.state),
           PositionControlDiag_CommandResultString(cmd.result),
           target_steer_deg,
           current_steer_deg,
           error_steer_deg,
           s.output,
           (int)dir_state,
           (long)enc_count,
           (unsigned long)enc_raw,
           adc_steering_deg,
           crosscheck_error_deg,
           (unsigned long)encoder_validity,
           (unsigned long)adc_validity,
           (unsigned long)sensor_warn_flags,
           (unsigned long)sensor_fault_flags,
           sensor_reason,
           (long)pulse_status.requested_frequency_hz,
           (unsigned long)pulse_status.applied_frequency_hz,
           (unsigned long)pulse_status.autoreload,
           (unsigned long)pulse_status.compare,
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
    (void)adc_steering_deg;
    (void)crosscheck_error_deg;
    (void)encoder_validity;
    (void)adc_validity;
    (void)sensor_warn_flags;
    (void)sensor_fault_flags;
    (void)sensor_reason;
#endif
}

/* Service the 1 ms application path that runs from the timer interrupt flag. */
static void AppRuntime_ServiceFastTick(void)
{
    AppRuntime_UpdateVirtualEncoder();
    AppRuntime_ServiceSensorSupervisor();

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

    Relay_Init();
    PulseControl_Init();
    EncoderReader_Init();
#if APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE
    EncoderReader_EnableVirtualFeedback(1U);
#else
    EncoderReader_EnableVirtualFeedback(0U);
#endif
#if APP_RUNTIME_ADC_POT_ENABLE
    ADC_Pot_Init(NULL);
#endif
    Homing_Init();
    PositionControl_Init();
#if CAN_ENCODER_BRIDGE_ENABLE
    (void)CanEncoderBridge_Init();
#endif
#if RS422_ENCODER_READER_ENABLE
    (void)Rs422Encoder_Init();
#endif

    Relay_ServoOn();
    HAL_Delay(500);

    {
        char msg[] = "Servo Start!\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
    }

    AppRuntime_PrintSensorContract();

#if APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE
    printf("[VENC] Putty ENC/RAW uses pulse-integrated virtual encoder display.\r\n");
#endif

#if APP_RUNTIME_RESET_ENCODER_ON_BOOT
    EncoderReader_Reset();
#endif
    AppRuntime_ResetVirtualEncoder();
    AppRuntime_ResetSensorDiag();

#if (APP_RUNTIME_AUTO_HOME_ON_BOOT && APP_RUNTIME_ADC_POT_ENABLE)
    if (Homing_FindZero() != 0) {
        printf("[Homing] Boot homing failed: reason=%s xerr=%.3f deg\r\n",
               Homing_GetLastFailureReason(),
               Homing_GetLastCrosscheckErrorDeg());
    }
#endif

    EncoderReader_Service();
#if APP_RUNTIME_ADC_POT_ENABLE
    ADC_Pot_Service();
#endif
    AppRuntime_ServiceSensorSupervisor();
    PositionControl_SetTargetWithSource(AppRuntime_TargetSteeringDegToMotorDeg(0.0f), CMD_SRC_LOCALTEST);
#if APP_RUNTIME_KEYBOARD_TEST_MODE
    g_keyboard_target_steer_deg = 0.0f;
#else
    g_prev_mode = STEER_MODE_NONE;
    g_current_mode = STEER_MODE_NONE;
#endif
    g_debug_print_divider = 0U;
#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
    g_periodic_csv_enabled = 1U;
#endif
#if APP_RUNTIME_AUTO_START_CONTROL_ENABLE
    AppRuntime_RequestControlEnable("boot_auto");
#endif

#if APP_RUNTIME_KEYBOARD_TEST_MODE
    AppRuntime_KeyboardPrintHelp();
    AppRuntime_PrintPeriodicCsvHeader();
#else
    EthComm_UDP_Init();
#endif
}

/* Run one application super-loop iteration on top of the CubeMX main loop. */
void AppRuntime_RunIteration(void)
{
#if CAN_ENCODER_BRIDGE_ENABLE
    CanEncoderBridge_Service();
#endif
#if RS422_ENCODER_READER_ENABLE
    Rs422Encoder_Service();
#endif

#if APP_RUNTIME_KEYBOARD_TEST_MODE
    LAT_BEGIN(LAT_STAGE_COMMS);
    AppRuntime_KeyboardProcessInput();
    LAT_END(LAT_STAGE_COMMS);
#else
    AppRuntime_ServiceUdpComms();
#endif

    if (interrupt_flag != 0U) {
        interrupt_flag = 0U;
        AppRuntime_ServiceFastTick();
    }

    AppRuntime_ServicePeriodicCsv();
    AppRuntime_TryLatencyAutoReport();
    HAL_IWDG_Refresh(&hiwdg);
}
