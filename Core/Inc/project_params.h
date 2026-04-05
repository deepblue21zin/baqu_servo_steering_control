#ifndef PROJECT_PARAMS_H
#define PROJECT_PARAMS_H

/*
 * Central place for user-editable tuning and bench switches.
 * Physical/mechanical conversion constants stay in constants.h.
 * Internal implementation constants stay local to each module.
 * Each section lists the primary code path that consumes the setting.
 */

/* ========== App Runtime / Bench Switches ==========
 * Primary use: Core/Src/app_runtime.c
 */
#define APP_RUNTIME_AUTO_FIXED_PULSE_TEST        0        /* bench-only fixed pulse mode in AppRuntime_ServiceFastTick() */
#define APP_RUNTIME_AUTO_FIXED_PULSE_HZ          500000   /* fixed pulse frequency when the bench mode above is enabled */
#define APP_RUNTIME_KEYBOARD_TEST_MODE           1        /* keyboard-local test path vs UDP path selection */
#define APP_RUNTIME_KEYBOARD_STEP_DEG            1.0f     /* +/- steering step used by keyboard jog commands */
#define APP_RUNTIME_ENCODER_DIAG_ENABLE          1        /* periodic encoder runtime diagnostic prints */
#define APP_RUNTIME_ENCODER_DIAG_PERIOD_MS       100U     /* encoder diagnostic report period */
#define APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE   0        /* Putty-only pulse-integrated virtual encoder display */
#define APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE      1        /* periodic CSV telemetry output */
#define APP_RUNTIME_PERIODIC_CSV_LOG_PERIOD_MS   100U     /* periodic CSV telemetry period */
#define APP_RUNTIME_PERIODIC_DIAG_DIVIDER        100U     /* 1 ms loop divider for slow periodic diagnostics */

/* ========== App Runtime / Boot Sequence ==========
 * Primary use: Core/Src/app_runtime.c::AppRuntime_Init()
 */
#define APP_RUNTIME_AUTO_START_CONTROL_ENABLE    1        /* auto-enable closed-loop control after module initialization */
#define APP_RUNTIME_RESET_ENCODER_ON_BOOT        1        /* zero the logical encoder origin during boot to match current firmware baseline */

/* ========== Position Control ==========
 * Primary use: Core/Inc/position_control.h, Core/Src/position_control.c
 */
#define MAX_ANGLE_DEG                    4320.0f          /* software motor-angle travel clamp */
#define MIN_ANGLE_DEG                   -4320.0f          /* software motor-angle travel clamp */
#define MAX_TRACKING_ERROR_DEG           4500.0f          /* tracking-error fault threshold */
#define DEFAULT_KP                         50.0f          /* closed-loop proportional gain baseline */
#define DEFAULT_KI                          5.0f          /* closed-loop integral gain baseline */
#define DEFAULT_KD                         20.0f          /* closed-loop derivative gain baseline */
#define DEFAULT_INTEGRAL_LIMIT          1000.0f           /* PID integrator clamp */
#define DEFAULT_OUTPUT_LIMIT           10000.0f           /* controller output clamp before pulse conversion */
#define STABLE_ERROR_THRESHOLD             0.5f           /* in-position error band */
#define STABLE_TIME_MS                     100U           /* time inside the in-position band before CMD_REACHED */
#define POSITION_COMMAND_TIMEOUT_MS          0U           /* command watchdog; 0 disables timeout */

/* ========== Position Safety ==========
 * Primary use: Core/Src/position_control_safety.c
 */
#define POSITION_SAFETY_ANGLE_MARGIN_DEG    5.0f          /* extra angle margin outside MAX/MIN_ANGLE_DEG before fault */

/* ========== Pulse Output ==========
 * Primary use: Core/Inc/pulse_control.h, Core/Src/pulse_control.c
 */
#define DIR_ACTIVE_HIGH_FOR_CW               1            /* direction pin polarity for CW rotation */
#define PULSECONTROL_MIN_FREQ_HZ            10U           /* non-zero pulse clamp to avoid too-slow pulse output */
#define PULSECONTROL_MAX_FREQ_HZ        100000U           /* firmware-side pulse clamp for TIM1 generation */
#define PULSECONTROL_DIRECTION_GUARD_MS      1U           /* stop-to-reverse guard time before direction flip */

/* ========== Sensor Contract / Diagnostics ==========
 * Primary use: Core/Src/encoder_reader.c, Core/Src/adc_potentiometer.c,
 *              Core/Src/homing.c, Core/Src/app_runtime.c
 */
#define SENSOR_POSITIVE_STEERING_IS_CW                 1  /* +steering_deg corresponds to physical CW steering rotation */
#define SENSOR_POSITIVE_MOTOR_IS_CW                    1  /* +motor_deg corresponds to physical CW motor rotation */
#define ENCODER_COUNT_POLARITY                         1  /* +1: encoder count increase means +motor_deg, -1 flips sensor polarity */
#define ADC_POT_STEERING_POLARITY                      1  /* +1: increasing raw moves toward +steering_deg, -1 flips sensor polarity */
#define SENSOR_DIR_PIN_ONE_IS_CW        DIR_ACTIVE_HIGH_FOR_CW /* DIR GPIO level 1 physical direction contract */

#define ENCODER_SAMPLE_STALE_WARN_MS                20U  /* warning when encoder cache is older than this */
#define ENCODER_SAMPLE_STALE_FAULT_MS               50U  /* fault when encoder cache is older than this */
#define ADC_POT_SAMPLE_STALE_WARN_MS                20U  /* warning when ADC cache is older than this */
#define ADC_POT_SAMPLE_STALE_FAULT_MS               50U  /* fault when ADC cache is older than this */
#define ADC_POT_CONVERSION_TIMEOUT_MS               20U  /* timeout for non-blocking ADC conversion completion */

#define ENCODER_VELOCITY_WARN_STEERING_DPS          60.0f    /* steering-axis plausibility warning threshold */
#define ENCODER_VELOCITY_FAULT_STEERING_DPS        120.0f    /* steering-axis plausibility fault threshold */
#define ENCODER_ACCEL_WARN_STEERING_DPS2         50000.0f    /* steering-axis acceleration warning threshold */
#define ENCODER_ACCEL_FAULT_STEERING_DPS2       150000.0f    /* steering-axis acceleration fault threshold */

#define SENSOR_CROSSCHECK_WARN_STEERING_DEG         2.0f     /* runtime encoder-vs-ADC warning threshold */
#define SENSOR_CROSSCHECK_FAULT_STEERING_DEG        5.0f     /* runtime encoder-vs-ADC fault threshold */
#define SENSOR_HOMING_CROSSCHECK_FAULT_DEG          2.0f     /* homing acceptance threshold after encoder offset is applied */
#define SENSOR_CROSSCHECK_WARN_PERSIST_MS          20U       /* persistence window before runtime warning log */
#define SENSOR_CROSSCHECK_FAULT_PERSIST_MS         50U       /* persistence window before runtime fault trip */

#define SENSOR_DIRECTION_MIN_APPLIED_HZ          5000U       /* ignore direction plausibility below this command magnitude */
#define SENSOR_DIRECTION_MIN_STEERING_DELTA_DEG   0.0003f    /* about half an encoder-count on steering axis */
#define SENSOR_DIRECTION_WARN_PERSIST_MS           20U       /* mismatch persistence window before warning */
#define SENSOR_DIRECTION_FAULT_PERSIST_MS          75U       /* mismatch persistence window before fault */

#define ADC_POT_CALIBRATION_VERSION                 1U       /* persisted calibration format version */
#define ADC_POT_CALIBRATION_STORAGE_ENABLED         1        /* backup-SRAM persistence switch */

/* ========== Ethernet / UDP Integration ==========
 * Primary use: Core/Inc/ethernet_communication.h, Core/Src/ethernet_communication.c, Core/Src/app_runtime.c
 */
#define AUTODRIVE_UDP_PORT                5000U           /* UDP listen port for upper-controller packets */
#define ETHCOMM_RX_TIMEOUT_MS              300U           /* upper-controller receive timeout used by app runtime */
#define ETHCOMM_LOG_ENABLE                   0            /* verbose Ethernet/UDP parser log enable */

/* ========== Latency Profiler ==========
 * Primary use: Core/Inc/latency_profiler.h, Core/Src/latency_profiler.c, Core/Src/app_runtime.c
 */
#define LATENCY_PROFILER_ENABLE              1            /* compile-time enable for latency measurement */
#define LATENCY_LOG_ENABLE                   0            /* per-tick latency print enable */
#define LATENCY_MAX_SAMPLES               2048U           /* retained samples per stage before saturation */
#define LATENCY_AUTO_REPORT_ENABLE           1            /* automatic batch report emission enable */
#define LATENCY_AUTO_REPORT_SAMPLES       2000U           /* samples required before each auto-report batch */

/* ========== Runtime Sensor Supervision ==========
 * Primary use: Core/Src/app_runtime.c
 */
#define APP_RUNTIME_AUTO_HOME_ON_BOOT          1          /* perform ADC-backed homing before enabling control */
#define APP_RUNTIME_SENSOR_DIAG_ENABLE         1          /* enable runtime sensor supervision and logs */

#endif /* PROJECT_PARAMS_H */
