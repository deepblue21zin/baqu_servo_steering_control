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

#endif /* PROJECT_PARAMS_H */
