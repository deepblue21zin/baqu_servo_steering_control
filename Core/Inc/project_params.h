#ifndef PROJECT_PARAMS_H
#define PROJECT_PARAMS_H

/*
 * Central place for user-editable tuning and bench switches.
 * Physical/mechanical conversion constants stay in constants.h.
 * Internal implementation constants stay local to each module.
 * Each section lists the primary code path that consumes the setting.
 */

/* ========== Active Runtime Profile ==========
 * Exactly one profile should be enabled.
 * TUNING: minimal keyboard scenario direction test, Teleplot enabled by default.
 * EVIDENCE_LOG: CSV/diagnostic logging-first evidence collection.
 * FIELD_SAFE: final vehicle-side behavior with stricter startup/safety choices.
 */
#define APP_RUNTIME_PROFILE_TUNING           1
#define APP_RUNTIME_PROFILE_EVIDENCE_LOG     0
#define APP_RUNTIME_PROFILE_FIELD_SAFE       0
#if ((APP_RUNTIME_PROFILE_TUNING + APP_RUNTIME_PROFILE_EVIDENCE_LOG + APP_RUNTIME_PROFILE_FIELD_SAFE) != 1)
#error "Enable exactly one APP_RUNTIME_PROFILE_* option"
#endif

/* ========== App Runtime / Bench Switches ==========
 * Primary use: Core/Src/app_runtime.c
 */
#define APP_RUNTIME_AUTO_FIXED_PULSE_TEST        0        /* bench-only fixed pulse mode in AppRuntime_ServiceFastTick() */
#define APP_RUNTIME_AUTO_FIXED_PULSE_HZ          500000   /* fixed pulse frequency when the bench mode above is enabled */
#define APP_RUNTIME_KEYBOARD_TEST_MODE           0        /* keyboard-local test path vs UDP path selection */
#define APP_RUNTIME_KEYBOARD_AUTO_ENABLE_ON_TARGET 1      /* bench-only: target input also enables control */
#define APP_RUNTIME_EMERGENCY_LATCH_ENABLE       0        /* bench-only: 0 stops output without latching CTRL_MODE_EMERGENCY */
#define APP_RUNTIME_KEYBOARD_STEP_DEG            1.0f     /* +/- steering step used by keyboard jog commands */
#define APP_RUNTIME_ENCODER_DIAG_ENABLE          0        /* direction-test: keep ENCDBG build output disabled */
#define APP_RUNTIME_ENCODER_DIAG_DEFAULT_ENABLE  APP_RUNTIME_PROFILE_EVIDENCE_LOG /* default diag logging follows evidence profile */
#define APP_RUNTIME_ENCODER_DIAG_PERIOD_MS       100U     /* encoder diagnostic report period */
#define APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE      0        /* direction-test: CSV logging disabled */
#define APP_RUNTIME_PERIODIC_CSV_LOG_DEFAULT_ENABLE APP_RUNTIME_PROFILE_EVIDENCE_LOG /* tuning keeps CSV off until requested */
#define APP_RUNTIME_PERIODIC_CSV_LOG_PERIOD_MS   100U     /* periodic CSV telemetry period */
#define APP_RUNTIME_TELEPLOT_ENABLE              1        /* Teleplot-compatible streaming telemetry */
#define APP_RUNTIME_TELEPLOT_DEFAULT_ENABLE      APP_RUNTIME_PROFILE_TUNING /* tuning default: stream target/current/error traces */
#define APP_RUNTIME_TELEPLOT_PERIOD_MS          50U       /* direction-test: moderate Teleplot report period */
#define APP_RUNTIME_LIVE_DEBUG_ENABLE            1        /* direction-test: keyboard path only */
#define APP_RUNTIME_LIVE_PID_ENABLE              1        /* direction-test: fixed PID defaults */
#define APP_RUNTIME_IWDG_ENABLE                  0        /* direction-test: disable watchdog while using debugger/breakpoints */
#define APP_RUNTIME_PERIODIC_DIAG_DIVIDER        100U     /* 1 ms loop divider for slow periodic diagnostics */
#define APP_RUNTIME_KEYBOARD_SCENARIO_ENABLE     1        /* key '1' runs the baseline keyboard scenario */
#define APP_RUNTIME_KEYBOARD_SCENARIO_DWELL_MS 500U       /* pause between automatic scenario targets */
#define APP_RUNTIME_KEYBOARD_SCENARIO_STEP_TIMEOUT_MS 30000U /* per-step timeout for automatic scenario targets */

/* ========== App Runtime / Boot Sequence ==========
 * Primary use: Core/Src/app_runtime.c::AppRuntime_Init()
 */
#define APP_RUNTIME_AUTO_START_CONTROL_ENABLE    0        /* bench-test: wait for keyboard target after boot zero */
#define APP_RUNTIME_RESET_ENCODER_ON_BOOT        1        /* zero the logical encoder origin at the current boot position */

/* ========== Position Control ==========
 * Primary use: Core/Inc/position_control.h, Core/Src/position_control.c
 */
#define MAX_MOTOR_ANGLE_DEG               562.5f          /* motor-axis soft limit: +/-45 steering deg * 12.5 gear ratio */
#define MIN_MOTOR_ANGLE_DEG              -562.5f          /* motor-axis soft limit: +/-45 steering deg * 12.5 gear ratio */
#define MAX_TRACKING_ERROR_MOTOR_DEG     4500.0f          /* tracking-error fault threshold in motor deg */
#define MAX_ANGLE_DEG                    MAX_MOTOR_ANGLE_DEG /* legacy alias; use MAX_MOTOR_ANGLE_DEG in new code */
#define MIN_ANGLE_DEG                    MIN_MOTOR_ANGLE_DEG /* legacy alias; use MIN_MOTOR_ANGLE_DEG in new code */
#define MAX_TRACKING_ERROR_DEG           MAX_TRACKING_ERROR_MOTOR_DEG /* legacy alias */
#define DEFAULT_KP                         200.0f          /* direction-verified conservative response gain */
#define DEFAULT_KI                           0.0f          /* keep integral off while response/direction is being tuned */
#define DEFAULT_KD                          20.0f          /* closed-loop derivative gain baseline */
#define DEFAULT_D_FILTER_ALPHA               0.85f         /* 0=no filter, closer to 1=more D-term smoothing */
#define DEFAULT_INTEGRAL_LIMIT            500.0f          /* PID integrator clamp */
#define DEFAULT_OUTPUT_LIMIT            100000.0f          /* high-speed bench clamp; raise live only after direction is stable */
#define STABLE_ERROR_MOTOR_DEG              2.5f          /* motor deg; about +/-0.2 steering deg with 12.5:1 gear */
#define STABLE_ERROR_THRESHOLD             STABLE_ERROR_MOTOR_DEG /* legacy alias */
#define STABLE_TIME_MS                     100U           /* time inside the in-position band before CMD_REACHED */
#define POSITION_HOLD_REARM_ERROR_MOTOR_DEG (STABLE_ERROR_MOTOR_DEG * 2.0f) /* hold re-control threshold in motor deg */
#define POSITION_OUTPUT_SLEW_HZ_PER_S 10000000.0f         /* signed pulse-Hz ramp limit; 0 disables shaping */
#define POSITION_COMMAND_TIMEOUT_MS          0U           /* command watchdog; 0 disables timeout */

/* ========== Position Safety ==========
 * Primary use: Core/Src/position_control_safety.c
 */
#define POSITION_CONTROL_SAFETY_ENABLE       0            /* direction-test only: bypass safety trips while checking polarity */
#define POSITION_SAFETY_ANGLE_MARGIN_DEG    5.0f          /* extra angle margin outside MAX/MIN_ANGLE_DEG before fault */
#define POSITION_FAILSAFE_EXTRA_ENABLE       0            /* direction-test: no tracking/velocity/timeout trips */
#define POSITION_FAILSAFE_PROFILE_PARAM_TEST 1U           /* tuning profile with relaxed nuisance-trip limits */
#define POSITION_FAILSAFE_PROFILE_VEHICLE_TEST 2U         /* vehicle/field-test profile with active safety limits */
#define POSITION_FAILSAFE_PROFILE            POSITION_FAILSAFE_PROFILE_PARAM_TEST
#if ((POSITION_FAILSAFE_PROFILE != POSITION_FAILSAFE_PROFILE_PARAM_TEST) && \
     (POSITION_FAILSAFE_PROFILE != POSITION_FAILSAFE_PROFILE_VEHICLE_TEST))
#error "POSITION_FAILSAFE_PROFILE must be PARAM_TEST or VEHICLE_TEST"
#endif
#define POSITION_FAILSAFE_PARAM_TEST_MAX_ERROR_MOTOR_DEG  MAX_TRACKING_ERROR_MOTOR_DEG
#define POSITION_FAILSAFE_PARAM_TEST_MAX_VELOCITY_MOTOR_DEG_PER_S 0.0f
#define POSITION_FAILSAFE_PARAM_TEST_TIMEOUT_MS           0U
#define POSITION_FAILSAFE_VEHICLE_TEST_MAX_ERROR_MOTOR_DEG 800.0f
#define POSITION_FAILSAFE_VEHICLE_TEST_MAX_VELOCITY_MOTOR_DEG_PER_S 450.0f
#define POSITION_FAILSAFE_VEHICLE_TEST_TIMEOUT_MS         5000U

/* ========== Pulse Output ==========
 * Primary use: Core/Inc/pulse_control.h, Core/Src/pulse_control.c
 */
#define DIR_ACTIVE_HIGH_FOR_CW               0            /* direction pin polarity for CW rotation */
#define PULSECONTROL_MIN_FREQ_HZ            10U           /* non-zero pulse clamp to avoid too-slow pulse output */
#define PULSECONTROL_MAX_FREQ_HZ       1000000U           /* firmware-side pulse clamp for TIM1 generation */
#define PULSECONTROL_DIRECTION_GUARD_MS     20U           /* stop-to-reverse guard time before direction flip */
#define PULSECONTROL_RAMP_HZ_PER_S    10000000U           /* pulse-frequency slew limit; 0 disables ramp shaping */

/* ========== Encoder Contract / Diagnostics ==========
 * Primary use: Core/Src/encoder_reader.c, Core/Src/encoder_diag.c, Core/Src/app_runtime.c
 */
#define SENSOR_POSITIVE_STEERING_IS_CW                 1  /* +steering_deg corresponds to physical CW steering rotation */
#define SENSOR_POSITIVE_MOTOR_IS_CW                    1  /* +motor_deg corresponds to physical CW motor rotation */
#define ENCODER_COUNT_POLARITY                        -1  /* +1: encoder count increase means +motor_deg, -1 flips sensor polarity */
#define SENSOR_DIR_PIN_ONE_IS_CW        DIR_ACTIVE_HIGH_FOR_CW /* DIR GPIO level 1 physical direction contract */
#if ((ENCODER_COUNT_POLARITY != 1) && (ENCODER_COUNT_POLARITY != -1))
#error "ENCODER_COUNT_POLARITY must be +1 or -1"
#endif

#define ENCODER_SAMPLE_STALE_WARN_MS                20U  /* warning when encoder cache is older than this */
#define ENCODER_SAMPLE_STALE_FAULT_MS               50U  /* fault when encoder cache is older than this */

#define ENCODER_VELOCITY_WARN_STEERING_DPS          60.0f    /* steering-axis plausibility warning threshold */
#define ENCODER_VELOCITY_FAULT_STEERING_DPS        120.0f    /* steering-axis plausibility fault threshold */
#define ENCODER_ACCEL_WARN_STEERING_DPS2         50000.0f    /* steering-axis acceleration warning threshold */
#define ENCODER_ACCEL_FAULT_STEERING_DPS2       150000.0f    /* steering-axis acceleration fault threshold */

/* ========== Ethernet / UDP Integration ==========
 * Primary use: Core/Inc/ethernet_communication.h, Core/Src/ethernet_communication.c, Core/Src/app_runtime.c
 */
#define ETHCOMM_ASMS_IP_LAST_OCTET            5U           /* sender IP x.x.x.5 for ASMS mode/joystick packets */
#define ETHCOMM_PC_IP_LAST_OCTET              1U           /* sender IP x.x.x.1 for PC steering packets */
#define ETHCOMM_ASMS_PACKET_SIZE              5U           /* ASMS packet length: mode + joy_x + joy_y */
#define ETHCOMM_PC_PACKET_SIZE                9U           /* PC packet length: steer + speed + misc */
#define AUTODRIVE_UDP_PORT                 5000U           /* UDP listen port for upper-controller packets */
#define ETHCOMM_RX_TIMEOUT_MS               300U           /* upper-controller receive timeout used by app runtime */
#define ETHCOMM_JOY_Y_POLARITY               -1            /* -1: invert joystick Y so positive input commands the opposite steering sign */
#define ETHCOMM_JOY_Y_MAX_STEERING_DEG      45.0f          /* full-scale joystick Y maps to +/- steering angle */
#define ETHCOMM_JOY_Y_DEADBAND_RAW          40             /* ignore small joystick center noise */
#define ETHCOMM_LOG_ENABLE                    0            /* verbose Ethernet/UDP parser log enable */
#if ((ETHCOMM_JOY_Y_POLARITY != 1) && (ETHCOMM_JOY_Y_POLARITY != -1))
#error "ETHCOMM_JOY_Y_POLARITY must be +1 or -1"
#endif

/* ========== Latency Profiler ==========
 * Primary use: Core/Inc/latency_profiler.h, Core/Src/latency_profiler.c, Core/Src/app_runtime.c
 */
#define LATENCY_PROFILER_ENABLE              0             /* direction-test: latency profiling disabled */
#define LATENCY_LOG_ENABLE                   0             /* per-tick latency print enable */
#define LATENCY_MAX_SAMPLES               2048U            /* retained samples per stage before saturation */
#define LATENCY_AUTO_REPORT_ENABLE           0             /* direction-test: no automatic latency reports */
#define LATENCY_AUTO_REPORT_SAMPLES       2000U            /* samples required before each auto-report batch */

#endif /* PROJECT_PARAMS_H */
