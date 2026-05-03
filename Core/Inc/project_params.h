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
#define APP_RUNTIME_KEYBOARD_AUTO_ENABLE_ON_TARGET 1      /* bench-only: target input also enables control */
#define APP_RUNTIME_EMERGENCY_LATCH_ENABLE       0        /* bench-only: 0 stops output without latching CTRL_MODE_EMERGENCY */
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

/* ========== CAN Encoder Bridge ==========
 * Primary use: Core/Src/can_encoder_bridge.c, Core/Src/usart.c
 */
#define CAN_ENCODER_BRIDGE_ENABLE                1        /* USART1 4-byte encoder stream -> CAN1 frame bridge */
#define CAN_ENCODER_UART_BAUDRATE            38400U       /* must match the servo drive RS422 encoder-port setting */
#define CAN_ENCODER_BRIDGE_STD_ID            0x100U       /* CAN standard ID used for encoder frames */
#define CAN_ENCODER_BRIDGE_MAX_BYTES_PER_SERVICE 16U      /* bounded non-blocking UART drain per super-loop pass */
#define CAN_ENCODER_BRIDGE_LOG_ENABLE            1        /* boot log for bridge readiness */

/* ========== RS422 Servo Encoder Reader ==========
 * Primary use: Core/Src/rs422_encoder_uart.c, Core/Src/usart.c
 */
#define RS422_ENCODER_READER_ENABLE              1        /* USART2 PA3 RX -> Putty hex/count/degree diagnostic */
#define RS422_ENCODER_UART_BAUDRATE          38400U       /* must match the servo CN4 RS422 setting */
#define RS422_ENCODER_FRAME_SIZE                 4U       /* default: 4-byte little-endian signed encoder count */
#define RS422_ENCODER_MAX_BYTES_PER_SERVICE     32U       /* bounded non-blocking UART drain per super-loop pass */
#define RS422_ENCODER_PRINT_EACH_FRAME           0        /* 1: print every decoded frame; use briefly only */
#define RS422_ENCODER_PRINT_FIRST_FRAMES        16U       /* first frames printed with HEX for bring-up */
#define RS422_ENCODER_PRINT_PERIOD_MS          100U       /* after first frames, print latest decoded value at this period */
#define RS422_ENCODER_IDLE_DUMP_MS              50U       /* dump partial HEX line when bytes stop arriving */
#define RS422_ENCODER_STATUS_LOG_PERIOD_MS    1000U       /* print bytes/frames counters while bringing up wiring */

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
#define POSITION_FAILSAFE_EXTRA_ENABLE       1            /* enable tracking/velocity/timeout fail-safe profile */
#define POSITION_FAILSAFE_PROFILE_PARAM_TEST 1U           /* tuning profile with relaxed nuisance-trip limits */
#define POSITION_FAILSAFE_PROFILE_VEHICLE_TEST 2U         /* vehicle/field-test profile with active safety limits */
#define POSITION_FAILSAFE_PROFILE            POSITION_FAILSAFE_PROFILE_PARAM_TEST
#if ((POSITION_FAILSAFE_PROFILE != POSITION_FAILSAFE_PROFILE_PARAM_TEST) && \
     (POSITION_FAILSAFE_PROFILE != POSITION_FAILSAFE_PROFILE_VEHICLE_TEST))
#error "POSITION_FAILSAFE_PROFILE must be PARAM_TEST or VEHICLE_TEST"
#endif
#define POSITION_FAILSAFE_PARAM_TEST_MAX_ERROR_DEG        MAX_TRACKING_ERROR_DEG
#define POSITION_FAILSAFE_PARAM_TEST_MAX_VELOCITY_DEG_PER_S 0.0f
#define POSITION_FAILSAFE_PARAM_TEST_TIMEOUT_MS           0U
#define POSITION_FAILSAFE_VEHICLE_TEST_MAX_ERROR_DEG      800.0f
#define POSITION_FAILSAFE_VEHICLE_TEST_MAX_VELOCITY_DEG_PER_S 450.0f
#define POSITION_FAILSAFE_VEHICLE_TEST_TIMEOUT_MS         5000U

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
#define SENSOR_DIRECTION_PLAUSIBILITY_ENABLE        0        /* bench-only: 0 disables command-vs-encoder direction checks */
#define SENSOR_DIRECTION_MIN_STEERING_DELTA_DEG   0.0003f    /* about half an encoder-count on steering axis */
#define SENSOR_DIRECTION_WARN_PERSIST_MS           20U       /* mismatch persistence window before warning */
#define SENSOR_DIRECTION_FAULT_PERSIST_MS          75U       /* mismatch persistence window before fault */

#define ADC_POT_CALIBRATION_VERSION                 1U       /* persisted calibration format version */
#define ADC_POT_CALIBRATION_STORAGE_ENABLED         1        /* backup-SRAM persistence switch */

/* ========== Ethernet / UDP Integration ==========
 * Primary use: Core/Inc/ethernet_communication.h, Core/Src/ethernet_communication.c, Core/Src/app_runtime.c
 */
#define ETHCOMM_ASMS_IP_LAST_OCTET            5U           /* sender IP x.x.x.5 for ASMS mode/joystick packets */
#define ETHCOMM_PC_IP_LAST_OCTET              1U           /* sender IP x.x.x.1 for PC steering packets */
#define ETHCOMM_ASMS_PACKET_SIZE              5U           /* ASMS packet length: mode + joy_x + joy_y */
#define ETHCOMM_PC_PACKET_SIZE                9U           /* PC packet length: steer + speed + misc */
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
#define APP_RUNTIME_ADC_POT_ENABLE             0          /* 0: encoder-only bench mode; disables ADC homing/crosscheck */
#define APP_RUNTIME_AUTO_HOME_ON_BOOT          0          /* perform ADC-backed homing before enabling control */
#define APP_RUNTIME_SENSOR_DIAG_ENABLE         1          /* enable runtime sensor supervision and logs */

#endif /* PROJECT_PARAMS_H */
