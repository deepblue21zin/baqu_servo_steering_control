#ifndef PROJECT_PARAMS_H
#define PROJECT_PARAMS_H

/*
 * User-editable runtime tuning and bench switches.
 * Physical conversion constants such as gear ratio and degree/pulse stay in constants.h.
 */

/* ========== App Runtime / Bench Switches ========== */
#define APP_RUNTIME_AUTO_FIXED_PULSE_TEST        0
#define APP_RUNTIME_AUTO_FIXED_PULSE_HZ          500000
#define APP_RUNTIME_KEYBOARD_TEST_MODE           1
#define APP_RUNTIME_KEYBOARD_STEP_DEG            1.0f
#define APP_RUNTIME_ENCODER_DIAG_ENABLE          1
#define APP_RUNTIME_ENCODER_DIAG_PERIOD_MS       100U
#define APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE   0
#define APP_RUNTIME_AUTO_START_CONTROL_ENABLE    1
#define APP_RUNTIME_RESET_ENCODER_ON_BOOT        0
#define APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE      1
#define APP_RUNTIME_PERIODIC_CSV_LOG_PERIOD_MS   100U
#define APP_RUNTIME_PERIODIC_DIAG_DIVIDER        100U

/* ========== Encoder Reader ========== */
#define ENCODER_READER_FORCE_CENTER_ON_INIT      0

/* ========== Position Control ========== */
#define MAX_ANGLE_DEG                    4320.0f
#define MIN_ANGLE_DEG                   -4320.0f
#define MAX_TRACKING_ERROR_DEG           4500.0f
#define DEFAULT_KP                         50.0f
#define DEFAULT_KI                          5.0f
#define DEFAULT_KD                         20.0f
#define DEFAULT_INTEGRAL_LIMIT          1000.0f
#define DEFAULT_OUTPUT_LIMIT           10000.0f
#define STABLE_ERROR_THRESHOLD             0.5f
#define STABLE_TIME_MS                     100U
#define POSITION_COMMAND_TIMEOUT_MS          0U

/* ========== Position Safety ========== */
#define POSITION_SAFETY_ANGLE_MARGIN_DEG    5.0f

/* ========== Pulse Output ========== */
#define DIR_ACTIVE_HIGH_FOR_CW               1
#define PULSECONTROL_MIN_FREQ_HZ            10U
#define PULSECONTROL_MAX_FREQ_HZ        100000U
#define PULSECONTROL_DIRECTION_GUARD_MS      1U

#endif /* PROJECT_PARAMS_H */
