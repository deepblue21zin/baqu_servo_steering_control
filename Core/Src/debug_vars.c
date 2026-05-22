#include "debug_vars.h"
#include "project_params.h"

volatile uint32_t dbg_enc_raw = 0U;
volatile int32_t  dbg_pos_mdeg = 0;
volatile int32_t  dbg_target_mdeg = 0;
volatile int32_t  dbg_err_mdeg = 0;
volatile int32_t  dbg_pwm_cmd = 0;

volatile float dbg_kp = DEFAULT_KP;
volatile float dbg_ki = DEFAULT_KI;
volatile float dbg_kd = DEFAULT_KD;
volatile float dbg_integral_limit = DEFAULT_INTEGRAL_LIMIT;
volatile float dbg_output_limit = DEFAULT_OUTPUT_LIMIT;
volatile uint8_t dbg_pid_live_enable = APP_RUNTIME_LIVE_PID_ENABLE;
volatile int32_t dbg_stable_error_mdeg = (int32_t)(STABLE_ERROR_MOTOR_DEG * 1000.0f);
volatile uint32_t dbg_stable_time_ms = STABLE_TIME_MS;

volatile float dbg_target_steer_deg = 0.0f;
volatile uint32_t dbg_target_apply = 0U;
volatile uint8_t dbg_control_enable = 0U;
volatile uint8_t dbg_control_disable = 0U;
volatile uint8_t dbg_zero_request = 0U;
volatile uint8_t dbg_estop_request = 0U;

volatile uint8_t dbg_teleplot_enable = APP_RUNTIME_TELEPLOT_DEFAULT_ENABLE;
volatile uint8_t dbg_csv_log_enable = APP_RUNTIME_PERIODIC_CSV_LOG_DEFAULT_ENABLE;
volatile uint8_t dbg_encoder_diag_enable = APP_RUNTIME_ENCODER_DIAG_DEFAULT_ENABLE;
