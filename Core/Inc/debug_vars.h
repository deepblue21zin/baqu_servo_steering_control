#ifndef DEBUG_VARS_H
#define DEBUG_VARS_H

#include <stdint.h>

extern volatile uint32_t dbg_enc_raw;
extern volatile int32_t  dbg_pos_mdeg;       /* steering milli-deg mirror for Live Expressions */
extern volatile int32_t  dbg_target_mdeg;    /* steering milli-deg mirror for Live Expressions */
extern volatile int32_t  dbg_err_mdeg;       /* steering milli-deg mirror for Live Expressions */
extern volatile int32_t  dbg_pwm_cmd;        /* signed output frequency in Hz */

extern volatile float dbg_kp;
extern volatile float dbg_ki;
extern volatile float dbg_kd;
extern volatile float dbg_integral_limit;
extern volatile float dbg_output_limit;
extern volatile uint8_t dbg_pid_live_enable;
extern volatile int32_t dbg_stable_error_mdeg; /* motor milli-deg stable threshold */
extern volatile uint32_t dbg_stable_time_ms;

extern volatile float dbg_target_steer_deg;
extern volatile uint32_t dbg_target_apply;
extern volatile uint8_t dbg_control_enable;
extern volatile uint8_t dbg_control_disable;
extern volatile uint8_t dbg_zero_request;
extern volatile uint8_t dbg_estop_request;

extern volatile uint8_t dbg_teleplot_enable;
extern volatile uint8_t dbg_csv_log_enable;
extern volatile uint8_t dbg_encoder_diag_enable;

#endif /* DEBUG_VARS_H */
