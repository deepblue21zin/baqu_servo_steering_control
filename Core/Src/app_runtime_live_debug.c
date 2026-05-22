#include "app_runtime_live_debug.h"

#include "constants.h"
#include "debug_vars.h"
#include "encoder_reader.h"
#include "main.h"
#include "position_control.h"
#include "project_params.h"
#include "pulse_control.h"

#include <stdint.h>
#include <stdio.h>

#if APP_RUNTIME_LIVE_DEBUG_ENABLE
#if APP_RUNTIME_LIVE_PID_ENABLE
static uint8_t AppRuntimeLiveDebug_PidChanged(const PID_Params_t *lhs, const PID_Params_t *rhs)
{
    if ((lhs->Kp != rhs->Kp) ||
        (lhs->Ki != rhs->Ki) ||
        (lhs->Kd != rhs->Kd) ||
        (lhs->integral_limit != rhs->integral_limit) ||
        (lhs->output_limit != rhs->output_limit)) {
        return 1U;
    }

    return 0U;
}

static void AppRuntimeLiveDebug_ApplyPidFromDebugVars(void)
{
    PID_Params_t current = {0};
    PID_Params_t next = {0};
    int32_t stable_error_motor_mdeg = dbg_stable_error_mdeg;
    float stable_error_motor_deg = STABLE_ERROR_MOTOR_DEG;
    uint32_t stable_time_ms = dbg_stable_time_ms;

    if (dbg_pid_live_enable == 0U) {
        return;
    }

    PositionControl_GetPID(&current);
    next = current;

    if ((dbg_kp >= 0.0f) && (dbg_kp <= 3000.0f)) {
        next.Kp = dbg_kp;
    }
    if ((dbg_ki >= 0.0f) && (dbg_ki <= 1000.0f)) {
        next.Ki = dbg_ki;
    }
    if ((dbg_kd >= 0.0f) && (dbg_kd <= 1000.0f)) {
        next.Kd = dbg_kd;
    }
    if ((dbg_integral_limit >= 0.0f) && (dbg_integral_limit <= 100000.0f)) {
        next.integral_limit = dbg_integral_limit;
    }
    if ((dbg_output_limit >= 0.0f) && (dbg_output_limit <= (float)PULSECONTROL_MAX_FREQ_HZ)) {
        next.output_limit = dbg_output_limit;
    }

    if (AppRuntimeLiveDebug_PidChanged(&current, &next) != 0U) {
        PositionControl_SetPIDParams(&next);
    }

    if (stable_error_motor_mdeg > 0) {
        if (stable_error_motor_mdeg > 100000) {
            stable_error_motor_mdeg = 100000;
        }
        stable_error_motor_deg = ((float)stable_error_motor_mdeg) / 1000.0f;
    }
    if (stable_time_ms == 0U) {
        stable_time_ms = STABLE_TIME_MS;
    }
    if (stable_time_ms > 10000U) {
        stable_time_ms = 10000U;
    }

    PositionControl_SetStableCriteriaMotorDeg(stable_error_motor_deg, stable_time_ms);
}
#else
#define AppRuntimeLiveDebug_ApplyPidFromDebugVars() ((void)0)
#endif

static void AppRuntimeLiveDebug_ApplyZeroRequest(const AppRuntimeLiveDebug_Hooks_t *hooks)
{
    PositionControl_Disable();
    PulseControl_Stop();
    EncoderReader_Reset();

    PositionControl_Reset();
    dbg_target_steer_deg = 0.0f;
    (void)PositionControl_SetTargetSteeringDegWithSource(0.0f, CMD_SRC_SERVICE);

    if ((hooks != NULL) && (hooks->set_keyboard_target_deg != NULL)) {
        hooks->set_keyboard_target_deg(0.0f);
    }

    printf("[LIVE] zero set tim2=reset\r\n");
}
#endif

void AppRuntimeLiveDebug_Service(const AppRuntimeLiveDebug_Hooks_t *hooks)
{
#if APP_RUNTIME_LIVE_DEBUG_ENABLE
    static uint32_t prev_target_apply = 0U;

#if APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE
    if ((hooks != NULL) &&
        (hooks->set_periodic_csv_enabled != NULL) &&
        (hooks->get_periodic_csv_enabled != NULL)) {
        uint8_t requested_csv = (dbg_csv_log_enable != 0U) ? 1U : 0U;
        if (requested_csv != hooks->get_periodic_csv_enabled()) {
            hooks->set_periodic_csv_enabled(requested_csv, 1U);
            printf("[LIVE] csv log %s\r\n", (requested_csv != 0U) ? "enabled" : "disabled");
        }
    }
#else
    dbg_csv_log_enable = 0U;
#endif

#if APP_RUNTIME_TELEPLOT_ENABLE
    dbg_teleplot_enable = (dbg_teleplot_enable != 0U) ? 1U : 0U;
#else
    dbg_teleplot_enable = 0U;
#endif

#if APP_RUNTIME_ENCODER_DIAG_ENABLE
    dbg_encoder_diag_enable = (dbg_encoder_diag_enable != 0U) ? 1U : 0U;
#else
    dbg_encoder_diag_enable = 0U;
#endif

    AppRuntimeLiveDebug_ApplyPidFromDebugVars();

    if (dbg_zero_request != 0U) {
        dbg_zero_request = 0U;
        AppRuntimeLiveDebug_ApplyZeroRequest(hooks);
    }

    if (dbg_control_disable != 0U) {
        dbg_control_disable = 0U;
        PositionControl_Disable();
        printf("[LIVE] control disabled\r\n");
    }

    if (dbg_estop_request != 0U) {
        dbg_estop_request = 0U;
        PositionControl_EmergencyStop();
        printf("[LIVE] emergency stop\r\n");
    }

    if (dbg_control_enable != 0U) {
        dbg_control_enable = 0U;
        if ((hooks != NULL) && (hooks->request_control_enable != NULL)) {
            hooks->request_control_enable("live_expr");
        } else {
            PositionControl_Enable();
        }
    }

    if (dbg_target_apply != prev_target_apply) {
        float target = dbg_target_steer_deg;

        prev_target_apply = dbg_target_apply;

        if (target > MAX_STEERING_ANGLE) {
            target = MAX_STEERING_ANGLE;
        }
        if (target < MIN_STEERING_ANGLE) {
            target = MIN_STEERING_ANGLE;
        }

        dbg_target_steer_deg = target;
        if ((hooks != NULL) && (hooks->set_keyboard_target_deg != NULL)) {
            hooks->set_keyboard_target_deg(target);
        }

        if (PositionControl_SetTargetSteeringDegWithSource(target, CMD_SRC_SERVICE) == POS_CTRL_OK) {
            if ((hooks != NULL) && (hooks->request_control_enable != NULL)) {
                hooks->request_control_enable("live_target");
            } else {
                PositionControl_Enable();
            }
        }
    }
#else
    (void)hooks;
#endif
}
