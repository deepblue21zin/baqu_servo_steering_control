#include "app_runtime_teleplot.h"

#include "constants.h"
#include "debug_vars.h"
#include "main.h"
#include "position_control.h"
#include "project_params.h"
#include "pulse_control.h"

#include <stdio.h>

void AppRuntimeTeleplot_Service(void)
{
#if APP_RUNTIME_TELEPLOT_ENABLE
    static uint32_t last_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    PositionControl_State_t state = {0};
    CommandLifecycle_t command = {0};
    PulseControl_Status_t pulse_status = {0};

    if (dbg_teleplot_enable == 0U) {
        return;
    }

    if ((uint32_t)(now_ms - last_ms) < APP_RUNTIME_TELEPLOT_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    state = PositionControl_GetState();
    command = PositionControl_GetCommandLifecycle();
    pulse_status = PulseControl_GetStatus();

    printf(">target:%.3f\r\n", state.target_steering_deg);
    printf(">current:%.3f\r\n", state.current_steering_deg);
    printf(">error:%.3f\r\n", state.error_steering_deg);
    printf(">output:%.0f\r\n", state.output_hz);
    printf(">req_hz:%ld\r\n", (long)pulse_status.requested_frequency_hz);
    printf(">applied_hz:%lu\r\n", (unsigned long)pulse_status.applied_frequency_hz);
    printf(">dir:%d\r\n", (int)pulse_status.direction);
    printf(">run:%u\r\n", (unsigned int)pulse_status.output_active);
    printf(">cmd_state:%d\r\n", (int)command.state);
#endif
}
