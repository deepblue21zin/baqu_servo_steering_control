#ifndef APP_RUNTIME_LIVE_DEBUG_H
#define APP_RUNTIME_LIVE_DEBUG_H

#include <stdint.h>

typedef struct {
    void (*request_control_enable)(const char *source);
    void (*set_keyboard_target_deg)(float target_deg);
    void (*set_periodic_csv_enabled)(uint8_t enabled, uint8_t print_header);
    uint8_t (*get_periodic_csv_enabled)(void);
} AppRuntimeLiveDebug_Hooks_t;

void AppRuntimeLiveDebug_Service(const AppRuntimeLiveDebug_Hooks_t *hooks);

#endif /* APP_RUNTIME_LIVE_DEBUG_H */
