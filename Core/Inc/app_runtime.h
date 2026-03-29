#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

/* Initialize application-level modules after CubeMX peripheral setup. */
void AppRuntime_Init(void);

/* Run one super-loop iteration of the application runtime. */
void AppRuntime_RunIteration(void);

#endif /* APP_RUNTIME_H */
