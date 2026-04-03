#ifndef LATENCY_PROFILER_H
#define LATENCY_PROFILER_H

#include "project_params.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LAT_STAGE_SENSE = 0,
    LAT_STAGE_CONTROL = 1,
    LAT_STAGE_ACTUATE = 2,
    LAT_STAGE_COMMS = 3,
    LAT_STAGE_COUNT
} LatencyStage_t;

typedef struct {
    uint32_t sample_count;
    uint32_t avg_cycles;
    uint32_t p99_cycles;
    uint32_t max_cycles;
    float avg_us;
    float p99_us;
    float max_us;
} LatencyStageStats_t;

void LatencyProfiler_Init(uint32_t core_clock_hz);
void LatencyProfiler_Reset(void);

void LatencyProfiler_Begin(LatencyStage_t stage);
void LatencyProfiler_End(LatencyStage_t stage);

void LatencyProfiler_OnDeadlineTick(bool previous_tick_pending);
uint32_t LatencyProfiler_GetDeadlineMissCount(void);

bool LatencyProfiler_GetStageStats(LatencyStage_t stage, LatencyStageStats_t *out_stats);
uint32_t LatencyProfiler_GetStageSampleCount(LatencyStage_t stage);
const char *LatencyProfiler_StageName(LatencyStage_t stage);

#if LATENCY_PROFILER_ENABLE
#define LAT_BEGIN(stage) LatencyProfiler_Begin(stage)
#define LAT_END(stage) LatencyProfiler_End(stage)
#else
#define LAT_BEGIN(stage) ((void)(stage))
#define LAT_END(stage) ((void)(stage))
#endif

#endif /* LATENCY_PROFILER_H */
