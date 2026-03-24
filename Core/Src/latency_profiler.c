#include "latency_profiler.h"
#include "main.h"

#include <stddef.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
    uint32_t start_cycle;
    uint32_t samples[LATENCY_MAX_SAMPLES];
    uint32_t sample_count;
    uint64_t sum_cycles;
    uint32_t max_cycles;
} LatencyStageBuffer_t;

static LatencyStageBuffer_t g_stage[LAT_STAGE_COUNT];
static uint32_t g_core_clock_hz = 0U;
static volatile uint32_t g_deadline_miss_count = 0U;
static uint32_t g_sort_buf[LATENCY_MAX_SAMPLES];

static int cmp_u32(const void *a, const void *b)
{
    const uint32_t va = *(const uint32_t *)a;
    const uint32_t vb = *(const uint32_t *)b;
    if (va < vb) {
        return -1;
    }
    if (va > vb) {
        return 1;
    }
    return 0;
}

static float cycles_to_us(uint32_t cycles)
{
    if (g_core_clock_hz == 0U) {
        return 0.0f;
    }
    return ((float)cycles * 1000000.0f) / (float)g_core_clock_hz;
}

static void dwt_cyccnt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void LatencyProfiler_Init(uint32_t core_clock_hz)
{
    g_core_clock_hz = core_clock_hz;
    LatencyProfiler_Reset();
    dwt_cyccnt_init();
}

void LatencyProfiler_Reset(void)
{
    memset(g_stage, 0, sizeof(g_stage));
    g_deadline_miss_count = 0U;
}

void LatencyProfiler_Begin(LatencyStage_t stage)
{
    if ((uint32_t)stage >= (uint32_t)LAT_STAGE_COUNT) {
        return;
    }
    g_stage[stage].start_cycle = DWT->CYCCNT;
}

void LatencyProfiler_End(LatencyStage_t stage)
{
    uint32_t dt = 0U;
    LatencyStageBuffer_t *buf = NULL;

    if ((uint32_t)stage >= (uint32_t)LAT_STAGE_COUNT) {
        return;
    }

    buf = &g_stage[stage];
    if (buf->sample_count >= LATENCY_MAX_SAMPLES) {
        return;
    }

    dt = DWT->CYCCNT - buf->start_cycle;
    buf->samples[buf->sample_count] = dt;
    buf->sample_count++;
    buf->sum_cycles += (uint64_t)dt;
    if (dt > buf->max_cycles) {
        buf->max_cycles = dt;
    }
}

void LatencyProfiler_OnDeadlineTick(bool previous_tick_pending)
{
    if (previous_tick_pending) {
        g_deadline_miss_count++;
    }
}

uint32_t LatencyProfiler_GetDeadlineMissCount(void)
{
    return g_deadline_miss_count;
}

bool LatencyProfiler_GetStageStats(LatencyStage_t stage, LatencyStageStats_t *out_stats)
{
    LatencyStageBuffer_t *buf = NULL;
    uint32_t keep_count = 0U;
    uint32_t p99_index = 0U;
    uint32_t p99_cycles = 0U;
    uint32_t avg_cycles = 0U;

    if ((uint32_t)stage >= (uint32_t)LAT_STAGE_COUNT || out_stats == NULL) {
        return false;
    }

    buf = &g_stage[stage];
    if (buf->sample_count == 0U) {
        memset(out_stats, 0, sizeof(*out_stats));
        return false;
    }

    keep_count = buf->sample_count;

    memcpy(g_sort_buf, buf->samples, keep_count * sizeof(uint32_t));
    qsort(g_sort_buf, keep_count, sizeof(uint32_t), cmp_u32);

    p99_index = (keep_count * 99U) / 100U;
    if (p99_index >= keep_count) {
        p99_index = keep_count - 1U;
    }
    p99_cycles = g_sort_buf[p99_index];

    avg_cycles = (uint32_t)(buf->sum_cycles / buf->sample_count);

    out_stats->sample_count = buf->sample_count;
    out_stats->avg_cycles = avg_cycles;
    out_stats->p99_cycles = p99_cycles;
    out_stats->max_cycles = buf->max_cycles;
    out_stats->avg_us = cycles_to_us(avg_cycles);
    out_stats->p99_us = cycles_to_us(p99_cycles);
    out_stats->max_us = cycles_to_us(buf->max_cycles);
    return true;
}

uint32_t LatencyProfiler_GetStageSampleCount(LatencyStage_t stage)
{
    if ((uint32_t)stage >= (uint32_t)LAT_STAGE_COUNT) {
        return 0U;
    }
    return g_stage[stage].sample_count;
}

const char *LatencyProfiler_StageName(LatencyStage_t stage)
{
    switch (stage) {
    case LAT_STAGE_SENSE:
        return "Sense";
    case LAT_STAGE_CONTROL:
        return "Control";
    case LAT_STAGE_ACTUATE:
        return "Actuate";
    case LAT_STAGE_COMMS:
        return "Comms";
    default:
        return "Unknown";
    }
}
