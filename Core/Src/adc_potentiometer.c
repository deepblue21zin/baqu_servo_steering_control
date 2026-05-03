/**
 * @file adc_potentiometer.c
 * @brief ADC Potentiometer implementation
 */

#include "adc_potentiometer.h"

#include "adc.h"
#include "constants.h"
#include "project_params.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ADC_POT_CALIBRATION_MAGIC 0x504F5443UL /* "POTC" */

#if ADC_POT_CALIBRATION_STORAGE_ENABLED && defined(BKPSRAM_BASE)
#define ADC_POT_RUNTIME_STORAGE_ENABLED 1
#else
#define ADC_POT_RUNTIME_STORAGE_ENABLED 0
#endif

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint16_t min_raw;
    uint16_t max_raw;
    float min_angle_deg;
    float max_angle_deg;
    uint32_t checksum;
} ADC_PotCalibrationBlob_t;

typedef struct {
    uint16_t prev_raw;
    uint16_t same_count;
    uint16_t jump_threshold_raw;
    uint16_t stuck_threshold_count;
    uint16_t disconnect_low_raw;
    uint16_t disconnect_high_raw;
    uint16_t valid_min_raw;
    uint16_t valid_max_raw;
    uint16_t pending_min_raw;
    uint32_t conversion_start_tick_ms;
    uint32_t last_sample_tick_ms;
    uint32_t last_base_validity;
    ADC_PotSample_t latest_sample;
    ADC_PotCalibration_t calibration;
    ADC_PotCalibrationState_t calibration_state;
    float pending_min_angle_deg;
    float pending_max_angle_deg;
    uint8_t conversion_in_progress;
    uint8_t sample_valid;
    uint8_t initialized;
} ADC_PotDiag_t; /* Local ADC diagnostics, cached sample, and persistent calibration metadata. */

static ADC_PotConfig_t pot_config = {
    .hadc = NULL,
    .channel = ADC_CHANNEL_4,
    .min_angle = POT_MIN_ANGLE_DEG,
    .max_angle = POT_MAX_ANGLE_DEG,
    .min_raw = 0U,
    .max_raw = (uint16_t)ADC_MAX_COUNT
};

static ADC_PotDiag_t g_pot_diag = {
    .prev_raw = 0U,
    .same_count = 0U,
    .jump_threshold_raw = 128U,
    .stuck_threshold_count = 50U,
    .disconnect_low_raw = 1U,
    .disconnect_high_raw = 4094U,
    .valid_min_raw = 0U,
    .valid_max_raw = (uint16_t)ADC_MAX_COUNT,
    .pending_min_raw = 0U,
    .conversion_start_tick_ms = 0U,
    .last_sample_tick_ms = 0U,
    .last_base_validity = ADC_POT_INVALID_NOT_INIT,
    .latest_sample = {0},
    .calibration = {0},
    .calibration_state = ADC_POT_CAL_IDLE,
    .pending_min_angle_deg = POT_MIN_ANGLE_DEG,
    .pending_max_angle_deg = POT_MAX_ANGLE_DEG,
    .conversion_in_progress = 0U,
    .sample_valid = 0U,
    .initialized = 0U
};

#if ADC_POT_RUNTIME_STORAGE_ENABLED
static volatile ADC_PotCalibrationBlob_t * const g_pot_calibration_store =
    (volatile ADC_PotCalibrationBlob_t *)BKPSRAM_BASE;
#endif

static uint32_t ADC_Pot_CalculateChecksum(const ADC_PotCalibrationBlob_t *blob)
{
    const uint8_t *bytes = (const uint8_t *)blob;
    size_t i = 0U;
    uint32_t checksum = 2166136261UL;

    if (blob == NULL) {
        return 0U;
    }

    for (i = 0U; i < offsetof(ADC_PotCalibrationBlob_t, checksum); i++) {
        checksum ^= (uint32_t)bytes[i];
        checksum *= 16777619UL;
    }

    return checksum;
}

static void ADC_Pot_EnablePersistentStorage(void)
{
#if ADC_POT_RUNTIME_STORAGE_ENABLED
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    HAL_PWREx_EnableBkUpReg();
#endif
}

static void ADC_Pot_UpdateCalibrationMetadata(uint16_t min_raw,
                                              uint16_t max_raw,
                                              float min_angle_deg,
                                              float max_angle_deg,
                                              uint32_t checksum,
                                              uint8_t valid,
                                              uint8_t loaded_from_persistent_store)
{
    g_pot_diag.calibration.version = ADC_POT_CALIBRATION_VERSION;
    g_pot_diag.calibration.min_raw = min_raw;
    g_pot_diag.calibration.max_raw = max_raw;
    g_pot_diag.calibration.min_angle_deg = min_angle_deg;
    g_pot_diag.calibration.max_angle_deg = max_angle_deg;
    g_pot_diag.calibration.checksum = checksum;
    g_pot_diag.calibration.valid = valid;
    g_pot_diag.calibration.loaded_from_persistent_store = loaded_from_persistent_store;
}

static void ADC_Pot_ApplyCalibrationToConfig(uint16_t min_raw,
                                             uint16_t max_raw,
                                             float min_angle_deg,
                                             float max_angle_deg)
{
    pot_config.min_raw = min_raw;
    pot_config.max_raw = max_raw;
    pot_config.min_angle = min_angle_deg;
    pot_config.max_angle = max_angle_deg;
    g_pot_diag.valid_min_raw = min_raw;
    g_pot_diag.valid_max_raw = max_raw;
}

static float ADC_Pot_ConvertRawToAngle(uint16_t raw)
{
    float ratio = 0.0f;
    uint16_t span_raw = 0U;

    if (pot_config.max_raw <= pot_config.min_raw) {
        return pot_config.min_angle;
    }

    span_raw = pot_config.max_raw - pot_config.min_raw;
    if (ADC_POT_STEERING_POLARITY >= 0) {
        ratio = (float)((int32_t)raw - (int32_t)pot_config.min_raw) / (float)span_raw;
    } else {
        ratio = (float)((int32_t)pot_config.max_raw - (int32_t)raw) / (float)span_raw;
    }

    return pot_config.min_angle + (ratio * (pot_config.max_angle - pot_config.min_angle));
}

static uint32_t ADC_Pot_EvaluateBaseValidity(uint16_t raw)
{
    uint32_t flags = ADC_POT_VALID;
    uint16_t diff = (raw >= g_pot_diag.prev_raw) ? (raw - g_pot_diag.prev_raw) :
                    (g_pot_diag.prev_raw - raw);

    if (g_pot_diag.initialized == 0U) {
        flags |= ADC_POT_INVALID_NOT_INIT;
    }

    if ((raw <= g_pot_diag.disconnect_low_raw) || (raw >= g_pot_diag.disconnect_high_raw)) {
        flags |= ADC_POT_INVALID_DISCONNECT;
    } else if ((raw < g_pot_diag.valid_min_raw) || (raw > g_pot_diag.valid_max_raw)) {
        flags |= ADC_POT_INVALID_RANGE;
    }

    if ((g_pot_diag.sample_valid != 0U) && (diff > g_pot_diag.jump_threshold_raw)) {
        flags |= ADC_POT_INVALID_JUMP;
    }

    if ((g_pot_diag.sample_valid != 0U) && (raw == g_pot_diag.prev_raw)) {
        g_pot_diag.same_count++;
        if (g_pot_diag.same_count >= g_pot_diag.stuck_threshold_count) {
            flags |= ADC_POT_INVALID_STUCK;
        }
    } else {
        g_pot_diag.same_count = 0U;
    }

    g_pot_diag.prev_raw = raw;
    return flags;
}

static void ADC_Pot_FinalizeSample(uint16_t raw, uint32_t validity, uint32_t now_ms)
{
    g_pot_diag.latest_sample.raw = raw;
    g_pot_diag.latest_sample.voltage = ((float)raw * ADC_VREF) / ADC_MAX_COUNT;
    g_pot_diag.latest_sample.calibrated_angle_deg = ADC_Pot_ConvertRawToAngle(raw);
    g_pot_diag.latest_sample.sample_tick_ms = now_ms;
    g_pot_diag.latest_sample.age_ms = 0U;
    g_pot_diag.latest_sample.validity = validity;
    g_pot_diag.last_sample_tick_ms = now_ms;
    g_pot_diag.last_base_validity = validity;
    g_pot_diag.sample_valid = 1U;
}

static void ADC_Pot_LoadDefaultCalibration(void)
{
    ADC_Pot_ApplyCalibrationToConfig(0U,
                                     (uint16_t)ADC_MAX_COUNT,
                                     POT_MIN_ANGLE_DEG,
                                     POT_MAX_ANGLE_DEG);
    ADC_Pot_UpdateCalibrationMetadata(pot_config.min_raw,
                                      pot_config.max_raw,
                                      pot_config.min_angle,
                                      pot_config.max_angle,
                                      0U,
                                      0U,
                                      0U);
}

static uint8_t ADC_Pot_LoadPersistentCalibration(void)
{
#if ADC_POT_RUNTIME_STORAGE_ENABLED
    ADC_PotCalibrationBlob_t blob = {0};

    ADC_Pot_EnablePersistentStorage();
    memcpy(&blob, (const void *)g_pot_calibration_store, sizeof(blob));

    if ((blob.magic != ADC_POT_CALIBRATION_MAGIC) ||
        (blob.version != ADC_POT_CALIBRATION_VERSION) ||
        (blob.max_raw <= blob.min_raw)) {
        ADC_Pot_LoadDefaultCalibration();
        return 0U;
    }

    if (ADC_Pot_CalculateChecksum(&blob) != blob.checksum) {
        ADC_Pot_LoadDefaultCalibration();
        return 0U;
    }

    ADC_Pot_ApplyCalibrationToConfig(blob.min_raw,
                                     blob.max_raw,
                                     blob.min_angle_deg,
                                     blob.max_angle_deg);
    ADC_Pot_UpdateCalibrationMetadata(blob.min_raw,
                                      blob.max_raw,
                                      blob.min_angle_deg,
                                      blob.max_angle_deg,
                                      blob.checksum,
                                      1U,
                                      1U);
    return 1U;
#else
    ADC_Pot_LoadDefaultCalibration();
    return 0U;
#endif
}

int ADC_Pot_Init(ADC_PotConfig_t *config)
{
    uint8_t loaded_persistent = 0U;

    if (config != NULL) {
        pot_config = *config;
    }

    if (pot_config.hadc == NULL) {
        pot_config.hadc = &hadc1;
    }

    if ((config == NULL) || (config->max_raw <= config->min_raw)) {
        loaded_persistent = ADC_Pot_LoadPersistentCalibration();
    } else {
        ADC_Pot_ApplyCalibrationToConfig(config->min_raw,
                                         config->max_raw,
                                         config->min_angle,
                                         config->max_angle);
        ADC_Pot_UpdateCalibrationMetadata(config->min_raw,
                                          config->max_raw,
                                          config->min_angle,
                                          config->max_angle,
                                          0U,
                                          1U,
                                          0U);
    }

    g_pot_diag.latest_sample.validity = ADC_POT_INVALID_NOT_INIT;
    g_pot_diag.calibration_state = ADC_POT_CAL_IDLE;
    g_pot_diag.conversion_in_progress = 0U;
    g_pot_diag.sample_valid = 0U;
    g_pot_diag.initialized = 1U;

    ADC_Pot_Service();

    printf("[ADC] Initialized: calib=%s v%lu raw=[%u,%u] angle=[%.2f,%.2f]\r\n",
           (loaded_persistent != 0U) ? "restore" :
           (g_pot_diag.calibration.valid != 0U ? "config" : "default"),
           (unsigned long)g_pot_diag.calibration.version,
           (unsigned int)pot_config.min_raw,
           (unsigned int)pot_config.max_raw,
           pot_config.min_angle,
           pot_config.max_angle);
    return 0;
}

void ADC_Pot_Service(void)
{
    uint32_t now_ms = 0U;

    if ((g_pot_diag.initialized == 0U) || (pot_config.hadc == NULL)) {
        return;
    }

    now_ms = HAL_GetTick();

    if (g_pot_diag.conversion_in_progress == 0U) {
        if (HAL_ADC_Start(pot_config.hadc) == HAL_OK) {
            g_pot_diag.conversion_in_progress = 1U;
            g_pot_diag.conversion_start_tick_ms = now_ms;
        }
        return;
    }

    if (HAL_ADC_PollForConversion(pot_config.hadc, 0U) == HAL_OK) {
        uint16_t raw = (uint16_t)HAL_ADC_GetValue(pot_config.hadc);
        uint32_t validity = ADC_Pot_EvaluateBaseValidity(raw);

        g_pot_diag.conversion_in_progress = 0U;
        ADC_Pot_FinalizeSample(raw, validity, now_ms);
        return;
    }

    if ((now_ms - g_pot_diag.conversion_start_tick_ms) >= ADC_POT_CONVERSION_TIMEOUT_MS) {
        g_pot_diag.conversion_in_progress = 0U;
        g_pot_diag.latest_sample.validity = g_pot_diag.last_base_validity | ADC_POT_INVALID_TIMEOUT;
    }
}

uint16_t ADC_Pot_GetRaw(void)
{
    ADC_PotSample_t sample = {0};

    if (ADC_Pot_GetSample(&sample) == 0U) {
        return 0U;
    }

    return sample.raw;
}

float ADC_Pot_GetVoltage(void)
{
    ADC_PotSample_t sample = {0};

    if (ADC_Pot_GetSample(&sample) == 0U) {
        return 0.0f;
    }

    return sample.voltage;
}

float ADC_Pot_GetAngle(void)
{
    ADC_PotSample_t sample = {0};

    if (ADC_Pot_GetSample(&sample) == 0U) {
        return 0.0f;
    }

    return sample.calibrated_angle_deg;
}

uint8_t ADC_Pot_GetSample(ADC_PotSample_t *out_sample)
{
    uint32_t age_ms = 0U;
    uint32_t validity = 0U;

    if (out_sample == NULL) {
        return 0U;
    }

    if ((g_pot_diag.initialized == 0U) || (g_pot_diag.sample_valid == 0U)) {
        *out_sample = g_pot_diag.latest_sample;
        out_sample->validity |= ADC_POT_INVALID_NOT_INIT;
        return 0U;
    }

    *out_sample = g_pot_diag.latest_sample;
    age_ms = HAL_GetTick() - g_pot_diag.last_sample_tick_ms;
    validity = out_sample->validity;

    if (age_ms >= ADC_POT_SAMPLE_STALE_FAULT_MS) {
        validity |= ADC_POT_INVALID_TIMEOUT;
    }

    out_sample->age_ms = age_ms;
    out_sample->validity = validity;
    return 1U;
}

uint8_t ADC_Pot_GetCalibration(ADC_PotCalibration_t *out_calibration)
{
    if (out_calibration == NULL) {
        return 0U;
    }

    *out_calibration = g_pot_diag.calibration;
    return (g_pot_diag.calibration.valid != 0U) ? 1U : 0U;
}

ADC_PotCalibrationState_t ADC_Pot_GetCalibrationState(void)
{
    return g_pot_diag.calibration_state;
}

uint8_t ADC_Pot_SaveCalibration(uint16_t min_raw,
                                uint16_t max_raw,
                                float min_angle_deg,
                                float max_angle_deg)
{
    ADC_PotCalibrationBlob_t blob = {0};

    if (max_raw <= min_raw) {
        return 0U;
    }

    blob.magic = ADC_POT_CALIBRATION_MAGIC;
    blob.version = ADC_POT_CALIBRATION_VERSION;
    blob.min_raw = min_raw;
    blob.max_raw = max_raw;
    blob.min_angle_deg = min_angle_deg;
    blob.max_angle_deg = max_angle_deg;
    blob.checksum = ADC_Pot_CalculateChecksum(&blob);

#if ADC_POT_RUNTIME_STORAGE_ENABLED
    ADC_Pot_EnablePersistentStorage();
    g_pot_calibration_store->magic = blob.magic;
    g_pot_calibration_store->version = blob.version;
    g_pot_calibration_store->min_raw = blob.min_raw;
    g_pot_calibration_store->max_raw = blob.max_raw;
    g_pot_calibration_store->min_angle_deg = blob.min_angle_deg;
    g_pot_calibration_store->max_angle_deg = blob.max_angle_deg;
    g_pot_calibration_store->checksum = blob.checksum;
#endif

    ADC_Pot_ApplyCalibrationToConfig(min_raw,
                                     max_raw,
                                     min_angle_deg,
                                     max_angle_deg);
    ADC_Pot_UpdateCalibrationMetadata(min_raw,
                                      max_raw,
                                      min_angle_deg,
                                      max_angle_deg,
                                      blob.checksum,
                                      1U,
                                      0U);
    return 1U;
}

void ADC_Pot_Calibrate(float min_angle, float max_angle)
{
    ADC_PotSample_t sample = {0};

    if (ADC_Pot_GetSample(&sample) == 0U) {
        printf("[ADC] Calibration sample unavailable\r\n");
        return;
    }

    if (sample.validity != ADC_POT_VALID) {
        printf("[ADC] Calibration rejected: validity=0x%02lX\r\n", (unsigned long)sample.validity);
        return;
    }

    if (g_pot_diag.calibration_state == ADC_POT_CAL_IDLE) {
        g_pot_diag.pending_min_raw = sample.raw;
        g_pot_diag.pending_min_angle_deg = min_angle;
        g_pot_diag.pending_max_angle_deg = max_angle;
        g_pot_diag.calibration_state = ADC_POT_CAL_WAIT_HIGH_POINT;
        printf("[ADC] Calibration step1 captured raw=%u. Move to high point and call again.\r\n",
               (unsigned int)sample.raw);
        return;
    }

    if (ADC_Pot_SaveCalibration(g_pot_diag.pending_min_raw,
                                sample.raw,
                                g_pot_diag.pending_min_angle_deg,
                                g_pot_diag.pending_max_angle_deg) != 0U) {
        printf("[ADC] Calibration saved: raw=[%u,%u] checksum=0x%08lX\r\n",
               (unsigned int)g_pot_diag.pending_min_raw,
               (unsigned int)sample.raw,
               (unsigned long)g_pot_diag.calibration.checksum);
    } else {
        printf("[ADC] Calibration save failed: raw=[%u,%u]\r\n",
               (unsigned int)g_pot_diag.pending_min_raw,
               (unsigned int)sample.raw);
    }

    g_pot_diag.calibration_state = ADC_POT_CAL_IDLE;
}

uint8_t ADC_Pot_IsInitialized(void)
{
    return g_pot_diag.initialized;
}
