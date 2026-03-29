# 팀원 C 문서

분석 기준일: 2026-03-14

담당 범위: `adc_potentiometer`, `encoder_reader`

분석 기준:
- 실제 구현은 `Core/Src/encoder_reader.c`, `Core/Src/adc_potentiometer.c`, `Core/Src/homing.c`, `Core/Src/position_control.c`, `Core/Src/tim.c`, `Core/Src/adc.c`를 기준으로 봤다.
- C 파트는 현재 구조에서 센서 계층 전체를 맡는다고 보는 것이 맞다.

## 1. 전체 아키텍처에서 C 파트의 위치

현재 센서 데이터 흐름은 아래와 같다.

1. TIM4가 엔코더 A/B상을 하드웨어 카운트한다.
2. `encoder_reader.c`가 TIM4 counter를 읽어 angle로 환산한다.
3. `position_control.c`가 그 값을 현재각으로 사용한다.
4. ADC1 CH4가 포텐셔미터 raw 값을 읽는다.
5. `adc_potentiometer.c`가 raw를 angle로 환산한다.
6. `homing.c`가 ADC angle을 이용해 encoder offset을 맞춘다.

현재 실동작 기준으로는:
- 엔코더는 runtime에서 실제 사용 중
- ADC는 코드상 존재하지만 main runtime에서는 사실상 미사용

즉 C 파트는 일부는 실제 제어 루프에 들어가 있고, 일부는 아직 준비 단계다.

## 2. 지금 서로 주고받는 정보와 형식

| Producer | Consumer | 항목 | 형식 | 현재 의미 |
|---|---|---|---|---|
| TIM4 | `encoder_reader.c` | raw counter | `uint16_t` | 0~65535 |
| `encoder_reader.c` | `position_control.c` | current angle | `float` | `deg`로 환산된 현재값 |
| `encoder_reader.c` | `homing.c` | offset apply | `int32_t` | 엔코더 영점 보정 |
| ADC1 | `adc_potentiometer.c` | raw value | `uint16_t` | 0~4095 |
| `adc_potentiometer.c` | `homing.c` | pot angle | `float` | 절대 위치 추정값 |

현재 인터페이스의 핵심 문제는 아래 두 가지다.

1. 엔코더는 `float angle`만 바로 소비되고, `raw_count`, `accum_count`, wrap 상태가 외부에 드러나지 않는다.
2. ADC는 `raw`와 `angle`은 있지만, 유효성 진단과 calibration 상태가 없다.

## 3. 현재 코드 구현 상세 분석

### 3.1 encoder_reader.c

현재 구현은 TIM4 16비트 counter를 중앙값 `32768` 기준으로 해석한다.

동작 방식:

1. `EncoderReader_Init()`
- counter를 `32768`로 맞춤
- `initialized = 1`

2. `EncoderReader_GetAngleDeg()`
- `raw = __HAL_TIM_GET_COUNTER(&htim4)`
- `encoder_count = raw - 32768`
- `adjusted_count = encoder_count - encoder_offset`
- `angle = adjusted_count * 360 / 48000`

장점:
- TIM2에서 TIM4로 바뀐 후에도 최소 동작은 단순하고 안정적이다.
- 영점 offset 개념이 있어 homing과 연결 가능하다.

핵심 한계:
- 16비트 counter wrap을 누적하지 않는다.
- 유효 범위는 `-32768 ~ +32767 count`, 즉 약 `-245.76 ~ +245.75 deg` 정도다.
- 그런데 position control은 `+/-4320 deg`를 허용하므로, 장시간 다회전 기준과 맞지 않는다.

즉 현재 엔코더 계층은 "짧은 범위 테스트"에는 쓸 수 있지만 "다회전 운용 제어기" 기준으로는 부족하다.

### 3.2 adc_potentiometer.c

현재 ADC 모듈은 아래 수준이다.

- `ADC_Pot_Init()`: 기본 config를 잡고 `HAL_ADC_Start`
- `ADC_Pot_GetRaw()`: software start 후 poll
- `ADC_Pot_GetVoltage()`: raw -> voltage
- `ADC_Pot_GetAngle()`: raw -> angle 선형 환산
- `ADC_Pot_Calibrate()`: 3초 delay 후 2점 보정

장점:
- API가 단순해서 사용은 쉽다.
- raw/voltage/angle을 분리한 방향은 맞다.

핵심 한계:
- `HAL_ADC_PollForConversion(..., 100)`은 최대 100ms blocking이다.
- runtime 실시간 경로에서 그대로 쓰기엔 부담이 크다.
- disconnect, jump, stuck, saturation 같은 센서 이상 진단이 없다.
- calibration이 RAM에만 있고 재부팅 후 유지되지 않는다.
- main runtime에서 실제 init/사용이 빠져 있다.

### 3.3 homing과의 연동

`homing.c`는 ADC angle을 바로 encoder offset으로 바꾼다.

문제는 아래와 같다.

- pot angle의 물리적 의미가 조향축 각인지 모터축 각인지 명확하지 않다.
- `offset_count = pot_angle * 48000 / 360`은 encoder와 ADC 기준축이 같다는 가정을 쓴다.
- gear ratio, 설치 위치, linkage ratio가 반영돼 있지 않다.

즉 C 파트는 단순 센서 읽기 이상의 "센서 의미 정의"를 맡아야 한다.

### 3.4 상수 관리 문제

`constants.h`에는 아래 문제가 있다.

- include guard가 없다.
- `POSITION_TOLERANCE`가 `position_control.h`와 중복된다.
- `MAX_STEERING_ANGLE +/-45`와 `MAX_ANGLE_DEG +/-4320`가 따로 논다.
- `MAX_PULSE_FREQ = 1000000`인데 실제 pulse 모듈 clamp는 `100000`이다.

센서 파트에서 단위와 변환상수를 정리하지 않으면 팀 전체가 같은 문제를 반복한다.

## 4. 현재 평가

성숙도 기준:
- L1: 센서 읽기 골격
- L2: 단기 bench 사용 가능
- L3: 장시간/다회전 운용 가능
- L4: sensor fault와 cross-check까지 검증
- L5: 현업형 sensor pipeline으로 설명 가능

현재 성숙도: `L2`

평가 근거:
- 엔코더는 짧은 범위에서는 사용 가능하다.
- ADC는 API는 있지만 runtime 통합과 진단이 부족하다.
- cross-check와 신뢰도 판단이 아직 없다.

자율주행 조향모터의 현업 기준으로 보면 아래 항목이 더 필요하다.

- fail-safe 완성도:
  - 센서 freshness, implausible velocity, command-sign mismatch를 감지하지 않는다.
  - 센서 fault가 언제 safe state로 이어지는지 bounded-time 계약이 없다.
  - homing 기준축 정의가 모호해 잘못된 offset이 조용히 들어갈 수 있다.
- debug 완성도:
  - sample timestamp, raw/calibrated pair, calibration version, health state가 남지 않는다.
  - 센서 fault 발생 시 직전 raw 데이터와 비교값을 남기는 진단 버퍼가 없다.

한 줄 평가:
- "센서를 읽는 기능은 있지만, 센서를 신뢰할 수 있게 만드는 기능은 아직 부족한 상태"

## 5. C 파트 REQ 초안

- `REQ-C-001`: 엔코더는 16비트 wrap을 고려한 `accum_count`를 제공해야 한다.
<!--
구현 방향:
- TIM4 raw counter의 직전값을 저장하고, 현재 raw와의 차이를 int16_t로 해석해 wrap을 자동 보정한다.
- 누적값은 int32_t 또는 int64_t accum_count로 별도 유지한다.

예시 코드:
typedef struct {
    uint16_t prev_raw_count;
    int32_t delta_count;
    int64_t accum_count;
    int32_t offset_count;
} EncoderReader_State_t;

static EncoderReader_State_t g_encoder;

static void EncoderReader_UpdateCounts(void)
{
    uint16_t raw = __HAL_TIM_GET_COUNTER(&htim4);
    int16_t signed_delta = (int16_t)(raw - g_encoder.prev_raw_count);

    g_encoder.delta_count = (int32_t)signed_delta;
    g_encoder.accum_count += (int32_t)signed_delta;
    g_encoder.prev_raw_count = raw;
}
-->
- `REQ-C-002`: 엔코더 API는 최소한 `raw_count`, `delta_count`, `accum_count`, `motor_deg`를 설명 가능해야 한다.
<!--
구현 방향:
- angle만 반환하지 말고 샘플 구조체 전체를 조회하는 API를 만든다.
- PositionControl은 motor_deg만 써도 되지만, 진단과 로그는 raw/delta/accum을 함께 본다.

예시 코드:
typedef struct {
    uint16_t raw_count;
    int32_t delta_count;
    int64_t accum_count;
    float motor_deg;
    uint32_t age_ms;
    uint32_t sample_tick_ms;
} EncoderSample_t;

bool EncoderReader_GetSample(EncoderSample_t *out_sample)
{
    if ((out_sample == NULL) || (g_encoder.initialized == 0U)) {
        return false;
    }

    out_sample->raw_count = g_encoder.raw_count;
    out_sample->delta_count = g_encoder.delta_count;
    out_sample->accum_count = g_encoder.accum_count - g_encoder.offset_count;
    out_sample->motor_deg = (float)(out_sample->accum_count) * ENCODER_DEG_PER_COUNT;
    out_sample->sample_tick_ms = g_encoder.sample_tick_ms;
    out_sample->age_ms = HAL_GetTick() - g_encoder.sample_tick_ms;
    return true;
}
-->
- `REQ-C-003`: ADC API는 `raw`, `voltage`, `calibrated_angle`, `validity`를 함께 제공해야 한다.
<!--
구현 방향:
- ADC도 단순 getter 여러 개 대신 샘플 구조체를 만들고 validity/fault bits를 포함한다.
- angle은 calibration 파라미터를 거친 calibrated_angle으로 명시한다.

예시 코드:
typedef enum {
    ADC_POT_VALID = 0x00,
    ADC_POT_INVALID_NOT_INIT = 0x01,
    ADC_POT_INVALID_DISCONNECT = 0x02,
    ADC_POT_INVALID_TIMEOUT = 0x04,
    ADC_POT_INVALID_STUCK = 0x08,
    ADC_POT_INVALID_JUMP = 0x10,
    ADC_POT_INVALID_RANGE = 0x20
} ADC_PotValidity_t;

typedef struct {
    uint16_t raw;
    float voltage;
    float calibrated_angle_deg;
    uint32_t sample_tick_ms;
    uint32_t age_ms;
    uint32_t validity;
} ADC_PotSample_t;

bool ADC_Pot_GetSample(ADC_PotSample_t *out_sample)
{
    uint16_t raw = 0U;

    if ((out_sample == NULL) || !ADC_Pot_GetRawNonBlocking(&raw)) {
        return false;
    }

    out_sample->raw = raw;
    out_sample->voltage = (float)raw * ADC_VREF / ADC_MAX_COUNT;
    out_sample->calibrated_angle_deg = ADC_Pot_ConvertRawToAngle(raw);
    out_sample->sample_tick_ms = HAL_GetTick();
    out_sample->age_ms = 0U;
    out_sample->validity = ADC_Pot_EvaluateValidity(raw);
    return (out_sample->validity == ADC_POT_VALID);
}
-->
- `REQ-C-004`: ADC는 disconnect, out-of-range, jump, stuck fault를 진단해야 한다.
<!--
구현 방향:
- 최근 raw/history를 저장하고 임계값 기반 fault를 누적 판정한다.
- 단발성 노이즈와 실제 fault를 구분하려면 연속 N회 조건을 둔다.

예시 코드:
typedef struct {
    uint16_t prev_raw;
    uint16_t same_count;
    uint16_t jump_count;
    uint16_t jump_threshold_raw;
    uint16_t jump_persist_count;
    uint16_t stuck_threshold_count;
    uint16_t disconnect_low_raw;
    uint16_t disconnect_high_raw;
    uint16_t valid_min_raw;
    uint16_t valid_max_raw;
} ADC_PotDiag_t;

static uint32_t ADC_Pot_EvaluateValidity(uint16_t raw)
{
    uint32_t flags = ADC_POT_VALID;
    uint16_t diff = (raw > g_pot_diag.prev_raw) ? (raw - g_pot_diag.prev_raw) : (g_pot_diag.prev_raw - raw);

    if ((raw <= g_pot_diag.disconnect_low_raw) || (raw >= g_pot_diag.disconnect_high_raw)) {
        flags |= ADC_POT_INVALID_DISCONNECT;
    } else if ((raw < g_pot_diag.valid_min_raw) || (raw > g_pot_diag.valid_max_raw)) {
        flags |= ADC_POT_INVALID_RANGE;
    }
    if (diff > g_pot_diag.jump_threshold_raw) {
        if (++g_pot_diag.jump_count >= g_pot_diag.jump_persist_count) {
            flags |= ADC_POT_INVALID_JUMP;
        }
    } else {
        g_pot_diag.jump_count = 0U;
    }
    if (raw == g_pot_diag.prev_raw) {
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
-->
- `REQ-C-005`: 엔코더와 ADC는 homing 시점과 runtime 시점에 각각 어떤 센서를 기준으로 삼는지 정책이 있어야 한다.
<!--
구현 방향:
- homing과 runtime의 기준센서를 enum으로 고정하고, 상태머신에서만 선택한다.
- 예: homing은 ADC absolute 기준, runtime은 encoder tracking 기준, fault 시 ADC cross-check만 사용.

예시 코드:
typedef enum {
    SENSOR_REF_NONE = 0,
    SENSOR_REF_ADC_ABSOLUTE,
    SENSOR_REF_ENCODER_INCREMENTAL
} SensorReference_t;

typedef struct {
    SensorReference_t homing_reference;
    SensorReference_t runtime_reference;
    SensorReference_t fallback_reference;
} SensorPolicy_t;

static const SensorPolicy_t g_sensor_policy = {
    .homing_reference = SENSOR_REF_ADC_ABSOLUTE,
    .runtime_reference = SENSOR_REF_ENCODER_INCREMENTAL,
    .fallback_reference = SENSOR_REF_ADC_ABSOLUTE
};
-->
- `REQ-C-006`: gear ratio, deg per count, pulse per deg, pot angle range는 단일 상수 헤더에서 관리해야 한다.
<!--
구현 방향:
- constants.h 또는 sensor_constants.h에 센서/구동/축변환 상수를 일원화한다.
- adc_potentiometer.c 내부 기본값(-90~90) 같은 매직넘버를 제거한다.

예시 코드:
#ifndef SENSOR_CONSTANTS_H
#define SENSOR_CONSTANTS_H

#define POT_MIN_ANGLE_DEG             (-45.0f)
#define POT_MAX_ANGLE_DEG             (45.0f)
#define ADC_VREF                      3.3f
#define ADC_MAX_COUNT                 4095.0f

#endif
-->
- `REQ-C-007`: 센서 API는 blocking call이 1ms 제어 루프를 방해하지 않도록 설계돼야 한다.
<!--
구현 방향:
- HAL_ADC_PollForConversion(..., 100)와 HAL_Delay(3000)는 제어 루프 경로에서 제거해야 한다.
- ADC는 DMA/interrupt/background start 방식으로 최신 샘플만 읽고, calibration은 setup 상태에서만 수행한다.

예시 코드:
static volatile uint16_t g_adc_latest_raw;
static volatile uint8_t g_adc_data_ready;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) {
        g_adc_latest_raw = (uint16_t)HAL_ADC_GetValue(hadc);
        g_adc_data_ready = 1U;
    }
}

bool ADC_Pot_GetRawNonBlocking(uint16_t *out_raw)
{
    if ((out_raw == NULL) || (g_adc_data_ready == 0U)) {
        return false;
    }
    *out_raw = g_adc_latest_raw;
    return true;
}
-->
- `REQ-C-008`: 센서 변환 결과는 코드와 문서에서 같은 단위를 써야 한다.
<!--
구현 방향:
- 변수명에 motor_deg / steering_deg를 명시하고, API/로그/문서가 같은 이름을 사용하게 맞춘다.
- EncoderReader_GetAngleDeg() 같은 모호한 이름보다 EncoderReader_GetMotorDeg()가 안전하다.

예시 코드:
float EncoderReader_GetMotorDeg(void);
float SensorModel_ConvertPotRawToSteeringDeg(uint16_t raw);

// 문서 표기 예시:
// encoder.accum_count -> motor_deg -> steering_deg
-->
- `REQ-C-009`: 엔코더-ADC 차이를 이용한 cross-check 진단 로직이 있어야 한다.
<!--
구현 방향:
- runtime에서 encoder motor_deg를 steering_deg로 환산한 뒤 ADC absolute steering_deg와 비교한다.
- 차이가 일정 임계값을 넘는 상태가 연속 유지되면 warning -> fault로 승격한다.

예시 코드:
typedef struct {
    float warn_threshold_deg;
    float fault_threshold_deg;
    uint16_t warn_persist_count;
    uint16_t fault_persist_count;
    uint16_t warn_count;
    uint16_t mismatch_count;
} SensorCrossCheck_t;

static SensorCrossCheck_t g_cross_check = { 2.0f, 5.0f, 3U, 10U, 0U, 0U };

uint32_t Sensor_CheckCrossMismatch(float encoder_steering_deg, float adc_steering_deg)
{
    float diff = fabsf(encoder_steering_deg - adc_steering_deg);

    if (diff > g_cross_check.warn_threshold_deg) {
        if (++g_cross_check.warn_count >= g_cross_check.warn_persist_count) {
            SensorWarn_Raise(SENSOR_WARN_CROSSCHECK_DRIFT);
        }
    } else {
        g_cross_check.warn_count = 0U;
    }

    if (diff > g_cross_check.fault_threshold_deg) {
        if (++g_cross_check.mismatch_count >= g_cross_check.fault_persist_count) {
            return SENSOR_FAULT_CROSSCHECK_MISMATCH;
        }
    } else {
        g_cross_check.mismatch_count = 0U;
    }

    return SENSOR_FAULT_NONE;
}
-->
- `REQ-C-010`: 센서 fault는 fault code와 복귀 조건을 함께 제공해야 한다.
<!--
구현 방향:
- uint8_t fault_flag 대신 enum fault code와 clear 조건 함수를 만든다.
- fault 발생 조건과 clear 조건을 문서/코드에 같이 적는다.

예시 코드:
typedef enum {
    SENSOR_FAULT_NONE = 0,
    SENSOR_FAULT_ADC_TIMEOUT,
    SENSOR_FAULT_ADC_RANGE,
    SENSOR_FAULT_ADC_STUCK,
    SENSOR_FAULT_ENCODER_STALE,
    SENSOR_FAULT_CROSSCHECK_MISMATCH,
    SENSOR_FAULT_IMPLAUSIBLE_VELOCITY
} SensorFaultCode_t;

typedef struct {
    SensorFaultCode_t code;
    uint8_t latched;
    uint32_t detected_tick_ms;
    uint32_t clear_after_ms;
} SensorFault_t;

bool SensorFault_CanClear(const SensorFault_t *fault, uint32_t now_ms)
{
    return (fault->latched != 0U) && ((now_ms - fault->detected_tick_ms) >= fault->clear_after_ms);
}
-->
- `REQ-C-011`: 모든 센서 샘플은 timestamp 또는 age 정보와 함께 제공되어 freshness를 판단할 수 있어야 한다.
<!--
구현 방향:
- encoder/adc sample 구조체에 sample_tick_ms와 age_ms를 넣는다.
- 제어기에서는 age가 기준값보다 크면 stale sensor fault를 발생시킨다.

예시 코드:
bool SensorSample_IsFresh(uint32_t sample_tick_ms, uint32_t max_age_ms)
{
    uint32_t now_ms = HAL_GetTick();
    return ((now_ms - sample_tick_ms) <= max_age_ms);
}

if (!SensorSample_IsFresh(enc.sample_tick_ms, 5U)) {
    sensor_fault.code = SENSOR_FAULT_ENCODER_STALE;
}
-->
- `REQ-C-012`: 엔코더는 velocity estimate를 제공해야 하며, implausible velocity/acceleration을 fault 또는 warning으로 진단해야 한다.
<!--
구현 방향:
- delta_count / dt로 velocity를 구하고, 현재속도와 이전속도의 차이로 acceleration을 계산한다.
- 기계 한계를 넘는 값은 warning 또는 fault로 기록한다.

예시 코드:
typedef struct {
    float velocity_deg_s;
    float acceleration_deg_s2;
} EncoderKinematics_t;

static float g_prev_velocity_deg_s;

static void EncoderReader_UpdateKinematics(float dt_s, EncoderKinematics_t *out_kin)
{
    if ((out_kin == NULL) || (dt_s <= 0.0f)) {
        return;
    }

    out_kin->velocity_deg_s = ((float)g_encoder.delta_count * DEG_PER_COUNT) / dt_s;
    out_kin->acceleration_deg_s2 = (out_kin->velocity_deg_s - g_prev_velocity_deg_s) / dt_s;
    g_prev_velocity_deg_s = out_kin->velocity_deg_s;

    if (fabsf(out_kin->velocity_deg_s) > ENCODER_MAX_VELOCITY_DEG_S) {
        SensorFault_Raise(SENSOR_FAULT_IMPLAUSIBLE_VELOCITY);
    } else if (fabsf(out_kin->acceleration_deg_s2) > ENCODER_MAX_ACCEL_DEG_S2) {
        SensorWarn_Raise(SENSOR_WARN_IMPLAUSIBLE_ACCELERATION);
    }
}
-->
- `REQ-C-013`: homing에 쓰는 ADC angle이 `steering_deg`인지 `motor_deg`인지 명시하고, gear/linkage 변환식을 코드와 문서에서 동일하게 유지해야 한다.
<!--
구현 방향:
- ADC는 absolute steering sensor로 정의할지, motor-side sensor로 정의할지 먼저 고정한다.
- 아래 예시는 ADC가 steering_deg를 준다고 가정하고, homing 직전에 motor_deg로 변환해 offset을 맞추는 방식이다.

예시 코드:
int Homing_FindZero(void)
{
    ADC_PotSample_t adc_sample;
    float steering_deg;
    float motor_deg;
    int32_t offset_count;

    ADC_Pot_GetSample(&adc_sample);
    steering_deg = adc_sample.calibrated_angle_deg;
    motor_deg = SteeringDegToMotorDeg(steering_deg);
    offset_count = (int32_t)(motor_deg / ENCODER_DEG_PER_COUNT);

    EncoderReader_Reset();
    EncoderReader_SetOffset(offset_count);
    return 0;
}
-->
- `REQ-C-014`: calibration 데이터는 version, range, checksum 또는 동등한 무결성 정보를 포함해 재부팅 후에도 복원 가능해야 한다.
<!--
구현 방향:
- calibration 파라미터를 구조체로 만들고 flash/EEPROM/백업영역에 저장한다.
- 부팅 시 version/checksum 검증 후 복원 실패하면 defaults + invalid flag로 시작한다.

예시 코드:
typedef struct {
    uint32_t version;
    uint16_t min_raw;
    uint16_t max_raw;
    float min_angle_deg;
    float max_angle_deg;
    uint32_t checksum;
} ADC_PotCalibration_t;

static uint32_t Calibration_Crc32Step(uint32_t crc, uint8_t data)
{
    uint32_t i = 0U;

    crc ^= data;
    for (i = 0U; i < 8U; i++) {
        crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
    }

    return crc;
}

static uint32_t Calibration_CalcChecksum(const ADC_PotCalibration_t *cal)
{
    const uint8_t *bytes = (const uint8_t *)cal;
    uint32_t crc = 0xFFFFFFFFU;
    size_t i = 0U;

    for (i = 0U; i < (sizeof(*cal) - sizeof(cal->checksum)); i++) {
        crc = Calibration_Crc32Step(crc, bytes[i]);
    }

    return ~crc;
}

bool ADC_Pot_ValidateCalibration(const ADC_PotCalibration_t *cal)
{
    if (cal == NULL) {
        return false;
    }
    if (cal->version != ADC_POT_CAL_VERSION) {
        return false;
    }
    if (cal->min_raw >= cal->max_raw) {
        return false;
    }
    if (cal->checksum != Calibration_CalcChecksum(cal)) {
        return false;
    }

    return true;
}

bool ADC_Pot_LoadCalibration(ADC_PotCalibration_t *out_cal)
{
    ADC_PotCalibration_t stored = {0};

    if (out_cal == NULL) {
        return false;
    }
    if (!Flash_ReadCalibration(&stored) || !ADC_Pot_ValidateCalibration(&stored)) {
        *out_cal = ADC_Pot_GetDefaultCalibration();
        return false;
    }

    *out_cal = stored;
    return true;
}

bool ADC_Pot_SaveCalibration(const ADC_PotCalibration_t *cal);
-->
- `REQ-C-015`: commanded direction과 encoder delta 방향이 일정 시간 이상 불일치하면 sensor/actuator fault 후보로 보고해야 한다.
<!--
구현 방향:
- pulse_control이 마지막 commanded direction을 상태로 보관하고, encoder delta 부호와 비교한다.
- deadband와 startup grace time을 두고, 연속 mismatch만 fault 후보로 본다.

예시 코드:
typedef struct {
    int8_t commanded_sign;
    uint16_t startup_grace_count;
    uint16_t persist_count;
    int32_t min_delta_count;
    uint16_t mismatch_count;
} DirectionMonitor_t;

void Sensor_CheckDirectionMismatch(int8_t commanded_sign, int32_t delta_count)
{
    int8_t encoder_sign = (delta_count > 0) ? 1 : ((delta_count < 0) ? -1 : 0);

    if (g_direction_monitor.startup_grace_count > 0U) {
        g_direction_monitor.startup_grace_count--;
        g_direction_monitor.mismatch_count = 0U;
        return;
    }

    if ((delta_count < g_direction_monitor.min_delta_count) &&
        (delta_count > -g_direction_monitor.min_delta_count)) {
        g_direction_monitor.mismatch_count = 0U;
        return;
    }

    if ((commanded_sign != 0) && (encoder_sign != 0) && (commanded_sign != encoder_sign)) {
        if (++g_direction_monitor.mismatch_count >= g_direction_monitor.persist_count) {
            SensorFault_Raise(SENSOR_FAULT_DIRECTION_MISMATCH);
        }
    } else {
        g_direction_monitor.mismatch_count = 0U;
    }
}
-->
- `REQ-C-016`: 센서 fault는 정의된 최대 시간 내 safe state 전이와 진단 로그를 유발해야 한다.
<!--
구현 방향:
- SensorFault_Raise()에서 fault code를 기록하고, PositionControl_EmergencyStop() 또는 fault manager로 즉시 전달한다.
- 동시에 UART/trace buffer에 timestamp, raw, angle, diff를 로그로 남긴다.

예시 코드:
void SensorFault_Raise(SensorFaultCode_t code)
{
    SensorFaultLog_t log_entry;

    g_sensor_fault.code = code;
    g_sensor_fault.latched = 1U;
    g_sensor_fault.detected_tick_ms = HAL_GetTick();

    log_entry.code = code;
    log_entry.tick_ms = g_sensor_fault.detected_tick_ms;
    log_entry.encoder_count = g_encoder.accum_count;
    log_entry.adc_raw = g_adc_latest_raw;
    log_entry.crosscheck_diff_deg = g_last_crosscheck_diff_deg;
    SensorFaultLog_Push(&log_entry);

    PositionControl_EmergencyStop();
}
-->

인수 기준:

1. 장시간 정/역회전 중 angle discontinuity가 없다.
2. ADC calibration 절차와 코드가 일치한다.
3. `raw -> calibrated` 변환을 로그로 설명할 수 있다.
4. 엔코더와 ADC 차이가 기준을 넘으면 경고 또는 fault가 발생한다.
5. 센서 sample age와 calibration version을 로그로 설명할 수 있다.
6. disconnect, stuck, implausible speed 주입 시 fault가 정해진 시간 안에 검출된다.

## 6. 어떻게 더 발전시켜야 하는가

### P0

- 엔코더 unwrap 누적을 구현한다.
- `EncoderReader_GetAngleDeg()` 외에 `GetRaw`, `GetAccumCount`, `GetMotorDeg`를 분리한다.
- ADC init, calibration, valid flag를 정식 API로 만든다.
- 상수와 단위 정의를 한 곳으로 모은다.
- 센서 sample timestamp/age와 velocity estimate를 넣는다.
- homing에 쓰는 ADC 기준축과 gear ratio 적용 위치를 명확히 한다.
- sensor fault를 position/fault manager 경로로 실제 연결한다.

### P1

- ADC 필터링과 plausibility check를 추가한다.
- `encoder vs adc` 교차검증 임계값을 넣는다.
- runtime에서 blocking 없는 측정 경로를 만든다.
- 센서 로그 CSV를 자동 저장한다.
- commanded direction 대비 encoder sign mismatch 검출을 추가한다.
- calibration 값을 flash 또는 별도 설정 저장소에 보존한다.

### P2

- velocity estimate, noise sigma, drift monitor를 추가한다.
- 센서 health score를 상위 fault manager와 연결한다.
- 센서 노화, 온도 드리프트, 장시간 재현성까지 포함한 health model로 확장한다.
- 센서 이중화 또는 degraded mode 판단 정책까지 확장한다.

## 7. 정량 증거를 어떻게 남기면 좋은가

C 파트는 "센서를 얼마나 정확하고 연속적으로 읽었는가"를 남기면 강하다.

| 지표 | 의미 | 권장 증거 |
|---|---|---|
| discontinuity count | wrap 오류 여부 | long-run log |
| drift | 시간 경과에 따른 영점 변화 | 30분 이상 정지 로그 |
| ADC noise sigma | 아날로그 품질 | raw sample CSV |
| cross-check delta | encoder-ADC 차이 | 비교 그래프 |
| sensor fault detection rate | 이상 감지 품질 | fault injection 결과 |
| sample latency | 센서 읽기 비용 | latency CSV |

권장 파일:
- `Doc/measurements/encoder_longrun_2026-03-xx_run01.csv`
- `Doc/measurements/adc_noise_2026-03-xx_run01.csv`
- `Doc/measurements/sensor_crosscheck_2026-03-xx_run01.csv`

## 8. "얼마나 향상됐는가"를 보여주는 기준

| 항목 | Baseline | Target | 개선 표시법 |
|---|---|---|---|
| encoder discontinuity | 예: 30분에 4회 | 0회 | `count 감소` |
| zero drift | 예: 1.6deg/h | 0.2deg/h 이하 | `deg/h 감소` |
| ADC noise sigma | 예: 1.8deg | 0.3deg 이하 | `sigma 감소` |
| cross-check error | 예: 6deg max | 1deg 이하 | `deg 감소` |
| fault detect coverage | 예: 2/5 case | 5/5 case | `coverage 증가` |

면접에서 좋은 표현:
- "엔코더를 읽었다"보다 "16비트 wrap을 unwrap 누적으로 바꾸고 30분 구동에서 discontinuity 0건을 확인했다"가 훨씬 강하다.

## 9. 취업에 도움이 되는 기록 전략

C 파트는 아래 자료가 좋다.

1. 센서 파이프라인 도식
- raw count/raw ADC -> calibrated angle -> validation -> consumer

2. 센서 데이터셋
- 정지 노이즈
- 저속 sweep
- 장시간 반복 회전

3. 비교 그래프
- encoder angle vs ADC angle
- raw vs filtered

4. fault injection
- ADC 분리
- saturation
- encoder wrap 구간 통과

5. 설계 의도
- 왜 raw와 calibrated를 분리했는지
- 왜 cross-check가 필요한지

가장 추천하는 포트폴리오 문장:

> TIM4 encoder와 ADC absolute sensor를 각각 raw 데이터와 공학 단위로 정규화하고, wrap 처리와 plausibility check를 통해 제어기가 믿고 사용할 수 있는 센서 파이프라인으로 발전시켰다.

## 10. 지금부터 팀원 C에게 할당할 상세 개발 REQ

이 섹션은 C 파트를 "그냥 센서를 읽는 역할"이 아니라, 제어기가 믿어도 되는 센서 데이터를 만들어내는 책임으로 다시 쪼갠 실행형 요구사항이다. 지금 가장 시급한 것은 encoder sign과 wrap 문제를 먼저 닫는 것이다.

### `REQ-C-017`: encoder 회전 방향과 count 부호 관계를 물리 기준으로 확정해야 한다.
- 목적:
  - 지금처럼 `target`은 바뀌는데 `current` 부호가 맞는지 확신할 수 없는 상태를 끝낸다.
- 구현 범위:
  - 축을 실제 시계/반시계로 움직였을 때 `raw_count`, `delta_count`, `motor_deg`, `steering_deg`가 어떻게 변해야 하는지 정의한다.
  - 필요하면 `ENCODER_SIGN` 매크로를 도입해 부호를 한 곳에서 반전 가능하게 한다.
- 완료 기준:
  - 손회전 시험에서 방향별 `current` 부호가 기대와 항상 일치한다.
- 검증 방법:
  - hand turn CW/CCW 테스트
  - `P` snapshot과 logic analyzer `ENC_A/B` 비교
- 산출물:
  - encoder sign 확인표
  - 캡처 1세트

### `REQ-C-018`: 16비트 encoder를 누적 unwrap count로 확장해야 한다.
- 목적:
  - 현재 `TIM4` wrap 때문에 큰 각도에서 위치가 튀는 문제를 제거한다.
- 구현 범위:
  - 이전 raw count와 현재 raw count 차이를 이용해 `delta_count`를 계산한다.
  - wrap crossing 시 `accum_count`가 연속적으로 증가/감소하도록 만든다.
  - `GetRawCounter()`, `GetDeltaCount()`, `GetAccumCount()`, `GetAngleDeg()` 역할을 분리한다.
- 완료 기준:
  - wrap 구간을 지나도 angle discontinuity가 없어야 한다.
  - 최소 30분 long-run에서 discontinuity 0건이어야 한다.
- 검증 방법:
  - long rotation test
  - wrap boundary 집중 시험
- 산출물:
  - unwrap 설계 문서
  - long-run CSV

### `REQ-C-019`: 모든 센서 샘플은 freshness와 timestamp를 가져야 한다.
- 목적:
  - 오래된 값을 현재값처럼 써서 제어가 잘못 판단하는 것을 막는다.
- 구현 범위:
  - encoder와 ADC 모두 마지막 갱신 tick 또는 age를 제공한다.
  - sample age가 threshold를 넘으면 stale warning/fault를 발생시킨다.
- 완료 기준:
  - 센서 업데이트가 멈추면 bounded time 내 stale fault가 검출된다.
- 검증 방법:
  - 센서 입력 차단 또는 업데이트 중단 시뮬레이션
- 산출물:
  - freshness rule
  - stale detection log

### `REQ-C-020`: velocity / acceleration plausibility 진단을 도입해야 한다.
- 목적:
  - 엔코더 glitch나 계산 오류를 단순 위치값만으로는 놓치지 않게 한다.
- 구현 범위:
  - `velocity estimate`, `acceleration estimate`를 계산한다.
  - 물리적으로 불가능한 급변이나 sign flip을 warning/fault로 분리한다.
- 완료 기준:
  - glitch 주입 시 velocity fault 또는 warning이 발생한다.
  - 정상 저속 움직임에서는 false positive가 낮아야 한다.
- 검증 방법:
  - 저속/고속 수동 회전
  - injected jump test
- 산출물:
  - threshold 근거
  - fault log

### `REQ-C-021`: ADC absolute sensor는 calibration, range, 무결성 정보를 포함해야 한다.
- 목적:
  - ADC를 homing용 참고 센서로 쓰려면 raw 값만으로는 부족하다.
- 구현 범위:
  - `raw`, `voltage`, `angle`, `valid`, `range_status`를 제공한다.
  - calibration offset/gain/range/version/checksum 또는 동등한 무결성 정보를 둔다.
  - 재부팅 후 calibration 복원 정책을 정한다.
- 완료 기준:
  - calibration 적용 전후 angle 차이를 설명할 수 있다.
  - out-of-range와 disconnect를 구분해 진단할 수 있다.
- 검증 방법:
  - ADC sweep
  - sensor disconnect / saturation 시험
- 산출물:
  - calibration format
  - ADC validation log

### `REQ-C-022`: encoder와 ADC는 cross-check 진단 경로를 가져야 한다.
- 목적:
  - 단일 센서 오류를 다른 센서로 교차 검증해 조기에 잡는다.
- 구현 범위:
  - 두 센서를 같은 축 기준 단위로 맞춘다.
  - `|encoder_angle - adc_angle|`가 threshold를 넘는 경우 warning/fault 정책을 정의한다.
  - homing 중과 runtime 중 허용 오차를 분리한다.
- 완료 기준:
  - 센서 불일치 시 로그와 fault가 남는다.
  - 정상 상태에서는 허용 오차 안으로 유지된다.
- 검증 방법:
  - cross-check trend 기록
  - 센서 편향 인위 주입
- 산출물:
  - cross-check threshold 표
  - 비교 그래프

### `REQ-C-023`: commanded direction과 sensor delta 방향 불일치를 진단해야 한다.
- 목적:
  - D 파트의 direction 경로 문제와 C 파트의 sensor sign 문제를 빠르게 분리한다.
- 구현 범위:
  - `requested direction`, `DIR pin state`, `encoder delta sign`을 같은 주기 기준으로 비교한다.
  - 일정 시간 이상 불일치하면 `direction mismatch` 이벤트를 올린다.
- 완료 기준:
  - 실제 방향이 반대로 돌거나 sensor sign이 뒤집히면 탐지된다.
- 검증 방법:
  - `+1deg`, `-1deg` 반복 시험
  - deliberate polarity inversion 시험
- 산출물:
  - mismatch log
  - triage guide

### `REQ-C-024`: 센서 API는 raw / normalized / validated 계층으로 분리해야 한다.
- 목적:
  - 다른 파트가 센서 내부 구현을 몰라도 일관된 의미의 데이터를 사용할 수 있게 한다.
- 구현 범위:
  - raw layer: raw count, raw ADC
  - normalized layer: motor_deg, steering_deg, voltage
  - validated layer: validity, age, fault, plausibility
  - 함수와 구조체 이름에 계층 의미가 드러나게 정리한다.
- 완료 기준:
  - A/B 파트가 같은 API를 보고 센서 사용 의미를 이해할 수 있다.
- 검증 방법:
  - header review
  - API 사용처 점검
- 산출물:
  - sensor API 정리안
  - call-site mapping

### `REQ-C-025`: 장시간 안정성 데이터셋을 남겨야 한다.
- 목적:
  - 센서 품질을 감이 아니라 연속 데이터로 증명한다.
- 구현 범위:
  - 정지 drift
  - 저속 sweep
  - 반복 왕복
  - 장시간 유지
  - 최소 4종 데이터셋을 정의한다.
- 완료 기준:
  - 각 데이터셋에 측정 조건, 보드 상태, 샘플 수, 결과 요약이 붙는다.
- 검증 방법:
  - CubeMonitor 또는 CSV logger
- 산출물:
  - `encoder_longrun_*.csv`
  - `adc_noise_*.csv`
  - `sensor_crosscheck_*.csv`
