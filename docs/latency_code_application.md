# Latency Spec Code Application (Steering Only)

이 문서는 `docs/latency_measurement_spec.md`를 실제 코드에 반영한 내역을 Step 순서대로 기록한다.

## Step 0) Deadline 숫자 고정 반영

- 제어 루프 deadline: `1 ms (1 kHz)`
- 근거: `SysTick`에서 `interrupt_flag`를 1ms마다 세팅하는 현재 구조
- 관련 파일:
  - `Core/Src/stm32f4xx_it.c`
  - `Core/Src/main.c`

적용 포인트:
- `SysTick` tick이 왔는데 이전 tick(`interrupt_flag==1`)이 미처리면 deadline miss로 정의하고 카운트한다.

---

## Step 1) 구간 ID 고정 반영

구간 ID는 아래로 고정했다.

- `ID1 Sense`
- `ID2 Control`
- `ID3 Actuate`
- `ID4 Comms`

### 1-1. 공통 계측 모듈 추가

추가 파일:
- `Core/Inc/latency_profiler.h`
- `Core/Src/latency_profiler.c`

구현 내용:
- `DWT CYCCNT` 초기화
- `LAT_BEGIN(stage)`, `LAT_END(stage)` 매크로
- stage별 샘플 저장 (`LATENCY_MAX_SAMPLES`, 기본 2048)
- `avg`, `p99`, `max` 계산 API
- `deadline_miss_count` 카운트 API

### 1-2. ID4 (Comms) 계측 삽입

수정 파일:
- `Core/Src/main.c`

삽입 위치:
- `MX_LWIP_Process()`부터 `EthComm_HasNewData()` 처리 종료까지

적용 코드 패턴:
- `LAT_BEGIN(LAT_STAGE_COMMS);`
- 통신/모드 처리
- `LAT_END(LAT_STAGE_COMMS);`

### 1-3. ID1/ID2/ID3 계측 삽입

수정 파일:
- `Core/Src/position_control.c`

삽입 위치:
- `ID1 Sense`: 엔코더 읽기 + 오차 계산
- `ID2 Control`: Safety check + PID 계산
- `ID3 Actuate`: `PulseControl_SetFrequency()`

적용 코드 패턴:
- `LAT_BEGIN(LAT_STAGE_SENSE)` / `LAT_END(...)`
- `LAT_BEGIN(LAT_STAGE_CONTROL)` / `LAT_END(...)`
- `LAT_BEGIN(LAT_STAGE_ACTUATE)` / `LAT_END(...)`

---

## Step 2) 조건 고정 체크리스트 반영

### 2-1. 클럭 고정값 계측기 초기화

수정 파일:
- `Core/Src/main.c`

적용 내용:
- `SystemClock_Config()` 직후 `LatencyProfiler_Init(SystemCoreClock);` 호출
- 런타임 `SystemCoreClock` 기준으로 cycles->us 변환

### 2-2. 측정 중 로그 영향 분리

수정 파일:
- `Core/Inc/latency_profiler.h`
- `Core/Src/main.c`
- `Core/Src/position_control.c`

적용 내용:
- `LATENCY_LOG_ENABLE` 매크로 추가 (기본 0)
- 측정 구간에 영향을 주는 `printf`를 `#if LATENCY_LOG_ENABLE`로 감쌈

운영 방법:
- 기준 측정: `LATENCY_LOG_ENABLE=0`
- 영향 비교: `LATENCY_LOG_ENABLE=1`

---

## Step 3) DWT 기반 p99/worst-case 수집

수정 파일:
- `Core/Src/latency_profiler.c`

적용 내용:
- 시작: `DWT->CYCCNT` 저장
- 종료: `dt = DWT->CYCCNT - start`
- 샘플 누적 후 `qsort`로 p99 인덱스 계산
- 평균/최대 계산

주의:
- 기본 샘플 버퍼는 `2048`이다.
- `10,000` 샘플이 필요하면 `LATENCY_MAX_SAMPLES`를 늘리되 RAM 여유를 먼저 확인한다.

---

## Step 4) deadline_miss_count 반영

수정 파일:
- `Core/Src/stm32f4xx_it.c`

적용 코드:
- `LatencyProfiler_OnDeadlineTick(interrupt_flag != 0U);`
- 이후 `interrupt_flag = 1;`

의미:
- 새 tick 시점에 이전 loop flag가 남아 있으면 miss 1회 증가

---

## 반영 파일 목록

- `Core/Inc/latency_profiler.h` (신규)
- `Core/Src/latency_profiler.c` (신규)
- `Core/Src/main.c` (수정)
- `Core/Src/position_control.c` (수정)
- `Core/Src/stm32f4xx_it.c` (수정)

---

## 코드 수정 상세 (실제 반영 코드)

### A) 신규 계측 헤더 추가

파일:
- `Core/Inc/latency_profiler.h`

핵심 추가 코드:

```c
typedef enum {
    LAT_STAGE_SENSE = 0,
    LAT_STAGE_CONTROL = 1,
    LAT_STAGE_ACTUATE = 2,
    LAT_STAGE_COMMS = 3,
    LAT_STAGE_COUNT
} LatencyStage_t;

#if LATENCY_PROFILER_ENABLE
#define LAT_BEGIN(stage) LatencyProfiler_Begin(stage)
#define LAT_END(stage) LatencyProfiler_End(stage)
#else
#define LAT_BEGIN(stage) ((void)(stage))
#define LAT_END(stage) ((void)(stage))
#endif
```

의도:
- 구간 ID를 enum으로 고정
- 호출부에서는 `LAT_BEGIN/LAT_END`만 사용하도록 단순화

### B) 신규 DWT 계측 구현 추가

파일:
- `Core/Src/latency_profiler.c`

핵심 추가 코드:

```c
static void dwt_cyccnt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void LatencyProfiler_End(LatencyStage_t stage)
{
    ...
    dt = DWT->CYCCNT - buf->start_cycle;
    buf->samples[buf->sample_count] = dt;
    buf->sample_count++;
    buf->sum_cycles += (uint64_t)dt;
    if (dt > buf->max_cycles) {
        buf->max_cycles = dt;
    }
}
```

의도:
- DWT로 cycle 단위 시간 측정
- stage별 샘플/평균/최대 집계 기반 확보

### C) `main.c` 수정 (ID4 Comms + 초기화 + 로그 분리)

파일:
- `Core/Src/main.c`

핵심 수정 코드:

```c
#include "latency_profiler.h"
...
SystemClock_Config();
LatencyProfiler_Init(SystemCoreClock);
```

```c
LAT_BEGIN(LAT_STAGE_COMMS);
MX_LWIP_Process();
...
if (EthComm_HasNewData()) {
    ...
}
LAT_END(LAT_STAGE_COMMS);
```

```c
#if LATENCY_LOG_ENABLE
printf("[DIAG] ...");
#else
(void)arr; (void)ccr; (void)enc_raw; (void)dir_state; (void)s;
#endif
```

의도:
- 런타임 코어 클럭 기준 변환값 사용
- 통신 구간(ID4) 지연을 독립 측정
- 측정 모드에서 로그 오버헤드 제거

추가 수정(자동 배치 리포트):

```c
static void Latency_TryAutoReport(void)
{
    ...
    if (LatencyProfiler_GetStageSampleCount(stage) < LATENCY_AUTO_REPORT_SAMPLES) {
        return;
    }
    printf("LATENCY_BATCH_BEGIN,...");
    printf("LATENCY_STAGE,...");
    printf("LATENCY_BATCH_END,...");
    LatencyProfiler_Reset();
}
```

```c
while (1) {
    ...
    Latency_TryAutoReport();
    HAL_IWDG_Refresh(&hiwdg);
}
```

의도:
- `2000` 샘플이 모이면 자동으로 통계를 UART CSV 형태로 출력
- 출력 후 계측 버퍼를 리셋하여 다음 2000샘플 배치를 계속 수집

### D) `position_control.c` 수정 (ID1/2/3 + 로그 분리)

파일:
- `Core/Src/position_control.c`

핵심 수정 코드:

```c
#include "latency_profiler.h"
```

```c
LAT_BEGIN(LAT_STAGE_SENSE);
state.current_angle = EncoderReader_GetAngleDeg();
state.error = state.target_angle - state.current_angle;
LAT_END(LAT_STAGE_SENSE);
```

```c
LAT_BEGIN(LAT_STAGE_CONTROL);
if (!PositionControl_CheckSafety()) {
    LAT_END(LAT_STAGE_CONTROL);
    PositionControl_EmergencyStop();
    return;
}
...
state.output = PID_Calculate(state.error, dt);
LAT_END(LAT_STAGE_CONTROL);
```

```c
LAT_BEGIN(LAT_STAGE_ACTUATE);
PulseControl_SetFrequency((int32_t)state.output);
LAT_END(LAT_STAGE_ACTUATE);
```

```c
#if LATENCY_LOG_ENABLE
printf("[PosCtrl] ...");
#endif
```

의도:
- Sense/Control/Actuate 구간 분리 계측
- 에러 경로에서도 Control 구간 종료값이 기록되도록 처리
- 측정 중 `printf` 영향 차단

### E) `stm32f4xx_it.c` 수정 (deadline_miss_count)

파일:
- `Core/Src/stm32f4xx_it.c`

핵심 수정 코드:

```c
#include "latency_profiler.h"
...
void SysTick_Handler(void)
{
  LatencyProfiler_OnDeadlineTick(interrupt_flag != 0U);
  HAL_IncTick();
  interrupt_flag = 1;
}
```

의도:
- 새 tick 진입 시 이전 tick 미처리 여부를 deadline miss로 카운트

---

## 실행/측정 절차 (실무용)

1. `LATENCY_LOG_ENABLE=0` 상태로 빌드 (`-O2` 유지)
2. 동일 입력 시퀀스로 실행
3. UART에서 `LATENCY_BATCH_BEGIN/ LATENCY_STAGE/ LATENCY_BATCH_END` 로그를 파일로 저장
4. 배치별 `avg/p99/max`와 `deadline_miss_count`를 `docs/latency_contract.md` 표에 반영
5. 필요 시 `LATENCY_LOG_ENABLE=1` 재측정 후 영향 비교표 추가

---

## 결과 저장(증거자료) 방식

- 펌웨어는 2000샘플마다 CSV 로그를 계속 출력한다.
- 실제 “장기 보관”은 PC에서 시리얼 로그를 파일로 저장해 수행한다.
- Windows 환경에서는 `docs/measurements/start_latency_log.ps1`로 PuTTY 자동 저장을 사용한다.
- 저장 상세 절차는 `docs/latency_data_evidence.md`를 따른다.
