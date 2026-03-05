# Steering Latency Measurement Spec (Interview-Ready)

## 1) 먼저 확인: 지금 하시려는 방향이 맞는가?

네, 맞습니다. 핵심은 "빠르다"가 아니라 아래 3가지를 숫자로 고정하는 것입니다.

1. Deadline(몇 ms 안에 끝나야 하는지)
2. 조건 고정(빌드/클럭/캐시/입력/로그)
3. 구간별 p99/worst-case(max)

이 3개가 있어야 재현 가능한 성능 엔지니어링으로 인정받습니다.

---

## 2) 180 MHz 근거 (이 프로젝트 기준)

### 2.1 보드/칩 스펙

- STM32F429ZI는 최대 180 MHz 동작이 가능한 MCU입니다.

### 2.2 현재 코드 설정 근거

`Core/Src/main.c`의 `SystemClock_Config()`에서:

- `PLLM = 4`
- `PLLN = 180`
- `PLLP = 2`

설정이 들어가 있으며, 일반적인 HSE 8 MHz 기준 SYSCLK는:

- `SYSCLK = (HSE / PLLM) * PLLN / PLLP`
- `= (8 / 4) * 180 / 2 = 180 MHz`

즉, 이 프로젝트는 현재 180 MHz로 구성되어 있다고 보는 것이 타당합니다.
단, 최종 측정 보고서에는 `SystemCoreClock` 런타임 값을 같이 기록해 확정하세요.

---

## 3) Step 0: Deadline 숫자 고정

레이더는 현재 프로젝트 범위 밖이므로, 이 문서는 조향만 고정합니다.

### 3.1 조향 Deadline

- 루프 주기: `1 kHz (1 ms)`
- Hard deadline: `1.0 ms`
- 권장 예산 분해(초기안):
  - `ID1 Sense`: `<= 0.20 ms`
  - `ID2 Control`: `<= 0.35 ms`
  - `ID3 Actuate`: `<= 0.20 ms`
  - `ID4 Comms`: `<= 0.20 ms`
  - Margin: `0.05 ms`

비고: 현재 코드가 `SysTick` 기반 1ms 플래그로 제어 루프를 돌고 있으므로, 1ms deadline 정의가 코드 구조와 일치합니다.

---

## 4) Step 1: 측정 구간 ID 고정 (이 코드에 매핑)

### ID 정의

- `ID1 Sense`: 센서 읽기 + 오차 계산
- `ID2 Control`: 안전 체크 + PID 계산
- `ID3 Actuate`: PWM/펄스 출력 호출
- `ID4 Comms`: LwIP 처리 + 모드/패킷 반영

### 코드 삽입 위치

- `ID4 Comms`
  - 파일: `Core/Src/main.c`
  - 구간: `MX_LWIP_Process()`부터 `EthComm_HasNewData()` 처리 끝까지
- `ID1~ID3`
  - 파일: `Core/Src/position_control.c`
  - `ID1`: `EncoderReader_GetAngleDeg()` + `state.error` 계산
  - `ID2`: `PositionControl_CheckSafety()` + `PID_Calculate()`
  - `ID3`: `PulseControl_SetFrequency()`

---

## 5) Step 2: 조건 고정 체크리스트 (이거 안 하면 무효)

측정 시작 전에 아래를 체크하고 문서에 박으세요.

- [ ] Build option: `-O2` 고정
- [ ] 클럭/PLL 고정 (`SystemClock_Config` 변경 금지)
- [ ] Cache 상태 명시 (I-Cache, D-Cache ON/OFF)
- [ ] 측정 구간 내 `printf` 금지
- [ ] 입력 시퀀스 고정 (동일 UDP 리플레이/동일 seed)
- [ ] 동일 바이너리(Git SHA)로 반복 측정

### 로그 영향 분리 규칙

- 실험 A: `LOG OFF` (기준값)
- 실험 B: `LOG ON`  (영향 비교)

동일 입력으로 A/B를 각각 측정해 `p99`, `max` 차이를 별도 표로 남깁니다.

---

## 6) Step 3: DWT 계측 방법 (STM32F4 즉시 적용)

## 6.1 원리

- 시작: `t0 = DWT->CYCCNT`
- 종료: `dt = DWT->CYCCNT - t0`
- 각 샘플을 버퍼에 저장 (측정 중 출력 금지)
- N회 수집 후 오프라인 계산:
  - `avg`
  - `p99`
  - `max`

## 6.2 초기화 코드 예시

```c
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
```

## 6.3 변환식

- `us = cycles / (SystemCoreClock / 1,000,000.0)`

180MHz 고정이면 `us = cycles / 180.0`.

---

## 7) Step 4: deadline_miss_count 구현 (현재 코드 맞춤)

파일: `Core/Src/stm32f4xx_it.c`

현재 `SysTick_Handler()`에서 `interrupt_flag = 1`을 세팅합니다.
여기서 다음 규칙을 추가하면 deadline miss를 정확히 셀 수 있습니다.

- SysTick 진입 시 이미 `interrupt_flag == 1`이면
  - 이전 1ms 슬롯을 메인 루프가 아직 처리 못한 상태
  - `deadline_miss_count++`

즉, "다음 주기 tick이 왔는데 이전 주기가 안 비워진 경우"를 miss로 정의합니다.

---

## 8) Step 5: 계측 우선순위 (조향 프로젝트용)

1. `ID2 Control`의 `p99`/`max` 먼저 확인
2. `ID4 Comms`가 스파이크를 만드는지 확인
3. `printf` ON/OFF 차이 확인
4. 스파이크 원인 점검:
   - 동적할당
   - 컨테이너 재할당/rehash (해당 코드가 있을 때)
   - 인터럽트 과점유

참고: 현재 코드에는 STL 컨테이너가 보이지 않으므로, 우선순위는 `printf`/통신 부하/인터럽트 점유율입니다.

---

## 9) Step 6: 결과 산출 규칙

- 샘플 수(현재 구현): 각 ID당 `2,000` 자동 배치 출력
- 목표 샘플 수(권장): 각 ID당 최소 `10,000`
- 워밍업: 초기 `1~2초` 제외
- 보고 항목:
  - ID1~ID4 각각 `avg/p99/max` (cycles + us)
  - `deadline_miss_count`
  - (FreeRTOS 사용 시) `min_ever_free_heap`

현재 프로젝트는 FreeRTOS 태스크 구조가 아니므로 `min_ever_free_heap`은 `N/A` 표기.

비고:
- 현재 코드는 2000샘플이 모이면 UART CSV(`LATENCY_BATCH_*`)를 자동 출력하고 리셋함.
- 10,000샘플 보고가 필요하면 2000샘플 배치 5개를 합산하거나, `LATENCY_MAX_SAMPLES`/`LATENCY_AUTO_REPORT_SAMPLES`를 조정.

---

## 10) 적용 상태 (현재 코드 기준)

1. 완료: `latency_profiler.c/.h` 추가 (DWT + 샘플 버퍼 + 통계)
2. 완료: `main.c`에 `ID4` 계측 매크로 삽입
3. 완료: `position_control.c`에 `ID1/ID2/ID3` 계측 매크로 삽입
4. 완료: `stm32f4xx_it.c`에 `deadline_miss_count` 로직 삽입
5. 완료: 측정 중 로그 분리 (`LATENCY_LOG_ENABLE`)
6. 완료: 2000샘플 자동 CSV 배치 출력 (`LATENCY_AUTO_REPORT_SAMPLES`)
7. 진행 필요: 배치별 결과를 `docs/latency_contract.md` 표에 반영

---

## 11) 면접에서 설명하는 한 줄 버전

"조향 제어 루프 1ms deadline을 먼저 고정하고, DWT 사이클 기반으로 Sense/Control/Actuate/Comms 구간의 avg/p99/max와 deadline miss를 동일 조건에서 반복 측정해 재현성을 확보했습니다."
