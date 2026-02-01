# 버그 리포트 및 해결 이력 (problem.md)

> servo_control_baqu 프로젝트에서 발견된 문제점과 해결 과정을 기록합니다.
> 각 항목에 **문제 발견 경위 → 원인 분석 → 해결 방법 → 검증**을 체계적으로 정리합니다.

---

## 목차

| # | 제목 | 파일 | 심각도 | 상태 |
|---|------|------|--------|------|
| BUG-001 | PulseControl_SetFrequency() 방향 핀 미설정 | pulse_control.c | **Critical** | **해결 (2026-01-31)** |
| BUG-002 | PositionControl_Update() 주기적 호출 부재 | main.c / stm32f4xx_it.c | **Critical** | **해결 (2026-01-31)** |
| BUG-003 | PulseControl_SetFrequency() PWM 시작 누락 | pulse_control.c | **Critical** | **해결 (2026-01-31)** |
| BUG-004 | position_control.h 선언 vs .c 구현 불일치 (13개 함수 미구현) | position_control.c/h | **Major** | **해결 (2026-02-01)** |
| BUG-005 | PID 파라미터 헤더/소스 불일치 | position_control.c/h | **Minor** | **해결 (2026-02-01)** |
| BUG-006 | 안정화 판단 시간 측정 부정확 | position_control.c | **Minor** | 미해결 |

---

## BUG-001: PulseControl_SetFrequency() 방향 핀 미설정

**심각도:** Critical
**상태:** **해결 완료 (2026-01-31)**
**발견일:** 2026-01-31
**파일:** `Core/Src/pulse_control.c` (57~72행)

### 문제 발견 경위

position_control.c의 `PositionControl_Update()`가 PID 계산 결과를 `PulseControl_SetFrequency(output)`으로 전달한다. PID output은 양수(정방향)·음수(역방향)로 방향 정보를 포함하는데, `SetFrequency()`가 이를 처리하지 않는다.

### 문제 코드

```c
// pulse_control.c : 57~72행
void PulseControl_SetFrequency(int32_t freq_hz) {
    if (freq_hz < 0) freq_hz = -freq_hz;  // ← 부호(방향 정보)를 그냥 제거
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
        return;
    }

    uint32_t timer_clk = 180000000;
    uint32_t psc = p_htim1->Instance->PSC;
    uint32_t arr = (timer_clk / ((psc + 1) * freq_hz)) - 1;

    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);
    // ← DIR 핀(PE11) 설정 코드 없음
}
```

### 원인 분석

- `freq_hz < 0`일 때 절대값 처리만 하고, 방향 핀(PE11)을 HIGH/LOW로 설정하지 않음
- 같은 파일의 `PulseControl_SendSteps()` 함수(77~95행)는 방향 핀을 올바르게 설정하고 있음
- `SetFrequency()`는 position_control용으로 나중에 추가된 함수로 보이며, 방향 처리가 누락된 것

### 영향

- PID output이 음수(역방향 회전 필요)여도 모터는 항상 같은 방향으로만 회전
- **closed-loop 제어 자체가 불가능** — 목표를 지나쳐도 되돌아올 수 없음

### 참고: 정상 동작하는 함수 (같은 파일)

```c
// pulse_control.c : 77~95행 — 이 함수는 방향을 올바르게 설정함
void PulseControl_SendSteps(uint32_t steps, MotorDirection dir) {
    if (dir == DIR_CW) {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);    // CW
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);  // CCW
    }
    HAL_TIM_PWM_Start_IT(p_htim1, TIM_CHANNEL_1);
}
```

### 해결 방법 (예정)

`freq_hz`의 부호에 따라 DIR 핀을 설정한 후 절대값으로 주파수를 계산해야 한다.

### 해결 결과 (2026-01-31 해결)

**해결자:** position_control 담당

**수정 내용:** `freq_hz`의 부호를 확인하여 절대값 변환 **전에** 방향 핀을 설정하도록 코드 추가

**수정 후 코드:**
```c
void PulseControl_SetFrequency(int32_t freq_hz) {
    // [추가] 부호에 따라 방향 핀 설정
    if (freq_hz >= 0) {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);   // CW
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET); // CCW
    }
    if (freq_hz < 0) freq_hz = -freq_hz; // 음수 처리
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
        return;
    }
    // ... ARR/CCR 설정 (기존 코드 유지)
}
```

**핵심:** 절대값 변환(`freq_hz = -freq_hz`) 전에 부호를 먼저 읽어서 방향을 결정하는 순서가 중요

---

## BUG-002: PositionControl_Update() 주기적 호출 부재

**심각도:** Critical
**상태:** **해결 완료 (2026-01-31 PID 호출 추가, 2026-02-01 데모 코드 제거)**
**발견일:** 2026-01-31
**파일:** `Core/Src/main.c` (120~144행), `Core/Src/stm32f4xx_it.c` (183~192행)

### 문제 발견 경위

position_control.h의 주석에 "1ms 타이머 인터럽트에서 호출"이라고 명시되어 있지만, 실제로 이를 호출하는 코드가 프로젝트 어디에도 없다.

### 문제 코드

```c
// main.c : 120~144행 — 데모 루프만 존재, PositionControl_Update() 호출 없음
while (1) {
    Relay_ServoOn();
    pulse_forward(5000);
    HAL_Delay(1000);
    pulse_reverse(5000);
    HAL_Delay(1000);
    Relay_ServoOff();
    HAL_Delay(2000);
}
```

```c
// stm32f4xx_it.c : 183~192행 — SysTick은 1ms 주기이지만 HAL_IncTick()만 호출
void SysTick_Handler(void) {
    HAL_IncTick();
    // ← PositionControl_Update() 호출 없음
}
```

### 원인 분석

- 현재 main.c는 Phase 1(기본 동작 확인) 단계의 데모 코드가 그대로 남아있음
- position_control.c의 PID 로직은 작성되어 있지만, 이를 주기적으로 호출하는 "연결 고리"가 빠져있음
- Init은 호출됨 (main.c 110행: `PositionControl_Init()`) → 초기화는 했지만 실행은 안 하는 상태

### 영향

- PID 제어가 전혀 실행되지 않음
- 엔코더 피드백을 읽기는 하지만 제어에 활용하지 않음
- 사실상 position_control.c 전체가 사용되지 않는 dead code 상태

### 해결 방법 (예정)

**방법 A — SysTick 활용:**
`stm32f4xx_it.c`의 `SysTick_Handler`에서 `PositionControl_Update()`를 호출. SysTick은 이미 1ms 주기이므로 별도 타이머 설정 불필요. 단, ISR 내 실행 시간에 주의.

**방법 B — 별도 타이머 (TIM3 등):**
CubeMX에서 TIM3를 1ms 주기 인터럽트로 설정하고 해당 ISR에서 호출. 가장 깔끔하지만 CubeMX 재설정 필요.

**방법 C — main loop 폴링:**
main.c의 while(1)에서 `HAL_GetTick()` 기반으로 1ms마다 호출. 가장 간단하지만 다른 처리 지연 시 제어 주기 불안정.

### 해결 결과 (2026-01-31 해결)

**해결자:** position_control 담당

**채택한 방법:** SysTick + 플래그 방식 (A와 C의 장점 결합)

**설계 의도:**
- SysTick ISR에서 직접 Update()를 호출하면 ISR 내부에서 printf, HAL_UART 등 블로킹 함수가 실행되어 위험
- ISR에서는 플래그만 세우고(빠름), main loop에서 플래그를 확인 후 Update()를 실행(안전)
- 산업용 임베디드에서 표준적으로 사용되는 패턴

**수정 파일 2개:**

**① stm32f4xx_it.c — 플래그 변수 + SysTick에서 설정**
```c
// Private variables 영역
volatile uint8_t interrupt_flag = 0;

// SysTick_Handler 내부
void SysTick_Handler(void) {
    HAL_IncTick();
    interrupt_flag = 1;  // [추가] 1ms마다 플래그 설정
}
```

**② main_edit.c — extern 선언 + while 루프에서 확인**
```c
// Private variables 영역
extern volatile uint8_t interrupt_flag;

// while(1) 루프
while (1) {
    if (interrupt_flag == 1) {
        interrupt_flag = 0;
        PositionControl_Update();  // 1ms 주기 PID 제어
    }
}
```

**핵심:** `volatile` 키워드가 필수 — 인터럽트에서 변경된 값을 main loop에서 올바르게 읽기 위함

~~**남은 작업:** main_edit.c의 기존 데모 코드(pulse_forward/reverse, HAL_Delay) 제거 필요 (별도 작업)~~

**후속 해결 (2026-02-01):**
- 데모 코드 전부 삭제 (pulse_forward, pulse_reverse, HAL_Delay, Relay_ServoOff)
- while 전에 시스템 시동 시퀀스 추가: `Relay_ServoOn()` → `HAL_Delay(1000)` → `SetTarget(20.0f)` → `Enable()`
- while 안에는 flag 체크 + `PositionControl_Update()`만 남김
- 상세 변경 내용: [change_code.md 변경 #4](change_code.md) 참조

---

## BUG-003: PulseControl_SetFrequency() PWM 시작 누락

**심각도:** Critical
**상태:** **해결 완료 (2026-01-31)**
**발견일:** 2026-01-31
**파일:** `Core/Src/pulse_control.c` (57~72행)

### 문제 발견 경위

BUG-001과 같은 함수에서 추가로 발견. `SetFrequency()`는 타이머의 ARR/CCR 레지스터만 변경하고, PWM 출력을 시작하는 코드가 없다.

### 문제 코드

```c
// pulse_control.c : 57~72행
void PulseControl_SetFrequency(int32_t freq_hz) {
    if (freq_hz < 0) freq_hz = -freq_hz;
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);  // 정지는 구현됨
        return;
    }

    uint32_t timer_clk = 180000000;
    uint32_t psc = p_htim1->Instance->PSC;
    uint32_t arr = (timer_clk / ((psc + 1) * freq_hz)) - 1;

    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);
    // ← HAL_TIM_PWM_Start_IT() 호출 없음
}
```

### 원인 분석

- `freq_hz == 0`일 때 `PWM_Stop_IT`는 호출하지만, `freq_hz != 0`일 때 `PWM_Start_IT`는 호출하지 않음
- PWM이 이미 돌고 있는 상태에서 ARR만 바꾸면 주파수는 변경되지만, PWM이 멈춰있는 상태에서는 펄스가 출력되지 않음
- 최초 호출 시 또는 freq=0으로 Stop한 후 다시 호출 시 문제 발생

### 영향

- PID 계산은 정상이어도 모터가 안 움직이는 상황 발생 가능
- 특히 목표 도달 후(output=0 → Stop) 새 목표 설정 시(output!=0) 모터 무반응

### 해결 방법 (예정)

`freq_hz != 0`일 때 ARR/CCR 설정 후 `HAL_TIM_PWM_Start_IT()`를 호출하거나, PWM 동작 상태를 추적하여 조건부 시작.

### 해결 결과 (2026-01-31 해결)

**해결자:** position_control 담당

**수정 내용:**
1. ARR/CCR 설정 **후에** `HAL_TIM_PWM_Start_IT()` 호출 추가
2. ARR 범위 보호 추가 (`0xFFFF` 상한, `1` 하한)

**수정 후 코드:**
```c
    // ARR/CCR 설정
    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);

    // [추가] PWM 시작
    if(freq_hz != 0) {HAL_TIM_PWM_Start_IT(p_htim1, TIM_CHANNEL_1);}
```

**핵심:** ARR/CCR로 주파수를 먼저 설정한 뒤에 PWM을 시작해야 원하는 주파수로 출력됨

---

## BUG-004: position_control.h 선언 vs .c 구현 불일치

**심각도:** Major
**상태:** **해결 완료 (13/13, 2026-02-01)**
**발견일:** 2026-01-31
**파일:** `Core/Inc/position_control.h`, `Core/Src/position_control.c`

### 문제 발견 경위

position_control.h에 선언된 함수 중 13개가 position_control.c에 구현되어 있지 않다.

### 미구현 함수 목록

```
// 헤더에 선언만 있고, 소스에 구현이 없는 함수들

✅ PositionControl_GetTarget()              // 목표 각도 반환         → Group 1 (2026-02-01 해결)
✅ PositionControl_GetError()               // 현재 오차 반환         → Group 1 (2026-02-01 해결)
✅ PositionControl_GetPID()                 // PID 파라미터 읽기      → Group 1 (2026-02-01 해결)
✅ PositionControl_SetMode()                // 제어 모드 변경         → Group 2 (2026-02-01 해결)
✅ PositionControl_GetMode()                // 현재 모드 조회         → Group 2 (2026-02-01 해결)
✅ PositionControl_SetSafetyLimits()        // 안전 한계값 설정       → Group 3 (2026-02-01 해결)
✅ PositionControl_IsSafe()                 // 안전 상태 확인         → Group 3 (2026-02-01 해결)
✅ PositionControl_GetStats()               // 성능 통계 조회         → Group 3 (2026-02-01 해결)
✅ PositionControl_ResetStats()             // 통계 리셋             → Group 3 (2026-02-01 해결)
✅ PositionControl_RegisterErrorCallback()  // 에러 콜백 등록         → Group 4 (2026-02-01 해결)
✅ PositionControl_RegisterStableCallback() // 안정화 콜백 등록       → Group 4 (2026-02-01 해결)
✅ PositionControl_SetDebugLevel()          // 디버그 레벨 설정       → Group 4 (2026-02-01 해결)
✅ PositionControl_GetErrorString()         // 에러코드 → 문자열      → Group 4 (2026-02-01 해결)
```

### 원인 분석

- position_control.h를 설계할 때 향후 필요한 API를 미리 선언해 둔 것으로 보임
- position_control.c에는 핵심 기능(Init, Update, SetTarget, Enable/Disable 등)만 먼저 구현된 상태
- 설계(헤더)와 구현(소스)이 동기화되지 않음

### 영향

- 현재는 미구현 함수를 호출하는 코드가 없어 **빌드는 성공**할 수 있음
- 향후 UART 통신(Phase 2)에서 상태 조회나 모드 변경 명령을 구현할 때 링커 에러 발생
- 코드 리뷰 시 "선언만 하고 구현 안 한 함수" 지적 가능

### 해결 방법 (예정)

**방법 A — 필요한 함수만 구현:** 당장 사용할 함수(GetTarget, GetError 등)만 구현하고, 나머지는 헤더에서 제거
**방법 B — 전체 구현:** 13개 함수 모두 구현 (시간 소요)
**방법 C — 단계적 구현:** 우선순위별로 나눠서 필요할 때마다 추가

### 해결 결과 — Group 1: 상태 읽기 Getter (2026-02-01)

**해결자:** position_control 담당

**구현 내용:** 3개 함수 구현 (GetTarget, GetError, GetPID)

| 함수 | 구현 코드 | 비고 |
|------|-----------|------|
| `GetTarget()` | `return state.target_angle;` | 단순 값 반환 |
| `GetError()` | `return state.error;` | 단순 값 반환 |
| `GetPID()` | `*params = pid_params;` | 포인터를 통한 구조체 복사 + NULL 방어 |

**핵심:** `GetPID()`는 호출자가 NULL 포인터를 전달했을 때의 크래시를 방지하기 위해 `if (params != NULL)` 체크를 추가함

**상세 변경 내용:** [change_code.md 변경 #5](change_code.md) 참조

**남은 작업:** Groups 2~4 (10개 함수) 순차 구현 예정

---

## BUG-005: PID 파라미터 헤더/소스 불일치

**심각도:** Minor
**상태:** 미해결
**발견일:** 2026-01-31
**파일:** `Core/Inc/position_control.h` (20~24행), `Core/Src/position_control.c` (11~17행)

### 문제 코드

```c
// position_control.h (20~24행) — DEFAULT 상수
#define DEFAULT_KP             2.0f
#define DEFAULT_KI             0.1f
#define DEFAULT_KD             0.5f
#define DEFAULT_INTEGRAL_LIMIT 10.0f
#define DEFAULT_OUTPUT_LIMIT   100.0f
```

```c
// position_control.c (11~17행) — 실제 초기값
static PID_Params_t pid_params = {
    .Kp = 500.0f,              // 헤더 DEFAULT: 2.0
    .Ki = 5.0f,                // 헤더 DEFAULT: 0.1
    .Kd = 20.0f,               // 헤더 DEFAULT: 0.5
    .integral_limit = 1000.0f, // 헤더 DEFAULT: 10.0
    .output_limit = 10000.0f   // 헤더 DEFAULT: 100.0
};
```

### 원인 분석

- 헤더의 DEFAULT 값은 초기 설계 시 작성된 것으로 추정
- .c의 실제 값은 하드웨어 테스트 후 튜닝된 값으로 추정
- .c에서 DEFAULT 매크로를 사용하지 않고 직접 숫자를 넣어서 불일치 발생

### 영향

- 동작에는 영향 없음 (DEFAULT 매크로가 실제로 사용되지 않으므로)
- 코드 가독성 저하 — 헤더를 보고 "Kp=2.0이구나" 하고 오해할 수 있음
- 유지보수 시 혼란 유발

### 해결 방법 (예정)

.c의 초기값이 정확한 값이라면 .h의 DEFAULT 매크로를 .c의 값으로 맞추고, .c에서 DEFAULT 매크로를 사용하도록 변경.

### 해결 결과

> (해결 후 작성 예정)

---

## BUG-006: 안정화 판단 시간 측정 부정확

**심각도:** Minor
**상태:** 미해결
**발견일:** 2026-01-31
**파일:** `Core/Src/position_control.c` (119~128행)

### 문제 코드

```c
// position_control.c : 119~128행
if (fabsf(state.error) < POSITION_TOLERANCE) {
    state.stable_time_ms++;              // ← 1ms마다 호출된다는 가정
    if (state.stable_time_ms > 100) {    // 100ms 이상 안정
        state.is_stable = true;
    }
} else {
    state.stable_time_ms = 0;
    state.is_stable = false;
}
```

### 원인 분석

- `stable_time_ms++`는 "Update가 정확히 1ms마다 호출된다"는 가정 하에 작동
- 실제로는 호출 주기가 정확히 1ms가 아닐 수 있음 (ISR 지연, 다른 인터럽트 우선순위 등)
- 폴링 방식(방법 C)으로 구현하면 더 큰 오차 발생

### 영향

- 안정화 판단 시간이 실제 100ms와 차이날 수 있음
- 제어 품질에 큰 영향은 없지만, 정밀한 타이밍이 필요한 상위 시스템 연동 시 문제 가능

### 해결 방법 (예정)

`stable_time_ms++` 대신 `HAL_GetTick()` 기반으로 실제 경과 시간을 측정하도록 변경.

```c
// 개선 방향 예시
if (fabsf(state.error) < POSITION_TOLERANCE) {
    if (state.stable_start_ms == 0) {
        state.stable_start_ms = HAL_GetTick();  // 안정 시작 시점 기록
    }
    if (HAL_GetTick() - state.stable_start_ms > 100) {
        state.is_stable = true;
    }
} else {
    state.stable_start_ms = 0;
    state.is_stable = false;
}
```

### 해결 결과

> (해결 후 작성 예정)

---

## 해결 완료 항목

> (해결된 버그는 이 섹션으로 이동하여 정리합니다)

---

*마지막 업데이트: 2026-02-01*
