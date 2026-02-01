# 코드 변경 이력 (change_code.md)

> position_control 통합 작업 중 발생하는 모든 코드 변경을 기록합니다.
> 각 항목에 **변경 이유**, **변경 전/후 코드**, **영향 범위**를 명시합니다.

---

## 변경 예정 목록

| # | 파일 | 내용 | 상태 |
|---|------|------|------|
| 1 | pulse_control.c | SetFrequency() 방향 핀 버그 수정 | **완료** |
| 2 | stm32f4xx_it.c + main_edit.c | 1ms 주기 PositionControl_Update() 호출 추가 | **완료** |
| 3 | main_edit.c | 데모 루프 → closed-loop 제어 루프 교체 | **완료** |
| 4 | position_control.c | 미구현 함수 구현 (GetTarget, GetError 등) | **완료** (13/13) |
| 5 | position_control.h / .c | PID DEFAULT 값 불일치 정리 | **완료** |
| 6 | position_control.c | 안정화 판단 로직 개선 | 대기 |

---

## 변경 #1: SetFrequency() 방향 핀 설정 추가 (BUG-001)

**날짜:** 2026-01-31
**파일:** `Core/Src/pulse_control.c` (57~62행)
**관련 버그:** BUG-001

### 변경 이유

PID 제어 출력(`output`)은 양수(정방향)/음수(역방향)로 방향 정보를 포함한다.
`SetFrequency()`가 이 부호를 무시하고 절대값만 사용하여 모터가 항상 한 방향으로만 회전하는 문제.

### 변경 전

```c
void PulseControl_SetFrequency(int32_t freq_hz) {
    if (freq_hz < 0) freq_hz = -freq_hz; // 부호 정보 소실
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
        return;
    }
    // ... ARR/CCR 설정
}
```

### 변경 후

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
    // ... ARR/CCR 설정 (기존 유지)
}
```

### 영향 범위

- `position_control.c`의 `PositionControl_Update()`에서 호출하는 `PulseControl_SetFrequency(output)` 정상 동작 가능
- 기존 `pulse_forward()` / `pulse_reverse()`는 `SendSteps()`를 사용하므로 영향 없음

---

## 변경 #2: SetFrequency() PWM Start 추가 + ARR 범위 보호 (BUG-003)

**날짜:** 2026-01-31
**파일:** `Core/Src/pulse_control.c` (59~91행)
**관련 버그:** BUG-003

### 변경 이유

`SetFrequency()`가 ARR/CCR 레지스터만 변경하고 `HAL_TIM_PWM_Start_IT()`를 호출하지 않아서,
PWM이 멈춘 상태에서 호출하면 펄스가 출력되지 않는 문제.
또한 극단적인 주파수 입력 시 ARR 값이 16비트 범위를 벗어나거나 0이 되는 문제도 방지.

### 변경 전

```c
    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);
    // ← PWM Start 없음, ARR 범위 체크 없음
}
```

### 변경 후

```c
    if (arr > 0xFFFF) arr = 0xFFFF; //16비트 한계
    if (arr < 1) arr = 1;           //최소값 보장 (0 방지)

    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);

    // [추가] ARR/CCR 설정 후 PWM 시작
    if(freq_hz != 0) {HAL_TIM_PWM_Start_IT(p_htim1, TIM_CHANNEL_1);}
}
```

### 영향 범위

- PWM 정지 상태에서 `SetFrequency()`를 호출해도 정상적으로 펄스 출력 시작
- 목표 도달(output=0 → Stop) 후 새 목표 설정(output!=0) 시 모터 정상 재시작
- ARR 오버플로우/언더플로우 방지

---

## 변경 #3: SysTick 플래그 + main loop에서 PositionControl_Update() 1ms 호출 (BUG-002)

**날짜:** 2026-01-31
**파일:** `Core/Src/stm32f4xx_it.c` (44, 190행), `Core/Src/main_edit.c` (54, 123~127행)
**관련 버그:** BUG-002

### 변경 이유

`PositionControl_Update()`가 프로젝트 어디서도 호출되지 않아 PID 제어가 전혀 실행되지 않음.
1ms 주기로 호출해야 하며, ISR에서 직접 호출하면 내부 printf/HAL 블로킹 함수로 인해 위험하므로 플래그 방식 채택.

### 변경 전

```c
// stm32f4xx_it.c — SysTick_Handler
void SysTick_Handler(void) {
    HAL_IncTick();
    // ← 아무것도 없음
}

// main_edit.c — while(1) 루프
while (1) {
    // ← PositionControl_Update() 호출 없음
    pulse_forward(5000);  // 데모 코드만 존재
    ...
}
```

### 변경 후

```c
// stm32f4xx_it.c — [추가] 플래그 변수 + SysTick에서 설정
volatile uint8_t interrupt_flag = 0;

void SysTick_Handler(void) {
    HAL_IncTick();
    interrupt_flag = 1;  // 1ms마다 플래그 세움
}

// main_edit.c — [추가] extern 선언 + 플래그 확인
extern volatile uint8_t interrupt_flag;

while (1) {
    if (interrupt_flag == 1) {
        interrupt_flag = 0;
        PositionControl_Update();  // 1ms 주기 PID 제어
    }
}
```

### 영향 범위

- `PositionControl_Update()`가 1ms 주기로 실행되어 PID closed-loop 제어 가능
- SysTick ISR은 플래그만 세우므로 실행 시간 영향 최소 (수 ns)
- Update() 내부의 printf, HAL 호출은 main 컨텍스트에서 안전하게 실행
- ~~**주의:** 기존 데모 코드(pulse_forward/reverse, HAL_Delay)가 아직 남아있어 제거 필요~~ → 변경 #4에서 해결

---

## 변경 #4: main_edit.c 데모 코드 제거 + Closed-Loop 제어 루프 구성

**날짜:** 2026-02-01
**파일:** `Core/Src/main_edit.c` (116~136행)
**관련 버그:** BUG-002 (후속 작업)

### 변경 이유

while 루프 안에 데모 코드(pulse_forward, pulse_reverse, HAL_Delay, Relay_ServoOff)가 남아있어:
1. `HAL_Delay()`가 CPU를 블로킹 → 1ms PID 주기 파괴
2. `Relay_ServoOff()`가 매 루프마다 서보 전원 OFF → 제어 불가
3. open-loop 데모 코드와 closed-loop PID 코드가 충돌

### 변경 전

```c
// while 전: 아무 초기화 없음

// while 안: 데모 코드 + PID 코드 혼재
while (1) {
    if(interrupt_flag == 1) {
        interrupt_flag = 0;
        PositionControl_Update();
    }

    uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
    Relay_ServoOn();
    HAL_Delay(500);               // ← 블로킹! PID 주기 파괴
    pulse_forward(5000);          // ← open-loop, PID와 충돌
    HAL_Delay(1000);
    pulse_reverse(5000);
    HAL_Delay(1000);
    Relay_ServoOff();             // ← 매 루프마다 전원 OFF
    HAL_Delay(2000);
}
```

### 변경 후

```c
// while 전: 시스템 시동 시퀀스 (1회 실행)
Relay_ServoOn();                      // 서보 전원 ON
HAL_Delay(1000);                      // 드라이버 안정화 대기 (제어 시작 전이므로 OK)
PositionControl_SetTarget(20.0f);     // 테스트 목표 20° (±45° 범위 내)
PositionControl_Enable();             // PID 제어 활성화

// while 안: PID 제어만 실행
while (1) {
    if(interrupt_flag == 1) {
        interrupt_flag = 0;
        PositionControl_Update();     // 1ms 주기 PID closed-loop
    }
}
```

### 핵심 설계 판단

| 항목 | 변경 전 | 변경 후 | 이유 |
|------|---------|---------|------|
| Relay_ServoOn() | while 안 (매번) | while 전 (1회) | 서보 전원은 시동 시 1회만 필요 |
| HAL_Delay() | while 안 (블로킹) | while 전 (초기화용) | 제어 루프 진입 전에만 사용 가능 |
| SetTarget/Enable | 없음 / while 안 | while 전 (1회) | 목표 설정 및 활성화는 1회만 |
| 데모 코드 | while 안 | 전부 삭제 | PID closed-loop과 open-loop 공존 불가 |
| while 내부 | 데모 + PID 혼재 | PID flag 체크만 | 제어 주기 보장을 위해 최소 코드만 유지 |

### 영향 범위

- PID closed-loop 제어가 1ms 주기로 안정적으로 동작 가능
- 서보 전원이 항상 ON 유지되어 제어 중 전원 차단 문제 해결
- `HAL_Delay()` 블로킹으로 인한 PID 주기 파괴 문제 완전 해소
- 데모 테스트가 필요하면 원본 `main.c`를 빌드에 사용하면 됨

---

## 변경 #5: 미구현 함수 구현 — Group 1: 상태 읽기 Getter (BUG-004)

**날짜:** 2026-02-01
**파일:** `Core/Src/position_control.c` (148~165행)
**관련 버그:** BUG-004 (13개 미구현 함수 중 3개 해결)

### 변경 이유

`position_control.h`에 선언된 `GetTarget()`, `GetError()`, `GetPID()` 함수가 `.c`에 구현되어 있지 않음.
Phase 2 (UART 통신)에서 상태 조회 명령 처리에 필수적인 함수들이며, 내부 `static` 변수에 접근하는 유일한 경로.

### 변경 전

```c
// position_control.c — "상태 읽기" 섹션
// GetTarget(), GetError(), GetPID() 함수 본체 없음 (헤더에 선언만 존재)
```

### 변경 후

```c
// ========== 상태 읽기 ==========
float PositionControl_GetTarget(void) {
    return state.target_angle;
}
float PositionControl_GetError(void) {
    return state.error;
}
void PositionControl_GetPID(PID_Params_t* params) {
    if (params != NULL) {
        *params = pid_params;
    }
}
```

### 구현 패턴 설명

| 함수 | 반환 방식 | 패턴 |
|------|-----------|------|
| `GetTarget()` | `return state.target_angle` | 단순 값 반환 (float) |
| `GetError()` | `return state.error` | 단순 값 반환 (float) |
| `GetPID()` | `*params = pid_params` | 포인터를 통한 구조체 복사 + NULL 방어 |

**GetPID()의 NULL 체크:** 호출자가 `NULL`을 전달했을 때 역참조 크래시를 방지하는 방어적 프로그래밍. 임베디드 시스템에서 권장되는 패턴.

### 영향 범위

- Phase 2 UART 통신에서 `GetTarget()`, `GetError()`, `GetPID()` 호출 가능
- `PositionControl_GetState()`는 이미 구현되어 있었으므로, 개별 필드 접근이 필요한 경우 이 getter를 사용
- ~~나머지 10개 미구현 함수는 Groups 2~4에서 순차적으로 구현 예정~~ → 변경 #6에서 완료

---

## 변경 #6: 미구현 함수 구현 — Groups 2~4 전체 + BUG-005 해결 (BUG-004, BUG-005)

**날짜:** 2026-02-01
**파일:** `Core/Src/position_control.c`, `Core/Inc/position_control.h`
**관련 버그:** BUG-004 (나머지 10개 함수), BUG-005 (DEFAULT 값 불일치)

### 변경 이유

BUG-004의 나머지 10개 미구현 함수를 전부 구현하여 헤더와 소스의 선언/구현 불일치를 해소.
동시에 BUG-005 (헤더 DEFAULT 매크로와 소스 초기값 불일치)도 헤더 수정으로 해결.

### 변경 내용

#### A. position_control.c — 10개 함수 구현 + stats 변수 추가

**추가된 static 변수:**
```c
static PosCtrl_Stats_t stats = {0};  // 성능 통계 (line 35)
```

**Group 2: 제어 모드 (2개)**
```c
void PositionControl_SetMode(ControlMode_t mode) {
    (void)mode;  // 현재는 단일 모드만 지원
}
ControlMode_t PositionControl_GetMode(void) {
    return CTRL_MODE_POSITION;
}
```

**Group 3: 안전/통계 (4개)**
```c
void PositionControl_SetSafetyLimits(SafetyLimits_t* limits) {
    (void)limits;  // stub
}
bool PositionControl_IsSafe(void) {
    return PositionControl_CheckSafety();  // 기존 함수에 위임
}
PosCtrl_Stats_t PositionControl_GetStats(void) {
    return stats;
}
void PositionControl_ResetStats(void) {
    // stub
}
```

**Group 4: 콜백/디버그 (4개)**
```c
void PositionControl_RegisterErrorCallback(PosCtrl_ErrorCallback_t callback) {
    (void)callback;  // stub
}
void PositionControl_RegisterStableCallback(PosCtrl_StableCallback_t callback) {
    (void)callback;  // stub
}
void PositionControl_SetDebugLevel(DebugLevel_t level) {
    (void)level;  // stub
}
const char* PositionControl_GetErrorString(PosCtrl_Error_t error) {
    switch (error) {
        case POS_CTRL_OK:             return "No Error";
        case POS_CTRL_ERR_NOT_INIT:   return "Not Initialized";
        case POS_CTRL_ERR_DISABLED:   return "Control Disabled";
        case POS_CTRL_ERR_OVER_LIMIT: return "Target Out of Range";
        case POS_CTRL_ERR_ENCODER:    return "Encoder Error";
        case POS_CTRL_ERR_TIMEOUT:    return "Timeout Error";
        case POS_CTRL_ERR_SAFETY:     return "Safety Violation";
        default:                      return "Unknown Error";
    }
}
```

#### B. position_control.h — DEFAULT 값 수정 (BUG-005)

| 매크로 | 변경 전 | 변경 후 | 비고 |
|--------|---------|---------|------|
| `DEFAULT_KP` | 2.0f | 500.0f | .c 실제값과 일치 |
| `DEFAULT_KI` | 0.1f | 5.0f | .c 실제값과 일치 |
| `DEFAULT_KD` | 0.5f | 20.0f | .c 실제값과 일치 |
| `DEFAULT_INTEGRAL_LIMIT` | 10.0f | 1000.0f | .c 실제값과 일치 |
| `DEFAULT_OUTPUT_LIMIT` | 100.0f | 10000.0f | .c 실제값과 일치 |

### 구현 방식 분류

| 유형 | 함수 | 설명 |
|------|------|------|
| **stub** (8개) | SetMode, SetSafetyLimits, ResetStats, RegisterErrorCallback, RegisterStableCallback, SetDebugLevel | `(void)param` 처리, Phase 2 이후 본구현 예정 |
| **위임** (1개) | IsSafe → CheckSafety | 기존 함수 재활용 |
| **상수 반환** (1개) | GetMode → CTRL_MODE_POSITION | 현재 위치 제어 모드 고정 |
| **완전 구현** (1개) | GetErrorString | 7개 에러코드 전부 커버 (switch-case + 문자열 리터럴 반환) |

### 영향 범위

- position_control.h에 선언된 26개 함수가 전부 .c에 구현됨 → **BUG-004 완전 해결**
- 헤더 DEFAULT 값이 .c 초기값과 일치 → **BUG-005 해결**
- stub 함수는 호출해도 안전 (크래시 없음), Phase 2 이후 본구현으로 교체 가능
- `GetErrorString()`은 UART 통신 에러 응답에서 바로 사용 가능
