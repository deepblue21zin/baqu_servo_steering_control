# 버그 리포트 및 해결 이력 (problem.md)

> servo_control_baqu 프로젝트에서 발견된 문제점과 해결 과정을 기록합니다.
> 각 항목에 **문제 발견 경위 → 원인 분석 → 해결 방법 → 검증**을 체계적으로 정리합니다.
> ADAS/임베디드 산업 기준(ISO 26262, MISRA-C, IEC 61508)에 맞춰 심각도를 분류합니다.

---

## 목차

| # | 제목 | 파일 | 심각도 | 상태 |
|---|------|------|--------|------|
| BUG-001 | SetFrequency() 방향 핀 미설정 | pulse_control.c | **Critical** | **해결 (2026-01-31)** |
| BUG-002 | PositionControl_Update() 호출 부재 | main.c / stm32f4xx_it.c | **Critical** | **해결 (2026-01-31)** |
| BUG-003 | SetFrequency() PWM 시작 누락 | pulse_control.c | **Critical** | **해결 (2026-01-31)** |
| BUG-004 | position_control.h 13개 함수 미구현 | position_control.c/h | **Major** | **해결 (2026-02-01)** |
| BUG-005 | PID DEFAULT 헤더/소스 불일치 | position_control.c/h | **Minor** | **해결 (2026-02-01)** |
| BUG-006 | 안정화 시간 측정 부정확 | position_control.c | **Minor** | **해결 (2026-02-21)** |
| BUG-007 | ethernet_communication 모듈 부재 | ethernet_communication.c/h | **Major** | **해결 (2026-02-21, 골격)** |
| **BUG-008** | **PA1 핀 충돌 (TIM2 vs ETH RMII)** | **encoder_reader.c, .ioc** | **Critical** | **해결 (2026-02-21)** |
| **BUG-009** | **control_enabled 중복 선언 + volatile 누락** | **position_control.c** | **Critical** | **해결 (2026-02-21)** |
| **BUG-010** | **EmergencyStop에서 릴레이 미작동** | **position_control.c** | **Critical** | **해결 (2026-02-21)** |
| **BUG-011** | **DIR_PIN_GPIO_PORT 대소문자 불일치** | **pulse_control.c** | **Major** | **해결 (2026-02-21)** |
| **BUG-012** | **debug_cnt 타이밍 오류** | **main.c** | **Major** | **해결 (2026-02-21)** |
| **BUG-013** | **USER CODE 블록 위치 오류** | **main.c** | **Major** | **해결 (2026-02-21)** |
| **BUG-014** | **MX_LWIP_Init() IWDG 충돌 위험** | **main.c** | **Major** | **해결 (2026-02-21, 주석처리)** |
| **NEW-001** | **이더넷 통신 LwIP UDP 미구현** | **ethernet_communication.c** | **Major** | **미해결 (Phase 2)** |
| **NEW-002** | **통신 워치독 미구현** | **미정** | **Major** | **미해결 (Phase 2)** |
| **NEW-003** | **시스템 상태 머신 미구현** | **미정** | **Major** | **미해결 (Phase 3)** |
| **NEW-004** | **상수 중복 정의** | **constants.h, position_control.h** | **Minor** | **미해결** |

---

## BUG-001 ~ BUG-007: 이전 해결 이력

> 2026-01-31 ~ 2026-02-01 사이에 해결된 항목입니다.
> 상세 내용은 이전 커밋 기록 또는 [change_code.md](change_code.md) 변경 #1~#7을 참조하세요.

| # | 요약 | 근본 원인 | 해결 방법 |
|---|------|-----------|----------|
| BUG-001 | SetFrequency에서 방향 핀 미설정 → 단방향 회전 | 부호 정보 무시 | freq_hz 부호로 DIR 핀 설정 |
| BUG-002 | PositionControl_Update() 미호출 → PID 미실행 | 호출 코드 부재 | SysTick flag + main loop polling |
| BUG-003 | SetFrequency에서 PWM Start 누락 → 펄스 미출력 | HAL_TIM_PWM_Start 누락 | ARR 설정 후 PWM Start 추가 |
| BUG-004 | 13개 함수 헤더 선언만 존재 → 링커 에러 위험 | 설계/구현 비동기화 | 전체 구현 (stub 포함) |
| BUG-005 | PID DEFAULT 매크로 vs 실제 초기값 불일치 | 튜닝 후 헤더 미갱신 | 헤더 값을 실제값으로 통일 |
| BUG-006 | stable_time_ms++ 방식 → 실제 시간 미반영 | 1ms 가정 의존 | dt * 1000.0f 방식으로 변경 |
| BUG-007 | ethernet_communication.c가 0바이트 | 미구현 | API 구조 + 파싱/디스패치 골격 작성 |

---

## BUG-008: PA1 핀 충돌 (TIM2 Encoder vs ETH RMII)

**심각도:** Critical
**상태:** **해결 완료 (2026-02-21)**
**파일:** `Core/Src/encoder_reader.c`, `servo_control_baqu.ioc`

### 문제 발견 경위

CubeMX에서 ETH RMII 활성화 시도 → PA1 핀이 TIM2_CH2(Encoder B)에 이미 할당 → RMII 활성화 불가.
PA1은 STM32F429의 ETH_RMII_REF_CLK 전용 핀으로, 대체 핀 없음 (Alternate Function 고정).

### 원인 분석

- 초기 설계 시 이더넷 통신이 고려되지 않아 PA0/PA1을 엔코더에 할당
- STM32F429ZI에서 PA1은 ETH RMII의 REF_CLK로 고정되어 있어 핀 재배치 불가
- TIM2 엔코더 채널을 다른 핀으로 Remap하는 것은 가능하나, TIM4가 더 적합

### 해결 방법

| 항목 | TIM2 (변경 전) | TIM4 (변경 후) |
|------|---------------|---------------|
| 핀 | PA0, PA1 | PD12, PD13 |
| 비트 폭 | 32비트 | 16비트 |
| Counter 범위 | 0 ~ 4,294,967,295 | 0 ~ 65,535 |
| 오버플로우 대책 | 불필요 | Center-offset (32768 기준) |
| 유효 범위 | ±2.1B 카운트 | ±32,767 카운트 (±245°) |

### 산업 기준 코멘트

- **핀 할당 충돌 방지**: 자동차 ECU 설계에서는 Pin Mux Matrix를 프로젝트 초기에 확정하고 문서화해야 함
- **16비트 타이머 제약 대응**: ±45° 조향 범위에서 48,000 counts/rev 기준 최대 ±6,000 카운트 필요 → 32,767 범위로 충분
- **Center-offset 패턴**: 16비트 타이머의 표준 엔코더 처리 방식으로, 양/음 방향 모두 균등한 범위 확보

### 검증

- CubeMX에서 PA1 → ETH_RMII_REF_CLK 정상 할당
- TIM4 Encoder Mode 정상 동작 (PD12/PD13)
- 빌드 에러 없음

---

## BUG-009: control_enabled 중복 선언 + volatile 누락 + 임계영역 부재

**심각도:** Critical
**상태:** **해결 완료 (2026-02-21)**
**파일:** `Core/Src/position_control.c`

### 문제 코드

```c
// line 38 — 원본
static bool control_enabled = false;

// line 228 — 중복! (volatile 추가하려다 새로 선언)
static volatile bool control_enabled = false;

// SetTarget — 임계영역 보호 없음
int PositionControl_SetTarget(float target_deg) {
    state.target_angle = target_deg;  // ISR에서 동시 읽기 가능 → race condition
    // ...
}
```

### 영향

1. **빌드 에러**: `conflicting types for 'control_enabled'` → 컴파일 불가
2. **volatile 누락**: 컴파일러 최적화로 ISR에서 변경한 값을 main에서 감지 못할 수 있음
3. **Race condition**: SetTarget()에서 target_angle 변경 중 PID Update()가 읽으면 불완전한 값 사용

### 해결 방법

1. 중복 선언 제거 (line 228 삭제)
2. 원본에 `volatile` 추가: `static volatile bool control_enabled = false;`
3. SetTarget()에 `__disable_irq()` / `__enable_irq()` 임계영역 추가

### 산업 기준 코멘트 (MISRA-C / ISO 26262)

| 규칙 | 위반 내용 | 해결 후 |
|------|-----------|---------|
| MISRA-C Rule 8.9 | 변수 중복 정의 | 단일 정의로 수정 |
| MISRA-C Rule 2.2 | ISR 공유 변수 volatile 누락 | volatile 추가 |
| ISO 26262 Part 6 | 공유 리소스 상호 배제 없음 | __disable_irq/__enable_irq 추가 |

---

## BUG-010: EmergencyStop에서 릴레이 비상정지 미작동

**심각도:** Critical
**상태:** **해결 완료 (2026-02-21)**
**파일:** `Core/Src/position_control.c`

### 문제 발견 경위

`PositionControl_EmergencyStop()`이 호출되면 PulseControl_Stop()만 실행되고,
하드웨어 비상정지 릴레이(EMG, PD15)는 작동하지 않음.

### 영향 (ADAS 관점에서 치명적)

- **소프트웨어 크래시 시**: PulseControl_Stop()은 소프트웨어 레벨 → 크래시하면 무효
- **하드웨어 레벨 미보호**: 모터에 전력이 계속 공급 → 제어 불능 상태에서 조향 가능
- **ISO 26262 ASIL-D 위반**: 조향 시스템은 단일 고장에도 안전 상태 도달 보장 필요

### 해결 방법

```c
#include "relay_control.h"  // 의존성 추가

void PositionControl_EmergencyStop(void) {
    control_enabled = false;
    PulseControl_Stop();           // 소프트웨어 레벨 정지
    pid_state.integral = 0.0f;
    printf("[PosCtrl] EMERGENCY STOP!\n");
    Relay_Emergency();             // 하드웨어 레벨 비상정지 (추가)
}
```

---

## BUG-011: DIR_PIN_GPIO_PORT 대소문자 불일치

**심각도:** Major
**상태:** **해결 완료 (2026-02-21)**
**파일:** `Core/Src/pulse_control.c` (4개소)

### 문제 코드

```c
// 코드에서 사용
HAL_GPIO_WritePin(DIR_PIN_GPIO_PORT, DIR_PIN_Pin, ...);  // 'PORT' (대문자)

// CubeMX가 생성한 실제 매크로 (main.h)
#define DIR_PIN_GPIO_Port GPIOE  // 'Port' (소문자 ort)
```

### 해결

`DIR_PIN_GPIO_PORT` → `DIR_PIN_GPIO_Port` (4개소 전부 replace_all)

---

## BUG-012: debug_cnt 타이밍 오류

**심각도:** Major
**상태:** **해결 완료 (2026-02-21)**
**파일:** `Core/Src/main.c`

### 문제

`++debug_cnt`가 `interrupt_flag` 블록 밖에 위치 → while(1) 루프 속도(수 MHz)로 카운트.
100ms 주기 의도가 수 us 주기로 변질 → UART printf가 초당 수만 회 호출 → PID 주기 파괴.

### 해결

`++debug_cnt`를 `interrupt_flag` 블록 내부로 이동 → 정확히 1ms마다 카운트 → 100회 = 100ms 주기.

---

## BUG-013: USER CODE 블록 위치 오류 (CubeMX 코드 손실)

**심각도:** Major
**상태:** **해결 완료 (2026-02-21)**
**파일:** `Core/Src/main.c`

### 문제

while 루프 내 사용자 코드가 `USER CODE END WHILE`과 `USER CODE BEGIN 3` 사이에 위치.
CubeMX 코드 재생성 시 이 영역이 **자동 삭제**되어 PID 루프, IWDG 리프레시 등 모든 기능 소실.

### 해결

모든 while 루프 내 사용자 코드를 `/* USER CODE BEGIN 3 */` ~ `/* USER CODE END 3 */` 블록 안으로 이동.

### 산업 기준 코멘트

- 코드 생성기(CubeMX) 사용 시 **보호 영역 규칙**을 팀 전체가 숙지해야 함
- CI/CD에 CubeMX 재생성 → 빌드 → 테스트 파이프라인을 구축하면 이런 사고를 조기 발견 가능

---

## BUG-014: MX_LWIP_Init() IWDG 충돌 위험

**심각도:** Major
**상태:** **해결 완료 (2026-02-21, 테스트 모드에서 주석처리)**
**파일:** `Core/Src/main.c`

### 문제

`MX_IWDG_Init()` (line 106) 이후에 `MX_LWIP_Init()` (line 108) 호출.
이더넷 케이블 미연결 시 PHY 초기화가 지연될 수 있고, IWDG 타임아웃(~32초) 내에 완료 못하면 MCU 리셋.

### 현재 해결

```c
// MX_LWIP_Init();  // 이더넷 테스트 시 활성화
```

### 향후 해결 방향 (Phase 2)

1. `MX_LWIP_Init()`을 `MX_IWDG_Init()` 전으로 이동
2. 또는 PHY 초기화 중에도 IWDG 리프레시하는 커스텀 초기화 함수 작성

---

## 미해결 이슈

### NEW-001: 이더넷 통신 LwIP UDP 미구현

**심각도:** Major
**상태:** 미해결 (Phase 2 예정)
**파일:** `Core/Src/ethernet_communication.c`

- CubeMX에서 ETH + LwIP 설정 완료 (UDP Enabled, Static IP 192.168.1.100)
- `ethernet_communication.c`는 text-based stub 상태
- `MX_LWIP_Process()`가 main while 루프에 미추가
- LwIP UDP API (`udp_new`, `udp_bind`, `udp_recv`, `udp_sendto`)로 재구현 필요

### NEW-002: 통신 워치독 미구현

**심각도:** Major
**상태:** 미해결 (Phase 2 예정)

- 상위 제어기(Jetson/PC)와의 통신 두절 시 안전 대응 없음
- 일정 시간(예: 500ms) 명령 미수신 → 자동 비상정지 필요
- ADAS 조향 시스템에서 **필수** 안전 기능 (통신 실패 = 제어 불능)

### NEW-003: 시스템 상태 머신 미구현

**심각도:** Major
**상태:** 미해결 (Phase 3 예정)

- 현재: Init → Enable → Running (단순 흐름)
- 필요: IDLE → HOMING → READY → RUNNING → ERROR → EMERGENCY (상태 전이)
- ISO 26262: 시스템 상태와 전이 조건이 명확히 정의되어야 함

### NEW-004: 상수 중복 정의

**심각도:** Minor
**상태:** 미해결

- `CONTROL_PERIOD_MS`, `POSITION_TOLERANCE` 가 `constants.h`와 `position_control.h`에 중복
- `MAX_ANGLE_DEG` / `MIN_ANGLE_DEG` vs `MAX_STEERING_ANGLE` / `MIN_STEERING_ANGLE` 이름 불일치
- 값은 동일하지만, 향후 한쪽만 수정하면 불일치 발생 위험
- **해결 방향**: `constants.h`를 single source of truth로 통합

---

## 버그 심각도 기준 (ADAS/임베디드)

| 심각도 | 정의 | 예시 |
|--------|------|------|
| **Critical** | 시스템 안전에 직접적 위협, 즉시 수정 필수 | 비상정지 미작동, 제어 루프 미실행, 빌드 불가 |
| **Major** | 기능 장애 또는 데이터 무결성 위협 | CubeMX 코드 손실, 디버그 출력 폭주, 통신 미구현 |
| **Minor** | 기능에 영향 적으나 코드 품질/유지보수성 저하 | 상수 중복, 주석 오류, 미사용 변수 |

---

*마지막 업데이트: 2026-02-21*
