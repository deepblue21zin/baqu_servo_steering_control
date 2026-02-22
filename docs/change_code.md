# 코드 변경 이력 (change_code.md)

> position_control 통합 작업 중 발생하는 모든 코드 변경을 기록합니다.
> 각 항목에 **변경 이유**, **변경 전/후 코드**, **영향 범위**를 명시합니다.

---

## 변경 이력 요약

| # | 날짜 | 파일 | 내용 | 심각도 | 상태 |
|---|------|------|------|--------|------|
| 1 | 2026-01-31 | pulse_control.c | SetFrequency() 방향 핀 버그 수정 | Critical | **완료** |
| 2 | 2026-01-31 | pulse_control.c | SetFrequency() PWM Start 추가 + ARR 보호 | Critical | **완료** |
| 3 | 2026-01-31 | stm32f4xx_it.c, main.c | 1ms 주기 PositionControl_Update() 호출 추가 | Critical | **완료** |
| 4 | 2026-02-01 | main.c | 데모 루프 → closed-loop 제어 루프 교체 | Critical | **완료** |
| 5 | 2026-02-01 | position_control.c | 미구현 함수 구현 (GetTarget, GetError 등) | Major | **완료** |
| 6 | 2026-02-01 | position_control.c/h | Groups 2~4 전체 + DEFAULT 값 통일 | Major | **완료** |
| 7 | 2026-02-21 | ethernet_communication.c/h | 통신 모듈 골격 신규 작성 | Major | **완료** |
| **8** | **2026-02-21** | **encoder_reader.c** | **TIM2→TIM4 엔코더 이관 (PA1 핀 충돌 해결)** | **Critical** | **완료** |
| **9** | **2026-02-21** | **position_control.c** | **control_enabled 중복 제거 + volatile + 임계영역** | **Critical** | **완료** |
| **10** | **2026-02-21** | **position_control.c** | **EmergencyStop에서 Relay_Emergency() 호출 추가** | **Critical** | **완료** |
| **11** | **2026-02-21** | **pulse_control.c** | **DIR_PIN_GPIO_PORT 대소문자 수정** | **Major** | **완료** |
| **12** | **2026-02-21** | **main.c** | **debug_cnt 위치 수정 + USER CODE 블록 정리** | **Major** | **완료** |
| **13** | **2026-02-21** | **CubeMX (.ioc)** | **ETH RMII + LwIP + IWDG + TIM4 Encoder 설정** | **Major** | **완료** |
| **14** | **2026-02-21** | **main.c** | **초기화 순서 개선 + 테스트 모드 구성** | **Major** | **완료** |

---

## 변경 #1 ~ #7: 이전 변경 이력

> 2026-01-31 ~ 2026-02-21 사이의 변경 내용은 이전 커밋 기록을 참조하세요.
> 주요 내용: SetFrequency 방향 핀 수정, PWM Start 추가, SysTick 플래그 방식 채택,
> 데모 코드 제거, 미구현 함수 13개 전부 구현, PID DEFAULT 값 통일, 이더넷 골격 작성.

---

## 변경 #8: TIM2 → TIM4 엔코더 이관 (PA1 핀 충돌 해결)

**날짜:** 2026-02-21
**파일:** `Core/Src/encoder_reader.c`, `Core/Src/main.c`, CubeMX `.ioc`
**관련 버그:** BUG-008
**심각도:** Critical

### 변경 이유

ETH RMII 활성화에 필요한 PA1 핀(ETH_RMII_REF_CLK)이 TIM2 Encoder CH2(PA1)와 충돌.
이더넷 통신을 위해 엔코더를 TIM2(PA0/PA1, 32비트)에서 TIM4(PD12/PD13, 16비트)로 이관.

### 산업 기준 관점 (ADAS/임베디드)

- **핀 자원 관리**: 자동차 ECU에서는 핀 할당 충돌이 설계 초기에 Pin Mux Table로 관리됨
- **16비트 타이머 제약**: 32비트→16비트 변경 시 오버플로우 범위가 4,294,967,295 → 65,535로 축소
- **대응 전략**: Center-offset 방식(32768 기준)으로 ±32,767 카운트 범위 확보

### 변경 전

```c
// encoder_reader.c
#include "tim.h"
extern TIM_HandleTypeDef htim2;

int EncoderReader_Init(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // 32비트: 0~4,294,967,295
    // ...
}

float EncoderReader_GetAngleDeg(void) {
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);  // 직접 읽기
    return (float)count * DEG_PER_COUNT;
}
```

```c
// main.c
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // TIM2 사용
```

### 변경 후

```c
// encoder_reader.c
#define ENCODER_TIMER htim4
#define ENCODER_MAX_COUNT 65535  // 16비트

int EncoderReader_Init(void) {
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, 32768);  // 중간값 시작 (오버플로우 여유)
    // ...
}

float EncoderReader_GetAngleDeg(void) {
    uint16_t raw = __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    encoder_count = (int32_t)raw - 32768;  // -32768 ~ +32767 범위로 변환
    int32_t adjusted_count = encoder_count - encoder_offset;
    return (float)adjusted_count * DEG_PER_COUNT;
}
```

```c
// main.c
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // TIM4 사용
```

### CubeMX 변경 사항

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| 엔코더 타이머 | TIM2 (PA0, PA1) | TIM4 (PD12, PD13) |
| 타이머 비트 폭 | 32비트 | 16비트 |
| PA1 핀 용도 | TIM2_CH2 (Encoder B) | ETH_RMII_REF_CLK |
| Counter Period | 0xFFFFFFFF | 0xFFFF (65535) |
| Encoder Mode | TI1 and TI2 | TI1 and TI2 (동일) |

### 영향 범위

- 엔코더 유효 범위: ±32,767 카운트 = ±245.75° (±45° 조향 범위 충분)
- PA1이 ETH RMII에 사용 가능해져 이더넷 통신 기능 활성화
- `GetCount()`, `Reset()`, `SetOffset()` 모두 TIM4 기반으로 통일

---

## 변경 #9: control_enabled 중복 제거 + volatile + 임계영역

**날짜:** 2026-02-21
**파일:** `Core/Src/position_control.c`
**관련 버그:** BUG-009
**심각도:** Critical

### 변경 이유

1. `control_enabled` 변수가 line 38과 line 228에 **중복 선언**되어 빌드 에러 발생
2. ISR(SysTick)에서 flag를 설정하고 main에서 읽는 구조에서 `volatile` 없으면 컴파일러 최적화로 값 누락 가능
3. `SetTarget()`에서 `state.target_angle` 변경 시 ISR에서 동시 읽기 → race condition 가능

### 산업 기준 관점 (ISO 26262 / MISRA-C)

- **MISRA-C Rule 8.9**: 변수는 한 번만 정의되어야 함 (중복 정의 금지)
- **MISRA-C Rule 2.2**: ISR과 main 간 공유 변수는 반드시 `volatile` 선언
- **ISO 26262 Part 6**: 공유 리소스 접근 시 상호 배제(mutual exclusion) 메커니즘 필요

### 변경 전

```c
// line 38
static bool control_enabled = false;

// line 228 (중복!)
static volatile bool control_enabled = false;

// SetTarget
int PositionControl_SetTarget(float target_deg) {
    state.target_angle = target_deg;  // 임계영역 보호 없음
    state.is_stable = false;
    state.stable_time_ms = 0;
    return POS_CTRL_OK;
}
```

### 변경 후

```c
// line 39 (단일 선언, volatile 추가)
static volatile bool control_enabled = false;

// SetTarget (임계영역 보호 추가)
int PositionControl_SetTarget(float target_deg) {
    if (target_deg > MAX_ANGLE_DEG || target_deg < MIN_ANGLE_DEG) {
        return POS_CTRL_ERR_OVER_LIMIT;
    }
    __disable_irq();             // 인터럽트 비활성화
    state.target_angle = target_deg;
    state.is_stable = false;
    state.stable_time_ms = 0;
    __enable_irq();              // 인터럽트 재활성화
    return POS_CTRL_OK;
}
```

### 영향 범위

- 빌드 에러(conflicting types) 해결
- 런타임 race condition 방지 (target_angle 동시 접근)
- 상위 제어기에서 SetTarget() 호출 시 PID 루프와 안전하게 동기화

---

## 변경 #10: EmergencyStop에서 Relay_Emergency() 호출 추가

**날짜:** 2026-02-21
**파일:** `Core/Src/position_control.c`
**관련 버그:** BUG-010
**심각도:** Critical

### 변경 이유

`EmergencyStop()`이 PWM 정지 + PID 리셋만 수행하고, 실제 하드웨어 비상정지(릴레이)를 작동시키지 않음.
소프트웨어만 멈추고 하드웨어는 멈추지 않는 **치명적 안전 결함**.

### 산업 기준 관점

- **ISO 26262 (Functional Safety)**: 안전 관련 기능은 소프트웨어 + 하드웨어 양쪽에서 모두 작동해야 함
- **SIL/ASIL 요구사항**: 단일 고장(소프트웨어 크래시)에도 하드웨어 레벨에서 안전 상태 유지 필요
- **ADAS 조향 시스템**: 비상정지 시 모터 전원 차단은 최소 요구사항

### 변경 전

```c
void PositionControl_EmergencyStop(void) {
    control_enabled = false;
    PulseControl_Stop();
    pid_state.integral = 0.0f;
    printf("[PosCtrl] EMERGENCY STOP!\n");
    // ← 하드웨어 비상정지 없음!
}
```

### 변경 후

```c
#include "relay_control.h"  // 추가

void PositionControl_EmergencyStop(void) {
    control_enabled = false;
    PulseControl_Stop();
    pid_state.integral = 0.0f;
    printf("[PosCtrl] EMERGENCY STOP!\n");
    Relay_Emergency();  // 하드웨어 비상정지 릴레이 작동 (EMG=LOW)
}
```

### 영향 범위

- 안전 검사(CheckSafety) 실패 시 소프트웨어 + 하드웨어 모두 정지
- EMG 릴레이(PD15)가 LOW로 전환 → 서보 드라이버 비상정지 작동
- `relay_control.h` include 추가로 모듈 간 의존성 명시화

---

## 변경 #11: DIR_PIN_GPIO_PORT 대소문자 수정

**날짜:** 2026-02-21
**파일:** `Core/Src/pulse_control.c` (4개소)
**관련 버그:** BUG-011
**심각도:** Major

### 변경 이유

CubeMX가 생성하는 매크로 이름은 `DIR_PIN_GPIO_Port` (소문자 `ort`)인데,
코드에서는 `DIR_PIN_GPIO_PORT` (대문자 `ORT`)로 사용 → 빌드 에러.

### 산업 기준 관점

- **MISRA-C Rule 1.1**: 모든 코드는 컴파일 가능해야 함 (기본 요구사항)
- **CubeMX 코드 생성**: HAL 코드 생성기의 네이밍 컨벤션을 사용자 코드에서도 따라야 함
- **팀 컨벤션**: CubeMX User Label 규칙 (`<Label>_Pin`, `<Label>_GPIO_Port`)을 팀 전체가 숙지해야 함

### 변경 전

```c
HAL_GPIO_WritePin(DIR_PIN_GPIO_PORT, DIR_PIN_Pin, GPIO_PIN_RESET);  // 4개소 모두
```

### 변경 후

```c
HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_RESET);  // 4개소 모두
```

---

## 변경 #12: debug_cnt 위치 수정 + USER CODE 블록 정리

**날짜:** 2026-02-21
**파일:** `Core/Src/main.c`
**관련 버그:** BUG-012, BUG-013
**심각도:** Major

### 변경 이유

**문제 1 (debug_cnt):** `++debug_cnt`가 while(1) 루프의 외부(interrupt_flag 블록 밖)에 있어서
1ms가 아닌 CPU 속도(MHz)로 카운트 → `PositionControl_PrintStatus()`가 초당 수천~수만 회 호출.

**문제 2 (USER CODE 블록):** while 루프 코드가 `USER CODE END WHILE`과 `USER CODE BEGIN 3` 사이에 위치 →
CubeMX 코드 재생성 시 사용자 코드가 삭제됨.

### 산업 기준 관점

- **CubeMX 관리 원칙**: 사용자 코드는 반드시 `USER CODE BEGIN/END` 블록 안에 위치해야 함
- **디버그 출력 제어**: UART printf는 블로킹이므로 과도한 호출은 실시간 제어 주기를 파괴

### 변경 전

```c
while (1) {
    /* USER CODE END WHILE */
    if (interrupt_flag) {
        interrupt_flag = 0;
        PositionControl_Update();
    }
    // debug_cnt가 interrupt_flag 블록 밖에 있음!
    if (++debug_cnt >= 100) {  // CPU 속도로 카운트 (MHz)
        debug_cnt = 0;
        PositionControl_PrintStatus();  // 초당 수만 회 호출!
    }
    HAL_IWDG_Refresh(&hiwdg);
    /* USER CODE BEGIN 3 */
    // ← 코드가 여기 없으면 CubeMX 재생성 시 위 코드 전부 삭제
}
```

### 변경 후

```c
while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // PID 제어 (1ms 주기, SysTick interrupt_flag 기반)
    if (interrupt_flag) {
        interrupt_flag = 0;
        PositionControl_Update();

        // 디버그 출력 (100ms 주기 = 1ms × 100회)
        if (++debug_cnt >= 100) {  // interrupt_flag 블록 내부 (정확히 1ms마다)
            debug_cnt = 0;
            PositionControl_PrintStatus();
        }
    }

    // 워치독 리프레시
    HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END 3 */
```

### 영향 범위

- PrintStatus 호출 주기: **수만 Hz → 정확히 10Hz (100ms)** 로 정상화
- CubeMX 재생성 시 사용자 코드 보존 보장
- UART 블로킹으로 인한 PID 주기 파괴 문제 해결

---

## 변경 #13: CubeMX 설정 — ETH RMII + LwIP + IWDG + TIM4 Encoder

**날짜:** 2026-02-21
**파일:** `servo_control_baqu.ioc`
**심각도:** Major

### 변경 이유

1. **이더넷 통신 기반 구축**: 상위 제어기(Jetson/PC)와 UDP 통신을 위해 ETH RMII + LwIP 필요
2. **워치독**: MCU 행(hang) 시 자동 리셋을 위한 IWDG 필요 (ADAS 안전 요구사항)
3. **엔코더 이관**: PA1 핀 충돌로 TIM2→TIM4 변경

### CubeMX 변경 내역

| 카테고리 | 항목 | 설정 값 |
|----------|------|---------|
| **ETH** | Mode | RMII |
| | RMII 핀 | PA1(REF_CLK), PA2(MDIO), PA7(CRS_DV), PB13(TXD1), PC1(MDC), PC4(RXD0), PC5(RXD1), PG11(TX_EN), PG13(TXD0) |
| | NVIC | Ethernet global interrupt = Enabled, Priority 5 |
| **LwIP** | DHCP | Disabled |
| | IP Address | 192.168.1.100 |
| | Netmask | 255.255.255.0 |
| | Gateway | 192.168.1.1 |
| | LWIP_UDP | Enabled |
| | LWIP_TCP | Disabled |
| | MEM_SIZE | 8192 |
| | PBUF_POOL_BUFSIZE | 1524 |
| | PHY Driver | LAN8742 (Platform Settings 탭) |
| **IWDG** | Prescaler | 256 |
| | Reload | 4095 |
| | 타임아웃 | ~32초 |
| **TIM4** | Mode | Encoder Mode TI1 and TI2 |
| | Channel 1 | PD12 (Encoder A) |
| | Channel 2 | PD13 (Encoder B) |
| | Counter Period | 65535 (0xFFFF) |
| **TIM1** | CC IRQ | Enabled, Priority 1 |
| **Heap** | Size | 0x2000 (8KB) |
| **Stack** | Size | 0x1000 (4KB) |

### NVIC 우선순위 체계

| 인터럽트 | Preemption Priority | 용도 |
|----------|-------------------:|------|
| SysTick | 15 (최저) | 1ms tick + PID flag |
| TIM1_CC | 1 | 펄스 카운팅 (스텝 모드) |
| ETH | 5 | 이더넷 수신 처리 |

---

## 변경 #14: main.c 초기화 순서 개선 + 테스트 모드 구성

**날짜:** 2026-02-21
**파일:** `Core/Src/main.c`
**심각도:** Major

### 변경 이유

1. **TIM1 PWM Start 순서**: PulseControl_Init() 전에 PWM이 시작되어 초기화 전 펄스 출력 가능
2. **MX_LWIP_Init() 블로킹**: 이더넷 케이블 미연결 시 PHY 초기화 지연 → IWDG 타임아웃 위험
3. **Homing 실패 → while(1)**: IWDG 리프레시 불가 → 무한 리셋 루프
4. **테스트 안전**: 첫 테스트에서 큰 각도 이동은 위험

### 변경 전 (초기화 순서)

```c
// 문제 있는 순서
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PWM 시작 (Init 전)
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

Relay_Init();
PulseControl_Init();       // ← PWM이 이미 시작된 상태에서 Init
EncoderReader_Init();
PositionControl_Init();

// ...
MX_LWIP_Init();            // ← 이더넷 없으면 블로킹 가능
// ...
Homing_Init();
ADC_Pot_Init(NULL);
Homing_FindZero();
if (!Homing_IsComplete()) {
    printf("[Main] Homing failed!\n");
    while (1);              // ← IWDG 리프레시 없음 → 리셋 루프
}
PositionControl_SetTarget(20.0f);  // ← 첫 테스트에 20도는 위험할 수 있음
```

### 변경 후 (테스트 모드)

```c
// 안전한 순서
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 엔코더 먼저

Relay_Init();
PulseControl_Init();                               // Init 먼저
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);         // 그 다음 PWM 시작
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
EncoderReader_Init();
PositionControl_Init();

Relay_ServoOn();
HAL_Delay(1000);

// MX_LWIP_Init();  // 이더넷 테스트 전이므로 주석처리
// Homing 블록 전체 주석처리 (테스트 모드)
EncoderReader_Reset();                             // 현재 위치를 0도 기준으로
PositionControl_SetTarget(10.0f);                  // 안전한 각도 (10도)
PositionControl_Enable();
```

### 초기화 순서 비교

| 순서 | 변경 전 | 변경 후 | 이유 |
|------|---------|---------|------|
| 1 | PWM Start | Encoder Start | 엔코더는 읽기 전용이므로 먼저 시작해도 안전 |
| 2 | Encoder Start | Relay_Init + PulseControl_Init | 하드웨어 제어 초기화를 PWM 전에 |
| 3 | Relay/Pulse/Encoder Init | PWM Start | Init 완료 후 PWM 시작 |
| 4 | MX_LWIP_Init (활성) | MX_LWIP_Init (주석) | 이더넷 테스트 전까지 비활성화 |
| 5 | Homing 시퀀스 (활성) | Homing (주석) + EncoderReader_Reset | 테스트 모드: 현재 위치=0 |
| 6 | SetTarget(20.0) | SetTarget(10.0) | 안전한 테스트 각도 |

### 산업 기준 관점

- **초기화 의존성 순서**: 하위 드라이버 → 모듈 Init → 하드웨어 Start 순서가 올바름
- **단계적 기능 활성화**: 모터 테스트 → 이더넷 테스트 → 통합 테스트 순서로 위험 최소화
- **워치독 보호**: 모든 실행 경로에서 IWDG 리프레시가 보장되어야 함

---

*마지막 업데이트: 2026-02-21*
