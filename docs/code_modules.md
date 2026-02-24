# 코드 모듈별 상세 설명서

> 작성일: 2026-02-24
> 대상: 코드를 처음 보는 팀원도 이해할 수 있도록 작성
> 하드웨어: STM32 NUCLEO-F429ZI + LS L7 서보드라이브 + 서보모터

---

## 목차

1. [전체 모듈 관계도](#전체-모듈-관계도)
2. [실행 흐름 (전원 ON부터 모터 회전까지)](#실행-흐름)
3. [모듈 상세 설명](#모듈-상세-설명)
   - [main.c - 진입점 및 메인 루프](#mainc---진입점-및-메인-루프)
   - [stm32f4xx_it.c - 인터럽트 처리기](#stm32f4xx_itc---인터럽트-처리기)
   - [tim.c - 타이머 설정](#timc---타이머-설정)
   - [encoder_reader.c - 엔코더 읽기](#encoder_readerc---엔코더-읽기)
   - [pulse_control.c - 펄스 출력 제어](#pulse_controlc---펄스-출력-제어)
   - [position_control.c - PID 위치 제어](#position_controlc---pid-위치-제어)
   - [relay_control.c - 릴레이 제어](#relay_controlc---릴레이-제어)
   - [ethernet_communication.c - 이더넷 통신](#ethernet_communicationc---이더넷-통신)
   - [lwip.c - LwIP 네트워크 스택](#lwipc---lwip-네트워크-스택)

---

## 2026-02-24 최신 반영 사항

아래 항목은 현재 코드와 문서를 정합화하기 위한 최신 변경입니다.

1. `main.c` 모드 처리
- `AUTO`: PC 9B steer 기반 목표각 적용
- `MANUAL`: ASMS 5B joy_y 기반 목표각 적용
- `NONE`: 제어 비활성화
- `ESTOP`: 비상정지 진입

2. 통신 타임아웃 fail-safe
- `ETHCOMM_RX_TIMEOUT_MS`(현재 300ms) 초과 시 `AUTO/MANUAL -> ESTOP` 강제 전이

3. 제어 모드 진단값 정합
- `PositionControl_GetMode()`가 고정값이 아닌 실제 내부 상태(`IDLE/POSITION/EMERGENCY`)를 반환

4. 비상정지 하드웨어 경로 복구
- `PositionControl_EmergencyStop()`에서 `Relay_Emergency()` 호출
- 복귀 시 `Relay_EmergencyRelease()` 후 Enable

5. 테스트 토글
- `AUTO_FIXED_PULSE_TEST` 매크로 존재(기본 0=운영 경로)
- 1이면 AUTO에서 고정 펄스 출력, 0이면 PID 경로 실행

---

## 전체 모듈 관계도

```
외부 입력                    STM32 내부                     외부 출력
─────────────────────────────────────────────────────────────────────
                         ┌─────────────────┐
이더넷 (인지파트)  ──→   │ ethernet_comm.c  │
                         └────────┬────────┘
                                  │ steering_angle
                                  ↓
                         ┌─────────────────┐
                    ┌──← │ position_ctrl.c  │ ←── encoder_reader.c ←── 엔코더 (TIM4)
                    │    │  (PID 계산)      │
                    │    └────────┬────────┘
                    │             │ output (Hz)
                    │             ↓
                    │    ┌─────────────────┐
                    │    │ pulse_control.c  │ ──→ PE9 (PWM) ──→ RS422 ──→ 서보드라이브
                    │    │  (PWM 주파수)   │ ──→ PE10 (DIR)──→ 방향신호 ──→ 서보드라이브
                    │    └─────────────────┘
                    │
                    └──→ PositionControl_Disable() ──→ relay_control.c
                         (비상정지 시)                    └──→ PD14 (SVON)
                                                          └──→ PD15 (EMG)

타이머 인터럽트 (1ms)
  stm32f4xx_it.c
  SysTick_Handler()
  → interrupt_flag = 1
  → main.c에서 감지
  → PositionControl_Update() 호출
```

---

## 실행 흐름

### 전원 ON부터 모터가 조향각으로 움직이기까지

```
1. HAL_Init()
   └── 클럭, 플래시 레이턴시 초기화
   └── SysTick 1ms 타이머 시작

2. SystemClock_Config()
   └── 외부 클럭(HSE) 8MHz → PLL → 180MHz 시스템 클럭 설정

3. 주변장치 초기화
   ├── MX_GPIO_Init()        GPIO 핀 방향/속도 설정
   ├── MX_USART3_UART_Init() UART (printf 출력용)
   ├── MX_ADC1_Init()        ADC (포텐셔미터, 미사용)
   ├── MX_TIM1_Init()        TIM1: PWM 타이머 (펄스 출력)
   ├── MX_IWDG_Init()        워치독 타이머 (프리즈 방지)
   ├── MX_TIM4_Init()        TIM4: 엔코더 타이머 (위치 피드백)
   └── MX_LWIP_Init()        이더넷/LwIP 초기화

4. USER CODE BEGIN 2
   ├── PE10(DIR) → GPIO_Output 재설정 (방향 신호용)
   ├── HAL_TIM_Encoder_Start(&htim4) TIM4 엔코더 모드 시작
   ├── Relay_Init()           SVON=HIGH(OFF), EMG=HIGH(정상)
   ├── PulseControl_Init()    DIR 핀 LOW(CCW 초기화)
   ├── EncoderReader_Init()   엔코더 카운터 초기화
   ├── PositionControl_Init() PID 상태 초기화
   ├── Relay_ServoOn()        SVON=LOW(ON) → 서보 활성화
   ├── HAL_Delay(500)         서보 안정화 대기
   ├── EncoderReader_Reset()  현재 위치를 0도 기준으로 설정
   ├── PositionControl_SetTarget(0.0f) 초기 목표 0도 (UDP로 갱신됨)
   ├── PositionControl_Enable() PID 제어 시작
   └── EthComm_UDP_Init()     UDP 수신 소켓 열기

5. while(1) 메인 루프
   ├── MX_LWIP_Process()      이더넷 패킷 처리
   ├── if (새 UDP 데이터?)    조향각 목표값 갱신
   ├── if (interrupt_flag?)   1ms 제어 루프 실행
   │   └── PositionControl_Update()
   │       ├── 현재 각도 읽기 (EncoderReader_GetAngleDeg)
   │       ├── 오차 계산 (target - current)
   │       ├── 안전 체크
   │       ├── PID 계산 → 펄스 주파수
   │       └── PulseControl_SetFrequency(output)
   └── HAL_IWDG_Refresh()     워치독 리셋 (없으면 MCU 재시작)
```

---

## 모듈 상세 설명

---

### main.c - 진입점 및 메인 루프

**파일 위치:** `Core/Src/main.c`
**역할:** 전체 시스템의 시작점. 모든 모듈을 초기화하고, 메인 루프를 실행.

#### 핵심 코드 설명

```c
// ① SysTick 기반 1ms 제어 루프
if (interrupt_flag) {
    interrupt_flag = 0;           // 플래그 클리어 (다음 인터럽트를 위해)
    PositionControl_Update();     // PID 제어 실행
}
```

```c
// ② LwIP 폴링 (RTOS 없이 이더넷을 처리하는 방법)
MX_LWIP_Process();
// 이 함수 안에서: 이더넷 버퍼 확인 → 패킷 있으면 LwIP 처리 → UDP면 콜백 호출
```

```c
// ③ UDP 데이터 위치제어 연동 (모드 기반)
SteerMode_t mode = EthComm_GetCurrentMode();
if (EthComm_HasNewData()) {
    AutoDrive_Packet_t pkt = EthComm_GetLatestData();
    if (mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) {
        PositionControl_SetTarget(pkt.steering_angle);  // degree 기준 조향각 업데이트
    }
}
```

```c
// ④ 워치독 리셋 (매 루프마다 호출 필수)
HAL_IWDG_Refresh(&hiwdg);
// 설명: IWDG는 정해진 시간 안에 이 함수가 안 불리면 MCU를 강제 재시작.
//       프로그램이 무한 루프나 데드락에 빠졌을 때 자동 복구.
```

#### printf → UART 리다이렉션

```c
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
// printf() 출력이 UART3(USB 가상 시리얼)으로 나옴
// → PC에서 PuTTY/Tera Term으로 디버그 메시지 확인 가능
```

---

### stm32f4xx_it.c - 인터럽트 처리기

**파일 위치:** `Core/Src/stm32f4xx_it.c`
**역할:** 하드웨어 인터럽트 발생 시 자동으로 실행되는 함수들의 모음.

#### 인터럽트란?

CPU가 일반 코드를 실행하다가 특정 이벤트 발생 시 하던 일을 잠깐 멈추고
지정된 함수(ISR, Interrupt Service Routine)를 실행한 후 복귀하는 메커니즘.

```
메인 코드 실행 중...
  ↓
[SysTick 1ms 도달] ← 하드웨어 타이머 이벤트
  ↓ 인터럽트 발생
SysTick_Handler() 실행
  ├── HAL_IncTick()    (HAL 내부 ms 카운터 +1)
  └── interrupt_flag = 1 (main.c에 알림)
  ↓ 인터럽트 종료, 메인 코드로 복귀
  ↓
main.c if(interrupt_flag) → PositionControl_Update() 실행
```

#### 이 파일의 주요 함수

| 함수 | 트리거 | 역할 |
|------|--------|------|
| `SysTick_Handler()` | 매 1ms | `interrupt_flag=1` → 제어 루프 신호 |
| `TIM1_CC_IRQHandler()` | TIM1 캡처/비교 | 스텝 모드 펄스 카운팅 |
| `ETH_IRQHandler()` | 이더넷 패킷 수신 | HAL 이더넷 처리 → LwIP로 전달 |
| `HardFault_Handler()` | 잘못된 메모리 접근 등 | while(1) 무한 루프 (디버깅 필요) |

---

### tim.c - 타이머 설정

**파일 위치:** `Core/Src/tim.c`
**역할:** STM32의 타이머 하드웨어를 설정하는 CubeMX 자동 생성 파일.

#### TIM1 (펄스 출력용 PWM 타이머)

```
클럭 계산:
  APB2 클럭 = 90MHz (180MHz / 2)
  TIM1 클럭 = 180MHz (APB 타이머는 2배)

  Prescaler = 215 → 실제 카운터 클럭 = 180MHz / (215+1) = 833,333 Hz

  ARR (Auto Reload Register) = 주파수를 결정
  출력 주파수 = 833,333 / (ARR+1) Hz

  ARR=9 (초기값) → 833,333 / 10 = 83,333 Hz (83kHz)
  ARR=1666       → 833,333 / 1667 ≈ 500 Hz (PID 출력 500Hz 예시)
```

```
출력 핀: PE9 (TIM1_CH1)
PWM 모드: HIGH 동안 1, LOW 동안 0
듀티비:   CCR / (ARR+1) = 50% (CCR = ARR/2)
```

#### TIM4 (엔코더 카운터)

```
핀:    PD12 (TIM4_CH1, 엔코더 A상)
       PD13 (TIM4_CH2, 엔코더 B상)
모드:  Encoder Mode TI12 (4체배)
범위:  16비트 카운터 (0 ~ 65535)
       중간값 32768에서 시작 → 양방향 카운트 가능
```

```
4체배(Quadrature) 설명:
  엔코더 A, B상의 상승/하강 에지를 모두 카운트
  → 1회전당 12,000 PPR × 4 = 48,000 카운트
  → 분해능: 360° / 48,000 = 0.0075°/카운트

  CW 회전: 카운터 증가 (+)
  CCW 회전: 카운터 감소 (-)
```

---

### encoder_reader.c - 엔코더 읽기

**파일 위치:** `Core/Src/encoder_reader.c`
**역할:** TIM4 엔코더 카운터 값을 읽어 실제 각도(도, °)로 변환.

#### 엔코더란?

모터에 부착된 광학 또는 자기 센서로, 모터 회전량을 디지털 펄스로 출력합니다.

```
서보모터 샤프트 회전
  ↓
엔코더 디스크 회전 (슬릿 원판)
  ↓
광학 센서가 슬릿 통과 감지
  ↓
A상, B상 구형파 출력
  ↓
TIM4가 하드웨어로 카운트
```

#### 각도 계산 흐름

```c
float EncoderReader_GetAngleDeg(void) {
    // 1. TIM4 하드웨어 카운터 읽기 (0~65535)
    uint16_t raw = __HAL_TIM_GET_COUNTER(&htim4);

    // 2. 중간값(32768) 빼기 → -32768 ~ +32767 범위로 변환
    encoder_count = (int32_t)raw - 32768;
    //   raw=32768 → count=0    (기준점)
    //   raw=32769 → count=+1   (CW 1카운트)
    //   raw=32767 → count=-1   (CCW 1카운트)

    // 3. 오프셋 빼기 (EncoderReader_Reset() 호출 시 설정)
    int32_t adjusted_count = encoder_count - encoder_offset;

    // 4. 각도 변환
    // DEG_PER_COUNT = 360° / 48000 = 0.0075°/카운트
    return (float)adjusted_count * DEG_PER_COUNT;
}
```

#### 왜 중간값(32768)에서 시작하는가?

```
16비트 카운터는 0~65535 범위
0에서 시작하면:
  CW 회전: 1, 2, 3, ...   → 정상
  CCW 회전: 65535, 65534, ... → 음수 방향으로 가면 오버플로우!

32768에서 시작하면:
  CW 회전: 32769, 32770, ...   → 양수 방향 (최대 +32767까지 여유)
  CCW 회전: 32767, 32766, ...  → 음수 방향 (최소 -32768까지 여유)
```

#### EncoderReader_Reset() 동작

```c
void EncoderReader_Reset(void) {
    __HAL_TIM_SET_COUNTER(&htim4, 32768);  // 카운터를 32768로 리셋
    encoder_count = 0;
    encoder_offset = 0;
    // → 이 시점의 모터 위치를 0도 기준으로 설정
}
```

main.c에서 `EncoderReader_Reset()` 후 `PositionControl_SetTarget(0.0f)` 호출 →
**현재 위치가 0도 기준점**이 됩니다.

---

### pulse_control.c - 펄스 출력 제어

**파일 위치:** `Core/Src/pulse_control.c`
**역할:** STM32에서 서보드라이브로 보내는 펄스 신호(PWM) 제어. 모터의 속도와 방향을 결정.

#### 서보드라이브 제어 방식 (Pulse+Direction)

```
STM32 PE9 (PULS) ──→ RS422 모듈 ──→ 드라이브 PF+/PF-
                       각 펄스의 상승 에지 = 1스텝 이동
                       주파수 = 속도 (Hz가 높을수록 빠름)

STM32 PE10 (SIGN) ──→ 드라이브 PR+
                       HIGH (3.3V) = CW (시계 방향)
                       LOW  (0V)   = CCW (반시계 방향)
```

#### PulseControl_SetFrequency() - PID 제어용 핵심 함수

```c
void PulseControl_SetFrequency(int32_t freq_hz) {
    // ① 정지 명령 처리
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop(p_htim1, TIM_CHANNEL_1);
        return;
    }

    // ② 방향 결정
    if (freq_hz > 0) {
        HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_SET);   // CW
    } else {
        HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_RESET); // CCW
        freq_hz = -freq_hz;  // 이후 계산을 위해 양수로 변환
    }

    // ③ 속도 제한
    if (freq_hz > 100000) freq_hz = 100000;  // 최대 100kHz
    if (freq_hz < 10)     freq_hz = 10;      // 최소 10Hz

    // ④ ARR 계산 (TIM1 클럭으로 목표 주파수 생성)
    // TIM1 클럭 = 180MHz, PSC = 215
    // 카운터 클럭 = 180MHz / (215+1) = 833,333 Hz
    // ARR = 833,333 / freq_hz - 1
    uint32_t psc = p_htim1->Instance->PSC;
    uint32_t arr = (180000000 / ((psc + 1) * freq_hz)) - 1;

    // ⑤ 타이머 레지스터 업데이트
    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);           // 주기 설정
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);  // 50% 듀티

    // ⑥ PWM 시작
    HAL_TIM_PWM_Start(p_htim1, TIM_CHANNEL_1);
}
```

#### 두 가지 동작 모드

| 모드 | 함수 | 사용 상황 |
|------|------|---------|
| **Speed Mode** | `PulseControl_SetFrequency()` | PID 제어 (연속 펄스) |
| **Step Mode** | `PulseControl_SendSteps()` | 정해진 스텝만 이동 후 정지 |

```
Speed Mode: 주파수만 바꾸면서 계속 펄스 출력 (PID가 매 1ms 호출)
Step Mode:  N개 펄스 출력 후 자동 정지 (TIM1 인터럽트로 카운트)
```

---

### position_control.c - PID 위치 제어

**파일 위치:** `Core/Src/position_control.c`
**역할:** 엔코더로 현재 위치를 읽고, PID 알고리즘으로 목표 위치에 도달하도록 펄스 주파수를 계산.

#### PID 제어란?

```
목표각도 - 현재각도 = 오차(error)
오차를 줄이도록 출력(펄스 주파수)을 계산

P (비례): 오차에 비례한 출력 → 오차가 크면 빠르게, 작으면 천천히
I (적분): 오차가 계속 쌓이면 출력 증가 → 정상 상태 오차 제거
D (미분): 오차 변화율로 예측 → 오버슈트 억제

최종 출력 = Kp×e + Ki×∫e dt + Kd×(de/dt)
```

```
실제 예시 (target=10°, current=0°):
  오차 e = 10°
  P항 = 50 × 10 = 500 Hz
  I항 = 5 × (10 × 0.001) = 0.05 Hz (초기)
  D항 = 20 × (10-10)/0.001 = 0 Hz (Enable 시 현재 오차로 초기화했으므로)
  출력 = 500 Hz → 서보모터가 500Hz 펄스로 CW 회전

  시간이 지나 current=9°가 되면:
  오차 e = 1°
  P항 = 50 × 1 = 50 Hz → 속도 줄어듦
  ...
  current=10°가 되면:
  오차 e = 0°
  P항 = 0 → 정지
```

#### PositionControl_Update() - 1ms마다 실행

```c
void PositionControl_Update(void) {
    // 1. 활성화 확인
    if (!control_enabled) { PulseControl_Stop(); return; }

    // 2. 현재 각도 읽기
    state.current_angle = EncoderReader_GetAngleDeg();

    // 3. 오차 계산 (안전 체크 전에 먼저!)
    state.error = state.target_angle - state.current_angle;

    // 4. 안전 체크
    if (!PositionControl_CheckSafety()) {
        PositionControl_EmergencyStop();
        return;
    }

    // 5. 경과 시간 계산
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - pid_state.last_time_ms) / 1000.0f;
    dt = (dt <= 0) ? 0.001f : (dt > 0.1f) ? 0.1f : dt;  // 0.001~0.1s 범위 제한
    pid_state.last_time_ms = current_time;

    // 6. PID 계산
    state.output = PID_Calculate(state.error, dt);

    // 7. 펄스 출력 (양수=CW, 음수=CCW, 크기=속도)
    PulseControl_SetFrequency((int32_t)state.output);

    // 8. 안정화 판단 (오차 < 0.5° 상태가 100ms 이상 유지)
    if (fabsf(state.error) < POSITION_TOLERANCE) {
        state.stable_time_ms += (uint32_t)(dt * 1000.0f);
        if (state.stable_time_ms > 100) state.is_stable = true;
    } else {
        state.stable_time_ms = 0;
        state.is_stable = false;
    }
}
```

#### 적분 와인드업 방지

```
문제: 모터가 물리적 한계에 막혀 오차가 줄지 않으면
      integral이 계속 쌓여 매우 커짐 (와인드업)
      → 한계 해제 후 출력이 폭주

해결: integral 값에 상한선(1000) 설정
if (pid_state.integral > integral_limit) pid_state.integral = integral_limit;
```

#### 안전 기능

```c
bool PositionControl_CheckSafety(void) {
    // 물리적 한계 초과 (±365°)
    if (state.current_angle > MAX_ANGLE_DEG + 5.0f ||
        state.current_angle < MIN_ANGLE_DEG - 5.0f) {
        fault_flag = 1;
        return false;
    }
    // 오차가 너무 큼 (150° 초과 = 제어 불능 상태)
    if (fabsf(state.error) > 150.0f) {
        fault_flag = 2;
        return false;
    }
    return true;
}
```

#### PID 파라미터 현재 설정값

| 파라미터 | 값 | 의미 |
|---------|-----|------|
| Kp | 50.0 | 비례 게인 (큰 값 = 빠른 반응, 진동 위험) |
| Ki | 5.0 | 적분 게인 (큰 값 = 정상 오차 빠른 제거) |
| Kd | 20.0 | 미분 게인 (큰 값 = 오버슈트 억제) |
| integral_limit | 1000.0 | 적분 최대값 (와인드업 방지) |
| output_limit | 10000.0 Hz | 최대 펄스 주파수 (최대 속도) |

---

### relay_control.c - 릴레이 제어

**파일 위치:** `Core/Src/relay_control.c`
**역할:** 서보 ON/OFF(SVON)와 비상정지(EMG) GPIO 핀 제어.

#### Active LOW 논리

```
Active LOW = 핀이 LOW(0V)일 때 기능이 활성화

SVON 핀 (PD14):
  HIGH (3.3V) = 서보 OFF
  LOW  (0V)   = 서보 ON ← Relay_ServoOn() 호출 시

EMG 핀 (PD15):
  HIGH (3.3V) = 정상 동작
  LOW  (0V)   = 비상정지 ← Relay_Emergency() 호출 시
```

```c
void Relay_ServoOn(void) {
    HAL_GPIO_WritePin(SVON_PORT, SVON_PIN_CTRL, GPIO_PIN_RESET); // LOW = ON
}
void Relay_Emergency(void) {
    HAL_GPIO_WritePin(EMG_PORT, EMG_PIN_CTRL, GPIO_PIN_RESET);   // LOW = 정지
}
```

> **현재 상태:** EMG 릴레이는 24V 미연결, P2-02 비활성화 상태이므로
> `Relay_Emergency()`는 GPIO 상태만 바꾸고 실제 드라이브에 영향 없음.

---

### ethernet_communication.c - 이더넷 통신

**파일 위치:** `Core/Src/ethernet_communication.c`
**역할:** UDP 패킷 수신, 필터링, 위치 제어 연동.

자세한 설명은 [ethernet_communication.md](ethernet_communication.md) 참조.

#### 주요 함수 요약

| 함수 | 역할 |
|------|------|
| `EthComm_UDP_Init()` | UDP 소켓 생성 및 포트 바인드, 콜백 등록 |
| `udp_recv_cb()` (내부) | 패킷 수신 시 LwIP가 자동 호출, ASMS(5B)/PC(9B) 파싱 |
| `EthComm_HasNewData()` | 새 패킷 수신 여부 확인 |
| `EthComm_GetLatestData()` | 최신 패킷 반환 및 플래그 초기화 |
| `EthComm_GetCurrentMode()` | 현재 모드(NONE/AUTO/MANUAL/ESTOP) 반환 |

---

### lwip.c - LwIP 네트워크 스택

**파일 위치:** `LWIP/App/lwip.c`
**역할:** STM32의 IP 주소 설정, LwIP 초기화, 폴링 함수 제공.

#### IP 주소 설정

```c
IP_ADDRESS = { 10, 177, 21, 100 };  // 이 STM32의 IP
NETMASK    = { 255, 255, 255, 0 };  // 서브넷 마스크
GATEWAY    = { 10, 177, 21, 1 };    // 게이트웨이
```

#### MX_LWIP_Process() - 폴링 핵심 함수

```c
void MX_LWIP_Process(void) {
    ethernetif_input(&gnetif);  // 이더넷 버퍼에서 패킷 꺼내 LwIP에 전달
    sys_check_timeouts();       // LwIP 내부 타이머 처리
    Ethernet_Link_Periodic_Handle(&gnetif); // 링크 상태 100ms마다 확인
}
```

이 함수가 main의 while(1)에서 호출될 때마다:
1. 이더넷 NIC에 새 패킷이 있으면 가져와서 처리
2. UDP 패킷이면 목적지 포트 확인 → 5000이면 `udp_recv_cb()` 호출

---

## 파일별 의존성 정리

```
main.c
  ├── position_control.c  ← encoder_reader.c (각도 읽기)
  │                       ← pulse_control.c  (펄스 출력)
  ├── relay_control.c     (서보 ON/OFF)
  ├── ethernet_communication.c ← lwip/udp.h (LwIP UDP API)
  └── lwip.c              (이더넷 초기화 및 폴링)

stm32f4xx_it.c            (인터럽트: SysTick, TIM1, ETH)

tim.c                     (TIM1, TIM4 하드웨어 설정)
```

---

## 공통 개념 정리

### HAL (Hardware Abstraction Layer)

STM32의 하드웨어를 추상화한 라이브러리. 칩마다 다른 레지스터 주소를 몰라도
`HAL_GPIO_WritePin()`, `HAL_TIM_PWM_Start()` 등 통일된 API로 제어 가능.

### volatile 키워드

```c
static volatile bool control_enabled = false;
static volatile uint8_t interrupt_flag = 0;
```

`volatile`은 컴파일러에게 "이 변수는 언제든지 외부(인터럽트)에서 바뀔 수 있으니
캐시하지 말고 매번 메모리에서 읽어라"고 지시하는 키워드.

없으면: 컴파일러 최적화로 변수를 레지스터에 캐시 → 인터럽트에서 바꿔도 인식 못 함.

### __disable_irq() / __enable_irq()

```c
__disable_irq();
state.target_angle = target_deg;  // 이 작업 중에 인터럽트 차단
__enable_irq();
```

여러 바이트로 이루어진 float 변수를 쓰는 도중 인터럽트가 발생하면
절반만 쓰인 오염된 값을 읽을 수 있음 → IRQ를 잠깐 비활성화해 원자적 연산 보장.
