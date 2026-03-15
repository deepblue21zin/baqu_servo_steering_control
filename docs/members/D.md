# 팀원 D 문서

분석 기준일: 2026-03-14

담당 범위: `pulse_control`

분석 기준:
- 실제 구현은 `Core/Src/pulse_control.c`, `Core/Src/tim.c`, `Core/Src/main.c`, `Core/Src/position_control.c`를 기준으로 봤다.
- D 파트는 제어기 출력의 마지막 단계이며, 하드웨어 파형 품질과 safety stop 품질을 책임지는 모듈이다.

## 1. 전체 아키텍처에서 D 파트의 위치

현재 출력 경로는 아래와 같다.

1. `position_control.c`가 `state.output`을 계산한다.
2. `PulseControl_SetFrequency((int32_t)state.output)`를 호출한다.
3. `pulse_control.c`는 pulse 출력용 로직과 direction 출력용 로직을 분리해서 만든다.
4. `PE9(TIM1_CH1)`는 pulse line driver 입력으로 들어가고, 하드웨어에서 `PF+/PF-` 차동쌍으로 변환된다.
5. `PE10(GPIO)`는 direction line driver 입력으로 들어가고, 하드웨어에서 `PR+/PR-` 차동쌍으로 변환된다.
6. fault 또는 disable이면 `PulseControl_Stop()`으로 즉시 정지한다.

또한 step mode도 별도로 있다.

- `PulseControl_SendSteps(steps, dir)`
- TIM1 CC interrupt 기반으로 `remaining_steps`를 줄인다.

즉 D 파트는 continuous speed mode와 step mode를 둘 다 가진 출력 계층이며, pulse용 line driver와 direction용 line driver를 각각 구분해 설명해야 하는 파트다.

## 2. 지금 서로 주고받는 정보와 형식

| Producer | Consumer | 항목 | 형식 | 현재 의미 |
|---|---|---|---|---|
| `position_control.c` | `pulse_control.c` | `freq_hz` | `int32_t` | 부호 포함 연속 출력 명령 |
| test/main | `pulse_control.c` | `steps` | `uint32_t` | 정해진 펄스 수 |
| test/main | `pulse_control.c` | `dir` | enum | `DIR_CW`, `DIR_CCW` |
| `pulse_control.c` | pulse line driver | pulse input | PWM + register value | `PE9(TIM1_CH1)` 기반 PF 차동 출력 원천 |
| `pulse_control.c` | direction line driver | direction input | pin level | `PE10(GPIO)` 기반 PR 차동 출력 원천 |
| `pulse_control.c` | caller | busy state | `uint8_t` | step mode 진행 여부 |

핵심 포인트:
- D 파트의 외부 인터페이스는 단순하다.
- 실제 하드웨어는 pulse용 line driver와 direction용 line driver가 분리되어 있으므로, 문서도 `PF differential pair`와 `PR differential pair`를 분리해 서술하는 것이 맞다.
- 하지만 실제 파형, stop latency, direction 전환 정책이 계약화되어 있지 않으면 현업 품질로 보기 어렵다.

## 3. 현재 코드 구현 상세 분석

### 3.1 초기화

`PulseControl_Init()`은 아래만 수행한다.

- `p_htim1 = &htim1`
- `is_busy = 0`
- 방향 핀을 LOW로 초기화

장점:
- 초기화가 단순하고 안전하다.
- PWM을 init 단계에서 바로 켜지 않도록 구조가 바뀌었다.
- pulse 생성 경로(`PE9`)와 direction 생성 경로(`PE10`)가 논리적으로는 이미 분리되어 있다.

한계:
- 현재 mode를 기억하지 않는다.
- timer clock, current frequency, current direction을 상태로 들고 있지 않다.
- 하드웨어가 pulse용 line driver와 direction용 line driver로 분리되어 있어도, 펌웨어에서는 enable/status를 각각 따로 관리하지 않는다.

### 3.2 continuous mode

`PulseControl_SetFrequency()`는 현재 핵심 함수다.

순서:

1. `freq_hz == 0`이면 PWM stop
2. 부호에 따라 direction 입력 핀 `PE10` 설정
3. 절대값으로 변환
4. `10 ~ 100000 Hz`로 clamp
5. `timer_clk = 180000000` 하드코딩
6. `arr = timer_clk / ((psc + 1) * freq) - 1`
7. `ARR`, `CCR` 갱신
8. `PE9(TIM1_CH1)` PWM start

장점:
- 방향 핀 버그를 이미 수정한 상태다.
- ARR 범위 보호가 있다.
- 0 명령에서 stop 처리도 명확하다.

리스크:
- timer clock이 하드코딩이라 clock 변경 시 바로 틀어진다.
- 호출할 때마다 `HAL_TIM_PWM_Start()`를 반복 호출한다.
- direction change 시 guard time 정책이 없다.
- 실제 measured frequency와 commanded frequency를 비교하는 검증 API가 없다.
- pulse용 line driver와 direction용 line driver가 분리된 하드웨어인데도, 현재 문서와 코드에는 이를 명시적으로 구분한 진단 개념이 부족하다.

### 3.3 step mode

`PulseControl_SendSteps()`는 `steps`와 `dir`만 받는다.

실제 동작:

- `remaining_steps = steps`
- 방향 핀 설정
- `HAL_TIM_PWM_Start_IT()`

그리고 `HAL_TIM_PWM_PulseFinishedCallback()`에서 `remaining_steps--` 후 0이면 stop한다.

여기서 중요한 문제:

- step mode에는 frequency 인자가 없다.
- 즉 현재 타이머에 남아 있는 ARR/CCR 설정을 그대로 쓰게 된다.
- 초기값 기준이면 약 83kHz이고, 이전 PID 동작의 마지막 주파수를 그대로 물려받을 수도 있다.

즉 step mode는 기능은 있지만 "몇 Hz로 몇 펄스를 보낼지"가 명세화되지 않았다.

### 3.4 main/tim 연동

`tim.c` 기준 TIM1 설정은 아래와 같다.

- Prescaler = 215
- Period = 9
- PWM channel = CH1
- PE9 출력

실제 counter clock은 약 `180MHz / 216 = 833333Hz`다.

따라서:
- 100kHz 출력은 가능
- 더 높은 설정은 ARR 한계에 따라 제한된다

추가로 현재 `TIM1_CH2`도 설정돼 있지만 D 파트 구현에서는 실질적으로 사용하지 않는다.

### 3.5 stop 품질

`PulseControl_Stop()`은 아래를 수행한다.

- `__HAL_TIM_DISABLE_IT(..., TIM_IT_CC1)`
- `HAL_TIM_PWM_Stop()`
- `remaining_steps = 0`
- `is_busy = 0`

좋은 점:
- step mode와 continuous mode 모두 한 함수로 멈추도록 정리돼 있다.

부족한 점:
- stop latency를 측정하거나 보장하는 근거가 없다.
- estop 시 잔류 pulse가 몇 개 남는지 확인한 자료가 없다.

## 4. 현재 평가

성숙도 기준:
- L1: 단순 출력 골격
- L2: bench 파형 출력 가능
- L3: 제어 경로에 통합
- L4: 실제 파형 품질과 stop 근거가 있음
- L5: 출력 계약과 계측 증거가 완비

현재 성숙도: `L3`

평가 근거:
- PID output이 실제 pulse/direction으로 연결돼 있으므로 통합 경로는 살아 있다.
- 하지만 mode arbitration, step frequency contract, waveform evidence가 부족하다.

자율주행 조향모터의 현업 기준으로 보면 아래 갭이 추가로 보인다.

- fail-safe 완성도:
  - not-ready 상태에서 출력 inhibit 규칙이 모듈 계약으로 없다.
  - direction reverse 시 stop-before-reverse, setup/hold time이 없다.
  - reset/default 시 pulse off는 사실상 지켜지지만, 이를 요구사항과 진단 항목으로 고정하지 않았다.
- debug 완성도:
  - `requested_hz`, `applied_hz`, `ARR`, `CCR`, `current_direction`, `last_stop_reason`를 구조체로 조회하지 못한다.
  - line driver enable 상태와 잔류 pulse 계측 근거가 없다.

한 줄 평가:
- "기본 출력은 잘 되지만, 출력 계약과 검증 자료까지 포함한 actuator module로 보강할 필요가 있는 상태"

## 5. D 파트 REQ 초안

- `REQ-D-001`: 연속 출력 모드에서 양수/음수 명령에 대해 방향 핀이 일관되게 동작해야 한다.
- `REQ-D-002`: `freq_hz == 0` 명령 시 PWM 출력은 즉시 정지해야 한다.
- `REQ-D-003`: ARR/CCR 계산은 현재 timer clock과 prescaler 기준으로 설명 가능해야 한다.
- `REQ-D-004`: timer clock은 하드코딩하지 말고 실제 설정값 또는 계산값을 사용해야 한다.
- `REQ-D-005`: step mode는 `steps`, `dir`뿐 아니라 출력 주파수 계약도 가져야 한다.
- `REQ-D-006`: step mode와 continuous mode의 동시 사용 충돌을 방지해야 한다.
- `REQ-D-007`: direction reversal 시 필요한 guard time이나 stop-before-reverse 정책을 정의해야 한다.
- `REQ-D-008`: estop 또는 disable 시 잔류 pulse가 허용 범위를 넘지 않아야 한다.
- `REQ-D-009`: 출력 주파수 정확도와 stop latency는 계측 근거로 검증돼야 한다.
- `REQ-D-010`: busy state와 현재 적용 주파수는 진단용으로 조회 가능해야 한다.
- `REQ-D-011`: power-up, reset, fault 상태에서는 pulse 출력이 기본적으로 inhibit되어야 하며 direction 상태도 deterministic해야 한다.
- `REQ-D-012`: homing 미완료, EMG active, servo off, system not ready 상태에서는 출력 enable을 허용하지 않아야 한다.
- `REQ-D-013`: pulse width, duty, direction setup/hold time은 servo drive datasheet 요구를 만족해야 한다.
- `REQ-D-014`: `requested_hz`, `applied_hz`, `ARR`, `CCR`, `direction`, `mode`, `driver_enable`, `last_stop_reason`를 진단 구조체로 조회 가능해야 한다.
- `REQ-D-015`: 하드웨어가 pulse용/dir용 line driver enable을 분리 제공하면 펌웨어도 이를 분리 관리해야 하며, 공유 enable이면 그 사실을 명시해야 한다.
- `REQ-D-016`: direction reversal과 ESTOP 시 residual pulse 개수와 glitch 여부가 측정 가능한 기준으로 제한되어야 한다.

인수 기준:

1. 저속, 중속, 고속 구간에서 commanded vs measured frequency 오차가 허용 범위 내다.
2. 방향 전환 시 한쪽 고정 회전이나 glitch가 없다.
3. step mode가 명시된 속도와 스텝 수로 동작한다.
4. estop 후 pulse 출력이 멈추고 busy state가 정상 복귀한다.
5. not-ready 상태에서 pulse 출력이 억제되는 것이 실측으로 확인된다.
6. datasheet 기준 pulse width와 direction setup/hold time을 scope 캡처로 설명할 수 있다.

## 6. 어떻게 더 발전시켜야 하는가

### P0

- timer clock 하드코딩을 제거한다.
- step mode에 주파수 인자를 추가하거나 별도 설정 API를 만든다.
- continuous mode와 step mode를 enum 상태로 분리한다.
- `AUTO_FIXED_PULSE_HZ` 같은 테스트 상수와 실제 clamp 정책을 일치시킨다.
- 문서와 주석에서 `PE9 -> pulse line driver -> PF+/PF-`, `PE10 -> direction line driver -> PR+/PR-` 구조를 명시적으로 분리한다.
- stop-before-reverse와 direction setup/hold time을 구현한다.
- system ready/homing/EMG 상태와 연동된 output inhibit 정책을 넣는다.

### P1

- commanded vs measured frequency 검증 루틴을 만든다.
- direction reversal 정책을 추가한다.
- applied ARR/CCR/current direction/current mode를 진단 구조체로 노출한다.
- 사용하지 않는 CH2 설정을 제거하거나 용도를 명확히 한다.
- pulse/dir line driver enable 상태와 last stop reason을 진단 정보에 포함한다.
- residual pulse count와 jitter를 반복 계측하는 자동 시험을 만든다.

### P2

- HAL 호출 오버헤드를 줄이기 위한 direct register update를 검토한다.
- pulse jitter 계측과 오실로스코프 자동 캡처 프로세스를 만든다.
- driver datasheet 기준 minimum pulse width, setup/hold time을 요구사항으로 고정한다.
- 운영 빌드에서 step mode를 test-only로 제한할지, 운영 기능으로 정식 지원할지 정책을 분리한다.

## 7. 정량 증거를 어떻게 남기면 좋은가

D 파트는 파형 근거가 가장 중요하다.

| 지표 | 의미 | 권장 증거 |
|---|---|---|
| frequency error | 주파수 정확도 | 오실로스코프 캡처 |
| duty ratio error | 파형 품질 | 오실로스코프 캡처 |
| stop latency | 정지 반응 속도 | trigger 측정 |
| reversal glitch count | 방향전환 안정성 | 로직애널라이저 |
| step count accuracy | 지정 스텝 정확도 | 카운트 로그 |
| output jitter | 주기 안정성 | scope statistics |

권장 파일:
- `Doc/measurements/pulse_freq_2026-03-xx_run01.csv`
- `Doc/measurements/pulse_stop_2026-03-xx_run01.png`
- `Doc/measurements/pulse_reverse_2026-03-xx_run01.png`

## 8. "얼마나 향상됐는가"를 보여주는 기준

| 항목 | Baseline | Target | 개선 표시법 |
|---|---|---|---|
| frequency error | 예: 6.5% | 1% 이하 | `% 감소` |
| stop latency | 예: 18ms | 5ms 이하 | `ms 감소` |
| reversal glitch | 예: 7/100 | 0/100 | `count 감소` |
| step count error | 예: 12/10000 | 0/10000 | `count 감소` |
| output jitter | 예: 4.2us p99 | 1us 이하 | `us 감소` |

면접에서 좋은 표현:
- "PWM을 냈다"보다 "100Hz/1kHz/10kHz 구간에서 오실로스코프로 주파수 오차를 검증했고, stop latency를 18ms에서 4ms로 줄였다"가 훨씬 강하다.

## 9. 취업에 도움이 되는 기록 전략

D 파트는 아래 자료가 있으면 바로 포트폴리오 힘이 생긴다.

1. commanded vs measured 표
- `requested_hz`, `actual_hz`, `error_percent`

2. scope 캡처
- low speed
- high speed
- stop
- reverse

3. bug-fix 스토리
- 방향 핀 버그 수정
- stop 상태 꼬임 수정
- init 시 PWM 조기출력 방지

4. actuator contract 문서
- min/max frequency
- direction logic
- step mode 정책
- estop stop path

5. 테스트 자동화
- 주파수 sweep
- step count repeat
- emergency stop repeat

가장 추천하는 포트폴리오 문장:

> PID 출력값을 servo driver가 바로 사용할 수 있는 pulse/direction 신호로 변환하고, 주파수 계산식, 정지 경로, 방향 전환 정책을 하드웨어 파형 기준으로 검증해 actuator interface의 신뢰성을 높였다.
