# 팀원 A 문서

분석 기준일: 2026-03-14

담당 범위: `position_control`

분석 기준:
- 실제 구현은 `Core/Src/main.c`, `Core/Src/position_control.c`, `Core/Src/encoder_reader.c`, `Core/Src/pulse_control.c`, `Core/Src/ethernet_communication.c`를 우선 기준으로 봤다.
- `README.md`, `Doc/최종 구조.md` 등 일부 문서는 현재 코드와 불일치가 있으므로 참고용으로만 봐야 한다.
- 이 문서는 "지금 코드가 실제로 어떻게 연결되어 있는가"를 기준으로 작성했다.

## 1. 전체 아키텍처에서 A 파트의 위치

현재 시스템의 실동작 흐름은 아래와 같다.

1. `ethernet_communication.c`가 UDP 패킷을 받아 `SteerMode_t`와 `AutoDrive_Packet_t`로 정규화한다.
2. `main.c`가 모드 전이, RX timeout, ESTOP 요청을 처리한다.
3. `main.c`가 `PositionControl_SetTarget()` 또는 `PositionControl_Enable/Disable/EmergencyStop()`을 호출한다.
4. `SysTick_Handler()`가 1ms마다 `interrupt_flag`를 세운다.
5. `main.c`가 `interrupt_flag`를 보고 `PositionControl_Update()`를 1회 실행한다.
6. `position_control.c`가 엔코더 각도를 읽고 PID를 계산해 `PulseControl_SetFrequency()`로 출력한다.
7. 안전 조건 위반 시 `PositionControl_EmergencyStop()`가 `Relay_Emergency()`까지 같이 호출한다.

즉 A 파트는 단순 PID 모듈이 아니라, 현재 구조에서 실제 조향 동작의 중심이다.

## 2. 지금 서로 주고받는 정보와 형식

현재 흐름을 나열식으로 정리하면 아래와 같다.

- `ethernet_communication.c -> main.c`
  `SteerMode_t`를 넘긴다.
  형식은 enum이고 의미는 `NONE/AUTO/MANUAL/ESTOP`이다.

- `ethernet_communication.c -> main.c`
  `AutoDrive_Packet_t`를 넘긴다.
  형식은 struct이고 내부는 `float steering_angle`, `uint32_t speed`, `uint8_t misc`다.

- `main.c -> position_control.c`
  target 값을 넘긴다.
  형식은 `float`이고 실제 호출은 `PositionControl_SetTarget(target_deg)`다.

- `encoder_reader.c -> position_control.c`
  current angle을 넘긴다.
  형식은 `float`이고 `EncoderReader_GetAngleDeg()` 반환값이다.

- `position_control.c -> pulse_control.c`
  actuator command를 넘긴다.
  형식은 `int32_t`이고 부호 포함 `pulse_hz` 의미로 사용된다.

- `position_control.c -> relay_control.c`
  emergency 동작을 요청한다.
  형식은 함수 호출이고 실제로는 `Relay_Emergency()`를 호출한다.

- `position_control.c -> debug/DIAG`
  controller state를 제공한다.
  형식은 `PositionControl_State_t`이고 목표각, 현재각, 오차, 출력, 안정화 여부를 담는다.

현재 가장 중요한 사실은 "자료형은 단순하지만, 기준 축과 단위 역할이 아직 명확히 분리되지 않았다"는 점이다.

- `AutoDrive_Packet_t.steering_angle`는 상위 제어기에서 내려오는 값이므로 문서상 `steering_deg`로 해석하는 것이 자연스럽다.
- 반면 `EncoderReader_GetAngleDeg()`는 모터 엔코더 카운트를 환산한 값이므로 내부 피드백은 `motor_deg` 성격이 더 강하다.
- 그런데 `PositionControl_SetTarget()`과 safety limit는 현재 `MAX_ANGLE_DEG = +/-4320` 기준으로 동작하고 있어, 제어기 내부는 이미 `motor_deg` 범위를 일부 전제로 두고 있다.

즉 현재 구조는 "외부 명령은 `steering_deg`, 내부 피드백과 safety는 `motor_deg`"가 섞여 있는 상태에 가깝다. 따라서 문서와 코드에서는 `steering_deg -> motor_deg` 변환 위치를 명시하고, 변수명과 API 이름에서도 어느 축 기준 값인지 함께 드러내는 방식으로 정리하는 것이 좋다.

## 3. 현재 코드 구현 상세 분석

### 3.1 초기화와 활성화

`PositionControl_Init()`은 PID 내부 상태를 초기화하고 모드를 `CTRL_MODE_IDLE`로 둔다.

좋은 점:
- `pid_state.last_time_ms`를 초기화한다.
- `control_enabled = false`로 안전하게 시작한다.
- `state.mode`와 `control_mode`를 같이 맞춘다.

주의할 점:
- 소스의 PID 초기값은 `Kp=50, Ki=5, Kd=20`이다.
- 그런데 헤더의 `DEFAULT_KP`는 `500`이다.
- 문서와 소스, 헤더가 서로 다르므로 튜닝 기준점이 흔들린다.

### 3.2 1ms 제어 루프

`PositionControl_Update()`의 실제 순서는 아래와 같다.

1. `control_enabled == false`이면 즉시 `PulseControl_Stop()`
2. `EncoderReader_GetAngleDeg()`로 현재각 읽기
3. `state.error = target - current`
4. `PositionControl_CheckSafety()`
5. `HAL_GetTick()` 기반 `dt` 계산
6. `PID_Calculate()`
7. `PulseControl_SetFrequency((int32_t)state.output)`
8. `abs(error) < 0.5 deg`가 100ms 넘으면 stable 판정

구현상 장점:
- 안전 체크가 현재 오차 기준으로 계산되도록 수정돼 있다.
- Enable 시 `prev_error`를 현재 오차로 맞춰서 D-kick을 줄였다.
- 출력, 적분항 모두 saturation이 있다.

구현상 한계:
- `dt`가 하드웨어 타이머가 아니라 `HAL_GetTick()` 기반이라 1ms 정밀도에 한계가 있다.
- `PositionControl_Update()`는 float 연산과 `HAL_GetTick()` 호출에 의존한다.
- homing 완료 여부를 보지 않고도 Enable이 가능하다.
- comms fault, sensor fault, manual estop을 구분하는 정식 `fault code` 체계가 없다.

### 3.3 안전 처리

현재 A 파트가 직접 관리하는 안전 로직은 두 가지다.

1. 각도 제한
- `state.current_angle > MAX_ANGLE_DEG + 5`
- `state.current_angle < MIN_ANGLE_DEG - 5`

2. 추종 오차 제한
- `fabsf(state.error) > MAX_TRACKING_ERROR_DEG`

위반 시:
- `fault_flag`를 1 또는 2로 세팅
- `PositionControl_EmergencyStop()`
- `PulseControl_Stop()`
- `Relay_Emergency()`

좋은 점:
- 소프트 정지와 하드 EMG를 같이 호출한다.

부족한 점:
- `PositionControl_State_t.last_error`는 실제로 갱신되지 않는다.
- `fault_flag`는 정수 1, 2만 쓰고 있고 외부 문서화가 부족하다.
- ESTOP 원인이 comm timeout인지, tracking error인지, 각도 한계인지 구조체로 남지 않는다.

### 3.4 미구현 API

아래 항목은 인터페이스는 있으나 사실상 빈 껍데기다.

- `PositionControl_SetSafetyLimits()`
- `PositionControl_ResetStats()`
- `PositionControl_RegisterErrorCallback()`
- `PositionControl_RegisterStableCallback()`
- `PositionControl_SetDebugLevel()`

즉 A 파트는 "모터를 돌리는 핵심"은 있지만, "요구사항 기반 제어기"로 보기 위한 관리 기능은 아직 부족하다.

## 4. 현재 평가

성숙도 기준:
- L1: 함수 골격만 있음
- L2: 단일 모듈 bench 동작
- L3: 실제 통합 경로에서 동작
- L4: fail-safe와 시험 근거가 갖춰짐
- L5: 면접/포트폴리오에서 재현 가능한 수준

현재 성숙도: `L3`

평가 근거:
- `L3`인 이유: UDP -> target -> PID -> pulse 출력의 실제 통합 경로가 살아 있다.
- `L4`가 아닌 이유: 단위 정의, fault taxonomy, homing interlock, stats, 검증 증거가 부족하다.

현재 점수 감각:
- 기능 동작성: 높음
- 설계 정합성: 중간
- 안전 완성도: 중간 이하
- 검증 가능성: 낮음
- 포트폴리오 어필도: 잠재력 높음, 현재 근거는 부족

자율주행 조향모터의 현업 기준으로 추가 판단하면 아래 갭이 더 크다.

- fail-safe 완성도:
  - fault latch/clear 정책이 없다.
  - stale target, NaN 입력, 센서 이상, loop overrun을 별도 fault로 다루지 않는다.
  - ESTOP 후 자동 재활성화 차단 규칙이 모듈 내부에 없다.
- debug 완성도:
  - `last_error`, `fault timestamp`, `dt`, `saturation`, `deadline miss`가 상태 구조에 없다.
  - 사후 분석용 event log나 circular trace가 없다.

한 줄 평가:
- "핵심 루프는 잘 살아 있지만, 현업형 제어기라고 부르기 위해 필요한 단위, fault, 증거 체계가 아직 부족한 상태"

## 5. A 파트 REQ 초안

- `REQ-A-001`: 제어기 입력 단위는 `steering_deg`와 `motor_deg` 중 무엇인지 명시하고, 코드와 문서에서 하나의 변환 체계로 연결되어야 한다.
- `REQ-A-002`: `PositionControl_SetTarget()`은 외부 명령 단위와 내부 계산 단위를 분리해 처리해야 한다.
- `REQ-A-003`: 제어 루프는 1ms 실행을 전제로 하고, `dt` 산출 근거를 코드와 문서에서 설명 가능해야 한다.
- `REQ-A-004`: PID는 `Kp`, `Ki`, `Kd`, anti-windup, output saturation을 포함해야 한다.
- `REQ-A-005`: `Enable`, `Disable`, `EmergencyStop` 전환 시 출력, 모드, fault 상태가 일관되게 갱신되어야 한다.
- `REQ-A-006`: angle limit, tracking error limit, comm timeout, sensor fault, watchdog/loop overrun, operator estop을 구분 가능한 fault code로 관리해야 한다.
- `REQ-A-007`: homing 미완료 상태에서는 `RUN` 또는 `POSITION` enable을 허용하지 않아야 한다.
- `REQ-A-008`: `PositionControl_State_t`는 target, current, error, output, stable, mode, last_error를 실제 값으로 모두 제공해야 한다.
- `REQ-A-009`: PID gain과 safety threshold는 단일 설정 소스에서 관리되어야 한다.
- `REQ-A-010`: 제어기 성능 지표는 step response 로그로 남겨야 하며, 요구사항 ID와 시험 결과가 연결돼야 한다.
- `REQ-A-011`: 목표값 입력은 `NaN`, `Inf`, out-of-range, stale sequence를 거부해야 한다.
- `REQ-A-012`: fault는 latched state로 유지되어야 하며, 안전 조건 확인과 명시적 clear 없이 자동 해제되면 안 된다.
- `REQ-A-013`: 제어기는 target slew limit 또는 rate limit를 가져야 하며, 급격한 steering command 변화가 그대로 출력 급변으로 전달되지 않아야 한다.
- `REQ-A-014`: 출력 saturation 여부, actual `dt`, deadline miss, 마지막 fault timestamp는 진단 정보로 조회 가능해야 한다.
- `REQ-A-015`: 센서값이 stale하거나 물리적으로 implausible하면 bounded time 내 safe state로 전이해야 한다.
- `REQ-A-016`: ESTOP 또는 fault 이후에는 recovery 시퀀스 완료 전까지 `PositionControl_Enable()` 재진입을 허용하지 않아야 한다.

인수 기준:

1. `0 -> +10 -> 0 -> -10 -> 0 deg` 반복 시험 로그가 있다.
2. enable/disable/estop 전환 시 pulse 출력과 relay 동작이 기대와 일치한다.
3. 과대 목표값, 과대 오차, 통신 끊김 주입 시 fault code가 다르게 기록된다.
4. `PositionControl_State_t` 덤프로 현재 상태를 설명할 수 있다.
5. `NaN`, stale target, 센서 이상 주입 시 제어기가 fault를 latch하고 자동 재시작하지 않는다.
6. 1ms 제어 루프의 actual `dt`와 deadline miss 결과를 로그로 설명할 수 있다.

## 6. 어떻게 더 발전시켜야 하는가

### P0

- 외부 목표값과 내부 제어 단위를 분리한다.
- homing interlock을 넣어서 "영점 미완료 상태에서는 Enable 금지"를 강제한다.
- `fault_flag`를 enum 기반 fault code로 바꾼다.
- 헤더/소스/문서의 PID 기본값을 하나로 맞춘다.
- `NaN`, out-of-range, stale target을 차단하는 입력 검증을 넣는다.
- ESTOP/fault latch와 explicit clear 정책을 만든다.
- `dt`, saturation, deadline miss를 state/DIAG에 포함한다.

### P1

- `dt`를 더 정밀하게 관리하거나 최소한 측정 근거를 남긴다.
- `PositionControl_State_t.last_error`, stats, callbacks를 실제 동작하게 만든다.
- `PositionControl_Update()` 결과를 주기적으로 CSV 로그로 남길 수 있게 한다.
- target slew limit, derivative filter, output ramp 정책을 추가한다.
- post-mortem 분석용 fault/event trace buffer를 만든다.

### P2

- feedforward, disturbance compensation, gain scheduling을 검토한다.
- tuning profile을 speed range별로 분리한다.
- homing 후 zero offset drift 보정 정책까지 포함한다.
- 차량 상위 시스템과 연동되는 fault manager 인터페이스를 정식화한다.

## 7. 정량 증거를 어떻게 남기면 좋은가

A 파트는 아래 숫자가 핵심이다.

| 지표 | 의미 | 권장 증거 |
|---|---|---|
| settling time | 목표각 도달 시간 | step response CSV, 그래프 |
| overshoot | 과도 응답 품질 | 목표/현재각 비교 그래프 |
| steady-state error | 정착 후 오차 | 3초 유지 구간 평균 |
| deadline miss count | 1ms 루프 안정성 | latency CSV |
| estop reaction time | 안전 응답 속도 | GPIO/로그 timestamp |
| retune 횟수 대비 성능 개선 | 튜닝 품질 | before/after 표 |

권장 파일:
- `Doc/measurements/pos_step_2026-03-xx_run01.csv`
- `Doc/measurements/pos_step_2026-03-xx_run01_meta.md`
- `Doc/measurements/latency_2026-03-xx_run01.csv`

메타데이터에 반드시 넣을 것:
- Git SHA
- PID gain
- control mode
- target sequence
- SystemCoreClock
- 로그 on/off 여부

## 8. "얼마나 향상됐는가"를 보여주는 기준

가장 좋은 방식은 아래처럼 baseline을 고정하는 것이다.

| 항목 | Baseline | Target | 개선 표시법 |
|---|---|---|---|
| settling time | 예: 420ms | 250ms 이하 | `% 감소` |
| overshoot | 예: 18% | 5% 이하 | `%p 감소` |
| steady-state error | 예: 0.9deg | 0.2deg 이하 | `deg 감소` |
| deadline miss | 예: 14/10000 | 0/10000 | `count 감소` |
| estop reaction | 예: 35ms | 10ms 이하 | `ms 감소` |

면접에서 좋은 표현:
- "PID gain을 바꿨다"보다 "settling time을 420ms에서 240ms로 42.9% 줄였다"가 훨씬 강하다.

## 9. 취업에 도움이 되는 기록 전략

A 파트는 아래 흐름으로 정리하면 좋다.

1. 문제 정의
- "STM32 1ms 루프에서 encoder feedback 기반 position control을 만들었다"

2. 설계 결정
- 왜 float degree를 썼는지
- 왜 anti-windup와 saturation이 필요한지
- 왜 ESTOP을 relay까지 연결했는지

3. 개선 스토리
- D-kick 방지
- stale error safety bug 수정
- mode/state 실상태화

4. 숫자 근거
- settling time
- overshoot
- p99 control latency
- deadline miss

5. 한계와 다음 단계
- unit separation
- homing interlock
- formal fault manager

가장 추천하는 포트폴리오 문장:

> UDP 명령을 받아 1ms 주기의 encoder-feedback PID 위치제어로 pulse/direction을 생성했고, ESTOP과 latency 계측을 함께 설계해 제어기 자체의 동작성과 검증 가능성을 동시에 확보했다.
