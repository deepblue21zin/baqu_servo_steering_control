# 팀원 B 문서

분석 기준일: 2026-03-14

담당 범위: `homing`, `relay_control`

분석 기준:
- 실제 구현은 `Core/Src/main.c`, `Core/Src/homing.c`, `Core/Src/relay_control.c`, `Core/Src/adc_potentiometer.c`, `Core/Src/position_control.c`를 기준으로 봤다.
- 이 파트는 단독 모듈보다 startup, estop, recovery 전체 시퀀스를 책임지는 성격이 강하다.

## 1. 전체 아키텍처에서 B 파트의 위치

B 파트는 현재 코드에서 "안전한 시작과 정지"를 담당해야 하는 위치다.

이상적인 흐름은 아래와 같다.

1. 전원 인가
2. relay 안전 초기 상태 설정
3. ADC 절대각 읽기
4. homing으로 encoder 기준점 동기화
5. homing 성공 확인
6. ServoOn + EMG release
7. PositionControl_Enable
8. fault 발생 시 EMG와 Servo 상태를 안전하게 되돌림

하지만 현재 실제 구현은 아래에 가깝다.

1. `Relay_Init()`
2. `Relay_ServoOn()`
3. homing 경로는 주석 처리
4. `EncoderReader_Reset()`
5. `PositionControl_Enable()`

즉 B 파트는 아키텍처상 중요하지만, 현재 런타임에서는 일부만 반영된 상태다.

## 2. 지금 서로 주고받는 정보와 형식

| Producer | Consumer | 항목 | 형식 | 현재 의미 |
|---|---|---|---|---|
| `adc_potentiometer.c` | `homing.c` | pot angle | `float` | 절대 위치 추정값 |
| `homing.c` | `encoder_reader.c` | offset count | `int32_t` | 엔코더 원점 보정 |
| `main.c` | `relay_control.c` | Servo on/off | function call | 기동 시 서보 인가 |
| `position_control.c` | `relay_control.c` | emergency | function call | fault 시 EMG LOW |
| `main.c` | `position_control.c` | recovery sequence | function call | `Relay_EmergencyRelease()` 후 enable |

핵심 문제:
- B 파트가 실제로 시스템 상태를 publish하지 않는다.
- `Homing_IsComplete()`가 런타임 경로에서 gating 역할을 하지 않는다.
- relay state와 homing state가 하나의 state machine으로 묶여 있지 않다.

## 3. 현재 코드 구현 상세 분석

### 3.1 relay_control.c

`relay_control.c`는 현재 매우 단순하고 명확하다.

- `Relay_Init()`: `SVON=HIGH`, `EMG=HIGH`
- `Relay_ServoOn()`: `SVON=LOW`
- `Relay_ServoOff()`: `SVON=HIGH`
- `Relay_Emergency()`: `EMG=LOW`
- `Relay_EmergencyRelease()`: `EMG=HIGH`

장점:
- active low 논리가 코드에 잘 반영돼 있다.
- GPIO wrapper로 분리돼 있어 가독성이 좋다.
- fault 시 `PositionControl_EmergencyStop()`에서 직접 호출 가능하다.

한계:
- 상태 기억이 없다.
- 호출 순서 검증이 없다.
- ServoOn과 EMG release 사이 지연 시간 계약이 없다.
- recovery 조건 확인 없이 release가 가능하다.

### 3.2 homing.c

`Homing_FindZero()`의 실제 구현은 매우 짧다.

1. `ADC_Pot_GetAngle()`
2. `EncoderReader_Reset()`
3. `offset_count = pot_angle * ENCODER_COUNT_PER_REV / FULL_ROTATION_DEG`
4. `EncoderReader_SetOffset(offset_count)`
5. status를 `COMPLETE`로 설정

장점:
- 최소한의 homing 개념은 코드로 있다.
- ADC absolute reference를 encoder relative count와 연결하려는 방향은 맞다.

심각한 부족점:
- `ADC_Pot_Init()`가 선행됐는지 확인하지 않는다.
- ADC 실패, 이상값, 범위 오류를 전혀 검사하지 않는다.
- homing 실패 경로가 사실상 없다.
- 실제 기동 루프에서 호출되지 않는다.
- homing 완료 여부가 `PositionControl_Enable()`의 전제조건이 아니다.

### 3.3 main.c와의 실제 통합 상태

현재 `main.c`를 보면 homing 관련 코드는 모두 주석이다.

- `Homing_Init()`
- `ADC_Pot_Init(NULL)`
- `Homing_FindZero()`
- `Homing_IsComplete()` 검사

반대로 실제 동작은 아래와 같다.

1. `Relay_Init()`
2. `PulseControl_Init()`
3. `EncoderReader_Init()`
4. `PositionControl_Init()`
5. `Relay_ServoOn()`
6. `HAL_Delay(500)`
7. `EncoderReader_Reset()`
8. `PositionControl_SetTarget(0.0f)`
9. `PositionControl_Enable()`

즉 현재는 "현재 위치를 그냥 0으로 놓고 시작하는 테스트 모드"에 가깝다.

### 3.4 ESTOP와 recovery

현재 ESTOP 경로는 살아 있다.

- fault 또는 comm timeout 또는 brake bit
- `PositionControl_EmergencyStop()`
- 내부에서 `PulseControl_Stop() + Relay_Emergency()`

Recovery는 아래처럼 된다.

- `main.c`에서 `AUTO/MANUAL`로 진입
- `Relay_EmergencyRelease()`
- `PositionControl_Enable()`

문제는 recovery 전에 homing 재검증, sensor health 확인, servo on 상태 확인이 없다.

## 4. 현재 평가

성숙도 기준:
- L1: 함수 골격만 있음
- L2: bench 수준 동작
- L3: 실제 부팅 경로에 통합
- L4: fault/recovery 근거까지 갖춤
- L5: 현업형 startup safety로 설명 가능

현재 성숙도: `L2`

평가 근거:
- relay 제어 자체는 간단하고 동작 가능하다.
- 하지만 homing은 실제 startup에 아직 들어가 있지 않다.
- recovery와 startup safety가 문서/코드/시험으로 묶여 있지 않다.

자율주행 조향모터의 현업 기준으로 보면 아래 갭이 추가로 크다.

- fail-safe 완성도:
  - MCU 재부팅, brownout, watchdog reset 이후 safe restart 규칙이 없다.
  - EMG release와 ServoOn이 조건 검증 없이 호출될 수 있다.
  - homing timeout, retry budget, latched estop clear 절차가 없다.
- debug 완성도:
  - startup/recovery state transition timestamp와 reason log가 없다.
  - relay command와 실제 상태를 대조할 shadow state 또는 feedback contract가 없다.

한 줄 평가:
- "안전 구조의 뼈대는 있으나, 실제 부팅 시퀀스와 복귀 절차로 완성되지는 않은 상태"

## 5. B 파트 REQ 초안

- `REQ-B-001`: 전원 인가 또는 MCU reset 직후 기본 상태는 반드시 torque-off 상태(`SVON=OFF`)여야 하며, `EMG` 기본 상태는 hazard analysis 결과에 따라 고정하고 문서화해야 한다.
- `REQ-B-002`: homing은 `ADC init -> ADC read -> encoder reset -> offset apply -> validation` 순서로 수행되어야 한다.
- `REQ-B-003`: homing 실패 시 `RUN` 또는 `PositionControl_Enable()` 진입을 허용하지 않아야 한다.
- `REQ-B-004`: `Relay_ServoOn`, `Relay_ServoOff`, `Relay_Emergency`, `Relay_EmergencyRelease`의 허용 순서와 지연 시간을 문서화해야 한다.
- `REQ-B-005`: startup, homing, ready, running, estop, recovery 상태를 하나의 state machine으로 정의해야 한다.
- `REQ-B-006`: recovery 전에 homing 완료, sensor 정상, comm 정상, operator 승인 같은 조건을 확인해야 한다.
- `REQ-B-007`: EMG active 상태에서는 pulse 출력 재개를 허용하지 않아야 한다.
- `REQ-B-008`: relay state는 디버그 로그 또는 상태 구조체로 조회 가능해야 한다.
- `REQ-B-009`: homing 결과는 성공/실패뿐 아니라 실패 원인을 fault code로 남겨야 한다.
- `REQ-B-010`: startup과 recovery 절차는 실제 GPIO 타이밍 근거와 함께 검증되어야 한다.
- `REQ-B-011`: homing은 최대 수행 시간과 retry 횟수를 가져야 하며, 초과 시 즉시 fault 상태로 전이해야 한다.
- `REQ-B-012`: watchdog reset, brownout, software reset 이후에는 자동 운전 복귀가 아니라 safe startup 시퀀스로 재진입해야 한다.
- `REQ-B-013`: relay command와 실제 상태 사이 불일치를 감지할 수 있어야 하며, feedback 핀이 없으면 최소한 shadow state와 timeout 검증을 둬야 한다.
- `REQ-B-014`: `EMG release`, `ServoOn`, `Control Enable`은 개별 단계로 검증되어야 하며, 어느 한 단계라도 실패하면 다음 단계로 진행하면 안 된다.
- `REQ-B-015`: startup/recovery/estop 전이는 timestamp, reason, precondition 결과와 함께 로그로 남아야 한다.
- `REQ-B-016`: latched ESTOP에서 이탈하려면 명시적 fault clear 절차가 필요해야 하며, AUTO/MANUAL 모드 진입만으로 자동 복귀되면 안 된다.

인수 기준:

1. 전원 인가 후 homing 성공 시에만 `RUN`으로 진입한다.
2. ADC 이상 또는 homing 실패 시 control enable이 차단된다.
3. ESTOP 시 `EMG=LOW`, 필요 시 `SVON=OFF` 정책이 로그/실측과 일치한다.
4. recovery 절차가 state machine과 코드에서 동일하다.
5. watchdog reset 또는 brownout 이후 시스템이 자동 구동하지 않고 safe startup으로 돌아간다.
6. startup/recovery 전이 로그만으로 "왜 RUN에 들어갔는지"와 "왜 막혔는지"를 설명할 수 있다.

## 6. 어떻게 더 발전시켜야 하는가

### P0

- `main.c`에 homing 시퀀스를 실제로 통합한다.
- `Homing_FindZero()`에 ADC init 여부, 범위 체크, 실패 분기를 넣는다.
- `PositionControl_Enable()` 전에 `Homing_IsComplete()` 검사를 강제한다.
- relay와 homing을 하나의 system state table로 정리한다.
- `AUTO/MANUAL` 진입만으로 자동 복귀되지 않도록 latched recovery 절차를 만든다.
- power-up/reset/watchdog 이후 safe startup 재진입 정책을 명문화한다.

### P1

- recovery 절차를 정식으로 만든다.
- `EMG release -> ServoOn -> settle delay -> Enable` 순서를 명문화한다.
- GPIO 상태를 읽어 self-check하거나 최소한 shadow state를 둔다.
- startup 로그를 자동 저장해 부팅 시퀀스를 재현 가능하게 만든다.
- homing timeout, retry, operator reset 버튼/명령 정책을 추가한다.
- state transition reason code를 fault manager와 연결한다.

### P2

- power cycle, sensor disconnect, estop latch, operator reset 시나리오를 추가한다.
- homing retry 횟수와 제한시간을 설계한다.
- 하드웨어 인터락과 소프트웨어 state machine을 분리해 설계한다.
- relay 피드백 접점이나 drive ready 신호가 있다면 readback 기반 검증으로 확장한다.

## 7. 정량 증거를 어떻게 남기면 좋은가

B 파트는 "안전하게 시작하고, 안전하게 멈추고, 안전하게 복귀하는가"를 숫자로 보여줘야 한다.

| 지표 | 의미 | 권장 증거 |
|---|---|---|
| boot-to-ready time | 부팅 후 운전 가능까지 걸린 시간 | UART log timestamp |
| homing success rate | 재현성 | 100회 반복 결과 |
| EMG reaction time | fault 후 릴레이 반응 시간 | 로직애널라이저, GPIO 캡처 |
| recovery time | estop 후 재가동 시간 | state transition log |
| false release count | 잘못된 EMG 해제 횟수 | fault injection log |
| blocked-run count | homing 실패 시 RUN 차단 횟수 | negative test log |

권장 파일:
- `Doc/measurements/startup_2026-03-xx_run01.log`
- `Doc/measurements/homing_2026-03-xx_run01.csv`
- `Doc/measurements/estop_gpio_2026-03-xx_run01.png`

## 8. "얼마나 향상됐는가"를 보여주는 기준

| 항목 | Baseline | Target | 개선 표시법 |
|---|---|---|---|
| homing success rate | 예: 70/100 | 99/100 이상 | `% 향상` |
| boot-to-ready | 예: 2.3s | 1.2s 이하 | `s 감소` |
| estop reaction | 예: 28ms | 10ms 이하 | `ms 감소` |
| recovery sequence error | 예: 8/100 | 0/100 | `count 감소` |
| run-block on failure | 예: 일부 누락 | 100% 차단 | `coverage` |

면접에서 좋은 표현:
- "relay를 제어했다"보다 "fault injection 100회에서 EMG 반응 실패 0건을 확인했다"가 훨씬 강하다.

## 9. 취업에 도움이 되는 기록 전략

B 파트는 아래 자료가 있으면 강하다.

1. startup state machine 1장
- `INIT -> HOMING -> READY -> RUNNING -> ESTOP -> RECOVERY`

2. relay truth table 1장
- 상태별 `SVON`, `EMG`, `Enable` 허용 여부

3. fault injection 영상 또는 캡처
- 케이블 제거
- ADC 이상값
- operator estop

4. 로그 묶음
- boot log
- homing result
- recovery log

5. 설계 의도
- 왜 homing 이전에 run을 막아야 하는지
- 왜 recovery 전에 조건 확인이 필요한지

가장 추천하는 포트폴리오 문장:

> 안전 초기 상태, homing, ESTOP, recovery를 분리해서 설계하고, 서보 인가와 EMG 제어를 startup state machine으로 연결해 조향 액추에이터의 운전 가능 조건을 명확히 정의했다.

## 10. 지금부터 팀원 B에게 할당할 상세 개발 REQ

이 섹션은 B 파트를 "실제 차량/벤치에서 안전하게 켜고, 안전하게 멈추고, 안전하게 다시 올리는" 책임으로 다시 쪼갠 실행형 요구사항이다. B 파트는 릴레이와 homing을 각각 따로 만드는 것이 아니라, startup과 recovery를 하나의 state machine으로 닫는 것이 핵심이다.

### `REQ-B-017`: startup state machine을 코드로 구현해야 한다.
- 목적:
  - 지금처럼 초기화 코드가 순차 호출만 되고 운전 가능 조건이 불명확한 상태를 없앤다.
- 구현 범위:
  - 최소 상태를 `BOOT`, `SERVO_OFF`, `SERVO_ON_WAIT`, `HOMING`, `READY`, `RUN`, `ESTOP_LATCH`, `RECOVERY`로 정의한다.
  - 상태 전이마다 entry action, exit condition, timeout, failure action을 둔다.
- 완료 기준:
  - state diagram 1장과 실제 코드 enum/state variable이 일치한다.
  - 부팅 로그만 봐도 현재 상태를 알 수 있다.
- 검증 방법:
  - cold boot 10회 반복
  - UART state transition log 확인
- 산출물:
  - startup state machine 문서
  - state transition log

### `REQ-B-018`: homing은 startup path 안으로 실제 통합되어야 한다.
- 목적:
  - 부팅 시점을 무조건 `0deg`로 믿는 상대 영점 방식에서 벗어나 절대 기준 또는 검증된 영점을 확보한다.
- 구현 범위:
  - `ADC init -> sample validity check -> homing reference decision -> encoder reset/offset apply -> homing validation` 흐름을 startup state 안에 넣는다.
  - homing skip 조건이 있다면 그것도 명시적으로 관리한다.
- 완료 기준:
  - homing 성공 전에는 `RUN` 진입이 차단된다.
  - homing 실패 시 fault reason이 남고 `READY`로 가지 않는다.
- 검증 방법:
  - 정상 부팅
  - ADC 이상값/센서 분리 상태 부팅
- 산출물:
  - homing sequence 문서
  - success/fail 로그 세트

### `REQ-B-019`: relay truth table과 GPIO 시퀀스를 문서와 코드에서 동일하게 유지해야 한다.
- 목적:
  - `SVON`, `EMG`, `Enable`, `Pulse allowed`의 관계를 누구나 같은 의미로 이해하게 만든다.
- 구현 범위:
  - 각 상태에서 `SVON`, `EMG`, `pulse inhibit`, `control enable` 허용 여부를 표로 고정한다.
  - `Relay_ServoOn`, `Relay_ServoOff`, `Relay_Emergency`, `Relay_EmergencyRelease`의 지연 시간과 순서를 정의한다.
- 완료 기준:
  - GPIO 레벨 정의와 문서 truth table이 모순되지 않는다.
  - logic analyzer로 시퀀스가 재현된다.
- 검증 방법:
  - GPIO 캡처
  - 상태별 readback 로그
- 산출물:
  - relay truth table
  - GPIO timing capture

### `REQ-B-020`: ESTOP은 latch 상태로 유지되고 명시적 clear 절차 없이는 해제되면 안 된다.
- 목적:
  - 일시적 신호 복구만으로 갑자기 재가동하는 위험을 제거한다.
- 구현 범위:
  - `ESTOP_LATCH` 상태를 독립적으로 두고, clear API 또는 operator 승인 이벤트를 만들어야 한다.
  - `AUTO/MANUAL` 모드 복귀만으로 자동 해제되지 않게 한다.
- 완료 기준:
  - `X` 또는 fault로 ESTOP이 걸리면 명시적 clear 전까지 `RUN` 복귀가 차단된다.
  - clear 시 precondition 검증 로그가 남는다.
- 검증 방법:
  - ESTOP 20회 반복
  - clear 없이 재enable 시도
- 산출물:
  - ESTOP clear 절차 문서
  - negative test log

### `REQ-B-021`: watchdog reset, brownout, software reset 후에는 항상 safe startup으로 복귀해야 한다.
- 목적:
  - 리셋 원인이 불분명한 상태에서 이전 운전 상태를 이어받는 위험을 막는다.
- 구현 범위:
  - reset reason을 읽어서 로그에 남긴다.
  - 어떤 reset이든 `SERVO_OFF` 또는 equivalent safe state부터 재진입한다.
  - 필요하면 reset 후 fault history를 보존한다.
- 완료 기준:
  - watchdog reset 후 자동 pulse 출력이 없다.
  - reset cause가 boot log에 남는다.
- 검증 방법:
  - watchdog intentionally trigger
  - reset reason 로그 확인
- 산출물:
  - reset handling 정책
  - boot log 예시

### `REQ-B-022`: A, C, D와 연결되는 ready precondition contract를 정의해야 한다.
- 목적:
  - startup 완료 여부를 감으로 판단하지 않고, 파트 간 조건 계약으로 관리한다.
- 구현 범위:
  - `homing complete`, `sensor valid`, `driver ready`, `emg released`, `fault clear`, `manual arm`을 `READY` 조건으로 묶는다.
  - B 파트가 이 조건의 최종 gatekeeper 역할을 하도록 만든다.
- 완료 기준:
  - `READY=false` 이유를 비트마스크나 로그로 확인할 수 있다.
  - 어느 조건이 빠져도 `RUN` 진입이 차단된다.
- 검증 방법:
  - 조건 하나씩 누락시키는 negative test
- 산출물:
  - ready contract 표
  - precondition bitmask 정의

### `REQ-B-023`: startup / recovery / estop 전이는 timestamp와 reason을 남기는 event log를 가져야 한다.
- 목적:
  - 나중에 "왜 그 상태에서 멈췄는지"를 UART 로그만으로 복원할 수 있게 만든다.
- 구현 범위:
  - 각 상태 전이에 `timestamp`, `from`, `to`, `reason`, `precondition result`를 남긴다.
  - 최근 N개 event를 circular buffer로 조회 가능하게 해도 좋다.
- 완료 기준:
  - 현장 로그 하나로 startup에서 run까지, estop에서 recovery까지 추적 가능하다.
- 검증 방법:
  - boot-run-estop-recovery 시퀀스 1회 전체 로그 저장
- 산출물:
  - event log 포맷
  - 대표 로그 세트

### `REQ-B-024`: startup / homing / estop은 positive test뿐 아니라 negative test coverage까지 확보해야 한다.
- 목적:
  - "잘 될 때만 되는" 수준을 넘어서 현업형 안전 신뢰성을 확보한다.
- 구현 범위:
  - 센서 미연결
  - homing timeout
  - relay 명령 후 상태 미반영
  - operator estop during homing
  - recovery 중 재fault
  - 위 5개 이상의 negative case를 시험한다.
- 완료 기준:
  - 각 negative case에서 safe state로 귀결되고 로그가 남는다.
  - `RUN` 우회 진입이 없다.
- 검증 방법:
  - negative test checklist 수행
- 산출물:
  - failure case report
  - coverage 표
