# position_control 상세 설명서

> 작성일: 2026-03-08
> 대상 독자: `position_control`을 처음 보는 팀원
> 목적: 조향 제어 시스템이 "어떤 입력을 받아", "어떤 순서로 계산하고", "어떻게 모터를 움직이는지"를 코드 기준으로 바로 이해할 수 있게 설명

---

## 1. 한 줄 요약

이 프로젝트의 `position_control`은 "목표 조향각"과 "현재 조향각"의 차이(오차)를 1ms마다 계산해서, 그 오차를 줄이는 방향으로 펄스 주파수와 회전 방향을 만들어 서보 드라이브에 보내는 위치 제어기다.

즉, 역할을 한 문장으로 줄이면 아래와 같다.

- 입력: 목표 각도
- 피드백: 엔코더 현재 각도
- 계산: PID
- 출력: TIM1 기반 펄스 주파수 + DIR 핀 방향
- 보호: 범위 초과, 추종 오차 초과 시 Emergency Stop

---

## 2. 시스템 전체 구조

`position_control.c`만 보면 계산식은 보이지만, 실제로는 아래 흐름 안에서 동작한다.

```text
PC / ASMS
  -> UDP 패킷 수신
  -> main.c가 모드와 목표각 갱신
  -> 1ms tick 발생
  -> PositionControl_Update()
  -> EncoderReader_GetAngleDeg()로 현재각 읽기
  -> PID 계산
  -> PulseControl_SetFrequency()로 펄스 출력
  -> 서보 드라이브가 모터 구동
  -> 엔코더가 다시 현재 위치 피드백
```

핵심은 `main.c`가 상위 상태머신 역할을 하고, `position_control.c`는 실제 제어 연산을 담당한다는 점이다.

---

## 3. 관련 파일별 역할

### `Core/Src/main.c`

- 시스템 초기화
- UDP 통신 폴링
- AUTO / MANUAL / NONE / ESTOP 모드 전이 처리
- 새 조향 목표각이 들어오면 `PositionControl_SetTarget()` 호출
- 1ms마다 `PositionControl_Update()` 호출

### `Core/Src/stm32f4xx_it.c`

- `SysTick_Handler()`에서 1ms마다 `interrupt_flag = 1` 설정
- 메인 루프는 이 플래그를 보고 제어 루프를 1회 실행

즉, 제어 루프는 인터럽트 안에서 직접 돌지 않고, 인터럽트가 "지금 1ms 제어 한 번 돌려라"라는 신호만 올려주는 구조다.

### `Core/Src/position_control.c`

- 현재 각도 읽기
- 목표각과 현재각의 오차 계산
- PID 계산
- 안전 검사
- 펄스 출력 명령 생성
- 안정화 여부 판단

### `Core/Src/encoder_reader.c`

- TIM4 엔코더 카운터를 읽어서 각도로 변환
- 현재 위치 피드백 제공

### `Core/Src/pulse_control.c`

- PID 출력값을 펄스 주파수로 해석
- 부호에 따라 DIR 핀 설정
- TIM1 PWM으로 실제 펄스 발생

### `Core/Src/ethernet_communication.c`

- UDP 패킷을 받아 `steering_angle`로 변환
- 현재 조향 모드 저장
- 비상정지 요청과 RX timeout 판단용 정보 제공

---

## 4. 부팅 후 처음 실행되는 순서

부팅 후 `main()`에서 조향 제어와 직접 연결되는 순서는 아래와 같다.

1. 하드웨어 초기화
2. `Relay_Init()`
3. `PulseControl_Init()`
4. `EncoderReader_Init()`
5. `PositionControl_Init()`
6. `Relay_ServoOn()`
7. `EncoderReader_Reset()`
8. `PositionControl_SetTarget(0.0f)`
9. `PositionControl_Enable()`
10. `EthComm_UDP_Init()`

이 순서를 보면 설계 의도가 보인다.

- 먼저 출력부와 센서부를 준비한다.
- 초기 목표각을 0도로 둔다.
- 그 다음에 제어기를 enable한다.
- 마지막에 네트워크 입력을 받기 시작한다.

즉, 통신보다 제어기 기본 상태를 먼저 안정적으로 만들어 놓는 구조다.

---

## 5. 제어에 들어오는 입력은 어디서 오나

실제 목표각은 `main.c`에서 네트워크 수신 데이터로 들어온다.

### AUTO 모드

- PC가 9바이트 패킷 전송
- `ethernet_communication.c`가 `g_latest_pkt.steering_angle`에 저장
- `main.c`가 `EthComm_HasNewData()`를 확인
- `PositionControl_SetTarget(pkt.steering_angle)` 호출

### MANUAL 모드

- ASMS가 5바이트 패킷 전송
- 조이스틱 `joy_y` 값이 `joy_to_deg()`로 각도로 변환
- 역시 `PositionControl_SetTarget()`으로 전달

즉 `position_control` 입장에서는 AUTO든 MANUAL이든 상관없이, 결국 "목표 각도 float 하나"만 받는다.

---

## 6. 1ms 제어 루프는 어떻게 돌까

제어 루프의 시간 기준은 `SysTick`이다.

### 동작 방식

1. `SysTick_Handler()`가 1ms마다 실행
2. `interrupt_flag = 1`
3. `main()`의 while 루프가 이 플래그를 확인
4. 플래그를 0으로 내리고 `PositionControl_Update()` 실행

이 방식의 장점은 다음과 같다.

- 인터럽트 핸들러를 짧게 유지할 수 있다.
- 제어 루프 본체가 인터럽트 컨텍스트 밖에서 돌아 디버깅이 쉽다.
- `LatencyProfiler_OnDeadlineTick()`으로 데드라인 miss 여부를 같이 체크할 수 있다.

---

## 7. `position_control.h`에서 먼저 봐야 할 것

처음 코드를 읽을 때는 함수보다 상수와 상태 구조체를 먼저 보는 게 이해가 빠르다.

### 핵심 상수

- `CONTROL_PERIOD_MS = 1`
  - 제어 주기 1ms
- `MAX_ANGLE_DEG = 4320.0f`
- `MIN_ANGLE_DEG = -4320.0f`
  - 12:1 기어비 기준 다회전 운용 범위
- `MAX_TRACKING_ERROR_DEG = 4500.0f`
  - 현재 위치와 목표 위치 차이가 너무 크면 비정상으로 판단
- `POSITION_TOLERANCE = 0.5f`
  - 오차가 0.5도 이내면 목표 근처로 본다

### 제어 상태 구조체

`PositionControl_State_t`는 현재 제어기의 스냅샷이다.

- `target_angle`
  - 가야 하는 목표 각도
- `current_angle`
  - 엔코더로 읽은 현재 각도
- `error`
  - `target_angle - current_angle`
- `output`
  - PID 계산 결과
- `is_stable`
  - 목표 근처에서 안정화되었는지 여부
- `stable_time_ms`
  - 허용 오차 범위 안에 머문 누적 시간
- `mode`
  - IDLE / POSITION / MANUAL / EMERGENCY

이 구조체 하나만 보면 현재 제어 상태를 거의 다 파악할 수 있다.

---

## 8. 내부 변수 구조

`position_control.c` 내부에는 크게 4종류의 정적 상태가 있다.

### 1. PID 파라미터 `pid_params`

```c
Kp = 50.0f
Ki = 5.0f
Kd = 20.0f
integral_limit = 1000.0f
output_limit = 10000.0f
```

주의할 점:

- 헤더의 `DEFAULT_KP`는 500인데, 실제 소스 초기값은 50이다.
- 실제 런타임에서 무엇을 기준값으로 보는지 팀 내에서 정리할 필요가 있다.

### 2. PID 내부 상태 `pid_state`

- `prev_error`
  - D항 계산용 이전 오차
- `integral`
  - I항 누적값
- `last_time_ms`
  - 직전 제어 시각

### 3. 외부에 노출되는 제어 상태 `state`

- 목표각, 현재각, 오차, 출력, 안정화 상태 등을 저장

### 4. 운용 플래그

- `control_enabled`
  - 제어 루프 활성/비활성
- `control_mode`
  - 현재 모드
- `fault_flag`
  - 안전 위반 원인
  - `1`: 각도 범위 초과
  - `2`: 추종 오차 초과

---

## 9. 초기화 함수 `PositionControl_Init()`

이 함수는 제어기의 내부 상태를 깨끗한 시작 상태로 만든다.

### 하는 일

1. `prev_error = 0`
2. `integral = 0`
3. `last_time_ms = HAL_GetTick()`
4. 목표각과 현재각 0으로 초기화
5. `control_enabled = false`
6. 모드 `IDLE`로 설정

### 의미

이 시점에서는 아직 모터를 움직이지 않는다.

즉 `Init()`은 "제어 계산 준비"만 하고, 실제 시작은 `PositionControl_Enable()`이 맡는다.

---

## 10. 가장 중요한 함수 `PositionControl_Update()`

이 함수가 제어기의 본체다. 1ms마다 한 번씩 불린다.

실행 순서를 정확히 따라가면 아래와 같다.

### 10-1. 제어 활성 여부 확인

```c
if (!control_enabled) {
    PulseControl_Stop();
    return;
}
```

의미:

- 제어가 꺼져 있으면 PWM 출력을 멈추고 종료
- 제어 비활성 상태에서 모터가 계속 도는 것을 방지

### 10-2. 현재 각도 읽기

```c
state.current_angle = EncoderReader_GetAngleDeg();
```

여기서 엔코더 카운터가 각도로 바뀐다.

`encoder_reader.c` 기준 계산 방식은:

- TIM4 카운터 읽기
- 기준값 32768을 빼서 signed count로 변환
- `0.0075 deg/count`를 곱해 각도로 변환

즉 현재 모터 위치를 도(degree) 단위로 얻는 단계다.

### 10-3. 오차 계산

```c
state.error = state.target_angle - state.current_angle;
```

이 값이 제어의 핵심이다.

- 양수 오차: 더 정방향으로 가야 함
- 음수 오차: 반대 방향으로 가야 함

중요한 점:

이 코드는 안전 검사보다 먼저 실행된다. 주석에도 적혀 있듯이 예전에는 안전 검사에서 이전 루프의 `state.error`를 보는 1-step 지연 버그가 있었고, 지금은 그 순서를 바로잡아 현재 루프 기준 오차로 안전 판단한다.

### 10-4. 안전 검사

```c
if (!PositionControl_CheckSafety()) {
    PositionControl_EmergencyStop();
    return;
}
```

여기서 위험 상태를 먼저 걸러낸다.

검사 항목은 아래 두 가지다.

1. 현재 각도가 물리 범위를 넘었는가
   - `MAX_ANGLE_DEG + 5`
   - `MIN_ANGLE_DEG - 5`
2. 목표와 현재의 차이가 너무 큰가
   - `fabs(error) > MAX_TRACKING_ERROR_DEG`

걸리면 즉시 Emergency Stop으로 빠진다.

### 10-5. 시간차 `dt` 계산

```c
uint32_t current_time = HAL_GetTick();
float dt = (current_time - pid_state.last_time_ms) / 1000.0f;
```

PID에서 I항과 D항 계산을 위해 필수다.

그리고 아래 보호 로직이 있다.

- `dt <= 0.0f` 이면 `0.001f`
- `dt > 0.1f` 이면 `0.1f`

의미:

- 너무 작으면 0으로 나누기 또는 D항 폭주 방지
- 너무 크면 제어 품질 악화 방지

### 10-6. PID 계산

```c
state.output = PID_Calculate(state.error, dt);
```

출력은 결국 "얼마나 빠르게 어떤 방향으로 회전시킬 것인가"를 나타내는 값이다.

### 10-7. 펄스 출력

```c
PulseControl_SetFrequency((int32_t)state.output);
```

이 한 줄이 실제 모터 구동 명령이다.

중요한 해석은 아래와 같다.

- 출력 부호: 방향
- 출력 절대값: 펄스 주파수

즉 `position_control`은 전압이나 토크를 직접 제어하지 않고, "펄스 속도 제어" 형태로 서보 드라이브를 움직인다.

### 10-8. 안정화 판단

오차 절대값이 `POSITION_TOLERANCE`보다 작으면 안정 구간에 들어간다.

- 허용 범위 내면 `stable_time_ms` 누적
- 100ms 이상 유지되면 `is_stable = true`
- 한 번이라도 범위를 벗어나면 누적 시간 초기화

이 값은 "목표에 도달했는가"를 상위 로직이나 디버깅에서 판단할 때 유용하다.

---

## 11. PID 계산 함수 `PID_Calculate()`

공식 자체는 표준 PID다.

```text
output = Kp * error
       + Ki * integral(error)
       + Kd * derivative(error)
```

코드에서의 흐름은 아래와 같다.

### P항

- 현재 오차가 크면 즉시 큰 출력을 만든다.
- 반응이 빠르지만 너무 크면 진동이 생길 수 있다.

### I항

- 오차를 누적해서 장기적인 편차를 줄인다.
- 대신 과도하게 쌓이면 wind-up이 생긴다.
- 그래서 `integral_limit`로 누적값을 제한한다.

### D항

- 오차 변화율을 본다.
- 급격한 변화에 브레이크처럼 작용한다.
- 노이즈와 `dt`에 민감하다.

### 출력 제한

- 계산 결과는 `output_limit` 범위로 clamp된다.
- 현재 소스 기준 `±10000`

이 제한은 제어기가 비정상적으로 큰 주파수를 명령하지 않도록 막는다.

---

## 12. 목표값 설정 `PositionControl_SetTarget()`

외부에서 새 목표 조향각이 들어오면 이 함수가 호출된다.

### 하는 일

1. 목표각 범위 검사
2. 인터럽트 잠시 차단
3. `state.target_angle` 갱신
4. 안정화 상태 초기화
5. 인터럽트 복구

### 왜 인터럽트를 막나

`target_angle`는 메인 루프와 1ms 제어 루프가 공유하는 값이다.

따라서 갱신 도중 제어 루프가 동시에 읽으면 일관성 문제가 생길 수 있다. 이를 막기 위해 짧은 임계영역을 만든다.

이 함수는 아래 의미를 가진다.

- 제어기를 바로 움직이는 함수는 아니다.
- "다음 1ms 업데이트부터 따라가야 할 목표"를 바꾸는 함수다.

---

## 13. 활성화/비활성화/비상정지

이 세 함수의 차이를 정확히 알아야 시스템이 이해된다.

### `PositionControl_Enable()`

하는 일:

- `control_enabled = true`
- `fault_flag = 0`
- 모드를 `POSITION`으로 변경
- EMG 릴레이 해제
- 현재 각도를 다시 읽음
- `prev_error`를 현재 오차로 맞춤
- 적분값 리셋
- 시간 기준 리셋

여기서 중요한 포인트는 D항 킥 방지다.

만약 enable 직후 `prev_error = 0` 상태라면 첫 루프에서

```text
derivative = (현재오차 - 0) / 0.001
```

처럼 계산되어 D항이 과도하게 튈 수 있다. 그래서 enable 시점의 실제 오차로 `prev_error`를 미리 맞춰 놓는다.

### `PositionControl_Disable()`

하는 일:

- `control_enabled = false`
- 모드 `IDLE`
- `PulseControl_Stop()`

의미:

- 정상적인 정지
- 위험 상황은 아님
- 제어 출력을 끄고 대기 상태로 돌아감

### `PositionControl_EmergencyStop()`

하는 일:

- `control_enabled = false`
- 모드 `EMERGENCY`
- PWM 정지
- 적분값 리셋
- `Relay_Emergency()` 호출

이 함수는 단순한 소프트웨어 stop이 아니라, 하드웨어 EMG 릴레이까지 건드리는 강제 정지 경로다.

즉 설계 의도는 아래와 같다.

- `Disable()`: 정상 정지
- `EmergencyStop()`: 위험 상황 차단

---

## 14. 안전 검사 `PositionControl_CheckSafety()`

현재 구현은 단순하지만 매우 중요하다.

### 체크 1. 현재 각도 한계

```c
state.current_angle > MAX_ANGLE_DEG + 5.0f
state.current_angle < MIN_ANGLE_DEG - 5.0f
```

의미:

- 허용 운용 범위를 약간 넘으면 비정상
- 센서 이상, 기구 이상, 기준점 문제를 빠르게 감지

### 체크 2. 추종 오차 한계

```c
fabsf(state.error) > MAX_TRACKING_ERROR_DEG
```

의미:

- 목표와 실제 위치 차이가 너무 크면 정상 제어가 아니라고 판단
- 명령 오류, 기계적 걸림, 엔코더 문제, 방향 반전 문제 등을 간접 감지

### `fault_flag`

- `1`: 현재 각도 범위 초과
- `2`: 추종 오차 초과

비상정지 로그를 볼 때 원인 판단에 직접 사용된다.

---

## 15. 엔코더 피드백이 각도로 바뀌는 방식

`EncoderReader_GetAngleDeg()`를 보면 위치 피드백 계산은 생각보다 단순하다.

### 계산 순서

1. TIM4 현재 카운터 읽기
2. 기준값 32768을 빼서 signed 값으로 변환
3. offset 반영
4. `DEG_PER_COUNT` 곱하기

### 상수 의미

- `PULSE_PER_REV = 12000`
- `QUADRATURE = 4`
- `COUNT_PER_REV = 48000`
- `DEG_PER_COUNT = 360 / 48000 = 0.0075도`

즉 엔코더 카운트 1개가 약 0.0075도다.

### 왜 32768에서 시작하나

TIM4는 16비트 타이머라서 범위가 `0 ~ 65535`다. 가운데 값인 32768에서 시작하면 양/음 방향으로 여유가 생겨 signed 해석이 편해진다.

---

## 16. PID 출력이 실제 모터 구동으로 바뀌는 방식

`PulseControl_SetFrequency()`는 `position_control`의 출력값을 하드웨어 신호로 번역한다.

### 입력 해석

- `freq_hz == 0`
  - PWM 정지
- `freq_hz > 0`
  - DIR 핀 High
- `freq_hz < 0`
  - DIR 핀 Low, 절대값으로 변환

즉 `state.output = -3000`이면:

- 반대 방향
- 3000Hz 펄스 출력

### 추가 제한

- 최대 100000Hz
- 최소 10Hz

### 최종 동작

- TIM1 ARR 계산
- compare 값을 `arr / 2`로 넣어 50% duty 생성
- PWM 시작

결론적으로 `position_control`은 "오차를 줄이기 위한 회전 속도 명령"을 만들고, `pulse_control`은 그 명령을 "실제 전기적 펄스"로 바꾼다.

---

## 17. 모드 전이는 누가 관리하나

모드 전이는 `position_control`이 아니라 `main.c`가 관리한다.

### `main.c`의 역할

- 현재 조향 모드 읽기
- RX timeout이면 ESTOP으로 강제 전환
- `STEER_MODE_ESTOP` 진입 시 `PositionControl_EmergencyStop()`
- `STEER_MODE_NONE` 진입 시 `PositionControl_Disable()`
- `NONE` 또는 `ESTOP`에서 `AUTO/MANUAL`로 복귀 시
  - `Relay_EmergencyRelease()`
  - `PositionControl_Enable()`

즉 `position_control`은 제어기이고, "언제 켜고 끌지"는 상위 상태머신이 결정한다.

---

## 18. 실제 실행 예시

처음 보는 사람이 가장 빨리 이해하려면 구체적인 상황을 한 번 따라가면 된다.

### 예시: 목표각이 0도에서 120도로 바뀌는 경우

1. PC가 UDP로 `steering_angle = 120` 전송
2. `ethernet_communication.c`가 최신 패킷 저장
3. `main.c`가 새 데이터 확인 후 `PositionControl_SetTarget(120.0f)` 호출
4. 다음 1ms tick에서 `PositionControl_Update()` 실행
5. 엔코더가 현재각 예를 들어 `30도`라고 읽힘
6. 오차는 `120 - 30 = 90도`
7. PID 계산 결과 예를 들어 `+5200`
8. `PulseControl_SetFrequency(5200)` 호출
9. DIR 핀 정방향 설정, TIM1이 5200Hz 펄스 출력
10. 모터가 목표 방향으로 회전
11. 다음 루프에서는 현재각이 31도, 32도, 33도처럼 바뀜
12. 오차가 줄어들면서 출력도 점점 줄어듦
13. 오차가 0.5도 이하에서 100ms 유지되면 `is_stable = true`

이 예시 하나가 이 시스템 전체 동작 원리다.

---

## 19. 코드 읽을 때 꼭 알아야 할 포인트

### 1. `PositionControl_Update()`가 진짜 중심이다

다른 함수는 대부분 준비, 설정, 상태 변경이다. 실제 제어는 거의 전부 `Update()` 안에 있다.

### 2. 제어기는 "각도 PID"지만 출력은 "펄스 주파수"다

전통적인 전류 제어나 토크 제어가 아니라, 서보 드라이브의 pulse/direction 입력을 이용한 위치 제어 구조다.

### 3. `Enable()`과 `Init()`은 역할이 다르다

- `Init()`: 내부 변수 초기화
- `Enable()`: 실제 제어 시작 준비

### 4. 안전 검사는 계산 초반에 한다

위험 상태를 늦게 감지하면 이미 잘못된 구동 명령이 나갈 수 있으므로, 현재 오차를 계산한 직후 바로 검사한다.

### 5. `target_angle` 갱신은 임계영역 보호가 들어간다

이 부분은 인터럽트/메인 루프 동시 접근 문제를 막기 위한 장치다.

---

## 20. 디버깅할 때 먼저 볼 값

문제가 생기면 아래 순서로 보는 게 빠르다.

### 1. 모드

- `PositionControl_GetMode()`
- 지금 IDLE인지, POSITION인지, EMERGENCY인지 확인

### 2. 목표각 / 현재각 / 오차

- `state.target_angle`
- `state.current_angle`
- `state.error`

목표는 바뀌는데 현재가 안 따라오면 출력부나 배선 문제일 가능성이 크다.

### 3. 출력값

- `state.output`

출력이 0에 가깝다면 PID 게인이나 오차 계산을 먼저 봐야 한다.

### 4. DIR 핀과 TIM1 ARR/CCR

`main.c`의 진단 로그가 이미 아래 값을 찍도록 되어 있다.

- ARR
- CCR
- DIR
- ENC raw

즉 소프트웨어 출력이 실제 타이머 레지스터에 반영되는지 확인 가능하다.

### 5. `fault_flag`

EMERGENCY가 걸렸다면

- `1`: 각도 범위 문제
- `2`: 추종 오차 문제

부터 의심하면 된다.

---

## 21. 현재 코드 기준 주의할 점

처음 보는 사람이 놓치기 쉬운 부분들을 정리하면 아래와 같다.

### 1. 헤더 기본 PID와 소스 초기 PID가 다르다

- 헤더: `DEFAULT_KP = 500.0f`
- 소스: 실제 초기값 `Kp = 50.0f`

문서, 튜닝값, 실운용값 정합성을 한 번 정리하는 게 좋다.

### 2. `PositionControl_State_t`의 `last_error`는 현재 코드에서 갱신되지 않는다

구조체에는 있지만 실제 운용 정보로 쓰이지 않는다.

### 3. `PositionControl_SetSafetyLimits()` 등 일부 API는 아직 미구현이다

헤더만 보고 다 구현된 제어기라고 생각하면 안 된다.

### 4. 엔코더는 현재 단순 signed 변환 기반이다

장시간 다회전 누적 운용에서는 16비트 타이머 특성 때문에 unwrap 전략을 더 정교하게 가져갈 필요가 있을 수 있다.

### 5. `PositionControl_Update()`는 1ms 주기를 전제로 한다

주기가 크게 흔들리면 PID 성능이 바로 나빠진다.

---

## 22. 처음 보는 사람이 기억해야 할 핵심 5개

1. `main.c`가 목표각을 넣고, `position_control.c`가 계산하고, `pulse_control.c`가 모터를 실제로 움직인다.
2. `PositionControl_Update()`는 1ms마다 한 번씩 돌면서 현재각, 오차, PID 출력, 안전 상태를 갱신한다.
3. PID 출력값의 부호는 방향이고, 크기는 펄스 주파수다.
4. 위험하면 `PositionControl_EmergencyStop()`이 호출되고, 이때는 단순 소프트 stop이 아니라 EMG 릴레이까지 동작한다.
5. 제어기 이해의 핵심 변수는 `target_angle`, `current_angle`, `error`, `output`, `control_enabled` 다섯 개다.

---

## 23. 요약

이 시스템의 `position_control`은 독립적인 모듈처럼 보이지만, 실제로는 아래 연결 안에서 이해해야 한다.

```text
UDP 입력(main/ethernet)
  -> 목표각 설정(PositionControl_SetTarget)
  -> 1ms 루프(PositionControl_Update)
  -> 현재각 읽기(EncoderReader)
  -> 오차 계산 + PID
  -> 펄스 출력(PulseControl)
  -> 서보 구동
  -> 엔코더 피드백
```

따라서 `position_control`을 이해한다는 것은 단순히 PID 수식을 이해하는 것이 아니라,

- 목표값이 어디서 오는지
- 현재값을 어떻게 읽는지
- 출력이 어떤 하드웨어 신호로 바뀌는지
- 위험할 때 어떻게 멈추는지

까지 한 번에 연결해서 이해하는 것이다.

이 문서를 읽고 나면 다음 단계로는 실제 디버그 로그 한 번과 `PositionControl_Update()` 단위 코드 추적을 같이 해보는 것이 가장 효과적이다.
