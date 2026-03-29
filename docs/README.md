# Autonomous Steering Servo Control System

STM32F429ZI 기반 조향 서브컨트롤러 프로젝트다. 상위 제어기에서 받은 `steering_deg` 명령을 내부 `motor_deg`, `enc_count`, `pulse_hz`로 변환하고, 1 ms 제어 루프에서 pulse/direction 출력, safety, telemetry를 함께 관리한다.

## 1. Current Snapshot

| 항목 | 현재 기준 |
|---|---|
| MCU | STM32F429ZI, 180 MHz |
| Servo Driver | LS ELECTRIC XDL-L7SA004BAA |
| Motor / Encoder | XML-FBL04AMK1, 12000 PPR, quadrature x4 = 48000 count/rev |
| Loop | SysTick 기반 1 ms 제어 |
| External Unit | `steering_deg` |
| Internal Unit | `motor_deg`, `enc_count`, `pulse_hz` |
| Pulse Output | `PE9 = TIM1_CH1`, `PE10 = direction GPIO` |
| Encoder Input | `PA0 = TIM2_CH1`, `PB3 = TIM2_CH2` |
| Bench Default | keyboard bench ON, periodic CSV ON, real encoder diag ON |
| Virtual Feedback | 코드 경로는 존재하지만 현재 기본값은 `OFF` |
| Watchdog | IWDG 사용, 현재 설정 기준 약 32.8 s |
| Runtime Trace | CSV, command lifecycle, latency batch, `[ENCDBG]` real TIM2 snapshot |

## 2. Runtime Layout

- `main.c`는 CubeMX init 뒤 `AppRuntime_Init()`, `AppRuntime_RunIteration()`만 호출하는 얇은 부트 엔트리다.
- `app_runtime.c`는 startup sequence, keyboard bench, periodic CSV/DIAG, UDP handling, watchdog refresh, fast tick service를 담당한다.
- `position_control.c`는 PID, command lifecycle, enable/disable, emergency path를 담당한다.
- `position_control_diag.c`는 command state/result/source 문자열과 주기 상태 출력 같은 진단 책임을 맡는다.
- `position_control_safety.c`는 angle / tracking / velocity limit 평가를 담당한다.
- `pulse_control.c`는 signed `pulse_hz`를 `PE9` 펄스와 `PE10` 방향으로 변환한다.
- `encoder_reader.c`는 TIM2 raw counter를 읽고 누적 count와 각도로 변환하며, 선택적으로 virtual feedback도 받을 수 있다.

## 3. Current Pin / Timer Map

| 기능 | 핀 / 주변장치 | 비고 |
|---|---|---|
| Pulse | `PE9 / TIM1_CH1` | line driver input -> `PF+/PF-` |
| Direction | `PE10 / GPIO` | line driver input -> `PR+/PR-` |
| Encoder A | `PA0 / TIM2_CH1` | 현재 `GPIO_NOPULL` |
| Encoder B | `PB3 / TIM2_CH2` | 현재 `GPIO_NOPULL` |
| Timer Encoder | `TIM2` | 32-bit counter, `TIM_ENCODERMODE_TI12` |

현재 `tim.c` 기준:

- `TIM2` encoder mode
- `IC1Filter = 0`, `IC2Filter = 0`
- `GPIO_MODE_AF_PP`
- `GPIO_NOPULL`

즉 지금 실제 엔코더 truth를 보려면 `TIM2`, `PA0/PB3`, `[ENCDBG]` 기준으로 해석해야 한다.

## 4. Current Bench Defaults In Code

`app_runtime.c` 기준 기본 매크로는 현재 아래와 같다.

- `APP_RUNTIME_AUTO_FIXED_PULSE_TEST = 0`
- `APP_RUNTIME_KEYBOARD_TEST_MODE = 1`
- `APP_RUNTIME_ENCODER_DIAG_ENABLE = 1`
- `APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE = 0`
- `APP_RUNTIME_PERIODIC_CSV_LOG_ENABLE = 1`

부팅 시 동작:

- `HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL)`
- `Relay_Init()`, `PulseControl_Init()`, `EncoderReader_Init()`, `PositionControl_Init()`
- `Relay_ServoOn()`
- `EncoderReader_Reset()`
- `PositionControl_SetTargetWithSource(...0.0f...)`
- `PositionControl_Enable()`

즉 현재 baseline은 아직도 **startup auto-enable**이 남아 있는 bring-up 중심 구조다.

## 5. Current Feedback Modes

현재 `encoder_reader.c`는 두 가지 피드백 경로를 지원한다.

### 5.1 Real Encoder Path

- source: `TIM2->CNT`
- pins: `PA0/PB3`
- debug: `[ENCDBG] cnt / delta / A / B`
- 용도: 실제 하드웨어 truth 확인

### 5.2 Virtual Feedback Path

- source: `PulseControl_GetStatus().applied_frequency_hz`
- update: 1 ms 적분
- 용도: 실제 엔코더가 불안정할 때 controller-only bench

주의:

- 현재 기본 빌드는 `APP_RUNTIME_VIRTUAL_ENCODER_LOG_ENABLE = 0` 이다.
- virtual feedback을 켜면 CSV의 `current_deg`, `enc_cnt`, `enc_raw`가 실제 TIM2가 아니라 적분값이 될 수 있다.
- 실제 TIM2 하드웨어 엔코더 변화는 항상 `[ENCDBG]`로 확인하는 것이 안전하다.

## 6. Current Bench Interpretation

2026-03-29 기준 현재 소프트웨어 상태는 아래처럼 정리할 수 있다.

- keyboard bench에서 목표각을 주면 `mode`, `output`, `requested/applied Hz`는 정상적으로 갱신된다.
- `pulse_control.c` 기준 output contract는 살아 있고, `requested_frequency_hz`, `applied_frequency_hz`, `reverse_guard_active`를 함께 볼 수 있다.
- 실제 엔코더 테스트에서는 `[ENCDBG]`가 TIM2 카운터와 `A/B` 핀 상태를 찍는다.
- 최근 하드웨어 점검에서는 encoder A/B 채널 진폭이 균형적이지 않은 상황이 관찰되어, 실제 sensor truth는 아직 불안정하다.
- 따라서 현재 bench는 “소프트웨어 제어 구조와 pulse output 계약은 검증 가능하지만, 실제 encoder truth closure는 계속 하드웨어 bring-up 중”인 상태다.

## 7. Current Risks And Gaps

### 7.1 Real Encoder Truth

- 실제 TIM2 encoder path는 아직 완전히 닫히지 않았다.
- 최근 관찰 기준으로 A/B 두 채널 진폭이 균형적이지 않아, 실제 `cnt/delta`를 authoritative truth로 쓰기 어렵다.

### 7.2 Startup Safety

- boot-time auto enable이 남아 있다.
- homing, readiness, arm contract가 startup state machine으로 아직 닫히지 않았다.

### 7.3 Watchdog Policy

- IWDG는 존재하지만 timeout이 약 32.8 s라서 steering safe-state 기준으로는 너무 길다.
- “watchdog 존재”보다 “몇 ms 안에 SAFE로 전환하는가”를 다시 설계해야 한다.

### 7.4 Parameter Ownership

- `project_params.h`가 존재하지만 현재 app/runtime 전체가 그 파일 하나로 완전히 통일되진 않았다.
- 현재 운용 파라미터와 legacy 로컬 매크로가 혼재해 있어 추가 정리가 필요하다.

## 8. Important Source Files

| 파일 | 역할 |
|---|---|
| `Core/Src/main.c` | CubeMX init + app runtime 호출 |
| `Core/Src/app_runtime.c` | startup, keyboard bench, CSV/DIAG, watchdog, fast tick |
| `Core/Src/position_control.c` | PID, lifecycle, enable/disable, ESTOP |
| `Core/Src/position_control_diag.c` | 상태 문자열, diagnostic print |
| `Core/Src/position_control_safety.c` | safety limit evaluation |
| `Core/Src/pulse_control.c` | pulse/direction output, reverse guard, runtime status |
| `Core/Src/encoder_reader.c` | TIM2 raw -> count/angle, optional virtual feedback |
| `Core/Src/relay_control.c` | servo on / emergency relay control |
| `Core/Src/homing.c` | ADC potentiometer 기반 homing skeleton |

## 9. Documentation Index

| 문서 | 설명 |
|---|---|
| `Doc/README.md` | 문서 인덱스와 현재 runtime 기준 |
| `Doc/code_modules.md` | 모듈 역할과 ownership 관점 메모 |
| `Doc/command_lifecycle_no_homing_spec.md` | no-homing lifecycle 명세 |
| `Doc/REQ/steering_project_req_ownership_guide.html` | REQ / ownership / target state machine 정리 |
| `Doc/steering_portal/index.html` | 현재 구현과 evidence를 시각화한 로컬 포털 |
| `Doc/doxygen/html/index.html` | 코드 브라우저와 역할 요약 |
| `Doc/change_code/2026-03-29.md` | 오늘 변경 이력 |

## 10. One-Line Summary

현재 프로젝트는 `TIM2 real encoder debug`와 `virtual feedback bench`를 모두 가진 1 ms steering sub-controller baseline이며, 다음 핵심 과제는 **실제 encoder truth와 startup safety contract를 닫는 것**이다.

Last updated: 2026-03-29
