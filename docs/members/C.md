# 팀원 C 문서

분석 기준일: 2026-03-14

담당 범위: `adc_potentiometer`, `encoder_reader`

분석 기준:
- 실제 구현은 `Core/Src/encoder_reader.c`, `Core/Src/adc_potentiometer.c`, `Core/Src/homing.c`, `Core/Src/position_control.c`, `Core/Src/tim.c`, `Core/Src/adc.c`를 기준으로 봤다.
- C 파트는 현재 구조에서 센서 계층 전체를 맡는다고 보는 것이 맞다.

## 1. 전체 아키텍처에서 C 파트의 위치

현재 센서 데이터 흐름은 아래와 같다.

1. TIM4가 엔코더 A/B상을 하드웨어 카운트한다.
2. `encoder_reader.c`가 TIM4 counter를 읽어 angle로 환산한다.
3. `position_control.c`가 그 값을 현재각으로 사용한다.
4. ADC1 CH4가 포텐셔미터 raw 값을 읽는다.
5. `adc_potentiometer.c`가 raw를 angle로 환산한다.
6. `homing.c`가 ADC angle을 이용해 encoder offset을 맞춘다.

현재 실동작 기준으로는:
- 엔코더는 runtime에서 실제 사용 중
- ADC는 코드상 존재하지만 main runtime에서는 사실상 미사용

즉 C 파트는 일부는 실제 제어 루프에 들어가 있고, 일부는 아직 준비 단계다.

## 2. 지금 서로 주고받는 정보와 형식

| Producer | Consumer | 항목 | 형식 | 현재 의미 |
|---|---|---|---|---|
| TIM4 | `encoder_reader.c` | raw counter | `uint16_t` | 0~65535 |
| `encoder_reader.c` | `position_control.c` | current angle | `float` | `deg`로 환산된 현재값 |
| `encoder_reader.c` | `homing.c` | offset apply | `int32_t` | 엔코더 영점 보정 |
| ADC1 | `adc_potentiometer.c` | raw value | `uint16_t` | 0~4095 |
| `adc_potentiometer.c` | `homing.c` | pot angle | `float` | 절대 위치 추정값 |

현재 인터페이스의 핵심 문제는 아래 두 가지다.

1. 엔코더는 `float angle`만 바로 소비되고, `raw_count`, `accum_count`, wrap 상태가 외부에 드러나지 않는다.
2. ADC는 `raw`와 `angle`은 있지만, 유효성 진단과 calibration 상태가 없다.

## 3. 현재 코드 구현 상세 분석

### 3.1 encoder_reader.c

현재 구현은 TIM4 16비트 counter를 중앙값 `32768` 기준으로 해석한다.

동작 방식:

1. `EncoderReader_Init()`
- counter를 `32768`로 맞춤
- `initialized = 1`

2. `EncoderReader_GetAngleDeg()`
- `raw = __HAL_TIM_GET_COUNTER(&htim4)`
- `encoder_count = raw - 32768`
- `adjusted_count = encoder_count - encoder_offset`
- `angle = adjusted_count * 360 / 48000`

장점:
- TIM2에서 TIM4로 바뀐 후에도 최소 동작은 단순하고 안정적이다.
- 영점 offset 개념이 있어 homing과 연결 가능하다.

핵심 한계:
- 16비트 counter wrap을 누적하지 않는다.
- 유효 범위는 `-32768 ~ +32767 count`, 즉 약 `-245.76 ~ +245.75 deg` 정도다.
- 그런데 position control은 `+/-4320 deg`를 허용하므로, 장시간 다회전 기준과 맞지 않는다.

즉 현재 엔코더 계층은 "짧은 범위 테스트"에는 쓸 수 있지만 "다회전 운용 제어기" 기준으로는 부족하다.

### 3.2 adc_potentiometer.c

현재 ADC 모듈은 아래 수준이다.

- `ADC_Pot_Init()`: 기본 config를 잡고 `HAL_ADC_Start`
- `ADC_Pot_GetRaw()`: software start 후 poll
- `ADC_Pot_GetVoltage()`: raw -> voltage
- `ADC_Pot_GetAngle()`: raw -> angle 선형 환산
- `ADC_Pot_Calibrate()`: 3초 delay 후 2점 보정

장점:
- API가 단순해서 사용은 쉽다.
- raw/voltage/angle을 분리한 방향은 맞다.

핵심 한계:
- `HAL_ADC_PollForConversion(..., 100)`은 최대 100ms blocking이다.
- runtime 실시간 경로에서 그대로 쓰기엔 부담이 크다.
- disconnect, jump, stuck, saturation 같은 센서 이상 진단이 없다.
- calibration이 RAM에만 있고 재부팅 후 유지되지 않는다.
- main runtime에서 실제 init/사용이 빠져 있다.

### 3.3 homing과의 연동

`homing.c`는 ADC angle을 바로 encoder offset으로 바꾼다.

문제는 아래와 같다.

- pot angle의 물리적 의미가 조향축 각인지 모터축 각인지 명확하지 않다.
- `offset_count = pot_angle * 48000 / 360`은 encoder와 ADC 기준축이 같다는 가정을 쓴다.
- gear ratio, 설치 위치, linkage ratio가 반영돼 있지 않다.

즉 C 파트는 단순 센서 읽기 이상의 "센서 의미 정의"를 맡아야 한다.

### 3.4 상수 관리 문제

`constants.h`에는 아래 문제가 있다.

- include guard가 없다.
- `POSITION_TOLERANCE`가 `position_control.h`와 중복된다.
- `MAX_STEERING_ANGLE +/-45`와 `MAX_ANGLE_DEG +/-4320`가 따로 논다.
- `MAX_PULSE_FREQ = 1000000`인데 실제 pulse 모듈 clamp는 `100000`이다.

센서 파트에서 단위와 변환상수를 정리하지 않으면 팀 전체가 같은 문제를 반복한다.

## 4. 현재 평가

성숙도 기준:
- L1: 센서 읽기 골격
- L2: 단기 bench 사용 가능
- L3: 장시간/다회전 운용 가능
- L4: sensor fault와 cross-check까지 검증
- L5: 현업형 sensor pipeline으로 설명 가능

현재 성숙도: `L2`

평가 근거:
- 엔코더는 짧은 범위에서는 사용 가능하다.
- ADC는 API는 있지만 runtime 통합과 진단이 부족하다.
- cross-check와 신뢰도 판단이 아직 없다.

자율주행 조향모터의 현업 기준으로 보면 아래 항목이 더 필요하다.

- fail-safe 완성도:
  - 센서 freshness, implausible velocity, command-sign mismatch를 감지하지 않는다.
  - 센서 fault가 언제 safe state로 이어지는지 bounded-time 계약이 없다.
  - homing 기준축 정의가 모호해 잘못된 offset이 조용히 들어갈 수 있다.
- debug 완성도:
  - sample timestamp, raw/calibrated pair, calibration version, health state가 남지 않는다.
  - 센서 fault 발생 시 직전 raw 데이터와 비교값을 남기는 진단 버퍼가 없다.

한 줄 평가:
- "센서를 읽는 기능은 있지만, 센서를 신뢰할 수 있게 만드는 기능은 아직 부족한 상태"

## 5. C 파트 REQ 초안

- `REQ-C-001`: 엔코더는 16비트 wrap을 고려한 `accum_count`를 제공해야 한다.
- `REQ-C-002`: 엔코더 API는 최소한 `raw_count`, `delta_count`, `accum_count`, `motor_deg`를 설명 가능해야 한다.
- `REQ-C-003`: ADC API는 `raw`, `voltage`, `calibrated_angle`, `validity`를 함께 제공해야 한다.
- `REQ-C-004`: ADC는 disconnect, out-of-range, jump, stuck fault를 진단해야 한다.
- `REQ-C-005`: 엔코더와 ADC는 homing 시점과 runtime 시점에 각각 어떤 센서를 기준으로 삼는지 정책이 있어야 한다.
- `REQ-C-006`: gear ratio, deg per count, pulse per deg, pot angle range는 단일 상수 헤더에서 관리해야 한다.
- `REQ-C-007`: 센서 API는 blocking call이 1ms 제어 루프를 방해하지 않도록 설계돼야 한다.
- `REQ-C-008`: 센서 변환 결과는 코드와 문서에서 같은 단위를 써야 한다.
- `REQ-C-009`: 엔코더-ADC 차이를 이용한 cross-check 진단 로직이 있어야 한다.
- `REQ-C-010`: 센서 fault는 fault code와 복귀 조건을 함께 제공해야 한다.
- `REQ-C-011`: 모든 센서 샘플은 timestamp 또는 age 정보와 함께 제공되어 freshness를 판단할 수 있어야 한다.
- `REQ-C-012`: 엔코더는 velocity estimate를 제공해야 하며, implausible velocity/acceleration을 fault 또는 warning으로 진단해야 한다.
- `REQ-C-013`: homing에 쓰는 ADC angle이 `steering_deg`인지 `motor_deg`인지 명시하고, gear/linkage 변환식을 코드와 문서에서 동일하게 유지해야 한다.
- `REQ-C-014`: calibration 데이터는 version, range, checksum 또는 동등한 무결성 정보를 포함해 재부팅 후에도 복원 가능해야 한다.
- `REQ-C-015`: commanded direction과 encoder delta 방향이 일정 시간 이상 불일치하면 sensor/actuator fault 후보로 보고해야 한다.
- `REQ-C-016`: 센서 fault는 정의된 최대 시간 내 safe state 전이와 진단 로그를 유발해야 한다.

인수 기준:

1. 장시간 정/역회전 중 angle discontinuity가 없다.
2. ADC calibration 절차와 코드가 일치한다.
3. `raw -> calibrated` 변환을 로그로 설명할 수 있다.
4. 엔코더와 ADC 차이가 기준을 넘으면 경고 또는 fault가 발생한다.
5. 센서 sample age와 calibration version을 로그로 설명할 수 있다.
6. disconnect, stuck, implausible speed 주입 시 fault가 정해진 시간 안에 검출된다.

## 6. 어떻게 더 발전시켜야 하는가

### P0

- 엔코더 unwrap 누적을 구현한다.
- `EncoderReader_GetAngleDeg()` 외에 `GetRaw`, `GetAccumCount`, `GetMotorDeg`를 분리한다.
- ADC init, calibration, valid flag를 정식 API로 만든다.
- 상수와 단위 정의를 한 곳으로 모은다.
- 센서 sample timestamp/age와 velocity estimate를 넣는다.
- homing에 쓰는 ADC 기준축과 gear ratio 적용 위치를 명확히 한다.
- sensor fault를 position/fault manager 경로로 실제 연결한다.

### P1

- ADC 필터링과 plausibility check를 추가한다.
- `encoder vs adc` 교차검증 임계값을 넣는다.
- runtime에서 blocking 없는 측정 경로를 만든다.
- 센서 로그 CSV를 자동 저장한다.
- commanded direction 대비 encoder sign mismatch 검출을 추가한다.
- calibration 값을 flash 또는 별도 설정 저장소에 보존한다.

### P2

- velocity estimate, noise sigma, drift monitor를 추가한다.
- 센서 health score를 상위 fault manager와 연결한다.
- 센서 노화, 온도 드리프트, 장시간 재현성까지 포함한 health model로 확장한다.
- 센서 이중화 또는 degraded mode 판단 정책까지 확장한다.

## 7. 정량 증거를 어떻게 남기면 좋은가

C 파트는 "센서를 얼마나 정확하고 연속적으로 읽었는가"를 남기면 강하다.

| 지표 | 의미 | 권장 증거 |
|---|---|---|
| discontinuity count | wrap 오류 여부 | long-run log |
| drift | 시간 경과에 따른 영점 변화 | 30분 이상 정지 로그 |
| ADC noise sigma | 아날로그 품질 | raw sample CSV |
| cross-check delta | encoder-ADC 차이 | 비교 그래프 |
| sensor fault detection rate | 이상 감지 품질 | fault injection 결과 |
| sample latency | 센서 읽기 비용 | latency CSV |

권장 파일:
- `Doc/measurements/encoder_longrun_2026-03-xx_run01.csv`
- `Doc/measurements/adc_noise_2026-03-xx_run01.csv`
- `Doc/measurements/sensor_crosscheck_2026-03-xx_run01.csv`

## 8. "얼마나 향상됐는가"를 보여주는 기준

| 항목 | Baseline | Target | 개선 표시법 |
|---|---|---|---|
| encoder discontinuity | 예: 30분에 4회 | 0회 | `count 감소` |
| zero drift | 예: 1.6deg/h | 0.2deg/h 이하 | `deg/h 감소` |
| ADC noise sigma | 예: 1.8deg | 0.3deg 이하 | `sigma 감소` |
| cross-check error | 예: 6deg max | 1deg 이하 | `deg 감소` |
| fault detect coverage | 예: 2/5 case | 5/5 case | `coverage 증가` |

면접에서 좋은 표현:
- "엔코더를 읽었다"보다 "16비트 wrap을 unwrap 누적으로 바꾸고 30분 구동에서 discontinuity 0건을 확인했다"가 훨씬 강하다.

## 9. 취업에 도움이 되는 기록 전략

C 파트는 아래 자료가 좋다.

1. 센서 파이프라인 도식
- raw count/raw ADC -> calibrated angle -> validation -> consumer

2. 센서 데이터셋
- 정지 노이즈
- 저속 sweep
- 장시간 반복 회전

3. 비교 그래프
- encoder angle vs ADC angle
- raw vs filtered

4. fault injection
- ADC 분리
- saturation
- encoder wrap 구간 통과

5. 설계 의도
- 왜 raw와 calibrated를 분리했는지
- 왜 cross-check가 필요한지

가장 추천하는 포트폴리오 문장:

> TIM4 encoder와 ADC absolute sensor를 각각 raw 데이터와 공학 단위로 정규화하고, wrap 처리와 plausibility check를 통해 제어기가 믿고 사용할 수 있는 센서 파이프라인으로 발전시켰다.
