# Ethernet 기반 조향 제어 구현/분담 명세서 (4인 균등)

작성일: 2026-02-24  
대상 프로젝트: `servo_control_baqu`  
목표: 인지파트 UDP 입력 기반 조향 제어를 정확성/안전/실시간성 기준으로 운영 가능 상태까지 완성

---

## 1. 시스템 목표 (운영 기준)

1. 인지파트에서 UDP 패킷 수신 후 모드/조향 목표를 정확히 반영한다.
2. `AUTO`: PC steer 기반 목표각 추종, `MANUAL`: ASMS joy_y 기반 목표각 추종.
3. 통신 단절/비상상황에서 fail-safe(ESTOP + 하드웨어 EMG)를 보장한다.
4. 1ms 제어 루프 지터를 관리하고, 불필요 연산/로그를 줄여 실시간성을 확보한다.

---

## 2. 적용 표준/준수 기준

- 기능안전: ISO 26262 (Part 6, SW 안전 메커니즘)
- 소프트웨어 코딩: MISRA-C:2012 주요 규칙 준수
- 신뢰성 프로세스: IEC 61508의 fail-safe 사고방식 적용
- 진단/검증: 요구사항-테스트 추적성(요구사항 ID 기반)

### 필수 준수 원칙

1. 인터럽트/콜백 경로에서 블로킹 I/O 및 과도한 `printf` 금지
2. 통신 입력은 송신자/길이 검증 후만 처리
3. 모드 전이 시 1회성 동작(중복 Enable/Disable 방지)
4. 안전 동작은 소프트 정지 + 하드 EMG 동시 보장

---

## 3. 4인 균등 분담 (25%씩)

> 인원 표기는 역할명으로 정의. 실제 팀원 이름으로 치환해서 사용.

| 담당 | 역할 | 구현 범위(파일) | 핵심 구현 | 완료 기준 |
|---|---|---|---|---|
| 담당 A (통신/프로토콜) | UDP 파싱 책임 | `Core/Src/ethernet_communication.c`, `Core/Inc/ethernet_communication.h` | ASMS 5B/PC 9B 파싱 정합, sender 필터, mode/steer 업데이트, RX timeout API | 모드/타겟 반영률 100%, 잘못된 패킷 100% 무시 |
| 담당 B (제어/상태머신) | 모드 전이 + 제어 적용 | `Core/Src/main.c`, `Core/Src/position_control.c`, `Core/Inc/position_control.h` | AUTO/MANUAL/NONE/ESTOP 전이표 구현, SetTarget 경로 정립, CTRL_MODE 실상태화 | 모드 전이 테스트 케이스 통과, 제어 경로 단일화 |
| 담당 C (안전/하드웨어 인터락) | Fail-safe 책임 | `Core/Src/position_control.c`, `Core/Src/relay_control.c`, `Core/Inc/relay_control.h` | ESTOP 하드 EMG 동작, timeout/브레이크 비트 처리, 복귀 절차 정의 | 케이블 분리/브레이크/오류 주입 시 안전동작 100% |
| 담당 D (실시간/최적화/검증) | 성능 및 검증 책임 | `Core/Src/encoder_reader.c`, `Core/Src/pulse_control.c`, `Doc/*`, `Core/test/*` | 엔코더 unwrap 누적, 로그 경량화, dt/연산 최적화, 테스트 스크립트/리포트 | 제어주기 지터 기준 충족, 회귀 테스트 자동 재현 |

---

## 4. 파일별 구현 명세 (무엇을 어떻게 구현할지)

### 4.1 통신 모듈 (`ethernet_communication.*`) - 담당 A

요구사항 ID: `REQ-COMM-001` ~ `REQ-COMM-006`

1. 패킷 처리 규칙
- ASMS: `len=5`, sender `.5`만 허용
- PC: `len=9`, sender `.1`만 허용
- 예외 패킷은 즉시 drop

2. 모드/타겟 반영
- ASMS mode 유효 범위 `0~3`만 반영
- `MANUAL`에서만 joy_y -> degree 매핑
- `AUTO`에서만 PC steer 반영

3. 타임아웃 데이터 제공
- `g_last_rx_tick` 갱신
- API: `EthComm_GetLastRxTick()` 제공

4. 디버그 정책
- `ETHCOMM_DEBUG` 매크로로 RX 로그 제어
- 운영 빌드 default OFF

검증 항목:
- 정상 입력 반영, 비정상 길이/송신자 무시, mode 유지/전환 확인

### 4.2 메인 루프/상태머신 (`main.c`) - 담당 B

요구사항 ID: `REQ-MODE-001` ~ `REQ-MODE-005`

1. 모드 전이표 구현
- `NONE -> Disable(1회)`
- `AUTO/MANUAL 진입 -> Enable(1회)`
- `ESTOP 진입 -> EmergencyStop(1회)`

2. 모드별 타겟 반영
- `AUTO`: PC steer 기반 `PositionControl_SetTarget`
- `MANUAL`: ASMS joy_y 기반 `PositionControl_SetTarget`

3. timeout fail-safe
- `AUTO/MANUAL` 상태에서 `now-last_rx > timeout`이면 `ESTOP`

4. 테스트 토글 관리
- 강제 펄스 테스트 매크로는 기본 OFF 유지

검증 항목:
- 전이 중복호출 없음, mode와 target 경로 일치

### 4.3 제어/안전 (`position_control.c`) - 담당 B/C 공동

요구사항 ID: `REQ-SAFE-001` ~ `REQ-SAFE-005`

1. `CTRL_MODE` 실상태화
- `Enable/Disable/EmergencyStop/SetMode`에서 `control_mode` 갱신
- `PositionControl_GetMode()`는 실상태 반환

2. Emergency 동작
- `PulseControl_Stop()` + `Relay_Emergency()` 동시 수행
- 복귀 시 `Relay_EmergencyRelease()` 후 Enable

3. 안전 체크
- 각도/오차 제한 위반 시 즉시 ESTOP

검증 항목:
- fault 주입 시 소프트/하드 동시 정지 확인

### 4.4 엔코더/출력 최적화 (`encoder_reader.c`, `pulse_control.c`) - 담당 D

요구사항 ID: `REQ-RT-001` ~ `REQ-RT-004`

1. 엔코더 unwrap 누적
- `prev_raw` 기반 delta 계산
- wrap 구간 보정 후 `accum_count` 누적
- 각도는 누적 count로 계산

2. 펄스 출력 경량화
- 하드코딩 타이머클럭 정리
- ARR/CCR 연산 범위 보호 유지

3. 루프 경량화
- 불필요 float/printf 감축

검증 항목:
- 장시간 구동 시 각도 누적 안정성, 제어 지터 기준 통과

---

## 5. 인터페이스 명세 (운영 확정)

### 5.1 ASMS -> STM32
- 크기: 5 bytes
- 포맷: `[mode][joy_x_l][joy_x_h][joy_y_l][joy_y_h]`
- endian: little
- mode: `0 NONE, 1 AUTO, 2 MANUAL, 3 ESTOP`

### 5.2 PC -> STM32
- 크기: 9 bytes
- 포맷: `<iIB` = `steer(int32), speed(uint32), misc(uint8)`
- endian: little
- 적용 조건: 현재 mode가 `AUTO`일 때만 steer 적용

---

## 6. 상태머신(운영)

| 현재 상태 | 입력 | 다음 상태 | 동작 |
|---|---|---|---|
| NONE | ASMS mode=1 | AUTO | Enable, EMG Release |
| NONE | ASMS mode=2 | MANUAL | Enable, EMG Release |
| AUTO | ASMS mode=2 | MANUAL | target source 전환 |
| MANUAL | ASMS mode=1 | AUTO | target source 전환 |
| AUTO/MANUAL | timeout | ESTOP | EmergencyStop + EMG |
| AUTO/MANUAL | brake bit | ESTOP | EmergencyStop + EMG |
| ESTOP | ASMS mode=1/2 + 조건충족 | AUTO/MANUAL | EMG Release 후 Enable |

---

## 7. 성능 목표 (임베디드 기준)

- 제어 주기: 1ms (1000Hz)
- DIAG 주기: 100ms
- UDP RX 콜백 평균 처리시간: 200us 이하 목표
- timeout: 300ms (운영 중 튜닝 가능)
- 로그 정책: 운영 최소 로그, 디버그 빌드 상세 로그

---

## 8. 테스트/검증 계획

### 8.1 기능 테스트
1. AUTO 모드에서 PC steer(30/-30/0) -> target 반영 확인
2. MANUAL 모드에서 joy_y sweep -> target 반영 확인
3. NONE 모드에서 출력 정지 확인
4. ESTOP 모드에서 EMG 핀 LOW 확인

### 8.2 안전 테스트
1. UDP 송신 중단 -> 300ms 내 ESTOP
2. misc brake bit=1 -> 즉시 ESTOP
3. 센서값 비정상(각도/오차 임계 초과) -> ESTOP

### 8.3 회귀 테스트
- 모드 전환 반복 100회
- AUTO<->MANUAL 연속 전환 중 오동작/프리징 없음
- 장시간(30분+) 누적 동작 시 엔코더 drift 확인

---

## 9. 인수 기준 (Definition of Done)

1. 기능: AUTO/MANUAL 모두 목표조향 정상 반영
2. 안전: timeout/브레이크/오류 주입 시 fail-safe 100%
3. 성능: 1ms 루프 지터 허용 범위 내
4. 문서: 코드/패킷/상태머신 명세 일치
5. 추적성: 요구사항 ID별 테스트 결과 기록 완료

---

## 10. 작업 순서 권장 (병렬 가능)

1. 담당 A/B: 통신/상태머신 먼저 고정
2. 담당 C: 안전 경로(EMG) 검증 병행
3. 담당 D: 최적화 + 회귀 자동화
4. 통합: 시스템 테스트 -> 튜닝 -> 문서 확정

