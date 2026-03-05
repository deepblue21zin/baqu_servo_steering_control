# Autonomous Steering Sub-Controller (STM32F429ZI)

STM32F429ZI 기반 조향 서브 컨트롤러 프로젝트.
상위 인지/제어 시스템에서 UDP 패킷으로 목표 조향각을 받아 1ms 제어 루프(PID)로 모터를 구동한다.

## 1. 프로젝트 핵심

- 목적: 차량 조향 액추에이터를 실시간으로 안정 제어
- MCU/보드: STM32F429ZI / NUCLEO-F429ZI
- 제어 주기: 1kHz (1ms)
- 통신: Ethernet UDP (LwIP)
- 제어 방식: Encoder feedback + PID + Pulse/Direction 출력
- 안전 메커니즘: ESTOP, 통신 타임아웃 fail-safe, IWDG watchdog, 소프트웨어 safety check
- 성능 검증: DWT CYCCNT 기반 구간별 latency 계측 (avg/p99/max, deadline miss)

## 2. 현재 시스템 상태 (코드 기준)

### 2.1 제어/통신 구조

- 메인 루프: `while(1)` + `SysTick` 플래그 스케줄링
- 제어 루프: `PositionControl_Update()`를 1ms마다 실행
- 통신 처리: `MX_LWIP_Process()` + UDP 패킷 파싱
- 모드: `NONE / AUTO / MANUAL / ESTOP`

### 2.2 다회전 운용 정책

12:1 기어비 운용 요구를 반영해 다회전 제어 범위를 사용한다.

- 모터 각도 기준 허용 범위: `-4320° ~ +4320°`
- 추종 오차 안전 임계: `MAX_TRACKING_ERROR_DEG = 4500°`

참고: 값은 실제 기구/차량 통합 시험으로 재튜닝 대상.

### 2.3 Fail-safe/안전 동작

- 통신 타임아웃: `ETHCOMM_RX_TIMEOUT_MS` 초과 시 ESTOP 전환
- one-shot ESTOP 요청 처리: 패킷 플래그 기반 즉시 정지
- 안전 검사 실패 시 `PositionControl_EmergencyStop()`
- EmergencyStop 시 `PulseControl_Stop()` + `Relay_Emergency()` 수행
- IWDG watchdog 활성화 (약 0.5초 설정)

## 3. 하드웨어/신호 인터페이스

- Pulse 출력: TIM1_CH1
- Direction 출력: GPIO (PE10)
- Encoder 입력: TIM4 quadrature
- Servo relay: SVON/EMG (Active LOW)
- Debug UART: USART3
- Ethernet: RMII + LAN8742

## 4. 실시간/성능 검증 체계

이 프로젝트의 차별점은 “동작한다”가 아니라 “수치로 재현 가능”하게 관리하는 점이다.

### 4.1 계측 항목

- Stage: `Sense`, `Control`, `Actuate`, `Comms`
- KPI: `avg`, `p99`, `max` (cycles/us)
- 시스템 KPI: `deadline_miss_count`

### 4.2 계측 방식

- DWT CYCCNT 기반 구간 계측
- `LAT_BEGIN(stage)` / `LAT_END(stage)` 매크로 삽입
- SysTick에서 deadline miss 집계

### 4.3 자동 수집

- `LATENCY_AUTO_REPORT_SAMPLES = 2000`
- 2000샘플마다 UART로 배치 통계 출력:
  - `LATENCY_BATCH_BEGIN`
  - `LATENCY_STAGE`
  - `LATENCY_BATCH_END`

## 5. 증거자료(취업 포트폴리오) 관리 방법

### 5.1 로그 수집

Windows(레노버)에서 PuTTY 자동 로깅 사용:

- 스크립트: `docs/measurements/start_latency_log.ps1`
- 출력 파일: `docs/measurements/latency_YYYY-MM-DD_HHMMSS.csv`
- 메타 파일: `docs/measurements/latency_YYYY-MM-DD_HHMMSS_meta.md`

### 5.2 기준 문서

- 측정 명세: `docs/latency_measurement_spec.md`
- 결과 계약서: `docs/latency_contract.md`
- 코드 적용 기록: `docs/latency_code_application.md`
- 데이터 저장 가이드: `docs/latency_data_evidence.md`

## 6. 코드/문서 운영 규칙

- 측정 중 `printf` 최소화 (`LATENCY_LOG_ENABLE` 활용)
- 빌드 조건 고정 (`-O2`, clock 고정)
- 동일 입력 시퀀스로 반복 측정
- 원본 CSV는 수정하지 않고 보관
- 대표 결과는 `latency_contract.md`에 업데이트

## 7. 산업 관점 체크리스트

- 실시간성: 제어 루프 1ms deadline 정의
- 안전성: 통신 끊김/오류 시 정지 경로 확보
- 재현성: 동일 조건 반복 측정 가능
- 추적성: 날짜별 Fix log + 측정 로그/메타데이터 보관
- 유지보수성: 문서/코드 동기화

## 8. 남은 개선 과제 (실무 고도화)

- UART 로그 완전 비동기화(DMA + ring buffer)
- Safety 파라미터를 실차 시험 기반으로 재튜닝
- CSV 후처리 자동화(배치별 summary 리포트 생성)
- 오류 코드/진단 프레임 표준화

## 9. Fix 로그 누적 방식

- 일일 변경 이력: `docs/Fixes/YYYY-MM-DD.md`
- 템플릿: `docs/Fixes/_template.md`
- 자동 생성 스크립트: `docs/Fixes/new_fix_log.ps1`

## 10. 면접에서 설명할 때 (요약)

"STM32F429 기반 조향 서브컨트롤러를 1ms 주기로 운용하고, DWT 기반으로 Sense/Control/Actuate/Comms의 p99/worst-case를 자동 수집해 deadline miss까지 증거 데이터로 관리했습니다. 또한 ESTOP, 통신 timeout fail-safe, watchdog 경로를 코드와 로그로 추적 가능하게 설계했습니다."
