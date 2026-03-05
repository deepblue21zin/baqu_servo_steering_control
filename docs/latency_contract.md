# Latency Contract

## 1) 환경 고정 (Measurement Environment Lock)

- MCU: `STM32F429ZI`
- Core Clock: `180 MHz` (필요 시 실제 측정값으로 교체)
- Cache 상태: `I-Cache: ON/OFF`, `D-Cache: ON/OFF` (측정 시점의 상태를 반드시 명시)
- RTOS: `Non-FreeRTOS (while-loop + SysTick flag)` / FreeRTOS 사용 시 버전 명시
- Toolchain: `arm-none-eabi-gcc x.y.z`
- Build Type: `Release`
- Build Option: `-O2` (고정)
- Debug/Trace Print: `OFF` (측정 중 UART/ITM printf 금지)
- Auto Report: `ON` (`LATENCY_AUTO_REPORT_SAMPLES = 2000`)
- Log Collector: `Windows + PuTTY session log` (`docs/measurements/start_latency_log.ps1`)
- 측정 펌웨어 Git SHA: `<commit_hash>`
- 측정 일시(UTC+9): `<YYYY-MM-DD HH:MM>`

---

## 2) 측정 대상 구간 (Scope)

### 조향 파이프라인 (Steering)

- `Sense`
- `Control`
- `Actuate`
- `Comms`

---

## 3) KPI 정의

각 구간별로 아래를 `cycles` 및 `us` 단위로 모두 기록:

- `avg`
- `p99`
- `max`

시스템 레벨 KPI:

- `deadline_miss_count`
- `min_ever_free_heap` (FreeRTOS 사용 시, 현재 프로젝트는 `N/A`)

---

## 4) 측정 방법 (재현성 보장)

- 타이머 소스: `DWT CYCCNT` 사용 (권장)
- 계측 삽입 위치: 구간 `entry/exit`에 고정 매크로 사용
- 샘플 수: 구간당 최소 `N = 10,000` 이상
- 워밍업: 초기 `1~2초` 데이터 제외
- 인터럽트/스케줄링 정책: 측정 중 정책 고정 및 문서화
- 입력 데이터셋: 동일 입력 재사용 (`dataset_id`, `seed` 기록)
- 측정 중 금지: 로그 출력, 동적 설정 변경, 클럭 변경

예시 변환식:

- `us = cycles / (core_clock_hz / 1,000,000)`

---

## 5) 결과 표 (1페이지 핵심)

> 아래 표만 최신 상태로 유지하면 면접/리뷰에서 “재현 가능한 엔지니어링” 증거로 사용 가능.

| Domain | Stage | avg (cycles) | p99 (cycles) | max (cycles) | avg (us) | p99 (us) | max (us) | deadline_miss_count | min_ever_free_heap |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Steering | Sense | - | - | - | - | - | - | - | - |
| Steering | Control | - | - | - | - | - | - | - | - |
| Steering | Actuate | - | - | - | - | - | - | - | - |
| Steering | Comms | - | - | - | - | - | - | - | - |

---

## 6) Pass/Fail 기준 (권장)

- 각 Stage `p99`가 해당 Stage 예산 이내
- End-to-End `max`가 프레임 데드라인 이내
- `deadline_miss_count = 0` (목표)
- `min_ever_free_heap`가 안전 마진 이상

---

## 7) 변경 이력

- `<YYYY-MM-DD>`: 초기 템플릿 생성
- `<YYYY-MM-DD>`: 측정 조건/파이프라인/예산 업데이트
