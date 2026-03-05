# Docs Cleanup Candidates

현재 코드/운영 기준으로 점검한 삭제 후보 목록입니다.
삭제 전에는 `git diff`로 필요한 내용이 없는지 마지막 확인을 권장합니다.

## 1) 삭제 추천 (내용 없음 / 실사용 없음)

- `docs/CUBE_SETUP.md`
  - 현재 파일 크기 0 bytes
- `docs/PIN_CONFIGURATION.md`
  - 현재 파일 크기 0 bytes
- `docs/자동저장 형식 및 방법.md`
  - 현재 파일 크기 0 bytes

## 2) 통합/아카이브 후보 (중복 성격)

- `docs/수정내역_20260119.md`
- `docs/change_code.md`
- `docs/problem.md`

이 3개는 이력 성격이 겹칩니다. 아래 중 하나 권장:

1. 하나(`change_code.md`)만 남기고 나머지 archive 폴더로 이동
2. 모두 유지하되 읽기 우선순위를 문서에 명시

## 3) 현재 유지 권장 (최신 운영 문서)

- `docs/latency_measurement_spec.md`
- `docs/latency_contract.md`
- `docs/latency_code_application.md`
- `docs/latency_data_evidence.md`
- `docs/measurements/start_latency_log.ps1`

## 4) 정리 후 권장 구조

- `docs/latency_*` : 현재 측정/운영 문서
- `docs/measurements/` : 실측 CSV/메타/실행 스크립트
- `docs/archive/` : 과거 이력 문서
