# Latency Evidence Data Logging Guide (Windows/Lenovo 기준)

목표: 펌웨어가 2000샘플마다 출력하는 latency CSV를 Windows 노트북에서 계속 저장해 취업용 증거자료로 남긴다.

## 1) 펌웨어 출력 포맷

펌웨어는 아래 라인을 반복 출력한다.

- `LATENCY_BATCH_BEGIN,...`
- `LATENCY_STAGE,...` (Sense/Control/Actuate/Comms 각 1줄)
- `LATENCY_BATCH_END,...`

즉 1배치(2000샘플)당 6줄이 생성된다.

## 2) 권장 저장 위치 (Windows)

프로젝트 폴더 아래:

- `docs\measurements\`

파일명 규칙 예시:

- `latency_2026-03-05_run01.csv`
- `latency_2026-03-05_run02.csv`

## 3) Windows에서 시리얼 로그 저장 (PuTTY + 스크립트 권장)

1. STM32 보드를 USB로 연결
2. 장치관리자에서 COM 포트 확인 (예: `COM5`)
3. `docs\\measurements\\start_latency_log.ps1`를 열어 `$Port`를 실제 포트로 수정
4. `start_latency_log.ps1` 더블클릭 실행
5. PuTTY가 자동 실행되며 CSV/메타 파일 자동 생성
6. 측정 종료 시 PuTTY 창 닫기

주의:
- 로그 파일은 append가 아닌 새 파일로 저장 (run 번호 증가)
- 측정 도중 터미널에서 수동 입력/명령 전송 금지

## 4) 대안 도구 (원하면)

- Tera Term: `File -> Log...`
- RealTerm: Capture 탭에서 파일 저장

핵심은 동일:
- COM 포트 115200 연결
- 전체 시리얼 출력을 파일로 저장

## 5) 증거자료 품질을 높이는 메타데이터

각 CSV와 함께 아래를 같이 기록:

- Git SHA
- 빌드 옵션 (`-O2`)
- `LATENCY_LOG_ENABLE` 값
- `LATENCY_AUTO_REPORT_SAMPLES` 값 (현재 2000)
- `SystemCoreClock`
- 측정 시작/종료 시각
- 입력 시퀀스 ID
- 보드/PC 정보 (예: NUCLEO-F429ZI + Lenovo 모델명)

권장: 같은 이름의 메타 파일 저장

- `latency_2026-03-05_run01.csv`
- `latency_2026-03-05_run01_meta.md`

## 6) 운영 규칙 (재현성)

- 같은 입력 시퀀스로 반복 측정
- 같은 빌드(`-O2`, 동일 SHA)만 비교
- 원본 CSV는 수정하지 않고 보관
- 분석 결과는 별도 문서에 작성
- `docs/latency_contract.md`에는 최신 대표값(평균/p99/max)만 반영

## 7) Mac에서 잠깐 수정할 때

개발/문서 수정은 Mac에서 해도 되고, 실측 로그 수집은 Windows에서 수행한다.
즉 역할 분리:

- Mac: 코드 수정/커밋
- Lenovo(Windows): 실측 실행/CSV 증거 저장

## 8) 면접에서 말하는 한 줄

"2000샘플 단위로 자동 출력되는 p99/worst-case를 Windows 실측 환경에서 CSV로 누적 저장했고, Git SHA와 측정 조건 메타데이터를 함께 관리해 재현성을 확보했습니다."
