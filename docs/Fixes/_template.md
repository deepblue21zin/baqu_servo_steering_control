# Fix Log - {{DATE}}

## 1) 요약

- 목적:
- 결과:
- 영향 범위:

## 2) 변경 파일 목록

- `Core/...`
- `docs/...`

## 3) 코드 수정 상세 (필수)

아래 형식으로 파일별로 반드시 기록:

### 3.1 [파일 경로]

변경 의도:
- 

핵심 변경 코드:

```c
// Before

// After
```

검증 포인트:
- 

### 3.2 [파일 경로]

변경 의도:
- 

핵심 변경 코드:

```c
// Before

// After
```

검증 포인트:
- 

## 4) 문서 수정 상세

- `docs/...`:

## 5) 실행/검증

실행한 항목:
1. 
2. 

미실행 항목(있으면 이유 포함):
- 

## 6) 리스크/후속 작업

- 남은 리스크:
- 다음 작업:

## 7) 참고 명령어 (복붙용)

```bash
# 오늘 파일 생성
cp docs/Fixes/_template.md docs/Fixes/$(date +%F).md

# 변경 파일 확인
git status --short

# 코드 diff 확인
git diff -- Core/Src/main.c Core/Src/position_control.c
```
