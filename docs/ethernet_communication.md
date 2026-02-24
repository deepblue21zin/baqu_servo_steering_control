# 이더넷 통신 구조 및 동작 원리

> 작성일: 2026-02-24
> 사용 스택: LwIP 2.1.2 (No RTOS, 폴링 방식)
> 프로토콜: UDP (비연결, 브로드캐스트)

---

## 목차

1. [전체 시스템 구조](#전체-시스템-구조)
2. [왜 UDP인가?](#왜-udp인가)
3. [네트워크 주소 설정](#네트워크-주소-설정)
4. [LwIP 스택이란?](#lwip-스택이란)
5. [데이터 수신 흐름 (상세)](#데이터-수신-흐름-상세)
6. [패킷 구조 및 필터링](#패킷-구조-및-필터링)
7. [위치 제어와 연동](#위치-제어와-연동)
8. [관련 파일 목록](#관련-파일-목록)
9. [운영 체크 사항](#운영-체크-사항)

---

## 2026-02-24 최신 반영 사항

1. 수신자/길이 필터 정책 고정
- ASMS: `len=5` && sender `.5`
- PC: `len=9` && sender `.1`

2. 모드별 반영 정책
- ASMS는 mode를 갱신하고, `MANUAL`일 때만 joy_y를 조향각으로 반영
- PC는 `AUTO`일 때만 steer를 조향각으로 반영

3. 통신 타임아웃 안전정지
- `ETHCOMM_RX_TIMEOUT_MS`(현재 300ms) 초과 시 main 루프에서 `ESTOP` 강제

4. 내부 데이터 구조 정규화
- 외부 패킷(ASMS 5B / PC 9B)을 내부 `AutoDrive_Packet_t`(`steering_angle`, `speed`, `misc`)로 통합 저장

5. 디버그 로그 정책
- `ETHCOMM_DEBUG` 매크로(기본 0)로 RX 상세 로그 On/Off
- 운영 빌드는 상세 로그 비활성 권장

---

## 전체 시스템 구조

```
┌─────────────────────────────────────────────────────────────┐
│  자율주행 시스템                                              │
│                                                             │
│  [인지 모듈 PC]          [이더넷 허브/스위치]               │
│   10.177.21.1    ─────→  │                                  │
│   UDP 브로드캐스트        ├──────→ [조향 STM32] (설정 IP)      │
│   포트 5000               ├──────→ [구동 모듈]                │
│                           └──────→ [기타 파트]                │
└─────────────────────────────────────────────────────────────┘

브로드캐스트: 인지 모듈이 하나의 패킷을 허브로 보내면
              허브가 연결된 모든 장치에 동시에 전달
```

---

## 왜 UDP인가?

### TCP와 UDP 비교

```
TCP (Transmission Control Protocol)
  ┌────────────────────────────────────────────┐
  │ 전송자          수신자                       │
  │  패킷 ──→                                  │
  │       ←── ACK (확인 응답 대기)             │
  │  재전송 (손실 시)                           │
  └────────────────────────────────────────────┘
  장점: 데이터 손실 없음, 순서 보장
  단점: 지연 발생, 1:1만 가능 (브로드캐스트 불가)

UDP (User Datagram Protocol)
  ┌────────────────────────────────────────────┐
  │ 전송자          수신자들                    │
  │  패킷 ──→ (확인 응답 없음)                 │
  │  다음 패킷 ──→                             │
  └────────────────────────────────────────────┘
  장점: 빠름, 브로드캐스트 가능, 지연 최소
  단점: 패킷 손실 가능 (but 다음 패킷으로 대체)
```

### 자율주행에서 UDP가 적합한 이유

- 인지 모듈이 **10~100Hz**로 지속적으로 새 데이터 전송
- 패킷 하나 손실돼도 **다음 패킷(10~100ms 후)으로 자동 대체**
- 여러 모듈이 **동시에** 같은 데이터를 받아야 함 (브로드캐스트)
- 실시간 제어에서 **수ms 지연도 문제** → TCP의 ACK 대기 불허

---

## 네트워크 주소 설정

```
장치           IP 주소         역할
─────────────────────────────────────────────
인지 모듈 PC   10.177.21.1    UDP 패킷 송신자
조향 STM32     (설정 IP)      UDP 패킷 수신자 (우리 보드)
구동 모듈      10.177.21.x    UDP 패킷 수신자
서브넷 마스크  255.255.255.0  10.177.21.0~255 범위를 같은 네트워크로 인식
게이트웨이     10.177.21.1    외부 네트워크 출구 (로컬만 사용하면 의미 없음)
```

**설정 파일:** CubeMX `.ioc` (LwIP 설정 항목)

```c
// 예시 (운영망 기준)
IP_ADDRESS[0] = 10;   IP_ADDRESS[1] = 177;
IP_ADDRESS[2] = 21;   IP_ADDRESS[3] = 3;     // STM32 = 10.177.21.3

NETMASK_ADDRESS = { 255, 255, 255, 0 };
GATEWAY_ADDRESS = { 10, 177, 21, 1 };
```

---

## LwIP 스택이란?

LwIP(Lightweight IP)는 **마이크로컨트롤러용 TCP/IP 프로토콜 스택**입니다.
일반 PC의 네트워크 기능을 수백KB 메모리의 MCU에서 동작하도록 축소한 라이브러리입니다.

### 계층 구조 (OSI 모델 기준)

```
애플리케이션 층  │  udp_recv_cb()  ← 우리가 작성한 코드
────────────────┤
전송 층         │  UDP/TCP 처리   ← LwIP가 처리
────────────────┤
네트워크 층     │  IP, ARP 처리   ← LwIP가 처리
────────────────┤
데이터링크 층   │  이더넷 프레임  ← ethernetif.c (HAL 연동)
────────────────┤
물리 층         │  LAN8742 PHY 칩 ← STM32 하드웨어
```

### 이 프로젝트의 LwIP 설정 (`LWIP/Target/lwipopts.h`)

| 설정 | 값 | 의미 |
|------|----|----|
| `NO_SYS` | 1 | RTOS 없이 폴링 방식으로 동작 |
| `LWIP_TCP` | 0 | TCP 비활성화 (UDP만 사용) |
| `LWIP_SOCKET` | 0 | BSD 소켓 API 비활성화 |
| `LWIP_NETCONN` | 0 | Netconn API 비활성화 |
| `CHECKSUM_BY_HARDWARE` | 1 | 체크섬 계산을 하드웨어가 처리 |

> `NO_SYS=1`이므로 반드시 메인 루프에서 `MX_LWIP_Process()`를 주기적으로 호출해야 합니다.

---

## 데이터 수신 흐름 (상세)

```
① 인지 PC가 UDP 패킷 전송
   → 이더넷 케이블 → 허브 → STM32 RJ45 포트

② STM32 물리 층 (LAN8742 PHY 칩)
   → 전기 신호를 디지털 데이터로 변환
   → DMA로 STM32 메모리에 복사

③ ETH_IRQHandler() [stm32f4xx_it.c]
   → 이더넷 인터럽트 발생
   → HAL_ETH_IRQHandler(&heth) 호출
   → LwIP 내부 수신 버퍼에 패킷 저장 (플래그 설정)

④ MX_LWIP_Process() [main.c while(1) 루프]
   → ethernetif_input(&gnetif) 호출
   → LwIP가 이더넷 버퍼에서 패킷을 꺼내 처리
   → IP 헤더 파싱 → UDP 헤더 파싱
   → 목적지 포트 = 5000 확인

⑤ udp_recv_cb() 자동 호출 [ethernet_communication.c]
   → 등록된 콜백 함수 호출
   → 패킷 내용 분석 및 필터링

⑥ PositionControl_SetTarget() [position_control.c]
   → 조향각을 PID 제어 목표로 설정
```

### 핵심: 폴링 방식

```c
// main.c while(1) 루프
while (1) {
    MX_LWIP_Process();  // ← 이 함수가 없으면 패킷을 절대 수신하지 못함!
                        //   RTOS가 없으므로 사용자가 직접 주기적으로 호출해야 함

    if (EthComm_HasNewData()) {
        AutoDrive_Packet_t pkt = EthComm_GetLatestData();
        PositionControl_SetTarget(pkt.steering_angle);
    }

    if (interrupt_flag) {
        PositionControl_Update();  // 1ms 제어 루프
    }
}
```

---

## 패킷 구조 및 필터링

### 현재 적용된 패킷 구조 (모드 기반)

#### 1) ASMS → Steer (5 bytes)

```c
byte[0] = mode      // 0 NONE, 1 AUTO, 2 MANUAL, 3 ESTOP
byte[1] = joy_x L
byte[2] = joy_x H
byte[3] = joy_y L
byte[4] = joy_y H
```

#### 2) PC → Steer (9 bytes)

```c
byte[0..3] = steer (int32, little-endian)
byte[4..7] = speed (uint32, little-endian)
byte[8]    = misc  (bit7=brake)
```

### 엔디안/길이 규칙

- ASMS: 5 bytes 고정
- PC: 9 bytes 고정 (`<iIB`, little-endian)
- 현재 코드는 구조체 고정 크기 대신 **패킷 길이(5/9) + 발신자 IP**로 파싱합니다.

### 필터링 로직

브로드캐스트이므로 허브에 연결된 모든 장치가 패킷을 수신합니다.
각 장치가 **자신에게 필요한 데이터만 걸러내야** 합니다.

```c
static void udp_recv_cb(...) {
    uint8_t buffer[16] = {0};
    uint16_t len = p->tot_len;
    pbuf_copy_partial(p, buffer, len, 0);
    pbuf_free(p);  // LwIP 메모리 즉시 해제 (필수! 안 하면 메모리 누수)

    uint8_t sender = ip4_addr4(ip_2_ip4(addr));

    if (len == 5 && sender == 5) {
        // ASMS mode + joystick
    } else if (len == 9 && sender == 1) {
        // PC steer/speed/misc
    } else {
        // unknown packet ignore
    }
}
```

### pbuf란?

LwIP의 내부 메모리 구조입니다. 수신된 패킷은 LwIP가 관리하는 `pbuf` 구조체에 저장됩니다.

```
pbuf (Packet Buffer)
  ┌─────────────────────────────────┐
  │ tot_len: 전체 데이터 크기       │
  │ len:     이 조각의 크기         │
  │ payload: 실제 데이터 포인터 ──→ [AA][01][각도...]
  │ next:    다음 pbuf 포인터 ──→ NULL (단일) 또는 다음 조각
  └─────────────────────────────────┘

큰 패킷은 여러 pbuf가 체인(연결 리스트)을 이룰 수 있어서
pbuf_copy_partial()로 안전하게 복사합니다.
```

---

## 위치 제어와 연동

```
UDP 수신 (비동기, 언제 올지 모름)
        ↓
g_latest_pkt = pkt
g_new_data = true
        ↓
main.c while(1) 루프에서 감지
        ↓
PositionControl_SetTarget(pkt.steering_angle)
        ↓
SysTick 1ms 인터럽트 → interrupt_flag = 1
        ↓
PositionControl_Update()
  └── error = target - current
  └── PID 계산
  └── PulseControl_SetFrequency(output)
        ↓
서보모터 회전
```

### 타이밍 고려사항

```
인지파트 전송 주기: 예) 10Hz = 100ms마다 새 각도 전송
STM32 제어 주기:   1ms (1000Hz)

즉, 새 목표각이 오면 그 다음 1ms 루프에서 바로 적용.
목표각이 안 오는 동안에는 마지막 목표각을 유지하며 서보 제어.
```

---

## 관련 파일 목록

| 파일 | 역할 |
|------|------|
| `LWIP/App/lwip.c` | IP 주소 설정, LwIP 초기화, `MX_LWIP_Process()` |
| `LWIP/Target/lwipopts.h` | LwIP 옵션 설정 (UDP 전용, No RTOS) |
| `LWIP/Target/ethernetif.c` | 물리 계층 연동 (HAL ETH ↔ LwIP) |
| `Core/Inc/ethernet_communication.h` | 패킷 구조체, 포트 번호, API 선언 |
| `Core/Src/ethernet_communication.c` | UDP 수신 콜백, 필터링, 데이터 저장 |
| `Core/Src/stm32f4xx_it.c` | `ETH_IRQHandler()` (이더넷 인터럽트) |
| `Core/Src/main.c` | `MX_LWIP_Process()` 폴링, UDP→위치제어 연동 |

---

## 운영 체크 사항

아래 항목은 운영 시 반드시 일치시켜야 합니다.

### 1. UDP 포트 번호
```c
// ethernet_communication.h
#define AUTODRIVE_UDP_PORT    5000
```

### 2. 패킷 구조
인지파트와 아래 항목을 일치시켜야 합니다.
- ASMS 패킷 5 bytes (mode + joystick)
- PC 패킷 9 bytes (`<iIB`)
- Little-endian
- sender IP 기반 구분 (PC=.1, ASMS=.5)
- 조향 입력 범위(clamp): 현재 코드 기준 `-360 ~ +360 deg`

### 3. 전송 방식
- 브로드캐스트 (255.255.255.255) → 허브(더미 스위치)에 적합
- 유니캐스트 (STM32 설정 IP) → STM32에만 전송
- 멀티캐스트 → 특정 그룹에만 전송
