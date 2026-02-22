# Autonomous Steering Servo Control System

> STM32F429ZI 기반 자율주행 차량 조향 서보모터 위치 제어 시스템

---

## 1. Project Overview

| 항목 | 내용 |
|------|------|
| **프로젝트명** | servo_control_baqu |
| **목적** | 자율주행 차량의 조향 장치를 정밀 제어하는 임베디드 하위 제어기 개발 |
| **MCU** | STM32F429ZIT6 (ARM Cortex-M4, 180 MHz, FPU) |
| **보드** | NUCLEO-F429ZI |
| **서보 드라이버** | XDL-L7SA004BAA |
| **서보 모터** | XML-FBL04AMK1 (12,000 PPR Encoder) |
| **제어 방식** | PID Closed-Loop Position Control (1ms 주기) |
| **통신** | Ethernet UDP (LwIP, 192.168.1.100) / UART (디버그: USART3) |
| **IDE** | STM32CubeIDE 1.19.0 |
| **안전 기준** | ISO 26262 / MISRA-C 참조 설계 |

### 시스템 개요

상위 제어기(Jetson, PC 등)로부터 목표 조향 각도를 Ethernet UDP로 수신받아, PID 제어 알고리즘을 통해 서보모터를 정확한 위치로 구동하는 하위 제어기(Sub-Controller)입니다.

```
┌──────────────────────────┐
│   Upper Controller       │   경로 계획 / 장애물 회피 / 조향 각도 계산
│   (Jetson, PC, etc.)     │
└──────────┬───────────────┘
           │  Ethernet UDP (192.168.1.100:5000)
           ▼
┌──────────────────────────┐
│   STM32F429ZI            │   PID Position Control (1ms cycle)
│   servo_control_baqu     │   Encoder Feedback → Error → PID → Pulse Output
│                          │   IWDG Watchdog (~32s timeout)
└──────────┬───────────────┘
           │  Pulse + Direction (PE9/PE10 via SN75176 RS-485)
           ▼
┌──────────────────────────┐
│   L7 Servo Driver        │   서보 드라이버 → 서보 모터 → 조향 기구
│   + Servo Motor          │   Operating Range: ±45°
│   + Encoder (48000 CPR)  │   Position Feedback: TIM4 Quadrature
└──────────────────────────┘
```

---

## 2. Hardware Specification

### 2.1 MCU

| Parameter | Value |
|-----------|-------|
| MCU | STM32F429ZIT6 |
| Core | ARM Cortex-M4 + FPU |
| Clock | 180 MHz (HSE 8 MHz + PLL) |
| Flash | 2 MB |
| SRAM | 256 KB |
| Ethernet | 10/100 Mbps (RMII, LAN8742 PHY) |

### 2.2 Servo System

| Component | Model | Specification |
|-----------|-------|---------------|
| Servo Driver | XDL-L7SA004BAA | AC Servo, Pulse + Direction 입력 |
| Servo Motor | XML-FBL04AMK1 | AC Servo Motor |
| Encoder | 내장 (Motor) | 12,000 PPR (Quadrature ×4 = 48,000 counts/rev) |
| Resolution | — | 0.0075°/count (360° / 48,000) |
| Operating Range | — | ±45° |
| 전자기어비 | — | 1 pulse = 0.003° |

### 2.3 Pin Map

| Function | Pin | Peripheral | Description |
|----------|-----|-----------|-------------|
| Pulse Output | PE9 | TIM1_CH1 (PWM) | SN75176 → L7 PF+ (Pin 9) |
| Direction Output | PE10 | GPIO_Output | SN75176 → L7 PR+ (Pin 11) |
| Encoder A | PD12 | TIM4_CH1 | Quadrature Encoder Input |
| Encoder B | PD13 | TIM4_CH2 | Quadrature Encoder Input |
| Servo ON Relay | PD14 | GPIO_Output | Active LOW (SVON) |
| Emergency Relay | PD15 | GPIO_Output | Active LOW (EMG, B접점) |
| ADC Potentiometer | PA4 | ADC1_CH4 | Homing 절대위치 참조용 |
| Debug UART TX | PD8 | USART3_TX | ST-Link Virtual COM (115200) |
| Debug UART RX | PD9 | USART3_RX | ST-Link Virtual COM |
| Host UART TX | PA9 | USART1_TX | 상위 제어기 통신 (예비) |
| Host UART RX | PA10 | USART1_RX | 상위 제어기 통신 (예비) |
| ETH RMII REF_CLK | PA1 | ETH | 50 MHz Reference Clock |
| ETH RMII MDIO | PA2 | ETH | Management Data I/O |
| ETH RMII CRS_DV | PA7 | ETH | Carrier Sense / Data Valid |
| ETH RMII MDC | PC1 | ETH | Management Data Clock |
| ETH RMII RXD0 | PC4 | ETH | Receive Data 0 |
| ETH RMII RXD1 | PC5 | ETH | Receive Data 1 |
| ETH RMII TX_EN | PG11 | ETH | Transmit Enable |
| ETH RMII TXD0 | PG13 | ETH | Transmit Data 0 |
| ETH RMII TXD1 | PB13 | ETH | Transmit Data 1 |

---

## 3. System Architecture

### 3.1 Software Layer Diagram

```
┌─────────────────────────────────────────────────────┐
│                  Application Layer                   │
│                     main.c                           │
│         (System Init, Main Loop, 1ms Scheduler)     │
├─────────────────────────────────────────────────────┤
│                  Control Layer                       │
│              position_control.c                      │
│     (PID Algorithm, Safety Check, State Management) │
├──────────┬──────────┬──────────┬────────────────────┤
│  Pulse   │ Encoder  │  Relay   │     Homing         │
│  Control │ Reader   │  Control │     Module          │
│  (TIM1)  │ (TIM4)   │  (GPIO)  │  (ADC + Encoder)   │
├──────────┴──────────┴──────────┴────────────────────┤
│              Communication Layer                     │
│          ethernet_communication.c                    │
│      (LwIP UDP, Command Parse, Status Report)       │
├─────────────────────────────────────────────────────┤
│               STM32 HAL Driver Layer                 │
│  (TIM, GPIO, UART, ADC, ETH, IWDG, RCC, NVIC)     │
├─────────────────────────────────────────────────────┤
│           Hardware / BSP Layer                       │
│    STM32F429ZI → SN75176 → L7 Driver → Servo Motor │
│    LAN8742 PHY → RJ45 → Upper Controller           │
└─────────────────────────────────────────────────────┘
```

### 3.2 Control Loop Flow

```
SysTick ISR (1ms)          Main Loop
     │                        │
     │  interrupt_flag = 1    │
     │ ──────────────────→    │
     │                        ▼
     │                  ┌─────────────┐
     │                  │ flag == 1?  │──No──→ IWDG Refresh (idle)
     │                  └──────┬──────┘
     │                         │ Yes
     │                         ▼
     │                  ┌─────────────────────────────┐
     │                  │ PositionControl_Update()     │
     │                  │  1. Read Encoder (TIM4)      │
     │                  │  2. Safety Check (±50°, 60°) │
     │                  │  3. Error = Target - Current │
     │                  │  4. PID Calculate (P+I+D)    │
     │                  │  5. SetFrequency(output)     │
     │                  │  6. Stability Check (100ms)  │
     │                  └──────────────┬──────────────┘
     │                                 │
     │                                 ▼
     │                  ┌─────────────────────────────┐
     │                  │ Debug Print (every 100ms)    │
     │                  │ IWDG Refresh                 │
     │                  └─────────────────────────────┘
```

### 3.3 Safety Architecture

```
                    ┌──────────────┐
                    │ CheckSafety()│
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
        Angle Limit   Error Limit   (Future)
        ±50° hard     60° max err   Comm Timeout
              │            │            │
              └────────────┼────────────┘
                           │ fail
                           ▼
                  ┌──────────────────┐
                  │ EmergencyStop()  │
                  │  SW: PulseStop   │
                  │  SW: PID Reset   │
                  │  HW: Relay EMG   │ ← BUG-010에서 추가됨
                  └──────────────────┘
```

---

## 4. Module Description

### 4.1 position_control (PID Controller)

PID closed-loop 위치 제어의 핵심 모듈.

| Parameter | Value | Description |
|-----------|-------|-------------|
| Kp | 500.0 | 비례 게인 |
| Ki | 5.0 | 적분 게인 |
| Kd | 20.0 | 미분 게인 |
| Integral Limit | 1000.0 | Anti-Windup 한계 |
| Output Limit | 10,000 Hz | 최대 펄스 주파수 |
| Control Period | 1 ms | SysTick 기반 |
| Position Tolerance | 0.5° | 안정화 판정 오차 범위 |
| Stability Time | 100 ms | 안정화 판정 유지 시간 |
| Safety: Angle Limit | ±50° | 하드 리밋 (±45° + 5° 마진) |
| Safety: Error Limit | 60° | 오차 초과 시 비상정지 |

**주요 기능:**
- `PositionControl_Init()` — 초기화 (PID 상태 리셋)
- `PositionControl_Update()` — 1ms 주기 메인 제어 루프
- `PositionControl_SetTarget(float deg)` — 목표 각도 설정 (±45°, 임계영역 보호)
- `PositionControl_Enable()` / `Disable()` — 제어 활성화/비활성화
- `PositionControl_EmergencyStop()` — 비상정지 (SW + HW 릴레이)
- `PositionControl_CheckSafety()` — 각도 범위 + 오차 안전 검사

### 4.2 pulse_control (PWM Pulse Generator)

TIM1 PWM을 이용한 정밀 펄스 생성 모듈.

| Function | Description |
|----------|-------------|
| `PulseControl_SetFrequency(int32_t freq_hz)` | 부호 기반 방향(PE10) + 주파수 설정 (PID 출력용) |
| `PulseControl_SendSteps(uint32_t steps, MotorDirection dir)` | 지정 개수 펄스 전송 (Open-loop용) |
| `PulseControl_Stop()` | PWM 즉시 정지 (IT + 일반 모두) |
| `PulseControl_IsBusy()` | 스텝 모드 동작 중 상태 확인 |

**SetFrequency 동작:**
1. `freq_hz` 부호로 방향 핀(PE10) 설정 (양수: CW, 음수: CCW)
2. 주파수 범위 클램프 (10 Hz ~ 100 kHz)
3. ARR 계산: `ARR = TimerClock / ((PSC+1) × freq) - 1`
4. ARR 범위 보호 (1 ~ 0xFFFF)
5. 듀티비 50% 설정 (CCR = ARR/2)
6. PWM Start

### 4.3 encoder_reader (Position Feedback)

TIM4 Quadrature Encoder 모드를 이용한 위치 피드백 모듈.

| Parameter | Value |
|-----------|-------|
| Timer | TIM4 (PD12/PD13, 16비트) |
| PPR | 12,000 |
| Quadrature | ×4 (48,000 counts/rev) |
| Resolution | 0.0075°/count |
| Counter 방식 | Center-offset (32768 기준) |
| 유효 범위 | ±32,767 카운트 (±245.75°) |

**16비트 타이머 처리:**
```
TIM4 Counter: 0 ─────── 32768 ─────── 65535
              │          │ (중심)       │
              │          │              │
실제 카운트:  -32768     0            +32767
```

### 4.4 relay_control (Power Management)

서보 드라이버 전원 및 비상정지 릴레이 제어. 모두 **Active LOW** 방식.

| Function | GPIO | Logic | Description |
|----------|------|-------|-------------|
| `Relay_Init()` | PD14, PD15 | SET, SET | 서보 OFF, EMG 해제 (안전 초기 상태) |
| `Relay_ServoOn()` | PD14 | RESET | 서보 드라이버 전원 ON |
| `Relay_ServoOff()` | PD14 | SET | 서보 드라이버 전원 OFF |
| `Relay_Emergency()` | PD15 | RESET | 비상정지 릴레이 작동 |
| `Relay_EmergencyRelease()` | PD15 | SET | 비상정지 해제 |

### 4.5 homing (Zero Point Calibration)

ADC 포텐셔미터 기반 절대 위치 참조로 엔코더 원점을 설정하는 모듈.

**Homing 시퀀스:**
1. ADC 포텐셔미터에서 현재 절대 각도 읽기
2. 엔코더 카운터 리셋 (TIM4 → 32768)
3. 절대 각도를 엔코더 오프셋으로 변환 후 설정

### 4.6 ethernet_communication (Network Interface)

LwIP UDP 기반 상위 제어기 통신 모듈 (현재 골격 구현 상태).

| 설정 | 값 |
|------|-----|
| IP Address | 192.168.1.100 (Static) |
| Protocol | UDP |
| PHY | LAN8742 (RMII) |
| LwIP Version | 2.1.2 |

**지원 명령:**
- `SERVO ON` / `SERVO OFF` — 서보 전원 제어
- `SET TARGET <angle>` — 목표 각도 설정
- `STOP` — 제어 정지
- `GET STATUS` — 현재 상태 조회

### 4.7 IWDG (Independent Watchdog)

| Parameter | Value |
|-----------|-------|
| Prescaler | 256 |
| Reload | 4095 |
| Timeout | ~32초 |
| Refresh 위치 | main while(1) 루프 |

---

## 5. Project Structure

```
servo_control_baqu/
├── Core/
│   ├── Inc/                              # Header files
│   │   ├── main.h                        # CubeMX pin definitions (DIR, SVON, EMG, RMII)
│   │   ├── position_control.h            # PID controller API & data structures
│   │   ├── pulse_control.h               # Pulse generator API
│   │   ├── encoder_reader.h              # Encoder reader API
│   │   ├── relay_control.h               # Relay control API
│   │   ├── homing.h                      # Homing module API
│   │   ├── adc_potentiometer.h           # ADC potentiometer reader
│   │   ├── ethernet_communication.h      # Ethernet comm API & command types
│   │   ├── constants.h                   # System constants & parameters
│   │   ├── adc.h / tim.h / usart.h       # Peripheral config (CubeMX)
│   │   ├── gpio.h / iwdg.h / lwip.h      # Peripheral config (CubeMX)
│   │   ├── stm32f4xx_hal_conf.h          # HAL configuration
│   │   └── stm32f4xx_it.h               # Interrupt handler declarations
│   ├── Src/                              # Source files
│   │   ├── main.c                        # System init + PID control loop
│   │   ├── position_control.c            # PID controller (26 functions)
│   │   ├── pulse_control.c              # PWM pulse generation (TIM1)
│   │   ├── encoder_reader.c             # Encoder reading (TIM4, 16-bit center-offset)
│   │   ├── relay_control.c              # Relay GPIO control (Active LOW)
│   │   ├── homing.c                     # Zero-point calibration (ADC + Encoder)
│   │   ├── adc_potentiometer.c          # ADC reading (PA4, 12-bit)
│   │   ├── ethernet_communication.c     # Ethernet comm (stub, LwIP UDP 예정)
│   │   ├── stm32f4xx_it.c              # ISR: SysTick flag, TIM1_CC, ETH_IRQ
│   │   ├── adc.c / tim.c / usart.c      # Peripheral init (CubeMX)
│   │   ├── gpio.c / iwdg.c             # Peripheral init (CubeMX)
│   │   └── system_stm32f4xx.c          # System clock config
│   └── Startup/
│       └── startup_stm32f429zitx.s      # Startup assembly (vector table)
├── Drivers/
│   ├── CMSIS/                           # ARM CMSIS headers
│   └── STM32F4xx_HAL_Driver/            # ST HAL library
├── LWIP/                                # LwIP TCP/IP stack (CubeMX generated)
│   ├── App/                             # lwipconf.h, lwip.c
│   └── Target/                          # ethernetif.c (LAN8742 PHY)
├── Middlewares/
│   └── Third_Party/LwIP/               # LwIP core library
├── Doc/                                 # Documentation
│   ├── problem.md                       # Bug reports & resolutions (BUG-001~014)
│   ├── change_code.md                   # Code change history (변경 #1~#14)
│   ├── team_request.md                  # Team task assignments
│   ├── position_control_study.md        # PID control technical study
│   ├── 향후계획.md                       # Development roadmap
│   ├── 최종 구조.md                      # Architecture design
│   └── 수정내역_20260119.md              # Modification log
├── servo_control_baqu.ioc               # CubeMX project file
├── STM32F429ZITX_FLASH.ld              # Flash linker script
├── STM32F429ZITX_RAM.ld               # RAM linker script
└── README.md                            # This file
```

---

## 6. Build Environment

| Tool | Version |
|------|---------|
| IDE | STM32CubeIDE 1.19.0 |
| Toolchain | ARM GCC (arm-none-eabi-gcc) |
| HAL | STM32F4xx HAL Driver |
| LwIP | 2.1.2 |
| Target | STM32F429ZITX |
| Debugger | ST-Link V2-1 (onboard NUCLEO) |

### Clock Configuration

| Clock | Frequency | Source |
|-------|-----------|--------|
| SYSCLK | 180 MHz | HSE 8 MHz → PLL (PLLM=4, PLLN=180, PLLP=2) |
| AHB | 180 MHz | SYSCLK / 1 |
| APB1 | 45 MHz | AHB / 4 |
| APB2 | 90 MHz | AHB / 2 |
| TIM1 Clock | 180 MHz | APB2 × 2 |
| TIM4 Clock | 90 MHz | APB1 × 2 |

### Build & Flash

1. STM32CubeIDE에서 프로젝트 Import
2. `servo_control_baqu.ioc`로 CubeMX 설정 확인
3. Build: `Project → Build All` (Ctrl+B)
4. Flash: `Run → Debug` (F11)
5. UART Monitor: 115200 baud (ST-Link Virtual COM)

---

## 7. Execution Flow

### 7.1 Initialization Sequence (현재 테스트 모드)

```
main()
  ├── HAL_Init()
  ├── SystemClock_Config()              // 180 MHz
  ├── MX_GPIO_Init()                    // DIR(PE10), SVON(PD14), EMG(PD15)
  ├── MX_USART3_UART_Init()            // Debug (115200 baud)
  ├── MX_USART1_UART_Init()            // Host comm (예비)
  ├── MX_ADC1_Init()                    // PA4 포텐셔미터
  ├── MX_TIM1_Init()                    // PWM pulse output
  ├── MX_IWDG_Init()                    // Watchdog (~32s)
  ├── MX_TIM4_Init()                    // Encoder input (PD12/PD13)
  ├── // MX_LWIP_Init()                 // [주석] 이더넷 테스트 시 활성화
  ├── HAL_TIM_Encoder_Start(TIM4)       // 엔코더 시작
  ├── Relay_Init()                      // SVON=OFF, EMG=해제
  ├── PulseControl_Init()               // DIR 핀 초기화
  ├── HAL_TIM_PWM_Start(TIM1_CH1/CH2)  // PWM 출력 시작
  ├── EncoderReader_Init()              // 카운터=32768 (중심)
  ├── PositionControl_Init()            // PID 상태 리셋
  ├── Relay_ServoOn()                   // 서보 전원 ON
  ├── HAL_Delay(1000)                   // 서보 드라이버 안정화
  ├── EncoderReader_Reset()             // 현재 위치 = 0도
  ├── PositionControl_SetTarget(10.0f)  // 목표 10도
  └── PositionControl_Enable()          // PID 제어 시작
```

### 7.2 Main Control Loop

```c
while (1) {
    if (interrupt_flag) {           // SysTick 1ms flag
        interrupt_flag = 0;
        PositionControl_Update();   // PID closed-loop

        if (++debug_cnt >= 100) {   // 100ms 주기 디버그
            debug_cnt = 0;
            PositionControl_PrintStatus();
        }
    }
    HAL_IWDG_Refresh(&hiwdg);      // 워치독 리프레시
}
```

### 7.3 Expected UART Output

```
[Encoder] Initialized
[PosCtrl] Initialized
[Encoder] Reset
[PosCtrl] Enabled
Servo Start!
[PosCtrl] Target:10.00 Current:0.00 Error:10.00 Out:5000
[PosCtrl] Target:10.00 Current:3.42 Error:6.58 Out:3290
[PosCtrl] Target:10.00 Current:8.75 Error:1.25 Out:625
[PosCtrl] Target:10.00 Current:9.92 Error:0.08 Out:40 STABLE
```

---

## 8. Development Status

### Phase 1: Motor Control (현재)

| # | Task | Module | Status |
|---|------|--------|--------|
| 1 | PID closed-loop position control | position_control | **Done** |
| 2 | PWM pulse generation + direction | pulse_control | **Done** |
| 3 | TIM4 quadrature encoder (16-bit) | encoder_reader | **Done** |
| 4 | Relay control (SVON/EMG, Active LOW) | relay_control | **Done** |
| 5 | ADC homing (원점 보정) | homing | **Done** |
| 6 | IWDG watchdog | CubeMX / main | **Done** |
| 7 | SysTick 1ms flag + PID loop | stm32f4xx_it / main | **Done** |
| 8 | Safety check + Emergency stop (SW+HW) | position_control | **Done** |

### Phase 2: Ethernet Communication (예정)

| # | Task | Module | Status |
|---|------|--------|--------|
| 9 | LwIP UDP socket 구현 | ethernet_communication | **Pending** |
| 10 | MX_LWIP_Process() 통합 | main | **Pending** |
| 11 | 통신 워치독 (수신 타임아웃 → 비상정지) | ethernet_communication | **Pending** |
| 12 | 바이너리 프로토콜 + CRC16 | ethernet_communication | **Pending** |

### Phase 3: System Integration (계획)

| # | Task | Status |
|---|------|--------|
| 13 | 시스템 상태 머신 (IDLE→HOMING→READY→RUNNING→ERROR) | Planned |
| 14 | 상위 제어기(Jetson) 통합 테스트 | Planned |
| 15 | 센서 이중화 (엔코더 + ADC 교차 검증) | Planned |
| 16 | 상수 통합 (constants.h single source) | Planned |

---

## 9. Known Issues & Bug History

자세한 내용은 [Doc/problem.md](Doc/problem.md) 참조.

### 해결 완료 (14건)

| ID | Severity | Description | Resolution Date |
|----|----------|-------------|-----------------|
| BUG-001 | Critical | SetFrequency() 방향 핀 미설정 → 단방향 회전 | 2026-01-31 |
| BUG-002 | Critical | PositionControl_Update() 미호출 → PID 미실행 | 2026-01-31 |
| BUG-003 | Critical | SetFrequency() PWM Start 누락 → 펄스 미출력 | 2026-01-31 |
| BUG-004 | Major | position_control.h 13개 함수 미구현 | 2026-02-01 |
| BUG-005 | Minor | PID DEFAULT 헤더/소스 불일치 | 2026-02-01 |
| BUG-006 | Minor | stable_time_ms++ → 실제 시간 미반영 | 2026-02-21 |
| BUG-007 | Major | ethernet_communication 모듈 부재 | 2026-02-21 |
| BUG-008 | Critical | PA1 핀 충돌 (TIM2 vs ETH RMII) | 2026-02-21 |
| BUG-009 | Critical | control_enabled 중복 + volatile 누락 + race condition | 2026-02-21 |
| BUG-010 | Critical | EmergencyStop에서 릴레이 미작동 (안전 결함) | 2026-02-21 |
| BUG-011 | Major | DIR_PIN_GPIO_PORT 대소문자 불일치 → 빌드 에러 | 2026-02-21 |
| BUG-012 | Major | debug_cnt 타이밍 오류 → PrintStatus MHz 호출 | 2026-02-21 |
| BUG-013 | Major | USER CODE 블록 위치 → CubeMX 재생성 시 코드 손실 | 2026-02-21 |
| BUG-014 | Major | MX_LWIP_Init() + IWDG 충돌 위험 | 2026-02-21 |

### 미해결 (4건)

| ID | Severity | Description | Target Phase |
|----|----------|-------------|-------------|
| NEW-001 | Major | LwIP UDP 통신 미구현 | Phase 2 |
| NEW-002 | Major | 통신 워치독 미구현 | Phase 2 |
| NEW-003 | Major | 시스템 상태 머신 미구현 | Phase 3 |
| NEW-004 | Minor | constants.h / position_control.h 상수 중복 | Phase 3 |

---

## 10. Safety Features

| Feature | Implementation | Threshold | Status |
|---------|---------------|-----------|--------|
| Angle Hard Limit | `CheckSafety()` | ±50° (45° + 5° 마진) | **Active** |
| Error Limit | `CheckSafety()` | 60° max error | **Active** |
| Emergency Stop (SW+HW) | `EmergencyStop()` | PWM Stop + Relay EMG | **Active** |
| IWDG Watchdog | `HAL_IWDG_Refresh()` | ~32초 timeout | **Active** |
| PID Anti-Windup | `PID_Calculate()` | ±1000.0 integral clamp | **Active** |
| PID Output Limit | `PID_Calculate()` | ±10,000 Hz | **Active** |
| ARR Overflow Guard | `SetFrequency()` | 1 ~ 0xFFFF | **Active** |
| Frequency Clamp | `SetFrequency()` | 10 Hz ~ 100 kHz | **Active** |
| Critical Section | `SetTarget()` | `__disable_irq` / `__enable_irq` | **Active** |
| Volatile Shared Vars | `control_enabled`, `interrupt_flag` | ISR ↔ main 동기화 | **Active** |
| Comm Watchdog | — | 수신 타임아웃 → 비상정지 | **Planned** |
| Sensor Redundancy | — | 엔코더 + ADC 교차 검증 | **Planned** |

---

## 11. NVIC Priority Map

| Interrupt | Preemption Priority | Description |
|-----------|-------------------:|-------------|
| TIM1_CC_IRQn | 1 | 펄스 카운팅 (스텝 모드, 최고 우선순위) |
| ETH_IRQn | 5 | 이더넷 수신 처리 |
| SysTick | 15 | 1ms tick + PID flag (최저 우선순위) |

Priority Group: 4 bits for pre-emption (0~15 범위)

---

## 12. Communication Protocol (Phase 2 예정)

### Text Protocol (현재 골격)

| Command | Format | Description |
|---------|--------|-------------|
| Set Target | `SET TARGET 30.5\n` | 목표 각도 30.5° |
| Servo ON | `SERVO ON\n` | 서보 활성화 |
| Servo OFF | `SERVO OFF\n` | 서보 비활성화 |
| Stop | `STOP\n` | 제어 정지 |
| Get Status | `GET STATUS\n` | 상태 요청 |

### Binary Protocol (권장, 미구현)

| Field | Size | Description |
|-------|------|-------------|
| Header | 2 bytes | 0xAA 0x55 |
| Message ID | 1 byte | 명령 식별자 |
| Sequence | 1 byte | 순서 번호 |
| Timestamp | 4 bytes | ms 단위 시간 |
| Target Angle | 4 bytes | float (IEEE 754) |
| CRC16 | 2 bytes | CRC-CCITT |

---

## 13. Documentation

| Document | Path | Description |
|----------|------|-------------|
| Bug Reports | [Doc/problem.md](Doc/problem.md) | 버그 발견/분석/해결 기록 (BUG-001~014 + NEW) |
| Change Log | [Doc/change_code.md](Doc/change_code.md) | 코드 변경 이력 (전/후 비교, 14건) |
| Team Tasks | [Doc/team_request.md](Doc/team_request.md) | 팀원별 수정 요청 목록 |
| Technical Study | [Doc/position_control_study.md](Doc/position_control_study.md) | PID 제어 기술 분석 |
| Architecture | [Doc/최종 구조.md](Doc/최종%20구조.md) | 시스템 구조 설계 |
| Roadmap | [Doc/향후계획.md](Doc/향후계획.md) | 개발 로드맵 (Phase 1~3) |

---

## 14. License

This software is licensed under terms that can be found in the LICENSE file in the root directory of this software component. STM32 HAL drivers are provided by STMicroelectronics under their respective license terms.

---

*Last updated: 2026-02-21 | Code Quality: B+ (v3 evaluation)*
