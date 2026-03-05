# Hardware Pin Map (Current Runtime)

이 문서는 현재 코드 런타임 기준의 핵심 핀 매핑을 정리한다.

## 1) Motion Control

| Function | MCU Pin | Block | Usage |
|---|---|---|---|
| Pulse Output | PE9 | TIM1_CH1 | Servo pulse output |
| Direction Output | PE10 | GPIO Output | Direction control |
| Encoder A | PD12 | TIM4_CH1 | Quadrature A |
| Encoder B | PD13 | TIM4_CH2 | Quadrature B |

## 2) Safety Relay

| Function | MCU Pin | IO Type | Active Level |
|---|---|---|---|
| SVON Relay | PD14 | GPIO Output | LOW = ON |
| EMG Relay | PD15 | GPIO Output | LOW = Emergency |

## 3) Communication

| Function | MCU Pin | Peripheral | Note |
|---|---|---|---|
| Debug TX | PD8 | USART3_TX | 115200 |
| Debug RX | PD9 | USART3_RX | 115200 |
| Ethernet | RMII set | ETH + LAN8742 | LwIP UDP |

RMII 주요 핀(요약): PA1, PA2, PA7, PB13, PC1, PC4, PC5, PG11, PG13

## 4) Notes

- 최종 핀 정합은 CubeMX `.ioc`와 `main.h`를 기준으로 검증.
- 안전/실측 환경에서는 핀 재할당 변경 시 latency/fail-safe 재검증 필요.
