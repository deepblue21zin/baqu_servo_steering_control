// 시스템 상수
#define CONTROL_PERIOD_MS       1       // 제어 주기 1ms
#define ENCODER_PPR             12000   // 엔코더 해상도 (XML-FBL04AMK1)
#define ENCODER_QUADRATURE      4       // 4체배 (TIM2 Encoder Mode TI1 and TI2)
#define ENCODER_COUNT_PER_REV   (ENCODER_PPR * ENCODER_QUADRATURE)  // 48000
#define FULL_ROTATION_DEG       360.0f

// 서보드라이브 (XDL-L7SA004BAA) 설정
#define DEG_PER_PULSE           0.003f  // 1펄스 = 0.003도 (전자기어비)
#define PULSE_PER_DEG           (1.0f / DEG_PER_PULSE)  // 333.33 펄스/도
#define MAX_PULSE_FREQ          1000000 // 최대 펄스 주파수 1MHz
#define MIN_PULSE_WIDTH_US      0.5f    // 최소 펄스 폭 0.5us

// 안전 범위
#define MAX_STEERING_ANGLE      45.0f   // 최대 조향 각도
#define MIN_STEERING_ANGLE     -45.0f   // 최소 조향 각도

// 허용 오차
#define POSITION_TOLERANCE      0.5f    // 위치 허용 오차 (degree)
#define VELOCITY_MAX            1000.0f // 최대 각속도 (deg/s)
#define VELOCITY_MIN           -1000.0f // 최소 각속도 (deg/s)
