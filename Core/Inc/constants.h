#ifndef CONSTANTS_H
#define CONSTANTS_H

// System constants
#define CONTROL_PERIOD_MS       1
#define ENCODER_PPR             12000
#define ENCODER_QUADRATURE      4
#define ENCODER_COUNT_PER_REV   (ENCODER_PPR * ENCODER_QUADRATURE)
#define FULL_ROTATION_DEG       360.0f
#define ENCODER_DEG_PER_COUNT   (FULL_ROTATION_DEG / ENCODER_COUNT_PER_REV)  /* MODIFIED(Codex): shared encoder conversion */

// Mechanical conversion constants
#define STEERING_GEAR_RATIO             12.5f
#define MOTOR_DEG_PER_STEERING_DEG      STEERING_GEAR_RATIO
#define STEERING_DEG_PER_MOTOR_DEG      (1.0f / MOTOR_DEG_PER_STEERING_DEG)

// Servo drive constants
#define DEG_PER_PULSE           0.003f
#define PULSE_PER_DEG           (1.0f / DEG_PER_PULSE)
#define MAX_PULSE_FREQ          1000000
#define MIN_PULSE_WIDTH_US      0.5f

// Steering-axis limits
#define MAX_STEERING_ANGLE      45.0f
#define MIN_STEERING_ANGLE     -45.0f
#define POT_MIN_ANGLE_DEG      MIN_STEERING_ANGLE  /* MODIFIED(Codex): shared ADC steering range */
#define POT_MAX_ANGLE_DEG      MAX_STEERING_ANGLE  /* MODIFIED(Codex): shared ADC steering range */
#define ADC_VREF               3.3f                /* MODIFIED(Codex): shared ADC scaling */
#define ADC_MAX_COUNT          4095.0f             /* MODIFIED(Codex): shared ADC scaling */

// Control tolerances
#define POSITION_TOLERANCE      0.5f
#define VELOCITY_MAX            1000.0f
#define VELOCITY_MIN           -1000.0f

static inline float SteeringDegToMotorDeg(float steering_deg)
{
    return steering_deg * MOTOR_DEG_PER_STEERING_DEG;
}

static inline float MotorDegToSteeringDeg(float motor_deg)
{
    return motor_deg * STEERING_DEG_PER_MOTOR_DEG;
}

#endif /* CONSTANTS_H */
