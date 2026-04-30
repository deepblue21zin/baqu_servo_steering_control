#ifndef RS422_ENCODER_UART_H
#define RS422_ENCODER_UART_H

#include <stdint.h>

typedef struct {
    uint8_t initialized;
    uint8_t rx_len;
    uint32_t bytes;
    uint32_t frames;
    uint32_t uart_errors;
    uint32_t partial_dumps;
    int32_t last_count;
    float last_motor_deg;
    float last_steering_deg;
    uint32_t last_frame_tick_ms;
} Rs422Encoder_Status_t;

int Rs422Encoder_Init(void);
void Rs422Encoder_Service(void);
uint8_t Rs422Encoder_GetLatest(Rs422Encoder_Status_t *out_status);

#endif /* RS422_ENCODER_UART_H */
