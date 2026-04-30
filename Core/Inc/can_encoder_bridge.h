#ifndef CAN_ENCODER_BRIDGE_H
#define CAN_ENCODER_BRIDGE_H

#include <stdint.h>

typedef struct {
    uint8_t initialized;
    uint8_t can_started;
    uint8_t rx_len;
    uint32_t uart_frames;
    uint32_t can_frames;
    uint32_t uart_errors;
    uint32_t can_tx_errors;
    uint32_t can_tx_drops;
    int32_t last_encoder_value;
} CanEncoderBridge_Status_t;

int CanEncoderBridge_Init(void);
void CanEncoderBridge_Service(void);
void CanEncoderBridge_GetStatus(CanEncoderBridge_Status_t *out_status);

#endif /* CAN_ENCODER_BRIDGE_H */
