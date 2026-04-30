#include "can_encoder_bridge.h"

#include "can.h"
#include "project_params.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define CAN_ENCODER_UART_FRAME_SIZE 4U

typedef struct {
    uint8_t initialized;
    uint8_t can_started;
    uint8_t rx_buf[CAN_ENCODER_UART_FRAME_SIZE];
    uint8_t rx_len;
    uint32_t uart_frames;
    uint32_t can_frames;
    uint32_t uart_errors;
    uint32_t can_tx_errors;
    uint32_t can_tx_drops;
    int32_t last_encoder_value;
} CanEncoderBridge_Context_t;

static CanEncoderBridge_Context_t g_can_encoder_bridge;

static HAL_StatusTypeDef CanEncoderBridge_ConfigFilter(void);
static void CanEncoderBridge_ProcessByte(uint8_t byte);
static int CanEncoderBridge_SendEncoder(int32_t encoder_value);

int CanEncoderBridge_Init(void)
{
#if CAN_ENCODER_BRIDGE_ENABLE
    HAL_StatusTypeDef ret = HAL_OK;

    memset(&g_can_encoder_bridge, 0, sizeof(g_can_encoder_bridge));

    ret = CanEncoderBridge_ConfigFilter();
    if (ret != HAL_OK) {
        g_can_encoder_bridge.can_tx_errors++;
        printf("[CANBR] filter config failed\r\n");
        return -1;
    }

    ret = HAL_CAN_Start(&hcan1);
    if (ret != HAL_OK) {
        g_can_encoder_bridge.can_tx_errors++;
        printf("[CANBR] CAN start failed\r\n");
        return -2;
    }

    g_can_encoder_bridge.initialized = 1U;
    g_can_encoder_bridge.can_started = 1U;

#if CAN_ENCODER_BRIDGE_LOG_ENABLE
    printf("[CANBR] ready uart=USART1 baud=%lu can=CAN1 500k id=0x%03lX\r\n",
           (unsigned long)CAN_ENCODER_UART_BAUDRATE,
           (unsigned long)CAN_ENCODER_BRIDGE_STD_ID);
#endif

    return 0;
#else
    memset(&g_can_encoder_bridge, 0, sizeof(g_can_encoder_bridge));
    return 0;
#endif
}

void CanEncoderBridge_Service(void)
{
#if CAN_ENCODER_BRIDGE_ENABLE
    uint8_t byte = 0U;
    uint32_t i = 0U;

    if (g_can_encoder_bridge.initialized == 0U) {
        return;
    }

    for (i = 0U; i < CAN_ENCODER_BRIDGE_MAX_BYTES_PER_SERVICE; i++) {
        HAL_StatusTypeDef ret = HAL_UART_Receive(&huart1, &byte, 1U, 0U);

        if (ret == HAL_OK) {
            CanEncoderBridge_ProcessByte(byte);
            continue;
        }

        if (ret == HAL_ERROR) {
            g_can_encoder_bridge.uart_errors++;
            g_can_encoder_bridge.rx_len = 0U;
            (void)HAL_UART_AbortReceive(&huart1);
        }

        break;
    }
#endif
}

void CanEncoderBridge_GetStatus(CanEncoderBridge_Status_t *out_status)
{
    if (out_status == NULL) {
        return;
    }

    out_status->initialized = g_can_encoder_bridge.initialized;
    out_status->can_started = g_can_encoder_bridge.can_started;
    out_status->rx_len = g_can_encoder_bridge.rx_len;
    out_status->uart_frames = g_can_encoder_bridge.uart_frames;
    out_status->can_frames = g_can_encoder_bridge.can_frames;
    out_status->uart_errors = g_can_encoder_bridge.uart_errors;
    out_status->can_tx_errors = g_can_encoder_bridge.can_tx_errors;
    out_status->can_tx_drops = g_can_encoder_bridge.can_tx_drops;
    out_status->last_encoder_value = g_can_encoder_bridge.last_encoder_value;
}

static HAL_StatusTypeDef CanEncoderBridge_ConfigFilter(void)
{
    CAN_FilterTypeDef can_filter = {0};

    can_filter.FilterActivation = ENABLE;
    can_filter.FilterBank = 0;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.SlaveStartFilterBank = 14;

    return HAL_CAN_ConfigFilter(&hcan1, &can_filter);
}

static void CanEncoderBridge_ProcessByte(uint8_t byte)
{
    uint32_t raw_value = 0U;
    int32_t encoder_value = 0;

    g_can_encoder_bridge.rx_buf[g_can_encoder_bridge.rx_len] = byte;
    g_can_encoder_bridge.rx_len++;

    if (g_can_encoder_bridge.rx_len < CAN_ENCODER_UART_FRAME_SIZE) {
        return;
    }

    raw_value = ((uint32_t)g_can_encoder_bridge.rx_buf[0]) |
                ((uint32_t)g_can_encoder_bridge.rx_buf[1] << 8) |
                ((uint32_t)g_can_encoder_bridge.rx_buf[2] << 16) |
                ((uint32_t)g_can_encoder_bridge.rx_buf[3] << 24);
    encoder_value = (int32_t)raw_value;

    g_can_encoder_bridge.rx_len = 0U;
    g_can_encoder_bridge.uart_frames++;
    g_can_encoder_bridge.last_encoder_value = encoder_value;

    (void)CanEncoderBridge_SendEncoder(encoder_value);
}

static int CanEncoderBridge_SendEncoder(int32_t encoder_value)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox = 0U;
    uint32_t raw_value = (uint32_t)encoder_value;

    if (g_can_encoder_bridge.can_started == 0U) {
        g_can_encoder_bridge.can_tx_drops++;
        return -1;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        g_can_encoder_bridge.can_tx_drops++;
        return -2;
    }

    tx_header.StdId = CAN_ENCODER_BRIDGE_STD_ID;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = CAN_ENCODER_UART_FRAME_SIZE;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = (uint8_t)(raw_value & 0xFFU);
    tx_data[1] = (uint8_t)((raw_value >> 8) & 0xFFU);
    tx_data[2] = (uint8_t)((raw_value >> 16) & 0xFFU);
    tx_data[3] = (uint8_t)((raw_value >> 24) & 0xFFU);

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        g_can_encoder_bridge.can_tx_errors++;
        return -3;
    }

    g_can_encoder_bridge.can_frames++;
    return 0;
}
