#include "rs422_encoder_uart.h"

#include "constants.h"
#include "project_params.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#if RS422_ENCODER_FRAME_SIZE != 4U
#error "rs422_encoder_uart.c currently decodes exactly 4-byte little-endian signed count frames."
#endif

typedef struct {
    uint8_t initialized;
    uint8_t rx_buf[RS422_ENCODER_FRAME_SIZE];
    uint8_t rx_len;
    uint32_t bytes;
    uint32_t frames;
    uint32_t uart_errors;
    uint32_t partial_dumps;
    uint32_t last_byte_tick_ms;
    uint32_t last_frame_tick_ms;
    uint32_t last_print_tick_ms;
    uint32_t last_status_print_tick_ms;
    int32_t last_count;
    float last_motor_deg;
    float last_steering_deg;
} Rs422Encoder_Context_t;

static Rs422Encoder_Context_t g_rs422_encoder;

static void Rs422Encoder_ProcessByte(uint8_t byte);
static void Rs422Encoder_ProcessFrame(void);
static void Rs422Encoder_DumpPartialIfIdle(uint32_t now_ms);
static void Rs422Encoder_PrintStatusIfDue(uint32_t now_ms);

int Rs422Encoder_Init(void)
{
#if RS422_ENCODER_READER_ENABLE
    memset(&g_rs422_encoder, 0, sizeof(g_rs422_encoder));
    g_rs422_encoder.initialized = 1U;

    printf("[RS422] ready uart=USART2_RX pin=PA3 baud=%lu frame=%lu bytes\r\n",
           (unsigned long)RS422_ENCODER_UART_BAUDRATE,
           (unsigned long)RS422_ENCODER_FRAME_SIZE);
    printf("[RS422] wiring: CN4 TX+->SN75179 A, TX- ->SN75179 B, R->PA3, common GND\r\n");
    return 0;
#else
    memset(&g_rs422_encoder, 0, sizeof(g_rs422_encoder));
    return 0;
#endif
}

void Rs422Encoder_Service(void)
{
#if RS422_ENCODER_READER_ENABLE
    uint8_t byte = 0U;
    uint32_t i = 0U;

    if (g_rs422_encoder.initialized == 0U) {
        return;
    }

    for (i = 0U; i < RS422_ENCODER_MAX_BYTES_PER_SERVICE; i++) {
        HAL_StatusTypeDef ret = HAL_UART_Receive(&huart2, &byte, 1U, 0U);

        if (ret == HAL_OK) {
            Rs422Encoder_ProcessByte(byte);
            continue;
        }

        if (ret == HAL_ERROR) {
            g_rs422_encoder.uart_errors++;
            g_rs422_encoder.rx_len = 0U;
            (void)HAL_UART_AbortReceive(&huart2);
        }

        break;
    }

    Rs422Encoder_DumpPartialIfIdle(HAL_GetTick());
    Rs422Encoder_PrintStatusIfDue(HAL_GetTick());
#endif
}

uint8_t Rs422Encoder_GetLatest(Rs422Encoder_Status_t *out_status)
{
    if (out_status == NULL) {
        return 0U;
    }

    out_status->initialized = g_rs422_encoder.initialized;
    out_status->rx_len = g_rs422_encoder.rx_len;
    out_status->bytes = g_rs422_encoder.bytes;
    out_status->frames = g_rs422_encoder.frames;
    out_status->uart_errors = g_rs422_encoder.uart_errors;
    out_status->partial_dumps = g_rs422_encoder.partial_dumps;
    out_status->last_count = g_rs422_encoder.last_count;
    out_status->last_motor_deg = g_rs422_encoder.last_motor_deg;
    out_status->last_steering_deg = g_rs422_encoder.last_steering_deg;
    out_status->last_frame_tick_ms = g_rs422_encoder.last_frame_tick_ms;

    return (g_rs422_encoder.frames > 0U) ? 1U : 0U;
}

static void Rs422Encoder_ProcessByte(uint8_t byte)
{
    g_rs422_encoder.last_byte_tick_ms = HAL_GetTick();
    g_rs422_encoder.bytes++;

    if (g_rs422_encoder.rx_len >= RS422_ENCODER_FRAME_SIZE) {
        g_rs422_encoder.rx_len = 0U;
    }

    g_rs422_encoder.rx_buf[g_rs422_encoder.rx_len] = byte;
    g_rs422_encoder.rx_len++;

    if (g_rs422_encoder.rx_len >= RS422_ENCODER_FRAME_SIZE) {
        Rs422Encoder_ProcessFrame();
        g_rs422_encoder.rx_len = 0U;
    }
}

static void Rs422Encoder_ProcessFrame(void)
{
    uint32_t raw_value = 0U;
    int32_t encoder_count = 0;
    float motor_deg = 0.0f;
    float steering_deg = 0.0f;
    uint32_t now_ms = HAL_GetTick();
    uint8_t should_print = 0U;

    raw_value = ((uint32_t)g_rs422_encoder.rx_buf[0]) |
                ((uint32_t)g_rs422_encoder.rx_buf[1] << 8) |
                ((uint32_t)g_rs422_encoder.rx_buf[2] << 16) |
                ((uint32_t)g_rs422_encoder.rx_buf[3] << 24);
    encoder_count = (int32_t)raw_value;
    motor_deg = (float)encoder_count * ENCODER_DEG_PER_COUNT;
    steering_deg = MotorDegToSteeringDeg(motor_deg);

    g_rs422_encoder.frames++;
    g_rs422_encoder.last_count = encoder_count;
    g_rs422_encoder.last_motor_deg = motor_deg;
    g_rs422_encoder.last_steering_deg = steering_deg;
    g_rs422_encoder.last_frame_tick_ms = now_ms;

#if RS422_ENCODER_PRINT_EACH_FRAME
    should_print = 1U;
#else
    if (g_rs422_encoder.frames <= RS422_ENCODER_PRINT_FIRST_FRAMES) {
        should_print = 1U;
    } else if ((now_ms - g_rs422_encoder.last_print_tick_ms) >= RS422_ENCODER_PRINT_PERIOD_MS) {
        should_print = 1U;
    }
#endif

    if (should_print != 0U) {
        g_rs422_encoder.last_print_tick_ms = now_ms;
        printf("[RS422] HEX=%02X %02X %02X %02X Encoder Count:%ld Motor Deg:%.3f Current Deg:%.3f\r\n",
               (unsigned int)g_rs422_encoder.rx_buf[0],
               (unsigned int)g_rs422_encoder.rx_buf[1],
               (unsigned int)g_rs422_encoder.rx_buf[2],
               (unsigned int)g_rs422_encoder.rx_buf[3],
               (long)encoder_count,
               motor_deg,
               steering_deg);
    }
}

static void Rs422Encoder_DumpPartialIfIdle(uint32_t now_ms)
{
    uint8_t i = 0U;

    if (g_rs422_encoder.rx_len == 0U) {
        return;
    }

    if ((now_ms - g_rs422_encoder.last_byte_tick_ms) < RS422_ENCODER_IDLE_DUMP_MS) {
        return;
    }

    printf("[RS422] partial HEX len=%u", (unsigned int)g_rs422_encoder.rx_len);
    for (i = 0U; i < g_rs422_encoder.rx_len; i++) {
        printf(" %02X", (unsigned int)g_rs422_encoder.rx_buf[i]);
    }
    printf("\r\n");

    g_rs422_encoder.partial_dumps++;
    g_rs422_encoder.rx_len = 0U;
}

static void Rs422Encoder_PrintStatusIfDue(uint32_t now_ms)
{
    if ((now_ms - g_rs422_encoder.last_status_print_tick_ms) < RS422_ENCODER_STATUS_LOG_PERIOD_MS) {
        return;
    }

    g_rs422_encoder.last_status_print_tick_ms = now_ms;
    printf("[RS422][STAT] bytes=%lu frames=%lu rx_len=%u uart_errors=%lu partial=%lu last_count=%ld current_deg=%.3f\r\n",
           (unsigned long)g_rs422_encoder.bytes,
           (unsigned long)g_rs422_encoder.frames,
           (unsigned int)g_rs422_encoder.rx_len,
           (unsigned long)g_rs422_encoder.uart_errors,
           (unsigned long)g_rs422_encoder.partial_dumps,
           (long)g_rs422_encoder.last_count,
           g_rs422_encoder.last_steering_deg);
}
