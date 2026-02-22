/*
 * ethernet_communication.c
 *
 * 구조만 잡아둔 템플릿:
 * - 초기화
 * - 주기 업데이트
 * - 라인 파싱
 * - 명령 디스패치
 *
 * TODO 표시된 부분을 직접 구현해서 채우면 됩니다.
 */

#include "ethernet_communication.h"

#include "main.h"
#include "position_control.h"
#include "pulse_control.h"
#include "relay_control.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ========== Private Types ========== */

typedef struct {
    bool initialized;
    EthComm_Config_t cfg;
    uint16_t rx_len;
} EthComm_Context_t;

/* ========== Private Variables ========== */

static EthComm_Context_t g_eth = {0};

/* ========== Private Function Prototypes ========== */

static int EthComm_ReadRxByte(uint8_t *out_byte);
static int EthComm_ProcessRxByte(uint8_t byte);
static int EthComm_ParseCommand(const char *line, EthComm_Command_t *out_cmd);
static int EthComm_DispatchCommand(const EthComm_Command_t *cmd);
static int EthComm_SendStatus(void);

/* ========== Public Functions ========== */

int EthComm_Init(const EthComm_Config_t *config)
{
    if (config == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }
    if (config->rx_buffer == NULL || config->rx_buffer_size == 0) {
        return ETH_COMM_ERR_BAD_PARAM;
    }
    if (config->tx_buffer == NULL || config->tx_buffer_size == 0) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    memset(&g_eth, 0, sizeof(g_eth));
    g_eth.cfg = *config;
    g_eth.initialized = true;

    /* TODO: Ethernet PHY/Socket/UART 브리지 초기화 코드를 여기에 추가 */

    return ETH_COMM_OK;
}

void EthComm_Update(void)
{
    uint8_t byte = 0;

    if (!g_eth.initialized) {
        return;
    }

    /*
     * TODO:
     * - non-blocking 방식으로 수신 바이트를 반복 읽기
     * - 데이터가 있으면 EthComm_ProcessRxByte 호출
     * - 아래 while 조건부는 실제 드라이버에 맞게 교체
     */
    while (EthComm_ReadRxByte(&byte) == ETH_COMM_OK) {
        (void)EthComm_ProcessRxByte(byte);
    }
}

bool EthComm_IsInitialized(void)
{
    return g_eth.initialized;
}

int EthComm_SendString(const char *msg)
{
    size_t len = 0;

    if (!g_eth.initialized || msg == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    len = strlen(msg);
    if (len >= g_eth.cfg.tx_buffer_size) {
        len = g_eth.cfg.tx_buffer_size - 1;
    }

    memcpy(g_eth.cfg.tx_buffer, msg, len);
    g_eth.cfg.tx_buffer[len] = '\0';

    /* TODO: 실제 전송 함수로 교체 (LAN8742/LwIP, Wiznet, UART 등) */
    /* 예: HAL_UART_Transmit(...), netconn_write(...), send(...) */

    return ETH_COMM_OK;
}

int EthComm_HandleLine(const char *line)
{
    EthComm_Command_t cmd = {0};
    int ret = 0;

    if (!g_eth.initialized || line == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    ret = EthComm_ParseCommand(line, &cmd);
    if (ret != ETH_COMM_OK) {
        (void)EthComm_SendString("ERR PARSE\r\n");
        return ret;
    }

    ret = EthComm_DispatchCommand(&cmd);
    if (ret != ETH_COMM_OK) {
        (void)EthComm_SendString("ERR EXEC\r\n");
        return ret;
    }

    return ETH_COMM_OK;
}

/* ========== Private Functions ========== */

static int EthComm_ReadRxByte(uint8_t *out_byte)
{
    if (out_byte == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    /*
     * TODO:
     * - 수신 데이터가 있으면 *out_byte에 1바이트 저장 후 ETH_COMM_OK 반환
     * - 데이터가 없으면 ETH_COMM_ERR_RX_FAIL 또는 별도 코드 반환
     * - 블로킹 금지(주기 루프 성능 유지)
     */
    (void)out_byte;
    return ETH_COMM_ERR_RX_FAIL;
}

static int EthComm_ProcessRxByte(uint8_t byte)
{
    char *line = (char *)g_eth.cfg.rx_buffer;

    if (!g_eth.initialized) {
        return ETH_COMM_ERR_NOT_INIT;
    }

    /* 줄바꿈 기반 프로토콜: "CMD ARG\r\n" */
    if (byte == '\r') {
        return ETH_COMM_OK;
    }

    if (byte == '\n') {
        line[g_eth.rx_len] = '\0';
        g_eth.rx_len = 0;
        return EthComm_HandleLine(line);
    }

    if (g_eth.rx_len < (uint16_t)(g_eth.cfg.rx_buffer_size - 1)) {
        line[g_eth.rx_len++] = (char)byte;
    } else {
        /* 버퍼 오버플로우 방지: 라인 폐기 */
        g_eth.rx_len = 0;
        return ETH_COMM_ERR_RX_FAIL;
    }

    return ETH_COMM_OK;
}

static int EthComm_ParseCommand(const char *line, EthComm_Command_t *out_cmd)
{
    char cmd[24] = {0};
    char arg[32] = {0};
    int count = 0;

    if (line == NULL || out_cmd == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    memset(out_cmd, 0, sizeof(*out_cmd));

    /*
     * 프로토콜 예시:
     * - "SERVO ON"
     * - "SERVO OFF"
     * - "SET TARGET 12.5"
     * - "STOP"
     * - "GET STATUS"
     */
    count = sscanf(line, "%23s %31s", cmd, arg);
    if (count <= 0) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    if (strcmp(cmd, "STOP") == 0) {
        out_cmd->type = ETH_CMD_STOP;
        return ETH_COMM_OK;
    }

    if (strcmp(cmd, "GET") == 0 && strcmp(arg, "STATUS") == 0) {
        out_cmd->type = ETH_CMD_GET_STATUS;
        return ETH_COMM_OK;
    }

    if (strcmp(cmd, "SERVO") == 0) {
        if (strcmp(arg, "ON") == 0) {
            out_cmd->type = ETH_CMD_SERVO_ON;
            return ETH_COMM_OK;
        }
        if (strcmp(arg, "OFF") == 0) {
            out_cmd->type = ETH_CMD_SERVO_OFF;
            return ETH_COMM_OK;
        }
    }

    if (strncmp(line, "SET TARGET ", 11) == 0) {
        out_cmd->type = ETH_CMD_SET_TARGET;
        out_cmd->f32_value = strtof(&line[11], NULL);
        return ETH_COMM_OK;
    }

    return ETH_COMM_ERR_BAD_PARAM;
}

static int EthComm_DispatchCommand(const EthComm_Command_t *cmd)
{
    if (cmd == NULL) {
        return ETH_COMM_ERR_BAD_PARAM;
    }

    switch (cmd->type) {
    case ETH_CMD_SERVO_ON:
        Relay_ServoOn();
        return EthComm_SendString("OK SERVO ON\r\n");

    case ETH_CMD_SERVO_OFF:
        Relay_ServoOff();
        return EthComm_SendString("OK SERVO OFF\r\n");

    case ETH_CMD_SET_TARGET:
        if (PositionControl_SetTarget(cmd->f32_value) != 0) {
            return EthComm_SendString("ERR TARGET\r\n");
        }
        return EthComm_SendString("OK TARGET\r\n");

    case ETH_CMD_STOP:
        PositionControl_Disable();
        PulseControl_Stop();
        return EthComm_SendString("OK STOP\r\n");

    case ETH_CMD_GET_STATUS:
        return EthComm_SendStatus();

    default:
        return ETH_COMM_ERR_BAD_PARAM;
    }
}

static int EthComm_SendStatus(void)
{
    float cur = PositionControl_GetCurrentAngle();
    float tgt = PositionControl_GetTarget();
    float err = PositionControl_GetError();
    int n = 0;

    n = snprintf((char *)g_eth.cfg.tx_buffer,
                 g_eth.cfg.tx_buffer_size,
                 "OK STATUS CUR=%.2f TGT=%.2f ERR=%.2f\r\n",
                 cur, tgt, err);
    if (n <= 0) {
        return ETH_COMM_ERR_TX_FAIL;
    }

    /* TODO: 실제 네트워크 전송 함수로 교체 */
    return EthComm_SendString((char *)g_eth.cfg.tx_buffer);
}
