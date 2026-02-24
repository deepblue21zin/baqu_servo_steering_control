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
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"

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

/* ============================================================
 * UDP 자율주행 수신 구현
 * ============================================================ */

static struct udp_pcb      *g_udp_pcb   = NULL;
static AutoDrive_Packet_t   g_latest_pkt;
static volatile bool        g_new_data  = false;
static volatile SteerMode_t g_current_mode = STEER_MODE_NONE;
static volatile bool        g_emergency_request = false;
static volatile uint32_t    g_last_rx_tick = 0U;

#define ASMS_IP_LAST_OCTET   5U
#define PC_IP_LAST_OCTET     1U
#define ASMS_PACKET_SIZE     5U
#define PC_PACKET_SIZE       9U
#define ETHCOMM_DEBUG        0

static float clamp_deg(float v)
{
    if (v > 360.0f) return 360.0f;
    if (v < -360.0f) return -360.0f;
    return v;
}

static float joy_to_deg(int16_t joy_y)
{
    if (joy_y > 2047) joy_y = 2047;
    if (joy_y < -2048) joy_y = -2048;
    return clamp_deg((360.0f * (float)joy_y) / 2047.0f);
}

/**
 * @brief LwIP가 UDP 패킷 수신 시 자동 호출하는 콜백
 *
 * 흐름:
 *   인지PC 브로드캐스트 → 이더넷 허브 → STM32 NIC
 *   → LwIP 스택 → ethernetif_input() → udp_input()
 *   → 이 콜백 호출
 *
 * [필터링 로직]
 *   - ASMS 5B: sender .5 만 허용, mode + joy_y 처리
 *   - PC   9B: sender .1 만 허용, AUTO 모드에서 steer 처리
 */
static void udp_recv_cb(void *arg,
                        struct udp_pcb *pcb,
                        struct pbuf *p,
                        const ip_addr_t *addr,
                        u16_t port)
{
    (void)arg; (void)pcb; (void)port;

    if (p == NULL) {
        return;
    }

    uint16_t len = p->tot_len;
    uint8_t buffer[16] = {0};

    if (len > (uint16_t)sizeof(buffer)) {
        pbuf_free(p);
        return;
    }

    pbuf_copy_partial(p, buffer, len, 0);
    pbuf_free(p);   /* LwIP 메모리 즉시 해제 (필수!) */

    if (addr == NULL || !IP_IS_V4(addr)) {
        return;
    }
    uint8_t sender = ip4_addr4(ip_2_ip4(addr));
#if ETHCOMM_DEBUG
    printf("[EthComm][RX] len:%u sender:%u port:%u\r\n",
           (unsigned)len, (unsigned)sender, (unsigned)port);
#endif

    /* ── 5 bytes: ASMS mode + joystick ── */
    if (len == ASMS_PACKET_SIZE && sender == ASMS_IP_LAST_OCTET) {
        uint8_t mode = buffer[0];
        int16_t joy_y = (int16_t)((buffer[4] << 8) | buffer[3]);

        if (mode <= (uint8_t)STEER_MODE_ESTOP) {
            g_current_mode = (SteerMode_t)mode;
#if ETHCOMM_DEBUG
            printf("[EthComm][ASMS] mode:%u joyY:%d\r\n",
                   (unsigned)mode, (int)joy_y);
#endif
        }
        g_last_rx_tick = HAL_GetTick();

        if (g_current_mode == STEER_MODE_MANUAL) {
            g_latest_pkt.steering_angle = joy_to_deg(joy_y);
            g_latest_pkt.speed = 0U;
            g_latest_pkt.misc = 0U;
            g_new_data = true;
        } else if (g_current_mode == STEER_MODE_ESTOP) {
            g_emergency_request = true;
        }
        return;
    }

    /* ── 9 bytes: PC steer/speed/misc (AUTO mode only) ── */
    if (len == PC_PACKET_SIZE && sender == PC_IP_LAST_OCTET) {
        if (g_current_mode != STEER_MODE_AUTO) {
#if ETHCOMM_DEBUG
            printf("[EthComm][PC] ignored: mode=%d\r\n", (int)g_current_mode);
#endif
            return;
        }

        int32_t pc_steer = (int32_t)(
            ((uint32_t)buffer[0]) |
            ((uint32_t)buffer[1] << 8) |
            ((uint32_t)buffer[2] << 16) |
            ((uint32_t)buffer[3] << 24));
        uint8_t pc_misc = buffer[8];
        uint32_t pc_speed = (uint32_t)(
            ((uint32_t)buffer[4]) |
            ((uint32_t)buffer[5] << 8) |
            ((uint32_t)buffer[6] << 16) |
            ((uint32_t)buffer[7] << 24));

        if ((pc_misc >> 7) & 0x01U) {
            g_emergency_request = true;
            g_current_mode = STEER_MODE_ESTOP;
            g_last_rx_tick = HAL_GetTick();
            return;
        }

        g_latest_pkt.steering_angle = clamp_deg((float)pc_steer);
        g_latest_pkt.speed = pc_speed;
        g_latest_pkt.misc = pc_misc;
        g_new_data = true;
        g_last_rx_tick = HAL_GetTick();
#if ETHCOMM_DEBUG
        printf("[EthComm][PC] steer:%ld -> %.1f\r\n",
               (long)pc_steer, g_latest_pkt.steering_angle);
#endif
        return;
    }
}

/**
 * @brief UDP 수신 소켓 초기화 (MX_LWIP_Init() 이후 1회 호출)
 */
void EthComm_UDP_Init(void)
{
    g_udp_pcb = udp_new();
    if (g_udp_pcb == NULL) {
        printf("[EthComm] ERROR: udp_new() failed\r\n");
        return;
    }

    err_t err = udp_bind(g_udp_pcb, IP_ADDR_ANY, AUTODRIVE_UDP_PORT);
    if (err != ERR_OK) {
        printf("[EthComm] ERROR: udp_bind() failed (%d)\r\n", (int)err);
        udp_remove(g_udp_pcb);
        g_udp_pcb = NULL;
        return;
    }

    udp_recv(g_udp_pcb, udp_recv_cb, NULL);
    printf("[EthComm] UDP ready  PORT:%d\r\n", AUTODRIVE_UDP_PORT);
}

/**
 * @brief 새 패킷 수신 여부 확인
 * @return true : 새 데이터 있음 → EthComm_GetLatestData() 로 읽을 것
 */
bool EthComm_HasNewData(void)
{
    return g_new_data;
}

/**
 * @brief 가장 최근 수신 패킷 반환 (플래그 초기화)
 */
AutoDrive_Packet_t EthComm_GetLatestData(void)
{
    g_new_data = false;
    return g_latest_pkt;
}

SteerMode_t EthComm_GetCurrentMode(void)
{
    return g_current_mode;
}

bool EthComm_ConsumeEmergencyRequest(void)
{
    bool req = g_emergency_request;
    g_emergency_request = false;
    return req;
}

uint32_t EthComm_GetLastRxTick(void)
{
    return g_last_rx_tick;
}

void EthComm_ForceMode(SteerMode_t mode)
{
    if (mode <= STEER_MODE_ESTOP) {
        g_current_mode = mode;
    }
}
