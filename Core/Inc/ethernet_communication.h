#ifndef INC_ETHERNET_COMMUNICATION_H_
#define INC_ETHERNET_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 * [TODO: 인지파트 확인 필요]
 * AUTODRIVE_UDP_PORT : 인지파트가 브로드캐스트하는 UDP 포트 번호
 * AutoDrive_Packet_t : 실제 패킷 구조와 일치하도록 수정
 * ========================================================= */

/* UDP 포트 번호 (레거시 송신기 규격: 5000) */
#define AUTODRIVE_UDP_PORT    5000

/* 패킷 식별자 */
#define AUTODRIVE_PKT_HEADER  0xAA
#define AUTODRIVE_MSG_CTRL    0x01   /* 속도·조향·ASMS 통합 제어 메시지 */

/* 자율주행 통합 패킷 구조체 (총 20 bytes)
 * -- 인지파트의 실제 포맷과 다르면 필드/순서/크기를 수정할 것 -- */
#pragma pack(push, 1)
typedef struct {
    uint8_t  header;          /* 0xAA : 패킷 시작 식별자          */
    uint8_t  msg_type;        /* 0x01 : 제어 메시지 타입           */
    float    steering_angle;  /* 조향각  [deg]  -45.0 ~ +45.0    */
    float    speed;           /* 목표속도 [m/s]                   */
    float    asms;            /* ASMS 값                         */
    uint8_t  flags;           /* 비트 플래그 (bit0=비상정지 등)    */
    uint16_t checksum;        /* XOR 체크섬 (0이면 무시)           */
} AutoDrive_Packet_t;         /* 합계: 1+1+4+4+4+1+2 = 17 bytes  */
#pragma pack(pop)

typedef enum {
    ETH_COMM_OK = 0,
    ETH_COMM_ERR_NOT_INIT = -1,
    ETH_COMM_ERR_BAD_PARAM = -2,
    ETH_COMM_ERR_TX_FAIL = -3,
    ETH_COMM_ERR_RX_FAIL = -4
} EthComm_Status_t;

typedef enum {
    ETH_CMD_NONE = 0,
    ETH_CMD_SERVO_ON,
    ETH_CMD_SERVO_OFF,
    ETH_CMD_SET_TARGET,
    ETH_CMD_STOP,
    ETH_CMD_GET_STATUS
} EthComm_CommandType_t;

typedef struct {
    EthComm_CommandType_t type;
    int32_t i32_value;
    float f32_value;
} EthComm_Command_t;

typedef struct {
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
    uint8_t *tx_buffer;
    uint16_t tx_buffer_size;
} EthComm_Config_t;

typedef enum {
    STEER_MODE_NONE = 0,
    STEER_MODE_AUTO = 1,
    STEER_MODE_MANUAL = 2,
    STEER_MODE_ESTOP = 3
} SteerMode_t;

int EthComm_Init(const EthComm_Config_t *config);
void EthComm_Update(void);
bool EthComm_IsInitialized(void);

/* 테스트/디버깅용 API */
int EthComm_SendString(const char *msg);
int EthComm_HandleLine(const char *line);

/* ========== UDP 자율주행 수신 API ========== */
void EthComm_UDP_Init(void);
bool EthComm_HasNewData(void);
AutoDrive_Packet_t EthComm_GetLatestData(void);
SteerMode_t EthComm_GetCurrentMode(void);
bool EthComm_ConsumeEmergencyRequest(void);

#endif /* INC_ETHERNET_COMMUNICATION_H_ */
