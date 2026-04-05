#ifndef INC_ETHERNET_COMMUNICATION_H_
#define INC_ETHERNET_COMMUNICATION_H_

#include "project_params.h"

#include <stdint.h>
#include <stdbool.h>

/* 내부 정규화 데이터:
 * - ASMS 5B(mode + joystick), PC 9B(<iIB>)를 파싱해 공통 포맷으로 저장 */
typedef struct {
    float    steering_angle;  /* steering-axis degree from upper controller */
    uint32_t speed;           /* PC speed raw */
    uint8_t  misc;            /* PC misc raw  */
} AutoDrive_Packet_t;

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

/* Legacy text-command API:
 * 현재 런타임 경로는 UDP API를 사용한다.
 * 아래 API는 벤치/디버깅 용도로만 유지한다. */
int EthComm_Init(const EthComm_Config_t *config);
void EthComm_Update(void);
bool EthComm_IsInitialized(void);
int EthComm_SendString(const char *msg);
int EthComm_HandleLine(const char *line);

/* ========== UDP 자율주행 수신 API ========== */
void EthComm_UDP_Init(void);
bool EthComm_HasNewData(void);
AutoDrive_Packet_t EthComm_GetLatestData(void);
SteerMode_t EthComm_GetCurrentMode(void);
bool EthComm_ConsumeEmergencyRequest(void);
uint32_t EthComm_GetLastRxTick(void);
void EthComm_ForceMode(SteerMode_t mode);

#endif /* INC_ETHERNET_COMMUNICATION_H_ */
