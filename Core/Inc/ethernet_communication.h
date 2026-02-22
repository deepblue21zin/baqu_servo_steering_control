#ifndef INC_ETHERNET_COMMUNICATION_H_
#define INC_ETHERNET_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>

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

int EthComm_Init(const EthComm_Config_t *config);
void EthComm_Update(void);
bool EthComm_IsInitialized(void);

/* 테스트/디버깅용 API */
int EthComm_SendString(const char *msg);
int EthComm_HandleLine(const char *line);

#endif /* INC_ETHERNET_COMMUNICATION_H_ */
