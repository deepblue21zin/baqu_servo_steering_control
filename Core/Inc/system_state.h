


typedef enum {
    SYS_STATE_INIT = 0,
    SYS_STATE_HOMING,
    SYS_STATE_READY,       // 호밍 완료, 명령 대기
    SYS_STATE_RUNNING,     // PID 제어 중
    SYS_STATE_ERROR,
    SYS_STATE_EMERGENCY
} SystemState_t;