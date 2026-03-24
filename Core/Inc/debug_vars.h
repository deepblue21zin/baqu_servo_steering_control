#ifndef DEBUG_VARS_H
#define DEBUG_VARS_H

#include <stdint.h>

#define DBG_FAULT_POS_LIMIT   (1UL << 0)
#define DBG_FAULT_TRACKING    (1UL << 1)
#define DBG_FAULT_TIMEOUT     (1UL << 2)
#define DBG_FAULT_DISABLED    (1UL << 8)
#define DBG_FAULT_EMERGENCY   (1UL << 9)

extern volatile int32_t  dbg_enc_raw;
extern volatile int32_t  dbg_pos_mdeg;
extern volatile int32_t  dbg_target_mdeg;
extern volatile int32_t  dbg_err_mdeg;
extern volatile int16_t  dbg_pwm_cmd;
extern volatile uint32_t dbg_fault_flags;

#endif /* DEBUG_VARS_H */
